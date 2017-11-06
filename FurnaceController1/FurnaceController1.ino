#include <Arduino_FreeRTOS.h>
#include "semphr.h"
#include "Registry.h"

#if configUSE_TRACE_FACILITY
#else
  #error configUSE_TRACE_FACILITY required for configGENERATE_RUN_TIME_STATS
#endif


#include "PlayingWithFusion_MAX31856.h"
#include "PlayingWithFusion_MAX31856_STRUCT.h"
#include "SPI.h"
#include <AccelStepper.h>
#include <EEPROM.h>

#define TC0_CS  7
#define TC1_CS  8
#define TC2_CS  9
#define TC3_CS  10


#define TC0_FAULT 2
#define TC0_DRDY  2

#define MICROSTEP     (1)
#define STEPS_PER_REV (200)
#define ACME_TPI      (13.0 / 2) /*13 threads per inch, 5/16 ACME rod, 2 starts Apparently non-standard? 14TPI is standard*/
//#define MM_PER_REV    (25.4 / ACME_TPI)
#define INCHES_TO_STEPS(x)   (x * ACME_TPI * STEPS_PER_REV * MICROSTEP)

//#define MM_PER_REV    (4.21)



#define GAUGE_FLUE_PIN      5

#define GAUGE_FLUE      1
#define GAUGE_FIREBOX   2
#define GAUGE_DUCT_HOT  3
#define GAUGE_DUCT_RET  4

#define MAX_TC_NAME_LEN (8)
int hTS[4];
int hTT[4];
float TT[4];
char TN[4][MAX_TC_NAME_LEN];

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
       frac = (val - int(val)) * precision;
   else
       frac = (int(val)- val ) * precision;
   Serial.print(frac,DEC) ;
} 

int ThermocoupleStatusDebug(const char* p1, const char* p2)
{
  int i;
  for(i=0; i<4; i++)
  {
    long st = ReadIntegerRegByHandle(hTS[i]);
    Serial.print("Thermocouple ");
    Serial.print(i, DEC);
    Serial.print(" \"");
    Serial.print(TN[i]);
    Serial.print("\" ");
      
    if(st)
    {
      Serial.print(" Fault: ");
      if(0x01 & st){Serial.print("OPEN  ");}
      if(0x02 & st){Serial.print("Overvolt/Undervolt  ");}
      if(0x04 & st){Serial.print("TC Low  ");}
      if(0x08 & st){Serial.print("TC High  ");}
      if(0x10 & st){Serial.print("CJ Low  ");}
      if(0x20 & st){Serial.print("CJ High  ");}
      if(0x40 & st){Serial.print("TC Range  ");}
      if(0x80 & st){Serial.print("CJ Range  ");}
    }
    else
    {
      Serial.print(" OK: ");
      printDouble(TT[i], 100);
      Serial.print("C, ");
      printDouble(ToFahrenheit(TT[i]), 100);
      Serial.print("F");
    }
     Serial.println(" ");
  }
  return REG_OK;
}

void TaskBlink( void *pvParameters );
void TaskReadThermocouple( void *pvParameters );
void TaskRunStepper( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  Serial.println("\r\n\r\nBooted.");

  //Hardware setup

  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device


  RegistryInit();


  strncpy(TN[0], "Flue  ", MAX_TC_NAME_LEN);
  strncpy(TN[1], "Return", MAX_TC_NAME_LEN);
  strncpy(TN[2], "Duct  ", MAX_TC_NAME_LEN);
  strncpy(TN[3], "Spare ", MAX_TC_NAME_LEN);
  
  AddCommand("THSTAT", ThermocoupleStatusDebug);
  
//
  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  configMINIMAL_STACK_SIZE+50  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

//  xTaskCreate(
//    TaskUpdateGauges
//    ,  (const portCHAR *)"UpdGauges"   // A name just for humans
//    ,  configMINIMAL_STACK_SIZE  // This stack size can be checked & adjusted by reading the Stack Highwater
//    ,  NULL
//    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//    ,  NULL );
//
  xTaskCreate(
    TaskReadThermocouple
    ,  (const portCHAR *) "Thermoc."
    ,  configMINIMAL_STACK_SIZE+100  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
//
//  check_mem();
  xTaskCreate(
    TaskRunStepper
    ,  (const portCHAR *)"Stepper"   // A name just for humans
    ,  configMINIMAL_STACK_SIZE+25  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  //check_mem();

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

AccelStepper s(AccelStepper::DRIVER);
void TaskRunStepper(void *pvParameters) 
{

  TickType_t xLastWakeTime;

  const TickType_t xFrequency = 5; //5 ticks is 75ms

  s.setMaxSpeed(INCHES_TO_STEPS(1)); 
  s.setAcceleration(INCHES_TO_STEPS(3));

  int hnd = AddIntegerRegistryEntry("MAXD", 20, REG_CREATE_NV);
  Serial.println("MAXD");
  Serial.println(hnd, DEC);
//
//  while(1)
//  {
//    s.runToNewPosition(0);
//    s.runToNewPosition(1);
//  }

  //s.moveTo(

  float oscSpeed = 0; //Steps/sec
  float accel = 0; //Step/sec^2
  
  float accel_mag = 1;
  
  int max_speed = 4000;
  
  xLastWakeTime = xTaskGetTickCount();

  float ain_filt = 0;
  float f = 0.001;
  while(1)
  {
      int analog_in = analogRead(A0);
//      Serial.println(analog_in);
//      vTaskDelay(20);
      float frac = (analog_in - 300) / (float)(600 - 300);
      if(frac < 0)
      { frac = 0; }
      if(frac > 1)
      { frac = 1; }
      ain_filt = (frac * f) + (ain_filt * (1-f));
      
//      s.runToNewPosition(INCHES_TO_STEPS(3));
//      vTaskDelayUntil( &xLastWakeTime, xFrequency );
//      
//      s.runToNewPosition(0);
//      vTaskDelayUntil( &xLastWakeTime, xFrequency );


  
    //analogWrite(GAUGE_FLUE_PIN, ain_filt * 255);

    
 
    int maxd_mm = (int)ReadIntegerRegByHandle(hnd);

    s.moveTo(INCHES_TO_STEPS(ain_filt * maxd_mm / 25.4));
    s.run();
    
  }
  
  
}

#define LED_BLINK (LED_BUILTIN)
void TaskBlink(void *pvParameters) 
{
  (void) pvParameters;

  Serial.println("BLINK");
  //check_mem();
  
  int hnd = AddIntegerRegistryEntry("ON", 200, REG_CREATE_NV);

  pinMode(LED_BLINK, OUTPUT);

  for (;;) 
  {
    int ms = ReadIntegerRegByHandle(hnd);
    
    digitalWrite(LED_BLINK, HIGH);  
    vTaskDelay( ms / portTICK_PERIOD_MS );
    digitalWrite(LED_BLINK, LOW);    
    vTaskDelay( (1000-ms) / portTICK_PERIOD_MS );
  }
}



double CalculateGaugeFrac(double temp, double tmin, double t1, double t2, double tmax, double r1, double r2)
{
  double frac;
  if(temp < tmin)
  {
    frac = 0;
  }
  else if(temp < t1)
  {
    frac = 0 + (r1 - 0) * (temp - tmin) / (t1 - tmin);
  }
  else if(temp < t2)
  {
    frac = r1 + (r2 - r1) * (temp - t1) / (t2 - t1);
  }
  else if(temp < tmax)
  {
    frac = r2 + (1 - r2) * (temp - t2) / (tmax - t2);
  }
  else
  {
    frac = 1;
  }
  return frac;
}


double GaugeFlueFrac = 0.5;

//void TaskUpdateGauges(void *pvParameters) 
//{
//  Serial.println("GAUGES");
//  long last = millis();
//  int lastGaugePWM  = 0;
//  double effGaugePWM = 0;
//
//  int eff_ms = 50; //About 3 system ticks
//  while(1)
//  {
//
//    long now = millis();
//    long dt_ms = now - last;
//    last = now;
//  
//    if(dt_ms > 0)
//    {
//      effGaugePWM += (lastGaugePWM-effGaugePWM) * dt_ms / eff_ms; //Calculate effective (fractional) PMW over last the last eff_ms.
//      
//      double desiredGaugePWM = 255 * GaugeFlueFrac;
//      double proposedGaugePWM = desiredGaugePWM + (desiredGaugePWM - effGaugePWM); //Propose a new PWM, to correct the effective PWM to reach the desired PWM
//      
//      lastGaugePWM = round(proposedGaugePWM);
//      if(lastGaugePWM < 0) lastGaugePWM = 0;
//      if(lastGaugePWM > 255) lastGaugePWM = 255;
//
////      Serial.println(lastGaugePWM);
//      analogWrite(GAUGE_FLUE_PIN, lastGaugePWM);
//    }
//    
//    taskYIELD();
//  }
//}

double ToFahrenheit(double Celsius)
{
  return Celsius * 9 / 5 + 32;
}
void DisplayTemperatureF(int gaugeID, double temp_F)
{
  int pwm = 0;
  int pin = -1;
  if(gaugeID == GAUGE_FLUE)
  {
    GaugeFlueFrac = CalculateGaugeFrac(temp_F, 100, 300, 500, 1000, 50.0/255, 219.0/255);
    //GaugeFlueFrac = CalculateGaugeFrac(temp_F, 60, 80, 100, 110, 1/5.0, 4/5.0);
    analogWrite(GAUGE_FLUE_PIN, 255 * GaugeFlueFrac);
  }

}


void TaskReadThermocouple(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  Serial.println("THERMO.");

  static struct var_max31856 TC_CH0, TC_CH1, TC_CH2, TC_CH3;

  PWF_MAX31856  thermocouple0(TC0_CS, TC0_FAULT, TC0_DRDY);
  thermocouple0.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_16SAMP);

  PWF_MAX31856  thermocouple1(TC1_CS, TC0_FAULT, TC0_DRDY);
  thermocouple1.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_16SAMP);

  PWF_MAX31856  thermocouple2(TC2_CS, TC0_FAULT, TC0_DRDY);
  thermocouple2.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_16SAMP);

  PWF_MAX31856  thermocouple3(TC3_CS, TC0_FAULT, TC0_DRDY);
  thermocouple3.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_16SAMP);

  Serial.println("Thermocouples initialized.");

  int h1 = AddIntegerRegistryEntry("T", 128, 0);
  hTT[0] = AddIntegerRegistryEntry("T0", -1, REG_CREATE_RONLY);
  hTT[1] = AddIntegerRegistryEntry("T1", -1, REG_CREATE_RONLY);
  hTT[2] = AddIntegerRegistryEntry("T2", -1, REG_CREATE_RONLY);
  hTT[3] = AddIntegerRegistryEntry("T3", -1, REG_CREATE_RONLY);
  hTS[0] = AddIntegerRegistryEntry("TS0", -1, REG_CREATE_RONLY);
  hTS[1] = AddIntegerRegistryEntry("TS1", -1, REG_CREATE_RONLY);
  hTS[2] = AddIntegerRegistryEntry("TS2", -1, REG_CREATE_RONLY);
  hTS[3] = AddIntegerRegistryEntry("TS3", -1, REG_CREATE_RONLY);


  int cnt = 0;
  for (;;)
  {
    double temp = 0;
    
    thermocouple3.MAX31856_update(&TC_CH3);
    WriteIntegerRegByHandle(hTS[3], TC_CH3.status);
    if(TC_CH3.status)
    {
      TT[3] = -1;
      WriteIntegerRegByHandle(hTT[3], -1);    
    }
    else
    {
      temp = TC_CH3.lin_tc_temp * 0.0078125d;
      TT[3] = temp;
      WriteIntegerRegByHandle(hTT[3], (int)(temp * 10));
    }

    thermocouple2.MAX31856_update(&TC_CH2);
    WriteIntegerRegByHandle(hTS[2], TC_CH2.status);
    if(TC_CH2.status)
    {
      TT[2] = -1;
      WriteIntegerRegByHandle(hTT[2], -1);
    }
    else
    {
      temp = TC_CH2.lin_tc_temp * 0.0078125d;
      TT[2] = temp;
      WriteIntegerRegByHandle(hTT[2], (int)(temp * 10));
    }

    thermocouple1.MAX31856_update(&TC_CH1); 
    WriteIntegerRegByHandle(hTS[1], TC_CH1.status);
    if(TC_CH1.status)
    {
      TT[1] = -1;
      WriteIntegerRegByHandle(hTT[1], -1);
    }
    else
    {
      temp = TC_CH1.lin_tc_temp * 0.0078125d;
      TT[1] = temp;
      WriteIntegerRegByHandle(hTT[1], (int)(temp * 10));
    }

    thermocouple0.MAX31856_update(&TC_CH0); 
    WriteIntegerRegByHandle(hTS[0], TC_CH0.status);
    if(TC_CH0.status)
    {
      TT[0] = -1;
      WriteIntegerRegByHandle(hTT[0], -1);    
    }
    else
    {
      temp = TC_CH0.lin_tc_temp * 0.0078125d;
      TT[0] = temp;
      WriteIntegerRegByHandle(hTT[0], (int)(temp * 10));
    }

    
    vTaskDelay(50);  
  }
}
