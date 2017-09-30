#include <Arduino_FreeRTOS.h>

#include "PlayingWithFusion_MAX31856.h"
#include "PlayingWithFusion_MAX31856_STRUCT.h"
#include "SPI.h"
#include <AccelStepper.h>

//#define TC0_CS  10
//#define TC1_CS  9
//#define TC2_CS  8
//#define TC3_CS  7

#define TC0_CS  7
//#define TC1_CS  9
//#define TC2_CS  8
//#define TC3_CS  7

#define TC0_FAULT 2
#define TC0_DRDY  2

#define MICROSTEP     (1)
#define STEPS_PER_REV (200)
#define ACME_TPI      (13.0 / 2) /*13 threads per inch, 5/16 ACME rod, 2 starts Apparently non-standard? 14TPI is standard*/
//#define MM_PER_REV    (25.4 / ACME_TPI)
#define INCHES_TO_STEPS(x)   (x * ACME_TPI * STEPS_PER_REV * MICROSTEP)

//#define MM_PER_REV    (4.21)


// define two tasks for Blink & AnalogRead
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


  //Hardware setup

  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device


  

  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

//  xTaskCreate(
//    TaskUpdateGauges
//    ,  (const portCHAR *)"UpdGauges"   // A name just for humans
//    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
//    ,  NULL
//    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//    ,  NULL );
//
//  xTaskCreate(
//    TaskReadThermocouple
//    ,  (const portCHAR *) "ReadThermo"
//    ,  128  // Stack size
//    ,  NULL
//    ,  1  // Priority
//    ,  NULL );

  xTaskCreate(
    TaskRunStepper
    ,  (const portCHAR *)"RunStep"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void TaskRunStepper(void *pvParameters) 
{
  TickType_t xLastWakeTime;
  

  const TickType_t xFrequency = 5; //5 ticks is 75ms
  
  AccelStepper s(AccelStepper::DRIVER);
  s.setMaxSpeed(INCHES_TO_STEPS(1)); 
  s.setAcceleration(INCHES_TO_STEPS(3));

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
      ain_filt = (analog_in/1023.0 * f) + (ain_filt * (1-f));
      
//      s.runToNewPosition(INCHES_TO_STEPS(3));
//      vTaskDelayUntil( &xLastWakeTime, xFrequency );
//      
//      s.runToNewPosition(0);
//      vTaskDelayUntil( &xLastWakeTime, xFrequency );


    s.moveTo(INCHES_TO_STEPS(ain_filt * 2));
    s.run();
    
  }
  
  
}
void TaskBlink(void *pvParameters) 
{
  (void) pvParameters;


  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) 
  {
    digitalWrite(LED_BUILTIN, HIGH);  
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    digitalWrite(LED_BUILTIN, LOW);    
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}


#define GAUGE_FLUE_PIN      5

#define GAUGE_FLUE      1
#define GAUGE_FIREBOX   2
#define GAUGE_DUCT_HOT  3
#define GAUGE_DUCT_RET  4


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


double GaugeFlueFrac = 0;

void TaskUpdateGauges(void *pvParameters) 
{
  long last = millis();
  int lastGaugePWM  = 0;
  double effGaugePWM = 0;

  int eff_ms = 50; //About 3 system ticks
  while(1)
  {

    long now = millis();
    long dt_ms = now - last;
    last = now;
  
    if(dt_ms > 0)
    {
      effGaugePWM += (lastGaugePWM-effGaugePWM) * dt_ms / eff_ms; //Calculate effective (fractional) PMW over last the last eff_ms.
      
      double desiredGaugePWM = 255 * GaugeFlueFrac;
      double proposedGaugePWM = desiredGaugePWM + (desiredGaugePWM - effGaugePWM); //Propose a new PWM, to correct the effective PWM to reach the desired PWM
      
      lastGaugePWM = round(proposedGaugePWM);
      if(lastGaugePWM < 0) lastGaugePWM = 0;
      if(lastGaugePWM > 255) lastGaugePWM = 255;

//      Serial.println(lastGaugePWM);
      analogWrite(GAUGE_FLUE_PIN, lastGaugePWM);
    }
    
    vTaskDelay(1);
  }
}

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
    GaugeFlueFrac = CalculateGaugeFrac(temp_F, 100, 300, 500, 1000, 1/5.0, 4.4/5.0);
    //GaugeFlueFrac = CalculateGaugeFrac(temp_F, 60, 80, 100, 110, 1/5.0, 4/5.0);
    
  }

}
void TaskReadThermocouple(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  static struct var_max31856 TC_CH0, TC_CH1, TC_CH2, TC_CH3;

  PWF_MAX31856  thermocouple0(TC0_CS, TC0_FAULT, TC0_DRDY);
  thermocouple0.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_16SAMP);



  int cnt = 0;
  for (;;)
  {
    double temp = 0;
    
    thermocouple0.MAX31856_update(&TC_CH0);        // Update MAX31856 channel 0
    if(TC_CH0.status)
    {
      temp = -1;

      Serial.print("Fault List: ");
      if(0x01 & TC_CH0.status){Serial.print("OPEN  ");}
      if(0x02 & TC_CH0.status){Serial.print("Overvolt/Undervolt  ");}
      if(0x04 & TC_CH0.status){Serial.print("TC Low  ");}
      if(0x08 & TC_CH0.status){Serial.print("TC High  ");}
      if(0x10 & TC_CH0.status){Serial.print("CJ Low  ");}
      if(0x20 & TC_CH0.status){Serial.print("CJ High  ");}
      if(0x40 & TC_CH0.status){Serial.print("TC Range  ");}
      if(0x80 & TC_CH0.status){Serial.print("CJ Range  ");}
      Serial.println(" ");
    
    }
    else
    {
      temp = TC_CH0.lin_tc_temp * 0.0078125d;
    }




//    Serial.println(temp, 2);
    DisplayTemperatureF(GAUGE_FLUE, ToFahrenheit(temp));

   

//      Serial.println(temp, 4);

    
    vTaskDelay(10);  
  }
}
