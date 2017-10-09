#include "Arduino.h"
#include <EEPROM.h>
#include "semphr.h"
#include "Registry.h"

void TaskReg( void *pvParameters );


struct RegEntry RegRam[MAX_REG];
int nextRegRam_idx = 0;
int nextRegEEPROM_idx = 0;

char out_line[50];
  

SemaphoreHandle_t cmdSem;
struct COMMAND
{
  int (* cmdFcn)(const char* p1, const char* p2);
  char cmdName[MAX_CMDNAME_LEN+1];
  struct COMMAND * next;
};

COMMAND * cmdlist = {0};

static void DebugCmdList(void)
{
  struct COMMAND * ptr = cmdlist;
  while(ptr != NULL)
  {
    Serial.print((int)ptr, HEX);
    Serial.print(" -> ");

    ptr = ptr->next;
  }
  Serial.println("NULL");
}

static int AppendToList(struct COMMAND * cmd, struct COMMAND ** list)
{
  xSemaphoreTake(cmdSem, portMAX_DELAY);

  if(*list == NULL)
  {
    *list = cmd; //First
  }
  else
  {
    struct COMMAND * ptr = *list;
    while(ptr->next != NULL)
    {
      ptr = ptr->next;
    }   
    ptr->next = cmd; //Append
  }

  
  cmd->next = NULL;  

  //DebugCmdList();
  
  xSemaphoreGive(cmdSem);
  return 0;
}

static COMMAND* FindInList(const char * cmdName)
{
  xSemaphoreTake(cmdSem, portMAX_DELAY);
  struct COMMAND * list = cmdlist;

  do
  {
    if(strncmp(list->cmdName, cmdName, MAX_CMDNAME_LEN) == 0)
    {
      break;
    }

    list = list->next;
  }while(list != NULL);
  
  xSemaphoreGive(cmdSem);
  return list;
}





int AddCommand(const char* cmdName, int (* cmdFcn)(const char* p1, const char* p2))
{
  if(cmdFcn == NULL)
  {
    return -1;
  }
  
  struct COMMAND * cmd = (struct COMMAND *)pvPortMalloc(sizeof(struct COMMAND));
  if(cmd == NULL)
  {
    return -2;
  }

  cmd->cmdFcn = cmdFcn;
  strncpy(cmd->cmdName, cmdName, MAX_CMDNAME_LEN);
  
  AppendToList(cmd, &cmdlist);
  return REG_OK;
}


void ListReg(struct RegEntry * entry)
{
  if(entry->type == REG_INT)
  {
    sprintf(out_line, "%s\t%ld\t%d\t%x\t", entry->regname, entry->val, entry->isNV, entry->baseAddr);
    Serial.print(out_line);
    debugRegNV(entry);
    Serial.println("");
  }
  else
  {
    sprintf(out_line, "%x (Unknown type %d)", entry, (unsigned int)entry->type);
    Serial.println(out_line);
  }
}

int ListallReg(const char* p1, const char* p2)
{
  
  sprintf(out_line, "Name\tValue\tFlags\tbaseAddr\r\n");
  Serial.println(out_line);

  int i;
  for(i = 0; i<nextRegRam_idx; i++)
  {
    ListReg(&RegRam[i]);
  }
  return 0;
}

int FormatNV()
{
  int addr;
  for(addr = IDX_TO_EEPROM_ADDR(0); addr < IDX_TO_EEPROM_ADDR(MAX_REG_NV+1); addr++)
  {
    EEPROM.write(addr, 0);
  }
}

int ReadRegNV(int idx, struct RegEntry* entry)
{
  int baseAddr = IDX_TO_EEPROM_ADDR(idx);
  int addr = baseAddr;
  
  char regname[MAX_REGNAME_LEN+1];
  for(int i=0; i<MAX_REGNAME_LEN; i++)
  {        
    regname[i] = EEPROM.read(addr++);

    if(i == 0 && regname[i] == 0)
    {
      return REG_NO;
    }
  }

  memcpy(entry->regname, regname, MAX_REGNAME_LEN);
  entry->regname[MAX_REGNAME_LEN] = '\0';
  Serial.print(entry->regname);
  Serial.print(F(":"));

  entry->type = ((long)EEPROM.read(addr++));

  entry->val = 0;
  entry->val |= ((long)EEPROM.read(addr++)) << 24; 
  entry->val |= ((long)EEPROM.read(addr++)) << 16;
  entry->val |= ((long)EEPROM.read(addr++)) << 8;
  entry->val |= ((long)EEPROM.read(addr++)) << 0;

  Serial.println(entry->val, DEC);
  entry->baseAddr = baseAddr;
  entry->isNV = 1;
  
  nextRegRam_idx++;
  nextRegEEPROM_idx++;
  return REG_OK;
  
}

int WriteRegNV(struct RegEntry* entry)
{
  int baseAddr = entry->baseAddr;
  int addr = baseAddr + MAX_REGNAME_LEN + 1;
  
  char regname[MAX_REGNAME_LEN+1];

  Serial.print(F("Writing to EEPROM addr "));
  Serial.println(addr, HEX);

  EEPROM.write(addr++, entry->val >> 24);
  EEPROM.write(addr++, entry->val >> 16);
  EEPROM.write(addr++, entry->val >> 8);
  EEPROM.write(addr++, entry->val >> 0);

  return REG_OK;
  
}

void debugRegNV(struct RegEntry* entry)
{
  int addr = entry->baseAddr;

  for(int i=0; i<MAX_REGNAME_LEN; i++)
  {
    Serial.print((char)EEPROM.read(addr++));
  }
  Serial.print(": ");
  for(int i=0; i<5; i++)
  {
    Serial.print(EEPROM.read(addr++), HEX);
    Serial.print(" ");
  }
}


int CreateRegNV(struct RegEntry* entry)
{
  int baseAddr = IDX_TO_EEPROM_ADDR(nextRegEEPROM_idx);
  entry->baseAddr = baseAddr;
  nextRegEEPROM_idx++;

  int addr = baseAddr;
  char * regname = entry->regname;
  
  Serial.print(F("Writing to EEPROM addr "));
  Serial.println(addr, HEX);

  int nameEnded = 0;
  for(int i=0; i<MAX_REGNAME_LEN; i++)
  {
    if(nameEnded)
    {
      EEPROM.write(addr++, 0);
    }
    else
    {
      EEPROM.write(addr++, *regname);
      regname++;
    }
    if(*regname == 0)
    {
      nameEnded = 1;
    }
  }
  

  EEPROM.write(addr++, entry->type);
  EEPROM.write(addr++, entry->val >> 24);
  EEPROM.write(addr++, entry->val >> 16);
  EEPROM.write(addr++, entry->val >> 8);
  EEPROM.write(addr++, entry->val >> 0);

  return REG_OK;
  
}

void ReadAllNV(struct RegEntry* entries)
{
  Serial.println(F("Reading EEPROM: "));
  int i;
  for(i=0; i<MAX_REG_NV; i++)
  {
    if(REG_OK != ReadRegNV(i, &entries[i]))
    {
      break;
    }
  }
}
int cmdFormatNV(const char* p1, const char* p2)
{
  FormatNV();
}
int cmdWriteReg(const char* p1, const char* p2)
{
  int * hnd = GetHandle(p1);
  if(hnd == NULL)
  {
    Serial.println(F("No such entry."));
    return -1;
  }

  //TODO check that entry is an integer type
  struct RegEntry * entry = (struct RegEntry *)hnd;
  
  long val = atol(p2);
  WriteIntegerRegByHandle(hnd, val);

  
  return REG_OK;
  
}


int cmdHelp(const char* p1, const char* p2)
{
  DebugCmdList();

  struct COMMAND * ptr = cmdlist;
  while(ptr != NULL)
  {
    Serial.println(ptr->cmdName);
    ptr = ptr->next;
  }
  return REG_OK;
}

int cmdCPU(const char* p1, const char* p2)
{
  char *pcWriteBuffer = out_line;

  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint32_t ulTotalTime = 0;
  uint32_t ulStatsAsPercentage;

  /* Take a snapshot of the number of tasks in case it changes while this
  function is executing. */
  UBaseType_t uxCurrentNumberOfTasks = uxTaskGetNumberOfTasks();
  
  uxArraySize = uxCurrentNumberOfTasks;
  
  /* Allocate an array index for each task.  NOTE!  If
  configSUPPORT_DYNAMIC_ALLOCATION is set to 0 then pvPortMalloc() will
  equate to NULL. */
  pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxCurrentNumberOfTasks * sizeof( TaskStatus_t ) );
  
  if( pxTaskStatusArray != NULL )
  {
    /* Generate the (binary) data. */
    uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );
  
    /* For percentage calculations. */
    ulTotalTime /= 100UL;

    /* Avoid divide by zero errors. */
    if( ulTotalTime > 0 )
    {
      sprintf( pcWriteBuffer, "%s%s\t%s\r\n",  "Task    ", "CPU%", "RAM HWM");
      Serial.print(pcWriteBuffer);
      /* Create a human readable table from the binary data. */
      for( x = 0; x < uxArraySize; x++ )
      {      
        /* What percentage of the total run time has the task used?
        This will always be rounded down to the nearest integer.
        ulTotalRunTimeDiv100 has already been divided by 100. */
        ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalTime;

        /* Write the task name to the string, padding with
        spaces so it can be printed in tabular form more
        easily. */
        
        Serial.print(pxTaskStatusArray[ x ].pcTaskName);
        size_t y;
        for( y = strlen( pxTaskStatusArray[ x ].pcTaskName ); y < ( size_t ) ( configMAX_TASK_NAME_LEN - 1 ); y++ )
        {
           Serial.print(" ");
        }

        if( ulStatsAsPercentage > 0UL )
        {
          #ifdef portLU_PRINTF_SPECIFIER_REQUIRED
          {
            sprintf( pcWriteBuffer, "\t%lu%%\t%d\r\n", ulStatsAsPercentage, pxTaskStatusArray[ x ].usStackHighWaterMark );
            Serial.print(pcWriteBuffer);
          }
          #else
          {
            /* sizeof( int ) == sizeof( long ) so a smaller
            printf() library can be used. */
            sprintf( pcWriteBuffer, "\t%u%%\t%d\r\n", ( unsigned int ) ulStatsAsPercentage, pxTaskStatusArray[ x ].usStackHighWaterMark );
            Serial.print(pcWriteBuffer);
          }
          #endif
        }
        else
        {
          /* If the percentage is zero here then the task has
          consumed less than 1% of the total run time. */
          #ifdef portLU_PRINTF_SPECIFIER_REQUIRED
          {
            sprintf( pcWriteBuffer, "\t<1%%\t%d\r\n",  pxTaskStatusArray[ x ].usStackHighWaterMark);
            Serial.print(pcWriteBuffer);
          }
          #else
          {
            /* sizeof( int ) == sizeof( long ) so a smaller
            printf() library can be used. */
            sprintf( pcWriteBuffer, "\t<1%%\t%d\r\n",  pxTaskStatusArray[ x ].usStackHighWaterMark);
            Serial.print(pcWriteBuffer);
          }
          #endif
        }
      }
    }
  }

  /* Free the array again.  NOTE!  If configSUPPORT_DYNAMIC_ALLOCATION
  is 0 then vPortFree() will be #defined to nothing. */
  vPortFree( pxTaskStatusArray );
  
  Serial.println("");
  check_mem();

  return 0;
}

/*For heap_3
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
*/

void check_mem()
{
  sprintf(out_line, "Free RAM: %d", xPortGetFreeHeapSize()); 
  Serial.println(out_line);
}

void RegistryInit()
{ 
  check_mem();
  xTaskCreate(
    TaskReg
    ,  (const portCHAR *)"CLI"   // A name just for humans
    ,  configMINIMAL_STACK_SIZE+50  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  4  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

    memset(&RegRam, 0, sizeof(RegRam));
    ReadAllNV(RegRam);

    cmdSem = xSemaphoreCreateBinary();
    xSemaphoreGive(cmdSem);

    ListallReg(NULL, NULL);

    AddCommand("LISTALL", ListallReg);
    AddCommand("FORMAT", cmdFormatNV);
    AddCommand("CPU", cmdCPU);
    AddCommand("HELP", cmdHelp);
    AddCommand("WRITE", cmdWriteReg);
    
    

    
    Serial.println(F("Reg inited."));
    check_mem();

}


int* GetHandle(const char* regname)
{
  int i;
  for(i=0; i<nextRegRam_idx; i++)
  {
    if(strncmp(RegRam[i].regname, regname, MAX_REGNAME_LEN) == 0)
    {
      return (int*)&RegRam[i];
    }
  }
  return NULL;
}

int HandleIsValid(int hnd)
{
  int i;
  for(i=0; i<MAX_REG; i++)
  {
    if(hnd == (int)&RegRam[i])
    {
      return REG_YES;
    }
  }
  return REG_NO;
}
long ReadIntegerRegByHandle(int hnd)
{
  if(!HandleIsValid(hnd))
  {
    Serial.println(hnd, DEC);
    Serial.println(F("Bad handle"));
    return REG_BAD_HANDLE;
  }
  struct RegEntry * entry = (struct RegEntry *)hnd;
  
  
//  Serial.print(entry->regname);
//  Serial.print(" = ");
//  Serial.println(entry->val, DEC);
  return entry->val;
}

int WriteIntegerRegByHandle(int hnd, long val)
{
  if(!HandleIsValid(hnd))
  {
    Serial.println(hnd, DEC);
    Serial.println(F("Bad handle"));
    return REG_BAD_HANDLE;
  }
  struct RegEntry * entry = (struct RegEntry *)hnd;
  
  entry->val = val; //Write RAM cache
  
  if(entry->isNV)
  {
    WriteRegNV(entry); //Write Nonvolatile memory too
  }
}
int AddIntegerRegistryEntry(const char* regname, long initial, byte flags)
{ 
  if(strnlen(regname, MAX_REGNAME_LEN) > MAX_REGNAME_LEN)
  {
    return REG_ERR_MAXLEN;
  }

  int* hnd = GetHandle(regname);
  if(hnd == NULL)
  {
    struct RegEntry * entry = &RegRam[nextRegRam_idx++];
    strncpy(entry->regname, regname, MAX_REGNAME_LEN);
    entry->type = REG_INT;
    entry->val = initial;

    if(flags & REG_CREATE_NV)
    {
      CreateRegNV(entry);
    }
    
    Serial.println((int)entry, DEC);
    return (int)entry;  
  }
  else
  {
    return (int)hnd;
  }
}

#define MAX_CMD_LEN (100)
char line[MAX_CMD_LEN+1];

int GetCommand(void)
{
  static int lineptr = 0;
  while(Serial.available())
  {
    char rec = Serial.read();
    
    if(rec == '\b')     /*Backspace*/
    {
      lineptr--;
      if(lineptr < 0)
      {
        lineptr = 0;
      }
      Serial.print(rec);
    }
    else if(rec == 0x1B) /*Escape*/
    {
      Serial.println("");
      lineptr = 0;
      line[lineptr] = '\0'; //Make empty string
      return 1; //Return no command
    }
    else if(rec == '\r' || rec == '\n')
    {
      Serial.println("");
      line[lineptr] = '\0'; //Null terminate
      lineptr = 0;
      return 1; //Received a command
    }
    else
    {
      if(lineptr < MAX_CMD_LEN)
      {
        if(rec >= 'a' && rec <= 'z')
        {
          rec += 'A' - 'a'; //Convert to uppercase
        }
        line[lineptr++] = rec;
        Serial.print(rec);
      }
      else
      {
        Serial.print(F("\a")); //Bell - line full
      }
    }
  }
  
  return 0;
}


void TaskReg(void *pvParameters) 
{
  check_mem();
  Serial.print(">");
  while(1)
  {
    if(GetCommand())
    {
      const char * cmd = strtok(line, " ");
      struct COMMAND * command = FindInList(cmd);

      if(command != NULL)
      {
        const char * p1 = strtok(NULL, " ");
        const char * p2 = strtok(NULL, " ");
        command->cmdFcn(p1, p2);
      }
      else
      {
        Serial.print(F("Command not found:"));
        Serial.println(line);
      }
      Serial.print(F(">"));
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }

}

