#ifndef _REGISTRY_H_
#define _REGISTRY_H_


#define MAX_REGNAME_LEN (8)
#define MAX_REG_NV (5)
#define MAX_REG_RAM (10)
#define MAX_REG (MAX_REG_NV + MAX_REG_RAM)

#define MAX_CMDNAME_LEN (8)

#define EEPROM_NV_BASEADDR (0)

#define IDX_TO_EEPROM_ADDR(x)  (EEPROM_NV_BASEADDR + ((x) * (MAX_REGNAME_LEN + 5)))


enum
{
  REG_YES = 1,
  REG_NO = 0
};


enum
{
  REG_INT = 1,
  REG_FLOAT = 2
};

enum
{
  REG_OK = 0,
  REG_ERR_MAXLEN = -1,
  REG_NAME_TAKEN = -2,
  REG_BAD_HANDLE = -3
};


enum
{
  REG_CREATE_NONE = 0,
  REG_CREATE_NV = 1,
  REG_CREATE_RONLY = 2
};

struct RegEntry
{
  char regname[MAX_REGNAME_LEN+1];
  byte type;
  long val;
  byte isNV;
  int baseAddr;
  
  
};



#endif
