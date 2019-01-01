#ifndef STATE_H
#define STATE_H

typedef struct {
  byte dow; //First 7 bits - days of week, last bit - enabled (1) or disabled(0)
  byte h;
  byte m;
  byte targetState;
  float dps;
} AlarmSchedule;

typedef struct {
  float curState;
  byte targetState;
  float deltaPerSec;
  unsigned long lastUpdate;
} DimState;

typedef struct {
  byte scheduleCnt;
  AlarmSchedule* schedules;

  AlarmSchedule* nextSchedule = 0;
  
  DimState dim; 
} State;

typedef struct {
  char header[10];
  short version;
  byte scheduleCnt;
} EepromHeader;

#endif
