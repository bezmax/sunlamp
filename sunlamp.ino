#include <DS3232RTC.h>
#include <Streaming.h>
#include <TimeLib.h>
#include <limits.h>
#include <EEPROM.h>

#include "sunlamp.h"

const int PIN_RELAY = 12;
const int PIN_DIMMER = 11;
const int PIN_ALARM_INTERRUPT = 2;

const unsigned long MIN_TIME = 946684800;
const unsigned long MAX_TIME = 4102444800-1;

const short HEADER_VERSION = 1;

State s;

//PIN: 8402
void setup() {
  loadState();
  
  //Serial.begin(57600);
  Serial.begin(9600); //HC-06 default
  
  pinMode(PIN_DIMMER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);

  digitalWrite(PIN_RELAY, LOW);
  analogWrite(PIN_DIMMER, LOW);

  // RTC
  pinMode(PIN_ALARM_INTERRUPT, INPUT_PULLUP);

  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);

  setTime(RTC.get());

  Serial.setTimeout(10000); //TODO: Remove
}

void loop() {
  processSchedules();
  processSerial();
  processDimmer();
  delay(100);
}

void dim(int value) {
  if (value <= 0 || value > 255) {
    digitalWrite(PIN_RELAY, LOW); 
    analogWrite(PIN_DIMMER, LOW);
  } else {
    digitalWrite(PIN_RELAY, HIGH);
    analogWrite(PIN_DIMMER, value);
  }
}

void processDimmer() {
  if ((byte)s.dim.curState == s.dim.targetState) {
    return;
  }
  
  unsigned long msecPass = millis() - s.dim.lastUpdate;
  float delta = s.dim.deltaPerSec * msecPass / 1000;

  if (s.dim.targetState > (byte)s.dim.curState) {
    s.dim.curState = min((float)s.dim.targetState, s.dim.curState + delta);
  } else {
    s.dim.curState = max((float)s.dim.targetState, s.dim.curState - delta);
  }

  s.dim.lastUpdate = millis();

  dim((byte)s.dim.curState);
}

void serialSkipLine() {
  while (Serial.available() && Serial.read() != '\n');
}

void serialSkipSpaces() {
  while (Serial.available() && Serial.peek() == ' ') Serial.read();
}

bool serialReadWord(char *target, int maxBytes) {
  serialSkipSpaces();
  if (maxBytes <= 0) {
    return false;
  }
  while (Serial.available()) {
    char c = Serial.peek();
    if (c == '\n' || c == 0 || c == ' ') {
      break;
    } else if (maxBytes > 1) {
      *target = Serial.read();
      target++;
      maxBytes--;
    } else {
      *target = 0;
      return false;
    }
  }
  *target = 0;
  return true;
}

unsigned long serialParseUL() {
  char arg[16];
  if (!serialReadWord(arg, 16)) {
    return ULONG_MAX;
  }
  return strtoul(arg, NULL, 0);
}

void processSerial() {
  char cmd[4];
  
  if (Serial && Serial.available()) {
    if (!serialReadWord(cmd, 4)) {
      Serial << "ERR: Unknown command: " << cmd << endl;
      serialSkipLine();
    } else if (strcmp(cmd,"sdt") == 0) {
      processSerialSetDateTime();
    } else if (strcmp(cmd,"gdt") == 0) {
      processSerialGetDateTime();
    } else if (strcmp(cmd,"gdp") == 0) {
      processSerialGetDateParts();
    } else if (strcmp(cmd,"sdl") == 0) {
      processSerialSetDimLevel();
    } else if (strcmp(cmd,"gdl") == 0) {
      processSerialGetDimLevel();
    } else if (strcmp(cmd,"gsc") == 0) {
      processSerialGetSchedules();
    } else if (strcmp(cmd,"ssc") == 0) {
      processSerialSetSchedules();
    } else if (strcmp(cmd,"gna") == 0) {
      processSerialGetNextAlarm();
    } else {
      Serial << "ERR: Unknown command: " << cmd << endl;
      serialSkipLine();
    }
  }
}

void processSerialSetDateTime() {
  unsigned long timestamp = serialParseUL();
  if (timestamp >= MIN_TIME && timestamp < MAX_TIME) {
    time_t newTime = (time_t)timestamp;
    setTime(newTime);
    RTC.set(newTime);
    updateNextSchedule();
    Serial << "ACK " << timestamp << endl;
  } else {
    Serial << "ERR: Bad arguments for 'sdt'" << endl;
  }
  serialSkipLine();
}

void processSerialGetDateTime() {
  time_t t = RTC.get();
  Serial << "ACK " << (unsigned long)t << endl;
  serialSkipLine();
}

void processSerialGetDateParts() {
  time_t t = RTC.get();
  Serial << "ACK " << weekday(t) << " " << hour(t) << " " << minute(t) << " " << second(t) << endl;
  serialSkipLine();
}

void processSerialSetDimLevel() {
  int lvl = Serial.parseInt();
  if (lvl >= 0 && lvl <= 255) {
    s.dim.targetState = lvl;
    s.dim.deltaPerSec = 75;
    s.dim.lastUpdate = millis();
    Serial << "ACK " << lvl << endl;
  } else {
    Serial << "ERR: Wrong level" << endl;
  }
  serialSkipLine();
}

void processSerialGetDimLevel() {
  Serial << "ACK " << s.dim.curState << " " << s.dim.targetState << " " << s.dim.deltaPerSec << endl;
  serialSkipLine();
}

void processSerialGetSchedules() {
  Serial << "ACK " << s.scheduleCnt;
  for (int i=0; i < s.scheduleCnt; i++) {
    AlarmSchedule as = s.schedules[i];
    Serial << " " << as.dow << " " << as.h << " " << as.m << " " << as.targetState << " " << as.dps;
  }
  Serial << endl;
  serialSkipLine();
}

void processSerialSetSchedules() {
  int cntRaw = Serial.parseInt();
  int cnt = max(0, cntRaw);
  
  AlarmSchedule *newSchedules = new AlarmSchedule[cnt];
  for (int i=0; i< cnt; i++) {
    newSchedules[i].dow = Serial.parseInt();
    newSchedules[i].h = Serial.parseInt();
    newSchedules[i].m = Serial.parseInt();
    newSchedules[i].targetState = Serial.parseInt();
    newSchedules[i].dps = Serial.parseFloat();
  }

  delete[] s.schedules;
  s.schedules = newSchedules;
  s.scheduleCnt = cnt;
  updateNextSchedule();
  persistState();
  Serial << "ACK" << endl;
  serialSkipLine();
}

void processSerialGetNextAlarm() {
  if (s.nextSchedule != 0) {
    Serial << "ACK " << minUntil(RTC.get(), s.nextSchedule, false) << endl;
  } else {
    Serial << "ACK -1" << endl;
  }
  serialSkipLine();
}

void processSchedules() {
  if (s.nextSchedule != 0) {
    if (minUntil(RTC.get(), s.nextSchedule, true) == 0) {
      activateSchedule(s.nextSchedule);
      updateNextSchedule();
    }
  }
}

void activateSchedule(AlarmSchedule *sched) {
  s.dim.targetState = sched->targetState;
  s.dim.deltaPerSec = sched->dps;
  s.dim.lastUpdate = millis();
}

void updateNextSchedule() {
  s.nextSchedule = findNextSchedule();
}

AlarmSchedule* findNextSchedule() {
  if (s.scheduleCnt == 0) {
    return 0;
  }
  time_t t = RTC.get();

  int minMinutes = -1;
  AlarmSchedule* minSched = 0;
  for (int i=0; i< s.scheduleCnt; i++) {
    if (isEnabled(&(s.schedules[i]))) {
      int mins = minUntil(t, &(s.schedules[i]), false);
      if (mins < minMinutes || minSched == 0) {
        minMinutes = mins;
        minSched = &(s.schedules[i]);
      }
    }
  }

  return minSched;
}

boolean isEnabled(AlarmSchedule* s) {
  return s->dow & 0b10000000;
}

int minUntil(time_t t, AlarmSchedule* s, bool showCurrent) {
  int dow = weekday(t);
  int h = hour(t);
  int m = minute(t);

  int min_dmins = -1;
  for (int cur_dow=1;cur_dow<=7;cur_dow++) {
    bool active_at_dow = s->dow & (0b00000001 << (cur_dow-1));

    if (active_at_dow) {
      int d_mins = (cur_dow - dow)*24*60 + (s->h - h)*60 + (s->m - m);
  
      if (d_mins < 0 || d_mins == 0 && !showCurrent) {
        d_mins += 7*24*60;
      }
  
      if (min_dmins == -1 || d_mins < min_dmins) {
        min_dmins = d_mins;
      }
    }
  }

  return min_dmins;
}

void persistState() {
  EepromHeader header;
  strcpy(header.header, "sunlamp");
  header.version = HEADER_VERSION;
  header.scheduleCnt = s.scheduleCnt;
  EEPROM.put(0, header);
  for (int i=0; i<header.scheduleCnt; i++) {
    EEPROM.put(sizeof(header)+i*sizeof(AlarmSchedule), s.schedules[i]);
  }
}

bool loadState() {
  EepromHeader header;
  EEPROM.get(0, header);

  if (strcmp(header.header, "sunlamp") != 0 || header.version != HEADER_VERSION) {
    strcpy(header.header, "sunlamp");
    header.version = HEADER_VERSION;
    header.scheduleCnt = 0;
    EEPROM.put(0, header);

    s.schedules = new AlarmSchedule[0];
    s.scheduleCnt = 0;
  } else {
    s.scheduleCnt = header.scheduleCnt;
    s.schedules = new AlarmSchedule[header.scheduleCnt];
    for (int i=0; i< header.scheduleCnt; i++) {
      EEPROM.get(sizeof(header)+i*sizeof(AlarmSchedule), s.schedules[i]);
    }
  }

  s.dim.curState = 0;
  s.dim.targetState = 0;
  s.dim.deltaPerSec = 0;
  s.dim.lastUpdate = millis();

  s.nextSchedule = 0;
}
