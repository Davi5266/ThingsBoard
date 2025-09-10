#include <Arduino.h>
#include <FreematicsPlus.h>
#include <httpd.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

// States
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_GPS_ONLINE 0x20
#define STATE_CELL_CONNECTED 0x40
#define STATE_WIFI_CONNECTED 0x80
#define STATE_WORKING 0x100
#define STATE_STANDBY 0x200

CBufferManager bufman;
Task subtask;

// live data
String netop;
String ip;
int16_t rssi = 0;
int16_t rssiLast = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

void serverProcess(int timeout);

bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);
void processBLE(int timeout);

char isoTime[32] = {0};

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

State state;


FreematicsESP32 sys;
COBD obd;

void beep(int duration){
    sys.buzzer(2000);
    delay(duration);
    sys.buzzer(0);
}

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};


/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
#if ENABLE_OBD
void processOBD(CBuffer* buffer)
{
  static int idx[2] = {0, 0};
  int tier = 1;
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        // reset previous tier index
        idx[tier - 2] = 0;
        // keep new tier number
        tier = obdData[i].tier;
        // move up current tier index
        i += idx[tier - 2]++;
        // check if into next tier
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;
        buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
    } else {
        timeoutsOBD++;
        // printTimeoutStats();
        break;
    }
    if (tier > 1) break;
  }
  int kph = obdData[0].value;
  if (kph >= 2) lastMotionTime = millis();
}
#endif

bool initGPS()
{
  // start GNSS receiver
  if (sys.gpsBeginExt()) {
    Serial.println("GNSS:OK(E)");
  } else if (sys.gpsBegin()) {
    Serial.println("GNSS:OK(I)");
  } else {
    Serial.println("GNSS:NO");
    return false;
  }
  return true;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif
  if (!gd || lastGPStime == gd->time) return false;
  if (gd->date) {
    // generate ISO time string
    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  }
  if (gd->lng == 0 && gd->lat == 0) {
    // coordinates not ready
    if (gd->date) {
      Serial.print("[GNSS] ");
      Serial.println(isoTime);
    }
    return false;
  }
  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {
    // invalid coordinates data
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float)); /* m */
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }
  
  Serial.print("[GNSS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  Serial.print(" SATS:");
  Serial.print(gd->sat);
  Serial.print(" HDOP:");
  Serial.print(gd->hdop);
  Serial.print(" Course:");
  Serial.println(gd->heading);
  //Serial.println(gd->errors);
  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

void initialize()
{
  // dump buffer data
  bufman.purge();

// #if ENABLE_MEMS
//   if (state.check(STATE_MEMS_READY)) {
//     calibrateMEMS();
//   }
// #endif

#if GNSS == GNSS_STANDALONE
  if (!state.check(STATE_GPS_READY)) {
    if (initGPS()) {
      state.set(STATE_GPS_READY);
    }
  }
#endif

#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
#if ENABLE_OLED
      oled.println("OBD OK");
#endif
    } else {
      Serial.println("OBD:NO");
      //state.clear(STATE_WORKING);
      //return;
    }
  }
#endif

// #if STORAGE != STORAGE_NONE
//   if (!state.check(STATE_STORAGE_READY)) {
//     // init storage
//     if (logger.init()) {
//       state.set(STATE_STORAGE_READY);
//     }
//   }
//   if (state.check(STATE_STORAGE_READY)) {
//     fileid = logger.begin();
//   }
// #endif

  // re-try OBD if connection not established
#if ENABLE_OBD
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      memcpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
#if ENABLE_OLED
    oled.print("VIN:");
    oled.println(vin);
#endif
  }
#endif

  // check system time
//   printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);
  oled.clear();
  oled.print("DEVICE ID: ");
  oled.println(devid);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif
}

void process()
{
  static uint32_t lastGPStick = 0;
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.getFree();
  buffer->state = BUFFER_STATE_FILLING;

#if ENABLE_OBD
  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
    processOBD(buffer);
    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        Serial.println("[OBD] ECU OFF");
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
    }
  } else if (obd.init(PROTO_AUTO, true)) {
    state.set(STATE_OBD_READY);
    Serial.println("[OBD] ECU ON");
  }
#endif

  if (rssi != rssiLast) {
    int val = (rssiLast = rssi);
    buffer->add(PID_CSQ, ELEMENT_INT32, &val, sizeof(val));
  }
#if ENABLE_OBD
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 45) / 4095;
  } else {
    batteryVoltage = obd.getVoltage();
  }
  if (batteryVoltage) {
    uint16_t v = batteryVoltage * 100;
    buffer->add(PID_BATTERY_VOLTAGE, ELEMENT_UINT16, &v, sizeof(v));
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if ENABLE_MEMS
  processMEMS(buffer);
#endif

  bool success = processGPS(buffer);
#if GNSS_RESET_TIMEOUT
  if (success) {
    lastGPStick = millis();
    state.set(STATE_GPS_ONLINE);
  } else {
    if (millis() - lastGPStick > GNSS_RESET_TIMEOUT * 1000) {
      sys.gpsEnd();
      state.clear(STATE_GPS_ONLINE | STATE_GPS_READY);
      delay(20);
      if (initGPS()) state.set(STATE_GPS_READY);
      lastGPStick = millis();
    }
  }
#endif

//   if (!state.check(STATE_MEMS_READY)) {
//     deviceTemp = readChipTemperature();
//   }
//   buffer->add(PID_DEVICE_TEMP, ELEMENT_INT32, &deviceTemp, sizeof(deviceTemp));

//   buffer->timestamp = millis();
//   buffer->state = BUFFER_STATE_FILLED;

//   // display file buffer stats
//   if (startTime - lastStatsTime >= 3000) {
//     bufman.printStats();
//     lastStatsTime = startTime;
//   }

// #if STORAGE != STORAGE_NONE
//   if (state.check(STATE_STORAGE_READY)) {
//     buffer->serialize(logger);
//     uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
//     if (sizeKB != lastSizeKB) {
//       logger.flush();
//       lastSizeKB = sizeKB;
//       Serial.print("[FILE] ");
//       Serial.print(sizeKB);
//       Serial.println("KB");
//     }
//   }
// #endif

  const int dataIntervals[] = DATA_INTERVAL_TABLE;
#if ENABLE_OBD || ENABLE_MEMS
  // motion adaptive data interval control
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  if (stationary) {
    // stationery timeout
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    // trip ended, go into standby
    state.clear(STATE_WORKING);
    return;
  }
#else
  dataInterval = dataIntervals[0];
#endif
  do {
    long t = dataInterval - (millis() - startTime);
    processBLE(t > 0 ? t : 0);
  } while (millis() - startTime < dataInterval);
}

void standby()
{
  state.set(STATE_STANDBY);
// #if STORAGE != STORAGE_NONE
//   if (state.check(STATE_STORAGE_READY)) {
//     logger.end();
//   }
// #endif

#if !GNSS_ALWAYS_ON && GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    Serial.println("[GNSS] OFF");
    sys.gpsEnd(true);
    state.clear(STATE_GPS_READY | STATE_GPS_ONLINE);
    gd = 0;
  }
#endif

  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  // this will put co-processor into sleep mode
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
  obd.enterLowPowerMode();
#if ENABLE_MEMS
//   calibrateMEMS();
//   waitMotion(-1);
    delay(10);
#elif ENABLE_OBD
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  Serial.println("WAKEUP");
  sys.resetLink();
#if RESET_AFTER_WAKEUP
// #if ENABLE_MEMS
//   if (mems) mems->end();  
// #endif
//   ESP.restart();
    delay(10);
#endif  
  state.clear(STATE_STANDBY);
}

void setup(){

    #if ENABLE_OLED
      oled.begin();
      oled.setFontSize(FONT_SIZE_SMALL);
    #endif
      // initialize USB serial
      Serial.begin(115200);


    #if ENABLE_OBD
      if(sys.begin()) {
        Serial.print("TYPE:");
        Serial.println(sys.devType);
        obd.begin(sys.link);
      }
    #else
      sys.begin(false, true);
    #endif

    state.set(STATE_WORKING);
}

void loop(){
  if (!state.check(STATE_WORKING)) {
        standby();
    #ifdef PIN_LED
        // if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
        delay(10);
    #endif
        initialize();
    #ifdef PIN_LED
        digitalWrite(PIN_LED, LOW);
    #endif
        return;
    }

    process();
}