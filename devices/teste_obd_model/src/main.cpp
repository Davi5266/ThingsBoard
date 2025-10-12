// #include "Client.h"
#include "FreematicsBase.h"
#include <WiFi.h>
#include <Arduino.h>
// #include <cstdint>
// #include <tuple>

#define FREEMATICS_DEBUG // GPS logs details
#include <FreematicsPlus.h> // OBD lib

#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h> // MQTT
#include <Arduino_HTTP_Client.h> // HTTP
#include <ThingsBoardHttp.h>
#include <WiFiClientSecure.h>

// WiFiClient client;
WiFiClientSecure client;

// WiFi config
constexpr char WIFI_SSID[] = "WebTeste";
constexpr char WIFI_PASSWORD[] = "123123123";

// ThingsBoard config
constexpr char THINGSBOARD_SERVER[] = "192.168.123.114"; // Host or link to server
// constexpr uint16_t THINGSBOARD_PORT = 1883U; // MQTT Port
constexpr uint32_t MAX_MESSAGE_SIZE  = 1024U;

constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U; // serial camunication
constexpr size_t MAX_ATTRIBUTES = 3U;

constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

#define THINGSBOARD_SSL 0
// config thingsboard protocol
#define THINGSBOARD_PROTOCOL 1
#if THINGSBOARD_PROTOCOL == 0
constexpr uint16_t THINGSBOARD_PORT = 1883U;
// MQTT config
constexpr char CLIENT_TOKEN[] ="yuz8y66lc3efgxwwa5yi"; // userName
constexpr char CLIENT_ID[] = "q1fxlsmwkncve5gzrgua"; // clientId
constexpr char CLIENT_PASSWORD[] = "y7k1si0qrili2yvt1y9s"; // password
// Initializing Mqtt client instances
Arduino_MQTT_Client mqttClient(client);
// Initializing Thingsboard instances
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size);
#elif THINGSBOARD_PROTOCOL == 1
// HTTP config
constexpr char TOKEN[] = "v1";
constexpr uint16_t THINGSBOARD_PORT = 80U;
const char* root_ca =\
      "-----BEGIN CERTIFICATE-----\n" \
      "-----END CERTIFICATE-----\n";
// You can use x.509 client certificates if you want
const char* client_key = "";
const char* client_cert = "";
// Initalize the http client instance
Arduino_HTTP_Client httpClient(client,THINGSBOARD_SERVER, THINGSBOARD_PORT);
ThingsBoardHttp tb(httpClient, TOKEN, THINGSBOARD_SERVER, THINGSBOARD_PORT);
#endif

// telemetry config
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

// K02d@v12025
// curl -v -X POST --data "{"temperature":42,"humidity":73}" https://mobilidade.inmetro.gov.br/api/v1/08utdfxvbadhknv0szzj/telemetry --header "Content-Type:application/json"
// mosquitto_pub -d -q 1 -h mobilidade.inmetro.gov.br -p 1883 -t v1/devices/me/telemetry -u "08utdfxvbadhknv0szzj" -m "{temperature:25}"
// curl -v -X POST --data "{"attribute1": "value1", "attribute2":true, "attribute3": 43.0}" https://mobilidade.inmetro.gov.br/api/v1/08utdfxvbadhknv0szzj/attributes --header "Content-Type:application/json"

// Freematics OBD config
FreematicsESP32 sys; // Instances the system Freematics
COBD obd; // communication CAN
bool connected = false; // status communication OBD
unsigned long count = 0;
GPS_DATA* gd = nullptr; // pointer GPS data

#define STATE_MEMS_READY 0x8
#define ENABLE_MEMS true

/* MEMS Config*/
#if ENABLE_MEMS
float accBias[3] = {0};
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif
/*  data ICM_42627 */
class GYROSCOPE {
  public:
    // giroscópio (deg/s)
    float gyr_x = 0;
    float gyr_y = 0;
    float gyr_z = 0;
    // Acelerômetro (m/s²)
    float acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;
    // Magnetômetro (uT)
    float mag_x = 0;
    float mag_y = 0;
    float mag_z = 0;
    // msg
    char erroMsgData[33] = "Failed to read sensor data";
    char erroMsgInit[26] = "Uninitialized sensor"; 
};

/* MEMS functions */
MEMS_I2C* mems = 0;
class STATE {
  public:
    bool check(uint16_t flags) {return (state & flags) == flags;}
    void set(uint16_t flags) {state |= flags;}
    void clear(uint16_t flags) {state &= ~flags;}
    uint16_t state = 0;
};

STATE state;

/// @brief Init connection wifi
void InitWiFi(){

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
  }
}

/// @brief Reconnects the wifi in case any interference occurs
/// @return Returns true when the connection is successful
const bool reconnect() {
  const wl_status_t status = WiFi.status();
  if(status == WL_CONNECTED){
    return true;
  }

  InitWiFi();
  return true;
}

/// @brief It uses the beep (internal OBD sound device) to send sound response
/// @param duration receives a value of type int that determines the time this asset goes
void beep(int duration)
{
    // turn on buzzer at 2000Hz frequency 
    sys.buzzer(2000);
    delay(duration);
    // turn off buzzer
    sys.buzzer(0);
}

/* Data model for GPS */
struct MAIN_GPS{
  public:
  uint32_t ts = 0; // timestamp
  uint32_t date = 0; 
  uint32_t time = 0;
  float lat = 0; // latitude
  float lng = 0; // longitude
  float alt = 0; /* meter */
  float speed = 0; /* knot */
  uint16_t heading = 0; /* degree */
  uint8_t hdop = 0; 
  uint8_t sat = 0;  // satellites
  uint16_t sentences = 0; // 
  uint16_t errors = 0; // errors detected
};

int gpsStatus = 0; // Validates GPS startup

typedef struct {
  byte pid;
  String telemetry;
  // String telemetry;
} PIDTELEMETRY;

PIDTELEMETRY pidtelemetry[]={
  { (byte)PID_THROTTLE, "throttle"},
  { (byte)PID_BATTERY_VOLTAGE, "battery"},
  {PID_SPEED, "speed"},
  { (byte)PID_RPM, "rpm"},
  { (byte)PID_MAF_FLOW, "maf"},
  { (byte)PID_INTAKE_MAP, "map"},
  { (byte)PID_FUEL_LEVEL, "fuel_level"},
  { (byte)PID_AMBIENT_TEMP, "ambient_temp"},
  { (byte)PID_BATTERY_VOLTAGE, "battery_voltage"},
  { (byte)PID_ENGINE_LOAD, "engine_load"},
  { (byte)PID_COOLANT_TEMP,  "coolant_temp"},
  { (byte)PID_ODOMETER, "odometer"},
  { (byte)      PID_ENGINE_OIL_TEMP, "engine_oil_temp"},
  { (byte)PID_ENGINE_FUEL_RATE, "engine_fuel_rate"},
  { (byte)PID_ENGINE_LOAD, "engine_load"},
  { (byte)PID_ENGINE_REF_TORQUE, "engine_ref_torque"},
  { (byte)PID_ENGINE_TORQUE_DEMANDED, "engine_torque_demanded"},
  { (byte)PID_ENGINE_TORQUE_PERCENTAGE, "engine_torque_percentage"},
  { (byte)PID_ABSOLUTE_ENGINE_LOAD, "absolute_engine_load"},
  { (byte)PID_FUEL_INJECTION_TIMING, "fuel_injection_timing"},
};

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD); // Initiates serial communication
  delay(1000); // Small delay to stabilize
  
  // Sensor initialization ICM_42627
  if(!state.check(STATE_MEMS_READY)){
    // Serial.print("MEMS: ");
    mems = new ICM_42627;
    byte ret = mems->begin();
    if(ret){ // ICM_42627 successfully initialized
      state.set(STATE_MEMS_READY); 
    }else { // Failure to initialize ICM_42627
      delete mems;
      mems = 0;
    }
  }

  sys.begin(); // Inicia o sistema Freematics
  if (sys.gpsBegin()) { /*GPS started successfully */} 
  else { // Failed to start GPS!
    Serial.println("");
    gpsStatus = 1; // Please note that the GPS module has not been initialized
  }

  obd.begin(sys.link);

  InitWiFi();
  if(THINGSBOARD_PROTOCOL == 1){
    client.setCACert(root_ca);
    client.setCertificate(client_cert); // for client verification
    client.setPrivateKey(client_key);	// for client verification    return;
  }

}

void loop() {
  MAIN_GPS gps;
  GYROSCOPE gyroscope;

  // Checking CAN communication
  if(!connected){
    if(obd.init()){
      connected = true;
    }
    return;
  }
  // reconnect wifi
  if(!reconnect()){
    return;
  }

  // connect thingsboard server
  // if(!tb.connected()){
  //   // if(THINGSBOARD_PROTOCOL == 0){
  //   //   if(!tb.connect(THINGSBOARD_SERVER, CLIENT_TOKEN, THINGSBOARD_PORT, CLIENT_ID, CLIENT_PASSWORD)){
  //   //     return;
  //   //   }
  //   // } else 
  //   if (THINGSBOARD_PROTOCOL == 1) {
  //     if(!tb.connect(THINGSBOARD_SERVER, THINGSBOARD_PORT)){
  //       return;
  //     }
  //   }
  // }

  // detects errors in obd communication and reconnects communication
  if(obd.errors > 2){
    connected = false;
    obd.reset();
  }

  /* GPS read */
  int gpsTypeError = 0;
  if (sys.gpsGetData(&gd) && gd != nullptr) { // Gets pointer to GPS data
    if (gd->sat >= 4) { // Check for GPS fix (minimum of 4 satellites)
    
      // Serial.print("Latitude: "); Serial.println(gd->lat, 6);
      // Serial.print("Longitude: "); Serial.println(gd->lng, 6);
      // Serial.print("Altitude: "); Serial.println(gd->alt, 2); 
      // Serial.print("Velocidade: "); Serial.println(gd->speed * 1.852, 2); // Converte nós para km/h
      // Serial.print("Direção: "); Serial.println(gd->heading); // Graus
      // Serial.print("Satélites: "); Serial.println(gd->sat);
      // Serial.print("HDOP: "); Serial.println(gd->hdop); // Precisão
      // Serial.print("Data: "); Serial.println(gd->date); // Formato DDMMAA
      // Serial.print("Hora: "); Serial.println(gd->time); // Formato HHMMSSsss
      // Serial.print("Timestamp: "); Serial.println(gd->ts); // Milissegundos
    
      gps.lat = gd -> lat, 6;
      gps.lng = gd -> lng, 6;
      gps.alt = gd -> alt, 2; // Metros
      gps.speed = gd -> speed * 1.852, 2; // Converte nós para km/h
      gps.heading = gd -> heading; // Graus
      gps.sat = gd -> sat;
      gps.hdop = gd -> hdop; // Precisão
      gps.date = gd -> date; // Formato DDMMAA
      gps.time = gd -> time; // Formato HHMMSSsss
      gps.ts = gd -> ts; // Milissegundos
    } else {
      gpsTypeError = 2;
    }
  } else {
    gpsTypeError = 1;
  }

  int gyrosStates = 0;
  if (state.check(STATE_MEMS_READY)){
    if(mems->read(acc, gyr, mag)){
      gyroscope.acc_x = acc[0];
      gyroscope.acc_y = acc[1];
      gyroscope.acc_z = acc[2];

      gyroscope.gyr_x = gyr[0];
      gyroscope.gyr_y = gyr[1];
      gyroscope.gyr_z = gyr[2];

      gyroscope.mag_x = mag[0];
      gyroscope.mag_y = mag[1];
      gyroscope.mag_z = mag[2];
    }else{
      gyrosStates = 1;
    }
  }else{
    gyrosStates = 2;
  }

  if (millis() - previousDataSend > telemetrySendInterval){
    previousDataSend = millis();
    int value;
    // device status
    tb.sendTelemetryData("device_voltage", obd.getVoltage());
    tb.sendTelemetryData("device_state", obd.getState());
    tb.sendTelemetryData("device_temp",obd.readPID(PID_DEVICE_TEMP, value));
    tb.sendTelemetryData("device_hall", obd.readPID(PID_DEVICE_HALL,value));
  //   tb.sendTelemetryData("device_cpu_freq", ESP.getCpuFreqMHz());
  //   tb.sendTelemetryData("device_free_heap", ESP.getFreeHeap());
  //   tb.sendTelemetryData("device_heap_size", ESP.getHeapSize());
  //  //tb.sendTelemetryData("device_chip_id", ESP.getChipId());
  //   tb.sendTelemetryData("device_cycle_count", ESP.getCycleCount());
    // Status CAN
    // for(int i = 0; i < 18; i++){

    for(int i = 0; i < sizeof(pidtelemetry)/ sizeof(pidtelemetry[0]);i++){
      // printf("o");
      int value;
      obd.readPID(pidtelemetry[i].pid,value);
      // String telemetryData = String(pidtelemetry[i].telemetry);
      String car = pidtelemetry[i].telemetry;
      // const char tetete[] = pidfood[i].name;
      tb.sendTelemetryData(car.c_str(), value);
    }

    // GPS telemetry data
    if( gpsStatus == 0){
      tb.sendTelemetryData("gps_lat", gps.lat);
      tb.sendTelemetryData("gps_lng",gps.lng);
      tb.sendTelemetryData("gps_alt",gps.alt);
      tb.sendTelemetryData("gps_speed",gps.speed);
      tb.sendTelemetryData("gps_heading",gps.heading);
      tb.sendTelemetryData("gps_sat",gps.sat);
      tb.sendTelemetryData("gps_hdop",gps.hdop);
      tb.sendTelemetryData("gps_date",gps.date);
      tb.sendTelemetryData("gps_time",gps.time);
      tb.sendTelemetryData("timestamp",gps.ts);    

      if(gpsStatus == 1){
        tb.sendTelemetryData("gps_msg", "No GPS data available"); // Mensagem caso satélite esteja fora do ar ou não consiga se comunicar
      }else if(gpsStatus == 2){
        tb.sendTelemetryData("gps_msg", "Waiting to fix GPS Satellites");
      } else {
        tb.sendTelemetryData("gps_msg","gps ok");
      }
    } else {
      tb.sendTelemetryData("gps_msg","");
    }

    // ICM_42627 telemetry data
    if(gyrosStates == 1){
      tb.sendTelemetryData("msg", gyroscope.erroMsgData);
    } else if (gyrosStates == 2)
    {
      tb.sendTelemetryData("msg", gyroscope.erroMsgInit);
    } else {
      tb.sendTelemetryData("acc_x", gyroscope.acc_x);
      tb.sendTelemetryData("acc_y", gyroscope.acc_y);
      tb.sendTelemetryData("acc_z", gyroscope.acc_z);

      tb.sendTelemetryData("gyr_x", gyroscope.gyr_x);
      tb.sendTelemetryData("gyr_y", gyroscope.gyr_y);
      tb.sendTelemetryData("gyr_z", gyroscope.gyr_z);

      tb.sendTelemetryData("mag_x", gyroscope.mag_x);
      tb.sendTelemetryData("mag_y", gyroscope.mag_y);
      tb.sendTelemetryData("mag_z", gyroscope.mag_z);
    }
    

  }
  // tb.loop();
  delay(1000); // stop 1 second
}