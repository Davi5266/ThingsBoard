#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#include <Arduino.h>

#define FREEMATICS_DEBUG // logs detalhados
#include <FreematicsPlus.h>

#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>

// WiFi config
constexpr char WIFI_SSID[] = "WebTeste";
constexpr char WIFI_PASSWORD[] = "123123123";

// MQTT config
constexpr char CLIENT_TOKEN[] ="yuz8y66lc3efgxwwa5yi";
constexpr char CLIENT_ID[] = "q1fxlsmwkncve5gzrgua";
constexpr char CLIENT_PASSWORD[] = "y7k1si0qrili2yvt1y9s";

/*
{
	clientId:"q1fxlsmwkncve5gzrgua",
	userName:"yuz8y66lc3efgxwwa5yi",
	password:"y7k1si0qrili2yvt1y9s"
}
*/

// ThingsBoard config
constexpr char THINGSBOARD_SERVER[] = "192.168.37.114";
constexpr uint16_t THINGSBOARD_PORT = 1883U; // porta MQTT
constexpr uint32_t MAX_MESSAGE_SIZE  = 1024U;

constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U; // comunicação serial do Freematics OBD
constexpr size_t MAX_ATTRIBUTES = 3U;

constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

WiFiClient wifiClient;

// Inicializando as instancias Mqtt client
Arduino_MQTT_Client mqttClient(wifiClient);

// Inicializando as intancias do Thingsboard
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size);

// Para telemetria
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

// Freematics OBD config
FreematicsESP32 sys; // Instancia o sistema Freematics
COBD obd; // comunicação CAN
bool connected = false;
unsigned long count = 0;
GPS_DATA* gd = nullptr; // Ponteiro para os dados do GPS

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
/* dados do ICM_42627 */
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
    char erroMsgData[33] = "Falha ao ler os dados do sensor";
    char erroMsgInit[26] = "Sensor não inicializado"; 
};

MEMS_I2C* mems = 0;
class STATE {
  public:
    bool check(uint16_t flags) {return (state & flags) == flags;}
    void set(uint16_t flags) {state |= flags;}
    void clear(uint16_t flags) {state &= ~flags;}
    uint16_t state = 0;
};

STATE state;
GYROSCOPE gyroscope;
/// @brief Inicia a conexão wifi
void InitWiFi(){

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
  }
}

/// @brief Reconecta o wifi, caso ocorra alguma interferência
/// @return Retorna true quando a conexão for bem sucedida
const bool reconnect() {
  const wl_status_t status = WiFi.status();
  if(status == WL_CONNECTED){
    return true;
  }

  InitWiFi();
  return true;
}

/// @brief Utiliza o bip(dispositivo sonoro interno o OBD) para enviar reposta sonora
/// @param duration recebe um valor do tipo int que determina o tempo em que vai esta ativo
void beep(int duration)
{
    // turn on buzzer at 2000Hz frequency 
    //sys.buzzer(2000);
    delay(duration);
    // turn off buzzer
    //sys.buzzer(0);
}

struct MAIN_GPS{
  public:
  float latitude = 0;
  uint32_t ts = 0;
  uint32_t date = 0;
  uint32_t time = 0;
  float lat = 0;
  float lng = 0;
  float alt = 0; /* meter */
  float speed = 0; /* knot */
  uint16_t heading = 0; /* degree */
  uint8_t hdop = 0;
  uint8_t sat = 0;
  uint16_t sentences = 0;
  uint16_t errors = 0;
  bool gps_ok = false;
  char msg[30] = "Nenhum dado GPS disponível ";
};

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD); // Inicia a comunicação serial
  delay(1000); // Pequeno atraso para estabilizar
  
  //Inicialização do sensor
  if(!state.check(STATE_MEMS_READY)){
    Serial.print("MEMS: ");
    mems = new ICM_42627;
    byte ret = mems->begin();
    if(ret){
      state.set(STATE_MEMS_READY);
      Serial.println("ICM_42627 inicializado com sucesso");
    }else {
      Serial.println("Falha ao inicializar ICM_42627");
      delete mems;
      mems = 0;
    }
  }

  sys.begin(); // Inicia o sistema Freematics
  if (sys.gpsBegin()) { // Inicia o módulo GPS
    Serial.println("GPS iniciado com sucesso!");
  } else {
    Serial.println("Falha ao iniciar o GPS!");
  }

  obd.begin(sys.link);

  InitWiFi(); // Inicializando a comunicação wifi

  /* comunica que as configurações foram bem sucedidas */
  beep(500);
  delay(200);
  beep(500);
}

void loop() {
  MAIN_GPS gps;

  // Verificando a comunicação CAN
  if(!connected){
    if(obd.init()){
      connected = true;
      delay(500);
      beep(500);
      delay(500);
      beep(500);
      delay(500);
    }
    return;
  }

  if(!reconnect()){
      delay(500);
      beep(500);
      delay(500);
      beep(500);
      delay(500);
      beep(500);
    return;
  }

  if(!tb.connected()){
    if(!tb.connect(THINGSBOARD_SERVER, CLIENT_TOKEN, THINGSBOARD_PORT, CLIENT_ID, CLIENT_PASSWORD)){
      return;
    }
  }

  if(obd.errors > 2){
    connected = false;
    obd.reset();
  }

  /* Leitura do GPS */
  if (sys.gpsGetData(&gd) && gd != nullptr) { // Obtém ponteiro para os dados do GPS
    if (gd->sat >= 4) { // Verifica se há fix GPS (mínimo de 4 satélites)
      Serial.print("Latitude: "); Serial.println(gd->lat, 6);
      Serial.print("Longitude: "); Serial.println(gd->lng, 6);
      Serial.print("Altitude: "); Serial.println(gd->alt, 2); // Metros
      Serial.print("Velocidade: "); Serial.println(gd->speed * 1.852, 2); // Converte nós para km/h
      Serial.print("Direção: "); Serial.println(gd->heading); // Graus
      Serial.print("Satélites: "); Serial.println(gd->sat);
      Serial.print("HDOP: "); Serial.println(gd->hdop); // Precisão
      Serial.print("Data: "); Serial.println(gd->date); // Formato DDMMAA
      Serial.print("Hora: "); Serial.println(gd->time); // Formato HHMMSSsss
      Serial.print("Timestamp: "); Serial.println(gd->ts); // Milissegundos
      gps.lat = gd -> lat, 6;
      gps.lng = gd -> lng, 6;
      gps.alt = gd -> alt, 2;
      gps.speed = gd -> speed * 1.852, 2;
      gps.heading = gd -> heading;
      gps.sat = gd -> sat;
      gps.hdop = gd -> hdop;
      gps.date = gd -> date;
      gps.time = gd -> time;
      gps.ts = gd -> ts;
    } else {
      Serial.println("Aguardando fix GPS... Satélites: " + String(gd->sat));
      gps.gps_ok = true;
    }
  } else {
    Serial.println("Nenhum dado GPS disponível.");
    gps.gps_ok = true;
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
    tb.sendTelemetryData("state", obd.getState());
    tb.sendTelemetryData("device_temp",obd.readPID(PID_DEVICE_TEMP, value));

    // Status CAN
    obd.readPID(PID_THROTTLE, value);
    tb.sendTelemetryData("throttle", value);
    obd.readPID(PID_BATTERY_VOLTAGE, value);
    tb.sendTelemetryData("battery",value);
    obd.readPID(PID_SPEED, value);
    tb.sendTelemetryData("speed",value);
    obd.readPID(PID_RPM, value);
    tb.sendTelemetryData("rpm",value);
    obd.readPID(PID_MAF_FLOW, value);
    tb.sendTelemetryData("maf",value);
    obd.readPID(PID_INTAKE_MAP, value);
    tb.sendTelemetryData("map",value);
    obd.readPID(PID_FUEL_LEVEL, value);
    tb.sendTelemetryData("fuel_level",value);
    obd.readPID(PID_AMBIENT_TEMP, value);
    tb.sendTelemetryData("ambient_temp",value);
    obd.readPID(PID_BATTERY_VOLTAGE, value);
    tb.sendTelemetryData("battery_voltage",value);
    obd.readPID(PID_ENGINE_LOAD, value);
    tb.sendTelemetryData("engine_load",value);
    obd.readPID(PID_COOLANT_TEMP, value);
    tb.sendTelemetryData("coolant_temp",value);
    obd.readPID(PID_ODOMETER, value);
    tb.sendTelemetryData("odometer",value);
    obd.readPID(PID_ENGINE_OIL_TEMP, value);
    tb.sendTelemetryData("engine_oil_temp", value);
    obd.readPID(PID_ENGINE_FUEL_RATE, value);
    tb.sendTelemetryData("engine_fuel_rate", value);
    obd.readPID(PID_ENGINE_LOAD, value);
    tb.sendTelemetryData("engine_load", value);
    obd.readPID(PID_ENGINE_REF_TORQUE, value);
    tb.sendTelemetryData("engine_ref_torque", value);
    obd.readPID(PID_ENGINE_TORQUE_DEMANDED, value);
    tb.sendTelemetryData("engine_torque_demanded", value);
    obd.readPID(PID_ENGINE_TORQUE_PERCENTAGE, value);
    tb.sendTelemetryData("engine_torque_percentage", value);
    obd.readPID(PID_ABSOLUTE_ENGINE_LOAD, value);
    tb.sendTelemetryData("absolute_engine_load", value);
    obd.readPID(PID_FUEL_INJECTION_TIMING, value);
    tb.sendTelemetryData("fuel_injection_timing", value);

    // Telemtria do GPS
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

    if(gps.gps_ok){
      tb.sendTelemetryData("msg", gps.msg); // Mensagem caso satélite esteja fora do ar ou não consiga se comunicar
    }else{
      tb.sendTelemetryData("msg", "GPS ok");
    }

    // Telemetria do ICM_42627
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
  tb.loop();
  delay(1000); // Aguarda 1 segundo
}
