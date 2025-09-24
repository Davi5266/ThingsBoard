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
COBD obd;
bool connected = false;
unsigned long count = 0;
GPS_DATA* gd = nullptr; // Ponteiro para os dados do GPS

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
    sys.buzzer(2000);
    delay(duration);
    // turn off buzzer
    sys.buzzer(0);
}

struct MAIN_GPS{
  public:
  float latitude = 0;
  uint32_t ts_T = 0;
  uint32_t date_T = 0;
  uint32_t time_T = 0;
  float lat_T = 0;
  float lng_T = 0;
  float alt_T = 0; /* meter */
  float speed_T = 0; /* knot */
  uint16_t heading_T = 0; /* degree */
  uint8_t hdop_T = 0;
  uint8_t sat_T = 0;
  uint16_t sentences_T = 0;
  uint16_t errors_T = 0;
  bool gps_ok = false;
  char msg[30] = "Nenhum dado GPS disponível ";
};

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD); // Inicia a comunicação serial
  delay(1000); // Pequeno atraso para estabilizar
  
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
      gps.lat_T = gd -> lat, 6;
      gps.lng_T = gd -> lng, 6;
      gps.alt_T = gd -> alt, 2;
      gps.speed_T = gd -> speed * 1.852, 2;
      gps.heading_T = gd -> heading;
      gps.sat_T = gd -> sat;
      gps.hdop_T = gd -> hdop;
      gps.date_T = gd -> date;
      gps.time_T = gd -> time;
      gps.ts_T = gd -> ts;
    } else {
      Serial.println("Aguardando fix GPS... Satélites: " + String(gd->sat));
      gps.gps_ok = true;
    }
  } else {
    Serial.println("Nenhum dado GPS disponível.");
    gps.gps_ok = true;
  }

  if (millis() - previousDataSend > telemetrySendInterval){
    previousDataSend = millis();

    // device status
    tb.sendTelemetryData("device_voltage", obd.getVoltage());
    tb.sendTelemetryData("state", obd.getState());
    tb.sendTelemetryData("device_voltage", obd.getVoltage());
    
    // Status CAN
    int value;
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

    // Telemtria do GPS
    tb.sendTelemetryData("latitude", gps.lat_T);
    tb.sendTelemetryData("longitude",gps.lng_T);
    tb.sendTelemetryData("altitude",gps.alt_T);
    tb.sendTelemetryData("velocidade",gps.speed_T);
    tb.sendTelemetryData("direção",gps.heading_T);
    tb.sendTelemetryData("satélites",gps.sat_T);
    tb.sendTelemetryData("HDOP",gps.hdop_T);
    tb.sendTelemetryData("data",gps.date_T);
    tb.sendTelemetryData("hora",gps.time_T);
    tb.sendTelemetryData("timestamp",gps.ts_T);    

    if(gps.gps_ok){
      tb.sendTelemetryData("msg", gps.msg); // Mensagem caso satélite esteja fora do ar ou não consiga se comunicar
    }else{
      tb.sendTelemetryData("msg", "GPS ok");

    }
  }
  tb.loop();
  delay(1000); // Aguarda 1 segundo
}
