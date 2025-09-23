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
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";

// MQTT config
constexpr char CLIENT_TOKEN[] = "";
constexpr char CLIENT_ID[] = "";
constexpr char CLIENT_PASSWORD[] = "";

// ThingsBoard config
constexpr char THINGSBOARD_SERVER[] = "";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE  = 1024U;

constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U; // comunicação serial do esp32
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

float latitude;
uint32_t ts_T;
uint32_t date_T;
uint32_t time_T;
float lat_T;
float lng_T;
float alt_T; /* meter */
float speed_T; /* knot */
uint16_t heading_T; /* degree */
uint8_t hdop_T;
uint8_t sat_T;
uint16_t sentences_T;
uint16_t errors_T;

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
  beep(100);
  beep(100);
}

void loop() {

  // Verificando a comunicação CAN
  if(!connected){
    if(obd.init()){
      connected = true;
    }
    return;
  }

  if(!reconnect()){
    return;
  }

  if(!tb.connected()){
    if(!tb.connect(THINGSBOARD_SERVER, CLIENT_TOKEN, THINGSBOARD_PORT, CLIENT_ID, CLIENT_PASSWORD)){
      return;
    }
  }

  int value;
  obd.readPID(PID_RPM, value);

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
      lat_T = gd -> lat, 6;
      lng_T = gd -> lng, 6;
      alt_T = gd -> alt, 2;
      speed_T = gd -> speed * 1.852, 2;
      heading_T = gd -> heading;
      sat_T = gd -> sat;
      hdop_T = gd -> hdop;
      date_T = gd -> date;
      time_T = gd -> time;
      ts_T = gd -> ts;
    } else {
      Serial.println("Aguardando fix GPS... Satélites: " + String(gd->sat));
    }
  } else {
    Serial.println("Nenhum dado GPS disponível.");
  }

  if (millis() - previousDataSend > telemetrySendInterval){
    previousDataSend = millis();

    tb.sendTelemetryData("battery", obd.getVoltage());
    obd.readPID(PID_THROTTLE, value);
    tb.sendTelemetryData("throttle", value);
    tb.sendTelemetryData("latitude", lat_T);
    tb.sendTelemetryData("longitude",lng_T);
    tb.sendTelemetryData("altitude",alt_T);
    tb.sendTelemetryData("velocidade",speed_T);
    tb.sendTelemetryData("direção",heading_T);
    tb.sendTelemetryData("satélites",sat_T);
    tb.sendTelemetryData("HDOP",hdop_T);
    tb.sendTelemetryData("data",date_T);
    tb.sendTelemetryData("hora",time_T);
    tb.sendTelemetryData("timestamp",ts_T);
  }
  tb.loop();
  delay(1000); // Aguarda 1 segundo
}
