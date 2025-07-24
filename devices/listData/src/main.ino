#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <queue>
#include <ArduinoJson.h>

std::queue<String> dataQueue;

// ntp server utp3:  a.ntp.br 
const char* ntpServer = "a.ntp.br";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

// Wi-fi config
const char* ssid = "Paulo";
const char* password = "15151515";
// API URL
const char* apiUrl = "http://192.168.0.106:3000/teste";

String Live_Time();

String Live_Time(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return "0000-00-00 00:00:00";
  }

  //String DataTimeInfo = timeinfo;
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();

  int ano = timeinfo.tm_year + 1900;
  int mes = timeinfo.tm_mon + 1;
  int dia = timeinfo.tm_mday;
  int hora = timeinfo.tm_hour;
  int minuto = timeinfo.tm_min;
  int segundo = timeinfo.tm_sec;
  String DataTimeInfo = String(ano) + "-" + String(mes) + "-" + String(dia) + " " + String(hora) + ":" + String(minuto) + ":" + String(segundo);

  return DataTimeInfo;
  //String(ano) + "-" + String(mes) + "-" + String(dia) + " " + String(hora) + ":" + String(minuto) + ":" + String(segundo);
  //return DataTimeInfo;
  //return;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Config NTP
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  // printLocalTime();

  Serial.println("Conectado ao WiFi");
}

void loop() {
  JsonDocument doc;

  doc["sensor"] = "gps";
  // doc["time"] = 1351824120;
  doc["data"][0] = 48.756080;
  doc["data"][1] = 2.302038;
  doc["time"] = Live_Time();
  //String jsonString = printLocalTime();
  //serializeJson(doc, jsonString);
  // Simulação de dados gerados
  //String dado = "temperatura=" + String(random(20, 30));
  //String broadcast = "brodcast" + String(WiFi.broadcastIP());
  // dataQueue.push(dado);
  //dataQueue.push(jsonString);
  String jsonString;
  serializeJson(doc, jsonString);
  dataQueue.push(jsonString);
  // dataQueue.push(broadcast);
  // Serial.println("Dado adicionado à fila: " + dado);
  HTTPClient http;
  http.begin(apiUrl);
  http.addHeader("Content-Type", "application/json");
  //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  // int httpResponseCode = http.POST(serializeJson(doc, Serial));
  //int httpResponseCode = http.POST(jsonString);
  int httpResponseCode = http.POST(jsonString);
  // while(httpResponseCode != 200){
  //   int httpResponseCode = http.POST(jsonString);
  // }

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    while (!dataQueue.empty()) {
      String data = dataQueue.front();

      http.begin(apiUrl);
      http.addHeader("Content-Type", "application/json");

      int httpResponseCode = http.POST(data);
      if (httpResponseCode == 200) {
        Serial.println("Enviado com sucesso: " + data);
        dataQueue.pop(); // remove da fila
      } else {
        Serial.println("Falha no envio, mantendo na fila");
        break; // não tenta mais até próxima vez
      }

      http.end();
    }
  } else {
    Serial.println("Sem conexão, mantendo dados na fila");
  }

  delay(10000); // Aguarda 10 segundos
  // printLocalTime();
}

// String Live_Time(){
//   struct tm timeinfo;
//   if(!getLocalTime(&timeinfo)){
//     Serial.println("Failed to obtain time");
//     return "0000-00-00 00:00:00";
//   }

//   //String DataTimeInfo = timeinfo;
//   Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
//   Serial.print("Day of week: ");
//   Serial.println(&timeinfo, "%A");
//   Serial.print("Month: ");
//   Serial.println(&timeinfo, "%B");
//   Serial.print("Day of Month: ");
//   Serial.println(&timeinfo, "%d");
//   Serial.print("Year: ");
//   Serial.println(&timeinfo, "%Y");
//   Serial.print("Hour: ");
//   Serial.println(&timeinfo, "%H");
//   Serial.print("Hour (12 hour format): ");
//   Serial.println(&timeinfo, "%I");
//   Serial.print("Minute: ");
//   Serial.println(&timeinfo, "%M");
//   Serial.print("Second: ");
//   Serial.println(&timeinfo, "%S");

//   Serial.println("Time variables");
//   char timeHour[3];
//   strftime(timeHour,3, "%H", &timeinfo);
//   Serial.println(timeHour);
//   char timeWeekDay[10];
//   strftime(timeWeekDay,10, "%A", &timeinfo);
//   Serial.println(timeWeekDay);
//   Serial.println();

//   int ano = timeinfo.tm_year + 1900;
//   int mes = timeinfo.tm_mon + 1;
//   int dia = timeinfo.tm_mday;
//   int hora = timeinfo.tm_hour;
//   int minuto = timeinfo.tm_min;
//   int segundo = timeinfo.tm_sec;
//   String DataTimeInfo = String(ano) + "-" + String(mes) + "-" + String(dia) + " " + String(hora) + ":" + String(minuto) + ":" + String(segundo);

//   return DataTimeInfo;
//   //String(ano) + "-" + String(mes) + "-" + String(dia) + " " + String(hora) + ":" + String(minuto) + ":" + String(segundo);
//   //return DataTimeInfo;
//   //return;
// }