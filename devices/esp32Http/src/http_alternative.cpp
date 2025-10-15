#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

#define TOKEN "j1fzyb5iwr01xkd4wow2"

#define WIFI_SSID "Paulo"
#define WIFI_PASSWORD "15151515"

#define SERVER_ADDR ""
#define SERVER_PORT ""
#define SERVER_ROUTE ""

/// @brief Initalizes WiFi connection, will endlessly delay until a connection has been successfully established
void InitWifi() {

    if(WiFi.status() == WL_CONNECTED)
    {
        return;
    }
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
    }
}

void postTelemetry(){
    // curl -v -X POST http://0.0.0.0:8080/api/v1/j1fzyb5iwr01xkd4wow2/telemetry --header Content-Type:application/json --data "{temperature:25}"
    
    HTTPClient http;
    // http://mobilidade.inmetro.gov.br:8080/api/v1/n6432lwt7a0nstr4arhh/telemetry
    http.begin("http://mobilidade.inmetro.gov.br:8080/api/v1/n6432lwt7a0nstr4arhh/telemetry");
    http.addHeader("Content-Type", "application/json");
    // http.addHeader("Authorization","Bearer " + String(TOKEN));
        /*
        Resposta Json para o endpoint
        {
          "humidity": 0,
          "temperature_c": 0,
          "temperature_f": 0,
          "client_id": 0
        }
        */
       // arquivando dados em json
    // String jsonPayload = "{\"humidity\":" + String(h,2) +",\"temperature_c\":" + String(hic,2) +",\"temperature_f\":" + String(hif,2) +",\"client_id\":\"0\"}";
    
    String jsonPayload = "{\"testando\":\"http\"}";

    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode == 200) {
        String resposta = http.getString();
        Serial.println("Código HTTP: " + String(httpResponseCode));
        Serial.println("Resposta: " + resposta);
    } else {
        Serial.println("Erro na requisição: " + String(httpResponseCode));
    }

    http.end();
}

void setup(){
    Serial.begin(115200);
    InitWifi();
}

void loop() {
    InitWifi();
    Serial.println("SUCESSO!!!!!");
    postTelemetry();
}