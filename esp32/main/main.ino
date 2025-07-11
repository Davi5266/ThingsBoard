#include <WiFi.h>
#include <HTTPClient.h>
#include "DHT.h"

const char* ssid = "";
const char* password = "";
const char* urlApi = "http://192.168.0.106:8080/api/v1/16vW4lASI9p0OCmyTNSy/telemetry";
const char* clientId = "e3fdf020-5dd6-11f0-ab6e-2957d86ec524";


String accessToken = " ";

// Configuração do sensor DHT
#define DHTPIN 4 // Pino que vai entregar os dados
#define DHTTYPE DHT22 // Tipo de sensor DHT

DHT dht(DHTPIN, DHTTYPE);

void setup() {
    Serial.begin(115200);
    connect_wifi();
    //login();
    //testeRouter();
    dht.begin();
}

void loop() {
     delay(100); // 1 minuto

    // Coletando dados do DHT
    // Umidade
    float h = dht.readHumidity();
    // Temperatura em Celsius
    float t = dht.readTemperature();
    // Temperature em Fahrenheit
    float f = dht.readTemperature(true);

    // Verificação de leitura
    if(isnan(h) || isnan(t) || isnan(f)) {
        Serial.println(F("Falha na leitura do sensor DHT!"));
    }

    // Calculando o índice de temperatura 
    //em Fahrenheit
    float hif = dht.computeHeatIndex(f, h);
    //em Celsius
    float hic = dht.computeHeatIndex(t, h, false);
    // login();
    //testeRouter();
    register_dht(h, hic, hif);
}

/* Função: Registra dados do sensor DHT na API
 * Parâmetros: h(umidade), hic(temperatura em Celsius ), hic(temperatura em Fahrenheit)
 * Retorno: nenhum
 */
void register_dht(float h, float hic, float hif){
    connect_wifi();
    
    HTTPClient http;

    http.begin(urlApi);
    http.addHeader("Content-Type", "application/json");
    // http.addHeader("Authorization","Bearer " + accessToken);
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
    String jsonPayload = "{\"humidity\":" + String(h,2) +",\"temperature_c\":" + String(hic,2) +",\"temperature_f\":" + String(hif,2) +",\"client_id\":\"0\"}";
    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode == 200) {
        String resposta = http.getString();
        Serial.println("Código HTTP: " + String(httpResponseCode));
        Serial.println("Resposta: " + resposta);
    } else {
        Serial.println("Erro na requisição: " + String(httpResponseCode));
    }
    /*
    if(httpResponseCode >= 400){
	login();
	}
    */

    http.end();
    
}


/* Função: realiza conexão com a rede wi-fi
 * Parâmetros: nenhum
 * Retorno: nenhum 
 */
void connect_wifi(void) 
{
    //const char* ssid = ""; // Nome da rede
    //const char* password = ""; // Senha da rede
    if (WiFi.status() == WL_CONNECTED)
        return;
        
    WiFi.begin(ssid, password);
    Serial.print("Conectando ao Wi-Fi");    
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
    Serial.print("\nConectado!");  
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

/* Função: Realiza login com a API, e extrai o token
 * Parâmetros: nenhum
 * Retorno: nenhum 
 */
void login(){
    // String route = "client/login";
    // const char* loginUrl = "http://192.168.0.113:8000/client/login";
    connect_wifi();
    HTTPClient http;
    http.begin("http://192.168.0.113:8000/client/login");
    http.addHeader("Content-Type","application/x-www-form-urlencoded");

    String postData = "username=me&password=me";
    int httpResponseCode = http.POST(postData);

    if (httpResponseCode == 200){
      String payload = http.getString();
      Serial.println("Resposta do login:");
      Serial.println(payload);
      // Extrair o token manualmente (simplesmente)
      int start = payload.indexOf("access_token\":\"") + 15;
      int end = payload.indexOf("\"", start);
      accessToken = payload.substring(start, end);
      Serial.print("Token obtido: ");
      Serial.println(accessToken);
    } else {
      Serial.print("Erro no login: ");
      Serial.println(httpResponseCode);
    }

    http.end();
}

void testeRouter(){
    connect_wifi();
    HTTPClient http;
    http.begin("http://192.168.0.113:8000/");
    http.GET();    
}
