/******************************************************************************
* Simple OBD test sketch for Freematics ONE+
* Written by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 99
#endif

#include <FreematicsPlus.h> // Funcões do freematics
// Funcões e configurações para comunicação com o ThingsBoard
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>

// WiFi config
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";

// {
//clientId:"q1fxlsmwkncve5gzrgua",
//userName:"yuz8y66lc3efgxwwa5yi",
//password:"y7k1si0qrili2yvt1y9s"
// }

// MQTT config
constexpr char CLIENT_TOKEN[] = "yuz8y66lc3efgxwwa5yi";
constexpr char CLIENT_ID[] = "q1fxlsmwkncve5gzrgua";
constexpr char CLIENT_PASSWORD[] = "y7k1si0qrili2yvt1y9s";

// Configurações de comunicação com o servidor
constexpr char THINGSBOARD_SERVER[] = "192.168.0.107";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr size_t MAX_ATTRIBUTES = 3U;

constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;  


// Attribute names for attribute request and attribute updates functionality

constexpr const char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr const char LED_MODE_ATTR[] = "ledMode";
constexpr const char LED_STATE_ATTR[] = "ledState";


WiFiClient wifiClient;

// Arduino_MQTT_Client mqttClient(wifiClient);


#define PIN_LED 4

// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(wifiClient);

// Initialize used apis
Server_Side_RPC<3U, 5U> rpc;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
Shared_Attribute_Update<3U, MAX_ATTRIBUTES> shared_update;

const std::array<IAPI_Implementation*, 3U> apis = {
    &rpc,
    &attr_request,
    &shared_update
};

// Initialize ThingsBoard instance with the maximum needed buffer size, stack size and the apis we want to use
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);

// handle led state and mode changes
volatile bool attributesChanged = false;

// LED modes: 0 - continious state, 1 - blinking
volatile int ledMode = 0;

// Current led state
volatile bool ledState = false;

// Settings for interval in blinking mode
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

// For telemetry
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

// List of shared attributes for subscribing to their updates
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

// List of client attributes for requesting them (Using to initialize device states)
constexpr std::array<const char *, 1U> CLIENT_ATTRIBUTES_LIST = {
  LED_MODE_ATTR
};

/// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been succesfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}


/// @brief Processes function for RPC call "setLedMode"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
void processSetLedMode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set led state RPC method");

  // Process data
  int new_mode = data;

  Serial.print("Mode to change: ");
  Serial.println(new_mode);
  StaticJsonDocument<1> response_doc;

  if (new_mode != 0 && new_mode != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }

  ledMode = new_mode;

  attributesChanged = true;

  // Returning current mode
  response_doc["newMode"] = (int)ledMode;
  response.set(response_doc);
}


// Optional, keep subscribed shared attributes empty instead,
// and the callback will be called for every shared attribute changed on the device,
// instead of only the one that were entered instead
const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setLedMode", processSetLedMode }
};


/// @brief Update callback that will be called as soon as one of the provided shared attributes changes value,
/// if none are provided we subscribe to any shared attribute change instead
/// @param data Data containing the shared attributes that were changed and their current value
void processSharedAttributes(const JsonObjectConst &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      if (LED_BUILTIN != 99) {
        digitalWrite(LED_BUILTIN, ledState);
      }
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

void processClientAttributes(const JsonObjectConst &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), LED_MODE_ATTR) == 0) {
      const uint16_t new_mode = it->value().as<uint16_t>();
      ledMode = new_mode;
    }
  }
}

// Attribute request did not receive a response in the expected amount of microseconds 
void requestTimedOut() {
  Serial.printf("Attribute request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the keys actually exist on the target device\n", REQUEST_TIMEOUT_MICROSECONDS);
}

const Shared_Attribute_Callback<MAX_ATTRIBUTES> attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback<MAX_ATTRIBUTES> attribute_shared_request_callback(&processSharedAttributes, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES_LIST);
const Attribute_Request_Callback<MAX_ATTRIBUTES> attribute_client_request_callback(&processClientAttributes, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, CLIENT_ATTRIBUTES_LIST);


FreematicsESP32 sys;
COBD obd;
bool connected = false;
unsigned long count = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);

  // initializations
  while (!sys.begin());
  obd.begin(sys.link);
  InitWiFi();
}

void loop() {
  digitalWrite(PIN_LED, HIGH);
  // put your main code here, to run repeatedly:
  if (!connected) {
    digitalWrite(PIN_LED, HIGH);
    Serial.print("Connecting to OBD...");
    if (obd.init()) {
      Serial.println("OK");
      connected = true;
    } else {
      Serial.println();
    }
    digitalWrite(PIN_LED, LOW);
    return;
  }

  if( !reconnect()){
    return;
  }

  if(!tb.connected()){
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(CLIENT_TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, CLIENT_TOKEN, THINGSBOARD_PORT, CLIENT_ID, CLIENT_PASSWORD)) {
      Serial.println("Failed to connect");
      return;
    }
    // Sending a MAC address as an attribute
    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

    Serial.println("Subscribing for RPC...");
    // Perform a subscription. All consequent data processing will happen in
    // processSetLedState() and processSetLedMode() functions,
    // as denoted by callbacks array.
    if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    if (!shared_update.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Failed to subscribe for shared attribute updates");
      return;
    }

    Serial.println("Subscribe done");

    // Request current states of shared attributes
    if (!attr_request.Shared_Attributes_Request(attribute_shared_request_callback)) {
      Serial.println("Failed to request for shared attributes");
      return;
    }

    // Request current states of client attributes
    if (!attr_request.Client_Attributes_Request(attribute_client_request_callback)) {
      Serial.println("Failed to request for client attributes");
      return;
    }
  }
  
  if(!tb.connect(THINGSBOARD_SERVER, CLIENT_TOKEN,THINGSBOARD_PORT, CLIENT_ID, CLIENT_PASSWORD)){
    Serial.print("Failed to connect");
    return;
  }

  int value;
  Serial.print('[');
  Serial.print(millis());
  Serial.print("] #");
  Serial.print(count++);
  if (obd.readPID(PID_RPM, value)) {
    Serial.print(" RPM:");
    Serial.print(value);
  }
  if (obd.readPID(PID_SPEED, value)) {
    Serial.print(" SPEED:");
    Serial.print(value);
  }

  Serial.print(" BATTERY:");
  Serial.print(obd.getVoltage());
  Serial.print('V');

  Serial.print(" CPU TEMP:");
  Serial.print(readChipTemperature());
  Serial.println();
  if (obd.errors > 2) {
    Serial.println("OBD disconnected");
    connected = false;
    obd.reset();
  }
  digitalWrite(PIN_LED, LOW);

  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();
    tb.sendTelemetryData("battery", obd.getVoltage());
    obd.readPID(PID_SPEED, value);
    tb.sendTelemetryData("speed", value);
    obd.readPID(PID_RPM, value);
    tb.sendTelemetryData("rpm", value);
    tb.sendTelemetryData("cputemp", readChipTemperature());
    tb.sendAttributeData("state", obd.getState());
    // tb.sendTelemetryJson("datateste");
    // tb.sendAttributeData("speed", random(60, 100));
    // tb.sendAttributeData("batery", random(50, 100));
    
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
  }

  tb.loop();

  // delay(100);
}
