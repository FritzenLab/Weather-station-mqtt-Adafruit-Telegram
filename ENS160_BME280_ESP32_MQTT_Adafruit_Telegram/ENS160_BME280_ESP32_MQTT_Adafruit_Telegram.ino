
// NTP time: https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
// Telegram: https://randomnerdtutorials.com/telegram-esp32-motion-detection-arduino/
// Adafruit MQTT: https://fritzenlab.net/2024/10/07/mqtt-protocol-with-esp32-and-adafruit-io/
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_ENS160.h>
#include <HTTPClient.h>
#include <UrlEncode.h>
#include "time.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -10800; // (180 * 60) GTM-3 for Brazil
const int   daylightOffset_sec = 0; // no daylight savings time

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure clientAdafruit;
WiFiClientSecure clientTelegram;

#define SEALEVELPRESSURE_HPA (1013.25)
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
Adafruit_BME280 bme; // I2C

// Initialize Telegram BOT
#define BOTtoken ""  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define CHAT_ID "" // chat

UniversalTelegramBot bot(BOTtoken, clientTelegram);

/************************* WiFi Access Point *********************************/

#define WLAN_SSID ""
#define WLAN_PASS ""

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"

// Using port 8883 for MQTTS
#define AIO_SERVERPORT  8883

// Adafruit IO Account Configuration
// (to obtain these values, visit https://io.adafruit.com and click on Active Key)
#define AIO_USERNAME ""
#define AIO_KEY      ""


float temp= 0;
float hum= 0;

/************ Global State (you don't need to change this!) ******************/

#include <Wire.h>




// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&clientAdafruit, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com root CA
const char* adafruitio_root_ca = \
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n";

/****************************** Feeds ***************************************/

long oldtime;
long oldtimeled;
int hasnotpassedhere= 1;
struct tm timeinfo;  
int h, m;

// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temperaturereadings = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturereadings");
Adafruit_MQTT_Publish humidityreadings = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidityreadings");
Adafruit_MQTT_Publish pressurereadings = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressurereadings");
Adafruit_MQTT_Publish aqireadings = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/aqireadings");
Adafruit_MQTT_Publish tvocreadings = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tvocreadings");
Adafruit_MQTT_Publish eco2readings = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/eco2readings");
/*************************** Sketch Code ************************************/

void setup() {
  Serial.begin(115200);
  pinMode(15, OUTPUT);
  delay(10);
  
  Serial.println(F("Adafruit IO MQTTS (SSL/TLS) Example"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  clientTelegram.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org

  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  clientAdafruit.setCACert(adafruitio_root_ca);

 bme.begin(0x76, &Wire);
  while( NO_ERR != ENS160.begin() ){
    Serial.println("Communication with device ENS160, please check connection");
    delay(3000);
  } 
  temp = bme.readTemperature();
  hum = bme.readHumidity();
  ENS160.setPWRMode(ENS160_STANDARD_MODE);
  ENS160.setTempAndHum(/*temperature=*/temp, /*humidity=*/hum);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  bot.sendMessage(CHAT_ID, "Starting Telegram");

}

uint32_t x=0;

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected! Attempting reconnect...");
    WiFi.disconnect();
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    unsigned long startAttemptTime = millis();

    // Wait until connected or timeout (10 seconds)
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnected to WiFi!");
    } else {
      Serial.println("Failed to reconnect WiFi. Will retry later.");
      return; // Skip rest of loop if WiFi isn't working
    }
  }

  if(millis() - oldtimeled > 200){
    oldtimeled= millis();
    digitalWrite(15, !digitalRead(15));
  }
  if(millis() - oldtime >= 60000){ // 60 seconds
    oldtime= millis();
  
    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();

    temp = bme.readTemperature();
    hum = bme.readHumidity();
    ENS160.setTempAndHum(/*temperature=*/temp, /*humidity=*/hum); // re-set ENS160's temperature and humidity for re-calibration
    float pressure = bme.readPressure() / 100.0F;
    float aqi = ENS160.getAQI();
    float tvoc = ENS160.getTVOC();
    float eco2 = ENS160.getECO2();

    // Now we can publish stuff!
    Serial.print(F("\nSending val "));
    Serial.print(temp);
    Serial.print(F(" to test feed..."));
    if (! temperaturereadings.publish(temp)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    if (! humidityreadings.publish(hum)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    if (! pressurereadings.publish(pressure)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    if (! aqireadings.publish(aqi)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    if (! tvocreadings.publish(tvoc)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    if (! eco2readings.publish(eco2)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    
    getLocalTimeVars(h, m); // NTP server "get time" function

    if ((h == 0 || h == 6 || h == 12 || h == 18) && (m == 0 || m == 1)) {  // Send message at 0h, 6h, 12h and 18h, 0 minutes or 1 minute (only once though)
      if(hasnotpassedhere == 1){ // guarantees that the message is sent only once
        hasnotpassedhere = 0;
        bot.sendMessage(CHAT_ID, "Temperature= " + String(temp) + " oC" + "\n Humidity= " + String(hum) +"%");
      }
      
    }else if((h == 0 || h == 6 || h == 12 || h == 18) && (m == 2 || m == 3)){ // on minute 2 or 3 it will reset the "send message" counter
      hasnotpassedhere = 1;
    }

  }    
  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3; // Only try a few times to reconnect
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      Serial.println("MQTT connection failed after retries.");
      return; // Give up and return. We will try again in next loop
    }
  }

  Serial.println("MQTT Connected!");
}

void getLocalTimeVars(int &hour, int &minute) { // get local time from NTP server
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    hour = -1;   // error value
    minute = -1;
    return;
  }

  // Option 1: Directly from struct tm
  hour = timeinfo.tm_hour;   // 0–23
  minute = timeinfo.tm_min;  // 0–59

}

