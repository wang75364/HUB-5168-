#include <HttpClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include "BH1750.h"
#include "DHT.h"

char ssid[] = "ssid"; // your network SSID (name)
char pass[] = "password";     // your network password (use for WPA, or use as key for WEP)
// Name of the server we want to connect to
const char kHostname[] = "api.thingspeak.com";
String kPath = "/update?api_key=APIKEY";
const int kNetworkTimeout = 30 * 1000;
const int kNetworkDelay = 1000;
int status = WL_IDLE_STATUS;

BH1750 lightMeter;
float lux = 0;

#define DHT_Pin 2
#define DHT_TYPE DHT11
DHT dht(DHT_Pin, DHT_TYPE);
unsigned long dht_delay_time=0;

#define analogPin A2
float soil_moisture_value=0;

#define relay_pin 4

void setup() {
    Serial.begin(115200);
    pinMode(relay_pin,OUTPUT);
    digitalWrite(relay_pin,LOW);
    Serial.println(F("DHTxx test!"));
    //dht.begin();
    Wire.begin();
    lightMeter.begin();
    Serial.println(F("BH1750 Test begin"));
    
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        // wait 10 seconds for connection:
        delay(10000);
    }
    Serial.println("Connected to wifi");
    printWifiStatus();
}

void loop() {
    if(millis() - dht_delay_time > 15000){
        
        float h = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float t = dht.readTemperature();
        // Check if any reads failed and exit early (to try again).
        if (isnan(h) || isnan(t)) {
            Serial.println(F("Failed to read from DHT sensor!"));
            return;
        }
        float h = 50.0;
        // Read temperature as Celsius (the default)
        float t = 25.5;
        Serial.print("Humidity: ");
        Serial.print(h);
        Serial.print("% ,Temperature: ");
        Serial.print(t);
        Serial.println("°C ");

        Wire.end();
        Wire.begin();
        lux = lightMeter.readLightLevel();
        if(lux == -2){
          lightMeter.configure();
          lux = lightMeter.readLightLevel();
        }
        Serial.print("Light: ");
        Serial.print(lux);
        Serial.println(" lx");
        Transmit_data(h,t,soil_moisture_value,lux,kPath);
        dht_delay_time = millis();
    }
    soil_moisture_value=analogRead(analogPin);
    //Serial.println(soil_moisture_value);
    soil_moisture_value=(767 - (soil_moisture_value - 300))/7.67;
    
    if(soil_moisture_value > 60){
      digitalWrite(relay_pin,LOW);
    }
    else if(soil_moisture_value < 30){
      digitalWrite(relay_pin,HIGH);
    }
    Serial.println(soil_moisture_value);
    delay(500);
}

void Transmit_data(float h ,float t ,float sh,float light,String API_Path){
    int err = 0;
    WiFiClient c;
    HttpClient http(c);
    String url_s =API_Path + "&field1=" + (int)t + "&field2=" + (int)h + "&field3=" + (int)sh + "&field4=" + (int)light;
    char url[128];
    url_s.toCharArray(url,url_s.length() + 1);
    err = http.get(kHostname, url);
    if (err == 0) {
        Serial.println("startedRequest ok");
        err = http.responseStatusCode();
        if (err >= 0) {
            Serial.print("Got status code: ");
            Serial.println(err);
            err = http.skipResponseHeaders();
            if (err >= 0) {
                int bodyLen = http.contentLength();
                Serial.print("Content length is: ");
                Serial.println(bodyLen);
                Serial.println();
                Serial.println("Body returned follows:");
                unsigned long timeoutStart = millis();
                char c;
                while ((http.connected() || http.available()) && ((millis() - timeoutStart) < kNetworkTimeout)) {
                    if (http.available()) {
                        c = http.read();
                        Serial.print(c);
                        bodyLen--;
                        timeoutStart = millis();
                    } /*else {
                        delay(kNetworkDelay);
                    }*/
                }
                Serial.println("");
            }
        }
    }
    http.stop();
}

void printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}
