#include <HttpClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

char ssid[] = "SSID"; // your network SSID (name)
char pass[] = "PASSWORD"; // your network password (use for WPA, or use as key for WEP)

const char kHostname[] = "api.thingspeak.com";

String kPath = "/update?api_key=key";
const int kNetworkTimeout = 10 * 100;
const int kNetworkDelay = 500;
int status = WL_IDLE_STATUS;

#include "DHT.h"
#define DHT_Pin 2
#define DHT_TYPE DHT11
DHT dht(DHT_Pin, DHT_TYPE);

float h=0;
float t=0;

#include <SoftwareSerial.h>
#if defined(BOARD_RTL8720DN_BW16)
    SoftwareSerial mySerial(PB2, PB1); // RX, TX
#else
    SoftwareSerial mySerial(0, 1); // RX, TX
#endif
#define pmsDataLen 32
uint8_t buf[pmsDataLen];
int idx = 0;
int pm10 = 0;
int pm25 = 0;
int pm100 = 0;

int post_time = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("System begin!"));
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        // wait 10 seconds for connection:
        delay(10000);
    }
    Serial.println("Connected to wifi");
    printWifiStatus();

    dht.begin();
    mySerial.begin(9600); // PMS 3003 UART has baud rate 9600
}

void loop() { // run over and over
    uint8_t c = 0;
    idx = 0;
    memset(buf, 0, pmsDataLen);

    while (true) {
        while (c != 0x42) {
            while (!mySerial.available());
            c = mySerial.read();
        }
        while (!mySerial.available());
        c = mySerial.read();
        if (c == 0x4d) {
            // now we got a correct header)
            buf[idx++] = 0x42;
            buf[idx++] = 0x4d;
            break;
        }
    }
    while (idx != pmsDataLen) {
        while(!mySerial.available());
        buf[idx++] = mySerial.read();
    }
    pm10 = (buf[10] << 8) | buf[11];
    pm25 = (buf[12] << 8) | buf[13];
    pm100 = (buf[14] << 8) | buf[15];

    Serial.print("pm1.0: ");
    Serial.print(pm10);
    Serial.println(" ug/m3");
    Serial.print("pm2.5: ");
    Serial.print(pm25);
    Serial.println(" ug/m3");
    Serial.print("pm10: ");
    Serial.print(pm100);
    Serial.println(" ug/m3");
    if(millis() - post_time > 15500){
      h = dht.readHumidity();
      t = dht.readTemperature();
      if (isnan(h) || isnan(t)) {
          Serial.println(F("Failed to read from DHT sensor!"));
          return;
      }
      Transmit_data(t,h,pm10,pm25,pm100,kPath);
      post_time = millis();
    }
    
    
}
void Transmit_data(float t ,float h ,int pm10,int pm25,int pm100,String API_Path){
    int err = 0;
    WiFiClient c;
    HttpClient http(c);
    String url_s =API_Path + "&field1=" + (int)t + "&field2=" + (int)h + "&field3=" + pm10 + "&field4=" + pm25 + "&field5=" + pm100;
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
                    } else {
                        delay(kNetworkDelay);
                    }
                }
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
