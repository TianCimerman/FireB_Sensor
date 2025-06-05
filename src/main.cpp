#include <Arduino.h>
#include "Adafruit_SHT4x.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

#define uS_TO_S_FACTOR 1000000ULL   // Conversion factor from microseconds to seconds
#define TIME_TO_SLEEP  60             // Time for ESP32-E to enter deep sleep
RTC_DATA_ATTR int bootCount = 0; 
int wifiCount = 0;

int ledPin = D9;   
// WiFi credentials
const char* ssid = **********";
const char* password = "*********";

const char* udpAddress = "192.168.1.160";  // Replace with your Telegraf server's IP
const int udpPort = 8094;                  // Port Telegraf is listening on

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
// Create a UDP instance
WiFiUDP udp;

#define VBAT_PIN 34  // VBAT is connected to GPIO35 on FireBeetle ESP32
#define VOLTAGE_DIVIDER_RATIO 2.0  // FireBeetle has a built-in 1:2 voltage divider
#define ADC_MAX 4095.0  // ESP32 ADC resolution (12-bit)
#define REF_VOLTAGE 3.3  // FireBeetle ADC reference voltage

void setup(){
  wifiCount = 0;
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);// Set ledPin as output mode 
  Serial.begin(115200);    
  delay(1000);
    Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sensors_event_t humidity, temp;
  
  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  timestamp = millis() - timestamp; 
  ++bootCount;             
  Serial.println("Boot number: " + String(bootCount));   
    // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED && wifiCount<10) {
    delay(1000);
    Serial.print(".");
    wifiCount++;
  }
  Serial.println("\nConnected to WiFi");
  // Begin UDP
  udp.begin(udpPort);
  Serial.println("UDP started");

  int rawADC = analogRead(VBAT_PIN);
  float voltage = (rawADC / ADC_MAX) * REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;

  String influxData = "climate_2,sensor=arduino temperature_in=" + String(temp.temperature) + ",humidity_in=" + String(humidity.relative_humidity)+ ",voltage_in=" + String(voltage);

  // Send the data via UDP
  udp.beginPacket(udpAddress, udpPort);
  udp.print(influxData);
  udp.endPacket();

  Serial.println("Data sent: " + influxData);


  delay(1000);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);   
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println("Going to sleep now");  // We have set the wake up reason. Now we can start go to sleep of the peripherals need to be in deep sleep. If no wake-up source is provided, but deep sleep is initiated, it will sleep forever unless a hardware reset occurs.
  Serial.flush();
  digitalWrite(ledPin, LOW); 
  esp_deep_sleep_start();   
  Serial.println("This will never be printed");  
}

void loop(){
}
