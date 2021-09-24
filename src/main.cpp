#include "Arduino.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include "ArduinoNvs.h"
#include <HTTPClient.h>
#include <NTPClient.h>


extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

///////OUTPUT PINS///////
#define STATUS_LED 17
#define WIND_LED 19
#define BLE_LED 22
#define WIFI_LED 21

///////INPUT PINS///////
#define HALL_1 14
#define HALL_2 16
#define HALL_3 23
#define HALL_4 25
#define BUTON_PIN 26

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

#define DEVICENAME "WIND SPEED LOGGER"

bool deviceConnected = false;
bool isConfigured = false;


#define WIFI_SSID  "HUAWEI-5GCPE-025C"
#define WIFI_PASSWORD "YBABM6ARH3A"

float revs;
float rpm;
volatile byte rpmcount;
long previousmicros = 0;
long interval = 1000000;

int val = 0;
int buttonCounter = 0;

int RPM1;
int RPM2;
int RPM3;
int RPM4;

float voltage = 0.0; 

volatile unsigned int count;

void count_pulse()
{
    count++;
}

bool setSSID = false;
bool setPassword = false;
bool setIP = false;
bool isWifi = false;


TimerHandle_t wifiReconnectTimer;

HTTPClient http;
char* googleSheetsUrl = "https://script.google.com/macros/s/AKfycbzoY6EZSLTuvokWxhLG-wF9NjN6SUtzVTYwRAObzdrnVG5BOv0JMNj9ebc5ASk0u_WC/exec?";
char uploadBuffer[255];

void ICACHE_RAM_ATTR sens1() {
  Serial.println("HAll1");
  RPM1++;
}

void ICACHE_RAM_ATTR sens2() {
  Serial.println("HAll2");
  RPM2++;
}

void ICACHE_RAM_ATTR sens3() {
  // Serial.println("HAll3");
  RPM3++;
}

void ICACHE_RAM_ATTR sens4() {
  Serial.println("HAll4");
  RPM4++;
}
 


void connectToWifi()
{
  Serial.println("Connecting to wifi...");
  // String ssid = NVS.getString("wifiSSID");
  // String password = NVS.getString("wifiPASS");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    digitalWrite(WIFI_LED, HIGH);
    Serial.println("Connected to Wifi!");
    isWifi = true;
    timeClient.begin();
    timeClient.setTimeOffset(7200);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    digitalWrite(WIFI_LED, LOW);
    isWifi = false;
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}
 

String strToString(std::string str)
{
  return str.c_str();
}
int strToInt(std::string str)
{
  const char *encoded = str.c_str();
  return 256 * int(encoded[1]) + int(encoded[0]);
}
double intToDouble(int value, double max)
{
  return (1.0 * value) / max;
}
bool intToBool(int value)
{
  if (value == 0)
  {
    return false;
  }
  return true;
}


void buttonChecker(void *pvParameters) // This is a task.
{
  (void)pvParameters;
  for (;;)
  {
    val = digitalRead(BUTON_PIN);

    if (val == LOW)
    {
      if (buttonCounter == 30)
      {
        ESP.restart();
      }
      else
      {
        buttonCounter++;
        Serial.println(buttonCounter);
      }
    }
    else
    {
      buttonCounter = 0;
    }
    vTaskDelay(100);
  }
}

void speedLoggerTask(void *pvParameters) // This is a task.
{
  int previousState = 0;
  (void)pvParameters;
  for (;;)
  {
    if(isWifi)
    {
   while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  int count = 0;
    voltage = ((float)analogRead(34) / 2048.0)* 3.3;
    Serial.println(voltage);
    for(int i = 0; i < 1000000; i++)
    {
      int hallVal = digitalRead(HALL_3);
       if(hallVal == 0 && previousState == 1)
      {
        count++;
      }
       previousState = hallVal;
      delayMicroseconds(1);
    }
    Serial.println(count);
    vTaskDelay(100);
    unsigned long formattedDate = timeClient.getEpochTime();
    HTTPClient http;
      char* googleSheetsUrl = "https://script.google.com/macros/s/AKfycbxYkUsh8lP6tItzVK_Og-O8Fdcs-2ESjZn8FsViL0BdbegSvWRCJ0_WUi34Yp5Xz9I3-Q/exec?";
      char uploadBuffer[255];
      sprintf(uploadBuffer, "%sid=Readings&Timestamp=%lu&WindSpeed=%d&BatteryVoltage=%.6f",googleSheetsUrl,formattedDate, count, voltage);
      Serial.println(uploadBuffer);
      http.begin(uploadBuffer);
      int httpResponseCode = http.GET();
      if (httpResponseCode>0) {
        String payload = http.getString();
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      http.end(); 
    count = 0;
    }
  }
}




void setup()
{
  Serial.begin(115200);
  Serial.print("Device Name:");
  Serial.println(DEVICENAME);
  ////INIT INPUT PINS////
  pinMode(HALL_1, INPUT);
  pinMode(HALL_2, INPUT);
  pinMode(HALL_3, INPUT);
  pinMode(HALL_4, INPUT);
  pinMode(BUTON_PIN, INPUT);
  ////INIT OUTPUT PINS////
  pinMode(BLE_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(WIND_LED, OUTPUT);

  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(WIND_LED, LOW);
  digitalWrite(WIFI_LED, LOW);
  digitalWrite(BLE_LED, LOW);

  pinMode(HALL_1, INPUT); 
  pinMode(HALL_2, INPUT); 
  pinMode(HALL_3, INPUT); 
  pinMode(HALL_4, INPUT); 

  NVS.begin();
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  connectToWifi();

  xTaskCreate(
      buttonChecker, "buttonChecker", 1024 * 2 // Stack size
      ,
      NULL, (configMAX_PRIORITIES - 1) // Priority
      ,
      NULL);
      xTaskCreatePinnedToCore(
      speedLoggerTask, "speedLoggerTask", 1024 * 10 // Stack size
      ,
      NULL, (configMAX_PRIORITIES) // Priority
      ,
      NULL, 1);
}

void loop()
{
}
