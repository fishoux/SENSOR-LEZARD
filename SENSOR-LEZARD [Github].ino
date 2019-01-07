#include <Arduino.h>
#include <math.h>
#include <TimeLib.h>
#include <ArduinoJson.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

/*#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
*/

#define PIN_DZero D4    // LCD PIN D0
#define PIN_DOne D3     // LCD PIN D1
#define PIN_RESET D2    // LCD PIN RST
#define PIN_DC D1       // LCD PIN DC

#define PIN_TEMP D5     // TEMP PIN
#define PIN_BtnMoins D6 // Button -
#define PIN_BtnPlus D7  // Button +
#define PIN_RELAY D8    // RELAY PIN
#define PIN_UV A0       // UV ANALOG PIN

// Temperature sensor
#include <Wire.h>
#include <DallasTemperature.h>
OneWire oneWire(PIN_TEMP); // Setup a oneWire instance to communicate with any OneWire devices  
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 

// LCP sensor
#include <U8g2lib.h>
#include <U8x8lib.h>
//U8G2_SSD1306_128X64_NONAME_2_SW_I2C u8g2 (U8G2_R0, D1, D2);
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, PIN_DZero, PIN_DOne, U8X8_PIN_NONE, PIN_DC, PIN_RESET);

ESP8266WiFiMulti WiFiMulti;

/*-----SENSOR VALUES-----*/
float sensorUVvolt ;
float sensorUV ;
float sensorTemp0 ;
float sensorTemp1 ;
String macAddress = "";
String txtWebService = "";
String txtTime ;
String txtSunRise;
String txtSunSet ;
int utcTime ;
int utcSunRise;
int utcSunSet ;
long timeLast = -55000;
bool isInternetConnected = 0;
bool isLampOn = 0;
boolean btnPlus;
boolean btnMoins;

time_t timeNow = now();
int minuteSunRise = 0;
int minuteSunSet = 0;

bool previousBlink = 0;
int menu = 0;
bool freezeMenu = 0;
unsigned int paramMinuteTimeNow; 
unsigned int paramMinuteSunRise;
unsigned int paramMinuteSunSet; 
int paramLampMode = 0; // AUTO

void setup() {
  Serial.begin(9600);
    
  digitalWrite(PIN_RELAY, LOW); 
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW); 
  
  pinMode(PIN_BtnMoins, INPUT_PULLUP);
  pinMode(PIN_BtnPlus, INPUT_PULLUP);
  
  txtTime = "1990-01-01 00:00:00";
  setTime(txtTime.substring(11, 13).toInt(), txtTime.substring(14, 16).toInt(), txtTime.substring(17, 19).toInt(), txtTime.substring(8, 10).toInt(), txtTime.substring(5, 7).toInt(), txtTime.substring(0, 4).toInt()); // Another way to set
 
  Serial.setDebugOutput(true);
  
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
 
  sensors.begin();
  
  u8g2.begin();
  
  u8g2.clear();
  
  delay(2000);
  

  WiFi.mode(WIFI_STA);
  
  WiFiMulti.addAP("USER", "PASSWORD");
  macAddress = WiFi.macAddress();
  
}

void GetAllSensorValues() {
  
  sensorUVvolt = 3.3 * analogRead(PIN_UV) / 1024; ;  //*100/1024;
    sensorUV = mapfloat(sensorUVvolt, 1.09, 2.9, 0.0, 15.0);
    if (sensorUV <= 0 ) sensorUV =0;
     
  //int sensorTempValue = analogRead(D1);  //*100/1024;
  sensors.requestTemperatures(); // Send the command to get temperature readings 
    sensorTemp0 = sensors.getTempCByIndex(0);
    sensorTemp1 = sensors.getTempCByIndex(1);

    if (sensorTemp0 <= 0 ) sensorTemp0 =0;
    if (sensorTemp1 <= 0 ) sensorTemp1 =0;
    
}


void ActuorAll() {
  if (paramLampMode == 3) {
    digitalWrite(PIN_RELAY, HIGH);
  } else if (paramLampMode == 2) {
    digitalWrite(PIN_RELAY, LOW);
  } else {
    if ( minuteSunSet != minuteSunRise && minuteSunSet != 0 && minuteSunSet > minuteSunRise && year(timeNow) != 1982 && hour(timeNow) + minute(timeNow) !=0 ) {
      if (  minuteSunRise + paramMinuteSunRise <= 60 * hour(timeNow) + minute(timeNow) + paramMinuteTimeNow && 60 * hour(timeNow) + minute(timeNow) + paramMinuteTimeNow <= minuteSunSet + paramMinuteSunSet  ) {
        isLampOn = 1;
        digitalWrite(PIN_RELAY, HIGH);
      } else {
        isLampOn = 0;
        digitalWrite(PIN_RELAY, LOW);
      }
    }
  }
}


//The Arduino Map function but for floats
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
 
  btnPlus = !digitalRead(PIN_BtnPlus);
  btnMoins = !digitalRead(PIN_BtnMoins);
  
  if ( !freezeMenu && btnMoins==HIGH && btnPlus==HIGH ) {
      if (menu >= 6) {
        menu = 0;
        freezeMenu = 1;
      } else {
        menu++;
        freezeMenu = 1;
      }
  }
  
  if ( btnMoins==LOW || btnPlus==LOW) {
    freezeMenu = 0;
  }
  
  if (menu != 0) {
    previousBlink = !previousBlink;
    
    if (menu == 2 && btnMoins==HIGH) { paramMinuteTimeNow=paramMinuteTimeNow-10; }
    if (menu == 2 && btnPlus==HIGH) { paramMinuteTimeNow=paramMinuteTimeNow+10; }
    if (menu == 3 && btnMoins==HIGH) { paramLampMode--; }
    if (menu == 3 && btnPlus==HIGH) { paramLampMode++; }
    if (menu == 5 && btnMoins==HIGH) { paramMinuteSunRise=paramMinuteSunRise-10; }
    if (menu == 5 && btnPlus==HIGH) { paramMinuteSunRise=paramMinuteSunRise+10; }
    if (menu == 6 && btnMoins==HIGH) { paramMinuteSunSet=paramMinuteSunSet-10; }
    if (menu == 6 && btnPlus==HIGH) { paramMinuteSunSet=paramMinuteSunSet+10; }
  } else {
    previousBlink = 0;
  }

  if (paramLampMode>3) {paramLampMode=0;}
  if (paramLampMode<0) {paramLampMode=3;}
  
  GetAllSensorValues();
  
  if ( millis()-timeLast >= 60000 ) { 
    
    WebServiceCall("UV", String(sensorUV), "Temp0", String(sensorTemp0), "Temp1", String(sensorTemp1) );
    
    timeLast = millis();
    
  }
  
  ActuorAll();
  
  DisplayLCD();
  
  delay(500);
}




void DisplayLCD(){
  String line1 = "";
  String line2 = "";
  String line3 = "";
  
  float txtTemp0 = round(sensorTemp0*10);
  txtTemp0 = txtTemp0/10;
  float txtTemp1 = round(sensorTemp1*10);    
  txtTemp1 = txtTemp1 /10;
  
  Serial.print(" | btnMoins:");
  Serial.print(btnMoins);
  Serial.print(" | btnPlus:");
  Serial.print(btnPlus);
  Serial.print(" | minute timeNow:");
  Serial.print(minute(timeNow));
  Serial.print(" | A0:");
  Serial.print(sensorUVvolt);
  Serial.print("V ");
  Serial.print(sensorUV);
  Serial.print(" mW/cm² | temp0:");
  Serial.print(sensorTemp0);
  Serial.print("°C | temp1 ");
  Serial.print(sensorTemp1);
  Serial.println("°C");
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB12_tf);
  
  /* ------- LINE 1 ------- */
  if (menu == 1 && previousBlink) line1 = macAddress;  
  if (menu != 1) line1 = String(txtTemp0).substring(0, 4) + " | " + String(txtTemp1).substring(0, 4) + " | " + String(sensorUV);

  /* ------- LINE 2 ------- */
  
  String dsplTime = MinuteToTime(60*hour(timeNow)+minute(timeNow)+paramMinuteTimeNow);
  if (menu == 2 && previousBlink) dsplTime = "__:__";  
  
  String txtLamp = GetLampMode();
  if (menu == 3 && previousBlink) txtLamp = "____";  

  String txtInternet = "X";
  if (isInternetConnected) txtInternet = "@";
  if (menu == 4 && previousBlink) txtInternet  = "_";  
  
    line2 = dsplTime+ "  " +txtLamp+ "  " +txtInternet;  
  
  /* ------- LINE 3 ------- */
  String dsplSunRise = MinuteToTime(minuteSunRise+paramMinuteSunRise);
  if (menu == 5 && previousBlink) dsplSunRise  = "__:__";  
 
  String dsplSunSet = MinuteToTime(minuteSunSet+paramMinuteSunSet);
  if (menu == 6 && previousBlink) dsplSunSet  = "__:__";  
 
    line3 = "  " +dsplSunRise+ " > " +dsplSunSet; 
 
  u8g2.drawStr(0,12, line1.c_str() );
  u8g2.drawStr(0,35, line2.c_str() );
  u8g2.drawStr(0,55, line3.c_str() );
  
  u8g2.sendBuffer();
  
}

String GetLampMode(){
  if (paramLampMode == 3) {return "ON";}
  else if (paramLampMode == 2) {return "OFF";}
  else {return "AUTO";}
  
}

String MinuteToTime(int min) {
  int resutHour = min/60;
  int resutMin = min - resutHour*60;
  String resultTime = String(resutHour);
  
  if (resutHour < 10) {
    resultTime = "0" + String(resutHour);
  }
  if (resutMin < 10 ) {
    resultTime = resultTime + ":0" + String(resutMin);
  } else {
    resultTime = resultTime + ":" + String(resutMin);
  }
  
  return resultTime;
  
}


int WebServiceCall(String sensor1Id , String sensor1Value, String sensor2Id , String sensor2Value, String sensor3Id , String sensor3Value) {
  String url = "http://WEBSERVICEADRESS:10010/save?mac=" + macAddress + "&sensor1id=" + sensor1Id + "&sensor1value=" + sensor1Value + "&sensor1id=" + sensor2Id + "&sensor2value=" + sensor2Value + "&sensor3id=" + sensor3Id + "&sensor3value=" + sensor3Value;
  Serial.println("[HTTP] url is:" + url);
  
  // wait for WiFi connection
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    isInternetConnected = 1;
    WiFiClient client;

    HTTPClient http;

    Serial.print("[HTTP] begin...\n");
      if (http.begin(client, url)) {  // HTTP
      
      Serial.print("[HTTP] GET...\n");
      // start connection and send HTTP header
      int httpCode = http.GET();
      
      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);

        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = http.getString();
          Serial.print("JSON:" + payload);
          txtWebService= payload;     
          StaticJsonDocument<200> doc;
          DeserializationError error = deserializeJson(doc, payload);
          
          if (error) {
            Serial.print("[HTTP] JSON error ");
            Serial.println(error.c_str());
            //return;
          }
          
          JsonObject root = doc.as<JsonObject>();
          txtTime = root["time"].as<String>();
          txtSunRise = root["sunRise"].as<String>();
          txtSunSet = root["sunSet"].as<String>();
          minuteSunRise = 60 * txtSunRise.substring(11, 13).toInt() + txtSunRise.substring(14, 16).toInt();
          minuteSunSet = 60 * txtSunSet.substring(11, 13).toInt() + txtSunSet.substring(14, 16).toInt();
          
          setTime(txtTime.substring(11, 13).toInt(), txtTime.substring(14, 16).toInt(), txtTime.substring(17, 19).toInt(), txtTime.substring(8, 10).toInt(), txtTime.substring(5, 7).toInt(), txtTime.substring(0, 4).toInt()); // Another way to set

        }
      } else {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }

      http.end();
    } else {
      Serial.printf("[HTTP} Unable to connect\n");
      //isInternetConnected = 0;
    }
  } else {
    isInternetConnected = 0;
  }
  

}
