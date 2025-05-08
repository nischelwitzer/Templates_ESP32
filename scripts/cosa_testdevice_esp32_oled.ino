/*
  LUFT AIR MQTT
  Exhibit /habitat/air
  (Luft)

  Chip is ESP32-D0WD-V3 (revision v3.1)
  Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
  Crystal is 40MHz
  MAC: f4:65:0b:e7:d5:64

  changed Library from ADS1015 to ADS1X15
  last update: 01.05.2025 nis

  Sensor: https://github.com/adafruit/Adafruit_ADS1X15
  https://www.adafruit.com/product/1083 
  ADS1015 12-Bit ADC - 4 Channel with Programmable Gain Amplifier 
 
  based on
  https://github.com/nischelwitzer/IOT-Master/blob/main/iot_master/iot_master.ino

  Multiple libraries were found for "WiFi.h"
  Used: C:\Users\nisch\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\WiFi
  Not used: D:\03_dev\02_arduino\libraries\WiFiNINA
  Not used: D:\03_dev\02_arduino\libraries\Seeed_Arduino_rpcWiFi
  Not used: D:\03_dev\02_arduino\libraries\WiFi
*/

#include <SPI.h>
#include <U8g2lib.h>
#include "WiFi.h"
#include <WiFiMulti.h>
#include "wlan_secrets.h"
 
#include <PubSubClient.h>
// #define MQTT_HABITAT
#define MQTT_DMT
#include "mqtt_secrets.h"

WiFiMulti wifiMulti;
WiFiClient wifi_client;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // Adress 0x03c

//////////////////////
///// VARIABLES /////
////////////////////

#define ESP_INFO_VERSION "ESP32 Testdevice 1.0"
#define EXHIBIT_NAME "Exhibit /habitat/testdevice"
byte mac[] = DEVICE_MAC;

#define OLEDLENGTH 30
char timeString[OLEDLENGTH] = "Time: init";
char oledInfo[OLEDLENGTH]   = "OLED init";

///// Timing for Alive checks /////

#define seconds() (millis()/1000)
#define minutes() (millis()/60000)
#define hours()   (millis()/3600000)

int oldSec = 99; // check if sec has changed
int myOldSecPuls = 99; 
String mySSID = "no SSID";

/////////////////
///// PINS /////
///////////////

// living button/led
const int ledCtrlPin    = 12;  // connector on PIN (normal D2) yellow     13=GPIO12 OUTPUT
const int buttonCtrlPin = 14;  // button    on PIN (normal D3) white      12=GPIO14  INPUT
bool buttonCtrlState = false;
bool buttonCtrlPrev  = false;

#define ONBAORD_LED 2

// I2C Inter-Integrated Circuit 
// GPIO21 P21 I2C SDA weiss 
// GPIO22 P22 I2C SCL gelb
// GROVE: gelb weiss +/VCC -/GND 

///// Network & MQTT /////

char* MQTT_TOPIC_OUT_XCOCOS_ALIVE = "xcocos/alive/";
char* MQTT_TOPIC_OUT_TESTBUTTON   = "esp/testbutton";
char* MQTT_TOPIC_OUT_DATA         = "cosa/habitat/test/out/data"; 

char* MQTT_TOPIC_IN_DEBUGLEVEL   = "cosa/habitat/test/debuglevel"; 
char* MQTT_TOPIC_IN_RESET        = "cosa/habitat/test/reset"; 
 
// --------------------------------------------------------------------------------

int button_counter = 0;
int mqtt_counter = 0;
int alive_counter = 0;

char charSend[80];  // char helper for sending infos - 79 Zeichen (+1 für \0)
int debugLevel = 0; // OFF=0 INFO=1, WARN=2, ERROR=3

// ##### init ######################################################################

//////////////////////
///// VARIABLES /////
////////////////////

// IPAddress ip(DEVICE_IP_ADRESS);
PubSubClient mqtt_client(wifi_client);

/////////////////
///// MAIN /////
///////////////

//////////////////
///// SETUP /////
////////////////

void setup() 
{
  setup_serial();
  Serial.print(EXHIBIT_NAME);
  Serial.println(": starting setup ------------------------------------------------------------");

  setup_pins();
  setup_wlan();
  setup_oled(); // and show wlan infos
  setup_mqtt(); 
  reset_sensors();
  
  Serial.print(EXHIBIT_NAME);
  Serial.println(": setup complete ------------------------------------------------------------");
  snprintf(oledInfo, OLEDLENGTH,"%s", "Setup completed."); 
}

/////////////////
///// LOOP /////
///////////////
// MAIN LOOP

void loop() 
{
  handle_connection();
  check_inputs();    // check inputs
  check_testbutton(); // check button
  check_alive();      // MQTT alive
  check_timer();      // pulsing
  showScreen();
  delay(50);
}

/////////////////////////
///// SETUP Helper /////
///////////////////////

void setup_serial() 
{
  Serial.begin(115200);
  while (!Serial) { }; // wait for serial port to connect. Needed for native USB port only 
  delay(2000);
  Serial.print(EXHIBIT_NAME);
  Serial.println(": starting... serial completed and OK.");
}

void setup_pins() {
  Serial.print(EXHIBIT_NAME);
  Serial.println(": setup pins...");

  pinMode(buttonCtrlPin, INPUT);  // INPUT_PULLUP
  pinMode(ledCtrlPin,    OUTPUT);
  pinMode(ONBAORD_LED,   OUTPUT);

  Serial.print(EXHIBIT_NAME);
  Serial.println(": setup pins completed.");
}

void setup_oled()
{
  Serial.print(EXHIBIT_NAME);
  Serial.println(": Setting up Display u8g2..."); 
  u8g2.begin();  // OLED 128 x 64
  u8g2.setDisplayRotation(U8G2_R2);
  showScreen();
  Serial.print(EXHIBIT_NAME);
  Serial.println(": Setting up Display u8g2 completed."); 
}

void setup_wlan()
{
  delay(10);  
  Serial.print(EXHIBIT_NAME);
  Serial.println(": Setting up WLAN..."); 

  // We start by connecting to a WiFi network
  
  wifiMulti.addAP(SECRET_SSID1, SECRET_PASS1);
  wifiMulti.addAP(SECRET_SSID2, SECRET_PASS2);
  wifiMulti.addAP(SECRET_SSID3, SECRET_PASS3);
  wifiMulti.addAP(SECRET_SSID4, SECRET_PASS4);
  wifiMulti.addAP(SECRET_SSID5, SECRET_PASS5);

  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("ESP32 MultiWiFi connected");
    Serial.print("🔹 IP address: ");
    Serial.println(WiFi.localIP());
  }

  while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
  }

  printWifiData();
  printCurrentNet();
  
  Serial.print(EXHIBIT_NAME);
  Serial.println(": Setting up WLAN completed."); 
  Serial.println("");
}


void setup_mqtt() {
  Serial.print(EXHIBIT_NAME);
  Serial.println(": Setting up  mqtt...");

  mqtt_client.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt_client.setCallback(handle_received_message);
  mqtt_login();

  Serial.print(EXHIBIT_NAME);
  Serial.println(": Setting up mqtt completed.");
}

void reset_sensors()
{
  mqtt_publish(MQTT_TOPIC_OUT_DATA, "0");
  button_counter = 0;
  // mqtt_counter = 0;
  // alive_counter = 0;
  Serial.print(EXHIBIT_NAME);
  Serial.println(": MQTT Reset completed.");
  snprintf(oledInfo, sizeof(oledInfo),"%s", "MQTT Reset done."); 
}

// WIFI Helper

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("🔹 SSID: ");
  mySSID = WiFi.SSID().c_str();
  Serial.println(mySSID);
  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid); // Basic Service Set Identifier - MAC des access points
  Serial.print("🔹 BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("🔹 Signalstärke (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void printWifiData() 
{
  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("🔹 MAC address: ");
  printMacAddress(mac);
  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("🔹 NetMask: ");
  Serial.println(subnet);

  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("🔹 Gateway: ");
  Serial.println(gateway);

  Serial.print("🔹 DNS: ");
  Serial.println(WiFi.dnsIP());
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

// ################################################################################

/****************************************
 * Auxiliar Functions for OLED
 ****************************************/

void showScreen()
{
  char charRow2[OLEDLENGTH];
  char charRow3[OLEDLENGTH];
  char charRow4[OLEDLENGTH];
  char charRow5[OLEDLENGTH];
  char charRow6[OLEDLENGTH];

  snprintf(charRow2, OLEDLENGTH, "SSID: %s ", String(mySSID)); // WiFi.SSID().c_str()
  IPAddress ip;  
  ip = WiFi.localIP();
  snprintf(charRow3, OLEDLENGTH, "IP: %s",ip.toString().c_str()); 
  snprintf(charRow4, OLEDLENGTH, "%s B:%04d",timeString, button_counter); 
  snprintf(charRow5, OLEDLENGTH, "MQTTsend A:%03d C:%04d", alive_counter, mqtt_counter); 
  snprintf(charRow6, OLEDLENGTH, "%s", oledInfo); 
  // snprintf(charRow6, OLEDLENGTH, "012345678901234567890"); 

  // u8g2.clearDisplay();
  u8g2.setDisplayRotation(U8G2_R0);
  u8g2.firstPage();
  do {
    // u8g2.setFont(u8g2_font_ncenB08_tr);
    // u8g2.setFont(u8g2_font_t0_16b_mr);
    u8g2.setFont(u8g2_font_t0_11_mr);   // choose a suitable font https://github.com/olikraus/u8g2/wiki/fntlist8
    //                 012345678901234567890
    u8g2.drawStr(0, 8,ESP_INFO_VERSION);    
    u8g2.drawStr(0,18,charRow2);   
    u8g2.drawStr(0,28,charRow3); 
    u8g2.drawHLine(0,30, 128);
    u8g2.drawStr(0,41,charRow4);    
    u8g2.drawStr(0,51,charRow5);    
    u8g2.drawStr(0,61,charRow6);    
    u8g2.drawHLine(0,63, 128);
  } while (u8g2.nextPage());
}

void showOLED(int x, int y, String str)
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_16b_mr);
    u8g2.setCursor(x, y);
    u8g2.print(str);
  } while (u8g2.nextPage());
}

// ################################################################################

////////////////////////
///// LOOP HELPER /////
//////////////////////

void handle_connection() {
  if (!mqtt_client.connected()) {
    mqtt_login();
  }
  mqtt_client.loop();
}

void check_inputs() {
  //
  // INPUT handling
  //
}

void check_testbutton()
{
  buttonCtrlState = digitalRead(buttonCtrlPin);
  if (buttonCtrlState != buttonCtrlPrev) 
  {
    snprintf(charSend, sizeof(charSend), "Button: S%1d",buttonCtrlState); 
    Serial.println(charSend); // send to pc/unity
    buttonCtrlPrev = buttonCtrlState;

    if (buttonCtrlState == 1)
    {
      button_counter++;
      snprintf(charSend, sizeof(charSend), "Button Counter: %04d",button_counter); 
      Serial.println(charSend);

      snprintf(charSend, sizeof(charSend), "%s %05d",ALIVE_ID, button_counter); 
      mqtt_publish(MQTT_TOPIC_OUT_TESTBUTTON, charSend);
      snprintf(charSend, sizeof(charSend), " button %s %05d",ALIVE_ID, button_counter); 
      mqtt_publish(MQTT_TOPIC_OUT_DATA, charSend);
    }
  }
}

void check_alive() {
  int mySec = seconds() % 60;
  if (mySec != oldSec)  // only when sec changed
  {
    oldSec = mySec;
    if ((mySec%60) == 0)  // alle 60 sec
    {
      alive_counter++;
      snprintf(charSend, sizeof(charSend), "%s %05d", ALIVE_ID, alive_counter); 
      mqtt_publish(MQTT_TOPIC_OUT_XCOCOS_ALIVE, charSend); // alive Message to XCOCOS
    }
  }
}

void check_timer()
{
  // ###################################################
  // helper functions - time 
  int mySec = seconds() % 60;
  int mySecPuls = seconds() % 2;
  // mylcd.setCursor(15, 1);
  int myMin = minutes() % 60;
  int myHou = hours(); 

    snprintf(timeString, sizeof(timeString), "Time: %02d:%02d.%02d",myHou,myMin,mySec); 

    if (mySecPuls != myOldSecPuls)
    {
      if (mySecPuls == 1)
      {
        // mylcd.print("x");
        digitalWrite(ledCtrlPin, true);
        digitalWrite(ONBAORD_LED, false);
      }
      else
      {
        // mylcd.print("o");  
        digitalWrite(ledCtrlPin, false);
        digitalWrite(ONBAORD_LED, true);
      }
      myOldSecPuls = mySecPuls;
    }
}

/////////////////
///// MQTT /////
///////////////

void handle_received_message(char* topic, byte* payload, unsigned int length) 
{
  Serial.print(EXHIBIT_NAME);
  Serial.print(": handle Message from topic [");
  Serial.print(topic);
  Serial.print("]: ");

  String payload_string = "";
  for (int i = 0; i < length; i++) {
    payload_string += (char)payload[i];
  }
  Serial.println(payload_string);

  if (strcmp(topic, MQTT_TOPIC_IN_DEBUGLEVEL) == 0) {
     debugLevel = payload_string.toInt();
     Serial.print("DEBUG LEVEL set to: ");
     Serial.println(debugLevel);
     snprintf(oledInfo, OLEDLENGTH,"set DebugLevel: %d", debugLevel); 
  } 
  else if (strcmp(topic, MQTT_TOPIC_IN_RESET) == 0) 
  {
     reset_sensors();
  } 
}

void mqtt_login() 
{
  while (!mqtt_client.connected()) {
    Serial.print(EXHIBIT_NAME);
    Serial.println(": Attempting MQTT connection...");

    if (mqtt_client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS)) {
      Serial.print(EXHIBIT_NAME);
      // MQTT subscribe topics
      mqtt_client.subscribe(MQTT_TOPIC_IN_DEBUGLEVEL);
      mqtt_client.subscribe(MQTT_TOPIC_IN_RESET);
      Serial.println(": connected");
    } else {
      Serial.print(EXHIBIT_NAME);
      Serial.print(": failed, rc=");
      Serial.print(mqtt_client.state()); 
      Serial.print(" to ");
      Serial.print(MQTT_BROKER);
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqtt_publish(char* topic, String payload) 
{
  mqtt_counter++;
  // snprintf(charSend, sizeof(charSend), "%s: Publishing %03d", EXHIBIT_NAME, mqtt_counter); 
  // Serial.println(charSend);
  snprintf(charSend, sizeof(charSend), "##### MQTT message: %s %s ", topic, payload.c_str()); 
  Serial.println(charSend);
  
  mqtt_client.publish(topic, payload.c_str(), true);
}

// #################################################################

///////////////////
///// EVENTS /////
/////////////////


// EOP
