#include <PubSubClient.h> //https://www.arduino.cc/reference/en/libraries/pubsubclient/
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESP32httpUpdate.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include<ModbusMaster.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>

//Modbus Master
char tagdata1[900];
char tagdata2[900];
char tagdata3[900];
String ercode;
ModbusMaster node;
String master_slave, master_baudrate, master_parity;
String master_tagid1, master_tagid2, master_tagid3;
uint16_t result_mastertagid1, result_mastertagid2, result_mastertagid3;
int register1, register2, register3, register4, register5, register6;

//Error Code
int flag = 1, flager;

//delay
int period;
unsigned long time_now = 0;
int heatbeatperiod;
unsigned long heatbeattime_now = 0;

//reset and restart
float pressLength_milliSeconds = 0;

// Define the *minimum* length of time, in milli-seconds, that the button must be pressed for a particular option to occur
int optionOne_milliSeconds = 100;
int optionTwo_milliSeconds = 2000;
int buttonState;


//FOTA version and URL
const int FW_VERSION = 2; // version of this firmware
const char* fwUrlBase = "https://"; // firmware server urlhttps://demo.isenzr.com

//MQTT Settings
String topicin, topicout, macadd;
char topic_in[50], topic_out[50], cid[50], broker[50], portadd[50], wifissid[50], wifipswd[50], ptp1char[50], ptp2char[50], ptp3char[50], pterc[50];
char mqtt_name[50], mqtt_pwd[50];
WiFiClient espclient;
PubSubClient client(espclient);
long lastMsg = 0;
int value = 0;
String textin, essid, epaswd, epssid, eppswd;
bool retain = false;
int connectflag = 1;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

//Variables
int i = 0;
bool rc;
int statusCode;
const char* ssid = "text";
const char* passphrase = "text";
String st;
String content;
int rec_count = 0;
// Global variable
String firstValue, sssid, secondValue, pswd, thirdvalue, brokeradd, fourthvalue, port, fifthvalue, ptp1, sixthvalue, ptp2, seventhvalue, ptp3, eigthvalue, stp1, ninthvalue, ip01;
String tenthvalue, i1ce, eleventhvalue, i1tc, tewelthvalue, i1tw, thirteenthvalue, ip02, fourteenthvalue, i2ce, fifteenthvalue, i2it,  sixteenthvalue, i2ma, seventeenthvalue, i2mi;
String eighteenthvalue, i2at, nineteenthvalue, ip03, twentethvalue, i3ce, twentyonethvalue, i3it, twentytwothvalue, i3mi, twentythreethvalue, i3ma, twentyfourthvalue, i3at;
String twentyfivthvalue, ddur, twentysixthvalue, rest, twentyseventhvalue, unm1, twentyeighthvalue, pwd1;
String condition, threshold1, threshold2, typeno, esid, epass = "";
byte mac[6];

//Function Decalration
bool testWifi();
HTTPClient httpClient;

// Set web server port number to 80
WiFiServer server(80);
//Pin declaration
int resetbutton = 13, wifistatusled = 25, mqttstatusled = 26;;
bool checkforupdates = false;

// RTD sensor
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 rtd = Adafruit_MAX31865(27);

//Thermocouple sensor
#define DRDY_PIN 5
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(4);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

//Digital Flow meter
#define Digital_SENSOR  33
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 1;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
void setup() {
  Serial.begin(115200);
  Serial.println("starting arduino: ");
  Serial.println("setting up Serial ");
  Serial.println("setting up RS485 port ");
  pinMode(resetbutton, INPUT_PULLUP);
  pinMode(mqttstatusled, OUTPUT);
  pinMode(wifistatusled, OUTPUT);
  digitalWrite(mqttstatusled, LOW);
  digitalWrite(wifistatusled, LOW);
  Serial.println("Disconnecting previously connected WiFi");
  WiFi.disconnect();
  EEPROM.begin(512); //Initialasing EEPROM
  delay(10);
  String bdrt;
  for (i = 405; i < 412; ++i)
  {
    bdrt += char(EEPROM.read(i));
  }
  int mbbdrt = bdrt.toInt();
  int mbbdrt_int = mbbdrt;
  if (mbbdrt_int == 0) {
    mbbdrt_int = 9600;
  }
  Serial.print("bdrt: ");
  Serial.println(bdrt);
  Serial.println(bdrt.toInt());
  Serial.print("mbbdrt_int: ");
  Serial.println(mbbdrt_int);
  String senseid;
  for (i = 401; i < 405; ++i)
  {
    senseid += char(EEPROM.read(i));
  }
  int sensorid = senseid.toInt();
  int sensorid_int = sensorid;
  if (sensorid_int == 0) {
    sensorid_int = 1;
  }
  Serial.print("senseid: ");
  Serial.println(senseid);
  Serial.println(senseid.toInt());
  Serial2.begin(mbbdrt_int);
  Serial.print("sensorid_int: ");
  Serial.println(sensorid_int);
  delay(2000);
  node.begin(sensorid_int, Serial2);
  Serial.println();
  Serial.println();
  Serial.println("Startup");
  for (i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);
  String sl = esid.c_str();
  Serial.println(sl.length());
  sl.replace("%20", " ");
  sl.replace("%26", "&");
  Serial.print("SSID: ");
  epssid = sl.c_str();
  Serial.println(epssid);
  delay(1000);
  Serial.println("Reading EEPROM pass");

  for (i = 32; i < 96; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);
  String pl = epass.c_str();
  Serial.println(pl.length());
  pl.replace("%20", " ");
  pl.replace("%26", "&");
  Serial.print("SSID: ");
  eppswd = pl.c_str();
  Serial.println(eppswd);
  Serial.print("esid.c_str(): ");
  essid = epssid.c_str();
  epaswd = eppswd.c_str();
  Serial.println(essid.length());
  if (essid.length() > 0) {
    WiFi.begin(epssid.c_str(), eppswd.c_str());
    if (testWifi()) {
      Serial.println("Succesfully Connected!!!");
      digitalWrite(wifistatusled, HIGH);
      // Initialize a NTPClient to get time
      timeClient.begin();    // GMT +1 = 3600 // GMT +8 = 28800 // GMT -1 = -3600 // GMT 0 = 0
      timeClient.setTimeOffset(0);
      timeClient.setUpdateInterval(1000);// 1 seconds
      for (i = 96; i < 111; ++i)
      {
        brokeradd += char(EEPROM.read(i));
      }
      Serial.print("brokeradd: ");
      Serial.println(brokeradd);
      for (i = 111; i < 116; ++i)
      {
        port += char(EEPROM.read(i));
      }
      Serial.print("port: ");
      Serial.println(port);

      for (i = 116; i < 126; ++i)
      {
        ptp1 += char(EEPROM.read(i));
      }
      Serial.print("ptp1: ");
      Serial.println(ptp1);

      for (i = 126; i < 142; ++i)
      {
        ptp2 += char(EEPROM.read(i));
      }
      Serial.print("ptp2: ");
      Serial.println(ptp2);

      for (i = 142; i < 162; ++i)
      {
        ptp3 += char(EEPROM.read(i));
      }
      Serial.print("ptp3: ");
      Serial.println(ptp3);

      for (i = 162; i < 182; ++i)
      {
        stp1 += char(EEPROM.read(i));
      }
      Serial.print("stp1: ");
      Serial.println(stp1);

      for (i = 182; i < 184; ++i)
      {
        ip01 += char(EEPROM.read(i));
      }
      Serial.print("ip01: ");
      Serial.println(ip01);

      for (i = 184; i < 188; ++i)
      {
        i1ce += char(EEPROM.read(i));
      }
      Serial.print("i1ce: ");
      Serial.println(i1ce);

      for (i = 188; i < 192; ++i)
      {
        i1tc += char(EEPROM.read(i));
      }
      Serial.print("i1tc: ");
      Serial.println(i1tc);

      for (i = 192; i < 194; ++i)
      {
        ip02 += char(EEPROM.read(i));
      }
      Serial.print("ip02: ");
      Serial.println(ip02);

      for (i = 194; i < 198; ++i)
      {
        i2ce += char(EEPROM.read(i));
      }
      Serial.print("i2ce: ");
      Serial.println(i2ce);

      for (i = 198; i < 200; ++i)
      {
        ip03 += char(EEPROM.read(i));
      }
      Serial.print("ip03: ");
      Serial.println(ip03);

      for (i = 200; i < 205; ++i)
      {
        i3ce += char(EEPROM.read(i));
      }
      Serial.print("i3ce: ");
      Serial.println(i3ce);

      for (i = 205; i < 210; ++i)
      {
        ddur += char(EEPROM.read(i));
      }
      Serial.print("ddur: ");
      Serial.println(ddur);

      for (i = 210; i < 215; ++i)
      {
        rest += char(EEPROM.read(i));
      }
      Serial.print("rest: ");
      Serial.println(rest);

      for (i = 215; i < 216; ++i)
      {
        i1tw += char(EEPROM.read(i));
      }
      Serial.print("i1tw: ");
      Serial.println(i1tw);

      for (i = 216; i < 217; ++i)
      {
        i2it += char(EEPROM.read(i));
      }
      Serial.print("i2it: ");
      Serial.println(i2it);

      for (i = 217; i < 222; ++i)
      {
        i2ma += char(EEPROM.read(i));
      }
      Serial.print("i2ma: ");
      Serial.println(i2ma);

      for (i = 222; i < 227; ++i)
      {
        i2mi += char(EEPROM.read(i));
      }
      Serial.print("i2mi: ");
      Serial.println(i2mi);

      for (i = 227; i < 233; ++i)
      {
        i2at += char(EEPROM.read(i));
      }
      Serial.print("i2at: ");
      Serial.println(i2at);

      for (i = 233; i < 235; ++i)
      {
        i3it += char(EEPROM.read(i));
      }
      Serial.print("i3it: ");
      Serial.println(i3it);

      for (i = 235; i < 240; ++i)
      {
        i3ma += char(EEPROM.read(i));
      }
      Serial.print("i3ma: ");
      Serial.println(i3ma);

      for (i = 240; i < 245; ++i)
      {
        i3mi += char(EEPROM.read(i));
      }
      Serial.print("i3mi: ");
      Serial.println(i3mi);

      for (i = 245; i < 251; ++i)
      {
        i3at += char(EEPROM.read(i));
      }
      Serial.print("i3at: ");
      Serial.println(i3at);

      for (i = 450; i < 461; ++i)
      {
        unm1 += char(EEPROM.read(i));
      }
      Serial.print("unm1: ");
      Serial.println(unm1);

      for (i = 461; i < 480; ++i)
      {
        pwd1 += char(EEPROM.read(i));
      }
      Serial.print("pwd1: ");
      Serial.println(pwd1);

      topicin = stp1;      //for subscribe
      topicout = ptp1;    //for publish
      macadd = getMAC();
      macadd.toCharArray(cid, 50);
      topicin.toCharArray(topic_in, 50);
      topicout.toCharArray(topic_out, 50);
      brokeradd.toCharArray(broker, 50);
      unm1.toCharArray(mqtt_name, 50);
      pwd1.toCharArray(mqtt_pwd, 50);
      Serial.println("---------------");
      Serial.println(mqtt_name);
      Serial.println(mqtt_pwd);
      Serial.println("---------------");
      int portadd = port.toInt();
      client.setServer(broker, portadd);
      client.setCallback(callback);
      if (i1tw == "1") {
        pinMode(DRDY_PIN, INPUT);
        if (!maxthermo.begin()) {
          Serial.println("Could not initialize thermocouple.");
        }
        maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
        Serial.print("Thermocouple type: ");
        switch (maxthermo.getThermocoupleType() ) {
          case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
          case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
          case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
          case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
          case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
          case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
          case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
          case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
          case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
          case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
          default: Serial.println("Unknown"); break;
        }
        maxthermo.setConversionMode(MAX31856_CONTINUOUS);
      }
      if (i1tw == "2") {
        rtd.begin(MAX31865_2WIRE);  // 2WIRE
      }
      if (i1tw == "3") {
        rtd.begin(MAX31865_3WIRE);  // 3WIRE
      }
      if (i1tw == "4") {
        rtd.begin(MAX31865_4WIRE);  //4WIRE
      }
      else {
        // do nothing
      }
      return;
    }
  }
  else
  {
    apmode();
  }
}
void apmode() {
  digitalWrite(mqttstatusled, LOW);
  digitalWrite(wifistatusled, LOW);
  Serial.println("Turning the HotSpot On");
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.macAddress(mac);
  String mac = getMAC();
  String apssid = "INFYIOT" + mac;
  Serial.println(mac);
  char apssidchar[50];
  apssid.toCharArray(apssidchar, 49);
  WiFi.softAP(apssidchar, "1122334455"); // password protected ap
  IPAddress IP = WiFi.softAPIP();
  server.begin();
  while (1) {
    WiFiClient client = server.available();     // Listen for incoming clients
    httpClient.begin(client, "http://192.168.4.1/");
    String temp_clientResponse = "";            // variable to store http get buffer
    if (client) {                              // If a new client connects,
      Serial.println("New Client.");          // print a message out in the serial port
      while (client.connected()) {           // loop while the client's connected
        Serial.println("New Client connected");
        Serial.println("Trying to read data");
        temp_clientResponse = client.readStringUntil('\n');
        Serial.println(temp_clientResponse);
        if (client.available()) {
          Serial.println("New Client available.");
        }
        else {
          Serial.println("But Client is not available");
        }
        int serialresponselength = temp_clientResponse.length();
        Serial.println(serialresponselength);
        if (serialresponselength > 150) {
          int i1 = temp_clientResponse.indexOf('&');                  //index id of the splited string
          String firstValue = temp_clientResponse.substring(0, i1);  //from index id get the string as GET /?SSID=abc
          String ssidstring = firstValue.substring(6, 10);            //from GET /?SSID=abc split SSID string for validation
          Serial.println(firstValue);              //GET /?SSID=abc
          Serial.println(ssidstring);             //SSID
          if (ssidstring == "ssid") {
            Serial.println("Done");
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: Close");
            client.println("");
            int httpResponseCode = httpClient.POST("200");
            int i1 = temp_clientResponse.indexOf('&');
            int i2 = temp_clientResponse.indexOf('&', i1 + 1);
            int i3 = temp_clientResponse.indexOf('&', i2 + 1);
            int i4 = temp_clientResponse.indexOf('&', i3 + 1);
            int i5 = temp_clientResponse.indexOf('&', i4 + 1);
            int i6 = temp_clientResponse.indexOf('&', i5 + 1);
            int i7 = temp_clientResponse.indexOf('&', i6 + 1);
            int i8 = temp_clientResponse.indexOf('&', i7 + 1);
            int i9 = temp_clientResponse.indexOf('&', i8 + 1);
            int i10 = temp_clientResponse.indexOf('&', i9 + 1);
            int i11 = temp_clientResponse.indexOf('&', i10 + 1);
            int i12 = temp_clientResponse.indexOf('&', i11 + 1);
            int i13 = temp_clientResponse.indexOf('&', i12 + 1);
            int i14 = temp_clientResponse.indexOf('&', i13 + 1);
            int i15 = temp_clientResponse.indexOf('&', i14 + 1);
            int i16 = temp_clientResponse.indexOf('&', i15 + 1);
            int i17 = temp_clientResponse.indexOf('&', i16 + 1);
            int i18 = temp_clientResponse.indexOf('&', i17 + 1);
            int i19 = temp_clientResponse.indexOf('&', i18 + 1);
            int i20 = temp_clientResponse.indexOf('&', i19 + 1);
            int i21 = temp_clientResponse.indexOf('&', i20 + 1);
            int i22 = temp_clientResponse.indexOf('&', i21 + 1);
            int i23 = temp_clientResponse.indexOf('&', i22 + 1);
            int i24 = temp_clientResponse.indexOf('&', i23 + 1);
            int i25 = temp_clientResponse.indexOf('&', i24 + 1);
            int i26 = temp_clientResponse.indexOf('&', i25 + 1);
            int i27 = temp_clientResponse.indexOf('&', i26 + 1);
            int i28 = temp_clientResponse.indexOf('&', i27 + 1);
            int i29 = temp_clientResponse.indexOf('&', i28 + 1);
            firstValue = temp_clientResponse.substring(0, i1);
            sssid = firstValue.substring(11);
            secondValue = temp_clientResponse.substring(i1 + 1, i2);
            pswd = secondValue.substring(5);
            thirdvalue = temp_clientResponse.substring(i2 + 1, i3);
            brokeradd = thirdvalue.substring(5);
            fourthvalue = temp_clientResponse.substring(i3 + 1, i4);
            unm1 = fourthvalue.substring(5);
            fifthvalue = temp_clientResponse.substring(i4 + 1, i5);
            pwd1 = fifthvalue.substring(5);
            sixthvalue = temp_clientResponse.substring(i5 + 1, i6);
            port = sixthvalue.substring(5);
            seventhvalue = temp_clientResponse.substring(i6 + 1, i7);
            ptp1 = seventhvalue.substring(5);
            eigthvalue = temp_clientResponse.substring(i7 + 1, i8);
            ptp2 = eigthvalue.substring(5);
            ninthvalue = temp_clientResponse.substring(i8 + 1, i9);
            ptp3 = ninthvalue.substring(5);
            tenthvalue = temp_clientResponse.substring(i9 + 1, i10);
            stp1 = tenthvalue.substring(5);
            eleventhvalue = temp_clientResponse.substring(i10 + 1, i11);
            ip01 = eleventhvalue.substring(5);
            tewelthvalue = temp_clientResponse.substring(i11 + 1, i12);
            i1ce = tewelthvalue.substring(5);
            thirteenthvalue = temp_clientResponse.substring(i12 + 1, i13);
            i1tc = thirteenthvalue.substring(5);
            fourteenthvalue = temp_clientResponse.substring(i13 + 1, i14);
            i1tw = fourteenthvalue.substring(5);
            fifteenthvalue = temp_clientResponse.substring(i14 + 1, i15);
            ip02 = fifteenthvalue.substring(5);
            sixteenthvalue = temp_clientResponse.substring(i15 + 1, i16);
            i2ce = sixteenthvalue.substring(5);
            seventeenthvalue = temp_clientResponse.substring(i16 + 1, i17);
            i2it  = seventeenthvalue.substring(5);
            eighteenthvalue = temp_clientResponse.substring(i17 + 1, i18);
            i2mi = eighteenthvalue.substring(5);
            nineteenthvalue = temp_clientResponse.substring(i18 + 1, i19);
            i2ma = nineteenthvalue.substring(5);
            twentethvalue = temp_clientResponse.substring(i19 + 1, i20);
            i2at = twentethvalue.substring(5);
            twentyonethvalue = temp_clientResponse.substring(i20 + 1, i21);
            ip03 = twentyonethvalue.substring(5);
            twentytwothvalue = temp_clientResponse.substring(i21 + 1, i22);
            i3ce = twentytwothvalue.substring(5);
            twentythreethvalue = temp_clientResponse.substring(i22 + 1, i23);
            i3it = twentythreethvalue.substring(5);
            twentyfourthvalue = temp_clientResponse.substring(i23 + 1, i24);
            i3ma = twentyfourthvalue.substring(5);
            twentyfivthvalue = temp_clientResponse.substring(i24 + 1, i25);
            i3mi  = twentyfivthvalue.substring(5);
            twentysixthvalue = temp_clientResponse.substring(i25 + 1, i26);
            i3at = twentysixthvalue.substring(5);
            twentyseventhvalue = temp_clientResponse.substring(i26 + 1, i27);
            ddur = twentyseventhvalue.substring(5);
            twentyeighthvalue = temp_clientResponse.substring(i27 + 1, i28);
            rest = twentyeighthvalue.substring(5);

            Serial.println("------------");
            Serial.println(firstValue);
            Serial.println(sssid);
            Serial.println(secondValue);
            Serial.println(pswd);
            Serial.println(thirdvalue);
            Serial.println(brokeradd);
            Serial.println(fourthvalue);
            Serial.println(unm1);
            Serial.println(fifthvalue);
            Serial.println(pwd1);
            Serial.println(sixthvalue);
            Serial.println(port);
            Serial.println("------------");
            Serial.println(seventhvalue);
            Serial.println(ptp1);
            Serial.println(eigthvalue);
            Serial.println(ptp2 );
            Serial.println(ninthvalue);
            Serial.println(ptp3 );
            Serial.println(tenthvalue);
            Serial.println(stp1);
            Serial.println(eleventhvalue);
            Serial.println(ip01);
            Serial.println(tewelthvalue);
            Serial.println(i1ce );
            Serial.println(thirteenthvalue);
            Serial.println(i1tc);
            Serial.println(fourteenthvalue);
            Serial.println(i1tw);
            Serial.println(fifteenthvalue);
            Serial.println(ip02);
            Serial.println(sixteenthvalue);
            Serial.println(i2ce);
            Serial.println(seventeenthvalue);
            Serial.println(i2it);
            Serial.println(eighteenthvalue);
            Serial.println(i2mi);
            Serial.println(nineteenthvalue);
            Serial.println(i2ma);
            Serial.println(twentethvalue);
            Serial.println(i2at);
            Serial.println(twentyonethvalue);
            Serial.println(ip03);
            Serial.println(twentytwothvalue);
            Serial.println(i3ce);
            Serial.println(twentythreethvalue);
            Serial.println(i3it);
            Serial.println(twentyfourthvalue);
            Serial.println(i3ma);
            Serial.println(twentyfivthvalue);
            Serial.println(i3mi);
            Serial.println(twentysixthvalue);
            Serial.println(i3at);
            Serial.println("------------");
            Serial.println(twentyseventhvalue);
            Serial.println(ddur);
            Serial.println(twentyeighthvalue);
            Serial.println(rest);
            Serial.println("------------");
            Serial.println("start");
            delay(5000);
            store();
            break;
          }
          else {
            Serial.println("Failed");
            client.println("HTTP/1.1 400 Failed");
            client.println("Content-type:text/html");
            client.println("Connection: keep-alive");
            client.println("");
            int httpResponseCode = httpClient.POST("400");
          }
        }
        if (serialresponselength < 150 || serialresponselength == 150) {
          Serial.println("Failed");
          client.println("HTTP/1.1 400 Failed");
          client.println("Content-type:text/html");
          client.println("Connection: keep-alive");
          client.println("");
          int httpResponseCode = httpClient.POST("400");
        }
      }
    }
  }
}
/**to get the mac id of the ESP to set as a topic for mqtt***/
String getMAC()
{
  uint8_t mac[6];
  String result;
  WiFi.macAddress(mac);
  //result = String(mac);
  //snprintf( result, sizeof( result ), "%02x%02x%02x%02x%02x%02x", mac[ 0 ], mac[ 1 ], mac[ 2 ], mac[ 3 ], mac[ 4 ], mac[ 5 ] );
  result = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  return result;
}
void store() {
  Serial.println(sssid);
  Serial.println(pswd);
  if (sssid.length() > 0 && pswd.length()) {
    Serial.println("Clearing EEPROM");
    for (i = 0; i < 96; ++i) {
      EEPROM.write(i, 0);
    }
    Serial.println("Writing EEPROM ssid");
    for (i = 0; i < sssid.length() ; ++i) {
      EEPROM.write(i, sssid[i]);
      Serial.println("wrote");
      Serial.println(sssid[i]);
    }
    Serial.println("Writing EEPROM pswd");
    for (i = 0; i < pswd.length() ; ++i) {
      EEPROM.write(32 + i, pswd[i]);
      Serial.println("wrote");
      Serial.println(pswd[i]);
    }
    Serial.println("Writing EEPROM mqtt server name");
    for (i = 0; i < brokeradd.length() ; ++i) {
      EEPROM.write(96 + i, brokeradd[i]);
      Serial.println("wrote");
      Serial.println(brokeradd[i]);
    }
    Serial.println("Writing EEPROM mqtt port");
    for (i = 0; i < port.length() ; ++i) {
      EEPROM.write(111 + i, port[i]);
      Serial.println("wrote");
      Serial.println(port[i]);
    }
    Serial.println("Writing EEPROM mqtt ptp1");
    for (i = 0; i < ptp1.length() ; ++i) {
      EEPROM.write(116 + i, ptp1[i]);
      Serial.println("wrote");
      Serial.println(ptp1[i]);
    }
    Serial.println("Writing EEPROM mqtt ptp2");
    for (i = 0; i < ptp2.length() ; ++i) {
      EEPROM.write(126 + i, ptp2[i]);
      Serial.println("wrote");
      Serial.println(ptp2[i]);
    }
    Serial.println("Writing EEPROM mqtt ptp3");
    for (i = 0; i < ptp3.length() ; ++i) {
      EEPROM.write(142 + i, ptp3[i]);
      Serial.println("wrote");
      Serial.println(ptp3[i]);
    }
    Serial.println("Writing EEPROM mqtt stp1");
    for (i = 0; i < stp1.length() ; ++i) {
      EEPROM.write(162 + i, stp1[i]);
      Serial.println("wrote");
      Serial.println(stp1[i]);
    }
    Serial.println("Writing EEPROM mqtt ip01");
    for (i = 0; i < ip01.length() ; ++i) {
      EEPROM.write(182 + i, ip01[i]);
      Serial.println("wrote");
      Serial.println(ip01[i]);
    }
    Serial.println("Writing EEPROM mqtt i1ce");
    for (i = 0; i < i1ce.length() ; ++i) {
      EEPROM.write(184 + i, i1ce[i]);
      Serial.println("wrote");
      Serial.println(i1ce[i]);
    }
    Serial.println("Writing EEPROM mqtt i1tc");
    for (i = 0; i < i1tc.length() ; ++i) {
      EEPROM.write(188 + i, i1tc[i]);
      Serial.println("wrote");
      Serial.println(i1tc[i]);
    }
    Serial.println("Writing EEPROM mqtt ip02");
    for (i = 0; i < ip02.length() ; ++i) {
      EEPROM.write(192 + i, ip02[i]);
      Serial.println("wrote");
      Serial.println(ip02[i]);
    }
    Serial.println("Writing EEPROM mqtt i2ce");
    for (i = 0; i < i2ce.length() ; ++i) {
      EEPROM.write(194 + i, i2ce[i]);
      Serial.println("wrote");
      Serial.println(i2ce[i]);
    }
    Serial.println("Writing EEPROM mqtt ip03");
    for (i = 0; i < i3ce.length() ; ++i) {
      EEPROM.write(198 + i, ip03[i]);
      Serial.println("wrote");
      Serial.println(ip03[i]);
    }
    Serial.println("Writing EEPROM mqtt i3ce");
    for (i = 0; i < i3ce.length() ; ++i) {
      EEPROM.write(200 + i, i3ce[i]);
      Serial.println("wrote");
      Serial.println(i3ce[i]);
    }
    Serial.println("Writing EEPROM mqtt ddur");
    for (i = 0; i < ddur.length() ; ++i) {
      EEPROM.write(205 + i, ddur[i]);
      Serial.println("wrote");
      Serial.println(ddur[i]);
    }
    Serial.println("Writing EEPROM mqtt rest");
    for (i = 0; i < rest.length() ; ++i) {
      EEPROM.write(210 + i, rest[i]);
      Serial.println("wrote");
      Serial.println(rest[i]);
    }
    Serial.println("Writing EEPROM mqtt i1tw");
    for (i = 0; i < i1tw.length() ; ++i) {
      EEPROM.write(215 + i, i1tw[i]);
      Serial.println("wrote");
      Serial.println(i1tw[i]);
    }
    Serial.println("Writing EEPROM mqtt i2it");
    for (i = 0; i < i2it.length() ; ++i) {
      EEPROM.write(216 + i, i2it[i]);
      Serial.println("wrote");
      Serial.println(i2it[i]);
    }
    Serial.println("Writing EEPROM mqtt i2ma");
    for (i = 0; i < i2ma.length() ; ++i) {
      EEPROM.write(217 + i, i2ma[i]);
      Serial.println("wrote");
      Serial.println(i2ma[i]);
    }
    Serial.println("Writing EEPROM mqtt i2mi");
    for (i = 0; i < i2mi.length() ; ++i) {
      EEPROM.write(222 + i, i2mi[i]);
      Serial.println("wrote");
      Serial.println(i2mi[i]);
    }
    Serial.println("Writing EEPROM mqtt i2at");
    for (i = 0; i < i2at.length() ; ++i) {
      EEPROM.write(227 + i, i2at[i]);
      Serial.println("wrote");
      Serial.println(i2at[i]);
    }
    Serial.println("Writing EEPROM mqtt i3it");
    for (i = 0; i < i3it.length() ; ++i) {
      EEPROM.write(233 + i, i3it[i]);
      Serial.println("wrote");
      Serial.println(i3it[i]);
    }
    Serial.println("Writing EEPROM mqtt i3ma");
    for (i = 0; i < i3ma.length() ; ++i) {
      EEPROM.write(235 + i, i3ma[i]);
      Serial.println("wrote");
      Serial.println(i3ma[i]);
    }
    Serial.println("Writing EEPROM mqtt i3mi");
    for (i = 0; i < i3mi.length() ; ++i) {
      EEPROM.write(240 + i, i3mi[i]);
      Serial.println("wrote");
      Serial.println(i3mi[i]);
    }
    Serial.println("Writing EEPROM mqtt i3at");
    for (i = 0; i < i3at.length() ; ++i) {
      EEPROM.write(245 + i, i3at[i]);
      Serial.println("wrote");
      Serial.println(i3at[i]);
    }
    Serial.println("Writing EEPROM mqtt name");
    for (i = 0; i < unm1.length() ; ++i) {
      EEPROM.write(450 + i, unm1[i]);
      Serial.println("wrote");
      Serial.println(unm1[i]);
    }
    Serial.println("Writing EEPROM mqtt password");
    for (i = 0; i < pwd1.length() ; ++i) {
      EEPROM.write(00000 + i, pwd1[i]);
      Serial.println("wrote");
      Serial.println(pwd1[i]);
    }
    EEPROM.commit();
    delay(500);
    ESP.restart();
  }
}
void callback(char* topic, byte* message, unsigned int length) {
  for (i = 0; i < length; i++) {
    textin += (char)message[i];
  }
  textin = textin;
  Serial.println(textin);
  //delay(10000);
  StaticJsonDocument <900> subdata;
  deserializeJson(subdata, message);
  String  datamacid = subdata["mac"];
  int datacondition = subdata["data"]["measurement_condition"]; //1 => ON, 2 => less_than_or_equal, 3 => greater_than_or_equal, 4 => Range
  int alerttypeid = subdata["data"]["alerttypeid"];  // caution (1) or critical (2)
  int inputid = subdata["inputid"]; // 1 for RTD, 2 for pressure, 3 for flow
  int categ = subdata["data"]["op"]; // 1 - Add, 2 - Delete, 3- Update
  int cmdtype = subdata["cmdtypeid"]; //  1 => Calibaration Command, 2 => Interval Command, 3 => Reboot Command, 4 => Alert Command, 5 => OTA Command
  //6 => reset Command
  int cmdidr = subdata["cmdid"];
  Serial.println(datamacid);
  Serial.println(datacondition);
  Serial.println(alerttypeid);
  Serial.println(inputid);
  Serial.println(categ);
  Serial.println(cmdtype);
  delay(5000);
  if (macadd.equalsIgnoreCase(datamacid)) {
    Serial.println("mac passed");
    if (cmdtype == 1) {
      if (inputid == 1) {
        float multifac = subdata["data"]["coef_value"];
        String MF = String(multifac);
        Serial.println("Received value");
        Serial.println(MF);
        Serial.println("Clearing EEPROM");
        for (i = 184; i < 188; ++i) {
          EEPROM.write(i, 0);
        }
        Serial.println("Writing EEPROM mqtt i1ce");
        for (i = 0; i < MF.length() ; ++i) {
          EEPROM.write(184 + i, MF[i]);
          Serial.println("wrote");
          Serial.println(MF[i]);
        }
        EEPROM.commit();
        delay(1000);
      }
      if (inputid == 2) {
        float multifac = subdata["data"]["coef_value"];
        String MF = String(multifac);
        Serial.println("Received value");
        Serial.println(MF);
        Serial.println("Clearing EEPROM");
        for (i = 194; i < 198; ++i) {
          EEPROM.write(i, 0);
        }
        for (i = 0; i < MF.length() ; ++i) {
          EEPROM.write(194 + i, MF[i]);
          Serial.println("wrote");
          Serial.println(MF[i]);
        }
        EEPROM.commit();
        delay(1000);
      }
      if (inputid == 3) {
        float multifac = subdata["data"]["coef_value"];
        String MF = String(multifac);
        Serial.println("Received value");
        Serial.println(MF);
        Serial.println("Clearing EEPROM");
        for (i = 200; i < 205; ++i) {
          EEPROM.write(i, 0);
        }
        for (i = 0; i < MF.length() ; ++i) {
          EEPROM.write(200 + i, MF[i]);
          Serial.println("wrote");
          Serial.println(MF[i]);
        }
        EEPROM.commit();
        delay(1000);
      }
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
    }
    if (cmdtype == 2) {
      String cmdddur = subdata["data"]["interval"];
      Serial.println(cmdddur);
      delay(1000);
      Serial.println("Clearing EEPROM");
      for (i = 205; i < 210; ++i) {
        EEPROM.write(i, 0);
      }
      Serial.println("Writing EEPROM mqtt ddur");
      for (i = 0; i < cmdddur.length() ; ++i) {
        EEPROM.write(205 + i, cmdddur[i]);
        Serial.println("wrote");
        Serial.println(cmdddur[i]);
      }
      EEPROM.commit();
      delay(500);
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
    }
    if (cmdtype == 3) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(500);
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<150> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected by software-restart";
      char buffer2[150];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(1000);
      ESP.restart();
    }
    if (cmdtype == 5) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(1000);
      /**this line helps to call the the http update check function***/
      //checkForUpdates();
      checkforupdates = true;
      return;
    }
    if (cmdtype == 6) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      digitalWrite(wifistatusled, LOW);
      digitalWrite(mqttstatusled, LOW);
      delay(1000);
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<150> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected by software-reset";
      char buffer2[150];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(1000);
      devicereset();
    }
    if (cmdtype == 9) {
      String cmdddur = subdata["data"]["interval"];
      delay(1000);
      for (i = 365; i < 370; ++i) {
        EEPROM.write(i, 0);
      }
      for (i = 0; i < cmdddur.length() ; ++i) {
        EEPROM.write(365 + i, cmdddur[i]);
      }
      EEPROM.commit();
      delay(500);
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(1000);
    }
    if (cmdtype == 8) {
      String master_slave = subdata["data"]["slave"];
      String master_baudrate = subdata["data"]["baudrate"];
      String master_parity = subdata["data"]["parity"];
      String master_tagid1 = subdata["data"]["tags"][0]["tagId"];
      String master_tagid2 = subdata["data"]["tags"][1]["tagId"];
      String master_tagid3 = subdata["data"]["tags"][2]["tagId"];
      Serial.println(master_slave);
      Serial.println(master_baudrate);
      Serial.println(master_parity);
      Serial.println(master_tagid1);
      Serial.println(master_tagid2);
      Serial.println(master_tagid3);
      delay(2000);
      Serial.println("Clearing EEPROM");
      for (int i = 401; i < 440; ++i) {
        EEPROM.write(i, 0);
      }
      Serial.println("Writing EEPROM master_slave");
      for (int i = 0; i < master_slave.length() ; ++i) {
        EEPROM.write(401 + i, master_slave[i]);
        Serial.println("wrote");
        Serial.println(master_slave[i]);
      }
      Serial.println("Writing EEPROM master_baudrate");
      for (int i = 0; i < master_baudrate.length() ; ++i) {
        EEPROM.write(405 + i, master_baudrate[i]);
        Serial.println("wrote");
        Serial.println(master_baudrate[i]);
      }
      Serial.println("Writing EEPROM master_parity");
      for (int i = 0; i < master_parity.length() ; ++i) {
        EEPROM.write(412 + i, master_parity[i]);
        Serial.println("wrote");
        Serial.println(master_parity[i]);
      }
      Serial.println("Writing EEPROM master_tagid1");
      for (int i = 0; i < master_tagid1.length() ; ++i) {
        EEPROM.write(415 + i, master_tagid1[i]);
        Serial.println("wrote");
        Serial.println(master_tagid1[i]);
      }
      Serial.println("Writing EEPROM master_tagid2");
      for (int i = 0; i < master_tagid2.length() ; ++i) {
        EEPROM.write(423 + i, master_tagid2[i]);
        Serial.println("wrote");
        Serial.println(master_tagid2[i]);
      }
      Serial.println("Writing EEPROM master_tagid3");
      for (int i = 0; i < master_tagid3.length() ; ++i) {
        EEPROM.write(430 + i, master_tagid3[i]);
        Serial.println("wrote");
        Serial.println(master_tagid3[i]);
      }
      EEPROM.commit();
      delay(500);
      StaticJsonDocument<100> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[100];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      boolean rc = client.publish(ptp3char, buffer, retain);
      delay(1000);
      ESP.restart();
    }
  }
}
void reconnect() {
  while (!client.connected()) {
    digitalWrite(mqtttatusled, LOW);
    Serial.print("Attempting MQTT connection...");
    digitalWrite(mqttstatusled, LOW);
    formattedDate = timeClient.getFormattedDate();
    StaticJsonDocument<120> doc2;
    JsonObject JSONencoder2 = doc2.to<JsonObject>();
    JSONencoder2["mac"] = macadd;
    JSONencoder2["connstatus"] = 3;
    JSONencoder2["reason"] = "Ungracefully disconnected ";
    char buffer2[120];
    serializeJson(JSONencoder2, buffer2);
    Serial.println(buffer2);
    flager = 1;
    if (client.connect(cid, mqtt_name, mqtt_pwd, "ConnectionStatus", 2, false, buffer2)) {
      rec_count = 0;
      Serial.println("connected");
      digitalWrite(mqttstatusled, HIGH);
      Serial.println(topic_in);
      client.subscribe(topic_in);
      flager = 1;
      loop();
      break;
    } else {
      digitalWrite(mqttstatusled, LOW);
      WiFi.begin(epssid.c_str(), eppswd.c_str());
      if (testWifi()) {
        Serial.println("Succesfully Connected!!!");
        digitalWrite(wifistatusled, HIGH);
      }
      else {
        Serial.println("Wifi Disconnected!!!");
        digitalWrite(wifistatusled, LOW);
        ESP.restart();
      }
      Serial.print("failed, rc=");
      Serial.println(client.state());
      flager = 1;
      rec_count = rec_count + 1;
      Serial.println(rec_count);
      Serial.println("----------------");
      Serial.println(" try again in 5 seconds");
      buttonState = digitalRead(resetbutton);
      Serial.print("buttonState = ");
      Serial.println(buttonState);
      while (buttonState == LOW ) {
        delay(100);  //if you want more resolution, lower this number
        buttonState = digitalRead(resetbutton);
        Serial.print("buttonState = ");
        Serial.println(buttonState);
        pressLength_milliSeconds = pressLength_milliSeconds + 100;
        //display how long button is has been held
        Serial.print("ms = ");
        Serial.println(pressLength_milliSeconds);
      }
      if (pressLength_milliSeconds >= optionTwo_milliSeconds) {
        Serial.println("Reset");
        delay(500);
        devicereset();
      }
      else if (pressLength_milliSeconds >= optionOne_milliSeconds) {
        Serial.println("Restart");
        delay(500);
        ESP.restart();
      }
    }
    if (rec_count > 30) {
      Serial.println("Restart");
      delay(500);
      ESP.restart();
    }
  }
  connectflag = 1;
}
void data_gen1(int tagid1x, int value1x, int value2x) {
  Serial.println("tagid1x");
  Serial.println(tagid1x);
  StaticJsonDocument<300> doc;
  JsonObject JSONencoder = doc.to<JsonObject>();
  JSONencoder["mac"] = macadd;
  JSONencoder["dt"] = formattedDate;
  JSONencoder["Subtopic"] = String(topic_in);
  JsonArray array = doc.createNestedArray("tags");
  JsonObject nested = array.createNestedObject();
  nested["inputtypeid"]  = 1;
  nested["tagid"] = tagid1x;
  nested["value1"] = value1x;
  nested["value2"] = value2x;
  serializeJson(JSONencoder, tagdata1);
}
void data_gen2(int tagid2x, int value3x, int value4x) {
  StaticJsonDocument<300> doc;
  JsonObject JSONencoder = doc.to<JsonObject>();
  JSONencoder["mac"] = macadd;
  JSONencoder["dt"] = formattedDate;
  JSONencoder["Subtopic"] = String(topic_in);
  JsonArray array = doc.createNestedArray("tags");
  JsonObject nested = array.createNestedObject();
  nested["inputtypeid"]  = 6;
  nested["tagid"] = tagid2x;
  nested["value1"] = value3x;
  nested["value2"] = value4x;
  serializeJson(JSONencoder, tagdata2);
  Serial.println("tagdata2");
  Serial.println(tagdata2);
}
void data_gen3(int tagid3x, int value5x, int value6x) {
  StaticJsonDocument<300> doc;
  JsonObject JSONencoder = doc.to<JsonObject>();
  JSONencoder["mac"] = macadd;
  JSONencoder["dt"] = formattedDate;
  JSONencoder["Subtopic"] = String(topic_in);
  JsonArray array = doc.createNestedArray("tags");
  JsonObject nested = array.createNestedObject();
  nested["inputtypeid"]  = 4;
  nested["tagid"] = tagid3x;
  nested["value1"] = value5x;
  nested["value2"] = value6x;
  serializeJson(JSONencoder, tagdata3);
  Serial.println("tagdata3");
  Serial.println(tagdata3);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    if (checkforupdates)
    {
      checkForUpdates();
    }
    while (!timeClient.update()) {
      timeClient.forceUpdate();
    }
    formattedDate = timeClient.getFormattedDate();
    if (connectflag == 1) {
      Serial.println("Send Connected.........................................");
      StaticJsonDocument<100> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 1;
      JSONencoder2["reason"] = "connected";
      char buffer2[100];
      serializeJson(JSONencoder2, buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(1000);
      connectflag = 0;
    }
    if (flag == 1) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = "null";
      JSONencoder["errcode"] = 25;
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer, retain);
      flag = 0;
    }
    if (flager == 1) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = "null";
      JSONencoder["errcode"] = 26;
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer, retain);
      flager = 0;
    }
    if (flager == 0) {
      flager = 2;
    }
    buttonState = digitalRead(resetbutton);
    Serial.print("buttonState = ");
    Serial.println(buttonState);
    while (buttonState == LOW ) {
      delay(100);  //if you want more resolution, lower this number
      buttonState = digitalRead(resetbutton);
      Serial.print("buttonState = ");
      Serial.println(buttonState);
      pressLength_milliSeconds = pressLength_milliSeconds + 100;
      //display how long button is has been held
      Serial.print("ms = ");
      Serial.println(pressLength_milliSeconds);
    }
    if (pressLength_milliSeconds >= optionTwo_milliSeconds) {
      Serial.println("Reset");
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<200> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected due to hardware reset";
      char buffer2[200];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(500);
      devicereset();
    }
    else if (pressLength_milliSeconds >= optionOne_milliSeconds) {
      Serial.println("Restart");
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<200> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected due to hardware restart";
      char buffer2[200];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(500);
      ESP.restart();
    }
    pressLength_milliSeconds = 0;

    float signalstrength = WiFi.RSSI();
    Serial.println(signalstrength);
    StaticJsonDocument<150> doc1;
    JsonObject JSONencoder1 = doc1.to<JsonObject>();
    JSONencoder1["mac"] = macadd;
    JSONencoder1["datetime"] = formattedDate;
    JSONencoder1["ssid"] = String(essid);
    JSONencoder1["pswd"] = String(epaswd);
    JSONencoder1["rssi"] = signalstrength;
    JSONencoder1["fwver"] = FW_VERSION;
    JSONencoder1["battery"] = "0";
    char buffer1[150];
    serializeJson(JSONencoder1, buffer1);
    Serial.println(buffer1);
    String tpec1 = "HeartBeat";
    tpec1.toCharArray(pterc, 50);
    int tempresult = 0;
    float presresult = 0;
    float flowresult = 0;
    int temperror = 0;
    if (ip01 == "1") {
      String ipo1mf;
      for (i = 184; i < 188; ++i)
      {
        ipo1mf += char(EEPROM.read(i));
      }
      //Serial.println("MFTemp: ");
      //Serial.println(ipo1mf);
      float MFTemp = ipo1mf.toFloat();
      //Serial.println("*******************************************");
      //Serial.println(MFTemp);
      if (i1tc == "1") {
        int thermo = maxthermo.readThermocoupleTemperature();
        uint8_t maxthermofault = maxthermo.readFault();
        if (maxthermofault) {
          Serial.print("maxthermofault 0x"); Serial.println(maxthermofault, HEX);
          tempresult = 0;
          if (maxthermofault & MAX31856_FAULT_CJRANGE) {
            temperror = 17;
            Serial.println("Cold Junction Range Fault");
          }
          if (maxthermofault & MAX31856_FAULT_TCRANGE) {
            temperror = 18;
            Serial.println("Thermocouple Range Fault");
          }
          if (maxthermofault & MAX31856_FAULT_CJHIGH) {
            temperror = 19;
            Serial.println("Cold Junction High Fault");
          }
          if (maxthermofault & MAX31856_FAULT_CJLOW) {
            temperror = 20;
            Serial.println("Cold Junction Low Fault");
          }
          if (maxthermofault & MAX31856_FAULT_TCHIGH) {
            temperror = 21;
            Serial.println("Thermocouple High Fault");
          }
          if (maxthermofault & MAX31856_FAULT_TCLOW) {
            temperror = 22;
            Serial.println("Thermocouple Low Fault");
          }
          if (maxthermofault & MAX31856_FAULT_OVUV) {
            temperror = 23;
            Serial.println("Over/Under Voltage Fault");
          }
          if (maxthermofault & MAX31856_FAULT_OPEN) {
            temperror = 24;
            Serial.println("Thermocouple Open Fault");
          }
        }
        else {
          tempresult = thermo * MFTemp;
          temperror = 0;
          if (tempresult < -10) {
            tempresult = thermo * MFTemp;
            temperror = 16;
          }
        }
      }
      if (i1tc == "100" || i1tc == "1000") {
        uint16_t rtdvalue = rtd.readRTD();
        float ratio = rtdvalue;
        ratio /= 32768;
        int rtdte = rtd.temperature(RNOMINAL, RREF);
        uint8_t fault = rtd.readFault();
        //Serial.println(fault);
        if (fault) {
          Serial.print("Fault 0x"); Serial.println(fault, HEX);
          rtd.clearFault();
          tempresult = 0;
          if (fault & MAX31865_FAULT_HIGHTHRESH) {
            Serial.println("RTD High Threshold");
            temperror = 10;
          }
          if (fault & MAX31865_FAULT_LOWTHRESH) {
            Serial.println("RTD Low Threshold");
            temperror = 11;
          }
          if (fault & MAX31865_FAULT_REFINLOW) {
            Serial.println("REFIN- > 0.85 x Bias");
            temperror = 12;
          }
          if (fault & MAX31865_FAULT_REFINHIGH) {
            Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
            temperror = 13;
          }
          if (fault & MAX31865_FAULT_RTDINLOW) {
            Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
            temperror = 14;
          }
          if (fault & MAX31865_FAULT_OVUV) {
            Serial.println("Under/Over voltage");
            temperror = 15;
          }
          rtd.clearFault();
        }
        else {
          tempresult = rtdte * MFTemp;
          temperror = 0;
          if (tempresult < -10) {
            tempresult = 0;
            temperror = 16;
          }
        }
      }
    }
    if (ip01 == "0") {
      tempresult = 0;
      temperror = 0;
    }
    if (ip02 == "1") {
      String i2coeff;
      for (i = 194; i < 198; ++i)
      {
        i2coeff += char(EEPROM.read(i));
      }
      //Serial.println("MFPresMFPres: ");
      //Serial.println(i2coeff);
      float MFPres = i2coeff.toFloat();
      Serial.println("*******************************************");
      //Serial.println(MFPres);
      if (i2it == "1") {
        float presmax = i2mi.toInt();
        float presmin = i2ma.toInt();
        int sensorValue = 0;
        float vge = 0;
        float voltage;
        if (i2at == "1") {
          for (int i = 0; i < 10; i++) {
            sensorValue = sensorValue + analogRead(Digital_SENSOR);
            voltage = sensorValue * (3.3 / 4095.0);
            vge = vge + voltage;
          }
          vge = vge / 10;
          int deciconv = float(vge) * 100;
          float vge = deciconv / 100;
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          presresult = MFPres * pressure;
        }
        if (i2at == "2") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          presresult = MFPres * pressure;
        }
        if (i2at == "3") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          presresult = MFPres * pressure;
        }
        if (i2at == "4") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          presresult = MFPres * pressure;
        }
        else {
          //do nothing
        }
      }
      else {
        //do nothing
      }
    }
    if (ip02 == "0") {
      presresult = 0;
    }
    StaticJsonDocument<400> doc;
    JsonObject JSONencoder = doc.to<JsonObject>();
    JSONencoder["mac"] = macadd;
    JSONencoder["dt"] = formattedDate;
    JSONencoder["Subtopic"] = String(topic_in);
    JsonArray array = doc.createNestedArray("temp");
    JsonObject nested = array.createNestedObject();
    nested["ipid"] = "1";
    nested["value"] = tempresult;
    nested["error_code"] = temperror;
    JsonArray array1 = doc.createNestedArray("pressure");
    JsonObject nested1 = array1.createNestedObject();
    nested1["ipid"] = "2";
    nested1["value"] = presresult;
    JsonArray array2 = doc.createNestedArray("flow");
    JsonObject nested2 = array2.createNestedObject();
    nested2["ipid"] = "3";
    nested2["value"] = flowresult;
    char buffer[400];
    serializeJson(JSONencoder, buffer);
    //Serial.println(buffer);
    ptp1.toCharArray(ptp1char, 50);

    String mastertagid1;
    for (int i = 415; i < 423; ++i)
    {
      mastertagid1 += char(EEPROM.read(i));
    }
    Serial.print("mastertagid1: ");
    Serial.println(mastertagid1);
    String tag1 = mastertagid1.c_str();
    Serial.print("--------tag1: ");
    Serial.println(tag1);
    Serial.print("--------tag1.length(): ");
    Serial.println(tag1.length());
    if (tag1 == 0) {
      mastertagid1 = "0";
    }
    else {
      mastertagid1 = mastertagid1;
    }
    Serial.print("mastertagid1: ");
    Serial.println(mastertagid1);
    String mastertagid2;
    for (int i = 423; i < 430; ++i)
    {
      mastertagid2 += char(EEPROM.read(i));
    }
    String tag2 = mastertagid2.c_str();
    if (tag2 == 0) {
      mastertagid2 = "2";
    }
    else {
      mastertagid2 = mastertagid2;
    }
    Serial.print("mastertagid2: ");
    Serial.println(mastertagid2);
    String mastertagid3;
    for (int i = 430; i < 440; ++i)
    {
      mastertagid3 += char(EEPROM.read(i));
    }
    String tag3 = mastertagid3.c_str();
    if (tag3 == 0) {
      mastertagid3 = "4";
    }
    else {
      mastertagid3 = mastertagid3;
    }
    Serial.print("mastertagid3: ");
    Serial.println(mastertagid3);
    Serial.println(mastertagid1.toInt());
    int tagid1 = mastertagid1.toInt();
    int tagid2 = mastertagid2.toInt();
    int tagid3 = mastertagid3.toInt();
    int mtag1 = tagid1;
    int mtag2 = tagid2;
    int mtag3 = tagid3;
    Serial.print("mtag1: ");
    Serial.println(mtag1);
    Serial.print("mtag2: ");
    Serial.println(mtag2);
    Serial.print("mtag3: ");
    Serial.println(mtag3);
    result_mastertagid1 = node.readHoldingRegisters(int(mtag1), 2);
    Serial.println("tag:");
    Serial.println(mtag1);
    //Serial.println(result_mastertagid1, DEC);
    if (result_mastertagid1 == node.ku8MBSuccess) {
      register1 = node.getResponseBuffer(0);
      Serial.println("register1:");
      Serial.println(register1);
      register2 = node.getResponseBuffer(1);
      Serial.println("register2:");
      Serial.println(register2);
      data_gen1(mtag1, register1, register2);
    }
    else {
      ercode =  String(result_mastertagid1, DEC);
      Serial.println("error code:");
      Serial.println(ercode);
      StaticJsonDocument<200> doc;
      JsonObject JSONencoder = doc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = 1;
      JSONencoder["tagid"] = mtag1;
      JSONencoder["errcode"] = ercode;
      char buffer[200];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      boolean rc = client.publish(pterc, buffer, retain);
    }

    result_mastertagid2 = node.readHoldingRegisters(int(mtag2), 2);
    Serial.println("tag:");
    Serial.println(mtag2);
    if (result_mastertagid2 == node.ku8MBSuccess) {
      register3 = node.getResponseBuffer(0);
      Serial.println("register3:");
      Serial.println(register3);
      register4 = node.getResponseBuffer(1);
      Serial.println("register4:");
      Serial.println(register4);
      data_gen2(mtag2, register3, register4);
    }
    else {
      ercode =  String(result_mastertagid2, DEC);
      Serial.println("error code:");
      Serial.println(ercode);
      StaticJsonDocument<200> doc;
      JsonObject JSONencoder = doc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = 6;
      JSONencoder["tagid"] = mtag2;
      JSONencoder["errcode"] = ercode;
      char buffer[200];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      boolean rc = client.publish(pterc, buffer, retain);
    }
    delay(1000);
    result_mastertagid3 = node.readHoldingRegisters(int(mtag3), 2);
    Serial.println("tag:");
    Serial.println(mtag3);
    //Serial.println(result_mastertagid3, DEC);
    if (result_mastertagid3 == node.ku8MBSuccess) {
      register5 = node.getResponseBuffer(0);
      Serial.println("register5:");
      Serial.println(register5);
      register6 = node.getResponseBuffer(1);
      Serial.println("register6:");
      Serial.println(register6);
      data_gen3(mtag3, register5, register6);
      if (millis() > time_now + period) {
        time_now = millis();
        //client.publish("TagData", tagdata1, retain);
        //client.publish("TagData", tagdata2, retain);
        //client.publish("TagData", tagdata3, retain);
      }
    }
    else {
      ercode =  String(result_mastertagid3, DEC);
      Serial.println("error code:");
      Serial.println(ercode);
      StaticJsonDocument<200> doc;
      JsonObject JSONencoder = doc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = 4;
      JSONencoder["tagid"] = mtag3;
      JSONencoder["errcode"] = ercode;
      char buffer[200];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      boolean rc = client.publish(pterc, buffer, retain);
    }
    delay(1000);

    ptp1.toCharArray(ptp1char, 50);
    String dura;
    for (i = 205; i < 210; ++i)
    {
      dura += char(EEPROM.read(i));
    }
    int sleepsec = dura.toInt();
    int sleepms = sleepsec * 1000;
    period = sleepms;
    if (millis() > time_now + period - 100) {
      time_now = millis();
      rc = client.publish(topic_out, buffer, retain);
    }
    String heatbeatdura;
    for (i = 365; i < 370; ++i)
    {
      heatbeatdura += char(EEPROM.read(i));
    }
    int heatbeatsleepsec = heatbeatdura.toInt();
    int heatbeatsleepms = heatbeatsleepsec * 1000;
    heatbeatperiod = heatbeatsleepms;
    Serial.println("==================================================================");
    Serial.println(heatbeatsleepsec);
    Serial.println(heatbeatsleepms);
    Serial.println(heatbeatperiod);
    Serial.println("==================================================================");
    if (heatbeatperiod == 0) {
      if (millis() > heatbeattime_now + heatbeatperiod) {
        heatbeattime_now = millis();
        rc = client.publish("HeartBeat", buffer1, retain);
      }
    }
    if (heatbeatperiod > 0) {
      if (millis() > heatbeattime_now + heatbeatperiod) {
        heatbeattime_now = millis();
        rc = client.publish("HeartBeat", buffer1, retain);
      }
    }
  }
  else {
    Serial.println("N/W Failed");
    ESP.restart();
  }
}
/**http update check at the http url and compare current version and the existing version in the server and also the macid's of the device if update available it downloads and compile the binary file function***/
void checkForUpdates() {
  String mac = getMAC();
  Serial.println(mac);
  String fwURL = String( fwUrlBase );
  fwURL = fwURL + brokeradd + "/iSenzrFirmware/";
  Serial.println(fwURL);
  fwURL.concat( mac );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );
  Serial.println(FW_VERSION);
  Serial.println( "Checking for firmware updates." );
  Serial.print( "MAC address: " );
  Serial.println( mac );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if ( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();

    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );

    int newVersion = newFWVersion.toInt();

    if ( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );

      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
      }
    }
    else {
      delay(100);
      Serial.println( "Already on latest version" );
      ESP.restart();
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
    delay(100);
  }
  httpClient.end();
}
bool testWifi(void) {
  int c = 0;
  Serial.println("waiting for wifi to connect");
  while (1) {
    buttonState = digitalRead(resetbutton);
    Serial.print("buttonState = ");
    Serial.println(buttonState);
    while (buttonState == LOW ) {
      delay(100);  //if you want more resolution, lower this number
      buttonState = digitalRead(resetbutton);
      pressLength_milliSeconds = pressLength_milliSeconds + 100;
      //display how long button is has been held
      Serial.print("ms = ");
      Serial.println(pressLength_milliSeconds);
    }
    if (pressLength_milliSeconds >= optionTwo_milliSeconds) {
      Serial.println("Reset");
      devicereset();
    }
    else if (pressLength_milliSeconds >= optionOne_milliSeconds) {
      Serial.println("Restart");
      ESP.restart();
    }
    pressLength_milliSeconds = 0;
    Serial.print("esid: ");
    Serial.println(epssid);
    Serial.print("epass: ");
    Serial.println(epass);
    //WiFi.begin(esid.c_str(), epass.c_str());
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(500);
    digitalWrite(wifistatusled, LOW);
    Serial.println("Connecting..");
    if (c > 30) {
      ESP.restart();
    }
    Serial.println(c);
    c++;
  }
  Serial.println("Connect timeout");
  return false;
}
void devicereset() {
  Serial.println("Clearing EEPROM");
  for (i = 0; i < 512; ++i) {
    EEPROM.write(i, 0);
    Serial.print("Done..");
    Serial.println(i);
  }
  EEPROM.commit();
  delay(500);
  ESP.restart();
}
