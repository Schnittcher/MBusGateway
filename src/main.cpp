#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"


#include "secrets.h"
#include "settings.h"

// declare telnet server (do NOT put in setup())
WiFiServer TelnetServer(port);
WiFiClient Telnet;

WiFiClient MQTTClient;
PubSubClient MQTTclient(MQTTClient);

WiFiServer TCPserver(iTCPport);
WiFiClient TCPclient;
uint8_t buf1[bufferSize];
uint16_t i1=0;
uint8_t buf2[bufferSize];
uint16_t i2=0;



void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived

  pinMode(led_builtin, OUTPUT);
for (int i=0; i < 10; i++) {
digitalWrite(led_builtin,LOW);
delay(50); 
digitalWrite(led_builtin,HIGH);
delay(50); 
}



Telnet.print("MQTT Rceived: ");
}


void handleTelnet()
{
  if (TelnetServer.hasClient())
  {
    // client is connected
    if (!Telnet || !Telnet.connected())
    {
      if (Telnet)
        Telnet.stop();                   // client disconnected
      Telnet = TelnetServer.available(); // ready for new client
    }
    else
    {
      TelnetServer.available().stop(); // have client, block new conections
    }
  }

  if (Telnet && Telnet.connected() && Telnet.available())
  {
    // client input processing
    while (Telnet.available())
      Serial1.write(Telnet.read()); // pass through
                                   // do other stuff with client input here
  }
}

void setup()
{
 
  Serial1.begin(115200);
  // Serial.setDebugOutput(true);
  delay(1000); // serial delay

  //connect to WiFi
  Serial1.printf("Connecting to %s ", wifi_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial1.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial1.println();

// OTA Stuff
  ArduinoOTA.setPort(iOTAPort);
  ArduinoOTA.setHostname(hostname.c_str());

  ArduinoOTA.onStart([]()
                     {
                       String type;
                       if (ArduinoOTA.getCommand() == U_FLASH)
                       {
                         type = "sketch";
                       }
                       else
                       { // U_FS
                         type = "filesystem";
                       }

                       // NOTE: if updating FS this would be the place to unmount FS using FS.end()
                       Serial1.println("Start updating " + type);
                     });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial1.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                       {
                         Serial1.println("Auth Failed");
                       }
                       else if (error == OTA_BEGIN_ERROR)
                       {
                         Serial1.println("Begin Failed");
                       }
                       else if (error == OTA_CONNECT_ERROR)
                       {
                         Serial1.println("Connect Failed");
                       }
                       else if (error == OTA_RECEIVE_ERROR)
                       {
                         Serial1.println("Receive Failed");
                       }
                       else if (error == OTA_END_ERROR)
                       {
                         Serial1.println("End Failed");
                       }
                     });

  Serial1.println("Starting OTA");
  ArduinoOTA.begin();

// TELNET Stuff
  TelnetServer.begin();
  Serial1.print("Starting telnet server on port " + (String)port);
  TelnetServer.setNoDelay(true); // ESP BUG ?
  Serial1.println();
  delay(100);

// MQTT Stuff
Serial1.println("Starting MQTT");
MQTTclient.setServer(MQTTBroker, MQTTPort);
MQTTclient.setCallback(callback);

 Serial.begin(UART_BAUD, SERIAL_8E1);

 Serial1.println("Starting TCP Server");
  TCPserver.begin(); // start TCP server 

}

void loop()
{
  ArduinoOTA.handle();
  handleTelnet();
//  Telnet.println("uptime: " + (String)millis() + " ms");
  
Serial1.print("IP: "); 
Serial1  .println(WiFi.localIP().toString().c_str());

  while (!MQTTclient.connected())
    {
      MQTTclient.connect(hostname.c_str());
      delay(1000);
    }
  
// MQTTclient.subscribe("ESP_MBusGateway/test");

/*
  String sTopic;
  sTopic = hostname + "/Device/" + hostname;
  MQTTclient.publish(sTopic.c_str() , hostname.c_str());

  sTopic = hostname + "/Device" + "/IP Adresse";
  MQTTclient.publish(sTopic.c_str() , WiFi.localIP().toString().c_str());
*/
/*
pinMode(led_builtin, OUTPUT);
for (int i=0; i < 5; i++) {
digitalWrite(led_builtin,LOW);
delay(100); 
digitalWrite(led_builtin,HIGH);
delay(100); 
}
*/

if(!TCPclient.connected()) { // if client not connected
    TCPclient = TCPserver.available(); // wait for it to connect
    return;
  }


if(TCPclient.available()) {
    while(TCPclient.available()) {
      buf1[i1] = (uint8_t)TCPclient.read(); // read char from client (RoboRemo app)
      if(i1<bufferSize-1) i1++;
    }
    // now send to UART:
    Serial.write(buf1, i1);
    i1 = 0;
    
    String str1 = (char*)buf1;
    Telnet.print("Data from Host:");
    Telnet.println(str1);
  }
  /*
  buf1[1] = "1";
  buf1[2] = "2";
  
    Serial.write(buf1, 2);
    i1 = 0;
    
    String str1 = (char*)buf1;
    Telnet.print("Data from Host:");
    Telnet.println(str1);
  
*/
 

if(Serial.available()) {
    // read the data until pause:
    
    while(1) {
      if(Serial.available()) {
        buf2[i2] = (char)Serial.read(); // read char from UART
        if(i2<bufferSize-1) i2++;
      } else {
        //delayMicroseconds(packTimeoutMicros);
        delay(packTimeout);
        if(!Serial.available()) {
          break;
        }
      }
    }

 TCPclient.write((char*)buf2, i2);
    i2 = 0;

  String str2 = (char*)buf2;
  Telnet.print("Data from Meter:");
  Telnet.println(str2);
}

delay(100); //wait for conversion

//delay(5000); //wait for conversion

}