int led_builtin = 2;

// Wifi Settings
String hostname = "ESP_MBusGateway";

// Telnet Settings
int port = 23;

// MQTT Settings
const char* MQTTBroker = "10.0.0.1"; //IPS Server
const int MQTTPort = 1883;  // IPS MQTT Port

// OTA Settings
int iOTAPort = 8266;    // ESP32: 3232, ESP8266: 8266


#define UART_BAUD 2400
#define packTimeout 5 // ms (if nothing more on UART, then send packet)
#define bufferSize 8192

const int iTCPport = 9876;