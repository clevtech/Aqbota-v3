#ifndef ROS_ARDUINO_TCP_HARDWARE_H_
#define ROS_ARDUINO_TCP_HARDWARE_H_

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h> // Using Espressif's WiFi.h
#else
  #include <SPI.h>
  #include <Ethernet.h>
#endif

class ArduinoHardware {
public:
  ArduinoHardware()
  {
  }

  void setConnection(IPAddress &server, int port = 11411)
  {
    server_ = server;
    serverPort_ = port;
  }

  IPAddress getLocalIP()
  {
#if defined(ESP8266) or defined(ESP32)
    return tcp_.localIP();
#else
    return Ethernet.localIP();
#endif
  }

  void init()
  {
    if(tcp_.connected())
    {
      tcp_.stop();
    }
    tcp_.connect(server_, serverPort_);
  }

  int read(){
    if (tcp_.connected())
    {
        return tcp_.read();
    }
    else
    {
      tcp_.stop();
      tcp_.connect(server_, serverPort_);
    }
    return -1;
  }

  void write(const uint8_t* data, int length)
  {
    tcp_.write(data, length);
  }

  unsigned long time()
  {
    return millis();
  }

  bool connected()
  {
    return tcp_.connected();
  }

protected:
#if defined(ESP8266) or defined(ESP32)
  WiFiClient tcp_;
#else
  EthernetClient tcp_;
#endif
  IPAddress server_;
  uint16_t serverPort_ = 11411;
};

#endif