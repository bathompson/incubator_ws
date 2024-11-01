#pragma once

#include <string>
#include <cstring>
#include <set>
#include <pigpio.h>
#include <stdexcept>
#include <memory>
#include <iostream>
#include <fstream>
#include <regex>
#include <limits>

class LED{
  private:
        int pin;
    public:
        LED(unsigned int pin) {
          this->pin = pin;
          gpioSetMode(pin, PI_OUTPUT);
          gpioWrite(pin, 0);
        }
        ~LED() {
          gpioWrite(pin, 0);
        }
        bool getState() {
          return gpioRead(pin);
        }
        void ON() {
          gpioWrite(pin, 1);
        }
        void OFF() {
          gpioWrite(pin, 0);
        }
};

class Heater: public LED {
    public:
        Heater(unsigned int pin) : LED(pin){}
        Heater() : LED(0){}
};

class Fan: public LED {
    public:
        Fan(unsigned int pin) : LED(pin){}
        Fan() : LED(0){}
};

class Thermometer {
    public:
        Thermometer(const std::string path) : devicePath(path){}
        Thermometer() : devicePath(""){}
        Thermometer &operator=(const Thermometer &t) {
          this->devicePath = t.devicePath;

          return *this;
        }
        float read() {

          if(devicePath == "") {
            throw new std::invalid_argument("Invalid Device Path!");
          }
            std::ifstream device(devicePath.c_str());
            std::string line;
            const std::regex r1("([0-9a-f]{2} ){9}: crc=[0-9a-f]{2} YES");
            const std::regex r2("([0-9a-f]{2} ){9}t=([+-]?[0-9]+)");
            float temp = -std::numeric_limits<float>::infinity();
            std::getline(device, line);
            if(std::regex_match(line, r1)) {
                std::getline(device, line);
                std::smatch m;
                std::regex_search(line, m, r2);
                temp = std::stoi(m[2].str())/1000.0;
            }
            device.close();
            return temp;
        }
    private:
        std::string devicePath;
};