#include <string>
#include <set>
#include <pigpio.h>
#include <stdexcept>
#include <memory>
#include <iostream>
#include <fstream>
#include <regex>
#include <limits>

class LED{
    public:
        LED(unsigned int pin) {
            if(rosUsedPins.find(pin) != rosUsedPins.end())
            {
                throw std::invalid_argument("Pin already in use");
            }
            rosUsedPins.insert(pin);
            this->pin = pin;
            gpioSetMode(pin, PI_OUTPUT);
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
    private:
        int pin;
        static std::set<unsigned int> rosUsedPins;
};

class Heater: public LED {
    public:
        Heater(unsigned int pin) : LED(pin){}
};

class Fan: public LED {
    public:
        Fan(unsigned int pin) : LED(pin){}
};

class Thermometer {
    public:
        Thermometer(const std::shared_ptr<std::string> path) : devicePath(path){}

        float read() {
            std::ifstream device(devicePath->c_str());
            std::string line;
            const std::regex r1("([0-9a-f]{2} ){9}: crc=[0-9a-f]{2} YES");
            const std::regex r2("([0-9a-f]{2} ){9}t=([+-]?[0-9]+)");
            float temp = -std::numeric_limits<float>::infinity();
            std::getline(device, line);
            if(std::regex_match(line, r1)) {
                std::getline(device, line);
                std::smatch m;
                std::regex_search(line, m, r2);
                float temp = std::stoi(m[2].str())/1000.0;
            }
            device.close();
            return temp;
        }
    private:
        const std::shared_ptr<std::string> devicePath;
};