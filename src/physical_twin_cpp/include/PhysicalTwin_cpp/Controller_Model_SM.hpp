#include <stdexcept>
#include <optional>

enum IncubatorState {
    Heating,
    CoolingDown,
    Waiting
};

class Controller_Model_SM {
    public:
        float desired_temp;
        float lower_bound;
        unsigned long heating_time;
        unsigned long heating_gap;
        IncubatorState cur_state;
        std::optional<unsigned long> nextTime;
        bool cached_heater_on;
        unsigned int actuatorEffort;
        Controller_Model_SM(float desired_temp, float lower_bound, 
        unsigned long heating_time, unsigned long heating_gap) {
            if(0 >= desired_temp || 0 >= lower_bound) {
                throw new std::invalid_argument("All parameters must be greater than zero");
            }
            this->desired_temp = desired_temp;
            this->lower_bound = lower_bound;
            this->heating_time = heating_time;
            this-> heating_gap = heating_gap;

            cur_state = IncubatorState::CoolingDown;
            nextTime = {};
            cached_heater_on = false;
            actuatorEffort = 0;
        }

        unsigned long printNextTime() {
            if(nextTime) {
                return *nextTime;
            }
            return 0;
        }

        void step(unsigned long time, float in_temp) {
            if(cur_state == IncubatorState::CoolingDown) {
                if(in_temp <= desired_temp - lower_bound) {
                    cur_state = IncubatorState::Heating;
                    cached_heater_on = true;
                    actuatorEffort++;
                    nextTime = {time + heating_time};
                }
            }
            else if(cur_state == IncubatorState::Heating) {
                if(nextTime && *nextTime <= time) {
                    cur_state = IncubatorState::Waiting;
                    cached_heater_on = false;
                    actuatorEffort++;
                    nextTime = time + heating_gap;
                } else if(in_temp > desired_temp) {
                    cur_state = IncubatorState::CoolingDown;
                    cached_heater_on = false;
                    actuatorEffort++;
                    nextTime = {};
                }
            } else if(cur_state == IncubatorState::Waiting) {
                    if(nextTime && *nextTime <= time) {
                        if (in_temp <= desired_temp) {
                            cur_state = IncubatorState::Heating;
                            cached_heater_on = true;
                            actuatorEffort++;
                            nextTime = time + heating_time;
                        }
                        else {
                            cur_state = IncubatorState::CoolingDown;
                            cached_heater_on = false;
                            nextTime = {};
                        }
                    }
                }
        }

};