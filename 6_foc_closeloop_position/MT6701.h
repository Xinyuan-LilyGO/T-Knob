#pragma once
#include "driver/spi_master.h"


#define MAG_CS  7
#define MAG_CLK 14
#define MAG_DIO 15

struct MT6701Error {
    bool error;
    uint8_t received_crc;
    uint8_t calculated_crc;
};

class MT6701Sensor {
    public:
        MT6701Sensor();

        // initialize the sensor hardware
        void init();

        // Get current shaft angle from the sensor hardware, and 
        // return it as a float in radians, in the range 0 to 2PI.
        //  - This method is pure virtual and must be implemented in subclasses.
        //    Calling this method directly does not update the base-class internal fields.
        //    Use update() when calling from outside code.
        float getSensorAngle();

        float getAngle();

        void update();

        MT6701Error getAndClearError();
    private:
        float angle_prev = 0;
        int32_t full_rotations=0; // full rotation tracking;

        spi_device_handle_t spi_device_;
        spi_transaction_t spi_transaction_ = {};

        float x_;
        float y_;
        uint32_t last_update_;

        MT6701Error error_ = {};
};