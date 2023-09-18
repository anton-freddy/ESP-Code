#include <ERROR.h>
#include <Arduino.h>
#include <TFLI2C.h>





class TF_LUNA
{
    private:
        int16_t address;
        uint16_t frame_rate = 0x00;
        int LiDAR_id;
        TFLI2C luna;
    
    public:
        TF_LUNA(int16_t address, uint16_t frame_rate, int id);
        bool begin();
        int16_t getDistance();
};
