#include <ERROR.h>

void send_ERROR(int error_code)
{
    Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    Serial.print("ERROR: ");
    Serial.print(error_code, HEX);
    Serial.print("\n");
    switch (error_code)
    {
    case 0x00:
        Serial.println(" 1 I2C communication issue");
        break;
    case 0x01:
        Serial.println(" 2 I2C communication issue");
        break;

    case 0x02:
        Serial.println(" 3 I2C communication issue");
        break;

    case 0x03:
        Serial.println("No valid direction for L stepper motor");
        break;

    case 0x04:
        Serial.println("No valid direction for R stepper motor");
        break;

    case 0x05:
        Serial.println("L Encoder magnet not detected");
        break;

    case 0x06:
        Serial.println("R Encoder magnet not detected");
        break;

    case 0x07:
        Serial.println("Encoder angle can't be found as no encoder was selected");
        break;

    case 0x08:
        Serial.println("MOVE QUEUE IS FULL---");
        break;

    default:
        return;
        break;
    }
    Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
}