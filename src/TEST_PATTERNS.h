// Pattern 1

#include "main.cpp"

void hexagon_test_pattern()
{
    delay(2000);
    currentMillis = millis();
    if (currentMillis > previousMillis + 300)
    {
        previousMillis = currentMillis;
        Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
    }
    if (ROOMBA.processMovement())
    {
        ROOMBA.moveTo(500, 1000);
        while (!ROOMBA.processMovement())
        {
            currentMillis = millis();
            if (currentMillis > previousMillis + 300)
            {
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
            }
        }
        ROOMBA.moveTo(1500, 1000);
        while (!ROOMBA.processMovement())
        {
            currentMillis = millis();
            if (currentMillis > previousMillis + 300)
            {
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
            }
        }
        ROOMBA.moveTo(2000, 0);
        while (!ROOMBA.processMovement())
        {
            currentMillis = millis();
            if (currentMillis > previousMillis + 300)
            {
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
            }
        }
        ROOMBA.moveTo(1500, -1000);
        while (!ROOMBA.processMovement())
        {
            currentMillis = millis();
            if (currentMillis > previousMillis + 300)
            {
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
            }
        }
        ROOMBA.moveTo(500, -1000);
        while (!ROOMBA.processMovement())
        {
            currentMillis = millis();
            if (currentMillis > previousMillis + 300)
            {
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
            }
        }
        ROOMBA.moveTo(0, 0);
        while (!ROOMBA.processMovement())
        {
            currentMillis = millis();
            if (currentMillis > previousMillis + 300)
            {
                previousMillis = currentMillis;
                Serial.println("X: " + (String)ROOMBA.getXCoordinate() + "  Y: " + (String)ROOMBA.getYCoordinate() + "  A: " + (String)ROOMBA.getOrientation());
            }
        }
    }
}