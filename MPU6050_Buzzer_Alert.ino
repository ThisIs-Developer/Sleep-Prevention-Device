/*
     vcc = 3.3v
     Gnd = gnd
     int = 2
     scl = a5
     sda = a4
*/

#include "I2Cdev.h"
#include "MPU6050.h"

int buzzer = 13;

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int val;
int prevVal;

int valax;
int valay;
int valaz;

// Tracks the time since last event fired
unsigned long previousMillis = 0;
unsigned long int previoussecs = 0;
unsigned long int currentsecs = 0;
unsigned long currentMillis = 0;

int secs = 0;

int interval = 1; // updated every 1 second

void setup()
{
    Wire.begin();
    Serial.begin(38400);
    Serial.println("Initialize MPU");
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
    pinMode(buzzer, OUTPUT);
}

void loop()
{

    currentMillis = millis();
    currentsecs = currentMillis / 1000;
    if ((unsigned long)(currentsecs - previoussecs) >= interval)
    {
        secs = secs + 1;
        previoussecs = currentsecs;
        if (secs >= 59)
        {
            secs = 0;
        }
        //  Serial.println("Seconds:");
        //  Serial.println(secs);
    }

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    valax = map(ax, -17000, 17000, 0, 255);
    valay = map(ay, -17000, 17000, 0, 255);
    valaz = map(az, -17000, 17000, 0, 255);

    Serial.println("ax: ");
    Serial.println(valax);
    // Serial.println("ay:");
    // Serial.println(valay);
    // Serial.println("az: ");
    // Serial.println(valaz);
    // Serial.println("");
    // Serial.println("");
    delay(200);

    if ((valax > 130) && (secs > 2))
    {
        digitalWrite(buzzer, HIGH);
    }

    if ((valax >= 60) && (valax <= 100))
    {
        digitalWrite(buzzer, LOW);
        secs = 0;
    }

    if ((valax < 50) && (secs > 2))
    {
        digitalWrite(buzzer, HIGH);
    }
}