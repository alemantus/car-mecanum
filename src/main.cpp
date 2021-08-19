
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/String.h>

Encoder FRenc(2, 3);
//Encoder FLenc(4, 5);
Encoder BRenc(0, 1);
//Encoder BLenc(6, 7);
//timers
IntervalTimer myTimer;

uint32_t enc_timer = 0;
int8_t FRenc_dir = 0; //-1 = backward, 1 = forwad
uint32_t mCurPosValue = 0;

uint8_t FRpwmA = 10;
uint8_t FRpwmB = 11;

double pulsePerRevolution = 932.8; //11*2*2*21.3=932.8 for 2 encoders with 11 pulses. quadrature means times another 2 and gear reduction is 21.3

void countSpeed()
{
}
void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 4000)
        ;

    myTimer.begin(countSpeed, 100000); // countspeed every 0.10 seconds
    enc_timer = millis();
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
}

long oldPosition = -999;
//long newPosition = 0;

void getSpeed(Encoder encNumber)
{

    if (millis() - enc_timer > 1000)
    {
        long newPosition = encNumber.read();
        if (newPosition != oldPosition)
        {

            if (oldPosition < newPosition)
            {
                FRenc_dir = 1;
            }
            else
            {
                FRenc_dir = -1;
            }
            Serial.println(newPosition);
            //Serial.println((double)newPosition / 0.05 * 60.0 / pulsePerRevolution * 20.42035225);
            oldPosition = newPosition;

            //Serial.println(FRenc_dir);
            FRenc.write(0);
        }
        //encNumber.write(0);
        enc_timer = millis();
    }
}

void loop()
{

    getSpeed(FRenc);
    //FRenc.write(0);
    //getSpeed(FLenc);
    //getSpeed(BRenc);
    //BRenc.write(0);
    //getSpeed(BLenc);
    //Serial.println(newPosition);
    delay(5);
    analogWrite(11, 200);
    analogWrite(10, 0);
}