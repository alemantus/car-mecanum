//this is for the teensy
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

std_msgs::Int16MultiArray encoder_val;
ros::Publisher chatter("encoders", &encoder_val);

Encoder FRenc(2, 3);
Encoder FLenc(4, 5);
Encoder BRenc(0, 1);
Encoder BLenc(6, 7);

#define ENCODER_SAMPLE_TIME 50

uint32_t enc_timer = 0;
int8_t FRenc_dir = 0; //-1 = backward, 1 = forwad
uint32_t mCurPosValue = 0;

uint8_t FRpwmA = 10;
uint8_t FRpwmB = 11;

float pulsePerRevolution = 932.8; //11*2*2*21.3=932.8 for 2 encoders with 11 pulses. quadrature means times another 2 and gear reduction is 21.3

void setup()
{
    Serial.begin(5600);
    while (!Serial && millis() < 4000)
        ;

    //myTimer.begin(countSpeed, 100000); // countspeed every 0.10 seconds
    enc_timer = millis();
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);

    //ROS variable init
    encoder_val.layout.data_offset = 0;
    encoder_val.data = (uint16_t *)malloc(sizeof(uint16_t) * 5);
    encoder_val.data_length = 5;
    encoder_val.data[4] = ENCODER_SAMPLE_TIME;

    //ROS init

    nh.initNode();
    nh.advertise(chatter);
}

long oldPosition_FR = -999;
long oldPosition_FL = -999;
long oldPosition_BR = -999;
long oldPosition_BL = -999;

void loop()
{

    //encoder_val.data[0] = getSpeed(FRenc);
    long newPosition_FR = FRenc.read();
    long newPosition_FL = FLenc.read();
    long newPosition_BR = BRenc.read();
    long newPosition_BL = BLenc.read();

    if (millis() - enc_timer > ENCODER_SAMPLE_TIME)
    {
        //samples encoder ticks for ENCODER_SAMPLE_TIME and checks whether or not a change occured
        if (newPosition_FR != oldPosition_FR)
        {
            //float speedVal = newPosition_FR / 0.05 * 60.0 / pulsePerRevolution * 20.42;
            //Serial.print(speedVal);
            //Serial.print(" ");
            //Serial.println(newPosition_FR);
            encoder_val.data[0] = newPosition_FR;
            FRenc.write(0);
        }
        if (newPosition_FL != oldPosition_FL)
        {
            //float speedVal = newPosition_FR / 0.05 * 60.0 / pulsePerRevolution * 20.42;
            //Serial.print(speedVal);
            //Serial.print(" ");
            //Serial.println(newPosition_FR);
            encoder_val.data[1] = newPosition_FL;
            FLenc.write(0);
        }
        if (newPosition_BR != oldPosition_BR)
        {
            //float speedVal = newPosition_FR / 0.05 * 60.0 / pulsePerRevolution * 20.42;
            //Serial.print(speedVal);
            //Serial.print(" ");
            //Serial.println(newPosition_FR);
            encoder_val.data[2] = newPosition_BR;
            BRenc.write(0);
        }
        if (newPosition_BL != oldPosition_BL)
        {
            //float speedVal = newPosition_FR / 0.05 * 60.0 / pulsePerRevolution * 20.42;
            //Serial.print(speedVal);
            //Serial.print(" ");
            //Serial.println(newPosition_FR);
            encoder_val.data[3] = newPosition_BL;
            BLenc.write(0);
        }
        chatter.publish(&encoder_val);
        enc_timer = millis();
    }

    analogWrite(10, 200);
    analogWrite(11, 0);
    nh.spinOnce();
    delay(10);
}
