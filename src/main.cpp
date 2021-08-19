//this is for the teensy
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

std_msgs::Float32MultiArray encoder_val;
ros::Publisher chatter("encoders", &encoder_val);

Encoder FRenc(2, 3);
//Encoder FLenc(4, 5);
//Encoder BRenc(0, 1);
//Encoder BLenc(6, 7);
//timers
IntervalTimer myTimer;

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
    encoder_val.layout.dim = (std_msgs::MultiArrayDimension *)
        malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    encoder_val.layout.dim[0].label = "encoder";
    encoder_val.layout.dim[0].size = 4;
    encoder_val.layout.dim[0].stride = 1;
    encoder_val.layout.data_offset = 0;
    encoder_val.data = (float *)malloc(sizeof(float) * 8);
    encoder_val.data_length = 4;

    //ROS init
    /*
    nh.initNode();
    nh.advertise(chatter);
    */
}

long oldPosition = -999;
long newPosition = 0;

float getSpeed(Encoder encNumber)
{
    newPosition = encNumber.read();

    if (millis() - enc_timer > 50)
    {
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
            //Serial.println(newPosition);
            return (float)(newPosition / (float)0.05 * (float)60.0 / pulsePerRevolution * (float)20.42);
            oldPosition = newPosition;

            //Serial.println(FRenc_dir);
            encNumber.write(0);
        }
        //encNumber.write(0);
        enc_timer = millis();
    }
}

void loop()
{

    //encoder_val.data[0] = getSpeed(FRenc);
    Serial.println(getSpeed(FRenc));
    FRenc.write(0);
    //getSpeed(FLenc);
    //getSpeed(BRenc);
    //BRenc.write(0);
    //getSpeed(BLenc);
    //Serial.println(newPosition);
    //delay(10);
    analogWrite(10, 200);
    analogWrite(11, 0);
    /*
    encoder_val.data[1] = 6.14;
    chatter.publish(&encoder_val);
    nh.spinOnce();
    delay(10);
    */
}
