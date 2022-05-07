
/////
//STAR SABER PAYLOAD SOURCE
//By: Tristan Mcginnis
/////
#include <Adafruit_BME280.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA260.h>
#include <Servo.h>
#include <Wire.h>

//Telemetry Values
float voltage, altitude, temp, pError;//voltage draw, altitude, temperature, pointing error
float gyro_r, gyro_p, gyro_y, accel_r, accel_p, accel_y, mag_r, mag_p, mag_y;//IMU data values
int packets, state, mh, mm;//Packet Count, Flight State, Mission Hours, Mission Minutes 
float ms;//Mission seconds + milliseconds

//Operation Values
enum states
{
    idle,
    active
};
states prev_state;
states state;
float lastAlt, pressure, heading;
unsigned int lastTime;






Adafruit_BMP3XX bmp;
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_BNO055 myIMU = Adafruit_BNO055();

//**Flight State Functions**//
void idle_state()
{
    //await commands from the container
    //poll sensors every 1 second to keep awake
    return;
}

void active_state()
{
    //updateEEPROM every 1 second
    //poll every 75ms
    //if cmd recieved parseCommand()
    return;
}



//**Flight Functions**//
bool recover()
{
    return false;

}


void updateEEPROM()
{
    return;
}



void parseCommand()
{
    if (Serial1.available())//if incoming packet from Xbee
    {
        //parse the packet
    }
    //Send packet if request for telemtry is made
    return;
}


void sampleSensors() //read all sensors, update values
{

    return;
}





void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);//xbee


    bmp.begin_I2C();



    /*temporarily remove recovery functonality
    if (!recover())
    {
        //default values if not recovering
        state = idle;
    }
    */
    state = idle;
}

void loop() {
    switch (state) 
    {
    case idle:
        idle_state();
        break;
    case active:
        active_state();
        break;
    default:
        while (1)
        {
            //error loop
        }
    }
    //parseCommand() if incoming serial
}
