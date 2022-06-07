
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

#define TESTLED 13
#define serialTimeout 500
#define TeamID 1092

//Telemetry Values
float voltage, altitude, temp, pError, seaLvlPres = 988.0;//voltage draw, altitude, temperature, pointing error
float gyro_r, gyro_p, gyro_y, accel_r, accel_p, accel_y, mag_r, mag_p, mag_y;//IMU data values
int packets, mh, mm;//Packet Count, Flight State, Mission Hours, Mission Minutes 
float ms, pointing_error;//Mission seconds + milliseconds
String curPacket, cmd;
char inChar;
String Packet1, Packet2, Packet3, Packet4;

//Operation Values
enum states
{
    Stby = 0,
    Active = 1,
    Grnd = 2
};
states state;
float lastAlt, prevAlt, pressure, heading, passedTime;
unsigned int lastSampleTime, serialWait, lastReadAlt, lastPoll, recordTime;


Adafruit_BMP3XX bmp;
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_BNO055 myIMU = Adafruit_BNO055();



//**Flight Functions**//
bool recover()
{
    return false;

}


void updateEEPROM()
{
    return;
}






void sampleSensors()
{
    
    //bmp.performReading();
    temp = bmp.temperature;

    voltage = ina260.readBusVoltage() * 0.001;
    altitude = bmp.readAltitude(seaLvlPres);

    //read the IMU

    
    return;
}

void adjust_camera()
{

    return;
}

void readSerial()
{
    cmd = "";
    curPacket = "";
    if (Serial1.available())
    {
        //digitalWrite(TESTLED, HIGH);
        inChar = ' ';
        serialWait = millis();
        //Serial1.println("Start Reading: " + millis() / 1000);
       // while (millis() - serialWait < serialTimeout)
        //{
            while (Serial1.available())
            {
                //inChar = Serial1.read();
                cmd += Serial1.readStringUntil('\n');
                if (cmd.indexOf('\n') != -1)
                {
                    break;
                }
            }
        //}

        
        if (cmd.substring(9, 13) == "POLL")//
        {

            for (int i = 0; i < 4; i++)
            {
                sampleSensors();
                Packet1 = String(TeamID) + "," + "T" + "," + String(altitude) + "," + String(temp) + "," + String(voltage) + "," + String(gyro_r) + "," + String(gyro_p) + "," + String(gyro_y) + "," + String(accel_r) + "," + String(accel_p) + "," + String(accel_y) + "," + String(mag_r) + "," + String(mag_p) + "," + String(mag_y) + "," + String(pointing_error) + "," + String(state);
                Serial1.println(Packet1);
                delay(5);
            }
            return;
        }

        if (cmd.substring(0, 14) == "CMD,1092,PWRON")//
        {
            seaLvlPres = cmd.substring(15).toFloat();
            state = Active;
            digitalWrite(TESTLED, HIGH);
            return;
            //delay(5000);
        }

        if (cmd.substring(0, 13) == "CMD,1092,LAND")//
        {
            state = Grnd;
            digitalWrite(TESTLED, HIGH);
            return;
            //delay(5000);
        }
    }
    return;
}

/*
bool check_landing()
{
    if (millis() - lastReadAlt >= 2000)//check every 2 sec
    {
        altitude = bmp.readAltitude(seaLvlPres);
        if (altitude < prevAlt + 1 && altitude > prevAlt - 1 && altitude < 100)
        {
            return true;
        }
        lastReadAlt = millis();
        prevAlt = altitude;
        return false;

    }
    return false;
}
*/

void setup() {
    //Serial.begin(9600);
    Serial1.begin(115200);//xbee
    Serial1.setTimeout(100);
    pinMode(TESTLED, OUTPUT);
    digitalWrite(TESTLED, HIGH);
    delay(200);
    digitalWrite(TESTLED, LOW);

    bmp.begin_I2C();
    ina260.begin();
    //myIMU.begin();



    /*temporarily remove recovery functonality
    if (!recover())
    {
        //default values if not recovering
        state = Stby;
    }
    */
    //state = Stby;
    state = Active;
    lastAlt = 10.0;
}

void loop() {
    switch (state)
    {
    case Stby:
        altitude = bmp.readAltitude(seaLvlPres);
        if (altitude > 500 && millis() - lastReadAlt >= 500)
        {
            if (altitude < prevAlt)
            {
                state = Active;
                break;
            }
            lastAlt = altitude;
            lastReadAlt = millis();
        }
        lastAlt = altitude;
        break;
    case Active:
        adjust_camera();
        readSerial();
        sampleSensors();
        break;
    case Grnd:
        //do landing stuff
        break;
    default:
        while (1)
        {
            //error loop
        }
    }
}
