
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
#define serialTimeout 100
#define TeamID 1092

//Telemetry Values
float voltage, altitude, temp, pError, seaLvlPres = 988.0;//voltage draw, altitude, temperature, pointing error
float gyro_r, gyro_p, gyro_y, accel_r, accel_p, accel_y, mag_r, mag_p, mag_y;//IMU data values
int packets, mh, mm;//Packet Count, Flight State, Mission Hours, Mission Minutes 
float ms, pointing_error;//Mission seconds + milliseconds
String curPacket, cmd;
char inChar;

//Operation Values
enum states
{
    Stby = 0,
    Active = 1,
    Polling = 2,
    Grnd = 3
};
states state;
float lastAlt, prevAlt, pressure, heading;
unsigned int lastSampleTime, serialWait, lastReadAlt, lastPoll;


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
    //if (millis() - lastSampleTime >= 10)//for limiting the sample speed
    //{
    bmp.performReading();
    temp = bmp.temperature;

    voltage = ina260.readBusVoltage() * 0.001;
    altitude = bmp.readAltitude(seaLvlPres);

    //read the IMU

        /*
    if (ledstat)
    {
        digitalWrite(TESTLED, LOW);
        ledstat = false;
    }
    else
    {
        digitalWrite(TESTLED, HIGH);
        ledstat = true;
    }
        */

        //lastSampleTime = millis();
   // }
    return;
}

void adjust_camera()
{

    return;
}

void readSerial()
{
    if (Serial1.available() > 0)
    {
        digitalWrite(TESTLED, HIGH);
        //delay(5000);
        inChar = ' ';
        serialWait = millis();
        while (millis() - serialWait < serialTimeout)
        {
            if (Serial1.available())
            {
                cmd += Serial1.readString();
                if (cmd.indexOf('\n') != -1)
                    break;
                //cmd += String(inChar);
            }

        }

        /*
        if (cmd.substring(0, 11) == "CMD1092POLL")//
        {
            sampleSensors();
            curPacket = String(TeamID) + "," + "T" + "," + String(altitude) + "," + String(temp) + "," + String(voltage) + "," + String(gyro_r) + "," + String(gyro_p) + "," + String(gyro_y) + "," + String(accel_r) + "," + String(accel_p) + "," + String(accel_y) + "," + String(mag_r) + "," + String(mag_p) + "," + String(mag_y) + "," + String(pointing_error) + "," + String(state);

            Serial1.println(curPacket);//send to Container
        }
        */

        if (cmd.substring(0, 14) == "CMD,1092,PWRON")//
        {
            seaLvlPres = cmd.substring(15).toFloat();
            state = Active;
            digitalWrite(TESTLED, HIGH);
            delay(5000);
        }

        cmd = "";
        curPacket = "";
    }
    return;
}

bool check_landing()
{
    if (millis() - lastReadAlt >= 2000)//check every 2 sec
    {
        altitude = bmp.readAltitude(seaLvlPres);
        if (altitude < prevAlt + 1 && altitude > prevAlt - 1 && altitude < -30.0)
        {
            return true;
        }
        lastReadAlt = millis();
        prevAlt = altitude;
        return false;

    }
    return false;
}

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);//xbee
    pinMode(TESTLED, OUTPUT);
    digitalWrite(TESTLED, HIGH);
    Serial.println("ON!");
    delay(500);
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
    state = Stby;
    lastAlt = 10.0;
}

void loop() {
    switch (state)
    {
    case Stby:
        altitude = bmp.readAltitude(seaLvlPres);
        Serial1.println(String(altitude) + " Prev: " + String(lastAlt));
        if (altitude > 500 && millis() - lastReadAlt >= 500)
        {
            Serial1.println("ALTITUDE PASS"+String(altitude) + "<" + String(lastAlt));
            if (altitude < prevAlt)
            {
                Serial1.println("NOW ACTIVE");
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
        //readSerial();
        if (bmp.readAltitude(seaLvlPres) <= 300)
        {
            state = Polling;
        }
        
        break;
    case Polling:
        adjust_camera();

        if (millis() - lastPoll >= 225)
        {
            digitalWrite(TESTLED, HIGH);
            sampleSensors();
            //curPacket = String(TeamID) + "," + "T" + "," + String(altitude) + "," + String(temp) + "," + String(voltage) + "," + String(gyro_r) + "," + String(gyro_p) + "," + String(gyro_y) + "," + String(accel_r) + "," + String(accel_p) + "," + String(accel_y) + "," + String(mag_r) + "," + String(mag_p) + "," + String(mag_y) + "," + String(pointing_error) + "," + String(state);
            Serial1.println(String(TeamID) + "," + "T" + "," + String(altitude) + "," + String(temp) + "," + String(voltage) + "," + String(gyro_r) + "," + String(gyro_p) + "," + String(gyro_y) + "," + String(accel_r) + "," + String(accel_p) + "," + String(accel_y) + "," + String(mag_r) + "," + String(mag_p) + "," + String(mag_y) + "," + String(pointing_error) + "," + String(state));//send to Container
            lastPoll = millis();
            digitalWrite(TESTLED, LOW);
        }

        //if (!check_landing())
        {
            //  state = Grnd;
        }
        break;
    case Grnd:

        break;
    default:
        while (1)
        {
            //error loop
        }
    }
    //sampleSensors();
    //Read incomming serial data (xbees)
}
