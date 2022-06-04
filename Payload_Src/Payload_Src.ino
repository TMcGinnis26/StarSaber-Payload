
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

#define serialTimeout 175
#define TeamID 1092

//Telemetry Values
float voltage, altitude, temp, pError, seaLvlPres;//voltage draw, altitude, temperature, pointing error
float gyro_r, gyro_p, gyro_y, accel_r, accel_p, accel_y, mag_r, mag_p, mag_y;//IMU data values
int packets, state, mh, mm;//Packet Count, Flight State, Mission Hours, Mission Minutes 
float ms, pointing_error;//Mission seconds + milliseconds
String curPacket, cmd;
char inChar;

//Operation Values
enum states
{
    idle,
    active
};
states prev_state;
states state;
float lastAlt, pressure, heading;
unsigned int lastSampleTime, serialWait;


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
    //if (millis() - lastSampleTime >= 75)//for limiting the sample speed
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
    //Read GPS
    
    lastSampleTime = millis();
    // }
    return;
}

void adjust_camera()
{

}

void readSerial()
{
    if (Serial1.available() > 0)
    {

        inChar = ' ';
        serialWait = millis();
        while (millis() - serialWait < serialTimeout)
        {
            if (Serial1.available())
            {
                inChar = Serial1.read();
                if (inChar == '\n')
                    break;
                cmd += String(inChar);
            }

        }

        if (cmd.substring(0,13) == "CMD,1092,POLL")//
        {
            curPacket = String(TeamID) + "," + "T" + "," + String(altitude) + "," + String(temp) + "," + String(voltage) + "," + String(gyro_r) + "," + String(gyro_p) + "," + String(gyro_y) + "," + String(accel_r) + "," + String(accel_p) + "," + String(accel_y) + "," + String(mag_r) + "," + String(mag_p) + "," + String(mag_y) + "," + String(pointing_error) + "," + String(state);

            Serial1.println(curPacket);//send to Container
        }
        if (cmd.substring(0,14) == "CMD,1092,PWRON")//
        {
            seaLvlPres = cmd.substring(15).toFloat();
            state = active;
        }
        cmd = "";
        curPacket = "";
    }
    return;
}





void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);//xbee


    bmp.begin_I2C();
    ina260.begin();
    myIMU.begin();



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
        delay(100);
        break;
    case active:
        adjust_camera();
        break;
    default:
        while (1)
        {
            //error loop
        }
    }
    sampleSensors();
    readSerial();//Read incomming serial data (xbees)
}
