
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
#define ServoPin 2

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
}state;

float lastAlt, prevAlt, pressure, heading, passedTime;
unsigned int lastSampleTime, serialWait, lastReadAlt, lastPoll, recordTime;
int movement, error;
bool servoActive;


Adafruit_BMP3XX bmp;
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_BNO055 bno = Adafruit_BNO055();
Servo servo;



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
    sensors_event_t event;
    bno.getEvent(&event);

    gyro_p = event.gyro.x;//pitch
    gyro_r = event.gyro.z;//roll
    gyro_y = event.gyro.y;//yaw

    mag_p = event.magnetic.x;//pitch
    mag_r = event.magnetic.z;//roll
    mag_y = event.magnetic.y;//yaw

    accel_p = event.acceleration.x;//pitch
    accel_r = event.acceleration.z;//roll
    accel_y = event.acceleration.y;//yaw

    
    return;
}

void adjust_camera()
{
    sensors_event_t event;
    bno.getEvent(&event);

    heading = event.orientation.x;
    error = 180 - heading;

    if (error < -5 || error > 5)//if need to move
    {
        if (!servoActive)
        {
            servo.attach(ServoPin);
        }
        movement = (int)(heading / 2.0);
        servo.write(movement);
    }
    else
    {
        servo.detach();
    }
    return;
}

void readSerial()
{
    cmd = "";
    curPacket = "";
    if (Serial1.available())
    {
            //while (Serial1.available())
            //{
                //inChar = Serial1.read();
                cmd += Serial1.readStringUntil('\n');
                if (cmd.indexOf('\n') != -1)
                {
                    return;
                }
            //}
        

        
        if (cmd.substring(9, 13) == "POLL")//
        {

            for (int i = 0; i < 4; i++)
            {
                sampleSensors();
                Serial1.print(TeamID);
                Serial1.print(",T,");
                Serial1.print(altitude); Serial1.print(",");
                Serial1.print(temp); Serial1.print(",");
                Serial1.print(voltage); Serial1.print(",");
                Serial1.print(gyro_r); Serial1.print(","); 
                Serial1.print(gyro_p); Serial1.print(",");
                Serial1.print(gyro_y); Serial1.print(","); 
                Serial1.print(accel_r); Serial1.print(",");
                Serial1.print(accel_p); Serial1.print(","); 
                Serial1.print(accel_y); Serial1.print(",");
                Serial1.print(mag_r); Serial1.print(","); 
                Serial1.print(mag_p); Serial1.print(",");
                Serial1.print(mag_y); Serial1.print(","); 
                Serial1.print(pointing_error); Serial1.print(",");
                Serial1.println(state);
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
    bno.begin();
    servo.attach(ServoPin);


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
        break;
    case Grnd:
        while (1)
            delay(500);
        break;
    default:
        while (1)
        {
            //error loop
        }
    }
}
