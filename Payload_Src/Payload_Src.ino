
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
float lastAlt, pressure, heading;
unsigned int lastTime;



Adafruit_BMP3XX bmp;
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_BNO055 myIMU = Adafruit_BNO055();
/*
 * sensors_event_t event;
   imu.getEvent(&event);
 * heading = event.orientation.x;
 *
 *
 */

void readEEPROM()
{


}


void updateEEPROM()
{



}



void parseCommand()
{
    if (Serial1.available())//if incoming packet from Xbee
    {
        //parse the packet
    }
    //Send packet if request for telemtry is made

}


void sampleSensors() //Poll all sensors, update values
{


}





void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);//xbee


    bmp.begin_I2C();
    for (int i = 0; i < 5; i++)
    {
        bmp.performReading();
        pressure = bmp.pressure;
    }
    //bmp.performReading()
    //bmp.temperature
    //bmp.altitude(seaLvlPressure)
    //bmp.pressure
    ina260.begin();
    //ina260.readBusVoltage()
    myIMU.begin();
    //IMU
    //

}

void loop() {
    bmp.performReading();
    sensors_event_t event;
    myIMU.getEvent(&event);


    //readings
    heading = event.orientation.x;
    pressure = bmp.pressure;
    temp = bmp.temperature;
    voltage = ina260.readBusVoltage();

    //print it via Xbee
    Serial1.println("Heading: " + (String)heading + ", Pressure: " + (String)pressure + ", Temperature: " + (String)temp + ", Voltage: " + (String)voltage);
    delay(3000);



}
