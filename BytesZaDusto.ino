//Basic Arduino Functions
#include <Arduino.h>
//ESP Comm Functionality
#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
//Sensor Comm
#include <Wire.h>
#include <VL53L0X.h>
//Servo Comm
#include <Servo.h>
//Custom Header Files
#include "comm.h"
//#include "file.h"
//#include "server.h"


//NOTE: Debug control is in the comm section


const int MPU9250_ADDR = 0x68;
const int MAG_ADDRESS = 0x0C;
const int RSIDE_LIDAR_ADDRESS = 0x16;
const int FRONT_LIDAR_ADDRESS = 0x19;

const int GYRO_FULL_SCALE_250_DPS = 0x00;
const int GYRO_FULL_SCALE_500_DPS = 0x08;
const int GYRO_FULL_SCALE_1000_DPS = 0x10;
const int GYRO_FULL_SCALE_2000_DPS = 0x18;

const int ACC_FULL_SCALE_2_G = 0x00;
const int ACC_FULL_SCALE_4_G = 0x08;
const int ACC_FULL_SCALE_8_G = 0x10;
const int ACC_FULL_SCALE_16_G = 0x18;

const int LED_PIN = 16;//D0
const int SDA_PIN = 14;//D7
const int SCL_PIN = 12;//D6
const int LEFT_MOTOR = 4;//D2
const int RIGHT_MOTOR = 5;//D1
const int EN_FRONT_LIDAR = 0;//D3
const int EN_RSIDE_LIDAR = 2;//D4

const int forward = 0;
const int backward = 1; 
const int left = 2; 
const int right = 3;
const int stay = 4;

//Servo
int LSERVO_NULL = 90;
int RSERVO_NULL = 90;
Servo Left;
Servo Right;

//Laser Range Finder
VL53L0X FRONT;
VL53L0X RSIDE;

WiFiClient client;

//.csv formatting
String csvLine;
String csvFileArr;
//Wireless Parameters
// WiFi AP parameters
char* ap_ssid = "SHEER_HEART_ATTACK";
char* ap_password = "";
WiFiServer server(80);


//Operating Variables
unsigned long oldTime;
unsigned long timet;

int brother_give_me_the_loops;

void setup()
{
	//Enable Comm
	Serial.begin(115200);
	Wire.begin(SDA_PIN, SCL_PIN);
	delay(1000);
	Serial.flush();
	log("Captian, internal communications diagnostic starting!\n");
	
		//wifi communications	 
    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("\t[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        delay(1000);
    }
    //setupSTA(sta_ssid, sta_password);
    WiFi.begin(ap_ssid, ap_password);
	while (WiFi.status() != WL_CONNECTED) 
		{
			Serial.println("Wifi Connecting");
		}
	 
	  Serial.println("Connected to the WiFi network. IP is ");
	  Serial.println(WiFi.localIP());
	 
	  server.begin();
    //setupMDNS(mDNS_name);
	
    
	
	log("Long range communications online!\n");
	//
	
	
	//motor attachment
	Left.attach(LEFT_MOTOR);
	Right.attach(RIGHT_MOTOR);
	log("Engines online.\n");
	
	//lasers
	pinMode(D3, OUTPUT);
	pinMode(D4, OUTPUT);
	//"enables" for LIDAR
	digitalWrite(EN_RSIDE_LIDAR, HIGH);
	delay(150);
	RSIDE.init(true);
	delay(100);
	RSIDE.setAddress(RSIDE_LIDAR_ADDRESS);
	digitalWrite(EN_FRONT_LIDAR, HIGH);
	delay(150);
	FRONT.init(true);
	delay(100);
	FRONT.setAddress(FRONT_LIDAR_ADDRESS);
	log("Laser array addressing complete.\n");
    delay(1000);
  	//I2C Verification 
	log ("Beginning Internal Communication verification\n");
	byte count = 0;
	for (byte i = 1; i < 120; i++)
	{
	
		Wire.beginTransmission (i);
		if (Wire.endTransmission () == 0)
		{
			log("Found address: ");
			log(i, DEC);
			log(" (0x");
			log(i, HEX);
			log(")\n");
			count++;
			delay(1);  // maybe unneeded?
		} // end of good response
	} // end of for loop
	log ("Internal diagnostic has verified ");
	log(count, DEC);
	log (" device(s).\n");
	
	if(count == 4)
		log("Captain, all internal systems show green.\n");
	else
		log("Captain, we have an internal comm error.\n");
	delay(1000);
	
		
	
	//Enable bypass mode for the I2C on the MPU
	I2CwriteByte(MPU9250_ADDR,0x37,0x02);
  
	// Request first magnetometer single measurement
	I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
	log("Sensor Array online. \n");
	
	
	delay(1000);

	
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	log("Running Lights Active.\n");
	
	
	move(stay, 0);
	log("Engines at 0%, captain.\n");
	
	csvLine = "";
	csvFileArr = "";
	
	//zero out the heading

	
	log("SHEER HEART ATTACK HAS NO WEAKNESS.\n");
	brother_give_me_the_loops=0;
	client = server.available();
	delay(5000);
}

void loop()
{
	
	while (!client.connected())//hold until a client is connected
	{}
	String msg = "";
	while(!client)//wait until client has stuff for me
	{}

	while (client.available()>0) //while the client's stuff is still there
	{
        msg += client.read();
    }
	//parse the message now
	char cmd = msg[0];
	String speed = msg;
	int spd = speed.toInt();
	log("The recieved Command was:\n");
	log(cmd);
	log(speed);
	log("\n");
	
	switch(cmd)//convert the command and speed to motion
	{
		case 'w':
		{
			move(forward, spd);
			break;
		}
		case 's':
		{
			move(backward, spd);
			break;
		}
		case 'a':
		{
			move(left, spd);
			break;
		}
		case 'd':
		{
			move(right, spd);
			break;
		}
		default:
		{
			move(stay, spd);
			break;
		}
	}
	
	
	
	brother_give_me_the_loops++;
	
	//collect data, and add to the csvLine
	//Magnetometer Data
		// Request first magnetometer single measurement
		
		
		I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
		
		// Read register Status 1 and wait for the DRDY: Data Ready
		  
		uint8_t ST1;
		do
		{
			I2Cread(MAG_ADDRESS,0x02,1,&ST1);
		}
		while (!(ST1&0x01));

		// Read magnetometer data  
		uint8_t Mag[7];  
		I2Cread(MAG_ADDRESS,0x03,7,Mag);

		// Create 16 bits values from 8 bits data
		  
		// Magnetometer
		int16_t mx=(Mag[1]<<8 | Mag[0]);
		int16_t my=(Mag[3]<<8 | Mag[2]);
		int16_t mz=(Mag[5]<<8 | Mag[4]);
		//mx, my, and mz contain the raw magnetometer data
		
		
	//Gyroscope
		// Read gyro data  
		uint8_t gyr[7];  
		I2Cread(MPU9250_ADDR,0x43,7,gyr);

		// Create 16 bits values from 8 bits data
		  
		// gyro conversion (131 lsb multiplied if gyro default)
		int16_t rx=(gyr[1]<<8 | gyr[0]);
		int16_t ry=(gyr[3]<<8 | gyr[2]);
		int16_t rz=(gyr[5]<<8 | gyr[4]);
		//rx, ry, and rz contain the raw magnetometer data
		
		
		//TODO: implement angle conversion with accelerometer
		//based measurement
		
		
		
	//LIDAR
		uint8_t frontLidar = FRONT.readRangeSingleMillimeters();
		if (FRONT.timeoutOccurred())
			frontLidar = 0; 

		uint8_t rightLidar = RSIDE.readRangeSingleMillimeters();
		if (RSIDE.timeoutOccurred()) 
			rightLidar = 0;
		
	//
	/*//
	Variables Available: 
	mx,my,mz,rx,ry,rz
	frontLidar,rightLidar
	*///
	
	oldTime = timet;
	timet = millis();
	unsigned long dt = timet-oldTime;
	
	csvFirst(&csvLine, String(dt));
	csvAdd(&csvLine, String(frontLidar));
	csvAdd(&csvLine, String(rightLidar));
	csvAdd(&csvLine, String(rx));
	csvAdd(&csvLine, String(ry));
	csvAdd(&csvLine, String(rz));
	csvAdd(&csvLine, String(mx));
	csvAdd(&csvLine, String(my));
	csvAdd(&csvLine, String(mz));
	csvAdd(&csvLine, "T");

	//Transmit csvLine
	client.print(csvLine);


	
}


void move(int direction, int speed)
{
	switch(direction)
	{
		case forward: //forward
		{
			Left.write(0+speed);
			Right.write(180-speed);
			break;
		}
		case backward: //back
		{
			Left.write(180-speed);
			Right.write(0+speed);
			break;
		}
		case left: //left
		{
			Left.write(0+speed);
			Right.write(0+speed);
			break;
		}
		case right: //right
		{
			Left.write(180-speed);
			Right.write(180-speed);
			break;
		}
		default: //stay
		{
			Left.write(LSERVO_NULL);
			Right.write(RSERVO_NULL);
		}
	}
}
