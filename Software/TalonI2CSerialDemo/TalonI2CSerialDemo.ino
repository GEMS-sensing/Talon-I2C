#include <Wire.h>
 // #include "Adafruit_MCP23008.h"
// #include <Adafruit_MCP23X17.h>
// #include <MCP2301x.h>
#include <Arduino.h>
#include <MCP23018.h>

// #define NUM_MCP2301x_ICs   1

const uint8_t TX = 1;
const uint8_t RX = 0;
const uint8_t FOut = 1;
const uint8_t Dir = 0;

//IO Expander pins
const uint8_t POS_DETECT = 12; 
const uint8_t SENSE_EN = 13;

//IO Sense pins
const uint8_t MUX_EN = 3;
const uint8_t MUX_SEL0 = 0;
const uint8_t MUX_SEL1 = 1;
const uint8_t MUX_SEL2 = 2;

const float VoltageDiv = 3; //Program voltage divider
const float CurrentDiv = 0.243902439; //82mOhm, 50V/V Amp

//Configured for pins on the Particle Boron
//const uint8_t EXT_EN = 8;
//const uint8_t SDA_CTRL = 2;

 const uint8_t I2C_EN = 13;

char ReadArray[25] = {0};

 // Adafruit_MCP23008 io;
// Adafruit_MCP23X17 io;
MCP23018 io(0x20);
// MCP2301x io;

// #define IOEXP_MODE  (IOCON_INTCC | IOCON_INTPOL | IOCON_ODR | IOCON_MIRROR)
// #define ADDRESS      (MCP2301X_CTRL_ID+7)
// #define LED_PIN      D4   // WeMOS D1 & D1 mini
// #define IN_PORT      0
// #define OUT_PORT     1

// #define RELAY_ON      LOW
// #define RELAY_OFF     HIGH
// #define ALL_OFF       ALL_LOW

// #define DIR 0x00
// #define GPIO 0x12
// #define LAT 0x14
// #define PU 0x0C
// #define ADR 0x20

/////////////// HAAR //////////////////////
#include <Dps368.h>
#include <Adafruit_SHT31.h>

Dps368 pressureSense = Dps368(); //Instantiate DPS368 Pressure sensor
Adafruit_SHT31 rhSense = Adafruit_SHT31(); //Instantiate the SHT31 RH/Temp sensor

const uint8_t dps368_oversampling = 7; //Define the oversampling amount for the DPS368 sensor
unsigned long period = 5000; //Number of ms to wait between sensor readings 

////////////// HEDORAH NDIR /////////////
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
SCD30 airSensor;

///////////// VOLTAGE SENSE /////////////
#include <SparkFun_PCA9536_Arduino_Library.h>
#include <MCP3421.h>

MCP3421 adc(0x6B); //Initialize MCP3421 with A3 address

PCA9536 ioSense;

void setup() 
{
	pinMode(I2C_EN, OUTPUT); 
	digitalWrite(I2C_EN, HIGH); //Turn on external port connection
	Serial.begin(115200);

	Wire.begin();
	SetDefaultPins();
	// SetPinMode();
	// SetPullups();
	// io.digitalWrite(6, HIGH); //Turn on 5V
	// io.digitalWrite(7, HIGH); //Turn on 12V
	// io.digitalWrite(2, LOW); //Enable Data 1
	// io.digitalWrite(3, LOW); //Enable Data 2
	// io.digitalWrite(4, LOW); //Enable Data 3

	
	delay(1000);
	Serial.println("Begin I2C Demo...");
}

void loop() 
{
	// io.pinMode(0, TX, LOW);
	// io.pinMode(0, FOut, LOW);
	// delay(1000);
	// io.pinMode(0, FOut, LOW);
	// io.pinMode(0, TX, HIGH);
	// delay(1000);
	// io.pinMode(0, FOut, HIGH);
	// io.pinMode(0, TX, LOW);
	// delay(1000);
	// io.pinMode(0, FOut, HIGH);
	// io.pinMode(0, TX, HIGH);
	// delay(1000);
	static int ReadLength = 0;
  	String ReadString;
	if(Serial.available() > 0) {
    char Input = Serial.read();

    // Increment counter while waiting for carrage return or newline
    // if(Input != 13 || Input != 10) {
    if(Input != '#' && Input != '!') { //Wait for SDI12 line end or control line end 
      ReadArray[ReadLength] = Input;
      ReadLength++;
    }

    // if(Input == 13 || Input == 10) { // carriage or newline
    if(Input == '!') { //SDI12 line ending
		ReadArray[ReadLength] = Input; //Append final character
		ReadLength++;

		ReadString = String(ReadArray);
		ReadString.trim();
		memset(ReadArray, 0, sizeof(ReadArray));
		ReadLength = 0;
		int Pin = (ReadString.substring(0,1)).toInt(); //Grab the pin to operate on

		Serial.print(">");
		Serial.println(ReadString); //Echo back to serial monitor

		switch(Pin) { //Call the interface function for the appropriate sensor to test
			case 0:
				VoltageSense();
				break;
			case 1:
				Sensor1();
				break;
			case 2:
				Sensor2();
				break;
			case 3:
				Sensor3();
				break;
			case 4:
				Sensor4();
				break;
			default:
				Serial.println(">You didn't say the magic word!"); 
		}
		
	}
	//Format - ppCs#
	//pp - two digit pin number (base 10)
	//C - command ('M' - Mode, 'R' - Read, 'W' - Write)
	//s - state (0 - Output/Off, 1 - Input/On), required only for write and mode operations 
	if(Input == '#') { //Control line ending
		ReadString = String(ReadArray);
		ReadString.trim();
		memset(ReadArray, 0, sizeof(ReadArray));
		ReadLength = 0;
		int Pin = (ReadString.substring(0,2)).toInt(); //Grab the pin to operate on
		String Operation = ReadString.substring(2,3); //Grab the middle char
		int State = (ReadString.substring(3,4)).toInt(); //Grab the state to set
		int Result = 0; //Used to grab result of either setting of pin mode or response from a read

		digitalWrite(I2C_EN, LOW); //Turn off external port connection
		if(Operation.equals("M")) { //if call is for pinmode setting
			Result = io.pinMode(Pin, !State); //Use inverse of state to corespond with 1 = input, 0 = output
		}
		
		if(Operation.equals("R")) { //If call is for digital read
			Result = io.digitalRead(Pin);
		}

		if(Operation.equals("W")) { //If call is for digital write
			Result = io.digitalWrite(Pin, State);
		}
		digitalWrite(I2C_EN, HIGH); //Turn on external port connection
		// while(Serial.peek() != '\n'); //Wait for new command
		// String NewCommand = Serial.readStringUntil('\n');
		Serial.print(">");
		Serial.println(ReadString); //Echo back to serial monitor
		Serial.print(">");
		Serial.println(Result); //Return various result 
	}
	// GetAddress();
	// delay(5000);

	}
}

void VoltageSense() //Voltage sense (AKA Sensor0)
{
	Serial.print(">Voltage Sense:\n");
	digitalWrite(I2C_EN, LOW); //Turn off external I2C
	io.digitalWrite(SENSE_EN, HIGH); //Make sure sense power is turned on
	ioSense.begin(); //Initalize voltage sensor IO expander
	for(int i = 0; i < 4; i++) { //Set all pins to output
		ioSense.pinMode(i, OUTPUT); 
	}
	ioSense.digitalWrite(MUX_EN, HIGH); //Turn MUX on 
	int SenseError = adc.Begin(); //Initialize ADC 
	if(SenseError == 0) { //Only proceed if ADC connects correctly
		adc.SetResolution(18); //Set to max resolution (we paid for it right?) 

		ioSense.digitalWrite(MUX_SEL2, LOW); //Read voltages
		for(int i = 0; i < 4; i++){ //Increment through 4 voltages
			ioSense.digitalWrite(MUX_SEL0, i & 0b01); //Set with lower bit
			ioSense.digitalWrite(MUX_SEL1, (i & 0b10) >> 1); //Set with high bit
			delay(1); //Wait for voltage to stabilize
	  		Serial.print("\tPort");
	  		Serial.print(i);
	  		Serial.print(":");
	  		Serial.print(adc.GetVoltage(true)*VoltageDiv, 6); //Print high resolution voltage
	  		Serial.print(" V\n");  
		}
		ioSense.digitalWrite(MUX_SEL2, HIGH); //Read currents
		for(int i = 0; i < 4; i++){ //Increment through 4 voltages
			ioSense.digitalWrite(MUX_SEL0, i & 0b01); //Set with lower bit
			ioSense.digitalWrite(MUX_SEL1, (i & 0b10) >> 1); //Set with high bit
			delay(1); //Wait for voltage to stabilize
	  		Serial.print("\tPort");
	  		Serial.print(i);
	  		Serial.print(":");
	  		Serial.print(adc.GetVoltage(true)*CurrentDiv*1000, 6); //Print high resolution current measure in mA
	  		Serial.print(" mA\n");  
		}
	}
	else Serial.print("\tFAIL!\n");
	digitalWrite(I2C_EN, HIGH); //Turn on external I2C
}


void Sensor1() 
{
	// Serial.println(">Spector1");
	/////////////// SETUP /////////////////////
	uint8_t error_dps368 = 0; //DEBUG! FIX! Change library to return stuff properly
	pressureSense.begin(Wire); //Initialize DPS368 sensor //DEBUG! FIX! Change library to return stuff properly
	uint8_t error_sht31 = rhSense.begin(0x44); //Initialize SHT31 sensor
	
	Serial.println(">Haar Status:"); //Print resulting sensor status 
	Serial.print("\tDPS368: "); 
	if(error_dps368 == 0) Serial.print("PASS\n"); //Print error grabbed from sensor
	else Serial.print("FAIL!\n"); //Fail if result is not 0
	Serial.print("\tSHT31: ");
	if(error_sht31 == 1) Serial.print("PASS\n"); //Print error grabbed from sensor
	else Serial.print("FAIL!\n"); //Fail if result is 0

	///////////////// RUN ////////////////////
	float temp_dps368; //Storage for temp measurement from DPS368 
  	float pressure; //Storage for pressure measurment from DPS368
  	

	int16_t error1 = pressureSense.measureTempOnce(temp_dps368, dps368_oversampling); //Grab new temp values [°C]
	int16_t error2 = pressureSense.measurePressureOnce(pressure, dps368_oversampling); //Grab new pressure values [Pa]
	pressure = pressure/100.0f; //Convert from Pa to mBar, because Pa is a stupid unit, and hPa is more stupid. 

	float humidity = rhSense.readHumidity(); //Grab new humidity values [%]
	float temp_sht31 = rhSense.readTemperature(); //Grab new temp values [°C]

	Serial.print("\tTemp (SHT31): "); Serial.print(temp_sht31, 4); Serial.print(" °C\t"); //Print temp to 4 decimal places
	Serial.print("Temp (DPS368): "); Serial.print(temp_dps368, 2); Serial.print(" °C\t"); //Print temp to 2 decimal places
	Serial.print("Pressure: "); Serial.print(pressure, 2); Serial.print(" mBar\t"); //Print pressure to 2 decimal places
	Serial.print("Humidity: "); Serial.print(humidity, 2); Serial.print(" %\n"); //Print humidity to 2 decimal places
}

void Sensor2() 
{
	// Serial.println(">Spector2");
	Wire.begin();

	Serial.println(">Hedorah Status:");
	Serial.print("\tSCD30: ");
	if (airSensor.begin() == false)
	{
		Serial.print("FAIL!\n");
	}
	else {
		Serial.print("PASS\n");
		Serial.print("\t...");
		unsigned long LocalTime = millis();
		while(millis() - LocalTime < 10000){
			if (airSensor.dataAvailable())
			{
				Serial.print("\tCO2: "); //Print sensor CO2 reading
				Serial.print(airSensor.getCO2());
				Serial.print(" PPM\t");

				Serial.print("Temp:"); //Print sensor temp
				Serial.print(airSensor.getTemperature(), 1);
				Serial.print(" °C\t");

				Serial.print("RH:"); //Print sensor RH
				Serial.print(airSensor.getHumidity(), 1);
				Serial.print(" %\n");
				break; //Exit while loop
			}
		}
		if(millis() - LocalTime > 10000) Serial.print("\tERROR: Timeout\n"); //Report timeout error if time has elapsed on exit
	}

}

void Sensor3() 
{

}

void Sensor4() 
{

}

void SetDefaultPins()
{
	digitalWrite(I2C_EN, LOW); //Turn off external I2C
	for(int i = 0; i < 8; i++) { //Enable all power, then all data
		io.pinMode(i, OUTPUT);
		io.digitalWrite(i, HIGH);
	} 

	for(int i = 8; i < 12; i++) { //Set FAULT lines as input pullups
		io.pinMode(i, INPUT_PULLUP);
	}

	io.pinMode(POS_DETECT, INPUT); //Set position detect as normal pullup (has external pullup)
	io.pinMode(SENSE_EN, OUTPUT); //Set sense control as output
	io.digitalWrite(SENSE_EN, HIGH); //Default sense to on 
	digitalWrite(I2C_EN, HIGH); //Turn on external I2C
}



