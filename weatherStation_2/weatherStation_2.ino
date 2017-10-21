// Library to control the SHT1x sensor.
#include <SHT1x.h>

// Library to control timer interrupt.
#include <TimerOne.h>

// Libraries to control downtime and energy savings.
#include <avr/sleep.h>
#include <avr/power.h>

// Library to Watchdog timer.
#include <avr/wdt.h>

//Library to control the NRF24L01 Module
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


// Define the CE and CSN pins for the NRF24L01 module.
#define CE_PIN   9
#define CSN_PIN 10

// Library to control BMP180.
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Channel or 'pipe' Note: The "LL" at the end of the constant indicate that the type is "LongLong".
const uint64_t pipe = 0xE8E8F0F0E1LL; 

// Initialize Radio.
RF24 radio(CE_PIN,CSN_PIN);

// Define the data pin and the clock pin for the sht1x module.
#define dataPin 6
#define clockPin 8

#define NUMBER_OF_PARAMETERS 7      // Number of sensors/parameters to measure.
const int n=NUMBER_OF_PARAMETERS;
float sensorData[n];

// Initialise SHT1x.
SHT1x sht1x(dataPin, clockPin);

// Initialise.
Adafruit_BMP085 bmp;

// These constants, define values needed for the LDR readings and ADC
#define LDR_PIN                   0
#define MAX_ADC_READING           1023
#define ADC_REF_VOLTAGE           5.0
#define REF_RESISTANCE            5030  // measure this for best results
#define LUX_CALC_SCALAR           12518931
#define LUX_CALC_EXPONENT         -1.405

// Global variable declarations.
volatile int f_wdt = 1;
int counter = 0;
int packetCounter = 0;

// Global variables for rain meter
const int REED = 2;      // The reed switch outputs to digital pin 2
int val = 0;             // Current value of reed switch
int old_val = 0;         // Old value of reed switch
int REEDCOUNT = 0;       // The intial count is zero
volatile float depth_of_rain;

// counterHandler: Controls how long the controller should stay asleep
void counterHandler(){
    //Increment Counter. 
    counter++;
    // If it controls the time the controller should stay asleep
    // 1: For tests
    // 75: 10 minutes (75 * 8 = 600 seconds = 10 minutes)
  
    if(counter == 1) {
        // Reset when counter expires.
        counter = 0; 
        
        // Turn on Device.
        power_all_enable();
    
        // Turn on Radio.
        radio.powerUp();
    
        // Wait for the radio to start.
        delay(2);
    }
    else {
        // if time not fulfilled, continue sleep.
        enterSleep();
    } 
}
ISR(WDT_vect){
    // Stop WDT
    f_wdt = 1;
}

// setupWDT: Configure WDT
void setupWDT(){
    // Configure WDT to interrupt every 8 sec.  
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 1<<WDP0 | 1<<WDP3;
    WDTCSR |= _BV(WDIE);
}

// enterSleep: Turns off radio and keeps the controller in energy saving
void enterSleep(){
    // Initialize WDT
    f_wdt = 0;
  
    // Turn off Radio.
    radio.powerDown();
    
    // controller energy saving mode.
    sleep_enable();
    sleep_mode();
    
    // Wake the controller.
    sleep_disable();
    
    // Increase the interrupt counter.
    counterHandler();
}
// setupRadio: Configure Radio.
void setupRadio(){
    // Initialize radio.
    radio.begin();
    
    // Define the number of retires.
    radio.setRetries(15,15);
    
    // Define a radio channel for broadcast(0-127).
    radio.setChannel(30);
    
    // Set the bitrate. 
    radio.setDataRate(RF24_250KBPS);
    
    // Defines the amplifier level of the radio module (RF24_PA_MIN for testing, RF24_PA_HIGH for long distances)
    radio.setPALevel(RF24_PA_MIN);
    
    // Enable Dynamic Payloads.
    radio.enableDynamicPayloads();
}

// Lux meter
float LuxMeter(void) {
    int   ldrRawData;
    float resistorVoltage, ldrVoltage;
    float ldrResistance;
    float ldrLux;
    
    // Perform the analog to digital conversion  
    ldrRawData = analogRead(LDR_PIN);
    
    // RESISTOR VOLTAGE_CONVERSION
    // Convert the raw digital data back to the voltage that was measured on the analog pin
    resistorVoltage = (float)ldrRawData / MAX_ADC_READING * ADC_REF_VOLTAGE;

    // voltage across the LDR is the 5V supply minus the 5k resistor voltage
    ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;
    
    // LDR_RESISTANCE_CONVERSION
    // resistance that the LDR would have for that voltage  
    ldrResistance = ldrVoltage/resistorVoltage * REF_RESISTANCE;
    
    // LDR_LUX
    // Change the code below to the proper conversion from ldrResistance to
    // ldrLux
    ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);
    return ldrLux;
}

// Rain calculation in mm.
void RainMeter(){

    float volume_of_rain;
    float area_of_funnel;
    // float depth_of_rain;

    // read REED pin.
    val = digitalRead(REED);
//    Serial.println("reading reed pin");
    if ((val == LOW) && (old_val == HIGH)){
        REEDCOUNT = REEDCOUNT + 1;
        delay(10);
        old_val = val;
        Serial.print('i');
        Serial.println(REEDCOUNT);
    }
    else {
        old_val = val;
    }

    volume_of_rain = (5.94*REEDCOUNT)/1000;
    area_of_funnel = (3.14/4)*0.165*0.165;
    depth_of_rain = volume_of_rain/area_of_funnel;
    // return depth_of_rain;
}

void setup(){
    // Initialise Serial port
//    Serial.begin(57600);
    Serial.println("Starting");

    // initialize BMP and Check if connected.
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP sensor, check wiring!");
        while (1) {}
    }

    // configure REED pin as InPUT
    pinMode(REED,INPUT);
    // attachInterrupt(digitalPinToInterrupt(REED), RainMeter, FALLING);

    Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
    Timer1.attachInterrupt(RainMeter); // attach the service routine here
  

    // Disable brownout Detection for low power consumption.
    // sleep_bod_disable();


    // Set sleep mode.
    // set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
    // Set WDT.
    // setupWDT();
        
    // Set Radio.
    setupRadio();

}

void loop(){
  
    // initialise sensor data with initial zero's.
    sensorData[0] = 0;
    sensorData[1] = 0;
    sensorData[2] = 0;
    sensorData[3] = 0;
    sensorData[4] = 0;
    sensorData[5] = 0;
    sensorData[6] = 0;
        
    // Read the temperature and Humidity from SHT1x into sensorData.
    sensorData[0] = sht1x.readTemperatureC();
    sensorData[1] = sht1x.readHumidity();
    sensorData[2] = LuxMeter();
    sensorData[3] = bmp.readAltitude();
    sensorData[4] = bmp.readPressure();
    sensorData[5] = bmp.readSealevelPressure();
    sensorData[6] = depth_of_rain; 
    // delay(2000);

    // Control Errors.
    if (isnan(sensorData[0]) || isnan(sensorData[1])|| isnan(sensorData[2])|| isnan(sensorData[3])|| isnan(sensorData[4])|| isnan(sensorData[5])|| isnan(sensorData[6])) {
        Serial.println("Failed to read sensor!!");
        return;
    }
    
    // open a write channel for radio
    radio.openWritingPipe(pipe);
    
    // Write the data.
    radio.write(sensorData, sizeof(sensorData));

    // serially print the sensor data.
    Serial.println(sensorData[0]);
    Serial.println(sensorData[1]);
    Serial.println(sensorData[2]);
    Serial.println(sensorData[3]);
    Serial.println(sensorData[4]);
    Serial.println(sensorData[5]);
    Serial.println(sensorData[6]);

    // Enter sleep mode
    enterSleep();

}
