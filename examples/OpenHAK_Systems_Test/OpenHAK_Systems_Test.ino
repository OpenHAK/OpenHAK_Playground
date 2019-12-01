/*
  OpenHAK 3.0.0 Test Code
  This code targets a Simblee as OpenHAK hardware profile
  I2C Interface with MAX30101 Sp02 Sensor Module
  Also got a BOSH MEMS thing

      >>>> YOU MUST DEFINE THE BOARD YOU ARE USING!
      >>>> THIS CODE WILL COMPILE FOR THE BETA_TESTER HARDWARE AND THE BIO_HACK_BADGE HARDWARE
*/


#define BIO_VILLAGE_BADGE 1  // use for BioHacking Village Badge from DEFCON 27
// #define BETA_TESTER 1 // use for 2019 beta hardware
#define OLED_PRESENT 1  // use when there is an OLED shield present

#include <Wire.h>
#include "OpenHAK_Playground.h"
#ifdef OLED_PRESENT
  #include <OpenHAK_MicroOLED.h>
  MicroOLED oled(OLED_RESET, DC);    // reset pin, I2C address mask, from library
	boolean inverse = false;
#endif

//This line added by Chip 2016-09-28 to enable plotting by Arduino Serial Plotter
//const int PRINT_ONLY_FOR_PLOTTER = 1;  //Set this to zero to return normal verbose print() statements

unsigned int LED_blinkTimer;
unsigned int LED_fadeTimer;
unsigned int rainbowTimer;
int LED_blinkTime = 300;
int LED_fadeTime = 50;
int rainbowTime = 50;
int LEDpin[] = {RED,GRN,BLU};
int LEDvalue[3];
boolean rising[] = {true,true,true};
boolean falling;
int fadeValue = 200;
int LEDcounter = 0;
int lightInPlay;
int colorWheelDegree = 0;
boolean rainbow = false;

volatile boolean MAX_interrupt = false;
short interruptSetting;
short interruptFlags;
float Celcius;
float Fahrenheit;
byte sampleCounter = 0;
int REDvalue;
int IRvalue;
int GRNvalue;
byte sampleAve;
byte mode;
byte sampleRange;
byte sampleRate;
byte pulseWidth;
int LEDcurrent;
byte readPointer;
byte writePointer;
byte ovfCounter;
int redAmp = 50;
int irAmp = 50;
int grnAmp = 50;
boolean is_running = false;
float volts;

//  TESTING
unsigned int thisTestTime;
unsigned int thatTestTime;


#define DEBUG 1// set to 0 to disable
void setup(){

  Wire.beginOnPins(SCL_PIN,SDA_PIN);
  Serial.begin(230400);
  for(int i=0; i<3; i++){
    pinMode(LEDpin[i],OUTPUT); analogWrite(LEDpin[i],255); // Enable RGB and turn them off
  }
#ifdef OLED_PRESENT
    splashOLED(inverse);
#endif

  LED_blinkTimer = LED_fadeTimer = millis();

	pinMode(MAX_INT,INPUT);
	attachPinInterrupt(MAX_INT,MAX_ISR,LOW);
	serviceInterrupts();
/*
 * Initialize MAX heart rate sensor
 * (Sample Average, Mode, ADC Range, Sample Rate, Pulse Width, LED Current)
 */
   sampleAve = SMP_AVE_4;
   mode = SPO2_MODE;
   sampleRange = ADC_RGE_4096;
   sampleRate = SR_400;
   pulseWidth = PW_411;
   LEDcurrent = 25;
   MAX_init(sampleAve, mode, sampleRange, sampleRate, pulseWidth, LEDcurrent);

  if (DEBUG) {
    Serial.println("OpenHAK Systems Check");
    getBMI_chipID();    // should print 0xD1
    getMAXdeviceInfo(); // prints rev and device ID
    Serial.println("");
    printHelpToSerial();

  } else {
    //when configured for the Arduino Serial Plotter, start the system running right away
    enableMAX30102(true);
    thatTestTime = micros();
  }


}


void loop(){

  if(is_running){
#ifdef BETA_TESTER
    interruptFlags = MAX_readInterrupts();
    if(interruptFlags > 0){
      serveInterrupts(interruptFlags); // go see what woke us up, and do the work
      if(DEBUG){
        thisTestTime = micros();
        printTab(); printTab(); Serial.println(thisTestTime - thatTestTime);
        thatTestTime = thisTestTime;
      }
      if(sampleCounter == 0x00){  // rolls over to 0 at 200
        MAX30102_writeRegister(TEMP_CONFIG,0x01); // take temperature
      }
		}
#endif
#ifdef BIO_VILLAGE_BADGE
	if(MAX_interrupt){
		serviceInterrupts();
  }
#endif
	}else if(rainbow){
    rainbowLEDs();
  }else{
    analogWrite(RED,180);
		inverse = !inverse;
		splashOLED(inverse);
    delay(1000);
    analogWrite(RED,255);
    analogWrite(GRN,180);
		inverse = !inverse;
		splashOLED(inverse);
    delay(1000);
    analogWrite(GRN,255);
    analogWrite(BLU,180);
		inverse = !inverse;
		splashOLED(inverse);
    delay(1000);
    analogWrite(BLU,255);
  }
  eventSerial();
}



int MAX_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
  MAX_interrupt = true;
  return 0; // gotta return something, somehow...
}



byte getBatteryVoltage(){
  if(DEBUG){ Serial.println("Battery"); }
    int thisCount, lastCount;
    byte returnVal;
    for(int i=0; i<100; i++){
      lastCount = analogRead(V_SENSE);
      delay(10);
      thisCount = analogRead(V_SENSE);
      if(DEBUG){
        Serial.print(i); Serial.print("\t"); Serial.print(lastCount); Serial.print("\t"); Serial.println(thisCount);
      }
      if(thisCount >= lastCount){ break; }
      delay(10);
    }
    volts = float(thisCount) * (3.0 / 1023.0);
    volts *= 2.0;
    if(DEBUG){
      Serial.print(thisCount); Serial.print("\t"); Serial.println(volts,3);
    }
    returnVal = byte(volts / BATT_VOLT_CONST); // compress value for OTA
    return returnVal;
}
