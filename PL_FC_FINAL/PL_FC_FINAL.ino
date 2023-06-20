#include <arduinoFFT.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include <string.h>
#include "send.h"

arduinoFFT FFT = arduinoFFT(); // CREATE FFT object

const uint16_t samples = 8192; // This value MUST ALWAYS be a power of 2. the FFT takes 4x5ms=20 ms to compute
unsigned long periodLength = 100; //delay between each time we collect analog data (in microseconds). Needs to be changed when samplingFrequency changes 
const double samplingFrequency = 10000;
const uint16_t flushFrequency = 3000; //time to wait between flushes

/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// x-axis collection
double vRealX[samples];
double vImagX[samples];
double GainX=4.34;
double adjFactorX = 470*0.56*GainX;
// y-axis ccollection
double vRealY[samples];
double vImagY[samples];
double GainY=1.96;
double adjFactorY = 470*0.56*GainY;
// z - axis collection
double vRealZ[samples];
double vImagZ[samples];
double GainZ=1;
double adjFactorZ = 470*0.56*GainZ;

//CAN BUS/ Payload State params
volatile bool isSampling = true;
volatile bool isScrubReset = false;
volatile bool isShutDown = false;

uint32_t getTime() { //offset time gets update each time AV sends a scrub CAN message. Solution to not being able to reset millis()
  return millis();
}



File dataCollection;
char fName[30];
bool overwrite=false;
//LED params. By default the params on 5 next line are for no LED blinking
int led = 13;

uint32_t t = getTime();
unsigned long lastExec = micros();
unsigned long delaySinceSample = 0;
uint32_t t_wait = 0;
uint32_t t_max = 18000000;
//FFT param
int counter = 0;
//Flush param
uint32_t flushTime = getTime(); // to k-eep track of when the last flush was

extern "C" uint32_t set_arm_clock(uint32_t frequency);

void setup()
{
  set_arm_clock(600'000'000);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  //while (!Serial) //remove for flight
  //  ;
  Serial.println("READY");
  Serial.begin(9600);

  while (!SD.begin(BUILTIN_SDCARD));
  Serial.println("SD Card initialized");

  // delete file if previously initialized
  if (overwrite) {
    SD.remove("dataCollection0.csv");
    dataCollection = SD.open("dataCollection0.csv", FILE_WRITE | FILE_READ);
    dataCollection.println("time,amplitude X (1023),amplitude Y (1023),amplitude Z,timeFFT,frqX,frqY,frqZ,ampX,ampY,ampZ");
    sprintf(fName, "dataCollection%d.csv",0);
  }
  else {
    bool fileSpotFound = false;
    
    for(int i = 0; i<100;i++){
      sprintf(fName, "dataCollection%d.csv",i);
      if(!SD.exists(fName)){
        dataCollection = SD.open(fName, FILE_WRITE | FILE_READ);
        dataCollection.println("time,amplitude X (1023),amplitude Y (1023),amplitude Z,timeFFT,frqX,frqY,frqZ,ampX,ampY,ampZ");
        fileSpotFound = true;
        break;
      }
    }
    if (!fileSpotFound) {
       SD.remove("dataCollection0.csv");
       dataCollection = SD.open("dataCollection0.csv", FILE_WRITE | FILE_READ);
       dataCollection.println("time,amplitude X (1023),amplitude Y (1023),amplitude Z,timeFFT,frqX,frqY,frqZ,ampX,ampY,ampZ");
       sprintf(fName, "dataCollection%d.csv",0);
    }
    
  } //end of if overwrite
  dataCollection.flush();

  //Setting up CAN Bus
  can2.begin();
  can2.setBaudRate(100000);
  can2.setMaxMB(16);
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(canSniff); // interrupts and goes into the canSniff function when a message is received
  can2.mailboxStatus();

  //Setting up filters for CAN bus
  can2.setFIFOFilter(REJECT_ALL); // Block data before setting filter
  can2.setFIFOFilter(1, 0x11, STD); // Only receive messages with ID 0x11
  can2.setFIFOFilter(2, 0x12, STD);
  can2.setFIFOFilter(3, 0x17, STD);
  can2.setFIFOFilter(4, 0x18, STD);
  can2.setFIFOFilter(5, 0x5, STD);
  can2.setFIFOFilter(6, 0x6, STD);

  t = getTime(); 

  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
}

void collectAnalog(int count)
{
  analogReadResolution(10);
  // Read count
  vRealX[count] = analogRead(A10);
  String TimeStamp = String(getTime());
  vRealY[count] = analogRead(A11);
  vRealZ[count] = analogRead(A12);

  // collecting data in csv file
  dataCollection.println(TimeStamp + "," + vRealX[count] + "," + vRealY[count] + "," + vRealZ[count]);
  vImagX[count] = 0;
  vImagY[count] = 0;
  vImagZ[count] = 0;
}

float findMaxInArr(double arr[])
{
  double m = 0;
  for (int i = 0; i < samples; i++)
  {
    if (arr[i] > m)
    {
      m = arr[i];
    }
  }
  return m;
}

float calcFFT(double vReal[], double vImag[], uint16_t samples)
{
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);                 /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples);                   /* Compute magnitudes */
  float x;
  x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  return x;
}

// Receive message from FC
void canSniff(const CAN_message_t &msg) {
  
  Serial.print("MB: "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");  
  for ( uint8_t i = 0; i < msg.len; i++ ) {
  Serial.print(msg.buf[msg.len-1-i], HEX); 
  Serial.print(" ");
  } 
  Serial.println();
  
  //Receive messages from AV
  if (msg.id == 0x11 || msg.id == 0x12) {
    if (msg.buf[0] == 1) {
      isSampling = true;
      if(!dataCollection) {
        dataCollection = SD.open(fName, FILE_WRITE | FILE_READ);
      }
    } else if (msg.buf[0] == 0) {
      //before sswitching isSampling to false, we need to send an ack to AV to so they see isSampling=0 and for data to be all 0's
      union my_msg p1; //this union creates an array of 0's
      union my_msg p2;
      struct Data ack = {0, 0, 0, 0, 0, 0, 0, 0}; //before sswitching isSampling to false, we need to send an ack to AV to so they see isSampling=0 and for data to be all 0's
      buildMsg(&p1, &p2, ack); // Concatenate the data and format it to be sent in 10 bytes
      sendMsg(&p1, &p2); // Send the messages
      isSampling = false;
      if(dataCollection) {
        dataCollection.close();
      }
      
    }
  }

  if (msg.id == 0x17 || msg.id == 0x18) {
    isShutDown = true;
    if(dataCollection) {
        dataCollection.close();
    }
  }

  if (msg.id == 0x5 || msg.id == 0x6) {
    isScrubReset = true;
  }
}

/**
   this is the main sampling loop.
   the loop is constatnly sampling unless the controls above are false.
   If the acknowledgement needs to be triggered it can be added within the loop.
   The loop will only sample if isSampling is on (when CAN bus sends the sampling mode signal)
   It is on low power mode by default.
   If CAN bus sends a scrub reset signal, we need switch isScrubReset to (true), the program
   will then perform the scrub reset and put isScrubReset back to (false) default value
   CAN bus MUST send a shutdown message before cutting power, which should switch isShutDown to true
*/

void loop()
{
  can2.events();
  t = getTime();

  if (isSampling && !isScrubReset && !isShutDown && t < t_max) {
    if (getTime() - flushTime > flushFrequency) {
      dataCollection.flush();
      char flushMsg[30];
      flushTime = getTime(); //record the time of the flush
      sprintf(flushMsg, "Flush took %lu ms", (flushTime - t));
      Serial.println(flushMsg);
    }

    // collecting analog data until frequency is achieved
    
    delaySinceSample = (micros() - lastExec);
    //Serial.println(delaySinceSample);
    if (delaySinceSample < periodLength)
    {
      delayMicroseconds(periodLength-delaySinceSample);
    }
    //Serial.println(delaySinceSample);
    lastExec = micros();
    collectAnalog(counter);
    counter++;

    if (counter == samples)
    {
      float frqX = calcFFT(vRealX, vImagX, samples);
      float ampX = PI*vRealX[(int) (frqX*samples/samplingFrequency)]/(samples*adjFactorX);
      float frqY = calcFFT(vRealY, vImagY, samples);
      float ampY = PI*vRealY[(int) (frqY*samples/samplingFrequency)]/(samples*adjFactorY);
      float frqZ = calcFFT(vRealZ, vImagZ, samples);
      float ampZ = PI*vRealZ[(int) (frqZ*samples/samplingFrequency)]/(samples*adjFactorZ);
      uint32_t tS = getTime();
      dataCollection.println(",,,,"+String(tS) + "," + String(frqX) + "," + String(frqY) + "," + String(frqZ) + "," + String(ampX,4) +"," + String(ampY,4) + "," + String(ampZ,4));

      /* Sending data on CANBus*/
      uint32_t t1 = getTime();
      //struct dateTime convertedTime = convertmillis(t1);
      uint32_t seconds_forCAN = (uint32_t) roundf(t1/1000.0);
      // Initialize the structure with relevant data
      struct Data dt = {frqX, frqY, frqZ, ampX, ampY, ampZ, isSampling, seconds_forCAN};
      //struct Data dt = {4001.2, 3506.7, 2222.2, 257.5, 112.1, 497.1, 1, 30767};

      // Declare 3 unions which will contain the messages to be sent
      union my_msg m1;
      union my_msg m2;

      buildMsg(&m1, &m2, dt); // Concatenate the data and format it to be sent in 10 bytes
      sendMsg(&m1, &m2); // Send the messages
      counter = 0;

    }
  } // end of sampling mode
  else if (isShutDown){
    delay(100);
  }
  else if (isScrubReset){
    dataCollection.close();
    delay(100);
    doReboot();
  }
  else{
    delay(100);
	  if(dataCollection) {
        dataCollection.close();
    }
  }
  
}

void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}
