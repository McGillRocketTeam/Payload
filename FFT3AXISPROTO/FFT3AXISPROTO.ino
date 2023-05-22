#include <arduinoFFT.h>
#include <SD.h>
#include <FlexCAN_T4.h>
#include <string.h>
#include "send.h"

arduinoFFT FFT = arduinoFFT(); // CREATE FFT object

const uint16_t samples = 8192; // This value MUST ALWAYS be a power of 2. the FFT takes 4x5ms=20 ms to compute
int periodLength = 100; //delay between each time we collect analog data (in microseconds). Needs to be changed when samplingFrequency changes 
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
double adjFactor = 470/4.4;
double vRealX[samples];
double vImagX[samples];
// y-axis ccollection
double vRealY[samples];
double vImagY[samples];
// z - axis collection
double vRealZ[samples];
double vImagZ[samples];

//CAN BUS/ Payload State params
volatile bool isSampling = false;
volatile bool isScrubReset = false;
volatile bool isShutDown = false;
uint32_t offsetTime=0; //offset time which will be used in scrub to reset time to 0


uint32_t getTime() { //offset time gets update each time AV sends a scrub CAN message. Solution to not being able to reset millis()
  return (millis()-offsetTime);
}


File dataCollection;
bool serialTest=true;
//LED params. By default the params on 5 next line are for no LED blinking
int led = 13;
int blinkState = 0;
bool blinkingLEDtoMonitorState=false; //if true we are monitoring the state of the payload via onboard LED blinking
uint32_t t = getTime();
uint32_t t_wait = 0;
uint32_t t_max = 4294967295;
bool initialWait = false;
//FFT param
int counter = 0;
//Flush param
uint32_t flushTime = getTime(); // to k-eep track of when the last flush was


// the setup routine runs once when you press reset:
void setup()
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  //while (!Serial) //remove for flight
  //  ;
  Serial.println("READY");
  Serial.begin(9600);



  while (!SD.begin(BUILTIN_SDCARD));
  if (serialTest) {Serial.println("SD Card initialized");};

  // delete file if previously initialized
  SD.remove("dataCollection.csv");
  dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);
  dataCollection.println("time,amplitude X (1023),amplitude Y (1023),amplitude Z,timeFFT,frqX,frqY,frqZ,ampX,ampY,ampZ");


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

  //LED
  t = getTime(); 
  if (blinkingLEDtoMonitorState) {  
    unsigned long waitTime = 20000;
    unsigned long maxTime = 30000;
    t_wait = getTime() + waitTime;
    t_max = getTime() + maxTime + waitTime;
    initialWait = true;
  }

  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);

  
  
}


struct dateTime { 
  uint8_t minutes;
  uint8_t seconds;
  uint32_t milliseconds;
};


struct dateTime convertmillis (uint32_t milli) {
  //3600000 milliseconds in an hour
  uint8_t hr = milli / 3600000;
  milli = milli - 3600000 * hr;
  //60000 milliseconds in a minute
  uint8_t mins = milli / 60000;
  milli = milli - 60000 * mins;

  //1000 milliseconds in a second
  uint8_t sec = milli / 1000;
  milli = milli - 1000 * sec;

  return (struct dateTime) {mins, sec, milli}; 
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

//for testing purposes
float randomFloat(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}



 


// Receive message from FC
void canSniff(const CAN_message_t &msg) {
  if (serialTest) {
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");  
    for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); 
    Serial.print(" ");
    } 
    Serial.println();
  }
  //Receive messages from AV
  if (msg.id == 0x11 || msg.id == 0x12) {
    if (msg.buf[0] == 1) {
      isSampling = true;
    } else if (msg.buf[0] == 0) {
      //before sswitching isSampling to false, we need to send an ack to AV to so they see isSampling=0 and for data to be all 0's
      union my_msg p1; //this union creates an array of 0's
      union my_msg p2;
      union my_msg p3;
      struct Data ack = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //before sswitching isSampling to false, we need to send an ack to AV to so they see isSampling=0 and for data to be all 0's
      buildMsg(&p1, &p2, &p3, ack); // Concatenate the data and format it to be sent in 10 bytes
      sendMsg(&p1, &p2, &p3); // Send the messages
      isSampling = false;
    }
  }

  if (msg.id == 0x17 || msg.id == 0x18) {
    isShutDown = true;
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
  if (isSampling && !isScrubReset && !isShutDown && t < t_max && !initialWait)
  {

    if (getTime() - flushTime > flushFrequency) {
      dataCollection.flush();
      flushTime = getTime(); //record the time of the flush
    }


    // collecting analog data until frequency is achieved
    int beforeExec = micros();
    collectAnalog(counter);
    int leftOverDelay = periodLength - (micros() - beforeExec);
    if (leftOverDelay > 0)
    {
      delayMicroseconds(leftOverDelay);
    }

    counter++;

    //                    Serial.println(counter);
    //                    Serial.println(samples);

    if (counter == samples)
    {

      //==========Test Data=============
      float frqX = calcFFT(vRealX, vImagX, samples);
      float ampX = PI*vRealX[(int) (frqX*samples/samplingFrequency)]/(samples*adjFactor);
      float frqY = calcFFT(vRealY, vImagY, samples);
      float ampY = PI*vRealY[(int) (frqY*samples/samplingFrequency)]/(samples*adjFactor);
      float frqZ = calcFFT(vRealZ, vImagZ, samples);
      float ampZ = PI*vRealZ[(int) (frqZ*samples/samplingFrequency)]/(samples*adjFactor);
      uint32_t tS = getTime();
      dataCollection.println(",,,,"+String(tS) + "," + String(frqX) + "," + String(frqY) + "," + String(frqZ) + "," + String(ampX) +"," + String(ampY) + "," + String(ampZ));


//            const float frqX = 43.23;//00 000010 1011
//            const float frqY = 752.23;  //101111 0000
//            const float frqZ = 902.32;  //1110 000110
//      
//            const float ampX = 3.2 ;  // 00000101 000000
//            const float ampY = 2.1;  //01 1010010
//            const float ampZ = 1.4;  //0 10001100
      
//            const int minutes = 0;
//            const int seconds = 12;
//            const int milliseconds = 45;

//            const float frqX = 4243.23;//00 000010 1011
//            const float frqY = 4952.23;  //101111 0000
//            const float frqZ = 5102.32;  //1110 000110
//      
//            const float ampX = 4.2 ;  // 00000101 000000
//            const float ampY = 24.1;  //01 1010010
//            const float ampZ = 14.4;  //0 10001100
      


      /*float frqX = random(0, 1100);
      float frqY = random(0, 1100);
      float frqZ = random(0, 1100);

      float ampX = randomFloat(0, 5);
      float ampY = randomFloat(0, 5);
      float ampZ = randomFloat(0, 5);

      int minutes = random(0, 59);
      int seconds = random(0, 59);
      int milliseconds = random(0, 1000);*/
      //===========End Test Data=============
          
      
      
      /* Sending data on CANBus*/
      uint32_t t1 = getTime();
      struct dateTime convertedTime = convertmillis(t1);
      // Initialize the structure with relevant data
      struct Data dt = {frqX, frqY, frqZ, ampX, ampY, ampZ, isSampling, convertedTime.minutes, convertedTime.seconds, convertedTime.milliseconds};

      // Declare 3 unions which will contain the messages to be sent
      union my_msg m1;
      union my_msg m2;
      union my_msg m3;

      buildMsg(&m1, &m2, &m3, dt); // Concatenate the data and format it to be sent in 10 bytes
      sendMsg(&m1, &m2, &m3); // Send the messages
      counter = 0;

      if (blinkState == 0 && blinkingLEDtoMonitorState){
        digitalWrite(led, HIGH);
        blinkState = 1;
      }
      else if (blinkState == 1 && blinkingLEDtoMonitorState){
        digitalWrite(led, LOW);
        blinkState = 0;
      }
    }
      t = getTime();
    }
    else if (initialWait && blinkingLEDtoMonitorState) {
      digitalWrite(led, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      delay(5000);
      t = getTime();
      if(t > t_wait){
      initialWait = false;
      }
    }  
    else
    {
      t = getTime();
      if (t > t_max && blinkingLEDtoMonitorState ){
        digitalWrite(led, HIGH);
        delay(100);
        digitalWrite(led, LOW);
        delay(100);
        //dont you dare change the value of isSmapling or close dataCollection
      }
      if (isShutDown)
      {
        dataCollection.close();
      }
      if (isScrubReset)
      {
        dataCollection.close();
        SD.remove("dataCollection.csv");
        dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);
        offsetTime=millis();
        t = getTime();
        isScrubReset = false;
      }
    }

}
