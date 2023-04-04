#include <arduinoFFT.h>
#include <SD.h>
#include <FlexCAN_T4.h>

#include "send.h"

arduinoFFT FFT = arduinoFFT(); // CREATE FFT object

const uint16_t samples = 2048; // This value MUST ALWAYS be a power of 2
const uint8_t amplitude = 1023;
const double samplingFrequency = 2500;
const uint16_t flushFrequency = 300; //time to wait between flushes

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
// y-axis ccollection
double vRealY[samples];
double vImagY[samples];
// z - axis collection
double vRealZ[samples];
double vImagZ[samples];
int led = 13;

File fileX;
File fileY;
File fileZ;
File dataCollection;

// the setup routine runs once when you press reset:
void setup()
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("READY");
  Serial.begin(9600);



  while (!SD.begin(BUILTIN_SDCARD))
    ;
  Serial.println("SD Card initialized");

  // delete file if previously initialized
  SD.remove("dataCollection.csv");
  dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);

  //==================TESTING SD===================
  //int current = millis();
  //bool runLoop = true;
  //while(runLoop){
  //// for(int i=0; i<10; i++){
  //int myTime = millis()-current;
  //  if (dataCollection)
  //  {
  //    Serial.println("Writing to the text file...");
  //      dataCollection.println(myTime);
  ////      if (i%2 == 0 )
  //      {dataCollection.flush();}
  //  } else
  //  {
  //    // if the file didn't open, report an error:
  //    Serial.println("error opening the text file!");
  //  }
  //  if (myTime >5000) runLoop = false;
  // }
  //  // re-open the text file for reading:
  //  dataCollection = SD.open("dataCollection.csv");
  //  if (dataCollection)
  //  {
  //    Serial.println("textFile.txt:");
  //
  //    // read all the text written on the file
  //    while (dataCollection.available())
  //    {
  //      Serial.write(dataCollection.read());
  //    }
  //  } else
  //  {
  //    // if the file didn't open, report an error:
  //    Serial.println("error opening the text file!");
  //  }
  //
  //===================END=========================



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
}

void collectAnalog(int count)
{
  analogReadResolution(10);

  // Read count
  vRealX[count] = analogRead(A0);
  String xTimeStamp = String(millis());
  vRealY[count] = analogRead(A1);
  String yTimeStamp = String(millis());
  vRealZ[count] = analogRead(A3);
  String zTimeStamp = String(millis());

  // collecting data in csv file

  dataCollection.println(xTimeStamp + "," + yTimeStamp + "," + zTimeStamp + "," + vRealX[count] + "," + vRealY[count] + "," + vRealZ[count]);
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

int counter = 0;
int t = millis();
int flushTime = millis(); // to keep track of when the last flush was
int periodLength = 397;
int maxCount = 250000;
int counter2 = 0;

bool isSampling = true;
bool isScrubReset = false;
bool isShutDown = false;

bool test = false;

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
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();

  //Receive messages from AV
  if (msg.id == 0x11 || msg.id == 0x12) {
    if (msg.buf[0] == 1) {
      isSampling = true;
    } else if (msg.buf[0] == 0) {
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
int startTime = millis(); // testing purposes

void loop()
{
  //    can2.events();
  if (isSampling && !isScrubReset && !isShutDown)
  {
    //====testing====
    dataCollection.println(millis() - startTime);

    if (millis() - startTime > 900) {
      test = true;
    }
    Serial.println(millis() - startTime );
    //==============

    if (millis() - flushTime > flushFrequency) {
      dataCollection.flush();
      flushTime = millis(); //record the time of the flush
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

      int t1 = micros();
      //==========Test Data=============
      //      float frqX = calcFFT(vRealX, vImagX, samples);
      //      float ampX = findMaxInArr(vRealX) / 100000;
      //      float frqY = calcFFT(vRealY, vImagY, samples);
      //      float ampY = findMaxInArr(vRealY) / 100000;
      //      float frqZ = calcFFT(vRealZ, vImagZ, samples);
      //      float ampZ = findMaxInArr(vRealZ) / 100000;


      //      const float frqX = 43.23;   //00 000010 10 11
      //      const float frqY = 752.23;  //101111 0000
      //      const float frqZ = 902.32;  //1110 000110
      //
      //      const float ampX = 3.2 ;  // 00000101 000000
      //      const float ampY = 2.1;  //01 1010010
      //      const float ampZ = 1.4;  //0 10001100
      //
      //      const int minutes = 0;
      //      const int seconds = 12;
      //      const int milliseconds = 0;

      float frqX = random(0, 1100);
      float frqY = random(0, 1100);
      float frqZ = random(0, 1100);

      float ampX = randomFloat(0, 5);
      float ampY = randomFloat(0, 5);
      float ampZ = randomFloat(0, 5);

      int minutes = random(0, 59);
      int seconds = random(0, 59);
      int milliseconds = random(0, 1000);
      //===========End Test Data=============

      // PRINT OUTPUT : Serial.println(String(frqX) + "," + String(frqY) + "," + String(frqZ));

      //      Serial.println(millis() - t);
      t = millis();

//      int minutes = getMinutes(millis() - startTime);
//      int seconds = getSeconds(millis() - startTime);
//      int milliseconds = getMillis(millis()startTime);

      /* Sending data on CANBus*/

      // Initialize the structure with relevant data
      struct Data dt = {frqX, frqY, frqZ, ampX, ampY, ampZ, minutes, seconds, milliseconds};

      // Declare 3 unions which will contain the messages to be sent
      union my_msg m1;
      union my_msg m2;
      union my_msg m3;

      buildMsg(&m1, &m2, &m3, dt); // Concatenate the data and format it to be sent in 10 bytes
      sendMsg(&m1, &m2, &m3); // Send the messages

      counter = 0;
    }

    if (test) {
      dataCollection = SD.open("dataCollection.csv");
      if (dataCollection)
      {
        Serial.println("textFile.txt:");

        // read all the text written on the file
        while (dataCollection.available())
        {
          Serial.write(dataCollection.read());
        }
      } else
      {
        // if the file didn't open, report an error:
        Serial.println("error opening the text file!");
      }
      exit(0);
    }

  }
  else
  {
    if (isShutDown)
    {
      dataCollection.close();
    }
    if (isScrubReset)
    {
      dataCollection.close();
      SD.remove("dataCollection.csv");
      dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);
      isScrubReset = false;
    }
  }
  counter2++;
}
