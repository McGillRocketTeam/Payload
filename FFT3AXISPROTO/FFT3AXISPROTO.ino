#include <arduinoFFT.h>
#include <SD.h>
#include <FlexCAN_T4.h>

#include "send.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

arduinoFFT FFT = arduinoFFT(); // CREATE FFT object

const uint16_t samples = 2048; // This value MUST ALWAYS be a power of 2
const uint8_t amplitude = 1023;
const double samplingFrequency = 2500;

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

  can1.begin();
  can1.setBaudRate(250000); //todo check baudrate
  
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
}

void collectAnalog(int count)
{
  // Serial.println("Foo Called");
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

int counter = 0;
int t = millis();
int periodLength = 397;
int maxCount = 250000;
int counter2 = 0;
// the loop runs when the data is

// TODO : Acknowledge from CAN bus to modify the the following values.
bool isSampling = false;
bool isScrubReset = false; // Value to be added in code
bool isShutDown = false;

// Structure containing all the data needing to be send
struct Data {
  float frqX;
  float frqY;
  float frqZ; 
  
  float ampX;
  float ampY;
  float ampZ;
  
  int minutes;
  int seconds;
  int milliseconds;
};

/** 
 *  Trick to split the int into bytes
 *  In a union, all members share the same memory location. So if a float is stored in 'f', and the 
 *  value of the array called 'bytes' is examined, each element of the array will contain one of the
 *  bytes of the 32 bit number encoding the float. 
 */
union my_msg {
  uint32_t msg;
  uint8_t bytes[4];
};

/**
 * this is the main sampling loop.
 * the loop is constatnly sampling unless the controls above are false.
 * If the acknowledgement needs to be triggered it can be added within the loop.
 * The loop will only sample if isSampling is on (when CAN bus sends the sampling mode signal)
 * It is on low power mode by default.
 * If CAN bus sends a scrub reset signal, we need switch isScrubReset to (true), the program
 * will then perform the scrub reset and put isScrubReset back to (false) default value
 * CAN bus MUST send a shutdown message before cutting power, which should switch isShutDown to true
 */
void loop()
{

  // TODO: POSSIBLE CAN bus acknowledgement of signals point.
  if (isSampling && !isScrubReset && !isShutDown)
  {

    // collecting analog data until frequency is achieved
    int beforeExec = micros();
    collectAnalog(counter);
    int leftOverDelay = periodLength - (micros() - beforeExec);
    if (leftOverDelay > 0)
    {
      delayMicroseconds(leftOverDelay);
    }

    counter++;

    if (counter == samples)
    {
      // int t1 = micros();

      float frqX = calcFFT(vRealX, vImagX, samples);
      float ampX = findMaxInArr(vRealX) / 100000;
      float frqY = calcFFT(vRealY, vImagY, samples);
      float ampY = findMaxInArr(vRealY) / 100000;
      float frqZ = calcFFT(vRealZ, vImagZ, samples);
      float ampZ = findMaxInArr(vRealZ) / 100000;

      // PRINT OUTPUT : Serial.println(String(frqX) + "," + String(frqY) + "," + String(frqZ));

      Serial.println(millis() - t);
      t = millis();

      /* Sending data on CANBus*/

      // Initialize the structure with relevant data
      struct Data dt = {frqX, frqY, frqZ, ampX, ampY, ampZ, 10, 11, 12}; //todo get time

      // Declare 3 unions which will contain the messages to be sent
      union my_msg m1;
      union my_msg m2;
      union my_msg m3;
      
      buildMsg(&m1, &m2, &m3, dt); // Concatenate the data and format it to be sent in 10 bytes
      sendMsg(&m1, &m2, &m3); // Send the messages
      
      counter = 0;
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
