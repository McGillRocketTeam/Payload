#include <arduinoFFT.h>
#include <SD.h>

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

File dataCollection;
File fftData; 
// the setup routine runs once when you press reset:
void setup()
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  while (!Serial);
  Serial.println("READY");
  Serial.begin(9600);

  // Sd card connection 
  while (!SD.begin(BUILTIN_SDCARD));
  Serial.println("SD Card initialized");

  // delete file if previously initialized
  SD.remove("dataCollection.csv");
  SD.remove("fftData.csv");
  dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);
  
  // from the test on saturday writing to two files was expensive, but feel free to try again. 
  fftData = SD.open("fftData.csv", FILE_WRITE | FILE_READ);
  dataCollection.println("time,amplitude X (1023),amplitude Y (1023), amplitude Z");
 // fftData.println("time, frqX, frqY, frqZ, ampX, ampY, ampZ");
}

void collectAnalog(int count)
{
  analogReadResolution(10);

  // Read count
  vRealX[count] = analogRead(A10);
  String xTimeStamp = String(millis());
  vRealY[count] = analogRead(A11);
  String yTimeStamp = String(millis());
  vRealZ[count] = analogRead(A12);
  String zTimeStamp = String(millis());

  // collecting data in csv file

  dataCollection.println(xTimeStamp + "," + vRealX[count] + "," + vRealY[count] + "," + vRealZ[count]);
  
  Serial.println(dataCollection.size());
  vImagX[count] = 0;
  vImagY[count] = 0;
  vImagZ[count] = 0;
}

// finding amplitude of major peak . 
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

// fft calculation of frequency of the Major Peak. 
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
// period of latency in microseconds between each data sample. 
int periodLength = 397;
int maxCount = 250000;
int counter2 = 0;
// the loop runs when the data is

// TODO : Acknowledge from CAN bus to modify the the following values.
bool isSampling = true;
bool isScrubReset = false; // Value to be added in code
bool isShutDown = false;

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
  if (isSampling && !isScrubReset && !isShutDown && counter2<maxCount)
  {

    // collecting analog data until frequency is achieved
    
    
    int beforeExec = micros();
    // collecting analog data
    collectAnalog(counter);
    // subtracting the time it takes to run "collectAnalog" from the period length to achieve exactly a "periodLength" delay (see definition above)
    int leftOverDelay = periodLength - (micros() - beforeExec);
    if (leftOverDelay > 0)
    {
      // delay by the amount of time left to add up the total time waited to "periodLength" in Microseconds delayMicroseconds is a built-in function.
      delayMicroseconds(leftOverDelay);
    }

    counter++;

    if (counter == samples)
    {
      // int t1 = micros();

      // collecting amplitude and frequency data from FFT.
      float frqX = calcFFT(vRealX, vImagX, samples);
      float ampX = findMaxInArr(vRealX) / 100000;
      float frqY = calcFFT(vRealY, vImagY, samples);
      float ampY = findMaxInArr(vRealY) / 100000;
      float frqZ = calcFFT(vRealZ, vImagZ, samples);
      float ampZ = findMaxInArr(vRealZ) / 100000;
      int tS = millis(); 

       dataCollection.println(",,,,"+String(tS) + "," + String(frqX) + "," + String(frqY) + "," + String(frqZ) + "," + String(ampX) +"," + String(ampY) + "," + String(ampZ));
      // DATA PRINTOUT TO MONITOR:  
      // Serial.println("frqX: " + String(frqX) + "Hz, frqY: " + String(frqY) + "Hz, frqZ: " + String(frqZ) +"Hz, ampX: " + String(ampX) +"Hz, ampY: " + String(ampY) + "Hz, ampZ: " + String(ampZ));


      Serial.println(millis() - t);
      t = millis();

      // TODO : send frequencies through CAN bus (frqX, frqY, frqZ)

      counter = 0;
    }
  }
  else
  {
 
       dataCollection.close();
    //  fftData.close();
      Serial.println(millis());
    if (isShutDown)
    {
      dataCollection.close();
      fftData.close();
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
