#include <arduinoFFT.h>
#include <SD.h>

arduinoFFT FFT = arduinoFFT(); // CREATE FFT object

const uint16_t samples = 2048; //This value MUST ALWAYS be a power of 2
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

//x-axis collection 
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
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600); 
  while(!Serial);
  Serial.println("READY");
    Serial.begin(9600);
  
  while (!SD.begin(BUILTIN_SDCARD));
  Serial.println("SD Card initialized");

  // delete file if previously initialized
  SD.remove("dataCollection.csv");
  dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);
}

void collectAnalog (int count){
  //Serial.println("Foo Called");
  analogReadResolution(10);  
  
 // Read count 
  vRealX[count] = analogRead(A0);
  String xTimeStamp = String(millis());
  vRealY[count] = analogRead(A1);
  String yTimeStamp = String(millis());
  vRealZ[count] = analogRead(A3);
  String zTimeStamp = String(millis());

  // collecting data in csv file 
  
  dataCollection.println(xTimeStamp + "," + yTimeStamp  + "," + zTimeStamp + "," + vRealX[count] + "," + vRealY[count] + "," + vRealZ[count]);
  vImagX[count] = 0;
  vImagY[count] = 0; 
  vImagZ[count] = 0;  

}

float findMaxInArr(double arr[]){
    double m = 0; 
    for (int i = 0; i< samples; i++){
        if (arr[i]>m){
          m = arr[i]; 
        }
    } 

    return m; 
}

float calcFFT(double vReal[],double vImag[], uint16_t samples){
      FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
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
void loop() {

  if (isSampling && !isScrubReset && !isShutDown){

  // collecting analog data until frequency is achieved 
  int beforeExec = micros();
  collectAnalog(counter);
  int leftOverDelay = periodLength - (micros() - beforeExec);
  if (leftOverDelay>0){
     delayMicroseconds(leftOverDelay);
   } 
 
  counter++;

    if (counter == samples){
    //int t1 = micros();
    
     
     float frqX = calcFFT(vRealX, vImagX, samples);
     float ampX = findMaxInArr(vRealX)/100000;
     float frqY = calcFFT(vRealY, vImagY, samples);
     float ampY = findMaxInArr(vRealY)/100000;
     float frqZ = calcFFT(vRealZ, vImagZ, samples);
     float ampZ = findMaxInArr(vRealZ)/100000;

    // PRINT OUTPUT : Serial.println(String(frqX) + "," + String(frqY) + "," + String(frqZ));

    Serial.println(millis()-t);
    t=millis();
    
    //TODO : send frequencies through CAN bus (frqX, frqY, frqZ)
     
     counter=0;
   
      
    } 
  }else{
    if (isShutDown){
       dataCollection.close(); 
      }
     if(isScrubReset){
      dataCollection.close();
      SD.remove("dataCollection.csv");
      dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);
      isScrubReset = false; 
     }
   }
   counter2++;
  }
