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


// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600); 
  while(!Serial);
  Serial.println("READY");
    Serial.begin(9600);
  
  while (!SD.begin(BUILTIN_SDCARD));
   // Serial.println("SD CARD FAILED, OR NOT PRESENT!");
   // while (1); // don't do anything more:
  Serial.println("SD Card initialized");

  // delete files if previously initialized 
  SD.remove("arduino.txt"); 
  SD.remove("arduino2.txt"); 
  SD.remove("frqX.txt");
  SD.remove("frqY.txt"); 
  SD.remove("frqZ.txt");
  
  // initialize x,y,z files 
  fileX = SD.open("frqX.txt", FILE_WRITE | FILE_READ);
  fileY = SD.open("frqY.txt", FILE_WRITE | FILE_READ);
  fileZ = SD.open("frqZ.txt", FILE_WRITE | FILE_READ);
}

void collectAnalog (int count){
  //Serial.println("Foo Called");
  analogReadResolution(18);  
  
 // Read count 
  vRealX[count] = analogRead(A0);
  String xTimeStamp = String(millis());
  vRealY[count] = analogRead(A1);
  String yTimeStamp = String(millis());
  vRealX[count] = analogRead(A2);
  String zTimeStamp = String(millis());

  // collecting x-axis data 
  fileX.println(xTimeStamp+": " + vRealX[count]);
  // collecting y-axis data
  fileY.println(yTimeStamp+": " + vRealY[count]);
  // collecting z-axis data
  fileZ.println(zTimeStamp+": " + vRealZ[count]);
  vImagX[count] = 0;
  vImagY[count] = 0; 
  vImagZ[count] = 0;  
  delay(5);
  
}

double calcFFT(double vReal[],double vImag[], uint16_t samples){
      FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
      double x; 
      double v;
      return FFT.MajorPeak(vReal, samples, samplingFrequency);  
}


int counter = 0; 
int t = millis();

// the loop runs when the data is 
void loop() {

  // collecting analog data until frequency is achieved 
  collectAnalog(counter);
  counter++;

    if (counter == samples){
    int t1 = micros();
    
     
     double frqX = calcFFT(vRealX, vImagX, samples);
     double frqY = calcFFT(vRealY, vImagY, samples);
     double frqZ = calcFFT(vRealZ, vImagZ, samples);


    Serial.println(micros()-t1);
     //TODO : send frequencies through CAN bus (frqX, frqY, frqZ)
     
     counter=0;
   
      
    }
 
  }
//  
//  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(100);               // wait for a second
//  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1);               // wait for a second
  
//}
