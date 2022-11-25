#include <arduinoFFT.h>
#include <StaticThreadController.h>
#include <Thread.h>
#include <ThreadController.h>
#include <SD.h>
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:


arduinoFFT FFT = arduinoFFT(); // CREATE FFT object

const uint16_t samples = 4096; //This value MUST ALWAYS be a power of 2
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


double vReal[samples];
double vImag[samples];
int led = 13;

File file; 


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
  //SD.remove("arduino.txt"); // delete the file if existed
  //SD.remove("arduino2.txt"); // delete the file if existed
  file = SD.open("arduino.txt", FILE_WRITE | FILE_READ);
}

void foo (int count){
  //Serial.println("Foo Called");
  analogReadResolution(18);  
  
 // Serial.println(analogRead(A1));
  vReal[count] = analogRead(A0);
  file.println(vReal[count]);
    Serial.println(file.size());
  if (file.size()<8190){
      Serial.println(file.size());
   }else {
      //file = SD.open("arduino2.txt", FILE_WRITE | FILE_READ);
      Serial.println("new file arduino2.txt opened");
    }

  vImag[count] = 0; 
  delay(5);
  
}

// the loop routine runs over and over again forever:
int counter = 0; 
int t = millis();
void loop() {
  
  foo(counter);
  counter++;

  if (counter == samples){
    int t1 = micros();
      FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
      double x; 
      double v;
      double frq  = FFT.MajorPeak(vReal, samples, samplingFrequency);
     // Serial.print(x, 6);
     // Serial.print(", ");
     // Serial.println(v, 6);
     if (file.size()<8190){
     Serial.println(micros()-t1);
     Serial.println(frq);
    counter=0;
    
    Serial.println("Major Peak Frequency: "+String(millis()-t));
    Serial.print("File Size: ");
    Serial.println(file.size());
    t=millis();
      
      }
 
  }
//  
//  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(100);               // wait for a second
//  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1);               // wait for a second
  
}
