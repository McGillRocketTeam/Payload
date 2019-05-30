#include <Adafruit_Sensor.h>
#include <DHT.h>
//#include <DHT_U.h>
#include <MPU9250.h>
#include <Wire.h>
#include "IntersemaBaro.h"
#include <SD.h>
#include <SPI.h>


Intersema::BaroPressure_MS5607B baro(true);    //The file named IntersemaBaro.h must be in the same folder as this code file


int status;
int vacuumPump1 = 1;        // set to the digital pin that pumps are connected to
int valve = 2;            // set to the digital pin that the valve is connected to
int buzzer = 3;

File datalog;
const int num_meas = 50;
int alt_previous[num_meas];
float alt_meas;
float ground_alt = 0;
float T; // sampling period, time of each loop
float t_previous_loop;
float average_gradient;
float threshold_altitude = 28000 ;
float  lower_threshold = 1800;
bool apogee_reached = false;

                              //SCL is pin 19 and SDL is pin 18

unsigned long time;

MPU9250 IMU(Wire,0x68);


const int chipSelect = BUILTIN_SDCARD;                   //initialization for SD card
  const int ledPin = 13;


void setup() {

Serial.begin(115200);
//while(!Serial);
delay(500);
Serial.println("Sensor suite test");
 Wire.begin();
                                                         // TWBR = 12;  // 400 kbit/sec I2C speed
 
baro.init();
pinMode(ledPin, OUTPUT);
pinMode(valve,OUTPUT);
pinMode(vacuumPump1,OUTPUT);
pinMode(buzzer,OUTPUT);
if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
                                                             // if SD card is not present, won't do anything more:
    return;
  }
  Serial.println("Card initialized.");
     //will create a file named datalog.txt on the SD card
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
   

  }
  for (int i = 0; i < 500; i++){
    ground_alt += (baro.getHeightCentiMeters()/30.48); //reads alt 500 times
  }
  ground_alt = ground_alt/500; //average of alt readings
  

  for (int j = 0; j < 2048 * 3; j++ ) 
  {
   // 1 / 2048Hz = 488uS, or 244uS high and 244uS low to create 50% duty cycle
   digitalWrite(buzzer, HIGH);
   delayMicroseconds(244);
   digitalWrite(buzzer, LOW);
   delayMicroseconds(244);
   } 
}
void loop() {
 datalog = SD.open("datalog.txt", FILE_WRITE); 

  time = millis();
  //read the barometer data
  float alt = baro.getHeightCentiMeters();
  float feet = (float)(alt) / 30.48;
  float meters = (float)(alt) / 100;
// read the IMU sensors
  IMU.readSensor();



   alt_meas = ((baro.getHeightCentiMeters()/30.48) - ground_alt);          
  if (apogee_reached == false){
     T = (millis() - t_previous_loop)/1000; //millis() = time since program start running T running time of curr loop (s)
     t_previous_loop = millis(); //total time 
      
       //Measures AGL altitude in feet
  
      for (int i = 0; i < num_meas-1; i++){
        alt_previous[i] = alt_previous[i+1];
      }
      alt_previous[num_meas-1] = alt_meas;

      //Average gradient of 50 past measurements.
      average_gradient = 0;
      for (int i = 0; i < num_meas-1; i++){
        average_gradient += (alt_previous[i+1]- alt_previous[i]);
      }
      if (T>0){
      average_gradient /= (num_meas);
      }
       if ((alt_meas > threshold_altitude) && (apogee_reached == false)){
        
        if (average_gradient < -1.0){ 
          apogee_reached = true;
        }
       }

  }
 

  // display the data
  Serial.print("Time: ");
  Serial.print(time);           //recording data in serial monitor
  Serial.print("\t");
  Serial.print(", Feet: ");
  Serial.print(feet, 2);
  Serial.print("\t");
  Serial.print(", Meters: ");
  Serial.print(meters, 2);
  Serial.print("\t");
  Serial.print("Height Differential:");
  Serial.println(alt_meas);
  Serial.print("\t");
  Serial.print("Gradient:");
  Serial.print(average_gradient);
  Serial.print("\t");
  

  datalog.print("Time: ");
  datalog.print(time);        //recording data in SD card
  datalog.print("\t");
  datalog.print(", Feet: ");
  datalog.print(feet, 2);
  datalog.print("\t");
  datalog.print(" Meters: ");
  datalog.print(meters, 2);
  datalog.print("\t");
  datalog.print("Height Differential:");
  datalog.print("\t");
  datalog.print("Height Differential:");
  datalog.println(alt_meas);
  datalog.print("\t");
  datalog.print("Gradient:");
  datalog.print(average_gradient);
  datalog.print("\t");

  Serial.print("x-accel: ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print("y-accel: ");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print("z-accel: ");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print("x-gyro: ");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print("y-gyro: ");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print("z-gyro: ");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print("x-gyro: ");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print("y-gyro: ");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print("z-gyro: ");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.print("Temp: ");
  Serial.println(IMU.getTemperature_C(),6);
  
  
  
 
  datalog.print("x-accel: ");
  datalog.print(IMU.getAccelX_mss(),6);
  datalog.print("\t");
  datalog.print("y-accel: ");
  datalog.print(IMU.getAccelY_mss(),6);
  datalog.print("\t");
  datalog.print("z-accel: ");
  datalog.print(IMU.getAccelZ_mss(),6);
  datalog.print("\t");
  datalog.print("x-gyro: ");
  datalog.print(IMU.getGyroX_rads(),6);
  datalog.print("\t");
  datalog.print("y-gyro: ");
  datalog.print(IMU.getGyroY_rads(),6);
  datalog.print("\t");
  datalog.print("z-gyro: ");
  datalog.print(IMU.getGyroZ_rads(),6);
  datalog.print("\t");
  datalog.print("x-gyro: ");
  datalog.print(IMU.getMagX_uT(),6);
  datalog.print("\t");
  datalog.print("y-gyro: ");
  datalog.print(IMU.getMagY_uT(),6);
  datalog.print("\t");
  datalog.print("z-gyro: ");
  datalog.print(IMU.getMagZ_uT(),6);
  datalog.print("\t");
  datalog.print("Temp: ");
  datalog.println(IMU.getTemperature_C(),6);
  
  
   
    if (apogee_reached == true ){
     digitalWrite(valve, HIGH);         //opens valve
    digitalWrite(vacuumPump1, HIGH);     //turns on pumps
    Serial.print("\t");
    Serial.print("\t");
    Serial.println("Apogee Reached");
    datalog.print("\t");
    datalog.print("\t");
    datalog.println("Apogee Reached");
    
    }
    if (alt_meas <= lower_threshold){
       digitalWrite(vacuumPump1, LOW);        //turns off pumps
      digitalWrite(valve, LOW);       //closes valve
      Serial.print("\t");
      Serial.print("\t");
      Serial.print("Lower Threshold Reached");
      datalog.print("\t");
      datalog.print("\t");
      datalog.println("Lower Threshold Reached");
      
    }
    
  datalog.flush();
  datalog.close();
    digitalWrite(ledPin, HIGH);
  delay(200);
    digitalWrite(ledPin, LOW);
  
}
