#include <Adafruit_Sensor.h>
#include <DHT.h>
//#include <DHT_U.h>
#include <MPU9250.h>
#include <Wire.h>
#include "IntersemaBaro.h"
#include <SD.h>
#include <SPI.h>


Intersema::BaroPressure_MS5607B baro(true);    //The file named IntersemaBaro.h must be in the same folder as this code file


// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

int status;
int valve = 2;            // set to the digital pin that the valve is connected to
int vacuumPump1 = 1;        // set to the digital pins that pumps are connected to
int DHTpin = 3;

//int heightVariable = 0;
//int pumpVariable = 0;
File datalog;
const int num_meas = 50;
int alt_previous[num_meas];
float alt_meas;
float ground_alt = 0;
float T; // sampling period, time of each loop
float t_previous_loop;
float average_gradient;
float threshold_altitude =4000 ;
float  lower_threshold = 1500;
bool apogee_reached = false;

                              //SCL is pin 19 and SDL is pin 18

unsigned long time;

MPU9250 IMU(Wire,0x68);
DHT dht(DHTpin, DHTTYPE);

const int chipSelect = BUILTIN_SDCARD;                   //initialization for SD card
  const int ledPin = 13;


void setup() {

Serial.begin(115200);
//while(!Serial);
delay(500);
Serial.println("Sensor suite test");
 Wire.begin();

 dht.begin();
                                                         // TWBR = 12;  // 400 kbit/sec I2C speed
 
baro.init();
pinMode(ledPin, OUTPUT);
pinMode(valve,OUTPUT);
pinMode(vacuumPump1,OUTPUT);
if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
                                                             // if SD card is not present, won't do anything more:
    return;
  }
  Serial.println("Card initialized.");
     //there must be a file named datalog.txt on the SD card
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
  
}
 

void loop() {
 datalog = SD.open("datalog1.txt", FILE_WRITE); 

  time = millis();
  //read the barometer data
  float alt = baro.getHeightCentiMeters();
  float feet = (float)(alt) / 30.48;
  float meters = (float)(alt) / 100;
// read the IMU sensors
  IMU.readSensor();
 float h = dht.readHumidity();

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

  datalog.print("Time: ");
  datalog.print(time);        //recording data in SD card
  datalog.print("\t");
  datalog.print(", Feet: ");
  datalog.print(feet, 2);
  datalog.print("\t");
  datalog.print(" Meters: ");
  datalog.print(meters, 2);
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
  Serial.print(IMU.getTemperature_C(),6);
  Serial.print("\t");
  Serial.print("Humidity: ");
  Serial.println(dht.readHumidity(),6);

 
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
  datalog.print(IMU.getTemperature_C(),6);
  datalog.print("\t");
  datalog.print("Humidity: ");
  datalog.println(h);
  datalog.flush();
  datalog.close();
      alt_meas = ((baro.getHeightCentiMeters()/30.48) - ground_alt);          
  if (apogee_reached == false){
     T = (millis() - t_previous_loop)/1000; //millis() = time since program start running T running time of curr loop (s)
     t_previous_loop = millis(); //total time 
      
       //Measures AGL altitude in feet
  
      // Low-pass filter - rocket at high speeds pressure fluctuates and affects altitude reading, usaully at a high frequency, so low pass filter filters those high freuqency changes out 
      //and keeps just the overall, low frequency changes (caused by altitude change)
      //alt_filtered = (1 - T * a) * alt_previous[num_meas-1] + a * T * alt_meas;
      
      // Slide window of 10 measurement history.
      for (int i = 0; i < num_meas-1; i++){
        alt_previous[i] = alt_previous[i+1];
      }
      alt_previous[num_meas-1] = alt_meas;

      //Average gradient of 10 past measurements.
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
    if (apogee_reached == true ){
     digitalWrite(valve, HIGH);         //opens valve
    digitalWrite(vacuumPump1, HIGH);     //turns on pumps
    }
    if (alt_meas <= lower_threshold){
       digitalWrite(vacuumPump1, LOW);        //turns off pumps
      digitalWrite(valve, LOW);       //closes valve
    }

    
    if (alt_meas >= threshold_altitude){
      Serial.println("Apogee");
    }
    Serial.println(alt_meas);
    Serial.println(average_gradient);
    digitalWrite(ledPin, HIGH);
  delay(200);
    digitalWrite(ledPin, LOW);
  
}
