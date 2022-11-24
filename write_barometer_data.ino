#include <FlexCAN_T4.h>
#include <Wire.h>
#include <MS5x.h>

MS5x barometer(&Wire);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;

// In a union, all members share the same memory location. So if a float is stored in f, and the 
// value of the array called 'bytes' is examined, each element of the array will contain one of 
// bytes of the 32 bit number encoding the float. 

union my_float {
  float f;
  uint8_t bytes[4];
};

void setup(void) {
  can1.begin();
  can1.setBaudRate(250000);

  Serial.begin(115200);
  
  while(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
    Serial.println(F("Error connecting..."));
    delay(500);
  }
  Serial.println(F("Connected to Sensor"));
  delay(5);
}

void loop() {
  double pressure=0;
  double temperature=0;
  delay(1000);
  barometer.checkUpdates();
     
  if (barometer.isReady()) { 
    temperature = barometer.GetTemp(); // Returns temperature in C
    pressure = barometer.GetPres(); // Returns pressure in Pascals
    Serial.print(F("The Temperature is: "));
    Serial.println(temperature);
    Serial.print(F("The Pressure is: "));
    Serial.println(pressure);
  }
  
  can1.events();

  static uint32_t timeout = millis();
  if ( millis() - timeout > 2000 ) {
    CAN_message_t msg;
    msg.id = random(0x1,0x2); 
    
    union my_float temp_float; //initialize union
    temp_float.f = temperature; //store the temperature obtained from the barometer into the union
    
    for ( uint8_t i = 0; i < 4; i++ ) {
      msg.buf[i] = temp_float.bytes[i]; //a float is 4 bytes and each one of the 8 message buffers can only store 1 byte. Here we are storing each byte of the float in a different buffer.
    }
    
    can1.write(msg); //write the temp data do the bus
    timeout = millis();
  }
}
