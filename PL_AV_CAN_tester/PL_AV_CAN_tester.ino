#include <FlexCAN_T4.h>
#include <string.h>
#include "send.h"

//CAN BUS/ Payload State params
volatile bool isSampling = true;
volatile bool isScrubReset = false;
volatile bool isShutDown = false;
uint32_t offsetTime=0; //offset time which will be used in scrub to reset time to 0

uint32_t getTime() { //offset time gets update each time AV sends a scrub CAN message. Solution to not being able to reset millis()
  return (millis()-offsetTime);
}

int garbageDelay = 10;
int garbageCount = 0;

bool serialTest=true;
uint32_t t = getTime();
int led = 13;

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

bool PL_State;
int PL_FrqX;
int PL_FrqY;
int PL_FrqZ;
int PL_AmpX;
int PL_AmpY;
int PL_AmpZ;
int PL_Time;

void setup() {
  
  // put your setup code here, to run once:
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  //while (!Serial) //remove for flight
  //  ;
  Serial.println("READY");
  Serial.begin(9600);

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
  can2.setFIFOFilter(1, 0x300, STD); // Only receive messages with ID 0x300
  can2.setFIFOFilter(2, 0x301, STD);

  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  


}

void canSniff(const CAN_message_t &msg) {
  if (serialTest) {
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");  
    for ( uint8_t i = 0; i < 8; i++ ) {
    Serial.print(msg.buf[7-i], HEX); 
    Serial.print(" ");
    } 
    Serial.println();
    
    AV_FC_Decode(msg.id, msg.buf);
    char dataBuffer[200];
    int i = sprintPayload(dataBuffer);
    Serial.println(dataBuffer);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  can2.events();
  t = getTime();
  delay(1);
  recvWithEndMarker();
  showNewData();
  if(garbageCount > garbageDelay){
    sendGarbage();
    garbageCount = 0;
  }
  garbageCount++;
  

}

void sendGarbage() {
  sendMsg(0x01, 1);
  sendMsg(0x02, 1);
  sendMsg(0x03, 1);
}


void decodeMsg1(const uint8_t* msg1Data, bool*PL_state, int* PL_FrqX, int* PL_FrqY, int* PL_FrqZ, int* PL_AmpX, int* PL_AmpY) {
  *PL_state = (bool)(msg1Data[0] & 1);
  *PL_FrqX = (int) ((msg1Data[0] & B11111110) >> 1) | ((msg1Data[1] & B01111111) << 7);
  
  return;
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
      if (receivedChars[0] == 'b'){
        if (receivedChars[1] == 's'){
          sendMsg(0x11, 1);
        }
        else if (receivedChars[1] == 't'){
          sendMsg(0x11, 0);
        }
        else if (receivedChars[1] == 'o'){
          sendMsg(0x05, 0);
        }
      }
        newData = false;
    }
}
