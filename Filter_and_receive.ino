#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

/*
This code receives messages on a bus and filters them to only interrupt when the
relevant IDs are on the bus.
*/

void setup(void) {
  Serial.begin(115200); delay(400);
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(100000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();

  //Filtering
  Can0.setFIFOFilter(REJECT_ALL); // Block data before setting filter
  Can0.setFIFOFilter(1, 0x3, STD); // Only receive messages with ID 0x3
  Can0.setFIFOFilter(2, 0x4, STD); // Only receive messages with ID 0x4

//  Not working for now
//  Can0.setFIFOFilter(0, 2, 1, STD);
//  Can0.setFIFOFilterRange(0, 0x1, 0x2, STD);
  
  Can0.onReceive(canSniff); // Interrupt: Go into the canSniff function when a message is received
  Can0.mailboxStatus();
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void loop() {
  Can0.events();
}

// See https://github.com/tonton81/FlexCAN_T4/tree/98654c7844408b536a44c3f3693a9c9a7335b21a for
// the complete documentation
