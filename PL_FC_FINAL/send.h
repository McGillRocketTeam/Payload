#include <math.h>
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;
CAN_message_t msg2;
/* Structure containing all the data needing to be send*/
struct Data {
  float frqX;
  float frqY;
  float frqZ; 
  
  float ampX;
  float ampY;
  float ampZ;

  bool isSampling;
  
  uint32_t seconds;
};

/** 
 *  Trick to split the int into bytes
 *  In a union, all members share the same memory location. So if a float is stored in 'f', and the 
 *  value of the array called 'bytes' is examined, each element of the array will contain one of the
 *  bytes of the 32 bit number encoding the float. 
 */
union my_msg {
  uint64_t msg;
  uint8_t bytes[8];
};



uint64_t formatMsg1(bool isSampling, float frqX, float frqY, float frqZ, float ampX, float ampY){
  uint64_t formattedMsg1;

  uint64_t Samplingbit; //"converting" a boolean to a 0 or 1  
  if (isSampling==true) {
    Samplingbit = 1;
  }
  else{
    Samplingbit = 0;   
  }
  
  // round the frequencies to the nearest unit before shifting
  uint64_t roundedFrqZ = roundf(frqZ);
  if (frqZ > 16383) roundedFrqZ = 16383; //use 11111111111111 to indicate overflow
  
  uint64_t roundedFrqY = roundf(frqY);
  if (frqY > 16383) roundedFrqY = 16383; //use 11111111111111 to indicate overflow

  uint64_t roundedFrqX = roundf(frqX);
  if (frqX > 16383) roundedFrqX = 16383; //use 11111111111111 to indicate overflow

  uint64_t roundedAmpX = roundf(ampX*100);
  if (roundedAmpX > 511) roundedAmpX = 511; //use 11111111 to indicate overflow

  uint64_t roundedAmpY = roundf(ampY*100);
  if (roundedAmpY > 511) roundedAmpY = 511; //use 11111111 to indicate overflow

  formattedMsg1 = (Samplingbit) | ((roundedFrqX & 16383) << 1) | ((roundedFrqY & 16383) << 15)| ((roundedFrqZ & 16383) << 29) | ((roundedAmpX & 511) << 46) | ((roundedAmpY & 511) << 55)   ;
  return formattedMsg1; 
}

uint64_t formatMsg2(float ampZ, uint32_t seconds){
  uint64_t formattedMsg2;
   
  uint64_t roundedAmpZ = roundf(ampZ*100);
  if (roundedAmpZ > 511) roundedAmpZ = 511; //use 11111111 to indicate overflow

  uint64_t formattedSeconds = 0;
  if (seconds > 32767) {
    formattedSeconds = 32767;
  }
  else {
    formattedSeconds = (uint64_t) seconds;
  }

  formattedMsg2 = (formattedSeconds & 32767) | ((roundedAmpZ & 511) << 15 ) | (0 << 24);
  return formattedMsg2;
}


void buildMsg(union my_msg *uMsg1, union my_msg *uMsg2, struct Data dt){

  uint64_t msg1 = formatMsg1(dt.isSampling,dt.frqX, dt.frqY, dt.frqZ, dt.ampX, dt.ampY);
  uint64_t msg2 = formatMsg2(dt.ampZ,dt.seconds);
  //uint64_t msg1 = 0x21EDCBA987654321; //Testing code
  //uint64_t msg2 = 0x654321;

  //char buf[100];
  //sprintf(buf, "%llu\t%llu\r\n", msg1, msg2);
  
  uMsg1->msg = msg1;
  uMsg2->msg = msg2;
}

/**
 * Send the 3 messages (total of 11 bytes) through CAN bus over 2 transmissions.
 * Each CAN transmission can carry 8 bytes of data
 * The first CAN transmission will deliver msg1 and msg2 (all 8 buffers will contain relevant data: 8 bytes) with CANID 0x300
 * The second transmission will deliver msg3 (only buffers 0,1,2 will contain relevant data: 3 bytes) with CANID 0x301
 * 
 */
void sendMsg(union my_msg *uMsg1, union my_msg *uMsg2){
  can2.events();

  msg.id = 0x300;
  for ( uint8_t i = 0; i < 8; i++ ) {
    msg.buf[i] = uMsg1->bytes[i];
  }
  can2.write(msg);

  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < 8; i++){
    Serial.print(msg.buf[7-i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  msg2.id = 0x301;
  for ( uint8_t i = 0; i < 8; i++ ) {
    msg2.buf[i] = uMsg2->bytes[i];
  }
  can2.write(msg2);

  Serial.print(" ID: "); Serial.print(msg2.id, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < 8; i++){
    Serial.print(msg2.buf[7-i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
