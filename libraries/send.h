#include <math.h>

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
  
  int minutes;
  int seconds;
  int milliseconds;
};

/** 
 *  Trick to split the int into bytes
 *  In a union, all members share the same memory location. So if a float is stored in 'f', and the 
 *  value of the array called 'bytes' is examined, each element of the array will contain one of the
 *  bytes of the 32 bit number encoding the float. 
 */
union my_msg {
  uint32_t msg;
  uint8_t bytes[4];
};

/**
 * Given 3 frequency values, concatenates and stores them into a 32 bit unsigned int. 
 * Only the least significant 30 of the 32 bits are used
 * 
 * Of these 30 bits:
 *   Most significant 10 bits encode the frqX, 
 *   then the next 10 encode frqY,
 *   the least significant 10 are for frqZ
 *   ie. xxxxxxxxxxyyyyyyyyyyzzzzzzzzzz
 *
 * The frequencies should only go up to 1000 Hz but if ever there is 
 * an overflow, it is indicated by all 1's. 
 */
uint32_t formatFrq(float frqX, float frqY, float frqZ) {
  uint32_t formattedFrqs;
  // round the frequencies to the nearest unit before shifting
  int roundedFrqX = lroundf(frqX);
  if (frqX > 1000) roundedFrqX = 1023; //use 1111111111 to indicate overflow

  int roundedFrqY = lroundf(frqY);
  if (frqY > 1000) roundedFrqY = 1023;

  int roundedFrqZ = lroundf(frqZ);
  if (frqZ > 1000) roundedFrqZ = 1023;

  formattedFrqs = (roundedFrqZ & 1023) | ((roundedFrqY & 1023) << 10) | ((roundedFrqX & 1023) << 20); // 1023 is 0b1111111111.
  return formattedFrqs;
}

/**
 * Given 3 amplitude values, concatenates and stores them into a 32 bit unsigned int. 
 * Only the least significant 27 of the 32 bits are used
 
 * Of these 27 bits:
 *   Most significant 9 bits encode the frqX, 
 *   then the next 9 encode frqY,
 *   the least significant 9 are for frqZ
 */
uint32_t formatAmp(float ampX, float ampY, float ampZ){
  uint32_t formattedAmps;
  //multiply the amplitudes by 100 such that 2 decimals of precision can be kept. Then round to the nearest unit
  int roundedAmpX = lroundf(ampX * 100);
  if (ampX > 3.3) roundedAmpX = 511; //use 11111111 to indicate overflow

  int roundedAmpY = lroundf(ampY * 100);
  if (ampY > 3.3) roundedAmpY = 511;

  int roundedAmpZ = lroundf(ampZ * 100);
  if (ampZ > 3.3) roundedAmpZ = 511;

  formattedAmps = (roundedAmpZ & 511) | ((roundedAmpY & 511) << 9) | ((roundedAmpX & 511) << 18); // 511 is 0b111111111. Most significant 9 bits encode the ampX, then the next 9 encode ampY,the leas significant 9 are for ampZ
  return formattedAmps;
}

int getMinutes(int milliseconds){
  Serial.println(milliseconds/60000);
  return (milliseconds/60000);
}
int getSeconds(int milliseconds){
  return (milliseconds/1000) % 60;
}
int getMillis(int milliseconds){
  return (milliseconds)- getMinutes(milliseconds)-getSeconds(milliseconds);
}

/**
 * Given a time value, concatenates minutes, seconds and milliseconds and stores them into a 32 bit unsigned int. 
 * Only the least significant 22 of the 32 bits are used
 
 * Of these 22 bits:
 *  Most significant 6 bits encode the minutes
 *  then the next 6 encode seconds
 *  the least significant 10 encode milliseconds
 *  MMMMMM ssssss mmmmmmmmmm
 */
uint32_t formatTime(int minutes, int seconds, int milliseconds){
  uint32_t formattedTime;
  formattedTime = (milliseconds & 1023) | ((seconds & 63) << 10) | ((minutes & 63) << 16); // 63 is 0b111111. 
  return formattedTime;
}

/**
 * Concatenates all the data into 3 messages so they can be sent through can bus.
 * msg structure:
 *     msg1: 00FFFFFF FFFFFFFF FFFFFFFF FFFFFFFF
 *     msg2: 0000000A AAAAAAAA AAAAAAAA AAAAAAAA
 *     msg3: 0SMMMMMM ssssssmm mmmmmmmm (where S is payload sampling, M is minute, s is seconds and m is milliseconds)
 *  Finally, store the formatted messages into the my_msg union
 */
void buildMsg(union my_msg *uMsg1, union my_msg *uMsg2, union my_msg *uMsg3, struct Data dt){
  uint32_t formattedFrq = formatFrq(dt.frqX, dt.frqY, dt.frqZ);
  uint32_t formattedAmp = formatAmp(dt.ampX, dt.ampY, dt.ampZ);
  uint32_t formattedTime = formatTime(dt.minutes, dt.seconds, dt.milliseconds);
 
  uint32_t msg1 = (formattedFrq); 
  uint32_t msg2 = (formattedAmp); 
  uint32_t msg3 = (1) << 22 | (formattedTime);

  uMsg1->msg = msg1;
  uMsg2->msg = msg2;
  uMsg3->msg = msg3;

}

/**
 * Send the 3 messages (total of 10 bytes) through CAN bus over 2 transmissions.
 * Each CAN transmission can carry 8 bytes of data
 * The first CAN transmission will deliver msg1 and msg2 (all 8 buffers will contain relevant data: 8 bytes) with CANID 0x300
 * The second transmission will deliver msg3 (only buffers 0 and 1 will contain relevant data: 2 bytes) with CANID 0x301
 */
void sendMsg(union my_msg *uMsg1, union my_msg *uMsg2, union my_msg *uMsg3){
  can2.events();

  msg.id = 0x300;
  for ( uint8_t i = 0; i < 4; i++ ) {
    msg.buf[3-i] = uMsg1->bytes[i];
  }
  for ( uint8_t i = 0; i < 4; i++ ) {
    msg.buf[7-i] = uMsg2->bytes[i]; 
  }
  can2.write(msg);

  msg2.id = 0x301;
  for ( uint8_t i = 0; i < 3; i++ ) {
    msg2.buf[2-i] = uMsg3->bytes[i];
  }
  can2.write(msg2);
}

//TODO test this function
//struct Data decode(uint8_t buf1[], uint8_t buf2[]) {
//  union my_msg uMsg1;
//  union my_msg uMsg2;
//  union my_msg uMsg3;
//
//  for ( uint8_t i = 0; i < 4; i++ ) {
//    uMsg1.bytes[i] = buf1[i];
//    uMsg2.bytes[i] = buf1[i + 4];
//  }
//
//  for ( uint8_t i = 0; i < 3; i++ ) {
//    uMsg3.bytes[i] = buf2[i];
//    Serial.print(uMsg3.bytes[i]);
//  }
//
//  int frqX = (uMsg1.msg) >> 20 & 1023;
//  int frqY = (uMsg1.msg) >> 10 & 1023;
//  int frqZ = (uMsg1.msg) & 1023;
//
//  float ampX = (((uMsg2.msg) >> 18 & 511)) / 100.0;
//  float ampY = (((uMsg2.msg) >> 9 & 511)) / 100.0;
//  float ampZ = ((uMsg2.msg) & 511) / 100.0;
//
//  int milliseconds = ((uMsg3.msg) & 1023)  ;
//  int seconds = ((uMsg3.msg) >> 10 & 63);
//  int minutes = ((uMsg3.msg) >> 16 & 63);
//
//  struct Data dt = {frqX, frqY, frqZ, ampX, ampY, ampZ, minutes, seconds, milliseconds};
//
//  return dt;
//}
