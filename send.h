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

  bool isSampling;
  
  uint8_t minutes;
  uint8_t seconds;
  uint32_t milliseconds;
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
 * Given 3 frequency values, concatenates and stores them (partly) into a 32 bit unsigned int. 
 * 
 * Of these 32 bits:
 *   Most significant 12 bits encode the frqX, 
 *   then the next 12 encode frqY,
 *   the least significant 8 are for the most significant part of frqZ
 *   ie. xxxxxxxxxxxxyyyyyyyyyyyyzzzzzzzz
 *
 * The frequencies should only go up to 4000 Hz but if ever there is 
 * an overflow, it is indicated by all 1's. 
 */
uint32_t formatFrq(float frqX, float frqY, float frqZ) {
  uint32_t formattedFrqs;
  // round the frequencies to the nearest unit before shifting
  int roundedFrqZ = lroundf(frqZ);
  if (frqZ > 4000) roundedFrqZ = 4095; //use 111111111111 to indicate overflow
  
  int roundedFrqY = lroundf(frqY);
  if (frqY > 4000) roundedFrqY = 4095; //use 111111111111 to indicate overflow

  int roundedFrqX = lroundf(frqX);
  if (frqX > 4000) roundedFrqX = 4095; //use 111111111111 to indicate overflow

  formattedFrqs = (roundedFrqZ & 4095) >> 4 | ((roundedFrqY & 4095) << 8) | ((roundedFrqX & 4095) << 20); // 4095 is 0b111111111111.
  return formattedFrqs;
}

/**
 * Given 3 amplitude values and part of the frequency value, concatenates and stores them into a 32 bit unsigned int. 
 * Only 31 of the 32 bits are used FzFzFzFz0AxAxAx AxAxAxAxAxAxAyAy AyAyAyAyAyAyAyAz  AzAzAzAzAzAzAzAz
 
 * Of these 31 bits:
 *   Most significant 4 bits encode 
 *   Next 9 bits encode the frqX, 
 *   then the next 9 encode frqY,
 *   the least significant 9 are for frqZ
 */
uint32_t formatAmp(float ampX, float ampY, float ampZ, float frqZ){
  uint32_t formattedAmps;
  //multiply the amplitudes by 100 such that 2 decimals of precision can be kept. Then round to the nearest unit
  int roundedAmpX = lroundf(ampX * 100);
  if (ampX > 3.3) roundedAmpX = 511; //use 11111111 to indicate overflow

  int roundedAmpY = lroundf(ampY * 100);
  if (ampY > 3.3) roundedAmpY = 511;

  int roundedAmpZ = lroundf(ampZ * 100);
  if (ampZ > 3.3) roundedAmpZ = 511;

  int roundedFrqZ = lroundf(frqZ);
  if (frqZ > 4000) roundedFrqZ = 4095;
  formattedAmps = (roundedAmpZ & 511) | ((roundedAmpY & 511) << 9) | ((roundedAmpX & 511) << 18 | 0 << 27 |(roundedFrqZ & 4095) << 28); // 511 is 0b111111111.
  return formattedAmps;
}


/**
 * Given a time value, concatenates minutes, seconds and milliseconds and stores them into a 32 bit unsigned int. 
 * Only the least significant 23 of the 32 bits are used OSMMMMMM ssssssmm  mmmmmmmm
 
 * Of these 23 bits:
 *  Most significant bit represent the Payload isSampling state
 *  then the next 6 bits encode the minutes
 *  then the next 6 encode seconds
 *  the least significant 10 encode milliseconds
 *  OSMMMMMM ssssssmm  mmmmmmmm
 */
uint32_t formatTime(uint8_t minutes, uint8_t seconds, uint32_t milliseconds, bool isSampling){
 uint32_t Samplingbit; //"converting" a boolean to a 0 or 1  
 if (isSampling==true) {
    Samplingbit = 1;
 }
  else{
    Samplingbit = 0;   
  }
  uint32_t formattedTime;
  formattedTime = (milliseconds & 1023) | ((seconds & 63) << 10) | ((minutes & 63) << 16) | (Samplingbit) << 22 ; // 63 is 0b111111. 
  return formattedTime;
}

/**
 * Concatenates all the data into 3 messages so they can be sent through can bus.
 * msg structure:
 *     msg1: FxFxFxFxFxFxFxFx FxFxFxFxFyFyFyFy  FyFyFyFyFyFyFyFy  FzFzFzFzFzFzFzFz
 *     msg2: FzFzFzFz0AxAxAx  AxAxAxAxAxAxAyAy  AyAyAyAyAyAyAyAz  AzAzAzAzAzAzAzAz
 *     msg3: 0SMMMMMM ssssssmm mmmmmmmm (where S is payload sampling, M is minute, s is seconds and m is milliseconds)
 *  Finally, store the formatted messages into the my_msg union
 */
void buildMsg(union my_msg *uMsg1, union my_msg *uMsg2, union my_msg *uMsg3, struct Data dt){
  uint32_t formattedFrq = formatFrq(dt.frqX, dt.frqY, dt.frqZ);
  uint32_t formattedAmp = formatAmp(dt.ampX, dt.ampY, dt.ampZ, dt.frqZ);
  uint32_t formattedTime = formatTime(dt.minutes, dt.seconds, dt.milliseconds, dt.isSampling);


  
  uint32_t msg1 = (formattedFrq); 
  uint32_t msg2 = (formattedAmp); 
  uint32_t msg3 = (formattedTime);
  char buf[100];
  sprintf(buf, "%x\t%x\t%x\r\n", msg1, msg2, msg3);
  Serial.println(buf);
  uMsg1->msg = msg1;
  uMsg2->msg = msg2;
  uMsg3->msg = msg3;

}

/**
 * Send the 3 messages (total of 11 bytes) through CAN bus over 2 transmissions.
 * Each CAN transmission can carry 8 bytes of data
 * The first CAN transmission will deliver msg1 and msg2 (all 8 buffers will contain relevant data: 8 bytes) with CANID 0x300
 * The second transmission will deliver msg3 (only buffers 0,1,2 will contain relevant data: 3 bytes) with CANID 0x301
 * 
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
