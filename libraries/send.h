#include <math.h>

/**
 * Given 3 frequency values, concatenates and stores them into a 32 bit unsigned int. 
 * Only the least significant 30 of the 32 bits are used
 * 
 * Of these 30 bits:
 *   Most significant 10 bits encode the frqX, 
 *   then the next 10 encode frqY,
 *   the least significant 10 are for frqZ
 */
uint32_t formatFrq(float frqX, float frqY, float frqZ) {
  uint32_t formattedFrqs;
  // round the frequencies to the nearest unit before shifting
  int roundedFrqX = lroundf(frqX);
  int roundedFrqY = lroundf(frqY);
  int roundedFrqZ = lroundf(frqZ);
  
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
  int roundedAmpX = lroundf(ampX*100);
  int roundedAmpY = lroundf(ampY*100);
  int roundedAmpZ = lroundf(ampZ*100);
  
  formattedAmps = (roundedAmpZ & 511) | ((roundedAmpY & 511) << 9) | ((roundedAmpX & 511) << 18); // 511 is 0b111111111. 
  
  return formattedAmps;
}

/**
 * Given a time value, concatenates minutes, seconds and milliseconds and stores them into a 32 bit unsigned int. 
 * Only the least significant 22 of the 32 bits are used
 
 * Of these 22 bits:
 *  Most significant 6 bits encode the minutes
 *  then the next 6 encode seconds
 *  the least significant 10 encode milliseconds
 */
uint32_t formatTime(int minutes, int seconds, int milliseconds){
  uint32_t formattedTime;
  formattedTime = (milliseconds & 1023) | ((seconds & 63) << 10) | ((minutes & 63) << 16); // 63 is 0b111111. 
  return formattedTime;
}

/**
 * Concatenates all the data into 3 messages so they can be sent through can bus.
 * msg structure:
 *    msg1(32 bits): Lower 2 bits of amp + all 30 bits of frq
 *    msg2 (32 bits): Lower 7 bits oftime + upper 25 bits of amp
 *    msg3(16 bits): 1 bit payload sampling + upper 15 bits of time
 *  Finally, store the formatted messages into the my_msg union
 */
void buildMsg(union my_msg *uMsg1, union my_msg *uMsg2, union my_msg *uMsg3, struct Data dt){
  uint32_t formattedFrq = formatFrq(dt.frqX, dt.frqY, dt.frqZ);
  uint32_t formattedAmp = formatAmp(dt.ampX, dt.ampY, dt.ampZ);
  uint32_t formattedTime = formatTime(dt.minutes, dt.seconds, dt.milliseconds);

  uint32_t msg1 = ((formattedAmp & 3)) << 30 | (formattedFrq); 
  uint32_t msg2 = ((formattedTime & 127) << 25) | (formattedAmp >> 2); 
  uint16_t msg3 = (1) << 15 | (formattedTime >> 7);

  uMsg1->msg = msg1;
  uMsg2->msg = msg2;
  uMsg3->msg = msg3;
}

/**
 * Send the 3 messages (total of 10 bytes) through CAN bus over 2 transmissions.
 * Each CAN transmission can carry 8 bytes of data
 * The first CAN transmission will deliver msg1 and msg2 (all 8 buffers will contain relevant data: 8 bytes) with CANID 0x300
 * The second transmission will deliver msg3 (only buffers 0 and 1 will contain relevant data: 2 bytes) with CANID 0x301
 * 
 */
void sendMsg(union my_msg *uMsg1, union my_msg *uMsg2, union my_msg *uMsg3){
  can1.events();
  
  msg.id = 0x300;
  for ( uint8_t i = 0; i < 4; i++ ) {
      msg.buf[i] = uMsg1->bytes[i]; 
  }
  for ( uint8_t i = 0; i < 4; i++ ) {
      msg.buf[i] = uMsg2->bytes[i];
  }
  can1.write(msg); //todo test
  
  msg.id = 0x301;
  for ( uint8_t i = 0; i < 2; i++ ) {
      msg.buf[i] = uMsg3->bytes[i];
  }
  can1.write(msg); //todo test
}

