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

typedef struct payload_t {
  //Data is compressed
  uint64_t PL1; //Frequency and amplitude
  uint16_t PL2; //Sampling, minutes, seconds, and start of milliseconds
  uint8_t PL3; //End of milliseconds
} payload_t;

payload_t hpayload;

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

  uint64_t roundedAmpX = roundf(ampX * 100);
  if (roundedAmpX > 511) roundedAmpX = 511; //use 11111111 to indicate overflow

  uint64_t roundedAmpY = roundf(ampY * 100);
  if (roundedAmpY > 511) roundedAmpY = 511; //use 11111111 to indicate overflow


  formattedMsg1 = (roundedAmpY & 511) | ((roundedAmpX & 511) << 9) |((roundedFrqZ & 16383) << 21) | ((roundedFrqY & 16383) << 35) | ((roundedFrqX & 16383) << 49)|((Samplingbit) << 63) ;
  return formattedMsg1; 
}

uint64_t formatMsg2(float ampZ, uint32_t seconds){
  uint64_t formattedMsg2;
   
  uint64_t roundedAmpZ = roundf(ampZ * 100);
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
 

  char buf[100];
  sprintf(buf, "%llu\t%llu\r\n", msg1, msg2);
  Serial.println(buf);
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
void sendMsg(uint8_t id, uint8_t payload){
  can2.events();

  msg.id = id;
  
  msg.buf[0] = payload;
  
  can2.write(msg);
  if( id > 4){
    Serial.print(" ID: "); Serial.print(id, HEX);
    Serial.print(" Buffer: ");  Serial.print(payload, HEX); 
    Serial.println();
  }
  

}

void AV_FC_Decode(uint32_t id, const uint8_t* data) {
  if (id == 0x300){

    Serial.println("Payload Packet #1");
    memcpy(&hpayload.PL1, data, sizeof(uint64_t));
  }
  else if (id == 0x301){

    Serial.println("Payload Packet #2");
    memcpy(&hpayload.PL2, data, 2);
    memcpy(&hpayload.PL3, data+sizeof(uint16_t), 1);
  }
  
}

#define FREQ_X_LENGTH 14
#define FREQ_Y_LENGTH 14
#define FREQ_Z_LENGTH 14
#define A_X_LENGTH 9
#define A_Y_LENGTH 9
#define A_Z_LENGTH 9

static int sprintPayload(char* buffer) {

    //Message 1

    uint64_t b = hpayload.PL1;
    //b = swapLong(b);
    Serial.println(b,HEX);
    // b = swapIt(b);

    // uint16_t PL2 = 0b1000100000010001;
    // Serial.println(PL2, BIN);
    // PL2 = swapShort(PL2);
    // Serial.println(PL2, BIN);
    
  
    uint64_t mask;
  mask = 1;
  uint32_t state = b & mask;
  b = b >> 1;
  mask = ((1 << FREQ_X_LENGTH) - 1);
    uint32_t freqX = b & mask;
    b = b >> FREQ_X_LENGTH;
    uint32_t freqY = b & mask;
    b = b >> FREQ_Y_LENGTH;
    uint32_t freqZ = b & mask;
    b = b >> FREQ_Z_LENGTH+3;
    mask = ((1 << A_X_LENGTH) - 1);
    uint32_t aX = b & mask;
    b = b >> A_X_LENGTH;
    uint32_t aY = b & mask;
   
    
    
    //Message 2

    //uint16_t a = swapShort(radioPacket.data.pl.PL2);
  uint16_t a = hpayload.PL2;
  Serial.println(a, HEX);

    b = a + (hpayload.PL3 << 16);
    Serial.println(b,HEX);
    // Serial.println(radioPacket.data.pl.PL3, BIN);
    // Serial.println(b, BIN);
    // Serial.println(b, BIN);
    
    mask = ((1 << 15) - 1);
    uint32_t secs = b & mask;
    b = b >> 15;
  mask = ((1 <<  A_Z_LENGTH) - 1);
    uint32_t aZ = b & mask;
  



    return sprintf(buffer, "PL_Fx:%lu;PL_Fy:%lu;PL_Fz:%lu;PL_AccX:%lu;PL_AccY:%lu;PL_AccZ:%lu;PL_state:%lu;PL_seconds:%lu;",
                   freqX,
                   freqY,
                   freqZ,
                   aX,
                   aY,
                   aZ,
                   state,
                   secs
           );
  }
