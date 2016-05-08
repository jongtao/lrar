#include <SPI.h>
#include "DW1000.h"



#define MY_ID 0x01
int ss = 10;
int rstpin = 9;
long temp;

DW1000 dw = DW1000(ss, rstpin);
volatile bool SPIevent = false;
bool is_sent = false;
bool is_receive = false;
uint8_t iter=0;
unsigned long now_time = 0;
unsigned long last_time = 0;
uint64_t range1_time =0;
uint64_t reply1_time =0;
uint64_t delta_time = 0;
byte temp_buffer[7];
byte tx_buffer[]={0x02,MY_ID,iter};
typedef enum MY_STATUS{
  INIT, 
  LISTEN1,
  REPLY1, 
  POST_RANGE
}MY_STATUS_t;

MY_STATUS_t my_status=INIT;




void isr_handler(){
  Serial.println("isr");
  SPIevent = true;
}

void event_handler(){
  uint16_t event_code = dw.readReg(0x0F, 0, 0, 2);
  Serial.println(event_code, BIN);
  if (bitRead(event_code, 7)){
    is_sent = true;
  }
  if (bitRead(event_code, 13)){
    is_receive=true;
  }
}



void clearReceive(){
  /*
  dw.clearSystemStatus(SYS_MASK_MRXPRD);
  dw.clearSystemStatus(SYS_MASK_MRXSFDD);
  dw.clearSystemStatus(SYS_MASK_MLDEDONE);
  dw.clearSystemStatus(SYS_MASK_MRXPHD);
  dw.clearSystemStatus(SYS_MASK_MRXDFR);
  dw.clearSystemStatus(SYS_MASK_MRXDFR);
  */
  dw.writeReg(0x0F, 0,0, (uint32_t)(0x3F<<11), 4);
}

void clearSent(){
  dw.writeReg(0x0f,0,0, (uint8_t)(0xF0),1);
}

void newsent(){
  dw.writeReg(0x0d,0,0, (uint32_t)(0x1<<6), 4);
}

void newDelaysent(){
  dw.writeReg(0x0d,0,0, (uint8_t)(0x1<<6), 1);
  //dw.writeReg(0x0d,0,0, (uint16_t)(0x02), 2);
}

void newreceive(){
  dw.writeReg(0x0d,0,0, (uint32_t)(0x1<<6), 4);
}


void sentBlink(){
//  byte tx_buffer[]={0x01,MY_ID,iter};
//  dw.writeTxBuffer(tx_buffer, 3);
  dw.startTx();
//  delay(10);
  iter++;
}

void printFixMsg(){
  
}

void reply1(){
 // dw.writeTxBuffer(tx_buffer, 3);
  dw.startTx();
  iter++;
}

void reply1Delay(){

 // dw.writeTxBuffer(tx_buffer, 3);
  dw.writeReg(0x0d,0,0, (uint8_t)(0x06), 1);
  iter++;
}

void noteActivity(){
  now_time = millis();
}


void softReset(){
  //reset in 5 seconds inactivity
    Serial.println("timeout. Soft Reset");
    clearReceive();
    clearSent();
    my_status = INIT;
    now_time = millis();
    last_time = millis();
}

void setDelay500(){
  dw.writeReg(0x0a,0,0,(uint32_t)63898160679, 4);
}

void setDelay800(uint64_t nowtime){
  uint64_t delay800uS = 51118211;
  //uint64_t delay800uS = 63897763;
  uint64_t delta_time = (uint64_t)(0xFFFFFFFFFF)-nowtime;
  uint64_t new_time = 0;
  if (delta_time<delay800uS){
    new_time = delay800uS-delta_time;
  }else{
    new_time = nowtime+delay800uS;
  }
  dw.writeReg(0x0a,0,0,(uint32_t)(0xFFFFFFFF&new_time), 4);
  dw.writeReg(0x0a,1,4,(uint8_t)(0xFF&(new_time>>32)), 1);
}


void setDelayBIG(uint64_t nowtime){
  //2ms
  uint64_t delayBIG = 127795527;
  uint64_t delta_time = (uint64_t)(0xFFFFFFFFFF)-nowtime;
  uint64_t new_time = 0;
  if (delta_time<delayBIG){
    new_time = delayBIG-delta_time;
  }else{
    new_time = nowtime+delayBIG;
  }
  dw.writeReg(0x0a,0,0,(uint32_t)(0xFFFFFFFF&new_time), 4);
  dw.writeReg(0x0a,1,4,(uint8_t)(0xFF&(new_time>>32)), 1);
 // Serial.print((uint8_t)(0xFF&(nowtime>>32)));
  //Serial.print((uint32_t)(0xFFFFFFFF&nowtime));
 // Serial.print("   ");
 // Serial.print((uint8_t)(0xFF&(new_time>>32)));
 // Serial.print((uint32_t)(0xFFFFFFFF&new_time));
 // Serial.println("");
}

uint64_t getTimestamp(uint8_t addr){
  return (uint64_t)(((uint64_t)dw.readReg(addr,1,4,1)<<32) | (uint64_t)(dw.readReg(addr,0,0,4)));
}

bool issent(){
  return bitRead((uint32_t)(dw.readReg(0x0F,0,0,4)), 7);
}


bool isreceive(){
  return bitRead((uint32_t)(dw.readReg(0x0F,0,0,4)), 13);
}


void setup() {
  Serial.begin(115200);
  dw.initialise();
  dw.setPanID(0xFFFF);
  dw.setShortID(0x0);
  
 // attachInterrupt(digitalPinToInterrupt(2), isr_handler, HIGH);

  // set interrupt for rx and tx
  //eventMask |= (0x1<<7);
 // eventMask |= (0x1<<13);    //only care receive msg
 // dw.writeReg(0x0E, 0, 0, (uint16_t)eventMask, 2);
  
  Serial.println("Initialisation complete!");
  
  //Serial.println((uint32_t)(dw.readReg(0x0E,0,0,4)),BIN);
  now_time = millis();
  my_status = INIT;
}

void loop() {
  switch(my_status){
    case INIT:
      my_status = REPLY1;
      clearReceive();
      clearSent();
      //dw.writeReg(0x1A, 0,0,0x3,1);
      Serial.print("ITER: ");
      Serial.println(iter);
      now_time = millis();
      newreceive();
      dw.startRx();
      break;
    case REPLY1:
      if(isreceive()){
        //Serial.println("Received sth!");
        clearReceive();
        clearSent();
        setDelayBIG(getTimestamp(RX_TIME_ID));

        newDelaysent();
        reply1Delay();
        while(!issent()){
        }
        //clearSent();
        my_status = POST_RANGE;
      }
      break;
    case POST_RANGE:
      now_time = millis();
      my_status = REPLY1;
      newreceive();
      dw.startRx();
      //delayMicroseconds(14);
      break;
    default:
      break;
  }
  if (millis()-now_time>1000){
    my_status = POST_RANGE;
  }


}
