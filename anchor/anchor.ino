#include <SPI.h>
#include <DW1000.h>


#define MY_ID 0x0

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
byte tx_buffer[]={0x1,MY_ID, iter};


uint32_t range_buffer[]={0,0,0,0,0};
void push_buffer(uint32_t inbuffer[], uint32_t value){
  uint8_t i=0;
  for(i=0;i<5-1;i++){
    inbuffer[i] = inbuffer[i+1];
  }
  inbuffer[4] = value;
}
uint32_t avg_buffer(uint32_t inbuffer[]){
  uint64_t total=0;
  uint8_t i=0;
  for(i=0;i<5;i++){
    total += inbuffer[i];
  }
  return (uint32_t)(total/5);
}

typedef enum MY_STATUS{
  INIT, 
  RANGE1, 
  LISTEN1,
  POST_RANGE
}MY_STATUS_t;

MY_STATUS_t my_status=INIT;


void isr_handler(){
 // Serial.println("isr");
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

void newreceive(){
  dw.writeReg(0x0d,0,0, (uint32_t)(0x1<<6), 4);
}

void sentFixMsg(){
  byte tx_buffer[]={0x1,0x2,iter};
  dw.writeTxBuffer(tx_buffer, 3);
  dw.startTx();
  delay(10);
  iter++;
}

void sentBlink(){
  byte tx_buffer[]={0x1,iter};
  dw.writeTxBuffer(tx_buffer, 2);
  dw.startTx();
  delay(10);
  iter++;
}

void sentRange1(){

  dw.writeTxBuffer(tx_buffer, 3);
  dw.startTx();
  iter++; 
  // setup delay
}



void softReset(){
  //reset in 5 seconds inactivity
    Serial.println("timeout. Soft Reset");
    my_status = INIT;
    now_time = micros();
    last_time = micros();
    clearSent();
    clearReceive();
}

void newDelaysent(){
  dw.writeReg(0x0d,0,0, (uint16_t)(0x40), 2);
  dw.writeReg(0x0d,0,0, (uint16_t)(0x02), 2);
}


void setup() {
  Serial.begin(115200);
  dw.initialise();
  dw.setPanID(0xFFFF);
  dw.setShortID(0x0);
  
//  attachInterrupt(digitalPinToInterrupt(2), isr_handler, HIGH);

  // set interrupt for rx and tx
  //uint64_t eventMask = dw.readReg(0x0E, 0, 0, 4);
  uint16_t eventMask = 0;
 // eventMask |= (0x1<<7);
 // eventMask |= (0x1<<13);    //Mast transmit frame sent event
 // dw.writeReg(0x0E, 0, 0, eventMask, 2);
  //setup delay

  Serial.println("Initialisation complete!");
  
  Serial.println((uint32_t)(dw.readReg(0x0F,0,0,4)),BIN);
  now_time = millis();
  my_status = INIT;
}

void setDelay500(){
  dw.writeReg(0x0a,0,0,(uint32_t)63898160679, 4);
}
void setDelay800(uint64_t nowtime){
  uint64_t delay800uS = 51118210;
  uint64_t delta_time = (uint64_t)(0xFFFFFFFFFF)-nowtime;
  uint64_t new_time = 0;
  if (delta_time<delay800uS){
    new_time = delay800uS-delta_time;
  }else{
    new_time = nowtime+delay800uS;
  }
  dw.writeReg(0x0a,0,0,(uint32_t)(0xFFFFFFFF&delay800uS), 4);
  dw.writeReg(0x0a,1,4,(uint8_t)(0xFF&(delay800uS>>32)), 1);
}


bool issent(){
  return bitRead((uint32_t)(dw.readReg(0x0F,0,0,4)), 7);
}


bool isreceive(){
  return bitRead((uint32_t)(dw.readReg(0x0F,0,0,4)), 13);
}


uint64_t getTimestamp(uint8_t addr){
  return (uint64_t)(((uint64_t)dw.readReg(addr,1,4,1)<<32) | (uint64_t)(dw.readReg(addr,0,0,4)));
}

void loop() {
  // the loop should be non-blocking
  if(SPIevent){
   // Serial.println((unsigned long)dw.readReg(0x0F, 0, 0, 4), BIN);
    uint16_t sys_status = dw.readReg(0x0F,0,0,2);
    is_receive = bitRead(sys_status, 13);
    is_sent = bitRead(sys_status, 7);
    SPIevent = false;
  }
  switch(my_status){
    case INIT:
      my_status = RANGE1;
      Serial.print("ITER: ");
      Serial.println(iter);
      range1_time = 0;
      reply1_time = 0;
      delay(100);
    case RANGE1:
      Serial.println("RANGE1");
      newsent();
      sentRange1();
      // waiting for sent
      while(!issent()){
      }
      // record range1_time
      range1_time = getTimestamp(TX_TIME_ID);
      clearSent();
      my_status = LISTEN1;
      now_time = micros();
      //newreceive();
      dw.startRx();
    case LISTEN1:
      //timeout, goto post_range
      if(isreceive()){
        clearReceive();
        reply1_time = getTimestamp(RX_TIME_ID);
       // Serial.print("Reply1_time to range1_time is: ");
        delta_time = reply1_time - range1_time;
        delta_time = delta_time*50/3/100-20000000;
        delta_time = delta_time-1310000;
        push_buffer(range_buffer, (uint32_t)delta_time);
        Serial.print("delta time is: ");
        Serial.println(avg_buffer(range_buffer));
        
        //Serial.println((uint32_t)delta_time);
        //Serial.print((uint32_t)(delta_time>1310600? 100+(delta_time-1310600)*3:100), DEC);
        //Serial.print(" cm");
        //Serial.print("   System elapse: ");
        //Serial.println(micros()-now_time);
        now_time = micros();
        my_status = RANGE1;


/*
        Serial.print((uint8_t)(0xFF&(range1_time>>32)));
        Serial.print((uint32_t)(0xFFFFFFFF&range1_time));
        Serial.print("   ");
        Serial.print((uint8_t)(0xFF&(reply1_time>>32)));
        Serial.print((uint32_t)(0xFFFFFFFF&reply1_time));
        Serial.println("");
*/
        
      }
      break;
    case POST_RANGE:
      Serial.println("Received reply!");
      Serial.print("Cost time is: ");
      //Serial.println(reply1_time-range1_time-31949);
      now_time = micros();
      my_status = INIT;
      delay(1000);
      break;
    default:
      break;
  }

  if (micros()-now_time>1000000){
    //softReset();
    my_status = RANGE1;
  }
}
