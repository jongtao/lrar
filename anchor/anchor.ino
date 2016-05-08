#include <SPI.h>
#include "DW1000.h"


#define MY_ID 0x0
#define DELAY_MS 20
#define TIMEOUT_US 3000 

int ss = 10;
int rstpin = 9;
long temp;

DW1000 dw = DW1000(ss, rstpin);
uint8_t iter=0;
unsigned long now_time = 0;
uint64_t range1_time =0;
uint64_t reply1_time =0;
uint64_t delta_time = 0;
byte tx_buffer[]={0x1,MY_ID, iter};

uint8_t timeout = 0;
#define NUM_TAGS 3
uint8_t tag_numbers[NUM_TAGS] = {0x01, 0x02, 0x03};
uint8_t tag = 0; // start with tag 0


uint32_t range_buffer[]={0,0,0,0,0}; // range to report

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



void clearReceive(){
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


void sentRange1(uint8_t target_ID){

	tx_buffer[1] = target_ID;
  dw.writeTxBuffer(tx_buffer, 3);
  dw.startTx();
  iter++; 
  // setup delay
}


void setup() {
  Serial.begin(115200);

  dw.initialise();
  dw.setPanID(0xFFFF);
  dw.setShortID(0x0);
  
//  attachInterrupt(digitalPinToInterrupt(2), isr_handler, HIGH);

  // set interrupt for rx and tx
  //uint64_t eventMask = dw.readReg(0x0E, 0, 0, 4);
	//
  uint16_t eventMask = 0;

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



inline void DW1000_reset(void)
{
	my_status = RANGE1;
	//Serial.print("ITER: ");
	//Serial.println(iter);
	range1_time = 0;
	reply1_time = 0;
	delay(100);
} // DW1000_init()



inline void DW1000_send(uint8_t target_ID)
{
	//Serial.println("RANGE1");
	newsent();
	sentRange1(target_ID);
	while(!issent())
		;
	// record range1_time
	range1_time = getTimestamp(TX_TIME_ID);
	clearSent();
	my_status = LISTEN1;
} // DW10000_send()



inline void DW1000_receive(void)
{
	timeout = 0;
	dw.startRx();

	// Check for timeout
	now_time = micros();
	while(!isreceive())
	{
		if (micros()-now_time > TIMEOUT_US)
		{
			timeout = 1;
			DW1000_reset(); // reset and restart on timeout
			return;
		}
	}

	clearReceive();
	reply1_time = getTimestamp(RX_TIME_ID);
	delta_time = reply1_time - range1_time;
	delta_time = (delta_time-65000000)*50/3/100;
	delta_time = delta_time-11000;
	push_buffer(range_buffer, (uint32_t)delta_time);
	Serial.print("Tag: ");
	Serial.println(tag_numbers[tag]);
	Serial.print("delta time is: ");
	Serial.println(avg_buffer(range_buffer));
} // DW1000_receive()



inline void post_range(void)
{
	//Serial.println("Received reply!");
	//Serial.print("Cost time is: ");
	//Serial.println(reply1_time-range1_time-31949);
	now_time = micros();
	my_status = INIT;
	delay(DELAY_MS);
}




void loop()
{
	DW1000_reset();
	DW1000_send(tag_numbers[tag]);
	DW1000_receive();

	tag++;
	if(tag >= NUM_TAGS)
		tag = 0;

	if(!timeout)
		post_range();
} // loop()

