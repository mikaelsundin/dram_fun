//------------------------------------------------------------------------------
//Mikael Sundin 2020
//www.linkedin.com/in/mikaelsundin
//
//------------------------------------------------------------------------------

//This sketch demostrate 4-bit 5V DRAM usage with Arduino.
//
//All available pins will be used on Arduino Nano V3. 
//Proof of concept. Made for fun.

//
//DRAM Pinout. 
//
//D2    PD2   CAS   Column lathch
//D3    PD3   !OE   Output enable
//D4    PD4   DQ2 
//D5    PD5   DQ3
//D6    PD6   DQ0
//D7    PD7   DQ1
//D8    PB0   A5
//D9    PB1   A4
//D10   PB2   A6
//D11   PB3   A7
//D12   PB4   !WR   Write DQ
//D13   PB5   !RAS  Row latch
//A0    PC0   A8    
//A1    PC1   A9
//A2    PC2   A3
//A3    PC3   A2
//A4    PC4   A1
//A4    PC5   A0

#include <stdint.h>

//Function Prototypes
uint8_t dram_read(uint32_t adr);
void dram_write(uint32_t adr, uint8_t val);

uint8_t dram_read4(uint32_t adr);
void dram_write4(uint32_t adr, uint8_t val);
void dram_set_adr(uint16_t adr);
void dram_burst_refresh(void);

//Macros for pin manipulation.
#define DRAM_OE_LOW()     PORTD &= ~(1<<PD3);
#define DRAM_OE_HIGH()    PORTD |= (1<<PD3);

#define DRAM_WR_LOW()     PORTB &= ~(1<<PB4);
#define DRAM_WR_HIGH()    PORTB |= (1<<PB4);

#define DRAM_CAS_LOW()     PORTD &= ~(1<<PD2);
#define DRAM_CAS_HIGH()    PORTD |= (1<<PD2);

#define DRAM_RAS_LOW()     PORTB &= ~(1<<PB5);
#define DRAM_RAS_HIGH()    PORTB |= (1<<PB5);

//Other macros
#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

//Read 8bit from DRAM
//Address is between 0 and 512k
uint8_t dram_read(uint32_t adr){
    uint8_t hi;
    uint8_t lo;
    
    hi = dram_read4(adr<<1 + 0);
    lo = dram_read4(adr<<1 + 1);
    
    return (hi<<4) + lo;
}

//Write 8bit to DRAM
//Address is between 0 and 512k
void dram_write(uint32_t adr, uint8_t val){
    uint8_t hi = (val>>4) & 0x0F;
    uint8_t lo = (val>>0) & 0x0F;
    
    dram_write4(adr<<1 + 0, hi);
    dram_write4(adr<<1 + 1, lo);
}


//Read 4bit to Dram
uint8_t dram_read4(uint32_t adr){
  uint8_t val=0;
  uint16_t row;
  uint16_t col;

  //Row, column
  row = adr & 0x03FF;
  adr >>= 10;
  col = adr & 0x03FF;

  //Be sure that the refresh is not run.
  noInterrupts();
  
  //Set Row
  dram_set_adr(row);
  DRAM_RAS_LOW();

  dram_set_adr(col);
  DRAM_CAS_LOW();
  DRAM_OE_LOW();

  DELAY_CYCLES(4);
  val = PIND >> 4;
  DRAM_OE_HIGH();


  DRAM_RAS_HIGH();
  DRAM_OE_HIGH();
  DRAM_CAS_HIGH();

    
  interrupts();
  
  return val;
}

//Write 4bit to DRAM (type: Early write cycle)
void dram_write4(uint32_t adr, uint8_t val){
  uint16_t row;
  uint16_t col;
  
  //Calculate row and column address
  row = adr & 0x03FF;
  adr >>= 10;
  col = adr & 0x03FF;

  //Be sure DRAM refresh is not run.
  noInterrupts();

  //Set Row
  dram_set_adr(row);
  DRAM_RAS_LOW();

  //Set PORTD to output and write value
  DDRD = 0xFF;
  PORTD = val << 4 | 0x0F; //WE, RAS high
  DRAM_WR_LOW();

  //Set Column
  dram_set_adr(col);
  DRAM_CAS_LOW();
  
  //Restore RAS, CAS
  DRAM_WR_HIGH();
  DRAM_RAS_HIGH();
  DRAM_CAS_HIGH();

  //Restore direction
  DDRD = 0x0F;
  interrupts();
}

//Set DRAM address, splitted between PORTB and PORTC.
void dram_set_adr(uint16_t adr){

  //Write to port. WE, RAS high
  PORTB &= 0xF0;
  PORTB |= (adr & 0x0F); 
  
  adr >>= 4;
  PORTC = (adr & 0x3F);
}


/* DRAM burst refresh, RAS-ONLY Refresh. Must be called every 16ms */
void dram_burst_refresh(void){
  uint16_t adr;
  uint8_t hi=0;
  uint8_t lo=0;

  for(adr=0;adr<1024;adr++){
    PORTB = lo | 0x30;  //set WE, RAS high
    PORTC = hi;

    PORTB = lo | 0x10;  //set WE, RAS low
    PORTB = lo | 0x30;  //set WE, RAS high

    //Calculate next address
    lo++; 
    lo &= 0x0F;
    
    hi += (lo == 0) ? 1 : 0;
  } 
}

//Use Timer0 comparator to get millisecond update for handling DRAM burst update.
//Takes around ~1.16ms @ 16Mhz (~7% CPU)
SIGNAL(TIMER0_COMPA_vect){
  static uint8_t cnt=0;

  //Burst Refresh every 15ms
  cnt++;
  if(cnt >= 15){
    cnt=0;
    dram_burst_refresh();
  }
  
}

void setup(void){
  Serial.begin(9600);
  Serial.println("---");

  //Timer0 Comaparator A interrupt in middle of millis to get 1ms update for DRAM burst refresh.
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  //Setup direction
  DDRB = 0x3F;  //PB0..PB5 output
  DDRC = 0x3F;  //PC0..PC5 output
  DDRD = 0x0F;  //PB2, PB3 output

  //All DRAM pins high.
  PORTB = 0x3F;
  PORTC = 0x3F;
  PORTD = 0x3F;

  //Be sure at least one DRAM burst is done.
  delay(16);

  //write some testdata to 8 DRAM locations
  for(uint8_t i=0;i<8;i++){
      dram_write(0x100 + i, i*8);  
  }
}


void loop(void){
  static uint8_t cnt=0;

  //Write out testdata
  Serial.println("----------------");
  for(uint8_t i=0;i<8;i++){
    uint8_t val = dram_read(0x100+i);
      Serial.println(val);
  }
  Serial.println("----------------");
  Serial.println("");

  //update first position
  dram_write(0x100, ++cnt);

  //wait 1 secound before check if data still in DRAM
  delay(1000);
}
