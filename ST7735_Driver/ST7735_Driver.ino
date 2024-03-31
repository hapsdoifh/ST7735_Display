#include <SPI.h>
#define CS 10
#define RES 9
#define DC 8

#define D_WIDTH 160
#define D_HEIGHT 128

#define SWRESET 0x01
#define DISPON 0x29
#define SLEEPOUT 0x11
#define COLMOD 0x3A
#define MADCTL 36
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C


SPISettings MySPISettings(100000,MSBFIRST,SPI_MODE0);
static char RegMap[D_WIDTH*D_HEIGHT][2];

void WriteRegMap(){
  
}

void WriteCommand(int Command, char Params[] = NULL, int NumParams = 0){
  digitalWrite(DC, LOW);
  SPI.transfer(Command);
  for(int i{0}; i < NumParams; i++){
    SPI.transfer(Params[i]);
  }
}

void setup() {
  // put your setup code here, to run once:
  memset(RegMap,0,sizeof(RegMap)/sizeof(char)); // divide by sizeof(char) for clarity to remind myself
  pinMode(CS, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(RES, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(MySPISettings);
  digitalWrite(RES, LOW);
  delay(100);
  digitalWrite(RES, HIGH);
  digitalWrite(CS, LOW);
  digitalWrite(DC, LOW);
  delay(10);
  SPI.transfer(0x01); //sw reset
  delay(200);
  SPI.transfer(0x11); //sleep out
  SPI.transfer(0x29); //display on
  SPI.transfer(0x20); //display inversion

  SPI.transfer(0x3A);
  digitalWrite(DC, HIGH);
  SPI.transfer(0x05); //set color bits
  digitalWrite(DC, LOW);

  SPI.transfer(0x36); //memory access data control
  digitalWrite(DC, HIGH);
  SPI.transfer(0b01100000);
  digitalWrite(DC, LOW);

  SPI.transfer(0x2A); //memory access data control
  digitalWrite(DC, HIGH);
  SPI.transfer16(0x00);
  SPI.transfer16(0x9F);
  digitalWrite(DC, LOW);

  SPI.transfer(0x2B); //memory access data control
  digitalWrite(DC, HIGH);
  SPI.transfer16(0x00);
  SPI.transfer16(0x7F);
  digitalWrite(DC, LOW);

  SPI.transfer(0x2C); 
  digitalWrite(DC, HIGH);
  // for(int i = 0; i<D_WIDTH*D_HEIGHT/2*3; i++){
  //   SPI.transfer(0x00);
  // }
  for(int i = 0; i<D_WIDTH*D_HEIGHT; i++){
    SPI.transfer16(0x00);
  }
  for(int i = 0; i<10; i++){
    SPI.transfer16(0xffff);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
