#include <SPI.h>
#include <stdarg.h>

#define CS 10
#define RES 9
#define DC 8

#define D_WIDTH 160
#define D_HEIGHT 128

#define SWRESET 0x01
#define SLEEPOUT 0x11
#define DISPON 0x29
#define INVOFF 0x20
#define COLMOD 0x3A
#define MADCTL 0x36
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C

#define max(a,b) (abs(a) > abs(b) ? abs(a) : abs(b))


SPISettings MySPISettings(1000000,MSBFIRST,SPI_MODE0);
static char RegMap[D_WIDTH*D_HEIGHT];


void WriteCommand(unsigned char Command, unsigned char* Params = NULL, unsigned char NumParams = 0);

// template<typename command, typename... VarArgs>
// void WriteCommandVargs(command Command, VarArgs... Args);
template<typename type>
void Swap(type &a, type &b){
  type c = a;
  a = b;
  b = c;
}

void SetAddr(int RowStart, int ColStart, int RowEnd = 0x7F, int ColEnd = 0x9F){
  WriteCommandVargs(CASET, 0x00, RowStart, 0x00, RowEnd);
  WriteCommandVargs(RASET, 0x00, ColStart, 0x00, ColEnd);
}

unsigned int GenColor(unsigned int R, unsigned int G, unsigned int B){
  R = R <= 0x1F ? R : 0x1F;
  G = G <= 0x3F ? G : 0x3F;
  B = B <= 0x1F ? B : 0x1F;
  return((R<<11) + (G<<5) + B);
}

unsigned int ColorRatio(float R, float G, float B){
  R = R <= 1.0 ? R : 1.0;
  G = G <= 1.0 ? G : 1.0;
  B = B <= 1.0 ? B : 1.0;
  return GenColor(int(R * 31), int(G * 63), int(B * 31));
}

void DrawPixel(int x, int y, unsigned int Color){
  SetAddr(x,y);
  WriteCommand(RAMWR);
  SPI.transfer16(Color);
}

// void DrawLine(int StartX, int StartY, int EndX, int EndY, unsigned int Color){
//     int Cycles = signed_max((EndX - StartX),(EndY - StartY)), CycleSign = 1;
//     float Slope = float((EndY-StartY))/(EndX - StartX);
//     int StartMapping[] = {StartX, StartY};
//     int CoordMapping[] = {0,0}, default_mapping = 1;
//     if(Cycles < 0){
//         CycleSign = -1;
//         Cycles *= CycleSign;
//     }
//     if (abs(EndY - StartY) >= abs(EndX - StartX)){ //if line is longer vertically rise > run
//         Slope = 1/Slope;
//         default_mapping = 0;
//     }
//     for(int i = 0; i <= Cycles; i++){
//         CoordMapping[default_mapping] = int(StartMapping[default_mapping] + i*CycleSign); //default: CoordMapping[x,y] modified:CoordMapping[y,x]
//         CoordMapping[1 - default_mapping] = int(StartMapping[1-default_mapping] + i*CycleSign*Slope);

//         if(CoordMapping[0] >= 0 && CoordMapping[0] < 160 && CoordMapping[1] >= 0 && CoordMapping[1] < 128)
//         DrawPixel(CoordMapping[0], CoordMapping[1], Color);
//     }
// }
void do_swap(int* a, int* b){
  int c = *a;
  *a = *b;
  *b = c;
}
void DrawLine(int StartX, int StartY, int EndX, int EndY, unsigned int Color){
    if((abs(EndX - StartX) >= abs(EndY-StartY) && EndX < StartX) || (abs(EndX - StartX) < abs(EndY-StartY) && EndY < StartY)){
      Swap<int>(StartX, EndX);
      Swap<int>(StartY, EndY);
    }
    int Cycles = max((EndX - StartX),(EndY - StartY));
    float Slope = 0;
    int StartMapping[] = {StartX, StartY};
    int CoordMapping[] = {0,0}, default_mapping = 1;

    if (abs(EndY - StartY) > abs(EndX - StartX)){ //if line is longer vertically rise > run
      if (EndY-StartY != 0){
        Slope = float((EndX - StartX))/(EndY-StartY);
      }
    }else{
      if (EndX - StartX != 0){
        Slope = float((EndY-StartY))/(EndX - StartX);
      }
      default_mapping = 0;
    }
    for(int i = 0; i <= Cycles; i++){
        CoordMapping[default_mapping] = int(StartMapping[default_mapping] + i); //default: CoordMapping[x,y] modified:CoordMapping[y,x]
        CoordMapping[1 - default_mapping] = int(StartMapping[1-default_mapping] + i*Slope);

        if(CoordMapping[0] >= 0 && CoordMapping[0] < 160 && CoordMapping[1] >= 0 && CoordMapping[1] < 128)
        DrawPixel(CoordMapping[0], CoordMapping[1], Color);
    }
}

void DrawRect(int StartX, int StartY, int EndX, int EndY, int Color){
  DrawLine(StartX, StartY, EndX, StartY, Color);
  DrawLine(StartX, StartY, StartX, EndY, Color);
  DrawLine(StartX, EndY, EndX,EndY, Color);
  DrawLine(EndX, StartY, EndX, EndY, Color);
}

void WriteParams(){}

template<typename First, typename... VarArgs>
void WriteParams(First first, VarArgs... Args){
  SPI.transfer(first);
  WriteParams(Args...);
}

template<typename command, typename... VarArgs>
void WriteCommandVargs(command Command, VarArgs... Args){
  digitalWrite(DC, LOW);
  SPI.transfer(Command);
  digitalWrite(DC, HIGH);
  WriteParams(Args...);
}

void WriteCommand(unsigned char Command, unsigned char* Params = NULL, unsigned char NumParams = 0){
  digitalWrite(DC, LOW);
  SPI.transfer(Command);
  digitalWrite(DC, HIGH);
  for(int i = 0; i < NumParams; i++){
    SPI.transfer(Params[i]);
  }
}

void HWReset(){
  digitalWrite(RES, LOW);
  delay(10);
  digitalWrite(RES, HIGH);
}

void InitST7735(){
  pinMode(CS, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(RES, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(MySPISettings);
  HWReset();
  digitalWrite(CS, LOW);
  digitalWrite(DC, LOW);
  delay(10);
  unsigned char InitSequence[] = {
    1, SWRESET,
    1, SLEEPOUT,
    1, DISPON,
    1, INVOFF,
    2, COLMOD, 0x05,
    2, MADCTL, 0b10100000,
    5, CASET, 0x00,0x00, 0x00, 0x9F,
    5, RASET, 0x00,0x00, 0x00, 0x7F,
  };
  int Instruction = 0;
  while(Instruction < sizeof(InitSequence)/sizeof(char)){
    WriteCommand(InitSequence[Instruction + 1], &InitSequence[Instruction + 2], InitSequence[Instruction]-1);
    Instruction += InitSequence[Instruction] + 1;
  }
}

void setup() {
  // put your setup code here, to run once:
  // memset(RegMap,0,sizeof(RegMap)/sizeof(char)); // divide by sizeof(char) for clarity to remind myself
  InitST7735();
  WriteCommand(RAMWR);

  for(int i = 0; i<D_WIDTH*D_HEIGHT; i++){
    SPI.transfer16(0x00);
  }
  DrawRect(40,10,10,40,ColorRatio(0.2, 0.3, 0.4));
  DrawLine(10,40,50,10,ColorRatio(0.2, 0.3, 0.4));
  DrawLine(10,10,50,40,ColorRatio(0.2, 0.3, 0.4));

}

void loop() {
  // put your main code here, to run repeatedly:

}
