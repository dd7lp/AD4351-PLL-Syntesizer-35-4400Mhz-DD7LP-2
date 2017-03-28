//***********************************************************************************************************************************************
// ****** Modifikation der Routinen vorgesehen für ADF4351 DDS bearbeitet von DD7LP Christian Petersen www.darc-husum.de im Februar    2017 *****
// *******Wesentliche Programmteile, Funktionen und Änderungen programmiert von DJ7OO Klaus Hirschelmann http://www.kh-gps.de/ im Februar 2017 **
//  ******Programmroutinen für das Ansprechen des ADF4351 aus einer Software von OE6OCG  Richard Posch 8302 Nestelbach 8452. Austria ************
// Achtung, kommerzielle Verwertung diese Software ist nicht gestattet bedarf der schriftlichen Zustimmmmung der Autoren, bzw Programmierer *****
//***********************************************************************************************************************************************
// Das Display ist mit 1 Anschlussleiste siehe
// 1.8 Inch Mini Serial SPI TFT LCD Module Display with PCB Adapter ST7735B IC look Ebay Number : 71946479239
// All Data lines connect to Display via 1 Kohm, look to http://www.darc-husum.de/funktionsgenerator.html

// 1 = RST        Arduino Nano          Pin RST  **** muss nicht unbedingt verwendet werden
// 2 = CS                               Pin D9
// 3 = D/C  (A0)                        Pin D8
// 4 = Din   (Data Mosi)                Pin D11
// 5 = Clk   (Clock)                    Pin D13
// 6 = VCC 5 Volt ( oder 3v3 )
// 7 = LED über 100 Ohm an 5 Volt ( oder über 22 Ohm an 3v3 )
// 8 = GND
//************************************************************************************************************************************************

#include <SPI.h>
#include <Rotary.h>   //****************************** Rotary encoder: https://github.com/brianlow/Rotary 

// ********** legt die Pins fest für den Anschluss vom Display *********
#define dc 8                         
#define cs 9 
              
// include Adafruit library OR QDTech library depending on the display's controller chip.
#include <Adafruit_GFX.h>               //************** https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7735.h>            //************** https://github.com/adafruit/Adafruit-ST7735-Library

// Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst); // RST wurde gesperrt
Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc);

// ************************** TFT bildschirm Farben defenieren ****************************
#define BLACK    0x0000                   // Define the display colours we'll be using
#define BLUE     0x001F                   // so they're constants regardless of which
#define VIO      0xF8FF                   // display library we use.
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF
#define GREY     0x632C
//*****************************************************************************************

 unsigned long currentTime;
 unsigned long loopTime;
 const int pin_A = 2;  // pin 2
 const int pin_B = 3;  // pin 3
 unsigned char encoder_A;
 unsigned char encoder_B;
 unsigned char encoder_A_prev=0;

 unsigned long loopTime2;
 const int pin_A2 = 5;  // pin 5
 const int pin_B2 = 6;  // pin 6
 unsigned char encoder_A2;
 unsigned char encoder_B2;
 unsigned char encoder_A2_prev=0;

 boolean mrk1, mrk1_old, mrk2, mrk2_old;
  
 int press = 0;
 int cnt_step = 4;
 int cnt_step_old;
 int cnt_fix = 1;
 int cnt_fix_old;
 int cnt_pwr = 1;
 int cnt_pwr_old;
 int mdbm = 0; 
  
const int slaveSelectPin = 10;  // SPI-SS bzw. enable ADF4350  wurde von mir von pin 3 auf pin 10 geändert

long Freq = 14500000;      // Ausgangsfrequenz = 145.000 MHz  
long Freq_Old;
long refin = 1000000;      // Referenzquarz = 10 Mhz
long ChanStep = 1250;      // Kanalraster = 12.5 Khz 
unsigned long Reg[6];      // ADF4351 Reg's
int pegelZ = 0;            // Zähler konstante für Ausgangspegel-Zähler 
int outPegel = mdbm;       // Ausgangspegel beim start = -4 dBm
const int CLK = 13;        // CLK and DATA pins are shared with the TFT display.
const int DATA = 11;

///////////////////////// Subroutine: Setze Frequenz ADF4351 ///////////////////////////
void SetFreq(long Freq)
{
  Serial.print("FRQ: ");
  Serial.println(Freq);
  
  ConvertFreq(Freq, Reg);
  WriteADF2(5);
  delayMicroseconds(2500);
  WriteADF2(4);
  delayMicroseconds(2500);
  WriteADF2(3);
  delayMicroseconds(2500);
  WriteADF2(2);
  delayMicroseconds(2500);
  WriteADF2(1);
  delayMicroseconds(2500);
  WriteADF2(0);
  delayMicroseconds(2500);
}

////////////////////////// Teil-Subroutine ADF4351 ////////////////////////////
void WriteADF2(int idx)
{ // make 4 byte from integer for SPI-Transfer
  byte buf[4];
  for (int i = 0; i < 4; i++)
    buf[i] = (byte)(Reg[idx] >> (i * 8));
  WriteADF(buf[3], buf[2], buf[1], buf[0]);
}

/////////////////////////// Teil-Subroutine ADF4351 ////////////////////////////
int WriteADF(byte a1, byte a2, byte a3, byte a4) {
 // write over SPI to ADF4350
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(10);
  SPI.transfer(a1);
  SPI.transfer(a2);
  SPI.transfer(a3);
  SPI.transfer(a4);
  Toggle();
}

///////////////////////////// Teil-Subroutine ADF4351 ////////////////////////////
int Toggle() {
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(slaveSelectPin, LOW);
}

////////////////////////////// Teil-Subroutine ADF4351 //////////////////////////
void ConvertFreq(long freq, unsigned long R[])
{
  // PLL-Reg-R0         =  32bit
  // Registerselect        3bit
  // int F_Frac = 4;       // 12bit
  // int N_Int = 92;       // 16bit
  // reserved           // 1bit

  // PLL-Reg-R1         =  32bit
  // Registerselect        3bit
  // int M_Mod = 5;        // 12bit
  int P_Phase = 1;         // 12bit bei 2x12bit hintereinander pow()-bug !!
  int Prescal = 0;         // 1bit geht nicht ???
  int PhaseAdj = 0;        // 1bit geht auch nicht ???
  // reserved              // 3bit

  // PLL-Reg-R2         =  32bit
  // Registerselect        3bit
  int U1_CountRes = 0;     // 1bit
  int U2_Cp3state = 0;     // 1bit
  int U3_PwrDown = 0;      // 1bit
  int U4_PDpola = 1;       // 1bit
  int U5_LPD = 0;          // 1bit
  int U6_LPF = 1;          // 1bit 1=Integer, 0=Frac not spported yet
  int CP_ChgPump = 7;      // 4bit
  int D1_DoublBuf = 0;     // 1bit
  //  int R_Counter = 1;   // 10bit
  //  int RD1_Rdiv2 = 0;   // 1bit
  //  int RD2refdoubl = 0; // 1bit
  int M_Muxout = 0;        // 3bit
  int LoNoisSpur = 0;      // 2bit
  // reserved              // 1bit

  // PLL-Reg-R3         =  32bit
  // Registerselect        3bit
  int D_Clk_div = 150;     // 12bit
  int C_Clk_mode = 0;      // 2bit
  //  reserved             // 1bit
  int F1_Csr = 0;          // 1bit
  //  reserved             // 2bit
  int F2_ChgChan = 0;      // 1bit
  int F3_ADB = 0;          // 1bit
  int F4_BandSel = 0;      // 1bit
  //  reserved             // 8bit

  // PLL-Reg-R4         =  32bit
  // Registerselect        3bit
  int D_out_PWR = (mdbm);      // 2bit  OutPwr 0-3 3= +5dBm   Power out 1
  int D_RF_ena = 1;            // 1bit  OutPwr 1=on           0 = off  Outport Null freischalten 
  int D_auxOutPwr = (mdbm);    // 2bit  aux OutPwr 0-3        Power out 2  
  int D_auxOutEna = 1;         // 1bit  aux OutEna 1=on       0 = off  Outport Aux freischalten 
  int D_auxOutSel = 1;         // 1bit  aux OutSel
  int D_MTLD = 0;              // 1bit
  int D_VcoPwrDown = 0;        // 1bit 1=VCO off

  //  int B_BandSelClk = 200; // 8bit
  int D_RfDivSel = 3;      // 3bit 3=70cm 4=2m
  int D_FeedBck = 1;       // 1bit
  // reserved              // 8bit

  // PLL-Reg-R5         =  32bit
  // Registerselect        // 3bit
  // reserved              // 16bit
  // reserved     11       // 2bit
  // reserved              // 1bit
  int D_LdPinMod = 1;      // 2bit muss 1 sein
  // reserved              // 8bit

  // Referenz Freg Calc
  int R_Counter = 1;       // 10bit
  int RD1_Rdiv2 = 0;       // 1bit
  int RD2refdoubl = 0;     // 1bit
  int B_BandSelClk = 200;  // 8bit
  //  int F4_BandSel = 0;  // 1bit

// int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;
  long RFout = Freq;       // VCO-Frequenz

// calc bandselect und RF-div
  int outdiv = 1;
  if (RFout >= 220000000) {
    outdiv = 1;
    D_RfDivSel = 0;
  }
  if (RFout < 220000000) {
    outdiv = 2;
    D_RfDivSel = 1;
  }
  if (RFout < 110000000) {
    outdiv = 4;
    D_RfDivSel = 2;
  }
  if (RFout < 55000000) {
    outdiv = 8;
    D_RfDivSel = 3;
  }
  if (RFout < 27500000) {
    outdiv = 16;
    D_RfDivSel = 4;
  }
  if (RFout < 13800000) {
    outdiv = 32;
    D_RfDivSel = 5;
  }
  if (RFout < 6900000) {
    outdiv = 64;
    D_RfDivSel = 6;
  }

  float PFDFreq = refin * ((1.0 + RD2refdoubl) / (R_Counter * (1.0 + RD1_Rdiv2))); //Referenzfrequenz
  float N = ((RFout) * outdiv) / PFDFreq;
  int N_Int = N;
  long M_Mod = PFDFreq * (100000 / ChanStep) / 100000;
  int F_Frac = round((N - N_Int) * M_Mod);

  R[0] = (unsigned long)(0 + F_Frac * pow(2, 3) + N_Int * pow(2, 15));
  R[1] = (unsigned long)(1 + M_Mod * pow(2, 3) + P_Phase * pow(2, 15) + Prescal * pow(2, 27) + PhaseAdj * pow(2, 28));
//  R[1] = (R[1])+1; // Registerselect adjust ?? because unpossible 2x12bit in pow() funktion
  R[2] = (unsigned long)(2 + U1_CountRes * pow(2, 3) + U2_Cp3state * pow(2, 4) + U3_PwrDown * pow(2, 5) + U4_PDpola * pow(2, 6) + U5_LPD * pow(2, 7) + U6_LPF * pow(2, 8) + CP_ChgPump * pow(2, 9) + D1_DoublBuf * pow(2, 13) + R_Counter * pow(2, 14) + RD1_Rdiv2 * pow(2, 24) + RD2refdoubl * pow(2, 25) + M_Muxout * pow(2, 26) + LoNoisSpur * pow(2, 29));
  R[3] = (unsigned long)(3 + D_Clk_div * pow(2, 3) + C_Clk_mode * pow(2, 15) + 0 * pow(2, 17) + F1_Csr * pow(2, 18) + 0 * pow(2, 19) + F2_ChgChan * pow(2, 21) +  F3_ADB * pow(2, 22) + F4_BandSel * pow(2, 23) + 0 * pow(2, 24));
  R[4] = (unsigned long)(4 + D_out_PWR * pow(2, 3) + D_RF_ena * pow(2, 5) + D_auxOutPwr * pow(2, 6) + D_auxOutEna * pow(2, 8) + D_auxOutSel * pow(2, 9) + D_MTLD * pow(2, 10) + D_VcoPwrDown * pow(2, 11) + B_BandSelClk * pow(2, 12) + D_RfDivSel * pow(2, 20) + D_FeedBck * pow(2, 23));
  R[5] = (unsigned long)(5 + 0 * pow(2, 3) + 3 * pow(2, 19) + 0 * pow(2, 21) + D_LdPinMod * pow(2, 22));
}
// to do instead of writing 0x08000000 you can use other two possibilities: (1ul << 27) or (uint32_t) (1 << 27).

//////////////////////////////////////////////////////////////////////////////
//                                      Setup                               //  
//////////////////////////////////////////////////////////////////////////////
void setup() { 
  Serial.begin(9600);    // USB to PC for Debug only
  Serial.println("Start");
  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, LOW);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.begin();

  pinMode(pin_A, INPUT_PULLUP);
  pinMode(pin_B, INPUT_PULLUP);
  pinMode(pin_A2, INPUT_PULLUP);
  pinMode(pin_B2, INPUT_PULLUP);

  pinMode(4,INPUT_PULLUP);      // fix channel select   
  pinMode(7,INPUT_PULLUP);      // power select
  pinMode(12, INPUT_PULLUP);    // lock/unlock
  pinMode(14, INPUT_PULLUP);    // intref/extref via #A0  *************************Pin für int/ext Referenz *******************
 // pinMode(21, INPUT_PULLUP);    // intref/extref via #A7
 
  currentTime = millis();
  loopTime = currentTime; 
  loopTime2 = currentTime; 
        
// SPI Mode kann noch nicht gesetzt werden, da Display und DDS unterschiedliche SPI-MODES haben *********
  SPI.begin();
  delay(50); 
  tft.initR(INITR_BLACKTAB);       // initialize a ST7735S chip, schwarz tab
  tft.setRotation(3);              // Bestimmt die Lage auf dem Display von 0-3
  tft.setTextWrap(false);          // Allow text to run off right edge
  tft.fillScreen(BLACK);

// ********************* Bildschirm Maske statischer Text *********************
  tft.setTextColor(WHITE); 
  tft.setTextSize(1);
  tft.setCursor(55, 10);
  tft.print("DIGITALER");
     tft.setCursor(35, 20);
  tft.print("SIGNALGENERATOR");
  tft.drawFastHLine(0,35,160,BLUE);
  tft.drawFastHLine(0,100,160,BLUE);
  tft.setTextColor(VIO);
  tft.setCursor(25,110);
  tft.println("ADF4351 35-4400 MHz ");
  tft.setCursor(7,120); 
  tft.print("DD7LP - Elektronik - 2017");

// sende Strartfrequenz zum ADF4351
    delay(500);
    SetFreq(Freq);
    delay(50);
    updateDisplay();
}

// *********************** Subroutie: update Display  **************************
void updateDisplay() {

// Zum Verständnis das Display benutzt SPI Mode0 das ADF4351 SPI MODE3  
// ************* Wichtig ist das umschalten vor der Ausgabe  ************
  SPI.setDataMode(SPI_MODE0);  
  tft.fillRect(1, 45, 160, 25, BLACK);            
  tft.setTextColor(YELLOW);
  tft.setCursor(18, 50);
  tft.setTextSize(1); 

  if(mdbm==0) { tft.print("Output Pegel = -4 dBm "); }
  else
  if(mdbm==1) { tft.print("Output Pegel = -1 dBm "); }
  else
  if(mdbm==2) { tft.print("Output Pegel = +2 dBm "); }
  else
  if(mdbm==3) { tft.print("Output Pegel = +5 dBm "); }
   
  tft.setCursor(18, 60);
  tft.setTextSize(1); 
  tft.print("Step = ");
    float ChanStep2 = ChanStep;
  if(ChanStep2<100000)
  { ChanStep2 = ChanStep2/100;
  tft.print(ChanStep2);
  tft.print(" KHz"); }
  else
  { ChanStep2 = ChanStep2/100000;
  tft.print(ChanStep2,0);
  tft.print(" MHz"); }
tft.fillRect(1, 80, 160, 14, BLACK);           
  tft.setTextColor(GREEN);
  tft.setTextSize(2); 
  tft.setCursor(12, 80);
  float Freq2;
  Freq2 = Freq;
  Freq2 = Freq2/100000;
  if(Freq2<1000) {tft.print(Freq2,4);}
  else  
  { tft.print(Freq2,3); } 
  tft.print(" MHz"); 
}

////////////////////////////////////////////////////////////////////////
//                      HAUPTPROGRAMMSCHLEIFE                         //
////////////////////////////////////////////////////////////////////////
void loop() {  
  rotary_enc2();
  if(cnt_step!=cnt_step_old) { updateDisplay(); cnt_step_old=cnt_step; } 

  fixfrq_select();
  if(cnt_fix!=cnt_fix_old) { updateDisplay(); cnt_fix_old=cnt_fix; } 

  pwr_select();
  if(cnt_pwr!=cnt_pwr_old) { updateDisplay(); cnt_pwr_old=cnt_pwr; SetFreq(Freq); } 

  tft.setTextColor(GREEN);
  tft.setTextSize(1); 
  tft.setCursor(5, 5);

  if(digitalRead(12)==HIGH)      // select lock/unlock
{ mrk1=1; } else { mrk1=0; }  
  if(mrk1!=mrk1_old) { 
  tft.fillRect(3, 3, 45, 13, BLACK); 
  if(digitalRead(12)==HIGH) 
  { tft.print("LOCK");   } 
   else
  { tft.setTextColor(RED); 
    tft.print("UNLOCK"); } 
  } 
  mrk1_old=mrk1; 
   
  if(digitalRead(14)==HIGH)  //  select intref/extref via #A0 
{ mrk2=1; } else { mrk2=0; }  
  if(mrk2!=mrk2_old) { 
  tft.fillRect(115, 3, 160, 13, BLACK); 
  tft.setCursor(120, 5);
  if(digitalRead(14)==HIGH)    // select intref/extref via #A0
  { tft.print("REF.int"); }    // REFINT = interne 10 Mhz Referenz für die PLL vom AD4351
   else
  {tft.setTextColor(RED);
    tft.print("REF.ext"); }    //REFEXT = externe 10 Mhz Referenz zum synchronisieren der PLL
  } 
  mrk2_old=mrk2; 
      
  rotary_enc();
  if (Freq != Freq_Old) {
  updateDisplay();
  SetFreq(Freq);
//  Serial.println(Freq);
  Freq_Old = Freq;
  }
 }
//*********************************************************** Ende der Hauptprogrammschleife  ********************************************

/////////////////////////////// Subroutine: Frequenz select /////////////////////////////////
void rotary_enc()
{
     currentTime = millis();
  if(currentTime >= (loopTime + 2)){
     encoder_A = digitalRead(pin_A);    
     encoder_B = digitalRead(pin_B);   
  if((!encoder_A) && (encoder_A_prev)){
  if(encoder_B) {
     Freq = Freq+ChanStep;               
     if(Freq>440000000) {Freq=3500000;}   
      }   
     else {
     Freq = Freq-ChanStep;               
     if(Freq<3500000) {Freq=440000000;} 
      }   
    }   
     encoder_A_prev = encoder_A;     // Store value of A for next time       
     loopTime = currentTime;         // Updates loopTime
    }
  }

/////////////////////////////// Subroutine: Step select ////////////////////////////////
void rotary_enc2()
{
     currentTime = millis();
  if(currentTime >= (loopTime2 + 2)){
     encoder_A2 = digitalRead(pin_A2);    
     encoder_B2 = digitalRead(pin_B2);   
  if((!encoder_A2) && (encoder_A2_prev)){
  if(encoder_B2) {
        cnt_step = cnt_step+1;
        if(cnt_step>8) {cnt_step=0;}   
      }   
      else {
        cnt_step= cnt_step-1;               
        if(cnt_step<0) {cnt_step=8;} 
      }   
    }   
    encoder_A2_prev = encoder_A2;     // Store value of A for next time       
    loopTime2 = currentTime;         // Updates loopTime
    }
// Serial.println(cnt_step);
  if(cnt_step==0) { ChanStep=100; }     // 1 KHz
  else
  if(cnt_step==1) { ChanStep=625; }     // 6.25 KHz
  else
  if(cnt_step==2) { ChanStep=1000; }    // 10 KHz 
  else
  if(cnt_step==3) { ChanStep=1250; }    // 12.5 KHz
  else
  if(cnt_step==4) { ChanStep=2500; }    // 25 KHz
  else
  if(cnt_step==5) { ChanStep=10000; }   // 100KHz
  else
  if(cnt_step==6) { ChanStep=100000; }  // 1 MHz 
  else
  if(cnt_step==7) { ChanStep=1000000; } // 10 MHz
  else
  if(cnt_step==8) { ChanStep=10000000; } // 100 MHz
  }

/////////////////////////// Subroutine: Fixfrequenzen select ////////////////////////////
void fixfrq_select()
 {
 press = digitalRead(4);
  if (press == LOW)
  {
//  Serial.println(cnt_fix);
  if(cnt_fix==0) { Freq=3500000; }    // 35.0 MHz
  else
  if(cnt_fix==1) { Freq=5050000; }    // 50.5 MHz
  else
  if(cnt_fix==2) { Freq=14500000; }   // 145.0 MHz
  else
  if(cnt_fix==3) { Freq=43500000; }   // 435.0 MHz
  else
  if(cnt_fix==4) { Freq=129600000; }  // 1296.0 MHz
  else
  if(cnt_fix==5) { Freq=150000000; }  // 1500.0 MHz
  else
  if(cnt_fix==6) { Freq=200000000; }  // 2000.0 MHz
  else
  if(cnt_fix==7) { Freq=250000000; }  // 2500.0 MHz  
  else
  if(cnt_fix==8) { Freq=300000000; }  // 3000.0 MHz
  else
  if(cnt_fix==9) { Freq=350000000; }  // 3500.0 MHz
  else
  if(cnt_fix==10) { Freq=400000000; }  // 4000.0 MHz
  else
  if(cnt_fix==11) { Freq=440000000; }  // 4400.0  MHz
  cnt_fix = cnt_fix+1; 
  if(cnt_fix==12) { cnt_fix=0 ; }
  delay(300);
    }
 }  

////////////////////////////////// Subroutine: Power select ///////////////////////////////
void pwr_select()
 { 
  press = digitalRead(7);
  if (press == LOW)
  {
//  Serial.println(pwr_step);
  if(cnt_pwr==0) { mdbm = 0; }    // -4dBm
  else
  if(cnt_pwr==1) { mdbm = 1; }    // -1dBm
  else
  if(cnt_pwr==2) { mdbm = 2; }    // +2dBm
  else
  if(cnt_pwr==3) { mdbm = 3; }    // +5dBm
  cnt_pwr = cnt_pwr+1; 
  if(cnt_pwr==4) { cnt_pwr=0 ; }
  delay(300);
    }
  }  
//////////////////////////////////////////////////////////////////////////////////////
