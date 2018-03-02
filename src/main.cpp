

/*
ad9851 informations:
Based on DS9851 documentation:
fout=(delta_phase*system_clock)/2^32
where fout and system_clock is in [Mhz]
delta_phase: counter/variable what is set
2^32=4294967296
system_clock=6*30Mhz=180[Mhz]
example: to set fout=1Mhz delta_phase should be:
dleta_phase=fout*2^32/system_clock
delta_phase=1*4294967296/180
delta_phase=23860929
*/

//TODO: fix probelm with standard LiquidCrystal_I2C.h library (used as dependency has some compilatioin errors)
//TODO: use english coments
//TODO: use oversampling https://github.com/stylesuxx/Oversample or external 12bit ADC
//TODO: do not use TimeONe lib (direct configuration needed)

#include <avr/pgmspace.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AD9850SPI.h>
#include <SPI.h>
#include <EEPROM.h>
#include <ClickEncoder.h>
#include <TimerOne.h>


#include "main.h"


unsigned char Mode;       //mode powerdown ou normal
unsigned long  StartF;    //DS_FTW fréquence de départ
unsigned int NumberF;     //nombre d'échantillons
unsigned long  StepF;     //Fréquence d'incrémentation en FTW (non en HZ)

unsigned int intTemp;    //variable de boucle pour le balayage
unsigned int adcmag;     //variables des 2 ADC measures
unsigned int adcphs;
boolean check=0;

float calMag;
float calPhs;

vector_reflection point;

volatile long freq = 5000000;
byte vnaMode=0;
volatile byte menuSwapp=0;
byte menuChoose=0;
byte bandChoose;
volatile byte bandSwapp=0;
volatile byte bandSwappPrec=11;
double dds_reg;

long freq_prec = 0;
long freqStep = 0;


LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //TODO: remove POSITIVE
ClickEncoder *encoder;
ClickEncoder::Button b;


volatile int16_t rotary_encoder_last, rotary_encoder_value;

/********************************************************
 * interrupts
 ********************************************************/
void doEncoder() { //TODO: here only change rotarty counters menu/band varabiles migrate to their main places

  rotary_encoder_value=encoder->getValue();
  if(rotary_encoder_value!=0){
    menuSwapp=(menuSwapp+1)%4;
    freq=freq+freqStep*rotary_encoder_value;
    if (freq>freqMax) freq=freqMax;
    if (freq<freqMin) freq=freqMin;
  }

}

void timerIsr() {
  encoder->service();
}

void setup()
{
  lcd.begin(4, LCD_CHARS_IN_ROW);  //4 lines *20 columns lcd char
  lcd.setBacklight(HIGH);
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(Rele, OUTPUT);
  DDS.begin(13,8,7);  //Ports connected to DDS W_CLK, FQ_UD et RESET
  //DDS.calibrate(180000000);    //pas utile dans l'immédiat mais on ne sait jamais
  analogReference(EXTERNAL);  //tension de référence extérieure 1.8V de l'ad8302
  dds_reg=((double)freq) * POWER32 /SYSTEM_CLOCK;  //standard conversion t
  DDS.vna(dds_reg,REFCLOCK6);
  delay(1);
  DDS.vna(dds_reg,PowerDown);
  pinMode(ENCODER_PORT_A, INPUT);
  digitalWrite(ENCODER_PORT_A, HIGH);
  pinMode(ENCODER_PORT_B, INPUT);
  digitalWrite(ENCODER_PORT_B, HIGH);
  pinMode(ENCODER_BUTTON, INPUT);
  digitalWrite(ENCODER_BUTTON, HIGH);

  adcmag=EEPROM.read(2)*256+EEPROM.read(1);
  adcphs=EEPROM.read(4)*256+EEPROM.read(3);
  calMag=((float)adcmag*adc2Db)+offsetDb;
  calPhs=((float)adcphs*Adc2Angle);
  //Serial.println(calMag);              //verif calibration
  //Serial.println(calPhs);

  lcd.clear();
  lcd.print(F("VNA v1.1"));    //intro
  lcd.setCursor(0, 1);
  lcd.print(F("SQ6KXQ 2018"));
  lcd.setCursor(0, 2);
  lcd.print(F("Boot menu->hold key"));

  delay(2000);
  lcd.clear();

  encoder = new ClickEncoder(ENCODER_PORT_A, ENCODER_PORT_B, ENCODER_BUTTON);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  boot_menu();
  menuChoose=EEPROM.read(0);
  switch(menuChoose)
  {
  case 0 :  menuJvna(0);   break;          //pc
  case 1 :  menuJvna(1);  check=0; break;  //bluetooth
  case 2 :  bandSelect(); break;           //standalone
}
}

//main loop

void loop()
{
  switch(menuChoose)
  {
    case 0 :  Jnva();   break;      //jvna
    case 1 :  Jnva();   break;      //bluetooth
    case 2 :  sweep();  break;      //standalone
  }
}


/********************************************************
 * Jvna
 ********************************************************/
void Jnva()
{
  char Temp;
  Serial.flush();
  Temp = DecodeCom(); //acuisition et décodage d'une demande de measure en provenance du logiciel IG_miniVNA
  if (menuChoose==1)
  {
    StartF=(StartF/9)*20;
    StepF=(StepF/9)*20;
  }
  affiche_freqs();    //pour debug

 if ((Temp==0) && (NumberF!=0)) // si le décodage est bon et que le nombre d'échantillons n'est pas nul alors measure
 {
   digitalWrite(LED, HIGH);      //vérification visuelle de la measure
   if (Mode == 0) digitalWrite(Rele, LOW);   // commande relais tansmission 0 ou refection 1
   else digitalWrite(Rele, HIGH);
   for (intTemp=0; intTemp < NumberF; intTemp++)    //boucle des measures
   {
     if ((StartF >= MinFrq) && (StartF <= MaxFrq))
     {
       Mode = REFCLOCK6;   // pour faire une measure  F > 0,5 mhz
      }else
      {
        Mode = PowerDown;  // F < 0,5 mhz, sinon Power Down ajouter test si F>fmax alors power down
      }
      DDS.vna(StartF,Mode);       //fonction dédiée au vna pour le dds 9851 meme si la lib est 9850
                                      // si le 9850 est utilisé, Normal doit etre à 0
            //delay(1);
      magPhsADC();
      Serial.write((byte)adcphs);     // LSB
      Serial.write((byte)(adcphs>>8));     // MSB
      Serial.write((byte)adcmag);     // LSB
      Serial.write((byte)(adcmag>>8));     // MSB
      StartF = StartF+StepF;
    }
  }
  digitalWrite(LED, LOW);        //fin de measure
}

void magPhsADC()  //TODO: use oversampling
{
  unsigned int amp;
  unsigned int phs;
  amp=0;
  phs=0;
  for (int n=0;n<20;n++)
  {
    amp = amp +  analogRead(ADC0);   //measure 10 bits
    phs = phs + analogRead(ADC1);
  }
  adcmag=amp/20;
  adcphs=phs/20;
}

// a virer plus tard
void debug_serial(void)
{
  Serial.print(F("Mode :"));
  Serial.println(Mode);
  Serial.print(F("Start F :"));
  Serial.println(StartF);
  Serial.print(F("Number F :"));
  Serial.println(NumberF);
  Serial.print(F("StepF :"));
  Serial.println(StepF);
}

// traitement chaine série

char DecodeCom (void)
{
  char data, i, err=0,  Param[11];
  data = getRX();                           // récupère un caractère réflextion , transmission
  switch (data)
  {
    case '0': Mode = 0; break;
    case '1': Mode = 1; break;
    default: err = 1;
  }
  data = getRX();                           // ok c'est un retour chariot
  if  (data !=0x0D) err = 1;
  if  (err != 0)  return (err);               // ou erreur
  for (i = 0; i <= 10; i++)                  // lire l'info sur le mot DS_FTW de start
  {
    data = getRX();
    if (data == 0x0D) break;                  // sort de la boucle si retour chariot
    if (isdigit(data) ==1) Param[i] = data;    // vérification si c'est un un chiffre
    else  err = 1;                         // sinon erreur
  }
  if  ((i==0) | (i>10))  err = 1;              // erreur si DS_FTW start nul ou trop long
  Param[i] = 0;                                // char nul avant conversion
  if  (err != 0)  return (err);                //  return si erreur
  StartF = atol (Param);                       // conversion en unsigned long
  for (i = 0; i <= 5; i++)                   // meme principe avec le  nombre d'échantillons
  {
    data = getRX();
    if (data == 0x0D) break;
    if (isdigit(data) ==1) Param[i] = data;
    else  err = 1;
  }
  if  ((i==0) | (i>5))  err = 1;
  Param[i] = 0;
  if  (err != 0)  return (err);
  NumberF = atoi (Param);
  for (i = 0; i <= 10; i++)                  // meme principe avec le  DS_FTW  step
  {
    data = getRX();
    if (data == 0x0D) break;
    if (isdigit(data) ==1) Param[i] = data;
    else  err = 1;
  }
  if  ((i==0) || (i>10))  err = 1;
  Param[i] = 0;
  if  (err != 0)  return (err);
  StepF = atol (Param);
  return (0);
}

/*
mode bluetooth 115200 bauds
use AT+BAUDX8 to change baud rate of the bluetooth device.
<\r><\n>CONNECT,BCCFCC36B9BC<\r> <\n>0<\r>0<\r>1<\r>0<\r>
0<\r>1073804<\r>1000<\r>1931773<\r>
<\r><\n>DISCONNECT<\r><\n>
*/

char getRX(void)
{
  char data=0;
  do
  {
    if (Serial.available())
    {
      data=Serial.read();
      if(check==0)
      {
        if (data==0x0D)
        {
          data=0;
          while (data!=0x0d)
          {
            if (Serial.available()) data=Serial.read();
          }
          while (data!='0')
          {
            if (Serial.available()) data=Serial.read();
          }
        }
      }
      check=1;
    }
}
while((data != 0x0D)&&(isdigit(data) !=1));
return data;
}


void affiche_freqs(void)
{
  if ((StartF>=MinFrq) && (StartF<=MaxFrq)) {
     delete_char(1,8,19);
     lcd.setCursor(8,1);
     lcd.print((unsigned long)ticksToFreq(StartF));
     lcd.print(" Hz");
     delete_char(2,8,19);
     lcd.setCursor(8,2);
     lcd.print((unsigned long)ticksToFreq(StepF));
     lcd.print(" Hz");
     delete_char(3,8,19);
     lcd.setCursor(8,3);
     lcd.print(NumberF);
   }
}

double ticksToFreq(long f)
{
 return ((double)f) * SYSTEM_CLOCK / POWER32;  //TODO: magic number
}


void menuJvna(byte PB)
{
  lcd.clear();
  lcd.print(PB==0?F("JVna PC"):F("Blue Vna Android"));
  lcd.setCursor(0, 1);
  lcd.print(F("Fstart:"));
  lcd.setCursor(0, 2);
  lcd.print(F("Fstep:"));
  lcd.setCursor(0, 3);
  lcd.print(F("Samples:"));
}

/********************************************************
 * Bluetooth
 ********************************************************/

/********************************************************
 * StandAlone
 ********************************************************/
void sweep()
{
char tab[10];

if (freq!=freq_prec)
{
   //double dds_reg=((double)freq) * 4294967296.0 / 180000000.0;
   double dds_reg=((double)freq) * POWER32 / SYSTEM_CLOCK;
   Serial.print('>');
   Serial.print(dds_reg);
   Serial.print('<');
   DDS.vna(dds_reg,REFCLOCK6);
   freq_prec=freq;
   BCD(freq,tab);
   lcd.setCursor(5, 0);
   lcd.write(tab[7]);
   lcd.write(tab[6]);
   lcd.write(46);
   lcd.write(tab[5]);
   lcd.write(tab[4]);
   measure();
 }
 b=encoder->getButton();
 if(b==ClickEncoder::Clicked)
 {
   bandSelect();
 }
}

void BCD (unsigned long b, char* o)
{
   for (int i=10; i; --i)
   {
      *o = (b % 10)+48;
      b /= 10;
      o++;
   }
}


void measure()
{

magPhsADC();
calculDut(adcmag,adcphs);
vna_print();
delete_char(1,3,9);
lcd.setCursor(3, 1);
lcd.print((int)point.RL);
lcd.print(F("dB"));
delete_char(2,4,9);
lcd.setCursor(4, 2);
lcd.print((int)point.Phi);
lcd.write(0xdf);
delete_char(1,13,19);
lcd.setCursor(13, 1);
lcd.print((int)point.Rs);
lcd.write(0xf4);
delete_char(2,13,19);
lcd.setCursor(13, 2);
lcd.print((int)point.Xs);
lcd.write(0xf4);
delete_char(3,2,9);
lcd.setCursor(2, 3);
lcd.print((int)point.Z);
lcd.write(0xf4);
delete_char(3,14,19);
lcd.setCursor(14, 3);
if (point.Swr>=1)
{
lcd.print((int)point.Swr);
lcd.write(46);
byte tempSwr=(int)((point.Swr-(int)(point.Swr))*100);
if (tempSwr/10==0) lcd.write(48);
lcd.print(tempSwr);
}
}

void delete_char(byte line, byte start, byte end)
{
while(start<=end) {
                  lcd.setCursor(start, line);
                  lcd.write(32);
                  start++;
                  }
}


/********************************************************
 * Compute Vna datas
 ********************************************************/
 /*
//formulas
RL=-20log(Rho)
Rho=10^(Rl/-20)
Z=(ZL-ZO)/(ZL+Z0) avec Z0=50ohms
Z=a+jb avec
a=Rho*cos(phi)
b=Rho*sin(phi)
ZL=(1+Z)/(1-Z)*Z0
ZL=RS+jXS avec
RS=abs(1-a²-b²)/((1-a)²+b²)
XS=abs(2b/((1-a)²-b²))
|Z|=sqrt(RS²+XS²)
SWR=(1+Rho)/(1-Rho)
*/

void calculDut(int adcMag, int adcPhs)
{
  point.Freq=freq;
  point.RL=((float)adcMag*adc2Db)+offsetDb-calMag;
  point.Phi=((float)adcPhs*Adc2Angle)-calPhs;
  point.Rho=pow(10.0,point.RL/-20.0);
  float re=point.Rho*cos(point.Phi * D2R);
  float im=point.Rho*sin(point.Phi * D2R);
  float denominator=((1-re)*(1-re)+(im*im));
  point.Rs=fabs((1-(re*re)-(im*im))/denominator)*50.0;
  point.Xs=fabs(2.0*im)/denominator*50.0;
  point.Z=sqrt(point.Rs*point.Rs+point.Xs*point.Xs);
  point.Swr=fabs(1.0+point.Rho)/(1.001-point.Rho);
  point.RL*=-1;
}

void vna_print()
{
  Serial.print(freq);
  Serial.write(9);
  Serial.print(adcmag);
  Serial.write(9);
  Serial.print(adcphs);
  Serial.write(9);
  Serial.print(point.RL);
  Serial.write(9);
  Serial.print(point.Phi);
  Serial.write(9);
  Serial.print(point.Rho);
  Serial.write(9);
  Serial.print(point.Rs);
  Serial.write(9);
  Serial.print(point.Xs);
  Serial.write(9);
  Serial.print(point.Z);
  Serial.write(9);
  Serial.println(point.Swr);
}

/********************************************************
 * Lcd menus
 ********************************************************/
void lcd_menu_analyse_refection()
{
  lcd.clear();
  lcd.print(F("FREQ:"));
  lcd.setCursor(11, 0);
  lcd.print(F("MHz"));
  lcd.setCursor(0, 1);
  lcd.print(F("RL:"));
  lcd.setCursor(10, 1);
  lcd.print(F("RS:"));
  lcd.setCursor(0, 2);
  lcd.print(F("Phi:"));
  lcd.setCursor(10, 2);
  lcd.print(F("XS:"));
  lcd.setCursor(0, 3);
  lcd.print(F("Z:"));
  lcd.setCursor(10, 3);
  lcd.print(F("SWR:"));
}

void boot_menu()
{
  uint8_t const menu_items_number=3;
  if (digitalRead(ENCODER_BUTTON)==1) return; //if rotary button is not pressed do not show the menu above
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("JVna PC"));
  lcd.setCursor(1, 1);
  lcd.print(F("Blue Vna"));
  lcd.setCursor(1, 2);
  lcd.print(F("Standalone Reflect."));
  lcd.setCursor(1, 3);
  lcd.print(F("Calib. V1.0"));
  while(digitalRead(ENCODER_BUTTON)==0) {}  //BP rise down detect
  while(digitalRead(ENCODER_BUTTON)==1) {
  for (byte n=0;n<=menu_items_number;n++)
  {
    lcd.setCursor(0, n);
    if (menuSwapp==n) lcd.write(42); else lcd.write(32);
  }
  }
  menuChoose=menuSwapp;
  for (uint8_t n=0;n<menu_items_number;n++)
  {
    if (menuChoose!=n)
    {
      for (uint8_t m=0;m<LCD_CHARS_IN_ROW;m++)
      {
        lcd.setCursor(m, n);
        lcd.write(32);
      }
    }
  }
  delay(1000);
  if (menuChoose<3) EEPROM.write(0,menuChoose); else calibration();
  return;
  //resetFunc();  //call reset
}


void bandSelect()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("160m  80m   60m"));
  lcd.setCursor(1, 1);
  lcd.print(F(" 40m  30m   20m"));
  lcd.setCursor(1, 2);
  lcd.print(F(" 17m  15m   12m"));
  lcd.setCursor(1, 3);
  lcd.print(F(" 10m   6m   Free"));
  delay(100);    //pour le bp
  while( (b=encoder->getButton())== ClickEncoder::Open){
  byte x,y;
  rotary_encoder_value=encoder->getValue();
  if(rotary_encoder_value!=0)
  {
    bandSwappPrec=bandSwapp;
    bandSwapp=(bandSwapp+rotary_encoder_value)%12;
    delay(100);
  }

  //remowe old mark
  x=(bandSwappPrec%3)*6; //3 items in rows every 6 characters
  y=(bandSwappPrec/3); //11 bands in total grouped 3 by row
  lcd.setCursor(x, y);
  lcd.write(32);
  //mark new selected band
  x=(bandSwapp%3)*6;
  y=(bandSwapp/3);
  lcd.setCursor(x, y);
  lcd.write(STAR_SYMBOL);
}
  //}
  delay(100);    //pour le bp
  bandChoose=bandSwapp;
  switch (bandChoose)
  {
  case 0 : freq =  1800000; break;
  case 1 : freq =  3500000; break;
  case 2 : freq =  5300000; break;
  case 3 : freq =  7000000; break;
  case 4 : freq = 10100000; break;
  case 5 : freq = 14000000; break;
  case 6 : freq = 18100000; break;
  case 7 : freq = 21000000; break;
  case 8 : freq = 24900000; break;
  case 9 : freq = 28000000; break;
  case 10 : freq =50000000; break;
  default : freq =14000000;
  }
  if (bandChoose<11) freqStep=10000; else freqStep=100000;
  lcd_menu_analyse_refection();
  freq_prec=0;
  vna_print_unites();
}


void vna_print_unites()
{
  Serial.println(F("Freq\tAdcmag\tAdcphs\tRL\tPhiMag\tRS\tXs\tZ\tSWR"));
}
/********************************************************
 * Calibration
 * one point calibration only
 ********************************************************/
void calibration()
{
  lcd.clear();
  lcd.print(F("Leave open DUT"));
  lcd.setCursor(0, 1);
  lcd.print(F("and press button"));
  while(digitalRead(ENCODER_BUTTON)==0) {}  //BP rise down detect
  while(digitalRead(ENCODER_BUTTON)==1) {}
  DDS.vna(dds_reg,REFCLOCK6);
  delay(10);
  lcd.clear();
  measure();
  DDS.vna(dds_reg,PowerDown);
  EEPROM.write(1,(byte) adcmag);
  EEPROM.write(2,(byte) (adcmag>>8));
  EEPROM.write(3,(byte) adcphs);
  EEPROM.write(4,(byte) (adcphs>>8));
  delay(2000);
  lcd.clear();
  lcd.print(F("Done"));
  lcd.setCursor(0, 1);
  lcd.print(F("Press button"));
  while(digitalRead(ENCODER_BUTTON)==0) {}  //BP rise down detect
  while(digitalRead(ENCODER_BUTTON)==1) {}
  delay(2000);
}
