#ifndef _MAIN_H
#define _MAIN_H

/* Vna
The code is based on the projecs mentioned below:
 Anthony LE CREN F4GOH@orange.fr
 Created 03/03/2015
 Ce programme dialogue avec le logiciel Jnva et blue vna app android
 Possiblilité de fonctionnement en standalone

Hardware : dds ad9851 + ad8302
http://ra4nal.lanstek.ru/vna.shtml

Dans le Jnva, menu Calibration frequency
remplacer   10737418
par
23860477
afin de calculer un DS_FTW correct

dialogue Jna avec la carte
http://wiki.oz9aec.net/index.php/MiniVNA_ICD

protocole de transfert:
MODE $0D   DDS_FTW $0D    SAMPLES $0D     DDS_STEP $0D
Mode       StartF         NumberF         StepF

exemples :

acquisition :
 30 0D 32 33 38 36 30 34 38 0D 31 30 30 0D 34 32   0.2386048.100.42
 39 32 34 39 39 38 0D                              924998
stop
 30 0D 30 0D 31 0D 30 0D                             0.0.1.0.

générator
 30 0D 32 33 38 36 30 34 37 37 30 0D 31 0D 30 0D     0.238604770.1.0.

30 0D 32 33 38 36 30 34 37 37 0D 36 32 38 0D 33    0.23860477.628.3
37 39 38 36 0D                                       7986

manque dans le code :
detection deconnection bluevna
faire alors un reset de l'arduino
la gestion d'une carte sd

*/
//TODO: fix probelm with standard LiquidCrystal_I2C.h library (used as dependency has some compilatioin errors)

#include <avr/pgmspace.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AD9850SPI.h>
#include <SPI.h>
#include <EEPROM.h>

#define PowerDown 0x04  //  Power down AD9851,50
#define REFCLOCK6 0x01     //  AD9851 RFCLK multiplier enable x 6 en mode normal

#define SYSTEM_CLOCK 180000000.0 //120000000.0  fake AD9851 in my hardware warks only x4 (120 000000 not x6  180000000.0)
#define POWER32 4294967296.0

#define freqMin 100000
#define freqMax 60000000

//#define MinFrq 23860900 // FTW  min freq = 1 Mhz,osc:180Mhz
#define MinFrq freqMin*POWER32/SYSTEM_CLOCK   //1MHz
//#define MaxFrq 1431655765 // FTW  max freq = 60 Mhz, osc:180Mhz
#define MaxFrq freqMax*POWER32/SYSTEM_CLOCK // FTW  max freq = 60 Mhz
//#define MaxFrq 2431655765 // FTW  max freq = 60 Mhz

#define LED  6        //Affectation des broches
#define Rele  5       //relais refexion, transmission
#define ADC0  A0       //entrée mag
#define ADC1  A1      //entrée phs

#define  ENCODER_PORT_A  2
#define  ENCODER_PORT_B  3
#define  ENCODER_BUTTON 4


#define adc2Db 60/1024  // pente pleine echelle Db / resolution ADC = 0.0586
#define offsetDb -30  // décallage de -30db, pour ADC=512 -> 0db
#define Adc2Angle 180/1024  // pente pleine echelle Angle / resolution ADC = 0.175
#define D2R 3.14159/180    //degrés to radians


#define LCD_CHARS_IN_ROW 20

#define STAR_SYMBOL 42

struct vector_reflection{
  double Freq;
  float RL;
  float Phi;
  float Rho;
  float Rs;
  float Xs;
  float Swr;
  float Z;
};

struct vector_transmission{
  double Freq;
  float TL;
  float TP;
};

enum communication_mode {JVNA,BLUETOOTH};

void delete_char(byte line, byte start, byte end);
void calculDut(int adcMag, int adcPhs);
void vna_print();
void calibration();
void vna_print_unites();
void bandSelect();
void BCD (unsigned long b, char* o);
double ticksToFreq(long f);
void measure();
void doEncoder();
void boot_menu();
void menuJvna(byte PB);
char getRX(void);
void Jnva();
void sweep();
char DecodeCom (void);
void affiche_freqs(void);
void magPhsADC();
void timerIsr();

#endif
