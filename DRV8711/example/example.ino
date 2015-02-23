#include <SPI.h>
#include <DRV8711.h>

#define DRIVER_SELECT_PIN 11
#define STEP_PIN 9
#define DIR_PIN 10
#define SLP_PIN 6
#define RST_PIN 8

DRV8711 DRV8711(DRIVER_SELECT_PIN);

void setup() {
  Serial.begin(9600);
  
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, HIGH);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  pinMode(SLP_PIN, OUTPUT);
  digitalWrite(SLP_PIN, HIGH);
  //pinMode(RST_PIN, OUTPUT);
  //digitalWrite(RST_PIN, HIGH);
  
  /*The registers can be written to directly.
  Make sure to call DRV8711.writeAllRegisters()
  afterwards, if you do.*/
  //---example values---
  /*CTRLn.DTIME = 0x03;
  CTRLn.ISGAIN = 0x03;
  CTRLn.EXSTALL = 0x00;
  CTRLn.MODE = 0x00;
  CTRLn.RSTEP = 0x01;
  CTRLn.RDIR = 0x00;
  CTRLn.ENBL = 0x01;
  
  TORQUEn.TORQUE = 0x1C;
  TORQUEn.SMPLTH = 0x01;
  
  OFFn.TOFF = 0x30;
  OFFn.PWMMODE = 0x00;
  
  BLANKn.TBLANK = 0x80;
  BLANKn.ABT = 0x00;
  
  DECAYn.TDECAY = 0x10;
  DECAYn.DECMOD = 0x01;
  
  STALLn.SDTHR = 0x40;
  STALLn.SDCNT = 0x00;
  STALLn.VDIV = 0x00;
  
  DRIVEn.OCPTH = 0x00;
  DRIVEn.OCPDEG = 0x01;
  DRIVEn.TDRIVEN = 0x01;
  DRIVEn.TDRIVEP = 0x01;
  DRIVEn.IDRIVEN = 0x00;
  DRIVEn.IDRIVEP = 0x00;
  
  STATUSn.OTS = 0x00;
  STATUSn.AOCP = 0x00;
  STATUSn.BOCP = 0x00;
  STATUSn.APDF = 0x00;
  STATUSn.BPDF = 0x00;
  STATUSn.UVLO = 0x00;
  STATUSn.STD = 0x00;
  STATUSn.STDLAT = 0x00;*/
  //-----------------
  
  //DRV8711.writeAllRegisters();
}

void loop() {
  float current;
  DRV8711.enableMotor(0);
  current = DRV8711.setCurrent(28, 3);
  DRV8711.setSteppingMode(4);
  DRV8711.setDirection(0);
  DRV8711.enableMotor(1);
  Serial.println(current);
  testrun();
  delay(1000);
  digitalWrite(DIR_PIN, LOW);
  testrun();
  delay(1000);
  digitalWrite(DIR_PIN, HIGH);
  
  DRV8711.enableMotor(0);
  current = DRV8711.setCurrent(10, 3);
  DRV8711.setSteppingMode(1);
  DRV8711.setDirection(1);
  DRV8711.enableMotor(1);
  Serial.println(current);
  testrun();
  delay(1000);
  digitalWrite(DIR_PIN, LOW);
  testrun();
  delay(1000);
  digitalWrite(DIR_PIN, HIGH);
}

void testrun(){
  for(int i = 0; i < 2000; i++){
    delayMicroseconds(2000);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
  }
}
