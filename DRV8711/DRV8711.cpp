#include <Arduino.h>
#include <SPI.h>
#include "DRV8711.h"

// DRV8711 Registers
struct CTRL_Register CTRL_Reg;
struct TORQUE_Register TORQUE_Reg;
struct OFF_Register OFF_Reg;
struct BLANK_Register BLANK_Reg;
struct DECAY_Register DECAY_Reg;
struct STALL_Register STALL_Reg;
struct DRIVE_Register DRIVE_Reg;
struct STATUS_Register STATUS_Reg;

DRV8711::DRV8711(int CS){
	SPI.begin();
	CS_pin = CS;
	pinMode(CS_pin, OUTPUT);
	digitalWrite(CS_pin, LOW);
}

void DRV8711::combineInstructions(void *structPointer, int structSize, uint16_t* msg){
	for(int i = 0; i < structSize; i++){
		*msg = *msg | *(((uint16_t*)(structPointer))+i);
	}
	*msg = *msg & ~(1<<15); //Set read/write bit to 0 (=write).
}

void DRV8711::composeMessage(uint16_t control){
	int structSize;
	uint16_t msg = 0;
	if(control == CTRL){
		DRV8711::CTRLr(&CTRL_Reg);
		structSize = (int)(sizeof(struct CTRL_Register)/2);
		DRV8711::combineInstructions(&CTRL_Reg, structSize, &msg);
	}else if(control == TORQUE){
		DRV8711::TORQUEr(&TORQUE_Reg);
		structSize = (int)(sizeof(struct TORQUE_Register)/2);
		DRV8711::combineInstructions(&TORQUE_Reg, structSize, &msg);
	}else if(control == OFF){
		DRV8711::OFFr(&OFF_Reg);
		structSize = (int)(sizeof(struct OFF_Register)/2);
		DRV8711::combineInstructions(&OFF_Reg, structSize, &msg);
	}else if(control == BLANK){
		DRV8711::BLANKr(&BLANK_Reg);
		structSize = (int)(sizeof(struct BLANK_Register)/2);
		DRV8711::combineInstructions(&BLANK_Reg, structSize, &msg);
	}else if(control == DECAY){
		DRV8711::DECAYr(&DECAY_Reg);
		structSize = (int)(sizeof(struct DECAY_Register)/2);
		DRV8711::combineInstructions(&DECAY_Reg, structSize, &msg);
	}else if(control == STALL){
		DRV8711::STALLr(&STALL_Reg);
		structSize = (int)(sizeof(struct STALL_Register)/2);
		DRV8711::combineInstructions(&STALL_Reg, structSize, &msg);
	}else if(control == DRIVE){
		DRV8711::DRIVEr(&DRIVE_Reg);
		structSize = (int)(sizeof(struct DRIVE_Register)/2);
		DRV8711::combineInstructions(&DRIVE_Reg, structSize, &msg);
	}else if(control == STATUS){
		DRV8711::STATUSr(&STATUS_Reg);
		structSize = (int)(sizeof(struct STATUS_Register)/2);
		DRV8711::combineInstructions(&STATUS_Reg, structSize, &msg);
	}
	spiMessage = msg;
	//DRV8711::spiSendWord(msg);
}

void DRV8711::STATUSr(struct STATUS_Register *reg){
	if(STATUS_Reg.Address != ((STATUS<<12) & 0x7000)){
		STATUS_Reg.Address = (STATUS<<12) & 0x7000;
	}
}

void DRV8711::DRIVEr(struct DRIVE_Register *reg){
	if(DRIVE_Reg.Address != ((DRIVE<<12) & 0x7000)){
		DRIVE_Reg.Address = (DRIVE<<12) & 0x7000;
	}
	if(DRIVE_Reg.IDRIVEP != reg->IDRIVEP || (DRIVE_Reg.IDRIVEP & 0x0C00) != ((reg->IDRIVEP<<10) & 0x0C00)){
		DRIVE_Reg.IDRIVEP = (reg->IDRIVEP<<10) & 0x0C00;
	}
	if(DRIVE_Reg.IDRIVEN != reg->IDRIVEN || (DRIVE_Reg.IDRIVEN & 0x0300) != ((reg->IDRIVEN<<8) & 0x0300)){
		DRIVE_Reg.IDRIVEN = (reg->IDRIVEN<<8) & 0x0300;
	}
	if(DRIVE_Reg.TDRIVEP != reg->TDRIVEP || (DRIVE_Reg.TDRIVEP & 0x00C0) != ((reg->TDRIVEP<<6) & 0x00C0)){
		DRIVE_Reg.TDRIVEP = (reg->TDRIVEP<<6) & 0x00C0;
	}
	if(DRIVE_Reg.TDRIVEN != reg->TDRIVEN || (DRIVE_Reg.TDRIVEN & 0x0030) != ((reg->TDRIVEN<<4) & 0x0030)){
		DRIVE_Reg.TDRIVEN = (reg->TDRIVEN<<4) & 0x0030;
	}
	if(DRIVE_Reg.OCPDEG != reg->OCPDEG || (DRIVE_Reg.OCPDEG & 0x000C) != ((reg->OCPDEG<<2) & 0x000C)){
		DRIVE_Reg.OCPDEG = (reg->OCPDEG<<2) & 0x000C;
	}
	if(DRIVE_Reg.OCPTH != reg->OCPTH){
		DRIVE_Reg.OCPTH = (reg->OCPTH) & 0x0003;
	}
}

void DRV8711::STALLr(struct STALL_Register *reg){
	if(STALL_Reg.Address != ((STALL<<12) & 0x7000)){
		STALL_Reg.Address = (STALL<<12) & 0x7000;
	}
	if(STALL_Reg.VDIV != reg->VDIV || (STALL_Reg.VDIV & 0x0C00) != ((reg->VDIV<<10) & 0x0C00)){
		STALL_Reg.VDIV = (reg->VDIV<<10) & 0x0C00;
	}
	if(STALL_Reg.SDCNT != reg->SDCNT || (STALL_Reg.SDCNT & 0x0300) != ((reg->SDCNT<<8) & 0x0300)){
		STALL_Reg.SDCNT = (reg->SDCNT<<8) & 0x0300;
	}
	if(STALL_Reg.SDTHR != reg->SDTHR){
		STALL_Reg.SDTHR = (reg->SDTHR) & 0x00FF;
	}
}

void DRV8711::DECAYr(struct DECAY_Register *reg){
	if(DECAY_Reg.Address != ((DECAY<<12) & 0x7000)){
		DECAY_Reg.Address = (DECAY<<12) & 0x7000;
	}
	if(DECAY_Reg.DECMOD != reg->DECMOD || (DECAY_Reg.DECMOD & 0x0700) != ((reg->DECMOD<<8) & 0x0700)){
		DECAY_Reg.DECMOD = (reg->DECMOD<<8) & 0x0700;
	}
	if(DECAY_Reg.TDECAY != reg->TDECAY){
		DECAY_Reg.TDECAY = (reg->TDECAY) & 0x00FF;
	}
}

void DRV8711::BLANKr(struct BLANK_Register *reg){
	if(BLANK_Reg.Address != ((BLANK<<12) & 0x7000)){
		BLANK_Reg.Address = (BLANK<<12) & 0x7000;
	}
	if(BLANK_Reg.ABT != reg->ABT || (BLANK_Reg.ABT & 0x0100) != ((reg->ABT<<8) & 0x0100)){
		BLANK_Reg.ABT = (reg->ABT<<8) & 0x0100;
	}
	if(BLANK_Reg.TBLANK != reg->TBLANK){
		BLANK_Reg.TBLANK = (reg->TBLANK) & 0x00FF;
	}
}

void DRV8711::OFFr(struct OFF_Register *reg){
	if(OFF_Reg.Address != ((OFF<<12) & 0x7000)){
		OFF_Reg.Address = (OFF<<12) & 0x7000;
	}
	if(OFF_Reg.PWMMODE != reg->PWMMODE || (OFF_Reg.PWMMODE & 0x0100) != ((reg->PWMMODE<<8) & 0x0100)){
		OFF_Reg.PWMMODE = (reg->PWMMODE<<8) & 0x0100;
	}
	if(OFF_Reg.TOFF != reg->TOFF){
		OFF_Reg.TOFF = (reg->TOFF) & 0x00FF;
	}
}

void DRV8711::TORQUEr(struct TORQUE_Register *reg){
	if(TORQUE_Reg.Address != ((TORQUE<<12) & 0x7000)){
		TORQUE_Reg.Address = (TORQUE<<12) & 0x7000;
	}
	if(TORQUE_Reg.SMPLTH != reg->SMPLTH || (TORQUE_Reg.SMPLTH & 0x0700) != ((reg->SMPLTH<<8) & 0x0700)){
		TORQUE_Reg.SMPLTH = (reg->SMPLTH<<8) & 0x0700;
	}
	if(TORQUE_Reg.TORQUE != reg->TORQUE){
		TORQUE_Reg.TORQUE = (reg->TORQUE) & 0x00FF;
	}
}

void DRV8711::CTRLr(struct CTRL_Register *reg){
	if(CTRL_Reg.Address != ((CTRL<<12) & 0x7000)){
		CTRL_Reg.Address = (CTRL<<12) & 0x7000;
	}
	if(CTRL_Reg.DTIME != reg->DTIME || (CTRL_Reg.DTIME & 0x0C00) != ((reg->DTIME<<10) & 0x0C00)){
		CTRL_Reg.DTIME = (reg->DTIME<<10) & 0x0C00;
	}
	if(CTRL_Reg.ISGAIN != reg->ISGAIN || (CTRL_Reg.ISGAIN & 0x0300) != ((reg->ISGAIN<<8) & 0x0300)){
		CTRL_Reg.ISGAIN = (reg->ISGAIN<<8) & 0x0300;
	}
	if(CTRL_Reg.EXSTALL != reg->EXSTALL || (CTRL_Reg.EXSTALL & 0x0080) != ((reg->EXSTALL<<7) & 0x0080)){
		CTRL_Reg.EXSTALL = (reg->EXSTALL<<7) & 0x0080;
	}
	if(CTRL_Reg.MODE != reg->MODE || (CTRL_Reg.MODE & 0x0078) != ((reg->MODE<<3) & 0x0078)){
		CTRL_Reg.MODE = (reg->MODE<<3) & 0x0078;
	}
	if(CTRL_Reg.RSTEP != reg->RSTEP || (CTRL_Reg.RSTEP & 0x0004) != ((reg->RSTEP<<2) & 0x0004)){
		CTRL_Reg.RSTEP = (reg->RSTEP<<2) & 0x0004;
	}
	if(CTRL_Reg.RDIR != reg->RDIR || (CTRL_Reg.RDIR & 0x0002) != ((reg->RDIR<<1) & 0x0002)){
		CTRL_Reg.RDIR = (reg->RDIR<<1) & 0x0002;
	}
	if(CTRL_Reg.ENBL != reg->ENBL/* || (CTRL_Reg.ENBL & 0x0001) != ((reg->ENBL<<0) & 0x0001)*/){
		CTRL_Reg.ENBL = (reg->ENBL) & 0x0001;
	}
}

void DRV8711::writeAllRegisters(){
	for(int i = 0; i <= STATUS; i++){
		DRV8711::composeMessage(i);
		DRV8711::printMsg();
		DRV8711::spiSendWord(spiMessage);
	}
}

void DRV8711::printMsg(){
	Serial.print("spiMsg: ");
	Serial.println(spiMessage, BIN);
}

void DRV8711::spiSendWord(uint16_t msg){
	SPI.setDataMode(SPI_MODE3);
	SPI.setBitOrder(MSBFIRST);
	digitalWrite(CS_pin, HIGH);
	delayMicroseconds(10);
	uint8_t MSB = (uint8_t)(msg>>8);
	uint8_t LSB = (uint8_t)(msg & 0x00FF);
	SPI.transfer(MSB);
	SPI.transfer(LSB);
	delayMicroseconds(10);
	digitalWrite(CS_pin, LOW);
}