/*
DRV8711.cpp - A library for Energia that handles SPI communication
with the stepper motor controller DRV8711.
*/

/*
  Copyright 2015 Dmitri Ranfft

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <SPI.h>
#include "DRV8711.h"

// DRV8711 Registers that will be used internally.
struct CTRL_Register CTRL_Reg;
struct TORQUE_Register TORQUE_Reg;
struct OFF_Register OFF_Reg;
struct BLANK_Register BLANK_Reg;
struct DECAY_Register DECAY_Reg;
struct STALL_Register STALL_Reg;
struct DRIVE_Register DRIVE_Reg;
struct STATUS_Register STATUS_Reg;

// Write to these externally.
struct CTRL_Register CTRLn;
struct TORQUE_Register TORQUEn;
struct OFF_Register OFFn;
struct BLANK_Register BLANKn;
struct DECAY_Register DECAYn;
struct STALL_Register STALLn;
struct DRIVE_Register DRIVEn;
struct STATUS_Register STATUSn;

DRV8711::DRV8711(int CS, float Rs){
	SPI.begin();
	CS_pin = CS;
	Rsens = Rs;
	pinMode(CS_pin, OUTPUT);
	digitalWrite(CS_pin, LOW);
	DRV8711::setDefaultRegisters();
}

void DRV8711::setDefaultRegisters(){
	//---default values---
	CTRLn.DTIME = 0x03;
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
	STATUSn.STDLAT = 0x00;
	//-----------------
	
	DRV8711::writeAllRegisters();
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
		DRV8711::CTRLr(&CTRLn);
		structSize = (int)(sizeof(struct CTRL_Register)/2);
		DRV8711::combineInstructions(&CTRL_Reg, structSize, &msg);
	}else if(control == TORQUE){
		DRV8711::TORQUEr(&TORQUEn);
		structSize = (int)(sizeof(struct TORQUE_Register)/2);
		DRV8711::combineInstructions(&TORQUE_Reg, structSize, &msg);
	}else if(control == OFF){
		DRV8711::OFFr(&OFFn);
		structSize = (int)(sizeof(struct OFF_Register)/2);
		DRV8711::combineInstructions(&OFF_Reg, structSize, &msg);
	}else if(control == BLANK){
		DRV8711::BLANKr(&BLANKn);
		structSize = (int)(sizeof(struct BLANK_Register)/2);
		DRV8711::combineInstructions(&BLANK_Reg, structSize, &msg);
	}else if(control == DECAY){
		DRV8711::DECAYr(&DECAYn);
		structSize = (int)(sizeof(struct DECAY_Register)/2);
		DRV8711::combineInstructions(&DECAY_Reg, structSize, &msg);
	}else if(control == STALL){
		DRV8711::STALLr(&STALLn);
		structSize = (int)(sizeof(struct STALL_Register)/2);
		DRV8711::combineInstructions(&STALL_Reg, structSize, &msg);
	}else if(control == DRIVE){
		DRV8711::DRIVEr(&DRIVEn);
		structSize = (int)(sizeof(struct DRIVE_Register)/2);
		DRV8711::combineInstructions(&DRIVE_Reg, structSize, &msg);
	}else if(control == STATUS){
		DRV8711::STATUSr(&STATUSn);
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
	if(DRIVE_Reg.IDRIVEP != reg->IDRIVEP/* || (DRIVE_Reg.IDRIVEP & 0x0C00) != ((reg->IDRIVEP<<10) & 0x0C00)*/){
		DRIVE_Reg.IDRIVEP = (reg->IDRIVEP<<10) & 0x0C00;
	}
	if(DRIVE_Reg.IDRIVEN != reg->IDRIVEN/* || (DRIVE_Reg.IDRIVEN & 0x0300) != ((reg->IDRIVEN<<8) & 0x0300)*/){
		DRIVE_Reg.IDRIVEN = (reg->IDRIVEN<<8) & 0x0300;
	}
	if(DRIVE_Reg.TDRIVEP != reg->TDRIVEP/* || (DRIVE_Reg.TDRIVEP & 0x00C0) != ((reg->TDRIVEP<<6) & 0x00C0)*/){
		DRIVE_Reg.TDRIVEP = (reg->TDRIVEP<<6) & 0x00C0;
	}
	if(DRIVE_Reg.TDRIVEN != reg->TDRIVEN/* || (DRIVE_Reg.TDRIVEN & 0x0030) != ((reg->TDRIVEN<<4) & 0x0030)*/){
		DRIVE_Reg.TDRIVEN = (reg->TDRIVEN<<4) & 0x0030;
	}
	if(DRIVE_Reg.OCPDEG != reg->OCPDEG/* || (DRIVE_Reg.OCPDEG & 0x000C) != ((reg->OCPDEG<<2) & 0x000C)*/){
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
	if(STALL_Reg.VDIV != reg->VDIV/* || (STALL_Reg.VDIV & 0x0C00) != ((reg->VDIV<<10) & 0x0C00)*/){
		STALL_Reg.VDIV = (reg->VDIV<<10) & 0x0C00;
	}
	if(STALL_Reg.SDCNT != reg->SDCNT/* || (STALL_Reg.SDCNT & 0x0300) != ((reg->SDCNT<<8) & 0x0300)*/){
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
	if(DECAY_Reg.DECMOD != reg->DECMOD/* || (DECAY_Reg.DECMOD & 0x0700) != ((reg->DECMOD<<8) & 0x0700)*/){
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
	if(BLANK_Reg.ABT != reg->ABT/* || (BLANK_Reg.ABT & 0x0100) != ((reg->ABT<<8) & 0x0100)*/){
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
	if(OFF_Reg.PWMMODE != reg->PWMMODE/* || (OFF_Reg.PWMMODE & 0x0100) != ((reg->PWMMODE<<8) & 0x0100)*/){
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
	if(TORQUE_Reg.SMPLTH != reg->SMPLTH/* || (TORQUE_Reg.SMPLTH & 0x0700) != ((reg->SMPLTH<<8) & 0x0700)*/){
		TORQUE_Reg.SMPLTH = (reg->SMPLTH<<8) & 0x0700;
	}
	if(TORQUE_Reg.TORQUE != reg->TORQUE){
		TORQUE_Reg.TORQUE = (reg->TORQUE) & 0x00FF;
	}
}

void DRV8711::CTRLr(struct CTRL_Register *reg){
	//Serial.println(CTRL_Reg.MODE);
	//Serial.println(reg->MODE);
	if(CTRL_Reg.Address != ((CTRL<<12) & 0x7000)){
		CTRL_Reg.Address = (CTRL<<12) & 0x7000;
	}
	if(CTRL_Reg.DTIME != reg->DTIME/* || (CTRL_Reg.DTIME & 0x0C00) != ((reg->DTIME<<10) & 0x0C00)*/){
		CTRL_Reg.DTIME = (reg->DTIME<<10) & 0x0C00;
	}
	if(CTRL_Reg.ISGAIN != reg->ISGAIN/* || (CTRL_Reg.ISGAIN & 0x0300) != ((reg->ISGAIN<<8) & 0x0300)*/){
		CTRL_Reg.ISGAIN = (reg->ISGAIN<<8) & 0x0300;
	}
	if(CTRL_Reg.EXSTALL != reg->EXSTALL/* || (CTRL_Reg.EXSTALL & 0x0080) != ((reg->EXSTALL<<7) & 0x0080)*/){
		CTRL_Reg.EXSTALL = (reg->EXSTALL<<7) & 0x0080;
	}
	if(CTRL_Reg.MODE != reg->MODE/* || (CTRL_Reg.MODE & 0x0078) != ((reg->MODE<<3) & 0x0078)*/){
		CTRL_Reg.MODE = (reg->MODE<<3) & 0x0078;
	}
	if(CTRL_Reg.RSTEP != reg->RSTEP/* || (CTRL_Reg.RSTEP & 0x0004) != ((reg->RSTEP<<2) & 0x0004)*/){
		CTRL_Reg.RSTEP = (reg->RSTEP<<2) & 0x0004;
	}
	if(CTRL_Reg.RDIR != reg->RDIR/* || (CTRL_Reg.RDIR & 0x0002) != ((reg->RDIR<<1) & 0x0002)*/){
		CTRL_Reg.RDIR = (reg->RDIR<<1) & 0x0002;
	}
	if(CTRL_Reg.ENBL != reg->ENBL/* || (CTRL_Reg.ENBL & 0x0001) != ((reg->ENBL<<0) & 0x0001)*/){
		CTRL_Reg.ENBL = (reg->ENBL) & 0x0001;
	}
}

void DRV8711::writeAllRegisters(){
	//uint16_t SPI_Response = 0;
	for(int i = 0; i <= STATUS; i++){
		DRV8711::composeMessage(i);
		//DRV8711::printMsg();
		/*SPI_Response = */DRV8711::spiSendWord(spiMessage);
		//Serial.println(SPI_Response);
	}
}

/*
void DRV8711::printMsg(){
	Serial.print("spiMsg: ");
	Serial.println(spiMessage, BIN);
}*/

uint16_t DRV8711::spiSendWord(uint16_t msg){
	SPI.setDataMode(SPI_MODE3);
	SPI.setBitOrder(MSBFIRST);
	digitalWrite(CS_pin, HIGH);
	delayMicroseconds(10);
	//uint8_t LSB = (uint8_t)(msg & 0x00FF);
	uint8_t LSB = (uint8_t)(msg);
	uint8_t MSB = (uint8_t)(msg>>8);
	uint16_t MSB_Ret = SPI.transfer(MSB);
	uint16_t LSB_Ret = SPI.transfer(LSB);
	delayMicroseconds(10);
	digitalWrite(CS_pin, LOW);
	
	MSB_Ret = MSB_Ret << 8;
	
	return MSB_Ret | LSB_Ret;
}

void DRV8711::enableMotor(uint16_t state){
	if(state > 0){
		CTRLn.ENBL = 0x01;
	}else{
		CTRLn.ENBL = 0x00;
	}
	
	DRV8711::composeMessage(CTRL);
	DRV8711::spiSendWord(spiMessage);
}

void DRV8711::setDirection(uint16_t dir){
	if(dir > 0){
		CTRLn.RDIR = 0x01;
	}else{
		CTRLn.RDIR = 0x00;
	}
	
	DRV8711::composeMessage(CTRL);
	DRV8711::spiSendWord(spiMessage);
}

//1 for full step, 2 for 1/2 step etc. Max: 256.
void DRV8711::setSteppingMode(uint16_t mode){
	if(mode == 1){
		CTRLn.MODE = 0x0;
	}else if(mode == 2){
		CTRLn.MODE = 0x1;
	}else if(mode == 4){
		CTRLn.MODE = 0x2;
	}else if(mode == 8){
		CTRLn.MODE = 0x3;
	}else if(mode == 16){
		CTRLn.MODE = 0x4;
	}else if(mode == 32){
		CTRLn.MODE = 0x5;
	}else if(mode == 64){
		CTRLn.MODE = 0x6;
	}else if(mode == 128){
		CTRLn.MODE = 0x7;
	}else if(mode == 256){
		CTRLn.MODE = 0x8;
	}
	
	DRV8711::composeMessage(CTRL);
	DRV8711::spiSendWord(spiMessage);
}

//torque: 0-255. isgain: 0-3. Returned is the current you get with these settings.
float DRV8711::setCurrent(uint16_t torque, uint16_t isgain){
	CTRLn.ISGAIN = isgain & 0x3;
	TORQUEn.TORQUE = torque & 0xFF;
	
	float gain;
	if(CTRLn.ISGAIN == 0x0){
		gain = 5;
	}else if(CTRLn.ISGAIN == 0x1){
		gain = 10;
	}else if(CTRLn.ISGAIN == 0x2){
		gain = 20;
	}else if(CTRLn.ISGAIN == 0x3){
		gain = 40;
	}
	float current = (2.75 * (float)TORQUEn.TORQUE)/(256 * gain * DRV8711::Rsens);
	
	DRV8711::composeMessage(CTRL);
	DRV8711::spiSendWord(spiMessage);
	DRV8711::composeMessage(TORQUE);
	DRV8711::spiSendWord(spiMessage);
	
	return current;
}

