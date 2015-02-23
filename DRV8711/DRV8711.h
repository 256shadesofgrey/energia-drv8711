/*
DRV8711.h - A library for Energia that handles SPI communication
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

#ifndef REGISTERS_H_
#define REGISTERS_H_

#include <Arduino.h>

//<<12
enum controlRegisters{
	CTRL,
	TORQUE,
	OFF,
	BLANK,
	DECAY,
	STALL,
	DRIVE,
	STATUS
};

// CTRL Register
struct CTRL_Register
{
	uint16_t Address;	// bits 14-12
	uint16_t DTIME;		// bits 11-10
	uint16_t ISGAIN;	// bits 9-8
	uint16_t EXSTALL;	// bit 7
	uint16_t MODE;		// bits 6-3
	uint16_t RSTEP;		// bit 2
	uint16_t RDIR;		// bit 1
	uint16_t ENBL;		// bit 0
};

// TORQUE Register
struct TORQUE_Register
{
	uint16_t Address;	// bits 14-12
	/* Reserved */ 			// bit 11
	uint16_t SMPLTH;  	// bits 10-8
	uint16_t TORQUE;	// bits 7-0
};

// OFF Register
struct OFF_Register
{
	uint16_t Address;	// bits 14-12
	/* Reserved */ 			// bits 11-9
	uint16_t PWMMODE;  	// bit 8
	uint16_t TOFF;		// bits 7-0
};

// BLANK Register
struct BLANK_Register
{
	uint16_t Address;	// bits 14-12
	/* Reserved */ 			// bits 11-9
	uint16_t ABT;  		// bit 8
	uint16_t TBLANK;	// bits 7-0
};

// DECAY Register
struct DECAY_Register
{
	uint16_t Address;	// bits 14-12
	/* Reserved */ 			// bit 11
	uint16_t DECMOD;  	// bits 10-8
	uint16_t TDECAY;	// bits 7-0
};

// STALL Register
struct STALL_Register
{
	uint16_t Address;	// bits 14-12
	uint16_t VDIV;  	// bits 11-10
	uint16_t SDCNT;		// bits 9-8
	uint16_t SDTHR;		// bits 7-0
};

// DRIVE Register
struct DRIVE_Register
{
	uint16_t Address;	// bits 14-12
	uint16_t IDRIVEP;  	// bits 11-10
	uint16_t IDRIVEN;	// bits 9-8
	uint16_t TDRIVEP;	// bits 7-6
	uint16_t TDRIVEN;	// bits 5-4
	uint16_t OCPDEG;	// bits 3-2
	uint16_t OCPTH;		// bits 1-0
};

// STATUS Register
struct STATUS_Register
{
	uint16_t Address;	// bits 14-12
	/* Reserved */			// bits 11-8
	uint16_t STDLAT;  	// bit 7
	uint16_t STD;		// bit 6
	uint16_t UVLO;		// bit 5
	uint16_t BPDF;		// bit 4
	uint16_t APDF;		// bit 3
	uint16_t BOCP;		// bit 2
	uint16_t AOCP;		// bit 1
	uint16_t OTS;		// bit 0
};
/*
extern CTRL_Register CTRL_Reg;
extern TORQUE_Register TORQUE_Reg;
extern OFF_Register OFF_Reg;
extern BLANK_Register BLANK_Reg;
extern DECAY_Register DECAY_Reg;
extern STALL_Register STALL_Reg;
extern DRIVE_Register DRIVE_Reg;
extern STATUS_Register STATUS_Reg;*/

extern CTRL_Register CTRLn;
extern TORQUE_Register TORQUEn;
extern OFF_Register OFFn;
extern BLANK_Register BLANKn;
extern DECAY_Register DECAYn;
extern STALL_Register STALLn;
extern DRIVE_Register DRIVEn;
extern STATUS_Register STATUSn;

class DRV8711{
	public:
		/*struct CTRL_Register CTRL_Reg;
		struct TORQUE_Register TORQUE_Reg;
		struct OFF_Register OFF_Reg;
		struct BLANK_Register BLANK_Reg;
		struct DECAY_Register DECAY_Reg;
		struct STALL_Register STALL_Reg;
		struct DRIVE_Register DRIVE_Reg;
		struct STATUS_Register STATUS_Reg;*/
	
		DRV8711(int CS, float Rs = 0.05);
		void writeAllRegisters();
		//void printMsg();
		
		void enableMotor(uint16_t state);
		void setDirection(uint16_t dir);
		void setSteppingMode(uint16_t mode);
		float setCurrent(uint16_t torque, uint16_t isgain);
	protected:
		int CS_pin;
		float Rsens;
		uint16_t spiMessage;
		
		
		void setDefaultRegisters();
		void composeMessage(uint16_t control = 0);
		void combineInstructions(void *structPointer, int structSize, uint16_t* msg);
		uint16_t spiSendWord(uint16_t msg);
		void CTRLr(struct CTRL_Register *reg);
		void TORQUEr(struct TORQUE_Register *reg);
		void OFFr(struct OFF_Register *reg);
		void BLANKr(struct BLANK_Register *reg);
		void DECAYr(struct DECAY_Register *reg);
		void STALLr(struct STALL_Register *reg);
		void DRIVEr(struct DRIVE_Register *reg);
		void STATUSr(struct STATUS_Register *reg);
};


#endif /* REGISTERS_H_ */