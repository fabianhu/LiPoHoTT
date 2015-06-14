/*
	LiPoHott.h

	Created: 09.12.2012 19:29:18
	Author: Fabian Huslik
	
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


#ifndef LIPOHOTT_H_
#define LIPOHOTT_H_

typedef struct data_tag
{
	int16_t UBatt; // total voltage in mV
	int16_t UCell[6]; // cell voltage in mV
	int16_t UminCellV; // cell voltage in mV
	uint8_t fill; // percentage linear 4.2V = full "empty" = 0V
	uint8_t almL; // cell alarms bitfield
	uint8_t almH; // cell alarms bitfield
	uint8_t UminCellNum;
	int32_t CurrentmA; 
	uint32_t CapamAs;
}data_t;


typedef struct par_tag
{
	int16_t Cal[6]; // cell voltagediff in mV
	int16_t limitL; // limit voltage in mV
	int16_t limitH; // limit voltage in mV
	uint16_t zeroBits; // zero offset of current measurement
	uint8_t cells;
}par_t;


void SafePars(void);
void Calibrate(void);
void CalibrateZeroCurrent(void);
uint32_t GetTime(void); // thread & isr safe time getter

#define MEASCELLCOUNT 3

#define SILENCE 0

#define CALEXPECTEDmV 4200
#define MINCELLVOLTCALC 2500

#if MEASCELLCOUNT == 6
#define UINMAX 25286 // calculated with 1M load for 6S variant with 33k 8k2 divider.
#else
#if MEASCELLCOUNT == 3
//#define UINMAX 13080 // calculated with 1M load for 3S variant with 8k2 5k1 divider.
#define UINMAX 13080 // calculated with 1M load for 3S variant with 8k2 5k1 divider.
#else
#error "there is no spoon"
#endif
#endif




#endif /* LIPOHOTT.H_H_ */
