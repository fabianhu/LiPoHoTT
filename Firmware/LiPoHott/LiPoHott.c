/*
	LiPoHott.c
	
	Created: 06.09.2012 22:56:32
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

 
 
 TODO:
 - Kalibrierung checken
 - Strom-Messung
 - mAh-Zähler
 
 Fuses:
 BOOTSZ = 1024W_0C00
 BOOTRST = [ ]
 RSTDISBL = [ ]
 DWEN = [ ]
 SPIEN = [X]
 WDTON = [ ]
 EESAVE = [ ]
 BODLEVEL = 4V3
 CKDIV8 = [X]
 CKOUT = [ ]
 SUT_CKSEL = INTRCOSC_8MHZ_6CK_14CK_65MS

 EXTENDED = 0xF9 (valid)
 HIGH = 0xDC (valid)
 LOW = 0x62 (valid)

 
 
  */ 

#ifdef F_CPU
#if F_CPU != 8000000
	#error "fcpu"
#endif
#else
	#define F_CPU 8000000
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "hottV4.h"
#include "LiPoHott.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "printf.h"
#include <math.h>


uint16_t adc_get_filter_channel(uint8_t ); // reads the adc x times and filters.
uint16_t a2dConvert10bit(uint8_t ch);
void getADC(void);
uint16_t ADC_to_mV(uint16_t val);
void init_Par(void);
void timing_init(void);

extern uint32_t lastComm;

static volatile uint32_t GlobalTimer_us; // the global timer, incremented by ISR every 100 us

data_t myData;
par_t myPar;


int main(void)
{
	//initialization
	DDRB  = 0; 
	PORTB = 0;
	DDRC  = 0;
	PORTC = 0;
	DDRD  = (1<<PD1)|(1<<PD2)|(1<<PD5);// rx/tx switch + LED
	PORTD = (1<<PD1);// tx high idle!
	
	// crank up the CPU to 8MHz
	while(CLKPR != 0) 
	{
		asm("nop");	
		CLKPR = 1<<7;// will not work without optimisation !!!!
		CLKPR = 0;

	}

	ADCSRA = 0b10000110; //ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0 // 125kHz adc-clock
	DIDR0 = 0b00111111;

	timing_init();

	init_Par();

	// init USART for 19200 baud
	uart_init(UART_BAUD_SELECT(19200,F_CPU));
	asm("sei");
    while(1)
    {
        
		getADC(); // calls Hott between conversions
		
		if (GetTime() - lastComm > 1000000)
		PORTD |= (1<<PD5); // toggle LED
    }
}

#define MEASURES 64 // max 64! // use 2^n for shift-divide.
uint16_t adc_get_filter_channel(uint8_t c) // takes about 6,4ms
{
    uint16_t a;	

	
	uint8_t z;
	
	a=0;
	for (z =0;z<MEASURES;z++)
	{
		a+=a2dConvert10bit(c); // takes 100 us
		
		uint32_t t = GetTime();
		_hott_serial_scheduler(t); // HOTT here, as timing allows it only here.
		
		
	}
	
	return a;///MEASURES; 
}

uint32_t GetTime(void)
{
	uint32_t t;
	cli();
	t= GlobalTimer_us;
	sei();
	return t;
}


//! Perform a 10-bit conversion
// starts conversion, waits until conversion is done, and returns result
uint16_t a2dConvert10bit(uint8_t ch) // ch 14 gets 1.1V bandgap reference fixme // takes about 0.1ms
{
	
	ADMUX = 0b01000000 | (ch & 0b00001111);
	
	ADCSRA |= (1<< ADSC) | (1<<ADIF);	

	while( !(ADCSRA & (1<<ADIF)) )
	{
		ch++; // dummmy
	}		// wait until conversion complete

	// CAUTION: MUST READ ADCL BEFORE ADCH!!!
	return ADC;//(ADCL) | ((ADCH)<<8);	// read ADC (full 10 bits);
}

#define limit(v,l,u) (((v)<(l))?(l):(((v)>(u))?(u):(v)))

int32_t propscale(int32_t _value, int32_t _minin, int32_t _maxin, int32_t _minout, int32_t _maxout)
{
	int32_t rangedvalue;
	int32_t inrange = _maxin-_minin;
	int32_t outrange = _maxout-_minout;
	if(inrange== 0) return 0; // divide by zero protection
	
	_value = limit(_value,_minin,_maxin); // do not extrapolate
	
	rangedvalue = (_value-_minin)*outrange/inrange;

	return rangedvalue + _minout;
}

volatile int32_t tmp;

int32_t ScaleCurrmA(int32_t _raw)
{
	/*
	5V = 1024 bits.
	2,5V = 512 bits
	1V = 204.8 bits.

	for ACS750xCA-050
	40 mV / A
	8,192 bit / A
	0,1220703125 A/bit
	122,070 mA/bit
	
	(+-50A = 0,5V / 4,5V )
	*/
	
	tmp = _raw/MEASURES;
	asm("nop");
	tmp= (int32_t)tmp - (int32_t)myPar.zeroBits; // center volt
	tmp = tmp * (int32_t)122; // 
	tmp = abs(tmp); // fixme return only positives
	
	return tmp;
}

void getADC(void)
{
	uint16_t raw[6];

	for (uint8_t i =0;i<MEASCELLCOUNT;i++)
	{
		raw[i] = ADC_to_mV(adc_get_filter_channel(5-i));
	}
	
	myData.CurrentmA = abs(ScaleCurrmA(adc_get_filter_channel(6)));
	
	if(myData.CurrentmA !=0)
	{
		asm("nop");
	}
	
	uint32_t _now = GetTime()/100;
	
	static uint32_t oldCapTime=0;
	uint32_t CapDiffTime = _now - oldCapTime;
	oldCapTime = _now;
	myData.CapamAs += ((CapDiffTime)*(myData.CurrentmA))/10000;
	
		
	myData.UCell[0] = raw[0]+propscale(raw[0],0,4200-myPar.Cal[0],0,myPar.Cal[0]);

	uint16_t batt=myData.UCell[0];
	for (uint8_t i =1;i<myPar.cells;i++) // calibration proportional to raw value?
	{
		int16_t v = raw[i]-raw[i-1];
		
		myData.UCell[i] = v+propscale(v,0,4200-myPar.Cal[i],0,myPar.Cal[i]);//   myPar.Cal[i];
		batt += myData.UCell[i];
	}		
	myData.UBatt = batt;
	
	// some calculations
	uint16_t Umin=4200;
	uint8_t nmin = 0;
	uint8_t almL = 0;
	uint8_t almH = 0;
	for (uint8_t i =0;i<myPar.cells;i++)
	{
		if(myData.UCell[i]<Umin && myData.UCell[i] > MINCELLVOLTCALC)
		{
			Umin = myData.UCell[i];
			nmin = i;
		}
		if(myData.UCell[i]<myPar.limitL && myData.UCell[i] > MINCELLVOLTCALC)
		{
			almL |= 1<<i;
		}
		if(myData.UCell[i]<myPar.limitH && myData.UCell[i] > MINCELLVOLTCALC)
		{
			almH |= 1<<i;
		}
	}
	myData.almL = almL;
	myData.almH = almH;

	myData.UminCellV = Umin;
	myData.UminCellNum = nmin+1;

	/*int16_t level = Umin - myPar.limitL;
	int16_t total = 4230 - myPar.limitL;
	int16_t res = level*100L / total;*/
	myData.fill = propscale(Umin,myPar.limitH,4200,0,100);//  (uint8_t)res;
	
}



#define MAXADC 1023*64


uint16_t ADC_to_mV(uint16_t val)
{
	uint32_t temp;
	temp = val * (uint32_t)UINMAX;
	return (uint16_t)(temp>>16);// / MAXADC; == divide by 1024*64
}

void init_Par(void)
{
	
	
	eeprom_read_block((uint8_t*)&myPar,(void*)1,sizeof(par_t));
	
	if(myPar.Cal[0] == 0xffff)
	{
		myPar.Cal[0]= 0;
		myPar.Cal[1]= 0;
		myPar.Cal[2]= 0;
		myPar.Cal[3]= 0;
		myPar.Cal[4]= 0;
		myPar.Cal[5]= 0;
	}
	if(myPar.limitL > 4200 || myPar.limitL < 2300)
		myPar.limitL = 3200;
	if(myPar.limitH > 4200 || myPar.limitH < 2300)
		myPar.limitH = 3200;	
	if(myPar.cells > MEASCELLCOUNT || myPar.cells < 1)
		myPar.cells = MEASCELLCOUNT;
	if(myPar.zeroBits >600 || myPar.zeroBits <400) 
		myPar.zeroBits = 512;
	
	
}

void SafePars(void)
{
	static uint8_t safecount=0;
	safecount++;
	if(safecount >5)	return; // prevent numerous safes.
	eeprom_update_block((uint8_t*)&myPar,(void*)1,sizeof(par_t));
}

void CalibrateZeroCurrent(void)
{
	myPar.zeroBits = adc_get_filter_channel(6)/MEASURES;
}


void Calibrate(void)
{
	// measure the raw value
	uint16_t raw[MEASCELLCOUNT];

	for (uint8_t i =0;i<MEASCELLCOUNT;i++)
	{
		raw[i] = ADC_to_mV(adc_get_filter_channel(5-i));
	}
	myData.UCell[0] = raw[0];
	for (uint8_t i =1;i<MEASCELLCOUNT-1;i++)
	{
		myData.UCell[i] = raw[i]-raw[i-1];
	}
	
	
	for (uint8_t i =0;i<MEASCELLCOUNT;i++)
	{
		myPar.Cal[i] = CALEXPECTEDmV - myData.UCell[i];
		if(abs(myPar.Cal[i]) > 300) // deny calibration if too far off.
			myPar.Cal[i] = 0;
	}
	// 
}

#define OS_ScheduleISR 			TIMER0_COMPA_vect // Interrupt Vector used for OS-tick generation (check out CustomOS_ISRCode if you want to add isr code)
#define OS_ALLOWSCHEDULING 		TIMSK0 |= (1<<OCIE0A);	// turn Timer Interrupt ON
#define OS_PREVENTSCHEDULING 	TIMSK0 &= ~(1<<OCIE0A); // turn Timer Interrupt OFF


void timing_init(void)
{
	GlobalTimer_us = 0;
	
	TCCR0A = 0;
//	TCCR0B = 0b00000011; // 125kHz Clock @ 8MHz,
	TCCR0B = 0b00000010; // 1000kHz Clock @ 8MHz,
	
	OCR0B = 100; //which overflows after 100 cycles, which gives the desired 0.1ms increment
	TIMSK0 = (1<<OCIE0B); // ISR for OS
	
}


ISR(TIMER0_COMPB_vect)
{
	TCNT0 =0;  // reset the timer on ISR to have correct timing
	GlobalTimer_us+=100;
}


void HandleHott( unsigned char addr, uint8_t* txtptr )
{
	static uint8_t line=0;
	static uint8_t edit=0;
	
	
	// key handling
	int8_t dlt=0;
	switch (addr&0x0f) // low nibble
	{
		case HOTT_TEXT_MODE_IDLE:
		// do nothing here, as this means that no button is pressed.
		break;
		case HOTT_TEXT_MODE_RIGHT:
		if(edit==1)
		dlt=1;
		else
		break;
		case HOTT_TEXT_MODE_UP:
		if(edit==0 && line>0)
		line--;
		break;
		case HOTT_TEXT_MODE_LEFT:
		if(edit==1)
		dlt=-1;
		else
		break;
		case HOTT_TEXT_MODE_DOWN:
		if(edit==0 && line<5)
		line++;
		break;
		case HOTT_TEXT_MODE_SET:
		if(line==0 || line == 1 || line == 2)
		{
			edit = (edit==0)?1:0;
		}
		if(line==3)
		{
			// calibrate
			Calibrate();
			line =0;
		}
		if(line==4)
		{
			// calibrate Zero Current
			CalibrateZeroCurrent();
			line =0;
		}
		if(line==5)
		{
			// safe to eeprom
			SafePars();
			line =0;
		}
		break;
		default:
		break;
	}
	if(dlt != 0)
	{
		int16_t t;
		switch(line)
		{
			case 0: // limit
			t = myPar.limitL+dlt*50;
			if (t>2500 && t<4200)
			{
				myPar.limitL = t;
			}
			
			break;
			case 1: // limit
			t = myPar.limitH+dlt*50;
			if (t>2500 && t<4200)
			{
				myPar.limitH = t;
			}
			
			break;

			case 2: // cellcount
			t = myPar.cells+dlt;
			if (t>0 && t<=MEASCELLCOUNT)
			{
				myPar.cells = t;
			}
			
			break;
			case 3:	// calibrate
			
			break;
			case 4:	// calibrate Zero Current
			
			break;
			case 5:	// safe
			
			break;
			default:
			break;
			
		}
	}
	
	// text building
	sprintf(txtptr,       "  LiPoHott V1.2      ");
	hott_invert_chars(txtptr,21);
	sprintf(&txtptr[1*21],"  Limit Low: %dmV ",myPar.limitL);
	sprintf(&txtptr[2*21],"  Limit Hi: %dmV ",myPar.limitH);
	sprintf(&txtptr[3*21],"  Cells: %d     ",myPar.cells);
	sprintf(&txtptr[4*21],"  Calibrate to %dmV",CALEXPECTEDmV);
	sprintf(&txtptr[5*21],"  Set Current to Zero ");
	sprintf(&txtptr[6*21],"  Safe_to_flash ");
	//sprintf(&txtptr[6*21],"---------------------");
	//1:1234 2:1234 3:1234
	sprintf(&txtptr[7*21]," (c) huslik.net 2015 ");
	hott_invert_chars(&txtptr[7*21],21);
	//sprintf(&txtptr[7*21],"t:%d",timer_ms);
	//sprintf(&txtptr[7*21],"1:%d 2:%d 3:%d",myData.UCell[0],myData.UCell[1],myData.UCell[2]);
	//sprintf(&txtptr[7*21],"line: %d add:%d ",line,addr&0x0f); // do not use the last char!!!
	//sprintf(&txtptr[7*21],"4:%d 5:%d 6:%d",myData.UCell[3],myData.UCell[4],myData.UCell[5]); // do not use the last char!!!
	
	// set '>'
	txtptr[(line+1)*21]= (edit ==0)?'>':'*';
	
	// clean up zeroes
	for(int i = 0;i< sizeof(HOTT_TEXTMODE_MSG_t);i++)
	{
		if(txtptr[i] == 0)
		txtptr[i] = ' ';
	}
	
	PORTD ^= (1<<PD5); // toggle LED
	lastComm = GetTime();
}
