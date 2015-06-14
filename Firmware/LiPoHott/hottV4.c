/*
 * hottV4.c
 *
 * Created: 06.09.2012 23:14:14
 * split the original arduino .pde to .c and .h
 * see header file for original license
 
 Timing stuff:
 Measured from GAM / Voltage Module:
 wait 6.5 ms before starting to answer;
 leave 2 ms between bytes on answer.
 
 */ 


#include <avr/io.h>
//#include <string.h>
#include "hottV4.h"
#include "uart.h"
#include "LiPoHott.h"
#include <avr/interrupt.h>


#define _HOTT_PORT_read uart_getc
#define _HOTT_PORT_write uart_putc
#define _HOTT_PORT_available uart_data
#define _HOTT_PORT_flush uart_flush

extern data_t myData;
extern par_t myPar;

static uint8_t _hott_telemetry_is_sending = false;
//static uint8_t _hott_telemetry_sendig_msgs_id = 0;

//HoTT serial buffer
#ifdef HOTT_SIM_TEXTMODE
static uint8_t _hott_serial_buffer_textmode[172+1]; // there has to be one more for the tailing 0.
#endif
static uint8_t _hott_serial_buffer[44];

// HoTT serial send buffer pointer
static uint8_t *_hott_msg_ptr = 0;
// Len of HoTT serial buffer
static int _hott_msg_len = 0;

uint32_t lastComm = 0;


void _hott_enable_transmitter(void) {
  //enables serial transmitter, disables receiver
	UCSR0B = (1<<TXEN0);

	PORTD |= _BV(PD2); // enable pull-up 
	// PORTD |= _BV(PD1); // high idle / no pull-down.
}

void _hott_enable_receiver(void) {
   //enables serial receiver, disables transmitter
	UCSR0B = _BV(RXCIE0)|(1<<RXEN0);
	PORTD &= ~_BV(PD2); // disable pull-up
	PORTD |= _BV(PD1); // high idle / no pull-down.
}



void _hott_send_telemetry_data(void);
void _hott_check_serial_data(uint32_t tnow);
void _hott_send_msg_text(uint8_t *buffer, int len);
void _hott_send_msg(uint8_t *buffer, int len);
void _hott_update_gam_msg(void);


//*****************************************************************************
/*
  Called periodically (?1ms) by timer scheduler
  
  @param tnow  timestamp in uSecond
*/
void _hott_serial_scheduler(uint32_t tnow) {
	static uint32_t _hott_serial_timer;
   _hott_check_serial_data(tnow);	// Check for data request
  if(_hott_msg_ptr == 0) return;   //no data to send
  if(_hott_telemetry_is_sending) {
    //we are sending already, wait for a delay of 2ms between data uint8_ts
    if(tnow - _hott_serial_timer < 3000)  //delay ca. 3,5 mS. 19200 baud = 520uS / uint8_t + 3ms required delay
    										// Graupner Specs claims here 2ms but in reality it's 3ms, they using 3ms too...
      return;
  } else {
  	//new data request
  	tnow = GetTime(); //correct the 5ms  delay in _hott_check_serial_data()...
 
  }
  _hott_send_telemetry_data();
  _hott_serial_timer = tnow;
}

/*
  Transmitts a HoTT message
*/
void _hott_send_telemetry_data() {
  static int msg_crc = 0;
  if(!_hott_telemetry_is_sending) {
  	// new message to send
    _hott_telemetry_is_sending = true;
    _hott_enable_transmitter();  //switch to transmit mode  
  }

  //send data
  if(_hott_msg_len == 0) {
    //all data send
    _hott_msg_ptr = 0;
    _hott_telemetry_is_sending = false;
    msg_crc = 0;
    _hott_enable_receiver();
    _HOTT_PORT_flush();
  } else {
    --_hott_msg_len;
    if(_hott_msg_len != 0) {
       	msg_crc += *_hott_msg_ptr; 
	    _HOTT_PORT_write(*_hott_msg_ptr++);
    } else {
    	//last uint8_t, send crc
	    _HOTT_PORT_write((uint8_t)msg_crc);
    } 
  }
}

/*
  Onetime setup for HoTT
*/
void _hott_setup() {
  //_HOTT_PORT_begin(19200);
  _hott_enable_receiver();
  //timer_scheduler.register_process(_hott_serial_scheduler);
}



/*
  Check for a valid HoTT requests on serial bus
*/

void _hott_check_serial_data(uint32_t tnow) {
	static uint32_t _hott_serial_request_timer = 0;
	if(_hott_telemetry_is_sending == true) return;
    if(_HOTT_PORT_available() > 1) {
      if(_HOTT_PORT_available() == 2) {
        if(_hott_serial_request_timer == 0) {
        	//new request, check required
        	_hott_serial_request_timer = tnow;	//save timestamp
        	return;
        } else {
        	if(tnow - _hott_serial_request_timer < 6500)	//wait ca. 5ms (Voltage module waits 6.5m)
        		return;
        	_hott_serial_request_timer = 0;	//clean timer
        }
        // we never reach this point if there is additionally data on bus, so is has to be valid request
        unsigned char c = _HOTT_PORT_read();
       static  unsigned char addr;//; = _HOTT_PORT_read();
	   addr = _HOTT_PORT_read();
        //ok, we have a valid request, check for address
		
  	    uint8_t tmp = (addr >> 4);  // Sensor type
        switch(c) {
//*****************************************************************************
#ifdef HOTT_SIM_TEXTMODE
          case HOTT_TEXT_MODE_REQUEST_ID:
           //Text mode
             {
               
               
               
             	HOTT_TEXTMODE_MSG_t	*hott_txt_msg =	(HOTT_TEXTMODE_MSG_t *)&_hott_serial_buffer_textmode[0];
				memset(hott_txt_msg, ' ', sizeof(HOTT_TEXTMODE_MSG_t));
				hott_txt_msg->start_uint8_t = 0x7b;
				hott_txt_msg->stop_uint8_t = 0x7d;
				hott_txt_msg->fill1 = 0xD0; 
				hott_txt_msg->warning_beeps = 0; // just prevent beeping
                
				//hott_txt_msg = HOTT_Clear_Text_Screen(hott_txt_msg);

				

#ifdef HOTT_SIM_GPS_SENSOR
             if(tmp == (HOTT_TELEMETRY_GPS_SENSOR_ID & 0x0f))    hott_txt_msg = HOTT_PrintWord(3*21, "GPS sensor module",0, hott_txt_msg);
#endif
#ifdef HOTT_SIM_EAM_SENSOR
             if(tmp == (HOTT_TELEMETRY_EAM_SENSOR_ID & 0x0f))    hott_txt_msg = HOTT_HandleTextMode(addr, hott_txt_msg);
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
             if(tmp == (HOTT_TELEMETRY_VARIO_SENSOR_ID & 0x0f))  hott_txt_msg = HOTT_PrintWord(3*21, "Vario sensor module",0, hott_txt_msg);
#endif
#ifdef HOTT_SIM_GAM_SENSOR
             if(tmp == (HOTT_TELEMETRY_GAM_SENSOR_ID & 0x0f))    
			 {
				HandleHott(addr, &hott_txt_msg->msg_txt);
				hott_txt_msg->stop_uint8_t = 0x7d;
				_hott_send_msg_text(_hott_serial_buffer_textmode, sizeof(HOTT_TEXTMODE_MSG_t));

			 }
			 else
			 {
				 // this question was for someone else
			 }
#endif           
		}
#endif
           break;
//*****************************************************************************
          case HOTT_BINARY_MODE_REQUEST_ID:
#ifdef HOTT_SIM_GPS_SENSOR
			//GPS module binary mode
            if(addr == HOTT_TELEMETRY_GPS_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_GPS",micros() / 1000);
			 _hott_update_gps_msg();
             _hott_send_msg(_hott_serial_buffer, sizeof(struct HOTT_GPS_MSG));
             break;
            }
#endif
#ifdef HOTT_SIM_EAM_SENSOR
            if(addr == HOTT_TELEMETRY_EAM_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_EAM",micros() / 1000);
			  _hott_update_eam_msg();
		      _hott_send_msg(_hott_serial_buffer, sizeof(struct HOTT_EAM_MSG));
              break;
		    }
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
            if(addr == HOTT_TELEMETRY_VARIO_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_VARIO",micros() / 1000);
			  _hott_update_vario_msg();
		      _hott_send_msg(_hott_serial_buffer, sizeof(struct HOTT_VARIO_MSG));
              break;
		    }
#endif
#ifdef HOTT_SIM_GAM_SENSOR
            if(addr == HOTT_TELEMETRY_GAM_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_GAM",micros() / 1000);
			  _hott_update_gam_msg();
		      _hott_send_msg(_hott_serial_buffer, sizeof(struct HOTT_GAM_MSG));
			  lastComm = GetTime();
			  PORTD ^= (1<<PD5); // toggle LED
              break;
		    }
#endif

//END: case HOTT_BINARY_MODE_REQUEST_ID:
           break;
//*****************************************************************************         
          default:
            //Serial.printf("0x%02x Mode for 0x%02x\n", c, addr);
            break;
        }
      } else {
        //ignore data from other sensors
        _HOTT_PORT_flush();
        _hott_serial_request_timer = 0;
      }
    }
}

void _hott_send_msg(uint8_t *buffer, int len) {
  if(_hott_telemetry_is_sending == true) return;
  _hott_msg_ptr = _hott_serial_buffer;
  _hott_msg_len = len +1; //len + 1 uint8_t for crc
}

#ifdef HOTT_SIM_TEXTMODE
void _hott_send_msg_text(uint8_t *buffer, int len) {
  if(_hott_telemetry_is_sending == true) return;
  _hott_msg_ptr = _hott_serial_buffer_textmode;
  _hott_msg_len = len +1; //len + 1 uint8_t for crc
}
#endif

#ifdef HOTT_SIM_GAM_SENSOR
#ifdef HOTT_ALARMS
	static uint8_t _hott_gam_voice_alarm = 0;
	static uint8_t _hott_gam_alarm_invers1 = 0;
	static uint8_t _hott_gam_alarm_invers2 = 0;
#endif





void _hott_update_gam_msg() {
	struct HOTT_GAM_MSG	*hott_gam_msg =	(struct HOTT_GAM_MSG *)&_hott_serial_buffer[0];
	uint16_t tmp;
	static uint8_t old_beep = 0;
	static uint32_t old_time = 0;
	uint32_t beepTime;
	
	memset(hott_gam_msg, 0, sizeof(struct HOTT_GAM_MSG));
	hott_gam_msg->start_uint8_t = 0x7c;
	hott_gam_msg->gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
	hott_gam_msg->sensor_id = 0xd0;
	hott_gam_msg->stop_uint8_t = 0x7d;
	uint8_t beep = 0;
	if(myData.almH>0)
	{	
		beep = 12; // 12 == 05 in langsam ; 37..: 1..5 piepser ; 46.. dto in hoch.
		beepTime = 100000000; //(5 beeps)
	}		
	if(myData.almL>0)
	{
		beep = 5;
		beepTime = 300000000;
	}	
	hott_gam_msg->alarm_invers1 = (beep>0)?0x07:0; // bit 0: all cells invers
	hott_gam_msg->alarm_invers2 = (beep>0)?0x03:0;
	
	uint32_t localTime = GetTime();	

	if(beep != old_beep)
	{
		old_time = localTime;
		old_beep = beep;
	}
	
	if(localTime - old_time> beepTime || SILENCE == 1)
	{
		beep = 0; // mute
	}
	
	hott_gam_msg->warning_beeps = beep;// 'Q' is min cell alm
	hott_gam_msg->temperature1 = 20;
	hott_gam_msg->temperature2 = 20; // 0°C
	hott_gam_msg->altitude_L = 0xf4; // -500m = 0
	hott_gam_msg->altitude_H = 0x01;
	hott_gam_msg->climbrate_L = 0x30 ;
	hott_gam_msg->climbrate_H = 0x75 ;
	hott_gam_msg->climbrate3s = 120 ;  // 0 m/3s using filtered data here
	tmp = myData.CurrentmA / 100;
	hott_gam_msg->current_L = tmp&0x00ff;
	hott_gam_msg->current_H = (tmp&0xff00)>>8;
	tmp = myData.UBatt/100; // 0.1 V steps
	hott_gam_msg->main_voltage_L = tmp&0x00ff;
	hott_gam_msg->main_voltage_H = (tmp&0xff00)>>8;
	hott_gam_msg->batt1_L = tmp&0x00ff;
	hott_gam_msg->batt1_H = (tmp&0xff00)>>8;
	hott_gam_msg->batt2_L = tmp&0x00ff;
	hott_gam_msg->batt2_H = (tmp&0xff00)>>8;
	hott_gam_msg->cell1 = myData.UCell[0]/20;
	hott_gam_msg->cell2 = myData.UCell[1]/20;
	hott_gam_msg->cell3 = myData.UCell[2]/20;
	hott_gam_msg->cell4 = myData.UCell[3]/20;
	hott_gam_msg->cell5 = myData.UCell[4]/20;	
	hott_gam_msg->cell6 = myData.UCell[5]/20;
	hott_gam_msg->min_cell_volt = myData.UminCellV/20;
	hott_gam_msg->min_cell_volt_num = myData.UminCellNum;
	uint32_t c = myData.CapamAs/60/60/10; // calc to 10mAh steps
	hott_gam_msg->batt_cap_L = c&0x00ff;
	hott_gam_msg->batt_cap_H = (c&0xff00)>>8;
	hott_gam_msg->fuel_procent = myData.fill;	// my fuel are electrons :)
	hott_gam_msg->rpm_L = 0; 
	hott_gam_msg->rpm_H = 0;
	hott_gam_msg->rpm2_L = 0x0; 
	hott_gam_msg->rpm2_H = 0x0;
	hott_gam_msg->speed_L = 0;
	hott_gam_msg->speed_H = 0;
	hott_gam_msg->fuel_ml_L = 0;
	hott_gam_msg->fuel_ml_H = 0;

    //display ON when motors are armed
   /* if (1) {
    	hott_gam_msg->alarm_invers2 |= 0x80;
    } else {
        hott_gam_msg->alarm_invers2 &= 0x7f;
    }*/
}
#endif	//HOTT_SIM_GAM_SENSOR

#ifdef HOTT_SIM_EAM_SENSOR
#ifdef HOTT_ALARMS
static uint8_t _hott_eam_voice_alarm = 0;
static uint8_t _hott_eam_alarm_invers1 = 0;
static uint8_t _hott_eam_alarm_invers2 = 0;
#endif
//Update EAM sensor data
void _hott_update_eam_msg() {
	struct HOTT_EAM_MSG	*hott_eam_msg =	(struct HOTT_EAM_MSG *)&_hott_serial_buffer[0];
	memset(hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));
	hott_eam_msg->start_uint8_t = 0x7c;
	hott_eam_msg->eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
	hott_eam_msg->sensor_id = 0xe0;
	hott_eam_msg->stop_uint8_t = 0x7d;
#ifdef HOTT_ALARMS
	hott_eam_msg->warning_beeps = _hott_eam_voice_alarm;
	hott_eam_msg->alarm_invers1 = _hott_eam_alarm_invers1;
	hott_eam_msg->alarm_invers2 = _hott_eam_alarm_invers2;
#endif
	
	(int &)hott_eam_msg->batt1_voltage_L = (int)(0);
	(int &)hott_eam_msg->batt2_voltage_L = (int)(0);
	hott_eam_msg->temp1 = (uint8_t)((barometer.get_temperature() / 10) + 20);
	hott_eam_msg->temp2 = 20;	//0°
	(int &)hott_eam_msg->altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;
	(int &)hott_eam_msg->current_L = current_amps1*10;
	(int &)hott_eam_msg->main_voltage_L = (int)(battery_voltage1 * 10.0);
	(int &)hott_eam_msg->batt_cap_L = current_total1 / 10;
	(int &)hott_eam_msg->speed_L = (int)((float)(g_gps->ground_speed * 0.036));

  	(int &)hott_eam_msg->climbrate_L = 30000 + climb_rate_actual;  
  	hott_eam_msg->climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s using filtered data here
  	
    //display ON when motors are armed
    if (motors.armed()) {
       hott_eam_msg->alarm_invers2 |= 0x80;
     } else {
       hott_eam_msg->alarm_invers2 &= 0x7f;
     }
}
#endif	//HOTT_SIM_EAM_SENSOR

#ifdef HOTT_SIM_GPS_SENSOR
// Updates GPS message values
#ifdef HOTT_ALARMS
static uint8_t _hott_gps_voice_alarm = 0;
static uint8_t _hott_gps_alarm_invers1 = 0;
static uint8_t _hott_gps_alarm_invers2 = 0;
#endif

void _hott_update_gps_msg() { 
  struct HOTT_GPS_MSG	*hott_gps_msg =	(struct HOTT_GPS_MSG *)&_hott_serial_buffer[0];
  memset(hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));
  hott_gps_msg->start_uint8_t = 0x7c;
  hott_gps_msg->gps_sensor_id = 0x8a;
  hott_gps_msg->sensor_id = 0xa0;
  
  hott_gps_msg->version = 0x00;
  hott_gps_msg->end_uint8_t = 0x7d;
#ifdef HOTT_ALARMS
  hott_gps_msg->warning_beeps = _hott_gps_voice_alarm;
  hott_gps_msg->alarm_invers1 = _hott_gps_alarm_invers1;
  hott_gps_msg->alarm_invers2 = _hott_gps_alarm_invers2;
#endif
  // update GPS telemetry data
  (int &)hott_gps_msg->msl_altitude_L = (int)g_gps->altitude / 100;  //meters above sea level  

  hott_gps_msg->flight_direction = (uint8_t)(g_gps->ground_course / 200);  // in 2* steps
  (int &)hott_gps_msg->gps_speed_L = (int)((float)(g_gps->ground_speed * 0.036));

  
  if(g_gps->status() == GPS::GPS_OK) {
    hott_gps_msg->alarm_invers2 = 0;
    hott_gps_msg->gps_fix_char = '3';  
    hott_gps_msg->free_char3 = '3';  //3D Fix according to specs...
  } else {
    //No GPS Fix
    hott_gps_msg->alarm_invers2 = 1;
    hott_gps_msg->gps_fix_char = '-';
    hott_gps_msg->free_char3 = '-';
    (int &)hott_gps_msg->home_distance_L = (int)0; // set distance to 0 since there is no GPS signal
  }
  
  switch(control_mode) {
	case AUTO:
        case LOITER:
          //Use home direction field to display direction an distance to next waypoint
          {
          	  int32_t dist = get_distance_cm(&current_loc, &next_WP);
	          (int &)hott_gps_msg->home_distance_L = dist < 0 ? 0 : dist / 100;
    	      hott_gps_msg->home_direction = get_bearing(&current_loc, &next_WP) / 200; //get_bearing() return value in degrees * 100
        	  //Display WP to mark the change of meaning!
           	hott_gps_msg->free_char1 ='W';
           	hott_gps_msg->free_char2 ='P';
           }
           break;

        default:
        //Display Home direction and distance
        {
          int32_t dist = get_distance_cm(&current_loc, &home);
          (int &)hott_gps_msg->home_distance_L = dist < 0 ? 0 : dist / 100;
          hott_gps_msg->home_direction = get_bearing(&current_loc, &home) / 200; //get_bearing() return value in degrees * 100
          hott_gps_msg->free_char1 = 0;
          hott_gps_msg->free_char2 = 0;
          break;
        }
	}

  
  //
  //latitude
  float coor = current_loc.lat / 10000000.0;
  if(coor < 0.0)
    coor *= -1.0;
  int lat = coor;  //degree
  coor -= lat;
  lat *= 100;
  
  coor *= 60;
  int tmp = coor;  //minutes
  lat += tmp;
  //seconds
  coor -= tmp;
  coor *= 10000.0;
  int lat_sec = coor;
  //
  // Longitude
  coor = current_loc.lng / 10000000.0;
  if(coor < 0.0)
    coor *= -1.0;
  int lng = coor;
  coor -= lng;
  lng *= 100;
  
  coor *= 60;
  tmp = coor;  //minutes
  lng += tmp;
  //seconds
  coor -= tmp;
  coor *= 10000.0;
  int lng_sec = coor;
  
  if(current_loc.lat >= 0) {
    hott_gps_msg->pos_NS = 0;  //north
  } else {
    hott_gps_msg->pos_NS = 1;  //south
  }
  (int &)hott_gps_msg->pos_NS_dm_L = (int)lat;
  (int &)hott_gps_msg->pos_NS_sec_L = (int)(lat_sec);
  
  if(current_loc.lng >= 0) {
     hott_gps_msg->pos_EW = 0; //east
  } else {
     hott_gps_msg->pos_EW = 1; //west
  }
  (int &)hott_gps_msg->pos_EW_dm_L = (int)(lng);
  (int &)hott_gps_msg->pos_EW_sec_L = (int)(lng_sec);
  
  (int &)hott_gps_msg->altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;  //meters above ground

  (int &)hott_gps_msg->climbrate_L = 30000 + climb_rate_actual;  
  hott_gps_msg->climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s
  
  hott_gps_msg->gps_satelites = (uint8_t)g_gps->num_sats;
  
  hott_gps_msg->angle_roll = ahrs.roll_sensor / 200;
  hott_gps_msg->angle_nick = ahrs.pitch_sensor / 200;
  
  hott_gps_msg->angle_compass = ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) / 2;
  //hott_gps_msg->flight_direction = hott_gps_msg->angle_compass;	//

  uint32_t t = g_gps->time;
  hott_gps_msg->gps_time_h = t / 3600000;
  t -= (hott_gps_msg->gps_time_h * 3600000);
  
  hott_gps_msg->gps_time_m = t / 60000;
  t -= hott_gps_msg->gps_time_m * 60000;
  
  hott_gps_msg->gps_time_s = t / 1000;
  hott_gps_msg->gps_time_sss = t - (hott_gps_msg->gps_time_s * 1000);
}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
#ifdef HOTT_ALARMS
static uint8_t _hott_vario_voice_alarm = 0;
static uint8_t _hott_vario_alarm_invers1 = 0;
#endif

void _hott_update_vario_msg() {
	static int _hott_max_altitude = 0;
	static int _hott_min_altitude = 0;

	struct HOTT_VARIO_MSG	*hott_vario_msg =	(struct HOTT_VARIO_MSG *)&_hott_serial_buffer[0];
	memset(hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));
	hott_vario_msg->start_uint8_t = 0x7c;
	hott_vario_msg->vario_sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
	hott_vario_msg->sensor_id = 0x90;
	hott_vario_msg->stop_uint8_t = 0x7d;
#ifdef HOTT_ALARMS
	hott_vario_msg->warning_beeps = _hott_vario_voice_alarm;
	hott_vario_msg->alarm_invers1 = _hott_vario_alarm_invers1;
#endif	
	(int &)hott_vario_msg->altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;

	if( (current_loc.alt - home.alt) > _hott_max_altitude)
		_hott_max_altitude = (current_loc.alt - home.alt);
	(int &)hott_vario_msg->altitude_max_L = (int)(_hott_max_altitude / 100)+500;

	if( (current_loc.alt - home.alt) < _hott_min_altitude)
		_hott_min_altitude = (current_loc.alt - home.alt);
	(int &)hott_vario_msg->altitude_min_L = (int)(_hott_min_altitude / 100)+500;
	
	(int &)hott_vario_msg->climbrate_L = 30000 + climb_rate_actual;
	(int &)hott_vario_msg->climbrate3s_L = 30000 + climb_rate;	//using filtered data here
	(int &)hott_vario_msg->climbrate10s_L = 30000;	//TODO: calc this stuff
	
	hott_vario_msg->compass_direction = ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) / 2;
	
	//Free text processing
	char mode[10];
	char armed[10];
	strcpy(mode,flight_mode_strings[control_mode]);
	
	if (motors.armed()) {
          strcpy(armed,"ARMED");
	} else {
          strcpy(armed,"DISAR");
	}
	memset(hott_vario_msg->text_msg,0x20,HOTT_VARIO_MSG_TEXT_LEN);
	snprintf((char*)hott_vario_msg->text_msg,HOTT_VARIO_MSG_TEXT_LEN, "%s %s", &armed, &mode);
}
#endif

/*
  Converts a "black on white" string into inverted one "white on black"
  Works in text mode only!
*/
uint8_t * hott_invert_chars(uint8_t *str, int cnt) {
	//if(str == 0) return str;
	//int len = strlen(str);
	//if((len < cnt)  && cnt > 0) len = cnt;
	//for(int i=0; i< len; i++) {
	for(int i=0; i< cnt; i++) {
		str[i] = (uint8_t)(0x80 + (uint8_t)str[i]);
	}
	return str;
}

/*uint8_t * _hott_invert_all_chars(uint8_t *str) {
    return _hott_invert_chars(str, 0);
}*/

//****************************************************************************************
// Alarm stuff
//
//HoTT alarm macro
#ifdef HOTT_ALARMS
#define HOTT_ALARM_NUM(a) (a-0x40)

struct _hott_alarm_event_T {
	uint16_t alarm_time; 		//Alarm play time in 1sec units
	uint16_t alarm_time_replay;	//Alarm repeat time in 1sec. units. 0 -> One time alarm
								//forces a delay between new alarms of the same kind
	uint8_t visual_alarm1;		//Visual alarm bitmask
	uint8_t visual_alarm2;		//Visual alarm bitmask
	uint8_t alarm_num;			//Alarm number 0..255 (A-Z)
	uint8_t alarm_profile;		//profile id ie HOTT_TELEMETRY_GPS_SENSOR_ID
};
typedef struct _hott_alarm_event_T _hott_alarm_event;

#define HOTT_ALARM_QUEUE_MAX		5
//TODO: better queueing solution
static _hott_alarm_event _hott_alarm_queue[HOTT_ALARM_QUEUE_MAX];
static _hott_alarm_event _hott_alarm_replay_queue[HOTT_ALARM_QUEUE_MAX];
static uint8_t _hott_alarmCnt = 0;
static uint8_t _hott_alarm_ReplayCnt = 0;

//
// checks if an alarm exists in active queue
//
uint8_t _hoot_alarm_active_exists(struct _hott_alarm_event_T *alarm) {
	//check active alarms
	for(uint8_t i=0; i<_hott_alarmCnt; i++) {
		if(_hott_alarm_queue[i].alarm_num == alarm->alarm_num &&
			_hott_alarm_queue[i].alarm_profile == alarm->alarm_profile) {
			//alarm exists.
			return true;
		}
	}
	return false;
}

//
// checks if an alarm exists in replay queue
//
uint8_t _hoot_alarm_replay_exists(struct _hott_alarm_event_T *alarm) {
	//check replay delay queue
	for(uint8_t i=0; i<_hott_alarm_ReplayCnt; i++) {
		if(_hott_alarm_replay_queue[i].alarm_num == alarm->alarm_num &&
			_hott_alarm_replay_queue[i].alarm_profile == alarm->alarm_profile) {
			//alarm exists
			return true;
		}
	}
	return false;
}

//
// checks if an alarm exists
//
uint8_t _hoot_alarm_exists(struct _hott_alarm_event_T *alarm) {
	if(_hoot_alarm_active_exists(alarm))
		return true;
	if(_hoot_alarm_replay_exists(alarm))
		return true;
	return false;
}

//
// adds an alarm to active queue
//
void _hott_add_alarm(struct _hott_alarm_event_T *alarm) {
	if(alarm == 0)
		return;
	if(_hott_alarmCnt >= HOTT_ALARM_QUEUE_MAX)
		return;	//no more space left...
	if(_hoot_alarm_exists(alarm)) 
		return;
	// we have a new alarm
	memcpy(&_hott_alarm_queue[_hott_alarmCnt++], alarm, sizeof(struct _hott_alarm_event_T));
//	Serial.print("\nadd to queue");
}

//
// adds an alarm to replay queue
//
void _hott_add_replay_alarm(struct _hott_alarm_event_T *alarm) {
	if(alarm == 0)
		return;
	if(_hott_alarm_ReplayCnt >= HOTT_ALARM_QUEUE_MAX)
		return;	//no more space left...
	if(_hoot_alarm_replay_exists(alarm)) 
		return;
	// we have a new alarm
	memcpy(&_hott_alarm_replay_queue[_hott_alarm_ReplayCnt++], alarm, sizeof(struct _hott_alarm_event_T));
//	Serial.print("\nadd to replay queue");
}

//
//removes an alarm from active queue
//first alarm at offset 1
//
void _hott_remove_alarm(uint8_t num) {
	if(num > _hott_alarmCnt || num == 0)	//has to be > 0
		return; // possibile error

	if(_hott_alarmCnt != 1) {
		memcpy(&_hott_alarm_queue[num-1], &_hott_alarm_queue[num], sizeof(struct _hott_alarm_event_T) * (_hott_alarmCnt - num) );
	}
	--_hott_alarmCnt;
//	Serial.print("\nremove from queue");
}

//
//removes an alarm from replay queue
//first alarm at offset 1
//
void _hott_remove_replay_alarm(uint8_t num) {
	if(num > _hott_alarm_ReplayCnt || num == 0)	//has to be > 0
		return; // possibile error

	if(_hott_alarm_ReplayCnt != 1) {
		memcpy(&_hott_alarm_replay_queue[num-1], &_hott_alarm_replay_queue[num], sizeof(struct _hott_alarm_event_T) * (_hott_alarm_ReplayCnt - num) );
	}
	--_hott_alarm_ReplayCnt;
//	Serial.print("\nremove from delay queue");
}

//
// Updates replay delay queue
//
void _hott_update_replay_queue(void) {
static uint8_t t = 0;
	if(++t < 50)
		return;
	//every second
	t = 0;

	for(uint8_t i=0; i< _hott_alarm_ReplayCnt; i++) {
		if(--_hott_alarm_replay_queue[i].alarm_time_replay == 0) {
			//remove it
			_hott_remove_replay_alarm(i+1);
			i--;
			continue;
		}
	}
}

//
// Sets a voice alarm value
//
void _hott_set_voice_alarm(uint8_t profile, uint8_t value) {
	switch(profile) {
#ifdef HOTT_SIM_EAM_SENSOR
		case HOTT_TELEMETRY_EAM_SENSOR_ID:
			_hott_eam_voice_alarm = value;
			break;
#endif  
#ifdef HOTT_SIM_GPS_SENSOR
		case HOTT_TELEMETRY_GPS_SENSOR_ID:
			_hott_gps_voice_alarm = value;
			break;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
		case HOTT_TELEMETRY_VARIO_SENSOR_ID:
			_hott_vario_voice_alarm = value;
			break;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
		 case HOTT_TELEMETRY_GAM_SENSOR_ID:
			_hott_gam_voice_alarm = value;
			break;
#endif
		default:
			break;
	}
}

//
// active alarm scheduler
//
void _hott_alarm_scheduler() {
	static uint8_t activeAlarmTimer = 3* 50;
	static uint8_t activeAlarm = 0;

	if(_hott_alarmCnt < 1)
		return;	//no alarms	

	uint8_t vEam = 0;
	uint8_t vEam2 = 0;
	uint8_t vVario = 0;
	uint8_t vGps = 0;
	uint8_t vGps2 = 0;
	uint8_t vGam = 0;
	uint8_t vGam2 = 0;
	
	for(uint8_t i = 0; i< _hott_alarmCnt; i++) {
		if(_hott_alarm_queue[i].alarm_time == 0) {
			//end of alarm, remove it
			//no check for current msg to be send, so maybe the crc will be wrong
			_hott_set_voice_alarm(_hott_alarm_queue[i].alarm_profile, 0);
			if(_hott_alarm_queue[i].alarm_time_replay != 0)
				_hott_add_replay_alarm(&_hott_alarm_queue[i]);
			_hott_remove_alarm(i+1);	//first alarm at offset 1
			--i;	//correct counter
//			Serial.print("\nremoved alarm");
			continue;
		}
		
		//
		switch(_hott_alarm_queue[i].alarm_profile) {
#ifdef HOTT_SIM_EAM_SENSOR
			case HOTT_TELEMETRY_EAM_SENSOR_ID:
				vEam |= _hott_alarm_queue[i].visual_alarm1;
				vEam2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif  
#ifdef HOTT_SIM_GPS_SENSOR  
			case HOTT_TELEMETRY_GPS_SENSOR_ID:
				vGps |= _hott_alarm_queue[i].visual_alarm1;
				vGps2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
               	case HOTT_TELEMETRY_VARIO_SENSOR_ID:
				vVario |= _hott_alarm_queue[i].visual_alarm1;
				break;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
                case HOTT_TELEMETRY_GAM_SENSOR_ID:
				vGam |= _hott_alarm_queue[i].visual_alarm1;		
				vGam2 |= _hott_alarm_queue[i].visual_alarm2;		
				break;
#endif
			default:
				break;
		}
	} //end: visual alarm loop

	// Set all visual alarms
#ifdef HOTT_SIM_EAM_SENSOR
		_hott_eam_alarm_invers1 |= vEam;
		_hott_eam_alarm_invers2 |= vEam2;
#endif  
        
#ifdef HOTT_SIM_GPS_SENSOR
        _hott_gps_alarm_invers1 |= vGps;
		_hott_gps_alarm_invers2 |= vGps2;
        
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
		_hott_vario_alarm_invers1 |= vVario;
#endif
        
#ifdef HOTT_SIM_GAM_SENSOR
        _hott_gam_alarm_invers1 |= vGam;
        _hott_gam_alarm_invers2 |= vGam2;
#endif
	

	if(activeAlarm != 0) { //is an alarm active
		if ( ++activeAlarmTimer % 50 == 0 ) {	//every 1sec
			_hott_alarm_queue[activeAlarm-1].alarm_time--;
		}
		if ( activeAlarmTimer < 50 * 2) //alter alarm every 2 sec
			return;
	}
	activeAlarmTimer = 0;

	if(++activeAlarm > _hott_alarmCnt) {
		activeAlarm = 1;
	}
	if(_hott_alarmCnt <= 0) {
		activeAlarm = 0;
		return;
	}
//Serial.printf("\nAlarm change %d",activeAlarm);
	_hott_set_voice_alarm(_hott_alarm_queue[activeAlarm-1].alarm_profile, _hott_alarm_queue[activeAlarm-1].alarm_num);
}

//****************************************************************************************
// Sensor specific code
//
#ifdef HOTT_SIM_EAM_SENSOR
//
// triggers max consumed mAh alarm
//
void _hott_eam_check_mAh() {
	_hott_alarm_event _hott_ema_alarm_event;
	if( (g.battery_monitoring == 4) && (g.pack_capacity - current_total1) <= 0.0) {
		_hott_ema_alarm_event.alarm_time = 6;	//1sec units
		_hott_ema_alarm_event.alarm_time_replay = 15;	//1sec units
		_hott_ema_alarm_event.visual_alarm1 = 0x01;	//blink mAh
		_hott_ema_alarm_event.visual_alarm2 = 0;
		_hott_ema_alarm_event.alarm_num = HOTT_ALARM_NUM('V');
		_hott_ema_alarm_event.alarm_profile = HOTT_TELEMETRY_EAM_SENSOR_ID;
		_hott_add_alarm(&_hott_ema_alarm_event);
	}
}

//
// triggers low main power voltage alarm
//
void _hott_eam_check_mainPower() {
	//voltage sensor needs some time at startup
	if((millis() > 10000) && (g.battery_monitoring != 0) && (battery_voltage1 < g.low_voltage)) {
		_hott_alarm_event _hott_ema_alarm_event;
		_hott_ema_alarm_event.alarm_time = 6;	//1sec units
		_hott_ema_alarm_event.alarm_time_replay = 30; //1sec unit
		_hott_ema_alarm_event.visual_alarm1 = 0x80;	//blink main power
		_hott_ema_alarm_event.visual_alarm2 = 0;
		_hott_ema_alarm_event.alarm_num = HOTT_ALARM_NUM('P');
		_hott_ema_alarm_event.alarm_profile = HOTT_TELEMETRY_EAM_SENSOR_ID;
		_hott_add_alarm(&_hott_ema_alarm_event);
	}
}
#endif

//
//	alarm triggers to check
//
void _hoot_check_alarm()  {
#ifdef HOTT_SIM_EAM_SENSOR
	_hott_eam_check_mAh();
	_hott_eam_check_mainPower();
#endif
}
#endif // HoTT Alarms





















