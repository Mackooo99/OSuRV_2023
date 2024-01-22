
///////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

#include "avr_io_bitfields.h"

#include "motor_ctrl_pkgs.hpp"

#include "VarSpeedServo.h"


#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

//////////////////

// Motors pins.
#define PG 2 // INT2
#define PWM 3 // OC2B
#define DIR 4
#define MAYBE_BREAK 5

// For local jog.
#define PB_CW 11 // Red.
#define PB_CCW 12 // Blue.

#define LED_NO_ERR LED_BUILTIN

//////////////////

#define MAX_EFF 100
#define RAMP_TIME 5 // [sec]

/////////////////
#define PACE 50

////////////////////////////////////////////////////////////////////////////////////
VarSpeedServo servo0, servo1, servo2, servo3;

bool packetCheck = false; 

i8 joyStickVal[4];
i8 *packetVal;

static volatile i16 pos = 0;

enum dir_t {
	CW = +1,
	CCW = -1
};
static volatile dir_t dir = CW;


static void set_abs_eff(u8 percents) {
	tc2.r.ocrb = percents;
}
static u8 get_abs_eff() {
	return tc2.r.ocrb;
}
static void set_dir(dir_t d) {
	dir = d;
	digitalWrite(DIR, dir == CW);
}

static void pos_pulse() {
	pos += dir;
}

///////////////////////////////////////////////////////////////////////////////

static pkg_payload(err_get_rsp) err_buf = {
	.err_code = pkg_err_code__none
};

static void error_occured(
	pkg_err_codes err_code,
	const pkg_header& h = pkg_header()
) {
	digitalWrite(LED_NO_ERR, 0);
	// This pkg type should target master.
	err_buf.err_code = err_code;
	err_buf.data.header = h;
}

///////////////////////////////////////////////////////////////////////////////

// 2x longer than LOOP_HZ.
#define WATCHDOG_TOP (2*10000/(LOOP_HZ)-1)
static volatile u16 watchdog = WATCHDOG_TOP;
static void rst_watchdog() {
	watchdog = WATCHDOG_TOP;
}

///////////////////////////////////////////////////////////////////////////////


static void set_jog_eff(i16 jog_eff) {
	if (jog_eff < 0) {
		set_dir(CCW);
		jog_eff = -jog_eff;
	} else {
		set_dir(CW);
	}
	u8 wr = (jog_eff >> 7) + (jog_eff >> 9) + 1;
	
	set_abs_eff(wr);
}

enum jog_dir_t {
	JOG_CW = +1,
	JOG_NONE = 0,
	JOG_CCW = -1
};
static volatile jog_dir_t jog_dir = JOG_NONE;

#define CEILING		MAX_EFF * 100
#define STEP 		10 / RAMP_TIME

// 10kHz
ISR(TIMER2_COMPA_vect) {
	if(get_abs_eff() != 0){
		// If master does not update effort for some time,
		// set it to 0.
		if(watchdog == 0){
			set_abs_eff(0);
			
			error_occured(pkg_err_code__watchdog_stop);
		}else{
			watchdog--;
		}
	}

	// Implement ramp up and down on jog,
	// not to jump when PB is pressed,
	// but to slowly increment eff from 0 to MAX_EFF,
	// and to slowly decrement eff from MAX_EFF to 0 when PB is released.
	// Check jog code in loop(), move it here, and refactor it to have ramps.
	
	// na pritisnuto dugme (bilo koje od dva) brojac jog_eff se inkrementira 
	// po taktu od 10khz. potom se taj broj uzima,
	// deli sa 10 i on predstavlja brzinu FLOOR-CEILING%
	// isto se desava kada je pusteno dugme,
	// samo sto brzina polako pada sa dostignute - na FLOOR.
	
	static i16 jog_eff = 0;
	switch(jog_dir){
		case JOG_CW:
			if(jog_eff >= CEILING){
				jog_eff = CEILING;
			}else{
				jog_eff += STEP; 
			}
			set_jog_eff(jog_eff);
			break;
		case JOG_CCW:
			if(jog_eff <= -CEILING){
				jog_eff = -CEILING;
			}else{
				jog_eff -= STEP;
			}
			set_jog_eff(jog_eff);
			break;
		case JOG_NONE:
			if(jog_eff < 0){
				++jog_eff;
				set_jog_eff(jog_eff);
			}else if(jog_eff > 0){
				--jog_eff;
				set_jog_eff(jog_eff);
			}
			break;
	}
}




static u8 addr = 0xe;

void setup() {

	 servo0.attach(11);  // Mount pin
	 servo1.attach(10);  // Shoulder pin
	 servo2.attach(9);   // Elbow pin
	 servo3.attach(5);   // Claw pin

	// WARNING: Do not print anything if not asked for by protocol.
	Serial.begin(BAUDRATE);
	
	// Error is indicated by this LED off.
	pinMode(LED_NO_ERR, OUTPUT);
	digitalWrite(LED_NO_ERR, 1);

	// Find out ID.
	pinMode(A5, OUTPUT);
	digitalWrite(A5, 0);
	for(i8 pin = A4; pin >= A0; pin--){
		pinMode(pin, INPUT_PULLUP);
	}
	u8 id = 0;
	for(i8 pin = A4; pin >= A0; pin--){
		id <<= 1;
		id |= !digitalRead(pin);
	}
	
	addr = id + pkg_addr__motor_id_0;


	pinMode(PB_CW, INPUT_PULLUP);
	pinMode(PB_CCW, INPUT_PULLUP);
	
	pinMode(PG, INPUT);
	// ISR on pin PG pin.
/*
	attachInterrupt(digitalPinToInterrupt(PG), pos_pulse, RISING);
	
	pinMode(DIR, OUTPUT);
	set_dir(CW);

	pinMode(PWM, OUTPUT);
	digitalWrite(PWM, 1); // Off.
	// Cleanup Arduino stuff.
	tc2.r.tccra = 0;
	tc2.r.tccrb = 0;
	irq.r.timsk[2] = 0;
	tc2.f.comb = 0b11; // Inverted output on OC2B.
	tc2.r.ocrb = 0; // Off.
	tc2.r.ocra = 100; // Before setting mode.
	// Phase Correct PWM [0,OCRA] aka mode 5.
	tc2.f.wgm0 = 1;
	tc2.f.wgm1 = 0;
	tc2.f.wgm2 = 1;
	// For 10kHz PWM.
	tc2.f.cs = 0b010; // CLK/8
	// Enable IRQ for Watchdog.
	irq.f.timsk2.ociea = 1;
*/

}


template<typename T>
static void read_serial(T& t) {
	Serial.readBytes((u8*)&t, sizeof(T));
}

template<typename T>
static void write_serial(const T& t) {
	Serial.write((u8*)&t, sizeof(T));
}

static void pass_through(const pkg_header& h) {
	static u8 pl_crc_buf[pkg_payload_max_size+1]; // +1 for crc
	
	u8 payload_size = pkg_payload_sizes[h.type];
	Serial.readBytes(pl_crc_buf, payload_size+1); // +1 for crc
	
	u8 exp_crc = pl_crc_buf[payload_size];
	u8 obs_crc = CRC8().add(h).add(pl_crc_buf, payload_size).get_crc();
	if(obs_crc != exp_crc){
		error_occured(pkg_err_code__crc_error, h);
	}
	//TODO Not to use pkg.
	{
		write_serial(h);
		Serial.write(pl_crc_buf, payload_size+1);
	}
}


int map_adc(
	i8 adc_value,
	int low_angle,
	int high_angle
) 	{
	int angle;
    if(adc_value < 512){
      angle = map(adc_value, 0, 511, low_angle, 90);
    }else{
      angle = map(adc_value, 512, 1023, 90, high_angle);
    }
    return angle;  
}

void DoCommands(i8* motor_index){
	
	if(!packetCheck){
		servo0.write(motor_index[0], PACE);
		servo1.write(motor_index[1], PACE);
		servo2.write(motor_index[2], PACE);
		servo3.write(motor_index[3], PACE);
	}
	else{
		//servo0.write(motor_index[0] + 90, PACE);
		//servo1.write(motor_index[1] + 90, PACE);
		//servo2.write(motor_index[2] + 90, PACE);
		//servo3.write(motor_index[3] + 90, PACE);
	}
}

void loop() {
	
	if(Serial.available()) {
		pkg_header h;
		
		read_serial(h);
		if(h.magic != PKG_MAGIC){
			error_occured(pkg_err_code__wrong_pkg_magic, h);
			//TODO Not to use pkg.
			//TODO Seek another pkg_magic.
		}
		
		if(h.daddr != addr && h.daddr != pkg_addr__broadcast){
			// Not adressed on me, directly or broadcasted.
			pass_through(h);
		}else{
			if(h.saddr != pkg_addr__master){
				error_occured(pkg_err_code__some_slave_ordering, h);
				//TODO Not to use pkg.
			}
			
			u8 crc;
			// Targeted to this slave.
			switch(h.type){
				case pkg_type(err_get_req): {
						read_serial(crc);
						if(crc != CRC8().add(h).get_crc()){
							error_occured(pkg_err_code__crc_error, h);
						}
						//TODO Not to use pkg.
						{
							if(h.daddr == pkg_addr__broadcast){
								write_serial(h);
								write_serial(crc);
							}
							
							pkg_header h2 = {
								.magic = PKG_MAGIC,
								.daddr = pkg_addr__master,
								.saddr = addr,
								.type = pkg_type(err_get_rsp),
								.id = h.id
							};
							write_serial(h2);
							write_serial(err_buf);
							write_serial(CRC8().add(h2).add(err_buf).get_crc());
							err_buf.err_code = pkg_err_code__none;
							digitalWrite(LED_NO_ERR, 1);
						}
					} break;
				case pkg_type(err_get_rsp):
					read_serial(crc);
					error_occured(pkg_err_code__master_responding, h);
					break;
				case pkg_type(addr_get_req): {
						read_serial(crc);
						if(crc != CRC8().add(h).get_crc()){
							error_occured(pkg_err_code__crc_error, h);
						}
						//TODO Not to use pkg.
						{
							if(h.daddr == pkg_addr__broadcast){
								write_serial(h);
								write_serial(crc);
							}
							
							pkg_with_paylod_init(
								p,
								addr_get_rsp,
								pkg_addr__master,
								h.id,
								{
									.addr = addr
								}
							);
							write_serial(p);
						}
					} break;
				case pkg_type(addr_get_rsp):
					read_serial(crc);
					error_occured(pkg_err_code__master_responding, h);
					break;
				case pkg_type(pos_set): {
						pkg_payload(pos_set) pl;
						read_serial(pl);
						read_serial(crc);
						if(crc != CRC8().add(h).add(pl).get_crc()){
							error_occured(pkg_err_code__crc_error, h);
						}else{
								//DoCommands(pl.pos);
								packetVal = pl.pos;
								packetCheck = true;
						}
					} break;
				case pkg_type(pos_get_req): {
						read_serial(crc);
						if(crc != CRC8().add(h).get_crc()){
							error_occured(pkg_err_code__crc_error, h);
						}
						//TODO Not to use pkg.
						{
							if(h.daddr == pkg_addr__broadcast){
								write_serial(h);
								write_serial(crc);
							}
							
							i16 pos2 = pos;
							// If inbetween reading pos,
							// pos is incremented by pos_pulse().
							if(pos2 != pos){
								// Update it again,
								// because pos_pulse() could changed it.
								// Do not need atomic,
								// because pos_pulse() 
								// will not occur for a long time.
								pos2 = pos;
							}
							pkg_with_paylod_init(
								p,
								pos_get_rsp,
								pkg_addr__master,
								h.id,
								{
									.pos = pos2
								}
							);
							write_serial(p);
						}
					} break;
				case pkg_type(pos_get_rsp):
					read_serial(crc);
					error_occured(pkg_err_code__master_responding, h);
					break;
				case pkg_type(calib_pos_set): {
						pkg_payload(calib_pos_set) pl;
						read_serial(pl);
						read_serial(crc);
						if(crc != CRC8().add(h).add(pl).get_crc()){
							error_occured(pkg_err_code__crc_error, h);
						}else{
							if(h.daddr == pkg_addr__broadcast){
								write_serial(h);
								write_serial(pl);
								write_serial(crc);
							}
							
							pos = pl.pos;
							// If inbetween writing pos,
							// pos is incremented by pos_pulse().
							if(pos != pl.pos){
								// Update it again, because pos_pulse() change it.
								// Do not need atomic,
								// because pos_pulse() will not occur for a long time.
								pos = pl.pos;
							}
						}
					} break;
				default:
					error_occured(pkg_err_code__nonexisting_pkg_type, h);
					break;
			}
		}
	}

	int v0 = map_adc(analogRead(A0), 179, 0);
	int v1 = map_adc(analogRead(A1), 179, 10);
	int v2 = map_adc(analogRead(A2), 10, 140);
	int v3 = map_adc(analogRead(A3), 160, 90);
	
	joyStickVal[0] = v0;
	joyStickVal[1] = v1;
	joyStickVal[2] = v2;
	joyStickVal[3] = v3;
	
	if((v0 != 0) || (v1 != 0) || (v2 != 0) || (v3 != 0)){
		packetCheck = false;
		DoCommands(joyStickVal);
	}
	else if(packetCheck){
		DoCommands(packetVal);
		packetCheck = false;
	}
	
	// Jog.
	if(!digitalRead(PB_CW)){
		jog_dir = JOG_CW;
	}else if(!digitalRead(PB_CCW)){
		jog_dir = JOG_CCW;
	}else{
		jog_dir = JOG_NONE;
	}
}
