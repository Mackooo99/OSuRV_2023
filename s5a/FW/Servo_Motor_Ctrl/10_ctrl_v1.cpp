
///////////////////////////////////////////////////////////////////////////////

#define EN_JOYSTICKS 0
//#define SPEED 50
#define SPEED 0 // Max speed

///////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

#include "avr_io_bitfields.h"

#include "motor_ctrl_pkgs.hpp"

#include "VarSpeedServo.h"


#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

//////////////////


#define LED_NO_ERR LED_BUILTIN

//////////////////

VarSpeedServo servo[N_JOINTS];

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

/*
//TODO WDT
// 2x longer than LOOP_HZ.
#define WATCHDOG_TOP (2*10000/(LOOP_HZ)-1)
static volatile u16 watchdog = WATCHDOG_TOP;
static void rst_watchdog() {
	watchdog = WATCHDOG_TOP;
}
*/


///////////////////////////////////////////////////////////////////////////////

/*
//TODO WDT
// 10kHz
ISR(TIMER2_COMPA_vect) {
	if(watchdog == 0){
		//TODO Stop it.
		
		error_occured(pkg_err_code__watchdog_stop);
	}else{
		watchdog--;
	}
}
*/

// Here it is hardcoded for now.
static u8 addr = pkg_addr__motor_id_0;


void setup() {
	static const i8 servo_pins[] = {
		// pin	s3a
		11, // Mount pin
		10, // Shoulder pin
		9,  // Elbow pin
		5,  // Claw pin
		7,
		6,
	};

	for(i8 i = 0; i < N_JOINTS; i++){
		servo[i].attach(servo_pins[i]);
	}
	

	// WARNING: Do not print anything if not asked for by protocol.
	Serial.begin(BAUDRATE);
	
	// Error is indicated by this LED off.
	pinMode(LED_NO_ERR, OUTPUT);
	digitalWrite(LED_NO_ERR, 1);

	//TODO WDT
/*
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


struct pos_cmd_msg_t {
	bool active;
	pkg_payload(pos_set) pl;
};

int map_adc(
	int adc_value,
	int low_angle,
	int high_angle
) {
	int angle;
	if(adc_value < 512){
		angle = map(adc_value, 0, 511, low_angle, 0);
	}else{
		angle = map(adc_value, 512, 1023, 0, high_angle);
	}
	return angle;
}

void loop() {
	static bool online = false;
	pos_cmd_msg_t from_pkg = { .active = false };
	static pos_cmd_msg_t final = { .active = false };
	
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
							from_pkg.pl = pl;
							from_pkg.active = true;
							online = true;
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
							
							pkg_payload(pos_get_rsp) pl;
							for(i8 i = 0; i < N_JOINTS; i++){
								//TODO Get it from servos.
								pl.pos[i] = final.pl.pos[i];
							}
							pkg_with_paylod_init(
								p,
								pos_get_rsp,
								pkg_addr__master,
								h.id,
								pl
							);
							write_serial(p);
						}
					} break;
				case pkg_type(pos_get_rsp):
					read_serial(crc);
					error_occured(pkg_err_code__master_responding, h);
					break;
				//TODO Remove this type.
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
							
						}
					} break;
				default:
					error_occured(pkg_err_code__nonexisting_pkg_type, h);
					break;
			}
		}
	}

	pos_cmd_msg_t from_js = { 
		.active = false,
		.pl = {
			.pos = {0}
		}
	};
	//TODO Why 89
#if EN_JOYSTICKS
	from_js.pl.pos[0] = map_adc(analogRead(A0), 89, -90);
	from_js.pl.pos[1] = map_adc(analogRead(A1), 89, -45);
	from_js.pl.pos[2] = map_adc(analogRead(A2), -80, 50);
	from_js.pl.pos[3] = map_adc(analogRead(A3), 70, 0);
	for(i8 i = 0; i < 4; i++){
		if(abs(from_js.pl.pos[i]) > 5){
			from_js.active = true;
			break;
		}
	}
#endif
	
	
	// Joystick have priority.
	if(online){
		if(from_js.active){
			final = from_js;
		}else if(from_pkg.active){
			final = from_pkg;
		}
	}else{
		// No pkgs, offline, return to 0.
		final = from_js;
		final.active = true;
		
	}
	if(final.active){
		for(i8 i = 0; i < N_JOINTS; i++){
			servo[i].write(final.pl.pos[i] + 90, SPEED);
		}
	}

}
