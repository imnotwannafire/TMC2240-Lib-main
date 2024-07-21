#include "TMCStepper.h"
#include "TMC_MACROS.h"
#include "SW_SPI.h"


TMC2240Stepper::TMC2240Stepper(uint16_t pinCS, float RS) :
  _pinCS(pinCS),
  Rsense(RS)
  {}

TMC2240Stepper::TMC2240Stepper(uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
  _pinCS(pinCS),
  Rsense(default_RS)
  {
    SW_SPIClass *SW_SPI_Obj = new SW_SPIClass(pinMOSI, pinMISO, pinSCK);
    TMC_SW_SPI = SW_SPI_Obj;
  }

TMC2240Stepper::TMC2240Stepper(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
  _pinCS(pinCS),
  Rsense(RS)
  {
    SW_SPIClass *SW_SPI_Obj = new SW_SPIClass(pinMOSI, pinMISO, pinSCK);
    TMC_SW_SPI = SW_SPI_Obj;
  }

void TMC2240Stepper::switchCSpin(bool state) {
  // Allows for overriding in child class to make use of fast io
  digitalWrite(_pinCS, state);
}




void TMC2240Stepper::begin() {
  //set pins
  TMC_SW_SPI->init();
  pinMode(_pinCS, OUTPUT);
  switchCSpin(HIGH);
}



void TMC2240Stepper::push() {
	GCONF(GCONF_register.sr);
    DRV_CONF(DRV_CONF_register.sr);
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
	TPWMTHRS(TPWMTHRS_register.sr);
  	SG4_THRS(SG4_THRS_register.sr);
	COOLCONF(COOLCONF_register.sr);
  	TCOOLTHRS(TCOOLTHRS_register.sr);
	GSTAT(GSTAT_register.sr);
  	PWMCONF(PWMCONF_register.sr);
}


bool TMC2240Stepper::isEnabled() { return !drv_enn() && toff(); }



void TMC2240Stepper::write(uint8_t addressByte, uint32_t config) {
  // uint32_t data = (uint32_t)addressByte<<17 | config;
  if (TMC_SW_SPI != nullptr) { // use sw spi
    switchCSpin(LOW);
    TMC_SW_SPI->transfer(addressByte|0x80);
    TMC_SW_SPI->transfer((config >> 24) & 0xFF);
    TMC_SW_SPI->transfer((config >> 16) & 0xFF);
    TMC_SW_SPI->transfer((config >>  8) & 0xFF);
    TMC_SW_SPI->transfer(config & 0xFF);
  } else { // use hw spi
    SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
    switchCSpin(LOW);
    TMC_SW_SPI->transfer(addressByte|0x80);
    TMC_SW_SPI->transfer((config >> 24) & 0xFF);
    TMC_SW_SPI->transfer((config >> 16) & 0xFF);
    TMC_SW_SPI->transfer((config >>  8) & 0xFF);
    TMC_SW_SPI->transfer(config & 0xFF);
    SPI.endTransaction();
  }
  switchCSpin(HIGH);
}




uint32_t TMC2240Stepper::read(uint8_t address) {
	char buf[4];
	uint32_t response = 0UL;
	
	memset(buf,0,sizeof(buf));
	
	if (TMC_SW_SPI != nullptr) { //use SW SPI
		// step 1: Send the register address 
		switchCSpin(LOW);
		buf[0] = TMC_SW_SPI->transfer(address & 0x7F); 	//set MSB of first byte to 0 for read operation
		buf[1] = TMC_SW_SPI->transfer(0x00);	//dummy data
		buf[2] = TMC_SW_SPI->transfer(0x00);	//dummy data
		buf[3] = TMC_SW_SPI->transfer(0x00);	//dummy data
		switchCSpin(HIGH);
		// step 2: Get response value of register
		// Short delay may be required
        delayMicroseconds(1);
		switchCSpin(LOW);
        buf[0] = TMC_SW_SPI->transfer(0x00); // dummy address for reading data
        buf[1] = TMC_SW_SPI->transfer(0x00); // dummy data
        buf[2] = TMC_SW_SPI->transfer(0x00); // dummy data
        buf[3] = TMC_SW_SPI->transfer(0x00); // dummy data
        switchCSpin(HIGH);



	} else { // use HW SPI
		SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
		switchCSpin(LOW);
		buf[0] = TMC_SW_SPI->transfer(address&0x7F);
		buf[1] = TMC_SW_SPI->transfer(0xFF);
		buf[2] = TMC_SW_SPI->transfer(0xFF);
		buf[3] = TMC_SW_SPI->transfer(0xFF);
		switchCSpin(HIGH);
		SPI.endTransaction();

		delayMicroseconds(1);

		SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        switchCSpin(LOW);
        buf[0] = SPI.transfer(0x00); // dummy address for reading data
        buf[1] = SPI.transfer(0x00); // dummy data
        buf[2] = SPI.transfer(0x00); // dummy data
        buf[3] = SPI.transfer(0x00); // dummy data
        switchCSpin(HIGH);
        SPI.endTransaction();

		
	}
	
	return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void TMC2240Stepper::send(uint32_t data)
{
  switchCSpin(LOW);
  TMC_SW_SPI->transfer(data);
  switchCSpin(HIGH);
}

uint8_t TMC2240Stepper::IFCNT() {
	return read(IFCNT_t::address);
}


void TMC2240Stepper::SLAVECONF(uint16_t input) {
	SLAVECONF_register.sr = input&0xF00;
	write(SLAVECONF_register.address, SLAVECONF_register.sr);
}

uint16_t TMC2240Stepper::SLAVECONF() {
	return SLAVECONF_register.sr;
}

void TMC2240Stepper::senddelay(uint8_t B) 	{ SLAVECONF_register.SENDDELAY = B; write(SLAVECONF_register.address, SLAVECONF_register.sr); }
uint8_t TMC2240Stepper::senddelay() 		{ return SLAVECONF_register.SENDDELAY; }



uint32_t TMC2240Stepper::IOIN() {
	return read(TMC2240_n::IOIN_t::address);
}


bool TMC2240Stepper::step()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.step;		}
bool TMC2240Stepper::dir()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.dir;		}
bool TMC2240Stepper::encb()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.encb;		}
bool TMC2240Stepper::enca()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.enca;		}
bool TMC2240Stepper::drv_enn()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn;	}
bool TMC2240Stepper::encn()			    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.encn;		}
bool TMC2240Stepper::uart_en()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.uart_en;	}
bool TMC2240Stepper::comp_a()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_a;	}
bool TMC2240Stepper::comp_b()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_b;	}
bool TMC2240Stepper::comp_a1_a2()	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_a1_a2;	}
bool TMC2240Stepper::comp_b1_b2()	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.comp_b1_b2;	}
bool TMC2240Stepper::output()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.output;	}
bool TMC2240Stepper::ext_res_det()	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.ext_res_det;	}
bool TMC2240Stepper::ext_clk()	        { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.ext_clk;	}
bool TMC2240Stepper::adc_err()		    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.adc_err;	}
uint8_t TMC2240Stepper::silicon_rv() 	{ TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.silicon_rv;	}
uint8_t TMC2240Stepper::version() 	    { TMC2240_n::IOIN_t r{0}; r.sr = IOIN(); return r.version;	}


uint32_t TMC2240Stepper::PWM_SCALE() {
	return read(TMC2240_n::PWM_SCALE_t::address);
}
uint8_t TMC2240Stepper::pwm_scale_sum() {
	TMC2240_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_sum;
}



int16_t TMC2240Stepper::pwm_scale_auto() {
	TMC2240_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_auto;
	// Not two's complement? 9nth bit determines sign
	/*
	uint32_t d = PWM_SCALE();
	int16_t response = (d>>PWM_SCALE_AUTO_bp)&0xFF;
	if (((d&PWM_SCALE_AUTO_bm) >> 24) & 0x1) return -response;
	else return response;
	*/
}



// R: PWM_AUTO
uint32_t TMC2240Stepper::PWM_AUTO() {
	return read(PWM_AUTO_t::address);
}
uint8_t TMC2240Stepper::pwm_ofs_auto()  { PWM_AUTO_t r{0}; r.sr = PWM_AUTO(); return r.pwm_ofs_auto; }
uint8_t TMC2240Stepper::pwm_grad_auto() { PWM_AUTO_t r{0}; r.sr = PWM_AUTO(); return r.pwm_grad_auto; }


//******zhou******//

  /**
   * 0:1A  1:2A  2:3A  3:3A
   */
  #define TMC2240_CURRENT_RANGE   3    

  /**
   * ('rref', 12000, minval=12000, maxval=60000)
  */     
  #define TMC2240_Rref            12000


uint16_t TMC2240Stepper::cs2rms(uint8_t CS) {
  return 2;
}


float TMC2240Stepper::calc_IFS_current_RMS(int8_t range, uint32_t Rref)
{
	uint32_t Kifs_values[]={11750,24000,36000,36000};
	float IFS_current_RMS=0;

	IFS_current_RMS=(float)(((float)(Kifs_values[range]) /Rref) /sqrt(2));

	return IFS_current_RMS;
}

uint32_t TMC2240Stepper::set_globalscaler(float current, float IFS_current_RMS)
{
	uint32_t globalscaler=0;

	globalscaler=(int)(((current * 256) / IFS_current_RMS) + 0.5);

	if(globalscaler<32)globalscaler=32;
	if(globalscaler>=256)globalscaler=0;

	return globalscaler;
}


void TMC2240Stepper::rms_current(uint16_t mA)
{
	uint32_t globalscaler,IFS_current_RMS,CS=0;

	IFS_current_RMS	=calc_IFS_current_RMS(TMC2240_CURRENT_RANGE,TMC2240_Rref);
	globalscaler	=set_globalscaler(mA/1000,IFS_current_RMS);

	CS=(int)((((mA/1000) * 256 * 32) / (globalscaler * IFS_current_RMS))-1+0.5);

	if(CS>=31)	CS=31;
	if(CS<=0)	CS=0;

 	irun(CS);
  	ihold(CS*holdMultiplier);
}

void TMC2240Stepper::rms_current(uint16_t mA, float mult) {
  holdMultiplier = mult;
  rms_current(mA);
}


uint16_t TMC2240Stepper::rms_current() {
  uint8_t cur_run = irun();
  return (cur_run * 3000 / 32);
}

void TMC2240Stepper::microsteps(uint16_t ms) {
  switch(ms) {
    case 256: mres(0); break;
    case 128: mres(1); break;
    case  64: mres(2); break;
    case  32: mres(3); break;
    case  16: mres(4); break;
    case   8: mres(5); break;
    case   4: mres(6); break;
    case   2: mres(7); break;
    case   0: mres(8); break;
    default: break;
  }
}

uint16_t TMC2240Stepper::microsteps() {
  switch(mres()) {
    case 0: return 256;
    case 1: return 128;
    case 2: return  64;
    case 3: return  32;
    case 4: return  16;
    case 5: return   8;
    case 6: return   4;
    case 7: return   2;
    case 8: return   0;
  }
  return 0;
}




// R+C: GSTAT

#define SET_REG_2240(SETTING) GSTAT_register.SETTING = B; write(GSTAT_register.address, GSTAT_register.sr)

uint8_t TMC2240Stepper::GSTAT()  			{ return read(TMC2240_n::GSTAT_t::address); }
void TMC2240Stepper::GSTAT(uint32_t input) {
  GSTAT_register.sr = input;
  write(GSTAT_register.address, GSTAT_register.sr);
}


bool  TMC2240Stepper::reset()   			{ TMC2240_n::GSTAT_t r; r.sr = GSTAT(); return r.reset; }
bool  TMC2240Stepper::drv_err()  			{ TMC2240_n::GSTAT_t r; r.sr = GSTAT(); return r.drv_err; }
bool  TMC2240Stepper::uv_cp()    			{ TMC2240_n::GSTAT_t r; r.sr = GSTAT(); return r.uv_cp; }
bool  TMC2240Stepper::register_reset()  	{ TMC2240_n::GSTAT_t r; r.sr = GSTAT(); return r.register_reset; }
bool  TMC2240Stepper::vm_uvlo()    			{ TMC2240_n::GSTAT_t r; r.sr = GSTAT(); return r.vm_uvlo; }

void TMC2240Stepper::reset(bool B){
  SET_REG_2240(reset);
}
void TMC2240Stepper::drv_err(bool B){
  SET_REG_2240(drv_err);
}
void TMC2240Stepper::uv_cp(bool B){
  SET_REG_2240(uv_cp);
}
void TMC2240Stepper::register_reset(bool B){
  SET_REG_2240(register_reset);
}
void TMC2240Stepper::vm_uvlo(bool B){
  SET_REG_2240(vm_uvlo);
}

// W: TPOWERDOWN
uint8_t TMC2240Stepper::TPOWERDOWN() { return TPOWERDOWN_register.sr; }
void TMC2240Stepper::TPOWERDOWN(uint8_t input) {
  TPOWERDOWN_register.sr = input;
  write(TPOWERDOWN_register.address, TPOWERDOWN_register.sr);
}

uint8_t TMC2240Stepper::test_connection() {
  uint32_t drv_status = DRV_STATUS();
  switch (drv_status) {
      case 0xFFFFFFFF: return 1;
      case 0: return 2;
      default: return 0;
  }
}

void TMC2240Stepper::hysteresis_end(int8_t value) { hend(value+3); }
int8_t TMC2240Stepper::hysteresis_end() { return hend()-3; };

void TMC2240Stepper::hysteresis_start(uint8_t value) { hstrt(value-1); }
uint8_t TMC2240Stepper::hysteresis_start() { return hstrt()+1; }

void TMC2240Stepper::blank_time(uint8_t value) {
  switch (value) {
    case 16: TBL(0b00); break;
    case 24: TBL(0b01); break;
    case 36: TBL(0b10); break;
    case 54: TBL(0b11); break;
  }
}

uint8_t TMC2240Stepper::blank_time() {
  switch (TBL()) {
    case 0b00: return 16;
    case 0b01: return 24;
    case 0b10: return 36;
    case 0b11: return 54;
  }
  return 0;
}
//******zhou******//
