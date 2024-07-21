#include "TMCStepper.h"
#include "TMC_MACROS.h"

#define SET_REG(SETTING) COOLCONF_register.SETTING = B; write(COOLCONF_register.address, COOLCONF_register.sr);
#define GET_REG(SETTING) return COOLCONF_register.SETTING;

// COOLCONF
uint32_t TMC2130Stepper::COOLCONF() { return COOLCONF_register.sr; }
void TMC2130Stepper::COOLCONF(uint32_t input) {
	COOLCONF_register.sr = input;
	write(COOLCONF_register.address, COOLCONF_register.sr);
}

void TMC2130Stepper::semin(	uint8_t B )	{ SET_REG(semin);	}
void TMC2130Stepper::seup(	uint8_t B )	{ SET_REG(seup);	}
void TMC2130Stepper::semax(	uint8_t B )	{ SET_REG(semax);	}
void TMC2130Stepper::sedn(	uint8_t B )	{ SET_REG(sedn);	}
void TMC2130Stepper::seimin(bool 	B )	{ SET_REG(seimin);	}
void TMC2130Stepper::sgt(	int8_t  B )	{ SET_REG(sgt);		}
void TMC2130Stepper::sfilt(	bool 	B )	{ SET_REG(sfilt);	}

uint8_t TMC2130Stepper::semin()	{ GET_REG(semin);	}
uint8_t TMC2130Stepper::seup()	{ GET_REG(seup);	}
uint8_t TMC2130Stepper::semax()	{ GET_REG(semax);	}
uint8_t TMC2130Stepper::sedn()	{ GET_REG(sedn);	}
bool 	TMC2130Stepper::seimin(){ GET_REG(seimin);	}
bool 	TMC2130Stepper::sfilt()	{ GET_REG(sfilt);	}

int8_t TMC2130Stepper::sgt() {
	// Two's complement in a 7bit value
	int8_t val = (COOLCONF_register.sgt &  0x40) << 1; // Isolate sign bit
	val |= COOLCONF_register.sgt & 0x7F;
	return val;
}

uint16_t TMC2209Stepper::COOLCONF() { return COOLCONF_register.sr; }
void TMC2209Stepper::COOLCONF(uint16_t input) {
	COOLCONF_register.sr = input;
	write(COOLCONF_register.address, COOLCONF_register.sr);
}

void TMC2209Stepper::semin(	uint8_t B )	{ SET_REG(semin);	}
void TMC2209Stepper::seup(	uint8_t B )	{ SET_REG(seup);	}
void TMC2209Stepper::semax(	uint8_t B )	{ SET_REG(semax);	}
void TMC2209Stepper::sedn(	uint8_t B )	{ SET_REG(sedn);	}
void TMC2209Stepper::seimin(bool 	B )	{ SET_REG(seimin);	}

uint8_t TMC2209Stepper::semin()	{ GET_REG(semin);	}
uint8_t TMC2209Stepper::seup()	{ GET_REG(seup);	}
uint8_t TMC2209Stepper::semax()	{ GET_REG(semax);	}
uint8_t TMC2209Stepper::sedn()	{ GET_REG(sedn);	}
bool 	TMC2209Stepper::seimin(){ GET_REG(seimin);	}

#define GET_REG_2240(SETTING) TMC2240_n::COOLCONF_t r{0}; r.sr = COOLCONF(); return r.SETTING
uint32_t TMC2240Stepper::COOLCONF() { return COOLCONF_register.sr; }
void TMC2240Stepper::COOLCONF(uint32_t data) {
  COOLCONF_register.sr = data;
  write(COOLCONF_register.address, COOLCONF_register.sr);
}
void TMC2240Stepper::sgt(uint8_t B) { SET_REG(sgt); }
uint8_t TMC2240Stepper::sgt(){ GET_REG_2240(sgt); }

void TMC2240Stepper::semin(uint8_t B) { SET_REG(semin); }
void TMC2240Stepper::seup(uint8_t B) { SET_REG(seup); }
void TMC2240Stepper::semax(uint8_t B) { SET_REG(semax); }
void TMC2240Stepper::sedn(uint8_t B) { SET_REG(sedn); }
void TMC2240Stepper::seimin(bool B) { SET_REG(seimin); }
void TMC2240Stepper::sfilt(bool B) { SET_REG(sfilt); }

uint8_t TMC2240Stepper::semin(){GET_REG_2240(semin);}
uint8_t TMC2240Stepper::seup(){GET_REG_2240(seup);}
uint8_t TMC2240Stepper::semax(){GET_REG_2240(semax);}
uint8_t TMC2240Stepper::sedn(){GET_REG_2240(sedn);}
bool TMC2240Stepper::seimin(){GET_REG_2240(seimin);}
bool TMC2240Stepper::sfilt(){GET_REG_2240(sfilt);}