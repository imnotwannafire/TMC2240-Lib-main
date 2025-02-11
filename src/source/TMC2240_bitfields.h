#pragma once
#pragma pack(push, 1)


namespace TMC2240_n {
  struct GCONF_t {
    constexpr static uint8_t address = 0x00;
    union {
      uint32_t sr;
      struct {
        bool : 1;
        bool fast_standstill: 1;
        bool en_pwm_mode : 1;
        bool multistep_filt: 1;
        bool shaft: 1;
        bool diag0_error : 1;
        bool diag0_otpw: 1;
        bool diag0_stall: 1;
        bool diag1_stall : 1;
        bool diag1_index: 1;
        bool diag1_onstate: 1;
        bool : 1;
        bool diag0_pushpull: 1;
        bool diag1_pushpull : 1;
        bool small_hysteresis: 1;
        bool stop_enable: 1;
        bool direct_mode: 1;
      };
    };
  };
}

namespace TMC2240_n {
  struct GSTAT_t {
      constexpr static uint8_t address = 0x01;
      union {
        uint8_t sr : 8;
        struct {
          bool    reset       : 1,
                  drv_err     : 1,
                  uv_cp       : 1,
                  register_reset : 1,
                  vm_uvlo     : 1;
       };
    };
  };
}      


namespace TMC2240_n {
  struct SLAVECONF_t {
    constexpr static uint8_t address = 0x03;
    union {
      uint16_t sr : 12;
      struct {
        uint16_t  SLAVEADDR : 8;
        uint8_t   SENDDELAY : 4 ;
        };
    };
  };
}

namespace TMC2240_n {
  struct IOIN_t {
    constexpr static uint8_t address = 0x04;
    union {
      uint32_t sr;
      struct {
        bool  step : 1,
              dir : 1,
              encb : 1,
              enca : 1,
              drv_enn : 1,
              encn : 1,
              uart_en : 1,
                : 1,
              comp_a : 1,
              comp_b : 1,
              comp_a1_a2 : 1,
              comp_b1_b2 : 1,
              output : 1,
              ext_res_det : 1,
              ext_clk : 1,
              adc_err : 1;
        uint8_t  silicon_rv : 3,
                : 5,
              version : 8;
      };
    };
  };
}




namespace TMC2240_n {
  struct DRV_CONF_t {
    constexpr static uint8_t address = 0x0A;
    union {
      uint32_t sr;
      struct {
        uint8_t current_range : 2,
                              : 2,
                slope_control : 2;
        uint16_t              : 16;
      };
    };
  };
};


namespace TMC2240_n {
  struct TPOWERDOWN_t {
    constexpr static uint8_t address = 0x11;
    union {
      uint32_t sr;
      struct {
          uint8_t TPOWERDOWN : 8;
      };
    };
  };
};

namespace TMC2240_n {
  struct TPWMTHRS_t {
    constexpr static uint8_t address = 0x13;
    union {
      uint32_t sr;
      struct {
        // uint8_t tpwmthrsb : 20;
        uint8_t tpwmthrsb;
      };
    };
  };
};

namespace TMC2240_n {
  struct TCOOLTHRS_t {
    constexpr static uint8_t address = 0x14;
    union {
      uint32_t sr;
      struct {
        // uint8_t tcoolthrs : 20;
        uint8_t tcoolthrs;
      };
    };
  };
};

namespace TMC2240_n {
  struct DRV_STATUS_t {
    constexpr static uint8_t address = 0x6F;
    union {
      uint32_t sr;
      struct {
        uint16_t SG_RESULT : 10;
        uint8_t            : 2; 
        uint8_t      s2vsa : 1,
                     s2vsb : 1,
                   stealth : 1,
                  fsactive : 1;
        uint16_t  CS_ACTUAL : 5;
        uint8_t            : 3;
        bool    stallguard : 1,
                        ot : 1,
                      otpw : 1,
                      s2ga : 1,
                      s2gb : 1,
                       ola : 1,
                       olb : 1,
                      stst : 1;
      };
    };
  };
};



namespace TMC2240_n {
  struct IHOLD_IRUN_t {
    constexpr static uint8_t address = 0x10;
    union {
      uint32_t sr;
      struct {
        uint8_t  ihold : 5,
                       : 3,
                 irun  : 5,
                       : 3,
            iholddelay : 4,
                       : 4,
            irundelay  : 4;
      };
    };
  };
};



namespace TMC2240_n {
  struct CHOPCONF_t {
    constexpr static uint8_t address = 0x6C;
      union {
        uint32_t sr;
        struct {
          uint8_t toff : 4;
          uint8_t hstrt : 3;
          uint8_t hend : 4;
          bool fd3 : 1;
          bool disfdcc : 1;
          bool : 1;
          bool chm : 1;
          uint8_t tbl : 2;
          bool : 1;
          bool vhighfs : 1;
          bool vhighchm : 1;
          uint8_t tpfd : 4;
          uint8_t mres : 4;
          bool intpol : 1;
          bool dedge : 1;
          bool diss2g : 1;
          bool diss2vs : 1;
        };
    };
  };
};



namespace TMC2240_n {
  struct COOLCONF_t {
    constexpr static uint8_t address = 0x6D;
    union {
      uint32_t sr;
      struct {
        uint8_t semin: 4;
        bool : 1;
        uint8_t seup: 2;
        bool : 1;
        uint8_t semax: 4;
        bool : 1;
        uint8_t sedn: 2;
        bool seimin: 1;
        uint8_t sgt: 7;
        bool : 1;
        bool sfilt: 1;
        uint8_t : 7;
      };
    };
  };
};



namespace TMC2240_n {
  struct PWMCONF_t {
    constexpr static uint8_t address = 0x70;
    union {
      uint32_t sr;
      struct {
        uint8_t pwm_ofs : 8,
                pwm_grad : 8,
                pwm_freq : 2;
        bool    pwm_autoscale : 1,
                pwm_autograd : 1;
        uint8_t freewheel : 2;
        bool    pwm_meas_sd_enable : 1,
                pwm_dis_reg_stst   : 1;
        uint8_t pwm_reg : 4,
                pwm_lim : 4;
      };
    };
  };



  struct PWM_SCALE_t {
    constexpr static uint8_t address = 0x71;
    union {
      uint32_t sr;
      struct {
        uint16_t pwm_scale_sum : 10;
        uint8_t        : 6;
        int16_t  pwm_scale_auto : 9;
      };
    };
  };


  struct PWM_AUTO_t {
    constexpr static uint8_t address = 0x72;
    union {
      uint32_t sr : 24;
      struct {
        uint8_t pwm_ofs_auto : 8,
                            : 8,
                pwm_grad_auto : 8;
      };
    };
  };

  struct SG4_THRS_t {
    constexpr static uint8_t address = 0x74;
    union {
      uint32_t sr;
      struct {
        uint8_t sg4_thrsb: 8;
        bool sg4_filt_en: 1;
        bool sg_angle_offset: 1;
      };
    };
  };
}

#pragma pack(pop)
