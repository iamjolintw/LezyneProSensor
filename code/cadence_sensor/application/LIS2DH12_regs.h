/*
 *	LIS2DH12_regs.h
 *
 *  Created on: Aug 21, 2019
 *	Author: Lezyne
 */


#ifndef LIS2DH12_REGS_h
#define LIS2DH12_REGS_h

/***********************************************************************************************\
* Public macros
\***********************************************************************************************/
#define LIS2DH12_I2C_ADDR            0x19 //!<This is the factory pre-configured I2C address for an LIS2DE12

/***********************************************************************************************
**
**  LIS2DE12 Sensor Internal Registers
*/

//enum
//{
//    /*  CONTROL REGISTERS   */
//    LIS2DE12_WHO_AM_I        =   0x0F,    /*  WhoAmI register     */
//    LIS2DE12_TEMP_CFG_REG    =   0x1F,    /*  temper sens control reg */
//    /* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
//    LIS2DE12_CTRL_REG1       =   0x20,    /*  control reg 1       */
//    LIS2DE12_CTRL_REG2       =   0x21,    /*  control reg 2       */
//    LIS2DE12_CTRL_REG3       =   0x22,    /*  control reg 3       */
//    LIS2DE12_CTRL_REG4       =   0x23,    /*  control reg 4       */
//    LIS2DE12_CTRL_REG5       =   0x24,    /*  control reg 5       */
//    LIS2DE12_CTRL_REG6       =   0x25,    /*  control reg 6       */
//
//    LIS2DE12_FIFO_CTRL_REG   =   0x2E,    /*  FiFo control reg    */
//
//    LIS2DE12_INT_CFG1        =   0x30,    /*  interrupt 1 config  */
//    LIS2DE12_INT_SRC1        =   0x31,    /*  interrupt 1 source  */
//    LIS2DE12_INT_THS1        =   0x32,    /*  interrupt 1 threshold   */
//    LIS2DE12_INT_DUR1        =   0x33,    /*  interrupt 1 duration    */
//
//
//    LIS2DE12_TT_CFG          =   0x38,    /*  tap config      */
//    LIS2DE12_TT_SRC          =   0x39,    /*  tap source      */
//    LIS2DE12_TT_THS          =   0x3A,    /*  tap threshold       */
//    LIS2DE12_TT_LIM          =   0x3B,    /*  tap time limit      */
//    LIS2DE12_TT_TLAT         =   0x3C,    /*  tap time latency    */
//    LIS2DE12_TT_TW           =   0x3D,    /*  tap time window     */
//    /*  end CONTROL REGISTRES   */
//};
//
#define LIS2DE12_STATUS              0x27
#define LIS2DE12_STATUS_ZYXOR        0x80
#define LIS2DE12_STATUS_ZOR          0x40
#define LIS2DE12_STATUS_YOR          0x20
#define LIS2DE12_STATUS_XOR          0x10
#define LIS2DE12_STATUS_ZYXDA        0x08
#define LIS2DE12_STATUS_ZDA          0x04
#define LIS2DE12_STATUS_YDA          0x02
#define LIS2DE12_STATUS_XDA          0x01
//
#define LIS2DE12_STATUS_AUX          0x07
#define LIS2DE12_STATUS_TOR          0x40
#define LIS2DE12_STATUS_TDA          0x04
//
#define LIS2DE12_WHO_AM_I            0x0F
#define LIS2DE12_WHO_AM_I_OUT        0x33
//
#define LIS2DE12_REG0                0x1E
#define LIS2DE12_REG0_SDO_PU         0x20
//
#define LIS2DE12_TEMP_CFG_REG        0x1F
#define LIS2DE12_TEMP_CFG_REG_EN     0xC0 // Enable Temperature
#define LIS2DE12_TEMP_CFG_REG_DIS    0x00 // Disable Temperature
//
#define LIS2DE12_REG1                0x20
#define LIS2DE12_REG1_POWER_DOWN     0x00
#define LIS2DE12_REG1_1HZ            0x10
#define LIS2DE12_REG1_10HZ           0x20
#define LIS2DE12_REG1_25HZ           0x30
#define LIS2DE12_REG1_50HZ           0x40
#define LIS2DE12_REG1_100HZ          0x50
#define LIS2DE12_REG1_200HZ          0x60
#define LIS2DE12_REG1_400HZ          0x70
#define LIS2DE12_REG1_1620HZ         0x80
#define LIS2DE12_REG1_5376HZ         0x90
#define LIS2DE12_REG1_ACTIVE_MSK     0xF0
#define LIS2DE12_REG1_LOW_POWER      0x08
#define LIS2DE12_REG1_Z_AXIS         0x04
#define LIS2DE12_REG1_Y_AXIS         0x02
#define LIS2DE12_REG1_X_AXIS         0x01
#define LIS2DE12_REG1_ALL_AXES       0x07
#define LIS2DE12_REG1_YZ_AXES        0x06
//
#define LIS2DE12_REG2                0x21
#define LIS2DE12_REG2_HPF_NORMAL     0x00
#define LIS2DE12_REG2_HPF_REF_SIG    0x40
#define LIS2DE12_REG2_HPF_NOR_MOD  	 0x80
#define LIS2DE12_REG2_HPF_INT_RESET  0xC0
/*cutoff frequencies use bits 4 and 5, see datasheet */
#define LIS2DE12_REG2_HPF_USE        0x08
#define LIS2DE12_REG2_HPF_CLICK      0x04
#define LIS2DE12_REG2_HPF_AOI_INT2   0x02    // AOI - And Or Interrupt events
#define LIS2DE12_REG2_HPF_AOI_INT1   0x01
//
#define LIS2DE12_REG3                0x22
#define LIS2DE12_REG3_CLICK_INT1     0x80
#define LIS2DE12_REG3_AOI1_INT1      0x40
#define LIS2DE12_REG3_AOI2_INT1      0x20
#define LIS2DE12_REG3_DRDY1_INT1     0x10
#define LIS2DE12_REG3_DRDY2_INT1     0x08
#define LIS2DE12_REG3_FIFO_WTR_INT1  0x04
#define LIS2DE12_REG3_FIFO_OVR_INT1  0x02
//
#define LIS2DE12_REG4                0x23
#define LIS2DE12_REG4_BDU_CONT       0x00    // Updates are continuous
#define LIS2DE12_REG4_BDU_READ       0x80    // Updates after read
#define LIS2DE12_REG4_BDU_MSK        0x80
#define LIS2DE12_REG4_BLE_MSK        0x40
#define LIS2DE12_REG4_HR_MSK         0x08	 //Operating mode selection
#define LIS2DE12_REG4_SCL_2G         0x00
#define LIS2DE12_REG4_SCL_4G         0x10
#define LIS2DE12_REG4_SCL_8G         0x20
#define LIS2DE12_REG4_SCL_16G        0x30
#define LIS2DE12_REG4_SCL_MSK        0x30

//
#define LIS2DE12_REG5                0x24
#define LIS2DE12_REG5_REBOOT         0x80
#define LIS2DE12_REG5_USE_FIFO       0x40
#define LIS2DE12_REG5_LIR_INT1       0x08
#define LIS2DE12_REG5_D4D_INT1       0x04
#define LIS2DE12_REG5_LIR_INT2       0x02
#define LIS2DE12_REG5_D4D_INT2       0x01
//
#define LIS2DE12_REG6                0x25
#define LIS2DE12_REG6_CLICK_INT2     0x80
#define LIS2DE12_REG6_INT1_FUNC_INT2 0x40
#define LIS2DE12_REG6_INT2_FUNC_INT2 0x20
#define LIS2DE12_REG6_BOOT_INT2      0x10
#define LIS2DE12_REG6_ACTIVITY_INT2  0x08
#define LIS2DE12_REG6_INT_ACT_LOW    0x02
//
#define LIS2DE12_INT_REFERENCE       0x26
//
#define LIS2DE12_FIFO_CTRL_REG       0x2E
#define LIS2DE12_FIFO_FM_MSK         0xC0
#define LIS2DE12_FIFO_BYPASS         0x00
#define LIS2DE12_FIFO_FIFO           0x40
#define LIS2DE12_FIFO_STREAM         0x80
#define LIS2DE12_FIFO_TRIGGER        0xC0
#define LIS2DE12_FIFO_TR_SELECT      0x20
//
#define LIS2DE12_FIFO_SRC_REG        0x2F
#define LIS2DE12_FIFO_SRC_REG_WTM    0x80
#define LIS2DE12_FIFO_SRC_REG_OVRN   0x40
#define LIS2DE12_FIFO_SRC_REG_CNT_MSK 0x1F
#define LIS2DE12_FIFO_SRC_REG_TR_MSK 0x20

//
#define LIS2DE12_INT1_CFG_REG        0x30

#define LIS2DE12_INT1_CFG_ZHIE_MSK   0x20
#define LIS2DE12_INT1_CFG_YHIE_MSK   0x08
#define LIS2DE12_INT1_CFG_XHIE_MSK   0x02
//
#define LIS2DE12_INT1_SRC_REG        0x31
#define LIS2DE12_INT1_SRC_IA_MSK     0x40
//
#define LIS2DE12_INT1_THRESHHOLD_REG 0x32
#define LIS2DE12_INT1_DURATION_REG   0x33    // 1 LSb = 1/ODR
//
#define LIS2DE12_INT2_CFG_REG        0x34
#define LIS2DE12_INT2_SRC_REG        0x35
#define LIS2DE12_INT2_THRESHHOLD_REG 0x36
#define LIS2DE12_INT2_DURATION_REG   0x37    // 1 LSb = 1/ODR
//
#define LIS2DE12_SLEEP_THRESHOLD_REG 0x3E
#define LIS2DE12_SLEEP_DURATION_REG  0x3F    // 1LSb = (8*1[LSb]+1)/ODR

// Usable with INT1 and INT2 cfg registers
#define LIS2DE12_INT_OR_EVTS         0x00
#define LIS2DE12_INT_AND_EVTS        0x80
#define LIS2DE12_INT_MOVEMENT        0x40
#define LIS2DE12_INT_DIRECTION       0xC0
#define LIS2DE12_INT_Z_HI            0x20
#define LIS2DE12_INT_Z_LO            0x10
#define LIS2DE12_INT_Y_HI            0x08
#define LIS2DE12_INT_Y_LO            0x04
#define LIS2DE12_INT_X_HI            0x02
#define LIS2DE12_INT_X_LO            0x01

#define LIS2DE12_OUT_TEMP_L          0x0C

#define LIS2DE12_OUT_FIFO            0x28
#define LIS2DE12_OUT_X_L             0x28
#define LIS2DE12_OUT_X_H             0x29
#define LIS2DE12_OUT_Y_L             0x2A
#define LIS2DE12_OUT_Y_H             0x2B
#define LIS2DE12_OUT_Z_L             0x2C
#define LIS2DE12_OUT_Z_H             0x2D

#define LIS2DE12_CLICK_CFG_REG		 0x38
#define LIS2DE12_CLICK_SRC_REG		 0x39
#define LIS2DE12_CLICK_THS_REG		 0x3A

#define LIS2DE12_TIME_LIMIT_REG		 0x3B
#define LIS2DE12_TIME_LATENCY_REG	 0x3C
#define LIS2DE12_TIME_WINDOW_REG	 0x3D

#define LIS2DE12_ACT_THS_REG		 0x3E
#define LIS2DE12_ACT_DUR_REG		 0x3F

/* end RESUME STATE INDICES */

#endif

