#ifndef COMMANDS_H
#define COMMANDS_H
// Command Groups, Modules and Functions

// BASE: Basic commandos like version, echo etc.
#define _CMDGRP_BASE_   			0x10
#define _CMDGRP_BASE_OUT_			0x11

// SYSTEM: System related calls like Battary Voltage etc.
#define _CMDGRP_SYSTEM_  			0x20
#define _CMDGRP_SYSTEM_OUT_			0x21

// ---------------------------------
// Motor Control Groups
#define _CMDGRP_MOTOR_CONFIN_		0x50
#define _CMDGRP_MOTOR_CONFOUT_		0x51
#define _CMDGRP_MOTOR_CTRL_			0x52
//		Round Robin list (NO 0x53)
#define _CMDGRP_MOTOR_STATUSIN_		0x54
#define _CMDGRP_MOTOR_STATUSOUT_	0x55
#define _CMDGRP_CONTROLLER_PARIN_	0x56
#define _CMDGRP_CONTROLLER_PAROUT_	0x57
#define _CMDGRP_MOTOR_ERR_			0x59
#define _CMDGRP_MOTOR_MASK_			0x50
// =====================================================================================

// ---------------------------------
// Controller configuration
// (group _CMDGRP_CONTROLLER_PARIN_ and _CMDGRP_CONTROLLER_PAROUT_)
#define _CTRL_PID_KP_			0x10
#define _CTRL_PID_TN_			0x11
#define _CTRL_PID_TV_			0x12
#define _CTRL_PRAMP_			0x20
#define _CTRL_NRAMP_			0x21
#define _CTRL_DEADBAND_			0x2A

// ---------------------------------
// Motor Control Commands
#define _SET_ALL_PWM_			0x10
#define _SET_ONE_PWM_	    	0x11
#define _SET_ALL_RPM_			0x20
#define _SET_ONE_RPM_			0x21
#define _SET_DIGITAL_OUT_		0x41
#define _SET_CLEAR_ALL_TICKS_ABS_ 0x65 
#define _SET_MODE_WITH_BUMPER_	0x70
// ---------------------------------

// ---------------------------------
// Motor Config Commands
#define _MCMD_CONTROLLER_		0x10
#define _MCMD_USE_PWM_	    	0x11
#define _MCMD_LIMITER_ 			0x12
#define _MCMD_TIMEOUT_			0x13
#define _MCMD_MAX_CUR_			0x20
#define _MCMD_MAX_RPM_			0x21
#define _MCMD_NOM_CUR_			0x22
#define _MCMD_NOM_RPM_			0x23
#define _MCMD_GEAR_RATIO_		0x30
#define _MCMD_WHEEL_RADIUS_		0x31
#define _MCMD_AXE_LENGTH_		0x32
#define _MCMD_TICKS_ROT_		0x33
#define _MCMD_SPEC_TORQUE_		0x34
#define _MCMD_DIRECTION_		0x35
#define _MCMD_MAX_TEMP_			0x40
#define _MCMD_NOM_TEMP_			0x41
#define _MCMD_ENV_TEMP_			0x42
#define _MCMD_WINDING_T_		0x43
#define _MCMD_WINDING_GN_		0x44
#define _MCMD_WINDING_GZ_		0x45
#define _MCMD_CHASSIS_T_		0x46
#define _MCMD_CHASSIS_GN_		0x47
#define _MCMD_CHASSIS_GZ_		0x48
#define _MCMD_SAVE_CONFIG_ 		0x50
#define _MCMD_LOAD_CONFIG_ 		0x51
#define _MCMD_IO_PORT_CONFIG_ 	0x60	//new
//#define _MCMD_DIGITAL_OUT_	 	0x61
//#define _MCMD_CLEAR_ALL_TICKS_ABS_ 0x65   // new
#define _MCMD_CLEAR_ONE_TICKS_ABS_ 0x66   // new
// ---------------------------------

// ---------------------------------
// Motor Status Commands

#define _SREG_MOT_RPM_		 	0x10
#define _SREG_MOT_PWM_		 	0x11
#define _SREG_MOT_VOLTAGE_	 	0x12
#define _SREG_MOT_CURRENT_ 	 	0x13
#define _SREG_MOT_TEMP_		 	0x14
#define _SREG_MOT_TORQUE_	 	0x15
#define _SREG_MOT_POWER_	 	0x16
#define _SREG_MOT_TICKS_ABS_ 	0x21
#define _SREG_MOT_TICKS_REL_ 	0x20
#define _SREG_BATTERY_ 		 	0x30
#define _SREG_TIME_ 		 	0x35
#define _SREG_DIGITAL_IN_ 		0x40	//new
//#define _SREG_DIGITAL_OUT_ 		0x41	//new
#define _SREG_ANALOG1_IN_ 		0x42	//new
#define _SREG_ANALOG2_IN_ 		0x43	//new
#define _SREG_USER_REQUEST_  	0xF0

// ---------------------------------

// ---------------------------------

// ---------------------------------
// Motor Error Commands
#define _UNKNOWN_ERR_			0x01
#define _PARAM_ERR_	    		0x10
#define _RANGE_ERR_				0x11
#define _BUMPER_ERR_			0x70

// ---------------------------------
// ---------------------------------
// Parameter Ranges
#define _MAX_CUR_				8000
#define _MAX_RPM_				25000
#define _NOM_CUR_				8000
#define _NOM_RPM_				25000
#define _GEAR_RATIO_			10000
#define _WHEEL_RADIUS_			1000
#define _AXE_LENGTH_			1000
#define _TICKS_ROT_				5000
#define _SPEC_TORQUE_			1000
#define _MAX_TEMP_				200
#define _NOM_TEMP_				200
#define _ENV_TEMP_				100
#define _WINDING_T_				500
#define _WINDING_GN_			100
#define _WINDING_GZ_			1000
#define _CHASSIS_T_				500
#define _CHASSIS_GN_			100
#define _CHASSIS_GZ_			1000
#define _CONFIG_				4
#define _TIMEOUT_				1000

// ---------------------------------
#endif /* COMMANDS_H */
