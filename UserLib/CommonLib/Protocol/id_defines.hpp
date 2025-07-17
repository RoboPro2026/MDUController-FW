/*
 * id_defines.hpp
 *
 *  Created on: Jul 16, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_PROTOCOL_ID_DEFINES_HPP_
#define COMMONLIB_PROTOCOL_ID_DEFINES_HPP_

#include <cstdint>
namespace CommonLib::Protocol{
enum class DataType{
	COMMON_ID,
	PCU_ID,
	MDC_ID,
	GPIO_ID,
	COMMON_ID_ENFORCE = 0xF
};

namespace PCURegister{
	inline constexpr uint16_t NOP = 0x0000;
	inline constexpr uint16_t PCU_STATE = 0x0001;
	inline constexpr uint16_t CELL_N = 0x0002;
	inline constexpr uint16_t EX_EMS_TRG = 0x0003;
	inline constexpr uint16_t EMS_RQ = 0x0004;
	inline constexpr uint16_t COMMON_EMS_EN = 0x0005;

	inline constexpr uint16_t BATT_V = 0x0010;
	inline constexpr uint16_t V_LIMIT_HIGH = 0x0011;
	inline constexpr uint16_t V_LIMIT_LOW = 0x0012;

	inline constexpr uint16_t BATT_I = 0x0020;
	inline constexpr uint16_t I_LIMIT = 0x0021;

	inline constexpr uint16_t MONITOR_PERIOD = 0x00F0;
	inline constexpr uint16_t MONITOR_REG = 0x00F1;
}

namespace MDCRegister{
	inline constexpr uint16_t NOP = 0x0000;
	inline constexpr uint16_t MOTOR_TYPE = 0x0001;
	inline constexpr uint16_t CONTROL_MODE = 0x0002;
	inline constexpr uint16_t ABS_GEAR_RATIO = 0x0003;
	inline constexpr uint16_t MOTOR_STATE = 0x0004;
	inline constexpr uint16_t CAN_TIMEOUT = 0x0005;
	inline constexpr uint16_t DOB_EN = 0x0006;
	inline constexpr uint16_t LOAD_J = 0x0007;
	inline constexpr uint16_t LOAD_D = 0x0008;
	inline constexpr uint16_t DOB_CF = 0x0009;

	inline constexpr uint16_t TRQ = 0x0010;
	inline constexpr uint16_t TRQ_TARGET = 0x0011;

	inline constexpr uint16_t SPD = 0x0020;
	inline constexpr uint16_t SPD_TARGET = 0x0021;
	inline constexpr uint16_t TRQ_LIM = 0x0022;
	inline constexpr uint16_t SPD_GAIN_P = 0x0023;
	inline constexpr uint16_t SPD_GAIN_I = 0x0024;
	inline constexpr uint16_t SPD_GAIN_D = 0x0025;

	inline constexpr uint16_t POS = 0x0030;
	inline constexpr uint16_t POS_TARGET = 0x0031;
	inline constexpr uint16_t SPD_LIMIT = 0x0032;
	inline constexpr uint16_t POS_GAIN_P = 0x0033;
	inline constexpr uint16_t POS_GAIN_I = 0x0034;
	inline constexpr uint16_t POS_GAIN_D = 0x0035;

	inline constexpr uint16_t ABS_POS = 0x0036;
	inline constexpr uint16_t ABS_SPD = 0x0037;
	inline constexpr uint16_t ABS_TURN_CNT = 0x0039;

	inline constexpr uint16_t VESC_MODE = 0x0040;
	inline constexpr uint16_t VESC_TARGET = 0x0041;

	inline constexpr uint16_t MONITOR_PERIOD = 0x00F0;
	inline constexpr uint16_t MONITOR_REG = 0x00F1;

	enum class ControlMode{
		OPEN_LOOP,
		SPEED,
		POSITION,
		ABS_POSITION,
	};
	enum class VescMode{
		PWM,
		CURRENT,
		SPEED
	};

	enum class MotorType{
		C610 = 0x00,
		C620 = 0x01,
		VESC = 0x10,
	};
}

namespace GPIORegister{
	inline constexpr uint16_t NOP = 0x0000;
	inline constexpr uint16_t PORT_MODE = 0x0001;
	inline constexpr uint16_t PORT_READ = 0x0002;
	inline constexpr uint16_t PORT_WRITE = 0x0003;
	inline constexpr uint16_t PORT_INT_EN = 0x0004;
	inline constexpr uint16_t ESC_MODE_EN = 0x0005;

	inline constexpr uint16_t PWM_PERIOD = 0x0010;
	inline constexpr uint16_t PWM_DUTY = 0x0020;

	inline constexpr uint16_t MONITOR_PERIOD = 0x00F0;
	inline constexpr uint16_t MONITOR_REG = 0x00F1;
}


} //namespace CommonLib::Protocol

namespace PReg = CommonLib::Protocol::PCURegister;
namespace MReg = CommonLib::Protocol::MDCRegister;
namespace GReg = CommonLib::Protocol::GPIORegister;




#endif /* COMMONLIB_PROTOCOL_ID_DEFINES_HPP_ */
