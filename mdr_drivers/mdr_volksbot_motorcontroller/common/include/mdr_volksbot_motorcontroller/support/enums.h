#ifndef EnumAdapter_H
#define EnumAdapter_H

#include <iostream>

namespace VMC
{

	typedef unsigned char BYTE;

	/**
	 *  @brief adapter types
	 **/
	enum EnumAdapter
	{
		eAdapterRS232,
		eAdapterFILE,
		eAdapterCAN,
		eAdapterUSB,
		eAdapterNONE
	};

	/**
	 *  @brief motor states
	 **/
	enum EnumMotorState
	{
		eMotorStateFORWARD,
		eMotorStateBACKWARDS,
		eMotorStateBRAKE,
		eMotorStateSTOP
	};

	/**
	 *  @brief velocity input types
	 **/
	enum EnumVelocityInputType
	{
		eVelocityInputTypePWM,
		eVelocityInputTypeRPM
	};

	/**
	 *  @brief data types in the messages
	 **/
	enum EnumDataType
	{
		eDataTypeUnsignedChar,
		eDataTypeSignedChar,
		eDataTypeUnsignedInteger,
		eDataTypeSignedInteger,
		eDataTypeSignedLong,
		eDataTypeString
	};

	/**
	 *  @brief unit prefixes
	 **/
	enum EnumUnitPrefix
	{
		eUnitPrefixGIGA,
		eUnitPrefixMEGA,
		eUnitPrefixKILO,
		eUnitPrefixHECTO,
		eUnitPrefixDECA,
		eUnitPrefixONE,
		eUnitPrefixDECI,
		eUnitPrefixCENTI,
		eUnitPrefixMILLI,
		eUnitPrefixMICRO,
		eUnitPrefixNANO
	};

	/**
	 *  @brief units
	 **/
	enum EnumPhysicalUnit
	{
		ePhysicalUnitNONE,
		ePhysicalUnitVOLT,
		ePhysicalUnitAMPERE,
		ePhysicalUnitWATT,
		ePhysicalUnitOHM,
		ePhysicalUnitRPM,
		ePhysicalUnitSECOND,
		ePhysicalUnitMETER,
		ePhysicalUnitNEWTONMETER,
		ePhysicalUnitDEGREECELSIUS,
		ePhysicalUnitNEWTONMETER_AMPERE,
		ePhysicalUnitRPM_VOLT,
		ePhysicalUnitRPM_NEWTONMETER,
		ePhysicalUnitKELVINWATT
	};

}  // namespace fair
#endif //EnumAdapter_H
