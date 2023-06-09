// REFERENCES

// ./documents/DOC-MANUAL-MPL.pdf
// ./documents/ModbusRTU_Manual.pdf



#include <Arduino.h>
#include <ModbusInterface.h>
#include <AlicatModbusRTU.h>



/// @brief Initialize the AlicatModbusRTU object
/// @param modbusID Modbus ID of the Alicat device (0-247)
/// @param deviceType specify the device type (see DEVICE_TYPE_* constants)
/// @param modbus handle to the ModbusInterface object
/// @param serial handle to the HardwareSerial object
/// @param verbose if true, print verbose success / error message output to the serial port
AlicatModbusRTU::AlicatModbusRTU(int modbusID, int deviceType, ModbusInterface& modbus, HardwareSerial& serial, bool verbose) 
: _deviceType(deviceType), _modbus(modbus), _serial(serial)
{
  _verbose = verbose;

  setModbusID(modbusID);
  setRegisterOffset(-1);
}

// @todo: check to make sure that the modbus interface is correct for Alicat Devices
/*
Baud Rate: 19200
Data Bits: 8
Stop Bits: 1
Parity: None (This cannot be changed, per Alicat Engineers)
Flow Control: None
*/


/** 
 * HELPER FUNCTIONS
*/

/// @brief Set the register offset of the Alicat device (All devices)
/// @param registerOffset offset added to each register address before read or write (default: -1)
void AlicatModbusRTU::setRegisterOffset(int registerOffset) {
  _registerOffset = registerOffset;
}



/// @brief Set the Modbus ID of the Alicat device (All devices)
/// @param modbusID Modbus ID of the Alicat device (0-247)
void AlicatModbusRTU::setModbusID(int modbusID) {
  if (modbusID < 0 || modbusID > 247) {
    if (_verbose) _serial.println("ERROR: function:'setModbusID', argument modbusID is out of bounds");

    return;
  }

  _modbusID = modbusID;
}



/// @brief Perform the register offset calculation
/// @param address desired register address
/// @return offset register address
int AlicatModbusRTU::offsetRegister(int address) {
  // prevent underflow of the register address
  if (address == 0 && _registerOffset < 0) {
    if (_verbose) _serial.println("WARNING: function:'offsetRegister', register address underflow");

    return 0;
  }

  // prevent overflow of the register address

  return address + _registerOffset;
}



/// @brief Get the register address of a device statistic
/// @param statisticIndex index of the desired statistic (1-20)
/// @param registerAddress calculated register address of the desired device statistic
void AlicatModbusRTU::getDeviceStatisticRegisterAddress(int statisticIndex, int *registerAddress) {
  if (statisticIndex < 1 || statisticIndex > 20) {
    if (_verbose) _serial.println("ERROR: function:'getDeviceStatisticRegisterAddress', argument statisticIndex is out of bounds");

    return;
  }

  *registerAddress = REGISTER_DEVICE_STATISTIC_1_VALUE + 2*(statisticIndex - 1);
}



/**
 * READ AND WRITE REGISTER FUNCTIONS
*/

/// @brief Read a single register from the Alicat device (All devices)
/// @param registerAddress desired register address
/// @param registerValue value of the register read from the Alicat device
void AlicatModbusRTU::readSingleRegister(int registerAddress, uint16_t *registerValue) {
  const int dataLength = 1;
  uint16_t response[dataLength];

  if (!_modbus.readHoldingRegisterValues(_modbusID, offsetRegister(registerAddress), dataLength, response)) {
      _serial.print("ERROR: Failed to read register: ");
      _serial.println(registerAddress);

      return;
  }

  *registerValue = response[0];
}



/// @brief Read two registers, starting at the specified address, and interpret the response as an IEEE 32-bit float
/// @param registerAddress starting register address
/// @param floatValue result of the read operation, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::readRegistersAsFloat(int registerAddress, float *floatValue) {
  const int dataLength = 2;
  uint16_t response[dataLength];

  union {
    float asFloat;
    uint16_t asBytes[dataLength];
  } floatValueUnion;

  if (!_modbus.readHoldingRegisterValues(_modbusID, offsetRegister(registerAddress), dataLength, response)) {
      _serial.print("ERROR: Failed to read register: ");
      _serial.println(registerAddress);
      return;
  }

  // concatenate the response bytes into a single float using the following format:
  // All 32-bit values are handled in consecutive Modbus registers in big-
  // endian format. This means bits 31:16 are in the lower numbered Modbus
  // register and bits 15:0 are in the higher register. All floating-point values
  // are IEEE 32-bit floats.

  floatValueUnion.asBytes[1] = response[0];
  floatValueUnion.asBytes[0] = response[1];

  *floatValue = floatValueUnion.asFloat;
}



/// @brief Write a float value to two 16 bit registers, starting at the specified address
/// @param registerAddress starting register address
/// @param floatValue desired float value to write to the Alicat device
void AlicatModbusRTU::writeRegistersAsFloat(int registerAddress, float floatValue) {
  const int dataLength = 2;
  
  union {
    float asFloat;
    uint16_t asBytes[dataLength];
  } floatValueUnion;

  floatValueUnion.asFloat = floatValue;

  uint16_t data[dataLength];
  data[0] = floatValueUnion.asBytes[1];
  data[1] = floatValueUnion.asBytes[0];

  _modbus.writeHoldingRegisterValues(_modbusID, offsetRegister(registerAddress), data, dataLength);
}



/// @brief Write a single register to the Alicat device (All devices)
/// @param registerAddress starting register address
/// @param registerValue value to write to the Alicat device
void AlicatModbusRTU::writeSingleRegister(int registerAddress, uint16_t registerValue) {
  uint16_t registerValueArray[1] = { registerValue };

  _modbus.writeHoldingRegisterValues(_modbusID, offsetRegister(registerAddress), registerValueArray, 1);
}


/**
 * Modbus READING AND STATUS REGISTERS
*/


/// @brief Set the setpoint of the Alicat device (Controller devices only)
/// @param setpoint desired setpoint value
void AlicatModbusRTU::setSetpoint(float setpoint) {
  if (!deviceIsController()) {
    if (_verbose) _serial.println("ERROR: function, 'setSetpoint' is not used for devices of this type");

    return;
  }

  writeRegistersAsFloat(REGISTER_SETPOINT, setpoint);
}



/// @brief Get the setpoint of the Alicat device (Controller devices only)
/// @param setPoint result of the read operation, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::getSetpoint(float *setPoint) {
  if (!deviceIsController()) {
    if (_verbose) _serial.println("ERROR: function, 'setSetpoint' is not used for devices of this type");

    return;
  }

  int registerAddress;

  if (deviceIsMassFlow()) {
    getDeviceStatisticRegisterAddress(5, &registerAddress);
  } else if (deviceIsPressureController()) {
    getDeviceStatisticRegisterAddress(2, &registerAddress);
  }

  readRegistersAsFloat(registerAddress, setPoint);
}



/// @brief Get the pressure statistic of the Alicat device (All devices)
/// @param pressure pressure reading, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::getPressure(float *pressure) {
  int registerAddress;
  
  getDeviceStatisticRegisterAddress(1, &registerAddress);

  readRegistersAsFloat(registerAddress, pressure);
}



/// @brief Set the gas mixture properties of the Alicat device (Mass flow devices only) (specify the gases and their percentages in the mixture)
/// @param mixtureIndex index of the mixture (1-5)
/// @param gasIndex index of the gas from the gas table (0-210)
/// @param gasPercent percentage of the gas in the mixture (0.0-100.0)
void AlicatModbusRTU::setMixtureGasProperties(int mixtureIndex, uint16_t gasIndex, float gasPercent) {
  if (!deviceIsMassFlow()) {
    if (_verbose) _serial.println("ERROR: function, 'setMixtureGasProperties' is not used for devices of this type");

    return;
  }

  // check to make sure the mixture index is between 1 and 5
  if (mixtureIndex < 1 || mixtureIndex > 5) {
    if (_verbose) _serial.println("ERROR: function, 'setMixtureGasProperties', mixtureIndex must be between 1 and 5");

    return;
  }

  // check to make sure the gas index is between 0 and 210
  if (gasIndex < 0 || gasIndex > 210) {
    if (_verbose) _serial.println("ERROR: function, 'setMixtureGasProperties', gasIndex must be between 0 and 210");

    return;
  }

  // check to make sure the gas percent is between 0 and 100
  if (gasPercent < 0.0 || gasPercent > 100.0) {
    if (_verbose) _serial.println("ERROR: function, 'setMixtureGasProperties', gasPercent must be between 0 and 100");

    return;
  }

  int gasIndexRegisterAddress    = REGISTER_MIXTURE_GAS_1_INDEX + 2*(mixtureIndex - 1);
  int gasPercentRegisterAddress  = gasIndexRegisterAddress + 1;

  // write the gas index
  writeSingleRegister(gasIndexRegisterAddress, gasIndex);

  // write the gas percent
  // "...to specify a mix of 50%, a value of 5000 is written into the gas percentage register."
  uint16_t gasPercentInt = (uint16_t)round(gasPercent * 100.0);

  writeSingleRegister(gasPercentRegisterAddress, gasPercentInt);
}



/// @brief Get the properties of the gas mixture of the Alicat device (Mass flow devices only)
/// @param mixtureIndex index of the mixture (1-5)
/// @param gasIndex index of the gas from the gas table (0-210)
/// @param gasPercent percentage of the gas in the mixture (0.0-100.0)
void AlicatModbusRTU::getMixtureGasProperties(int mixtureIndex, uint16_t *gasIndex, float *gasPercent) {
  if (!deviceIsMassFlow()) {
    if (_verbose) _serial.println("ERROR: function, 'getMixtureGasProperties' is not used for devices of this type");

    return;
  }

  // check to make sure the mixture index is between 1 and 5
  if (mixtureIndex < 1 || mixtureIndex > 5) {
    if (_verbose) _serial.println("ERROR: function, 'getMixtureGasProperties', mixtureIndex must be between 1 and 5");

    return;
  }

  int gasIndexRegisterAddress    = REGISTER_MIXTURE_GAS_1_INDEX + (mixtureIndex - 1);
  int gasPercentRegisterAddress  = gasIndexRegisterAddress + 1;

  // read the gas index
  readSingleRegister(gasIndexRegisterAddress, gasIndex);

  // read the gas percent
  uint16_t gasPercentageInt;
  readSingleRegister(gasPercentRegisterAddress, &gasPercentageInt);

  *gasPercent = ((float)gasPercentageInt) / 100.0;
}



/// @brief Set the gas number of the Alicat device (Mass flow devices only)
/// @param gasIndex index of the gas from the gas table (0-210)
void AlicatModbusRTU::setGasNumber(uint16_t gasIndex) {
  if (!deviceIsMassFlow()) {
    if (_verbose) _serial.println("ERROR: function, 'setGasNumber' is not used for devices of this type");

    return;
  }

  // check to make sure the gas index is between 0 and 210
  if (gasIndex < 0 || gasIndex > 210) {
    if (_verbose) _serial.println("ERROR: function, 'setGasNumber', gasIndex must be between 0 and 210");

    return;
  }

  writeSingleRegister(REGISTER_GAS_NUMBER, gasIndex);
}



/// @brief Get the gas number from the Alicat device (Mass flow devices only)
/// @param gasIndex index of the gas from the gas table (0-210)
void AlicatModbusRTU::getGasNumber(uint16_t *gasIndex) {
  if (!deviceIsMassFlow()) {
    if (_verbose) _serial.println("ERROR: function, 'getGasNumber' is not used for devices of this type");

    return;
  }

  readSingleRegister(REGISTER_GAS_NUMBER, gasIndex);
}



/// @brief Read the status flags from the Alicat device (All devices) and store them in the _status struct
void AlicatModbusRTU::getStatusFlags() {
    uint16_t status;
    readSingleRegister(REGISTER_DEVICE_STATUS, &status);

    _status.TEMPERATURE_OVERFLOW          = status & STATUS_BIT_TEMPERATURE_OVERFLOW;
    _status.TEMPERATURE_UNDERFLOW         = status & STATUS_BIT_TEMPERATURE_UNDERFLOW;
    _status.VOLUMETRIC_OVERFLOW           = status & STATUS_BIT_VOLUMETRIC_OVERFLOW;
    _status.VOLUMETRIC_UNDERFLOW          = status & STATUS_BIT_VOLUMETRIC_UNDERFLOW;
    _status.MASS_OVERFLOW                 = status & STATUS_BIT_MASS_OVERFLOW;
    _status.MASS_UNDERFLOW                = status & STATUS_BIT_MASS_UNDERFLOW;
    _status.PRESSURE_OVERFLOW             = status & STATUS_BIT_PRESSURE_OVERFLOW;
    _status.TOTALIZER_OVERFLOW            = status & STATUS_BIT_TOTALIZER_OVERFLOW;
    _status.PID_LOOP_IN_HOLD              = status & STATUS_BIT_PID_LOOP_IN_HOLD;
    _status.ADC_ERROR                     = status & STATUS_BIT_ADC_ERROR;
    _status.PID_EXHAUST                   = status & STATUS_BIT_PID_EXHAUST;
    _status.OVER_PRESSURE_LIMIT           = status & STATUS_BIT_OVER_PRESSURE_LIMIT;
    _status.FLOW_OVERFLOW_DURING_TOTALIZE = status & STATUS_BIT_FLOW_OVERFLOW_DURING_TOTALIZE;
    _status.MEASUREMENT_ABORTED           = status & STATUS_BIT_MEASUREMENT_ABORTED;
    _status.ANY_ERROR                     = status > 0;

    if (_verbose) {
        _serial.print("STATUS Bits: ");
        _serial.println(status, BIN);

        if (_status.TEMPERATURE_OVERFLOW)           _serial.println("STATUS: TEMPERATURE OVERFLOW bit is set");
        if (_status.TEMPERATURE_UNDERFLOW)          _serial.println("STATUS: TEMPERATURE UNDERFLOW bit is set");
        if (_status.VOLUMETRIC_OVERFLOW)            _serial.println("STATUS: VOLUMETRIC OVERFLOW bit is set");
        if (_status.VOLUMETRIC_UNDERFLOW)           _serial.println("STATUS: VOLUMETRIC UNDERFLOW bit is set");
        if (_status.MASS_OVERFLOW)                  _serial.println("STATUS: MASS OVERFLOW bit is set");
        if (_status.MASS_UNDERFLOW)                 _serial.println("STATUS: MASS UNDERFLOW bit is set");
        if (_status.PRESSURE_OVERFLOW)              _serial.println("STATUS: PRESSURE OVERFLOW bit is set");
        if (_status.TOTALIZER_OVERFLOW)             _serial.println("STATUS: TOTALIZER OVERFLOW bit is set");
        if (_status.PID_LOOP_IN_HOLD)               _serial.println("STATUS: PID LOOP IN HOLD bit is set");
        if (_status.ADC_ERROR)                      _serial.println("STATUS: ADC ERROR bit is set");
        if (_status.PID_EXHAUST)                    _serial.println("STATUS: PID EXHAUST bit is set");
        if (_status.OVER_PRESSURE_LIMIT)            _serial.println("STATUS: OVER PRESSURE LIMIT bit is set");
        if (_status.FLOW_OVERFLOW_DURING_TOTALIZE)  _serial.println("STATUS: FLOW OVERFLOW DURING TOTALIZE bit is set");
        if (_status.MEASUREMENT_ABORTED)            _serial.println("STATUS: MEASUREMENT ABORTED bit is set");
    }
}



/// @brief Get the flow temperature from the Alicat device (Mass or liquid flow devices only)
/// @param flowTemperature flow temperature reading, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::getFlowTemperature(float *flowTemperature) {
  if (!deviceIsMassFlow() && !deviceIsLiquid()) {
    if (_verbose) _serial.println("ERROR: function, 'getFlowTemperature' is not used for devices of this type");

    return;
  }

  int registerAddress;
  
  getDeviceStatisticRegisterAddress(2, &registerAddress);

  readRegistersAsFloat(registerAddress, flowTemperature);
}



/// @brief Get the volumetric flow from the Alicat device (Mass or liquid flow devices only)
/// @param volumetricFlow volumetric flow reading, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::getVolumetricFlow(float *volumetricFlow) {
  if (!deviceIsMassFlow() && !deviceIsLiquid()) {
    if (_verbose) _serial.println("ERROR: function, 'getVolumetricFlow' is not used for devices of this type");

    return;
  }

  int registerAddress;
  
  getDeviceStatisticRegisterAddress(3, &registerAddress);

  readRegistersAsFloat(registerAddress, volumetricFlow);
}



/// @brief Get the mass flow from the Alicat device (Mass flow devices only)
/// @param massFlow mass flow reading, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::getMassFlow(float *massFlow) {
  if (!deviceIsMassFlow()) {
    if (_verbose) _serial.println("ERROR: function, 'getMassFlow' is not used for devices of this type");

    return;
  }

  int registerAddress;
  
  getDeviceStatisticRegisterAddress(4, &registerAddress);

  readRegistersAsFloat(registerAddress, massFlow);
}



/// @brief Get the total mass that has passed through the Alicat device (Mass flow devices only)
/// @param massTotal total mass reading, interpreted as an IEEE 32-bit float
void AlicatModbusRTU::getMassTotal(float *massTotal) {
  if (!deviceIsMassFlow()) {
    if (_verbose) _serial.println("ERROR: function, 'getMassFlow' is not used for devices of this type");

    return;
  }

  int registerAddress;

  if (deviceIsController()) {
    getDeviceStatisticRegisterAddress(6, &registerAddress);
  } else {
    getDeviceStatisticRegisterAddress(5, &registerAddress);
  }

  readRegistersAsFloat(registerAddress, massTotal);
}



/**
 * SPECIAL COMMANDS
*/

/// @brief Send a special command to the Alicat device (All devices)
/// @param command id of the special command to send
/// @param argument argument of the special command to send
/// @return true if the resulting status code is STATUS_CODE_SUCCESS, false otherwise
bool AlicatModbusRTU::sendSpecialCommand(uint16_t command, uint16_t argument) {
  uint16_t data[2] = { command, argument };

  _modbus.writeHoldingRegisterValues(_modbusID, offsetRegister(REGISTER_COMMAND_ID), data, 2);

  uint16_t status;
  readSingleRegister(REGISTER_COMMAND_ARGUMENT, &status);

  return handleSpecialCommandStatusCode(status);
}



/// @brief Handle the status code returned from a special command (All devices)
/// @param status status code returned from the Alicat device
/// @return true if the status code is STATUS_CODE_SUCCESS, false otherwise
bool AlicatModbusRTU::handleSpecialCommandStatusCode(uint16_t status) {  
  switch(status) {
    case STATUS_CODE_SUCCESS:
      if (_verbose) _serial.println("SUCCESS: Special command status code returned 0");
      return true;
      break;
    case STATUS_CODE_INVALID_COMMAND_ID:
      if (_verbose) _serial.println("ERROR: Invalid command ID");
      break;
    case STATUS_CODE_INVALID_SETTING:
      if (_verbose) _serial.println("ERROR: Invalid setting");
      break;
    case STATUS_CODE_REQUESTED_FEATURE_IS_UNSUPPORTED:
      if (_verbose)  _serial.println("ERROR: Requested feature is unsupported");
      break;
    case STATUS_CODE_INVALID_GAS_MIX_INDEX:
      if (_verbose) _serial.println("ERROR: Invalid Gas Mix Index (Mass Flow Devices)");
      break;
    case STATUS_CODE_INVALID_GAS_MIX_CONSTITUENT:
      if (_verbose) _serial.println("ERROR: Invalid Gas Mix Constituent (Mass Flow Devices)");
      break;
    case STATUS_CODE_INVALID_GAS_MIX_PERCENTAGE:
      if (_verbose) _serial.println("ERROR: Invalid Gas Mix Percentage (Mass Flow Devices)");
      break;
    default:
      if (_verbose) _serial.println("ERROR: Unknown status code");
      break;
  }

  return false;
}



/// @brief Change the gas number of the Alicat device (Mass flow devices only)
/// @param gasTableIndex index of the gas from the gas table (0-210)
void AlicatModbusRTU::changeGasNumber(uint16_t gasTableIndex) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_GAS_NUMBER, gasTableIndex);
}



/// @brief Create a custom gas mixture on the Alicat device (Mass flow devices only)
/// @param gasMixtureIndex index of the gas mixture (1-5)
void AlicatModbusRTU::createCustomGasMixture(uint16_t gasMixtureIndex) {
  sendSpecialCommand(SPECIAL_COMMAND_CREATE_CUSTOM_GAS_MIXTURE, gasMixtureIndex);
}



/// @brief Delete a custom gas mixture on the Alicat device (Mass flow devices only)
/// @param gasMixtureIndex index of the gas mixture (1-5)
void AlicatModbusRTU::deleteCustomGasMixture(uint16_t gasMixtureIndex) {
  sendSpecialCommand(SPECIAL_COMMAND_DELETE_CUSTOM_GAS_MIXTURE, gasMixtureIndex);
}



/// @brief Tare the Alicat device (All devices)
/// @param tareArgument select the tare type (see TARE_TYPE_* constants)
void AlicatModbusRTU::tare(uint16_t tareArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_TARE, tareArgument);
}



/// @brief Tare the Alicat device for pressure (All devices)
void AlicatModbusRTU::tarePressure() {
  tare(TARE_TYPE_PRESSURE);
}



/// @brief Tare the Alicat device for absolute pressure (All devices)
void AlicatModbusRTU::tareAbsolutePressure() {
  tare(TARE_TYPE_ABSOLUTE_PRESSURE);
}



/// @brief Tare the Alicat device for volume (Mass flow and liquid devices only)
void AlicatModbusRTU::tareVolume() {
  tare(TARE_TYPE_VOLUME);
}



/// @brief Reset the totalizer value of the Alicat device (Mass flow and liquid devices only)
void AlicatModbusRTU::resetTotalizerValue() {
  sendSpecialCommand(SPECIAL_COMMAND_RESET_TOTALIZER_VALUE, 0);
}



/// @brief Set the valve setting of the Alicat device (Controller devices only)
/// @param valveSettingArgument valve position setting (see VALVE_SETTING_* constants)
void AlicatModbusRTU::valveSetting(uint16_t valveSettingArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_VALVE_SETTING, valveSettingArgument);
}



/// @brief Cancel the valve setting (Controller devices only)
void AlicatModbusRTU::cancelValveSetting() {
  valveSetting(VALVE_SETTING_CANCEL);
}



/// @brief Hold the valve closed (Controller devices only)
void AlicatModbusRTU::holdValveClosed() {
  valveSetting(VALVE_SETTING_HOLD_CLOSE);
}



/// @brief Hold the current position of the valve (Controller devices only)
void AlicatModbusRTU::holdValveCurrent() {
  valveSetting(VALVE_SETTING_HOLD_CURRENT);
}



/// @brief Hold the exhaust valve open (Dual valve controller devices only)
void AlicatModbusRTU::exhaustValve() {
  valveSetting(VALVE_SETTING_EXHAUST);
}



/// @brief Set the display lock status of the Alicat device (All devices)
/// @param displayLockArgument display lock status (see DISPLAY_LOCK_* constants)
void AlicatModbusRTU::displayLock(uint16_t displayLockArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_DISPLAY_LOCK, displayLockArgument);
}



/// @brief Unlock the display of the Alicat device (All devices)
void AlicatModbusRTU::unlockDisplay() {
  displayLock(DISPLAY_LOCK_UNLOCK);
}



/// @brief Lock the display of the Alicat device (All devices)
void AlicatModbusRTU::lockDisplay() {
  displayLock(DISPLAY_LOCK_LOCK);
}



/// @brief Change the P in the PID loop
/// @param p Proportional Coefficient (0-65535)
void AlicatModbusRTU::changePinPIDLoop(uint16_t p) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_P_IN_PID_LOOP, p);
}


/// @brief Change the D in the PID loop
/// @param d Differential Coefficient (0-65535)
void AlicatModbusRTU::changeDinPIDLoop(uint16_t d) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_D_IN_PID_LOOP, d);
}


/// @brief Change the I in the PID loop
/// @param i Integral Coefficient (0-65535)
void AlicatModbusRTU::changeIinPIDLoop(uint16_t i) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_I_IN_PID_LOOP, i);
}



/// @brief Change the control loop variable of the Alicat device (Controller devices only)
/// @param controlLoopVariableArgument control loop variable (see CONTROL_LOOP_VARIABLE_* constants)
void AlicatModbusRTU::changeControlLoopVariable(uint16_t controlLoopVariableArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_CONTROL_LOOP_VARIABLE, controlLoopVariableArgument);
}



/// @brief Control the mass flow (Mass flow controller devices only)
void AlicatModbusRTU::controlMassFlow() {
  changeControlLoopVariable(CONTROL_LOOP_VARIABLE_MASS_FLOW);
}


/// @brief Control the volumetric (Mass flow and liquid controller devices only)
void AlicatModbusRTU::controlVolumetricFlow() {
  changeControlLoopVariable(CONTROL_LOOP_VARIABLE_VOLUME_FLOW);
}


/// @brief Control the differential pressure (PSID controller devices only)
void AlicatModbusRTU::controlDifferentialPressure() {
  changeControlLoopVariable(CONTROL_LOOP_VARIABLE_DIFFERENTIAL_PRESSURE);
}


/// @brief Control the absolute pressure (Mass flow and absolute pressure controller devices only)
void AlicatModbusRTU::controlAbsolutePressure() {
  changeControlLoopVariable(CONTROL_LOOP_VARIABLE_ABSOLUTE_PRESSURE);
}


/// @brief Control the gauge pressure (Mass flow controllers with barometer, liquid flow controllers, and gauge pressure controller devices only)
void AlicatModbusRTU::controlGaugePressure() {
  changeControlLoopVariable(CONTROL_LOOP_VARIABLE_GAUGE_PRESSURE);
}



/// @brief Save setpoint for power-up (Controller devices only)
void AlicatModbusRTU::saveCurrentSetpointToMemory() {
  sendSpecialCommand(SPECIAL_COMMAND_SAVE_CURRENT_SETPOINT_TO_MEMORY, 0);
}



/// @brief Change the loop control algorithm of the Alicat device (Controller devices only)
/// @param loopControlAlgorithmArgument loop control algorithm (see LOOP_CONTROL_ALGORITHM_* constants)
void AlicatModbusRTU::changeLoopControlAlgorithm(uint16_t loopControlAlgorithmArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_LOOP_CONTROL_ALGORITHM, loopControlAlgorithmArgument);
}



/// @brief Read the PID value of the Alicat device (Controller devices only)
/// @param PIDValueArgument PID value (see PID_VALUE_* constants)
void AlicatModbusRTU::readPIDValue(uint16_t PIDValueArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_READ_PID_VALUE, PIDValueArgument);
}



/// @brief Read the P value of the Alicat device (Controller devices only)
void AlicatModbusRTU::readPValue() {
  readPIDValue(PID_VALUE_P);
}



/// @brief Read the D value of the Alicat device (Controller devices only)
void AlicatModbusRTU::readDValue() {
  readPIDValue(PID_VALUE_D);
}



/// @brief Read the I value of the Alicat device (Controller devices only)
void AlicatModbusRTU::readIValue() {
  readPIDValue(PID_VALUE_I);
}



/// @brief Change the Modbus ID of the Alicat device (All devices)
/// @param modbusIDArgument desired Modbus ID of the Alicat device (0-247)
void AlicatModbusRTU::changeModbusID(uint16_t modbusIDArgument) {
  sendSpecialCommand(SPECIAL_COMMAND_CHANGE_MODBUS_ID, modbusIDArgument);
}



/**
 * DEVICE TYPE CHECK
*/

/// @brief Check if the Alicat device is a mass flow device
/// @return true if the device is a mass flow device, false otherwise
bool AlicatModbusRTU::deviceIsMassFlow() {
  return _deviceType == DEVICE_TYPE_MASS_FLOW_METER || _deviceType == DEVICE_TYPE_MASS_FLOW_CONTROLLER;
}



/// @brief Check if the Alicat device is a controller device
/// @return true if the device is a controller device, false otherwise
bool AlicatModbusRTU::deviceIsController() {
  return _deviceType == DEVICE_TYPE_PSID_CONTROLLER || _deviceType == DEVICE_TYPE_GAUGE_PRESSURE_CONTROLLER || _deviceType == DEVICE_TYPE_MASS_FLOW_CONTROLLER;
}



/// @brief Check if the Alicat device is a pressure controller
/// @return true if the device is a pressure controller, false otherwise
bool AlicatModbusRTU::deviceIsPressureController() {
  return _deviceType == DEVICE_TYPE_PSID_CONTROLLER || _deviceType == DEVICE_TYPE_GAUGE_PRESSURE_CONTROLLER;
}



/// @brief Check if the Alicat device is a PSID controller
/// @return true if the device is a PSID controller, false otherwise
bool AlicatModbusRTU::deviceIsPSIDController() {
  return _deviceType == DEVICE_TYPE_PSID_CONTROLLER;
}



/// @brief Check if the Alicat device is a liquid controller
/// @return true if the device is a liquid controller, false otherwise
bool AlicatModbusRTU::deviceIsLiquid() {
  return _deviceType == DEVICE_TYPE_LIQUID_CONTROLLER;
}


