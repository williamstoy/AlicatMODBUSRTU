// REFERENCES

// ./documentation/DOC-MANUAL-MPL.pdf
// ./documentation/ModbusRTU_Manual.pdf

#ifndef AlicatModbusRTU_h
    #define AlicatModbusRTU_h
    #include <Arduino.h>
    #include <ModbusInterface.h>

    #define DEVICE_TYPE_MASS_FLOW_CONTROLLER                0
    #define DEVICE_TYPE_LIQUID_CONTROLLER                   1
    #define DEVICE_TYPE_MASS_FLOW_METER                     2
    #define DEVICE_TYPE_PSID_CONTROLLER                     3
    #define DEVICE_TYPE_GAUGE_PRESSURE_CONTROLLER           4                               

    #define STATUS_CODE_SUCCESS                             0       // All
    #define STATUS_CODE_INVALID_COMMAND_ID                  32769   // All
    #define STATUS_CODE_INVALID_SETTING                     32770   // All
    #define STATUS_CODE_REQUESTED_FEATURE_IS_UNSUPPORTED    32771   // All
    #define STATUS_CODE_INVALID_GAS_MIX_INDEX               32772   // Mass Flow
    #define STATUS_CODE_INVALID_GAS_MIX_CONSTITUENT         32773   // Mass Flow
    #define STATUS_CODE_INVALID_GAS_MIX_PERCENTAGE          32774   // Mass Flow

    #define STATUS_BIT_TEMPERATURE_OVERFLOW                 0x0001  // Interpretation: Temperature Overflow (TOV), Devices: Mass Flow and Liquid
    #define STATUS_BIT_TEMPERATURE_UNDERFLOW                0x0002  // Interpretation: Temperature Underflow (TOV), Devices: Mass Flow and Liquid
    #define STATUS_BIT_VOLUMETRIC_OVERFLOW                  0x0004  // Interpretation: Volumetric Overflow (VOV), Devices: Mass Flow and Liquid
    #define STATUS_BIT_VOLUMETRIC_UNDERFLOW                 0x0008  // Interpretation: Volumetric Underflow (VOV), Devices: Mass Flow
    #define STATUS_BIT_MASS_OVERFLOW                        0x0010  // Interpretation: Mass Overflow (MOV), Devices: Mass Flow
    #define STATUS_BIT_MASS_UNDERFLOW                       0x0020  // Interpretation: Mass Underflow (MOV), Devices: Mass Flow
    #define STATUS_BIT_PRESSURE_OVERFLOW                    0x0040  // Interpretation: Pressure Overflow (POV), Devices: All
    #define STATUS_BIT_TOTALIZER_OVERFLOW                   0x0080  // Interpretation: Totalizer Overflow (OVR), Devices: Mass Flow and Liquid
    #define STATUS_BIT_PID_LOOP_IN_HOLD                     0x0100  // Interpretation: PID Loop in Hold (HLD), Devices: Controllers
    #define STATUS_BIT_ADC_ERROR                            0x0200  // Interpretation: ADC Error (ADC), Devices: All
    #define STATUS_BIT_PID_EXHAUST                          0x0400  // Interpretation: PID Exhaust (EXH), Devices: Dual Valve Controllers
    #define STATUS_BIT_OVER_PRESSURE_LIMIT                  0x0800  // Interpretation: Over pressure limit (OPL), Devices: Custom OPL devices
    #define STATUS_BIT_FLOW_OVERFLOW_DURING_TOTALIZE        0x1000  // Interpretation: Flow overflow during totalize (TMF), Devices: Mass Flow and Liquid
    #define STATUS_BIT_MEASUREMENT_ABORTED                  0x2000  // Interpretation: Measurement was aborted, Devices: All

    #define SPECIAL_COMMAND_CHANGE_GAS_NUMBER               1       // Mass Flow
    #define SPECIAL_COMMAND_CREATE_CUSTOM_GAS_MIXTURE       2       // Mass Flow
    #define SPECIAL_COMMAND_DELETE_CUSTOM_GAS_MIXTURE       3       // Mass Flow
    #define SPECIAL_COMMAND_TARE                            4       // All
    #define SPECIAL_COMMAND_RESET_TOTALIZER_VALUE           5       // Mass Flow and Liquid
    #define SPECIAL_COMMAND_VALVE_SETTING                   6       // Controllers
    #define SPECIAL_COMMAND_DISPLAY_LOCK                    7       // All
    #define SPECIAL_COMMAND_CHANGE_P_IN_PID_LOOP            8       // Controllers
    #define SPECIAL_COMMAND_CHANGE_D_IN_PID_LOOP            9       // Controllers
    #define SPECIAL_COMMAND_CHANGE_I_IN_PID_LOOP            10      // Controllers
    #define SPECIAL_COMMAND_CHANGE_CONTROL_LOOP_VARIABLE    11      // All
    #define SPECIAL_COMMAND_SAVE_CURRENT_SETPOINT_TO_MEMORY 12      // Controllers
    #define SPECIAL_COMMAND_CHANGE_LOOP_CONTROL_ALGORITHM   13      // Controllers
    #define SPECIAL_COMMAND_READ_PID_VALUE                  14      // Controllers
    #define SPECIAL_COMMAND_CHANGE_MODBUS_ID                32767   // All

    #define REGISTER_COMMAND_ID                             1000    // Access: Read/Write,  Devices: All
    #define REGISTER_COMMAND_ARGUMENT                       1001    // Access: Read/Write,  Devices: All
    #define REGISTER_SETPOINT                               1010    // Access: Write,       Devices: Controllers (MPL Manual says this is R/W, but it appears to be W only)
    #define REGISTER_MIXTURE_GAS_1_INDEX                    1050    // Access: Read/Write,  Devices: Mass Flow (All gas mixture indicies, n, can be accessed by adding 2*(n-1) to this starting register value)
    #define REGISTER_MIXTURE_GAS_1_PERCENT                  1051    // Access: Read/Write,  Devices: Mass Flow (All gas mixture percents, n, can be accessed by adding 2*(n-1)+1 to the starting register value of REGISTER_MIXTURE_GAS_1_INDEX)
    #define REGISTER_GAS_NUMBER                             1200    // Access: Read/Write,  Devices: Mass Flow
    #define REGISTER_DEVICE_STATUS                          1201    // Access: Read,        Devices: All
    #define REGISTER_DEVICE_STATISTIC_1_VALUE               1203    // Access: Read,        Devices: All (All device statistic values, n, can be accessed by adding 2*(n-1) to this starting register value)
    #define REGISTER_MASS_FLOW                              1209    // Access: Read,        Devices: Mass Flow

    #define TARE_TYPE_PRESSURE                              0
    #define TARE_TYPE_ABSOLUTE_PRESSURE                     1
    #define TARE_TYPE_VOLUME                                2       // Only for Mass Flow and Liquid Devices

    #define VALVE_SETTING_CANCEL                            0
    #define VALVE_SETTING_HOLD_CLOSE                        1
    #define VALVE_SETTING_HOLD_CURRENT                      2
    #define VALVE_SETTING_EXHAUST                           3       // Dual valve controller devices only

    #define DISPLAY_LOCK_UNLOCK                             0
    #define DISPLAY_LOCK_LOCK                               1

    #define CONTROL_LOOP_VARIABLE_MASS_FLOW                 0       // Only for Mass Flow Controllers
    #define CONTROL_LOOP_VARIABLE_VOLUME_FLOW               1       // Only for Mass Flow and Liquid Controllers
    #define CONTROL_LOOP_VARIABLE_DIFFERENTIAL_PRESSURE     2       // Only for PSID Controllers
    #define CONTROL_LOOP_VARIABLE_ABSOLUTE_PRESSURE         3       // Only for Mass Flow and Absolute Pressure Controllers
    #define CONTROL_LOOP_VARIABLE_GAUGE_PRESSURE            4       // Only for Mass Flow Controllers with a Barometer, Liquid Controllers, and Gauge Pressure Controllers

    #define LOOP_CONTROL_ALGORITHM_PD                       1
    #define LOOP_CONTROL_ALGORITHM_PDDI                     2

    #define PID_VALUE_P                                     0
    #define PID_VALUE_D                                     1
    #define PID_VALUE_I                                     2

    class AlicatModbusRTU {
        private:
            HardwareSerial&     _serial;
            ModbusInterface&    _modbus;
            bool                _verbose;
            int                 _registerOffset;
            int                 _modbusID;
            int                 _deviceType;

            struct {
                bool            TEMPERATURE_OVERFLOW;
                bool            TEMPERATURE_UNDERFLOW;
                bool            VOLUMETRIC_OVERFLOW;
                bool            VOLUMETRIC_UNDERFLOW;
                bool            MASS_OVERFLOW;
                bool            MASS_UNDERFLOW;
                bool            PRESSURE_OVERFLOW;
                bool            TOTALIZER_OVERFLOW;
                bool            PID_LOOP_IN_HOLD;
                bool            ADC_ERROR;
                bool            PID_EXHAUST;
                bool            OVER_PRESSURE_LIMIT;
                bool            FLOW_OVERFLOW_DURING_TOTALIZE;
                bool            MEASUREMENT_ABORTED;
                bool            ANY_ERROR;
            } _status;

        public:
                 AlicatModbusRTU(int modbusID, int deviceType, ModbusInterface& modbus, HardwareSerial& serial, bool verbose);
            void setRegisterOffset(int registerOffset);
            void setModbusID(int modbusID);
            int  offsetRegister(int address);
            void getGasNumber(uint16_t *gasIndex);
            void getStatusFlags();
            void getFlowTemperature(float *flowTemperature);
            void getVolumetricFlow(float *volumetricFlow);
            void getMassFlow(float *massFlow);
            void getMassTotal(float *massTotal);
            bool sendSpecialCommand(uint16_t command, uint16_t argument);
            bool handleSpecialCommandStatusCode(uint16_t statusCode);
            void readPIDValue(uint16_t coefficientID, uint16_t *coefficientValue);
            void changeGasNumber(uint16_t gasTableIndex);
            void createCustomGasMixture(uint16_t gasMixtureIndex);
            void deleteCustomGasMixture(uint16_t gasMixtureIndex);
            void getDeviceStatisticRegisterAddress(int statisticIndex, int *registerAddress);
            void readSingleRegister(int registerAddress, uint16_t *registerValue);
            void readRegistersAsFloat(int registerAddress, float *floatValue);
            void writeRegistersAsFloat(int registerAddress, float floatValue);
            void writeSingleRegister(int registerAddress, uint16_t registerValue);
            void setSetpoint(float setpoint);
            void getSetpoint(float *setPoint);
            void getPressure(float *pressure);
            void setMixtureGasProperties(int mixtureIndex, uint16_t gasIndex, float gasPercent);
            void getMixtureGasProperties(int mixtureIndex, uint16_t *gasIndex, float *gasPercent);
            void setGasNumber(uint16_t gasIndex);
            void tare(uint16_t tareArgument);
            void tarePressure();
            void tareAbsolutePressure();
            void tareVolume();
            void resetTotalizerValue();
            void valveSetting(uint16_t valveSettingArgument);
            void cancelValveSetting();
            void holdValveClosed();
            void holdValveCurrent();
            void exhaustValve();
            void displayLock(uint16_t displayLockArgument);
            void unlockDisplay();
            void lockDisplay();
            void changePinPIDLoop(uint16_t p);
            void changeDinPIDLoop(uint16_t d);
            void changeIinPIDLoop(uint16_t i);
            void changeControlLoopVariable(uint16_t controlLoopVariableArgument);
            void controlMassFlow();
            void controlVolumetricFlow();
            void controlDifferentialPressure();
            void controlAbsolutePressure();
            void controlGaugePressure();
            void saveCurrentSetpointToMemory();
            void changeLoopControlAlgorithm(uint16_t loopControlAlgorithmArgument);
            void readPIDValue(uint16_t PIDValueArgument);
            void readPValue();
            void readDValue();
            void readIValue();
            void changeModbusID(uint16_t modbusIDArgument);
            bool deviceIsMassFlow();
            bool deviceIsController();
            bool deviceIsPressureController();
            bool deviceIsLiquid();
            bool deviceIsPSIDController();
    };
#endif