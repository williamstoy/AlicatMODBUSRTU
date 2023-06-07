# Modbus RTU Communication
From ALICAT OPERATING MANUAL FOR STANDARD FLOW AND PRESSURE DEVICES
Models M · MC · P · PC · L · LC
February 2023 • Rev. 0 • Standard Devices User Manual
https://documents.alicat.com//manuals/DOC-MANUAL-MPL.pdf

Modbus RTU can be used to read and log sensor data, switch between analog and digital control modes, adjust device settings, and control the device.

## Establishing Communication
The device can communicate via the input/output port on the device.
- When using a USB cable to connect your device to a Windows 10 computer, it should recognize your USB as a virtual COM port automatically.
- After physically connecting your device to a Windows PC, you can check which COM port number it uses by opening the Windows Device Manager and expanding "Ports (COM & LPT)".
- The default Modbus configuration has the following settings:
    - Baud Rate: 19200
    - Data Bits: 8
    - Stop Bits: 1
    - Parity: None
    - Flow Control: None

### Multidrop Information
Devices can operate on networks with other devices. When installing devices in an RS-485 network with multiple types of devices, confirm that the total load of all devices does not exceed 32 units on an unrepeated network segment. Consult the EIA-485 standard for more information.

## Modbus RTU Serial Protocol
Alicat uses the Modbus standard of offsetting registers by 1 from addresses, meaning register 1 is equivalent to address 0. However, some systems expect data to be mapped as 0-indexed addresses. Different Modbus control systems may refer to registers, offsets, or addresses in their documentation without clarifying their meaning. If your control system uses a 0-indexed numbering scheme then decrement all register references in this manual by 1.
If you are unsure of which addressing scheme your control system uses, perform a test read of register 1200. If the device responds with Error code 2: “Illegal Data Address”, then your system is using the standard 1-indexed numbering system and the values in this manual can be used as-is. If the device returns a value of 0 instead of an error, decrement all registers listed in this section by 1 to arrive at the correct offset.

### Process Data
Alicat devices make no distinction between “Input” and “Holding” registers. Modbus function codes FC03 and FC04 can be used interchangeably to read data from the device. Sensor and process values are stored as big-endian, 32-bit IEEE-754 floating point numbers spanning two registers. Your control system will need to chain these into a single value to interpret them correctly.

## Writing Control and Configuration Information
All command and control requests to a Device are issued with Modbus function code FC16: “write multiple registers”.

### Modbus Reading and Status Registers
| Parameter             | Register | Access    | Devices               |
|-----------------------|----------|-----------|-----------------------|
| Command ID            | 1000     | Read/Write| All                   |
| Command Argument      | 1001     | Read/Write| All                   |
| Setpoint              | 1010–1011| Read/Write| Controllers           |
| Mixture Gas 1 Index   | 1050     | Read/Write| Mass Flow             |
| Mixture Gas 1 Percent | 1051     | Read/Write| Mass Flow             |
| Mixture Gas 2 Index   | 1052     | Read/Write| Mass Flow             |
| Mixture Gas 2 Percent | 1053     | Read/Write| Mass Flow             |
| Mixture Gas 3 Index   | 1054     | Read/Write| Mass Flow             |
| Mixture Gas 3 Percent | 1055     | Read/Write| Mass Flow             |
| Mixture Gas 4 Index   | 1056     | Read/Write| Mass Flow             |
| Mixture Gas 4 Percent | 1057     | Read/Write| Mass Flow             |
| Mixture Gas 5 Index   | 1058     | Read/Write| Mass Flow             |
| Mixture Gas 5 Percent | 1059     | Read/Write| Mass Flow             |
| Gas Number            | 1200     | Read/Write| Mass Flow             |
| Status Flags          | 1201–1202| Read      | All                   |
| Pressure Statistic    | 1203–1204| Read      | All                   |
| Flow Temperature      | 1205–1206| Read      | Mass Flow and Liquid  |
| Volumetric Flow       | 1207–1208| Read      | Mass Flow and Liquid  |
| Mass Flow             | 1209–1210| Read      | Mass Flow             |

### Status Flags
| Bit | Interpretation                      | Devices               |
|-----|-------------------------------------|-----------------------|
| 0   | Temperature Overflow (TOV)          | Mass Flow and Liquid  |
| 1   | Temperature Underflow (TOV)         | Mass Flow and Liquid  |
| 2   | Volumetric Overflow (VOV)           | Controllers           |
| 3   | Volumetric Underflow (VOV)          | Mass Flow             |
| 4   | Mass Overflow (MOV)                 | Mass Flow             |
| 5   | Mass Underflow (MOV)                | Mass Flow             |
| 6   | Pressure Overflow (POV)             | All                   |
| 7   | Totalizer Overflow (OVR)            | Mass Flow and Liquid  |
| 8   | PID Loop in Hold (HLD)              | Controllers           |
| 9   | ADC Error (ADC)                     | All                   |
| 10  | PID Exhaust (EXH)                   | Dual Valve Controllers|
| 11  | Over pressure limit (OPL)           | Custom OPL devices    |
| 12  | Flow overflow during totalize (TMF) | Mass Flow and Liquid  |
| 13  | Measurement was aborted             | All                   |

## Special Commands
You can access special control functions on devices with an FC16 write to registers 1000–1001. Special commands consist of a Command ID and a Command Argument written in a single pass to these registers. Each command/argument pair issues as a set of two 16-bit unsigned integers. Commands start by a write to register 1000. If you send a command to register 1000 without sending an argument to 1001 the device will interpret the command with a default argument of 0.
At any time after sending a special command to registers 1000..1001 you can perform a read of the same registers to determine the success or failure of the last command. Register 1000 will store the last Command ID sent to the device, and register 1001 will return a status code indicating the command result.

### Custom Gas Mixtures
Custom gas mixtures can be saved with 2–5 gases using the mix registers 1050–1059. This does not instruct the device to mix the gas, it programs it to be able to measure a premixed custom gas blend.
Saving the mix is a two-step process. First, the desired constituent gas indexes and percentages must be written to the mix registers (1050–1059). Then, a write of the Create Custom Gas Mixture command (ID 2) into command register 1000 (with the command argument being optional).
Gas mix percentages are interpreted as integer hundredths of a percent and the total percentage must sum to 100%. For example, to specify a mix of 50%, a value of 5000 is written into the gas percentage register.
The mix is performed with the first N gases that have a non- zero percentage. As an example, if you wish to use a mixture of 3 gases, write the index and percentage for those gases into registers 1050–1055 and write a value of zero into 1057–1059.
If the command argument is 0 or is omitted, the custom gas mixture is allocated in the next empty gas mix index starting at 255 and working down to 236. If no user mix indexes are available, the command fails and an error is returned in the command argument register.
To specify which index to write the custom gas mixture to, use a command argument between 236 and 255. The specified index either creates or updates to the new composition. If the specified index is not valid (the command argument is neither 0 nor 236–255), an error is returned.
Upon completion, the command argument register is updated with the new mixture. If the mix is valid, the index of the mixed gas is returned. If one of the requested mix gas constituents did not exist, or the percentage does not add to 100%, an error value is returned and the mixture is not saved. 
All gas mixtures are accessible via Gas Select™ on the front panel.

### Special Commands and Arguments
| Command Name                    | Command ID (Register 1000) | Command Argument (Register 1001)                       | Devices                                    |
|---------------------------------|----------------------------|--------------------------------------------------------|--------------------------------------------|
| Change Gas Number               | 1                          | Gas Table Index                                        | Mass Flow                                  |
| Create Custom Gas Mixture       | 2                          | Gas mixture index (236–255) or 0 to use next available | Mass Flow                                  |
| Delete Custom Gas Mixture       | 3                          | Gas mixture index                                      | Mass Flow                                  |
| Tare                            | 4                          | 0: Pressure<br>1: Absolute Pressure<br>2: Volume (Mass Flow and Liquid Devices) | All                                    |
| Reset Totalizer Value           | 5                          | 0: Reset totalizer                                     | Mass Flow and Liquid                       |
| Valve Setting                   | 6                          | 0: Cancel<br>1: Hold Close<br>2: Hold Current<br>3: Exhaust (Dual valve controllers) | Controllers                          |
| Display Lock                    | 7                          | 0: Unlock<br>1: Lock                                   | All                                        |
| Change P in PID Loop            | 8                          | 0–65535                                                | Controllers                                |
| Change D in PID Loop            | 9                          | 0–65535                                                | Controllers                                |
| Change I in PID Loop            | 10                         | 0–65535                                                | Controllers                                |
| Change Control Loop Variable    | 11                         | 0: Mass Flow Controllers<br>1: Mass Flow and Liquid Controllers<br>2: PSID Controllers<br>3: Abs Press Controllers and Mass Flow Controllers<br>4: Gauge Pressure Controllers, Mass Flow Controllers with barometer, Liquid Controllers<br>0: Control Mass Flow<br>1: Control Volumetric Flow<br>2: Differential Pressure<br>3: Absolute Pressure<br>4: Gauge Pressure | All                                    |
| Save Current Setpoint to Memory | 12                         | 0: Save setpoint for power-up                          | Controllers                                |
| Change Loop Control Algorithm   | 13                         | 1: PD<br>2: PDDI                                       | Controllers                                |
| Read PID Value                  | 14                         | 0: PID P<br>1: PID D<br>2: PID I                       | Controllers                                |
| Change Modbus ID                | 32767                      | 1–247 = New ID                                         | All                                        |

### Special Command Result Status Codes
| Status Code | Result                                         |
|-------------|------------------------------------------------|
| 0           | Success                                        |
| 32769       | Invalid command ID                             |
| 32770       | Invalid setting                                |
| 32771       | Requested feature is unsupported               |
| 32772       | Invalid Gas Mix Index (Mass Flow Devices)      |
| 32773       | Invalid Gas Mix Constituent (Mass Flow Devices)|
| 32774       | Invalid Gas Mix Percentage (Mass Flow Devices) |
