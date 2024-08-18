
> [!IMPORTANT]
> The full simulation files and codes will be updated after the final presentation in November, ensuring all files are the final version.

> [!CAUTION]
> Always exercise caution when testing with motors and propellers.

# TO DO
- [x] Update the Readme File (18/08)
- [ ] Add the Simulink files
- [ ] Add specific Simulink information
- [ ] Add the MCU code

# Objective

This documentation covers the hardware and software for a device that controls BLDC motors using a BlHeli32 ESC and PWM to prevent exceeding the power limit in SAE Brasil. The device is for educational purposes only, and all data used in this project will be updated soon

# Hardware

This project is based on the STM32F411, with the first version using a Blackpill ([WeAct Studio](https://www.weact-tc.cn/2019/11/30/STM32Download/#more)) and open-source hardware.

## ESC
To run the BLDC motor and gather data for control law and safety functions, a BlHeli_32 ESC is used due to its internal telemetry and lightweight design. The following data are utilized in this project:

* System Voltage (V)
* Motor Current (A)
* Temperature (°C)

Additional data, such as *eRPM* and *Battery Capacity*, are available in the BlHeli data frame and may be used in future versions of the project.

The data frame sent by BlHeli_32 is as follows:

![Dataframe](/images/DataFrame.png)

The ESC communicates via USART at 115200 bps with 9-bit parity. To avoid issues, a 16-bit CRC is recommended. The ESC datasheet provides an example of how to implement the CRC.

*CRC Example:*

```
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++) crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
    uint8_t crc = 0, i;
    for (i = 0; i < BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}
```

## Assembly System

To prevent bad connections and organize all electrical components, a PCB was manufactured with the following features:

* LED Error Code
* User Communication (USART Channel)
* 2x 12-bit ADC Connections (For use with a generic ESC)
* Radio key connection (To bypass radio signal to ESC) for the safety system.

![PCB](/images/PCB.jpg)

![LP](/images/LP.png)

* **ESC-OUT:** PWM signal to command the ESC (Max 3.3V), output only, connected to PA0
* **ESC-TEL:** Based on BLHeli32 Firmware, connect to USART1, receive only, to receive ESC telemetry data (configurable via firmware)
* **EXT-TEL:** External telemetry in case the ESC doesn't have telemetry; includes 2 ports (PA1 and PA4), up to 12-bit ADC, max 3.3V (voltage and current ports must be configured by firmware)
* **USART:** General communication with MCU for IHM interface, bitrate 115200 bps, can be used to configure and calibrate parameters like input range, output range, fault codes, and more.
* **Radio_Ch3:** Used to read the throttle signal from RC input (PPM/PWM) - PA8
* **Radio_Ch8:** Used to read the activation/deactivation key from RC input (PPM/PWM); once activated, it bypasses the signal from the RC receiver to ESC - PA9

# Control Law

The mathematical model is based on the modeling of the ESC (the gate bridge and MOSFETs), motor, and propeller in Simulink, to obtain optimal control. Previous versions used PID control to prevent exceeding the power limit, but the Simulink model was created to enhance control and system precision.

*General Scheme*
![Dataframe](/images/Simulink_General.jpg)


*Motor Configuration*
@@Soon@@

*ESC Configuration*
@@Soon@@

*Propeller Configuration*
@@Soon@@

# References

Thank you to all authors who contributed to this work.

**BlHeli_32 Manual:**  
https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20manual%20ARM%20Rev32.x.pdf

**Propeller Technologies for Regional Aircraft:**  
Methven, P., "Propeller Technologies for Regional Aircraft," SAE Technical Paper 910997, 1991, https://doi.org/10.4271/910997.

**An Underactuated Propeller for Attitude Control in Micro Air Vehicles**  
J. Paulos and M. Yim, "An underactuated propeller for attitude control in micro air vehicles," 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems, Tokyo, Japan, 2013, pp. 1374-1379, doi: 10.1109/IROS.2013.6696528.

**Dynamic Model of an Aircraft Propulsion System with Electric Motor and Propeller**  
M. M. Rivera and L. B. Gutiérrez, "Dynamic model of an aircraft propulsion system with electric motor and propeller," 2021 IEEE 5th Colombian Conference on Automatic Control (CCAC), Ibague, Colombia, 2021, pp. 157-162, doi: 10.1109/CCAC51819.2021.9633324.

**Six Step Commutation of a BLDC Motor**  
Andrew Rayman (2024). Six Step Commutation of a BLDC Motor (https://github.com/andrewrays/bldc-six-step/releases/tag/v1.0.1), GitHub. Retrieved August 18, 2024.

# Contact Info

Email: gabrielfrancischetti22@gmail.com  
LinkedIn: https://www.linkedin.com/in/gabriel-francischetti22/
