# 2-Wheel_Self-balancing_Segway_Project
Project Overview

This project implements the digital control system for a two-wheeled, self-balancing "Segway" style device constructed from wood and old bicycle wheels. The system is designed to independently drive left and right motors forward or in reverse to maintain platform balance based on pitch readings. Additionally, it supports user steering via a slide potentiometer, detects rider presence and balance using floor-mounted load cells, and features Bluetooth Low Energy (BLE) integration for secure rider authentication.  

<img width="393" height="478" alt="Screenshot 2026-07-06 at 4 29 51 PM" src="https://github.com/user-attachments/assets/e927574f-6b7d-433e-902a-ad6d40173db5" />

To ensure the highest system stability, each submodules were first verified individually, then the top-level design underwent comprehensive pre-synthesis verification to validate the functional correctness of the RTL description. Following this, logic synthesis and area optimization were performed on the Segway top module, our design was a thousand square microns away from the targeted area, which is 11900 square microns. We then conducted thorough post-synthesis verification to ensure that the synthesized netlist met all timing constraints and hardware behavioral expectations. After successfully passing all debugging and verification stages, the design was mapped onto an FPGA for final hardware deployment on the physical Segway.

<img width="703" height="245" alt="image" src="https://github.com/user-attachments/assets/ebd19ed2-4906-47a5-8741-4e0d8ca595b4" />

Top-Level Architecture

The top-level module (Segway.sv) serves as the synthesized digital core of the device. It acts as the central hub, integrating various subsystems to process physical sensor inputs (inertial, analog-to-digital, and UART) and translate them into Motor PWM controls and audible piezo warnings.

Hardware Module Descriptions

Authentication Block (auth_SM.sv)
。Handles rider authorization by communicating with a BLE module over UART.  
。Powers up the balance control system when a valid 'G' (0x47) command is received from the authorized app.  
。Shuts down the system when an 'S' (0x53) disconnect command is received and the load cells indicate that the rider has stepped off the platform.  

。UART_rx (UART Receiver): Handles the reception of serial communication data. This module integrates a double-flop synchronizer on the external RX signal to mitigate metastability. Guided by a finite state machine (FSM) and an internal baud rate counter (dividing the 50MHz system clock down to 9600 baud), it accurately samples and converts the incoming serial bitstream into 8-bit parallel data, asserting a rdy signal upon completion.

。UART_tx (UART Transmitter): Handles the transmission of serial communication data. Upon receiving a transmission command, the internal FSM appends a start bit (0) and a stop bit (1) to the 8-bit parallel data. The data is then shifted out sequentially via a shift register onto the TX line. The transmission speed is precisely controlled by an internal baud rate generator.

A2D Interface (A2D_intf.sv)
。Manages communication with the ADC128S022 Analog-to-Digital converter via an SPI monarch interface.  
。Requests sequential conversions in a round-robin fashion.  
。Reads data from four specific channels: the left load cell (channel 0), right load cell (channel 4), steering potentiometer (channel 5), and battery voltage (channel 6)

