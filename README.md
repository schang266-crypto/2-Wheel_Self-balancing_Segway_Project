# 2-Wheel_Self-balancing_Segway_Project
## Project Overview
  This project implements the digital control system for a two-wheeled, self-balancing "Segway" style device. The system is designed to independently drive left and right motors forward or in reverse to maintain platform balance based on pitch readings. Additionally, it supports user steering via a slide potentiometer, detects rider presence and balance using floor-mounted load cells, and features Bluetooth Low Energy (BLE) integration for secure rider authentication.  

<img width="393" height="478" alt="Screenshot 2026-07-06 at 4 29 51 PM" src="https://github.com/user-attachments/assets/e927574f-6b7d-433e-902a-ad6d40173db5" />

  To ensure the highest system stability, each submodules were first verified individually, then the top-level design underwent comprehensive pre-synthesis verification to validate the functional correctness of the RTL description. Following this, logic synthesis and area optimization were performed on the Segway top module, our design was a thousand square microns away from the targeted area, which is 11900 square microns. We then conducted thorough post-synthesis verification to ensure that the synthesized netlist met all timing constraints and hardware behavioral expectations. After successfully passing all debugging and verification stages, the design was mapped onto an FPGA for final hardware deployment on the physical Segway.

<img width="703" height="245" alt="image" src="https://github.com/user-attachments/assets/ebd19ed2-4906-47a5-8741-4e0d8ca595b4" />


## Top-Level Architecture
    The top-level module (Segway.sv) serves as the synthesized digital core of the device. It acts as the central hub, integrating various subsystems to process physical sensor inputs (inertial, analog-to-digital, and UART) and translate them into Motor PWM controls and audible piezo warnings.

## Hardware Module Descriptions
## UART Communication & Authentication Subsystem
    These four modules form the secure interface of the Segway, handling external serial communication and system authorization, operating at a baud rate of 9600. Together, they manage everything from bit-level serial transport to high-level command processing and security validation.

# UART_rx
    Utilizes a shift register to sample the incoming serial bitstream and assemble it into 8-bit parallel data. Its state machine logic governs the transition from waiting for a start bit, counting through the 8 data bits, and validating the stop bit before asserting a data-ready flag. It also includes double-flopping to prevent metastability.

# UART_tx
    Functions in reverse by loading 8-bit parallel data into a shift register and shifting it out bit-by-bit onto the serial TX line. Its state machine logic automatically structures the frame by outputting the start bit, sequencing the data bits, and appending the stop bit to maintain protocol compliance.

# auth_SM (Authentication State Machine)
    Handles rider authorization by communicating with a BLE module over UART. It Powers up the balance control system when a valid 'G' (0x47) command is received from the authorized app, while safely shuts down the system when an 'S' (0x53) disconnect command is received and the load cells indicate the rider has stepped off the platform.  


## Analog-to-Digital (A/D) Data Acquisition Subsystem
  The three modules (A2D_intf.sv, SPI_mnrch.sv, SPI_ADC128S.sv) operate together to form the Segway's data acquisition pipeline. Within this architecture, the **A2D_intf** acts as the high-level master that initiates and manages data collection, while the **SPI_ADC128S** serves as the peripheral slave device during testing to ensure the communication logic is flawless before hardware implementation.

# A2D_intf (A2D Interface)
  Acting as the master of the data acquisition subsystem, this module orchestrates the entire analog-to-digital conversion process. It utilizes an internal finite state machine (FSM) to request sequential conversions in a round-robin fashion. Specifically, it continuously cycles through and reads data from four designated channels:
Channel 0: Left load cell
Channel 4: Right load cell
Channel 5: Steering potentiometer
Channel 6: Battery voltage
  It manages this sampling sequence by sending the target channel configurations to the underlying SPI core and then collects the resulting parallel digital data, routing it to the Segway's core control logic.

# SPI_mnrch (SPI Master/Monarch Interface)
  Instantiated directly within the A2D_intf master module, this component serves as the physical hardware driver for the SPI bus. It translates the channel requests from the A2D interface into serial communication. It manages protocol-level timing (transitioning through Front Porch, Transfer, and Back Porch states), drives the serial clock (SCLK) and Slave Select (SS_n) lines, and handles the actual 16-bit serial data exchange over the MOSI and MISO lines.

# SPI_ADC128S (ADC SPI Slave Simulator)
  Serving as the slave in the verification environment, this simulation-only behavioral module faithfully mimics the timing and protocol characteristics of a physical National Semiconductor ADC128S chip. By receiving channel commands from SPI_mnrch and returning mock sensor data for channels 0, 4, 5, and 6, it allows the team to thoroughly test the round-robin logic of the A2D_intf master during pre-synthesis simulation without requiring physical hardware.


## Steering Enable State Machine (steer_en.sv)
    Monitors the left and right load cell weights received from the A2D_intf to verify that a rider is present (exceeding a minimum weight threshold) and laterally balanced.  Only allows steering inputs to affect the motor drives if the rider's weight is properly distributed over a specific period of time (1.34 sec).


## Inertial Sensing & Attitude Estimation Subsystem
    These two modules work in tandem to capture raw motion data from an external Inertial Measurement Unit (IMU) and translate it into a stable, noise-filtered, and drift-compensated pitch angle for the vehicle's balancing controller.
    
# Inertial Sensor Interface (inert_intf.sv)
    This module acts as the hardware manager for the IMU, utilizing a finite state machine to orchestrate initial sensor configuration (setting sample rates and interrupts) and continuous data retrieval via an SPI master. It incorporates a multi-stage double-flop synchronizer on the external hardware interrupt (INT) to prevent metastability before sequentially reading the 8-bit low and high bytes of both pitch rate and Z-axis acceleration. Finally, it assembles these fragments into full 16-bit signals, filters out extreme spurious gyroscope readings during normal operation, and asserts a single-cycle data-valid strobe (vld) once a full packet is processed.

# Inertial Integrator & Sensor Fusion Engine (inertial_integrator.sv)
    This module implements a precise complementary sensor fusion algorithm to calculate a stable, drift-compensated vehicle pitch angle. After subtracting fixed calibration offsets from the raw gyroscope and accelerometer signals, it translates the Z-axis acceleration into a reference angle by multiplying it by a scaling factor of 327. To eliminate long-term gyro drift, the engine continuously compares this reference angle to the current system pitch and applies a bounded correction factor (±1024). Every time the vld signal pulses, it integrates these values into a 27-bit accumulator (ptch_int) and outputs the optimized pitch angle from the upper bits ([26:11])


## Balance Control Subsystem
    These two modules form the core balancing intelligence of the Segway. They work seamlessly together to translate raw physical sensor data (such as vehicle pitch and steering inputs) into precise, compensated motor drive signals. 

# PID (Proportional-Integral-Derivative Controller)
    This module computes the primary balancing effort required to keep the vehicle upright. It processes the system's pitch (ptch) and pitch rate (ptch_rt) to calculate the Proportional, Integral, and Derivative terms, which are summed to generate the PID_cntrl signal. The internal integrator features anti-windup saturation logic and a configurable fast_sim mode that lowers bit precision to dramatically accelerate simulation times during testing. Furthermore, it includes a large counter that functions as a soft-start timer (ss_tmr), which begins incrementing upon power-up to ensure gradual torque application.

# SegwayMath (Motor Speed & Dynamics Calculator)
    This module translates the centralized balancing effort into specific, physical drive signals for the left and right motors (lft_spd, rght_spd). It first multiplies the PID_cntrl by the ss_tmr to ensure a smooth, jerk-free power-up transition. When steering is enabled (en_steer), it applies a proportional offset derived from the steering potentiometer (steer_pot), adjusting the left and right torque values differentially to execute turns. To physically move the Segway, it also applies deadband compensation—injecting a minimum duty cycle (MIN_DUTY) to overcome the static friction and inertia of the real-world motors. Finally, it rigidly saturates the outputs to 12-bit signed values and monitors the speed, triggering a too_fast warning flag if either motor exceeds a hardcoded threshold of 1536.


## Motor Drive (mtr_drv.sv)
    It converts the signed speed from the balance_cntrl block into active PWM signals (PWM1 and PWM2) to control the left and right motor H-Bridges. It monitors instantaneous over-current signals from the motors and immediately shuts down the drives if consecutive fault conditions occur outside of a safe blanking period.


## Piezo Drive (peizo_drv.sv)
    It serves as the auditory feedback system for the Segway. It controls an external piezoelectric buzzer to play specific musical sequences (fanfares) that alert the rider to various vehicle states (too fast, enable steering, and battery low). The module uses internal timers (dividing the 50MHz clock) to generate square waves at precise musical frequencies (G6, C7, E7, and G7). A key feature of this module is its strict priority and sequencing logic, which handles three distinct alert conditions. When multiple alerts occur simultaneously, the system enforces a strict priority hierarchy: too_fast > batt_low > en_steer.


## SegwayModel.sv (Segway Physical Model / Testbench Helper)
    A simulation-only behavioral module used exclusively within the testbench environment. It models the physical dynamics of the actual Segway, including the PWM duty cycle response of the motors, torque-to-acceleration translation, wheel angular velocity, and chassis rotation physics. It also emulates the SPI interface of an Inertial Measurement Unit (IMU), enabling full closed-loop control algorithm verification without physical hardware.
