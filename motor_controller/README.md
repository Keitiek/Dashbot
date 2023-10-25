# ASI Motor Controller
We are using Accelerated Systems Inc. BAC555 motor controller for our project along with a communication harness. 

## Specifications
- **Power:** 500 Watts
- **Nom. Voltage:**	24 - 48 Volts
- **Peak Voltage:** 60 Volts
- **Nom. Current:**	50 Amps
- **Peak Current:** 70-75 Amps
- **CAN BUS:** Yes
- **Auto Tuning:**  Yes
- **Sensorless Start:**	Adaptive
- **IP Rating:** IP65 (IP67 with sealed connectors)
- **QC Verification:** 100% EOL Test under load
- **Additional Available Protocols:** TTL 485, Bluetooth, LIN, CAN
- **Qualifications:** EN15194:2017, ISO13849-1:2015

## Manufacturer and Product Page
- **Manufacturer:** [ASI](https://www.acceleratedsystems.com/)
- **Product Page:** [ASI BAC555 Official Page](https://www.acceleratedsystems.com/products/electric-motor-controllers/bac355-bac555)
- **User Manual** [User Manual](https://www.ebikes.ca/downloads/BAC500_Controller_Manual_Rev%201.0.pdf)
- **ASI Controller Configuration (Eggrider) https://manual.eggrider.com/ebike_settings/asi/#required-asi-controller-configuration


## Communication and Interface
- **TTL-232-CANOpen** Standard
- **CANOpen with BLE** Optional
- **TTL-232 with RS-485** Optional
- **TTL-232 with TTL-232** Optional
- **TTL-232 with BLE** Optional

## Terms and Abbreviations
### Terms
- **BacDoor** - ASI motor controller software

### Abbreviations
- **ESC** - electronic speed controller, uses ModBus as the protocol for configuration and diagnostics
- **PAS** - pedal assist level (1-4 to drive)

## Controller Power and Performance
- **PWM frequency** 13.5 kHz default / up to 16.5 kHz when operating in remote mode
- **Maximum Controller output frequency** 500 Hz
- **Electrical isolation to heat-sink** 500 VAC
- **Storage ambient temperature** -40°C to 75°C
- **Operating ambient temperature** -20°C to 50°C
- **Thermal cutback** Controller linearly reduces maximum current limit with an internal heat-sink temperature from 85°C to 95°C complete cutoff occurs above 95°C
- **Package environmental rating** IP67 (excluding electrical connections)
- **Speed regulation (range)** +/- 5% at top speed
- **Minimum motor phase to phase inductance** 20 μH
- **Motor control scheme** Sinusoidal field oriented (FOC)
- **Motors supported** PMAC and BLDC

# Study Resources

Ebike motor and brushless motor controller wiring explained: https://www.youtube.com/watch?v=lHJKBdvVDJc 

More detailed motor controller wiring explanation: https://www.youtube.com/watch?v=A8QcNvRh0wM

Housedillon reverse engineering motor controller: https://housedillon.com/blog/flash-part-five/
