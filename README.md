# RP2040_zero_dcc_decoder

The project assumed:
- using the existing board with the RP2040 controller called RP2040 zero
- using commercially available boards with a uSD card slot

![RP2040_and_adapter_view](20240126_182355.jpg)

## Adapter features
- voltage regulations from DCC track to RP2040 logic
- voltage regulations from DCC track to adaprer logic
- voltage  regulations from DCC track to DC motor
- 3 MOSFET outs pulled down
- 3 DC power outs 14V - 18V - depence of DCC track power
- 1 speaker out
- MISO, MOSI, CS, CLK outs for uSD adapter
- TX, RX, GND, VCC outputs for DFmini player adapter

## Software features
- decoding DCC signals
- DC motor power driving
- 3 digital outs driving
- 1 PWM output for speaker 
- 11025 kHz 8bit 1 channel .wav files playing
- 4 .wav files played together 
- config file on uSD card
- .wav files on the uSD card
