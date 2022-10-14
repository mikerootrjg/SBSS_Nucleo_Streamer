# SBSS_Nucleo_Streamer
FW for a Nucleo board that collects Stroke and Sequence signals and streams them over USB

## HW setup
Nucleo FW will read an encoder input and up to 5 sequence signals

### The following modifications should be made on the Nucleo board:

For PA0:
* SB173 ON (this is the default)
* SB180 OFF (this is the default)

For PA1:
* SB13 OFF (this is NOT default, please remove)

### Signals should be connected to the Nucleo as follows:
| Signal      | uC Periph ID | uC Pin ID | Nucleo pin  |
| ----------- | ------------ | --------- | ----------- |
| Enc A       | TIM5_CH1     | PA0       | CN11 Pin 28 |
| Enc B       | TIM5_CH2     | PA1       | CN11 Pin 30 |
| Enc Z       | GPIO Input   | PA4       | CN11 Pin 32 |
| MC          | GPIO Input   | PD3       | CN11 Pin 40 |
| IF          | GPIO Input   | PD4       | CN11 Pin 39 |
| FS          | GPIO Input   | PD5       | CN11 Pin 41 |
| SS          | GPIO Input   | PD6       | CN11 Pin 43 |
| SR          | GPIO Input   | PD7       | CN11 Pin 45 |