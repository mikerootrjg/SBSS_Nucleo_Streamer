# SBSS_Nucleo_Streamer

FW for a ST Nucleo board that collects Stroke and Sequence signals and streams them over USB

## HW setup

This project runs on a NUCLEO-F767ZI board<br>
The FW will read an encoder input and up to 5 sequence signals, and output the results over USB

### The following modifications should be made on the Nucleo board:

For PA0:
* SB173 ON (this is the default, should not need a mod)
* SB180 OFF (this is the default, should not need a mod)

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
| Data Out    | UART_TX      | PD1       | CN11 Pin 55 or CN9 Pin 27

## Output

FW sends out 1000 lines per second over USB<br>

Each line is in the following format:<br>
```[SEQ],[ENC_POS],[ENC_DIR],[MC],[IF],[FS],[SS],[SR]\r\n```<br>
* SEQ: sequence number starting at 1
* ENC_POS: the encoder position, not zeroed
* ENC_DIR: `0` for decreasing, and `1` for increasing
* MC: Mold Closed state
* IF: Injection Forward state
* FS: First Stage state
* SS: Second Stage state
* SR: Screw Run state
