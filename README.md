# BL602 LoRa Driver for Semtech SX1276 / HopeRF RF96

Ported from Apache Mynewt OS to BL602...

https://github.com/apache/mynewt-core/tree/master/hw/drivers/lora/sx1276

Read the articles...

- ["Connect PineCone BL602 to LoRa Transceiver"](https://lupyuen.github.io/articles/lora)

- ["PineCone BL602 RISC-V Board Receives LoRa Packets"](https://lupyuen.github.io/articles/lora2)

To add this driver to an existing BL602 / BL604 project:

```bash
cd bl_iot_sdk/components/3rdparty
git submodule add https://github.com/lupyuen/lora-sx1276
```

# Connect BL602 to SX1276

The pins are defined in [`include/sx1276.h`](include/sx1276.h)

Note that BL602 Pins are mapped to specific SPI Functions, so not all SPI Pins may be remapped.

| BL602 Pin     | SX1276 / RF96 Pin   | Wire Colour 
|:--------------|:--------------------|:-------------------
| __`GPIO 0`__  | `DIO1`              | Dark Green
| __`GPIO 1`__  | `ISO` _(MISO)_      | Light Green (Top)
| __`GPIO 2`__  | Do Not Connect      | (Unused Chip Select)
| __`GPIO 3`__  | `SCK`               | Yellow (Top)
| __`GPIO 4`__  | `OSI` _(MOSI)_      | Blue (Top)
| __`GPIO 5`__  | `DIO2`              | Blue (Bottom)
| __`GPIO 11`__ | `DIO0`              | Yellow (Bottom)
| __`GPIO 12`__ | `DIO3`              | Light Green (Bottom)
| __`GPIO 14`__ | `NSS`               | Orange
| __`GPIO 17`__ | `RST`               | White
| __`3V3`__     | `3.3V`              | Red
| __`GND`__     | `GND`               | Black

From [`include/sx1276.h`](include/sx1276.h):

```c
#define SX1276_SPI_IDX      0  //  SPI Port 0
#define SX1276_SPI_SDI_PIN  1  //  SPI Serial Data In Pin  (formerly MISO)
#define SX1276_SPI_SDO_PIN  4  //  SPI Serial Data Out Pin (formerly MOSI)
#define SX1276_SPI_CLK_PIN  3  //  SPI Clock Pin
#define SX1276_SPI_CS_PIN  14  //  SPI Chip Select Pin
#define SX1276_SPI_CS_OLD   2  //  Unused SPI Chip Select Pin
#define SX1276_NRESET      17  //  Reset Pin
#define SX1276_DIO0        11  //  DIO0: Trigger for Packet Received
#define SX1276_DIO1         0  //  DIO1: Trigger for Sync Timeout
#define SX1276_DIO2         5  //  DIO2: Trigger for Change Channel (Spread Spectrum / Frequency Hopping)
#define SX1276_DIO3        12  //  DIO3: Trigger for CAD Done
#define SX1276_DIO4        -1  //  DIO4: Unused (FSK only)
#define SX1276_DIO5        -1  //  DIO5: Unused (FSK only)
#define SX1276_SPI_BAUDRATE  (200 * 1000)  //  SPI Frequency (200 kHz)
#define SX1276_LF_USE_PA_BOOST  1  //  Enable Power Amplifier Boost for LoRa Frequency below 525 MHz
#define SX1276_HF_USE_PA_BOOST  1  //  Enable Power Amplifier Boost for LoRa Frequency 525 MHz and above
```

(CAD = Channel Activity Detection. We detect whether a Radio Channel is in use, by scanning very quickly for the LoRa Packet Preamble)

# Demo Firmware

To transmit and receive LoRa packets with the driver, run the `sdk_app_lora` BL602 Demo Firmware...

- [`sdk_app_lora`: BL602 Demo Firmware for LoRa SX1262 / SX1276 ](../../../customer_app/sdk_app_lora)

Here's a sample log...

```text
# create_task

# init_driver
SX1276 init
SX1276 interrupt init
SX1276 register handler: GPIO 11
SX1276 register handler: GPIO 0
SX126 register handler: GPIO 5
SX1276 register handler: GPIO 12
TODO: os_cputime_delay_usecs 1000
TODO: os_cputime_delay_usecs 6000
SX1276 DIO3: Channel activity detection

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca6
Rx done: 
48 65 6c 6c 6f 

# spi_result
DIO0 Interrupts: 1
DIO1 Interrupts: 0
DIO2 Interrupts: 0
DIO3 Interrupts: 1
DIO4 Interrupts: 0
DIO5 Interrupts: 0
Unknown Int:     0
Tx Interrupts:   302
Tx Status:       0x0
Tx Term Count:   0x0
Tx Error:        0x0
Rx Interrupts:   302
Rx Status:       0x0
Rx Term Count:   0x0
Rx Error:        0x0

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca6
Rx done: 
48 65 6c 6c 6f 

# spi_result
DIO0 Interrupts: 2
DIO1 Interrupts: 0
DIO2 Interrupts: 0
DIO3 Interrupts: 1
DIO4 Interrupts: 0
DIO5 Interrupts: 0
Unknown Int:     0
Tx Interrupts:   354
Tx Status:       0x0
Tx Term Count:   0x0
Tx Error:        0x0
Rx Interrupts:   354
Rx Status:       0x0
Rx Term Count:   0x0
Rx Error:        0x0

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca6
Rx done: 
48 65 6c 6c 6f 

# spi_result
DIO0 Interrupts: 3
DIO1 Interrupts: 0
DIO2 Interrupts: 0
DIO3 Interrupts: 1
DIO4 Interrupts: 0
DIO5 Interrupts: 0
Unknown Int:     0
Tx Interrupts:   406
Tx Status:       0x0
Tx Term Count:   0x0
Tx Error:        0x0
Rx Interrupts:   406
Rx Status:       0x0
Rx Term Count:   0x0
Rx Error:        0x0

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca6
Rx done: 
48 65 6c 6c 6f 

# spi_result
DIO0 Interrupts: 4
DIO1 Interrupts: 0
DIO2 Interrupts: 0
DIO3 Interrupts: 1
DIO4 Interrupts: 0
DIO5 Interrupts: 0
Unknown Int:     0
Tx Interrupts:   458
Tx Status:       0x0
Tx Term Count:   0x0
Tx Error:        0x0
Rx Interrupts:   458
Rx Status:       0x0
Rx Term Count:   0x0
Rx Error:        0x0

# receive_message
SX1276 receive timeout
Rx timeout

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca8
Rx done: 
48 65 6c 6c 6f 

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca8
Rx done: 
48 65 6c 6c 6f 

# receive_message
SX1276 receive timeout
Rx timeout

# receive_message
SX1276 DIO0: Packet received
Rx done: RadioEvents.RxDone=0x23000ca8
Rx done: 
48 65 6c 6c 6f 
```
