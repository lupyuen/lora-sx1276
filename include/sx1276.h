//  Based on https://github.com/apache/mynewt-core/blob/master/hw/drivers/lora/sx1276/src/sx1276.h
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1276 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __SX1276_H__
#define __SX1276_H__
#include <stdint.h>
#include <stdbool.h>
#include "radio.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"

/* Connect BL602 to SX1276 / RF96 LoRa Transceiver
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
*/

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

//  CAD = Channel Activity Detection. We detect whether a Radio Channel 
//  is in use, by scanning very quickly for the LoRa Packet Preamble.

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME                           1 // [ms]

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34

/*!
 * Radio FSK modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint8_t  PayloadLen;
    uint16_t PreambleLen;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    bool     FixLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
} RadioFskSettings_t;

/*!
 * Radio FSK packet handler state
 */
typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
    int32_t  AfcValue;
} RadioFskPacketHandler_t;

/*!
 * Radio LoRa modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint8_t  Coderate;
    uint8_t  HopPeriod;
    uint8_t  PayloadLen;
    uint16_t PreambleLen;
    uint32_t Bandwidth;
    uint32_t Datarate;
    uint32_t TxTimeout;
    bool     LowDatarateOptimize;
    bool     FixLen;
    bool     CrcOn;
    bool     FreqHopOn;
    bool     IqInverted;
    bool     RxContinuous;
    bool     PublicNetwork;
} RadioLoRaSettings_t;

/*!
 * Radio LoRa packet handler state
 */
typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_t;

/*!
 * Radio Settings
 */
typedef struct
{
    RadioState_t             State;
    RadioModems_t            Modem;
    uint32_t                 Channel;
    RadioFskSettings_t       Fsk;
    RadioFskPacketHandler_t  FskPacketHandler;
    RadioLoRaSettings_t      LoRa;
    RadioLoRaPacketHandler_t LoRaPacketHandler;
}RadioSettings_t;

/*!
 * Radio hardware and global parameters
 */
typedef struct SX1276_s
{
    RadioSettings_t Settings;
}SX1276_t;

struct ble_npl_event;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void (DioIrqHandler)(struct ble_npl_event *ev);

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

#define RX_BUFFER_SIZE                              256

/*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void SX1276Init(RadioEvents_t *events);

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t SX1276GetStatus(void);

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void SX1276SetModem(RadioModems_t modem);

/*!
 * \brief Sets the channels configuration
 *
 * \param [IN] freq         Channel RF frequency
 */
void SX1276SetChannel(uint32_t freq);

/*!
 * \brief Sets the channels configuration
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] freq       Channel RF frequency
 * \param [IN] rssiThresh RSSI threshold
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool SX1276IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh,
                         uint32_t maxCarrierSenseTime);

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either SX1276SetRxConfig or
 *         SX1276SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t SX1276Random(void);

/*!
 * \brief Sets the reception parameters
 *
 * \remark When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A (set to 0)
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A (set to 0)
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value (LoRa only)
 *                          FSK : N/A (set to 0)
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed lenght is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A (set to 0)
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols bewteen each hop
 *                          FSK : N/A (set to 0)
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A (set to 0)
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void SX1276SetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                         bool iqInverted, bool rxContinuous);

/*!
 * \brief Sets the transmission parameters
 *
 * \remark When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A (set to 0)
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A (set to 0)
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols bewteen each hop
 *                          FSK : N/A (set to 0)
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A (set to 0)
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void SX1276SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool FreqHopOn,
                        uint8_t HopPeriod, bool iqInverted, uint32_t timeout);

/*!
 * \brief Computes the packet time on air in us for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] pktLen     Packet payload length
 *
 * \retval airTime        Computed airTime (us) for the given packet payload length
 */
uint32_t SX1276GetTimeOnAir(RadioModems_t modem, uint8_t pktLen);

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1276Send(uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the radio in sleep mode
 */
void SX1276SetSleep(void);

/*!
 * \brief Sets the radio in standby mode
 */
void SX1276SetStby(void);

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms] [0: continuous, others timeout]
 */
void SX1276SetRx(uint32_t timeout);

/*!
 * \brief Start a Channel Activity Detection
 */
void SX1276StartCad(void);

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t SX1276ReadRssi(RadioModems_t modem);

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1276Write(uint16_t addr, uint8_t data);

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t SX1276Read(uint16_t addr);

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void SX1276SetMaxPayloadLength(RadioModems_t modem, uint8_t max);

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void SX1276SetPublicNetwork(bool enable);

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t SX1276GetWakeupTime(void);

void SX1276RxDisable(void);

#endif // __SX1276_H__
