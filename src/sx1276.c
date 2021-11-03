//  Based on https://github.com/apache/mynewt-core/blob/master/hw/drivers/lora/sx1276/src/sx1276.c
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1276 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
#include <assert.h>
#include <math.h>
#include <string.h>
#include <device/vfs_spi.h>  //  For spi_ioc_transfer_t
#include <hal/soc/spi.h>     //  For hal_spi_transfer
#include <hal_spi.h>         //  For spi_init
#include <bl_gpio.h>         //  For bl_gpio_output_set
#include <bl602_glb.h>       //  For GLB_GPIO_Func_Init
#include "nimble_npl.h"      //  For NimBLE Porting Layer (timer functions)
#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "sx1276-utilities.h"

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
} RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
} FskBandwidth_t;


/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration(void);

/*!
 * \brief Resets the SX1276
 */
void SX1276Reset(void);

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void SX1276SetTx(uint32_t timeout);

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo(uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo(uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1276SetOpMode(uint8_t opMode);

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq(struct ble_npl_event *ev);

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1276OnDio1Irq(struct ble_npl_event *ev);

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1276OnDio2Irq(struct ble_npl_event *ev);

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1276OnDio3Irq(struct ble_npl_event *ev);

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1276OnDio4Irq(struct ble_npl_event *ev);

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1276OnDio5Irq(struct ble_npl_event *ev);

/*!
 * \brief Tx & Rx timeout timer callback
 */
void SX1276OnTimeoutIrq(struct ble_npl_event *ev);

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t RadioEvents;  ////  Stores a copy of the callbacks
////  Previously: static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX1276_t SX1276;

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = { SX1276OnDio0Irq, SX1276OnDio1Irq,
                            SX1276OnDio2Irq, SX1276OnDio3Irq,
                            SX1276OnDio4Irq, NULL };

/*!
 * Tx and Rx Callout Timers
 */
struct ble_npl_callout TxTimeoutTimer;
struct ble_npl_callout RxTimeoutTimer;
struct ble_npl_callout RxTimeoutSyncWord;

static uint32_t rx_timeout_sync_delay = -1;

///////////////////////////////////////////////////////////////////////////////
//  Timer Functions

/// Initialise a timer. Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_init
void os_cputime_timer_init(
    struct ble_npl_callout *timer,  //  The timer to initialize. Cannot be NULL.
    ble_npl_event_fn *f,            //  The timer callback function. Cannot be NULL.
    void *arg)                      //  Pointer to data object to pass to timer.
{
    //  Implement with Callout Functions from NimBLE Porting Layer
    assert(timer != NULL);
    assert(f != NULL);

    //  Event Queue containing Events to be processed, defined in demo.c.  TODO: Move to header file.
    extern struct ble_npl_eventq event_queue;

    //  Init the Callout Timer with the Callback Function
    ble_npl_callout_init(
        timer,         //  Callout Timer
        &event_queue,  //  Event Queue that will handle the Callout upon timeout
        f,             //  Callback Function
        arg            //  Argument to be passed to Callback Function
    );
}

/// Stops a timer from running.  Can be called even if timer is not running.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_stop
void os_cputime_timer_stop(
    struct ble_npl_callout *timer)  //  Pointer to timer to stop. Cannot be NULL.
{
    //  Implement with Callout Functions from NimBLE Porting Layer
    assert(timer != NULL);

    //  If Callout Timer is still running...
    if (ble_npl_callout_is_active(timer)) {
        //  Stop the Callout Timer
        ble_npl_callout_stop(timer);
    }
}

/// Sets a timer that will expire ‘usecs’ microseconds from the current time.
/// NOTE: This must be called when the timer is stopped.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_relative
void os_cputime_timer_relative(
    struct ble_npl_callout *timer,  //  Pointer to timer. Cannot be NULL.
    uint32_t microsecs)             //  The number of microseconds from now at which the timer will expire.
{
    //  Implement with Callout Functions from NimBLE Porting Layer.
    //  Assume that Callout Timer has been stopped.
    assert(timer != NULL);

    //  Convert microseconds to ticks
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        microsecs / 1000  //  Duration in milliseconds
    );

    //  Wait at least 1 tick
    if (ticks == 0) { ticks = 1; }

    //  Trigger the Callout Timer after the elapsed ticks
    ble_npl_error_t rc = ble_npl_callout_reset(
        timer,  //  Callout Timer
        ticks   //  Number of ticks
    );
    assert(rc == 0);
}

/// Wait until ‘usecs’ microseconds has elapsed. This is a blocking delay.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_delay_usecs
void os_cputime_delay_usecs(
    uint32_t microsecs)  //  The number of microseconds to wait.
{
    //  Implement with Timer Functions from NimBLE Porting Layer.
    //  Convert microseconds to ticks.
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        microsecs / 1000  //  Duration in milliseconds
    );

    //  Wait at least 1 tick
    if (ticks == 0) { ticks = 1; }

    //  Wait for the ticks
    ble_npl_time_delay(ticks);
}

///////////////////////////////////////////////////////////////////////////////
//  SPI Functions

/// SPI Device Instance. TODO: Move to sx1276.h
extern spi_dev_t spi_device;

/// SPI Transmit Buffer (1 byte)
static uint8_t spi_tx_buf[1];

/// SPI Receive Buffer (1 byte)
static uint8_t spi_rx_buf[1];

/// Blocking call to send a value on the SPI. Returns the value received from the SPI Peripheral.
/// Assume that we are sending and receiving 8-bit values on SPI.
/// Assume Chip Select Pin has already been set to Low by caller.
/// TODO: We should combine multiple SPI DMA Requests, instead of handling one byte at a time
uint16_t hal_spi_tx_val(int spi_num, uint16_t val) {
    //  Populate the transmit buffer
    spi_tx_buf[0] = val;

    //  Clear the receive buffer
    memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));

    //  Prepare SPI Transfer
    static spi_ioc_transfer_t transfer;
    memset(&transfer, 0, sizeof(transfer));    
    transfer.tx_buf = (uint32_t) spi_tx_buf;  //  Transmit Buffer
    transfer.rx_buf = (uint32_t) spi_rx_buf;  //  Receive Buffer
    transfer.len    = 1;                      //  How many bytes

    //  Assume Chip Select Pin has already been set to Low by caller.

    //  Execute the SPI Transfer with the DMA Controller
    int rc = hal_spi_transfer(
        &spi_device,  //  SPI Device
        &transfer,    //  SPI Transfers
        1             //  How many transfers (Number of requests, not bytes)
    );
    assert(rc == 0);

    //  Return the received byte
    return spi_rx_buf[0];
}

///////////////////////////////////////////////////////////////////////////////
//  Driver Functions

double
ceil(double d)
{
    int64_t i;

    i = d;
    if (d == i) {
        return i;
    }
    return i + 1;
}

double
floor(double d)
{
    return (int64_t)d;
}

double
round(double d)
{
    return (int64_t)(d + 0.5);
}

static void
SX1276RxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    printf("Rx done: RadioEvents.RxDone=%p\r\n", RadioEvents.RxDone);
    if (RadioEvents.RxDone != NULL) {
        RadioEvents.RxDone(payload, size, rssi, snr);
    }
}

static void
SX1276RxError(void)
{
    if (RadioEvents.RxError != NULL) {
        RadioEvents.RxError();
    }
}

static void
SX1276RxTimeout(void)
{
    if (RadioEvents.RxTimeout != NULL) {
        RadioEvents.RxTimeout();
    }
}

static void
SX1276TxDone(void)
{
    if (RadioEvents.TxDone != NULL) {
        RadioEvents.TxDone();
    }
}

static void
SX1276TxTimeout(void)
{
    if (RadioEvents.TxTimeout != NULL) {
        RadioEvents.TxTimeout();
    }
}

/*
 * Radio driver functions implementation
 */
void
SX1276Init(RadioEvents_t *events)
{
    uint8_t i;

    ////  We copy the Event Callbacks from "events", because
    ////  "events" may be stored on the stack
    assert(events != NULL);
    memcpy(&RadioEvents, events, sizeof(RadioEvents));

    ////  Previously: RadioEvents = events;

    // Initialize driver timeout timers. NOTE: assumes timer configured.
    os_cputime_timer_init(&TxTimeoutTimer, SX1276OnTimeoutIrq, NULL);
    os_cputime_timer_init(&RxTimeoutTimer, SX1276OnTimeoutIrq, NULL);
    os_cputime_timer_init(&RxTimeoutSyncWord, SX1276OnTimeoutIrq, NULL);

    SX1276IoInit();
    SX1276IoIrqInit(DioIrq);

    SX1276Reset();

    RxChainCalibration();

    SX1276SetOpMode(RF_OPMODE_SLEEP);

    for (i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++) {
        SX1276SetModem(RadioRegsInit[i].Modem);
        SX1276Write(RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
    }

    SX1276SetModem(MODEM_FSK);

    SX1276.Settings.State = RF_IDLE;
}

RadioState_t
SX1276GetStatus(void)
{
    return SX1276.Settings.State;
}

void
SX1276SetChannel(uint32_t freq)
{
    SX1276.Settings.Channel = freq;
    freq = (uint32_t)((double)freq / (double)FREQ_STEP);
    SX1276Write(REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    SX1276Write(REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    SX1276Write(REG_FRFLSB, (uint8_t)(freq & 0xFF));
}

bool
SX1276IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh,
                    uint32_t maxCarrierSenseTime)
{
    bool status = true;
    int16_t rssi;
    uint32_t carrierSenseTime;

    SX1276SetModem(modem);

    SX1276SetChannel(freq);

    SX1276SetOpMode(RF_OPMODE_RECEIVER);

    /* Delay for 1 msec */
    os_cputime_delay_usecs(1000);

    carrierSenseTime = timer_get_current_time();

    // Perform carrier sense for maxCarrierSenseTime
    while (timer_get_elapsed_time(carrierSenseTime) < maxCarrierSenseTime) {
        rssi = SX1276ReadRssi(modem);
        if (rssi > rssiThresh) {
            status = false;
            break;
        }
    }
    SX1276SetSleep();
    return status;
}

uint32_t
SX1276Random(void)
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem(MODEM_LORA);

    // Disable LoRa modem interrupts
    SX1276Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED);

    // Set radio in continuous reception
    SX1276SetOpMode(RF_OPMODE_RECEIVER);

    for (i = 0; i < 32; i++) {
        os_cputime_delay_usecs(1000);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ((uint32_t)SX1276Read(REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    SX1276SetSleep();

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void
RxChainCalibration(void)
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276Read(REG_PACONFIG);
    initialFreq = (double)(((uint32_t)SX1276Read(REG_FRFMSB) << 16) |
                              ((uint32_t)SX1276Read(REG_FRFMID) << 8) |
                              ((uint32_t)SX1276Read(REG_FRFLSB))) * (double)FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write(REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    SX1276Write(REG_IMAGECAL, (SX1276Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while((SX1276Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel(868000000);

    // Launch Rx chain calibration for HF band
    SX1276Write(REG_IMAGECAL, (SX1276Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while((SX1276Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    // Restore context
    SX1276Write(REG_PACONFIG, regPaConfigInitVal);
    SX1276SetChannel(initialFreq);
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t
GetFskBandwidthRegValue(uint32_t bandwidth)
{
    uint8_t i;

    for (i = 0; i < (sizeof(FskBandwidths) / sizeof(FskBandwidth_t)) - 1; i++) {
        if ((bandwidth >= FskBandwidths[i].bandwidth) &&
            (bandwidth < FskBandwidths[i + 1].bandwidth)) {
            return FskBandwidths[i].RegValue;
        }
    }

    // ERROR: Value not found
    assert(false);
    while(1);
}

void
SX1276SetRxConfig(RadioModems_t modem, uint32_t bandwidth, uint32_t datarate,
                  uint8_t coderate, uint32_t bandwidthAfc, uint16_t preambleLen,
                  uint16_t symbTimeout, bool fixLen, uint8_t payloadLen,
                  bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                  bool iqInverted, bool rxContinuous)
{
    SX1276SetModem(modem);

    switch (modem) {
    case MODEM_FSK:
        SX1276.Settings.Fsk.Bandwidth = bandwidth;
        SX1276.Settings.Fsk.Datarate = datarate;
        SX1276.Settings.Fsk.BandwidthAfc = bandwidthAfc;
        SX1276.Settings.Fsk.FixLen = fixLen;
        SX1276.Settings.Fsk.PayloadLen = payloadLen;
        SX1276.Settings.Fsk.CrcOn = crcOn;
        SX1276.Settings.Fsk.IqInverted = iqInverted;
        SX1276.Settings.Fsk.RxContinuous = rxContinuous;
        SX1276.Settings.Fsk.PreambleLen = preambleLen;

        datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        SX1276Write(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        SX1276Write(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        SX1276Write(REG_RXBW, GetFskBandwidthRegValue(bandwidth));
        SX1276Write(REG_AFCBW, GetFskBandwidthRegValue(bandwidthAfc));

        SX1276Write(REG_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        SX1276Write(REG_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1) {
            SX1276Write(REG_PAYLOADLENGTH, payloadLen);
        } else {
            SX1276Write(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
        }

        SX1276Write(REG_PACKETCONFIG1,
                     (SX1276Read(REG_PACKETCONFIG1) &
                       RF_PACKETCONFIG1_CRC_MASK &
                       RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                       ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                       (crcOn << 4));
        break;
    case MODEM_LORA:
        // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        assert(bandwidth <= 2);

        bandwidth += 7;
        SX1276.Settings.LoRa.Bandwidth = bandwidth;
        SX1276.Settings.LoRa.Datarate = datarate;
        SX1276.Settings.LoRa.Coderate = coderate;
        SX1276.Settings.LoRa.PreambleLen = preambleLen;
        SX1276.Settings.LoRa.FixLen = fixLen;
        SX1276.Settings.LoRa.PayloadLen = payloadLen;
        SX1276.Settings.LoRa.CrcOn = crcOn;
        SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
        SX1276.Settings.LoRa.HopPeriod = hopPeriod;
        SX1276.Settings.LoRa.IqInverted = iqInverted;
        SX1276.Settings.LoRa.RxContinuous = rxContinuous;

        /* XXX: why is this done? Why not an error? */
        if (datarate > 12) {
            datarate = 12;
        } else if (datarate < 6) {
            datarate = 6;
        }

        if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 8) && (datarate == 12))) {
            SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
        } else {
            SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        SX1276Write(REG_LR_MODEMCONFIG1,
                     (SX1276Read(REG_LR_MODEMCONFIG1) &
                       RFLR_MODEMCONFIG1_BW_MASK &
                       RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                       RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                       (bandwidth << 4) | (coderate << 1) |
                       fixLen);

        SX1276Write(REG_LR_MODEMCONFIG2,
                     (SX1276Read(REG_LR_MODEMCONFIG2) &
                       RFLR_MODEMCONFIG2_SF_MASK &
                       RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                       RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                       (datarate << 4) | (crcOn << 2) |
                       ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

        SX1276Write(REG_LR_MODEMCONFIG3,
                     (SX1276Read(REG_LR_MODEMCONFIG3) &
                       RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                       (SX1276.Settings.LoRa.LowDatarateOptimize << 3));

        SX1276Write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

        SX1276Write(REG_LR_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        SX1276Write(REG_LR_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1) {
            SX1276Write(REG_LR_PAYLOADLENGTH, payloadLen);
        }

        if (SX1276.Settings.LoRa.FreqHopOn == true) {
            SX1276Write(REG_LR_PLLHOP, (SX1276Read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            SX1276Write(REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod);
        }

        if ((bandwidth == 9) && (SX1276.Settings.Channel > RF_MID_BAND_THRESH)) {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            SX1276Write(REG_LR_TEST36, 0x02);
            SX1276Write(REG_LR_TEST3A, 0x64);
        } else if (bandwidth == 9) {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            SX1276Write(REG_LR_TEST36, 0x02);
            SX1276Write(REG_LR_TEST3A, 0x7F);
        } else {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            SX1276Write(REG_LR_TEST36, 0x03);
        }

        if (datarate == 6) {
            SX1276Write(REG_LR_DETECTOPTIMIZE,
                         (SX1276Read(REG_LR_DETECTOPTIMIZE) &
                           RFLR_DETECTIONOPTIMIZE_MASK) |
                           RFLR_DETECTIONOPTIMIZE_SF6);
            SX1276Write(REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF6);
        } else {
            SX1276Write(REG_LR_DETECTOPTIMIZE,
                         (SX1276Read(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                         RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            SX1276Write(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }
        break;
    }
}

void
SX1276SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                  uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                  uint16_t preambleLen, bool fixLen, bool crcOn, bool freqHopOn,
                  uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    uint8_t paConfig;
    uint8_t paDac;

    SX1276SetModem(modem);

    paConfig = SX1276Read(REG_PACONFIG);
    paDac = SX1276Read(REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | SX1276GetPaSelect(SX1276.Settings.Channel);
    paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK) | 0x70;

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        } else {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }

        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }

            if (power > 20) {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        } else {
            if (power < 2) {
                power = 2;
            }

            if (power > 17) {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    } else {
        if (power < -1) {
            power = -1;
        }

        if (power > 14) {
            power = 14;
        }
        paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power + 1) & 0x0F);
    }
    SX1276Write(REG_PACONFIG, paConfig);
    SX1276Write(REG_PADAC, paDac);

    switch (modem) {
    case MODEM_FSK:
        SX1276.Settings.Fsk.Power = power;
        SX1276.Settings.Fsk.Fdev = fdev;
        SX1276.Settings.Fsk.Bandwidth = bandwidth;
        SX1276.Settings.Fsk.Datarate = datarate;
        SX1276.Settings.Fsk.PreambleLen = preambleLen;
        SX1276.Settings.Fsk.FixLen = fixLen;
        SX1276.Settings.Fsk.CrcOn = crcOn;
        SX1276.Settings.Fsk.IqInverted = iqInverted;
        SX1276.Settings.Fsk.TxTimeout = timeout;

        fdev = (uint16_t)((double)fdev / (double)FREQ_STEP);
        SX1276Write(REG_FDEVMSB, (uint8_t)(fdev >> 8));
        SX1276Write(REG_FDEVLSB, (uint8_t)(fdev & 0xFF));

        datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        SX1276Write(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        SX1276Write(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        SX1276Write(REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
        SX1276Write(REG_PREAMBLELSB, preambleLen & 0xFF);

        SX1276Write(REG_PACKETCONFIG1,
                     (SX1276Read(REG_PACKETCONFIG1) &
                       RF_PACKETCONFIG1_CRC_MASK &
                       RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                       ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                       (crcOn << 4));
        break;
    case MODEM_LORA:
        SX1276.Settings.LoRa.Power = power;
        if (bandwidth > 2) {
            // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            assert(false);
            while(1);
        }
        bandwidth += 7;
        SX1276.Settings.LoRa.Bandwidth = bandwidth;
        SX1276.Settings.LoRa.Datarate = datarate;
        SX1276.Settings.LoRa.Coderate = coderate;
        SX1276.Settings.LoRa.PreambleLen = preambleLen;
        SX1276.Settings.LoRa.FixLen = fixLen;
        SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
        SX1276.Settings.LoRa.HopPeriod = hopPeriod;
        SX1276.Settings.LoRa.CrcOn = crcOn;
        SX1276.Settings.LoRa.IqInverted = iqInverted;
        SX1276.Settings.LoRa.TxTimeout = timeout;

        if (datarate > 12) {
            datarate = 12;
        } else if (datarate < 6) {
            datarate = 6;
        }

        if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 8) && (datarate == 12))) {
            SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
        } else {
            SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        if (SX1276.Settings.LoRa.FreqHopOn == true) {
            SX1276Write(REG_LR_PLLHOP, (SX1276Read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            SX1276Write(REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod);
        }

        SX1276Write(REG_LR_MODEMCONFIG1,
                     (SX1276Read(REG_LR_MODEMCONFIG1) &
                       RFLR_MODEMCONFIG1_BW_MASK &
                       RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                       RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                       (bandwidth << 4) | (coderate << 1) |
                       fixLen);

        SX1276Write(REG_LR_MODEMCONFIG2,
                     (SX1276Read(REG_LR_MODEMCONFIG2) &
                       RFLR_MODEMCONFIG2_SF_MASK &
                       RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
                       (datarate << 4) | (crcOn << 2));

        SX1276Write(REG_LR_MODEMCONFIG3,
                     (SX1276Read(REG_LR_MODEMCONFIG3) &
                       RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                       (SX1276.Settings.LoRa.LowDatarateOptimize << 3));

        SX1276Write(REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
        SX1276Write(REG_LR_PREAMBLELSB, preambleLen & 0xFF);

        if (datarate == 6) {
            SX1276Write(REG_LR_DETECTOPTIMIZE,
                         (SX1276Read(REG_LR_DETECTOPTIMIZE) &
                           RFLR_DETECTIONOPTIMIZE_MASK) |
                           RFLR_DETECTIONOPTIMIZE_SF6);
            SX1276Write(REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF6);
        } else {
            SX1276Write(REG_LR_DETECTOPTIMIZE,
                         (SX1276Read(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                         RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            SX1276Write(REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }
        break;
    }
}

uint32_t
SX1276GetTimeOnAir(RadioModems_t modem, uint8_t pktLen)
{
    uint32_t airtime;
    double bw;

    switch (modem) {
    case MODEM_FSK:
        airtime = round((8 * (SX1276.Settings.Fsk.PreambleLen +
                                 ((SX1276Read(REG_SYNCCONFIG) & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1) +
                                 ((SX1276.Settings.Fsk.FixLen == 0x01) ? 0.0 : 1.0) +
                                 (((SX1276Read(REG_PACKETCONFIG1) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK) != 0x00) ? 1.0 : 0) +
                                 pktLen +
                                 ((SX1276.Settings.Fsk.CrcOn == 0x01) ? 2.0 : 0)) /
                                 SX1276.Settings.Fsk.Datarate) * 1e3);
        break;
    case MODEM_LORA:
        // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        switch (SX1276.Settings.LoRa.Bandwidth) {
        //case 0: // 7.8 kHz
        //    bw = 78e2;
        //    break;
        //case 1: // 10.4 kHz
        //    bw = 104e2;
        //    break;
        //case 2: // 15.6 kHz
        //    bw = 156e2;
        //    break;
        //case 3: // 20.8 kHz
        //    bw = 208e2;
        //    break;
        //case 4: // 31.2 kHz
        //    bw = 312e2;
        //    break;
        //case 5: // 41.4 kHz
        //    bw = 414e2;
        //    break;
        //case 6: // 62.5 kHz
        //    bw = 625e2;
        //    break;
        case 7: // 125 kHz
            bw = 125000;
            break;
        case 8: // 250 kHz
            bw = 250000;
            break;
        case 9: // 500 kHz
            bw = 500000;
            break;
        default:
            bw = 0;
            break;
        }

        // Symbol rate : time for one symbol (secs)
        double rs = bw / (1 << SX1276.Settings.LoRa.Datarate);
        double ts = 1 / rs;
        // time of preamble
        double tPreamble = (SX1276.Settings.LoRa.PreambleLen + 4.25) * ts;
        // Symbol length of payload and time
        double tmp = ceil((8 * pktLen - 4 * SX1276.Settings.LoRa.Datarate +
                             28 + 16 * SX1276.Settings.LoRa.CrcOn -
                             (SX1276.Settings.LoRa.FixLen ? 20 : 0)) /
                             (double)(4 * (SX1276.Settings.LoRa.Datarate -
                             ((SX1276.Settings.LoRa.LowDatarateOptimize > 0) ? 2 : 0)))) *
                             (SX1276.Settings.LoRa.Coderate + 4);
        double nPayload = 8 + ((tmp > 0) ? tmp : 0);
        double tPayload = nPayload * ts;
        // Time on air
        double tOnAir = tPreamble + tPayload;
        // return ms secs
        airtime = floor(tOnAir * 1000 + 0.999);
        break;
    default:
        airtime = 0;
        break;
    }
    return airtime;
}

void
SX1276Send(uint8_t *buffer, uint8_t size)
{
    uint32_t txTimeout = 0;

    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        SX1276.Settings.FskPacketHandler.NbBytes = 0;
        SX1276.Settings.FskPacketHandler.Size = size;

        if (SX1276.Settings.Fsk.FixLen == false){
            SX1276WriteFifo((uint8_t*)&size, 1);
        } else {
            SX1276Write(REG_PAYLOADLENGTH, size);
        }

        if ((size > 0) && (size <= 64)) {
            SX1276.Settings.FskPacketHandler.ChunkSize = size;
        } else{
            memcpy(RxTxBuffer, buffer, size);
            SX1276.Settings.FskPacketHandler.ChunkSize = 32;
        }

        // Write payload buffer
        SX1276WriteFifo(buffer, SX1276.Settings.FskPacketHandler.ChunkSize);
        SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
        txTimeout = SX1276.Settings.Fsk.TxTimeout;
        break;
    case MODEM_LORA:
        if (SX1276.Settings.LoRa.IqInverted == true) {
            SX1276Write(REG_LR_INVERTIQ, ((SX1276Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
            SX1276Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
        } else {
            SX1276Write(REG_LR_INVERTIQ, ((SX1276Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            SX1276Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
        }

        SX1276.Settings.LoRaPacketHandler.Size = size;

        // Initializes the payload size
        SX1276Write(REG_LR_PAYLOADLENGTH, size);

        // Full buffer used for Tx
        SX1276Write(REG_LR_FIFOTXBASEADDR, 0);
        SX1276Write(REG_LR_FIFOADDRPTR, 0);

        // FIFO operations can not take place in Sleep mode
        if ((SX1276Read(REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP) {
            SX1276SetStby();
            os_cputime_delay_usecs(1000);
        }
        // Write payload buffer
        SX1276WriteFifo(buffer, size);
        txTimeout = SX1276.Settings.LoRa.TxTimeout;
        break;
    }

    SX1276SetTx(txTimeout);
}

void
SX1276SetSleep(void)
{
    os_cputime_timer_stop(&RxTimeoutTimer);
    os_cputime_timer_stop(&TxTimeoutTimer);

    SX1276SetOpMode(RF_OPMODE_SLEEP);
    SX1276.Settings.State = RF_IDLE;
}

void
SX1276SetStby(void)
{
    os_cputime_timer_stop(&RxTimeoutTimer);
    os_cputime_timer_stop(&TxTimeoutTimer);

    SX1276SetOpMode(RF_OPMODE_STANDBY);
    SX1276.Settings.State = RF_IDLE;
}

void
SX1276SetRx(uint32_t timeout)
{
    bool rxcontinuous = false;

    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        rxcontinuous = SX1276.Settings.Fsk.RxContinuous;

        // DIO0=PayloadReady
        // DIO1=FifoLevel
        // DIO2=SyncAddr
        // DIO3=FifoEmpty
        // DIO4=Preamble
        // DIO5=ModeReady
        SX1276Write(REG_DIOMAPPING1, (SX1276Read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                                                                        RF_DIOMAPPING1_DIO1_MASK &
                                                                        RF_DIOMAPPING1_DIO2_MASK) |
                                                                        RF_DIOMAPPING1_DIO0_00 |
                                                                        RF_DIOMAPPING1_DIO1_00 |
                                                                        RF_DIOMAPPING1_DIO2_11);

        SX1276Write(REG_DIOMAPPING2, (SX1276Read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                                                                        RF_DIOMAPPING2_MAP_MASK) |
                                                                        RF_DIOMAPPING2_DIO4_11 |
                                                                        RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

        SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read(REG_FIFOTHRESH) & 0x3F;

        SX1276Write(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

        SX1276.Settings.FskPacketHandler.PreambleDetected = false;
        SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
        SX1276.Settings.FskPacketHandler.NbBytes = 0;
        SX1276.Settings.FskPacketHandler.Size = 0;
        break;
    case MODEM_LORA:
        if (SX1276.Settings.LoRa.IqInverted == true) {
            SX1276Write(REG_LR_INVERTIQ, ((SX1276Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
            SX1276Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
        } else {
            SX1276Write(REG_LR_INVERTIQ, ((SX1276Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            SX1276Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
        }

        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        if (SX1276.Settings.LoRa.Bandwidth < 9) {
            SX1276Write(REG_LR_DETECTOPTIMIZE, SX1276Read(REG_LR_DETECTOPTIMIZE) & 0x7F);
            SX1276Write(REG_LR_TEST30, 0x00);
            switch (SX1276.Settings.LoRa.Bandwidth) {
            case 0: // 7.8 kHz
                SX1276Write(REG_LR_TEST2F, 0x48);
                SX1276SetChannel(SX1276.Settings.Channel + 7810);
                break;
            case 1: // 10.4 kHz
                SX1276Write(REG_LR_TEST2F, 0x44);
                SX1276SetChannel(SX1276.Settings.Channel + 10420);
                break;
            case 2: // 15.6 kHz
                SX1276Write(REG_LR_TEST2F, 0x44);
                SX1276SetChannel(SX1276.Settings.Channel + 15620);
                break;
            case 3: // 20.8 kHz
                SX1276Write(REG_LR_TEST2F, 0x44);
                SX1276SetChannel(SX1276.Settings.Channel + 20830);
                break;
            case 4: // 31.2 kHz
                SX1276Write(REG_LR_TEST2F, 0x44);
                SX1276SetChannel(SX1276.Settings.Channel + 31250);
                break;
            case 5: // 41.4 kHz
                SX1276Write(REG_LR_TEST2F, 0x44);
                SX1276SetChannel(SX1276.Settings.Channel + 41670);
                break;
            case 6: // 62.5 kHz
                SX1276Write(REG_LR_TEST2F, 0x40);
                break;
            case 7: // 125 kHz
                SX1276Write(REG_LR_TEST2F, 0x40);
                break;
            case 8: // 250 kHz
                SX1276Write(REG_LR_TEST2F, 0x40);
                break;
            }
        } else {
            SX1276Write(REG_LR_DETECTOPTIMIZE, SX1276Read(REG_LR_DETECTOPTIMIZE) | 0x80);
        }

        rxcontinuous = SX1276.Settings.LoRa.RxContinuous;

        if (SX1276.Settings.LoRa.FreqHopOn == true) {
            SX1276Write(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                              //RFLR_IRQFLAGS_RXDONE |
                                              //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                              RFLR_IRQFLAGS_VALIDHEADER |
                                              RFLR_IRQFLAGS_TXDONE |
                                              RFLR_IRQFLAGS_CADDONE |
                                              //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                              RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=RxDone, DIO2=FhssChangeChannel
            SX1276Write(REG_DIOMAPPING1, (SX1276Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
        } else {
            SX1276Write(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                              //RFLR_IRQFLAGS_RXDONE |
                                              //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                              RFLR_IRQFLAGS_VALIDHEADER |
                                              RFLR_IRQFLAGS_TXDONE |
                                              RFLR_IRQFLAGS_CADDONE |
                                              RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                              RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=RxDone
            SX1276Write(REG_DIOMAPPING1, (SX1276Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
        }
        SX1276Write(REG_LR_FIFORXBASEADDR, 0);
        SX1276Write(REG_LR_FIFOADDRPTR, 0);
        break;
    }

    memset(RxTxBuffer, 0, (size_t)RX_BUFFER_SIZE);

    SX1276.Settings.State = RF_RX_RUNNING;
    if (timeout != 0) {
        os_cputime_timer_stop(&RxTimeoutTimer);
        os_cputime_timer_relative(&RxTimeoutTimer, timeout * 1000);
    }

    if (SX1276.Settings.Modem == MODEM_FSK) {
        SX1276SetOpMode(RF_OPMODE_RECEIVER);

        if (rxcontinuous == false) {
            rx_timeout_sync_delay =
                ceil((8.0 * (SX1276.Settings.Fsk.PreambleLen +
                             ((SX1276Read(REG_SYNCCONFIG) &
                                ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1.0) + 10.0) /
                      (double)SX1276.Settings.Fsk.Datarate) * 1e3) + 4;
            os_cputime_timer_stop(&RxTimeoutSyncWord);
            os_cputime_timer_relative(&RxTimeoutSyncWord, rx_timeout_sync_delay * 1000);
        }
    } else {
        if (rxcontinuous == true) {
            SX1276SetOpMode(RFLR_OPMODE_RECEIVER);
        } else {
            SX1276SetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
        }
    }
}

void
SX1276SetTx(uint32_t timeout)
{
    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        // DIO0=PacketSent
        // DIO1=FifoEmpty
        // DIO2=FifoFull
        // DIO3=FifoEmpty
        // DIO4=LowBat
        // DIO5=ModeReady
        SX1276Write(REG_DIOMAPPING1,
                    (SX1276Read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                     RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) |
                     RF_DIOMAPPING1_DIO1_01);

        SX1276Write(REG_DIOMAPPING2,
                    (SX1276Read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                     RF_DIOMAPPING2_MAP_MASK));
        SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read(REG_FIFOTHRESH) & 0x3F;
        break;
    case MODEM_LORA:
        if (SX1276.Settings.LoRa.FreqHopOn == true) {
            SX1276Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                              RFLR_IRQFLAGS_RXDONE |
                                              RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                              RFLR_IRQFLAGS_VALIDHEADER |
                                              //RFLR_IRQFLAGS_TXDONE |
                                              RFLR_IRQFLAGS_CADDONE |
                                              //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                              RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone, DIO2=FhssChangeChannel
            SX1276Write(REG_DIOMAPPING1, (SX1276Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
        } else {
            SX1276Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                              RFLR_IRQFLAGS_RXDONE |
                                              RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                              RFLR_IRQFLAGS_VALIDHEADER |
                                              //RFLR_IRQFLAGS_TXDONE |
                                              RFLR_IRQFLAGS_CADDONE |
                                              RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                              RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone
            SX1276Write(REG_DIOMAPPING1, (SX1276Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
        }
        break;
    }

    SX1276.Settings.State = RF_TX_RUNNING;
    os_cputime_timer_stop(&TxTimeoutTimer);
    os_cputime_timer_relative(&TxTimeoutTimer, timeout * 1000);
    SX1276SetOpMode(RF_OPMODE_TRANSMITTER);
}

void
SX1276StartCad(void)
{
    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        SX1276Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                    //RFLR_IRQFLAGS_CADDETECTED
                                   );

        // DIO3=CADDone
        SX1276Write(REG_DIOMAPPING1, (SX1276Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO3_MASK) | RFLR_DIOMAPPING1_DIO3_00);

        SX1276.Settings.State = RF_CAD;
        SX1276SetOpMode(RFLR_OPMODE_CAD);
        break;
    default:
        break;
    }
}

int16_t
SX1276ReadRssi(RadioModems_t modem)
{
    int16_t rssi;

    switch (modem) {
    case MODEM_FSK:
        rssi = -(SX1276Read(REG_RSSIVALUE) >> 1);
        break;
    case MODEM_LORA:
        if (SX1276.Settings.Channel > RF_MID_BAND_THRESH) {
            rssi = RSSI_OFFSET_HF + SX1276Read(REG_LR_RSSIVALUE);
        } else {
            rssi = RSSI_OFFSET_LF + SX1276Read(REG_LR_RSSIVALUE);
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void
SX1276Reset(void)
{
    //  Configure Reset pin as a GPIO Pin
    GLB_GPIO_Type pins[1];
    pins[0] = SX1276_NRESET;
    BL_Err_Type rc2 = GLB_GPIO_Func_Init(
        GPIO_FUN_SWGPIO,  //  Configure as GPIO 
        pins,             //  Pins to be configured
        sizeof(pins) / sizeof(pins[0])  //  Number of pins (1)
    );
    assert(rc2 == SUCCESS);    

    //  Configure Reset pin as a GPIO Output Pin (instead of GPIO Input)
    int rc = bl_gpio_enable_output(SX1276_NRESET, 0, 0);
    assert(rc == 0);

    //  Set Reset pin to Low
    rc = bl_gpio_output_set(SX1276_NRESET, 1);
    assert(rc == 0);

    // Wait 1 ms
    os_cputime_delay_usecs(1000);

    //  Configure Reset pin as a GPIO Input Pin, no pullup, no pulldown
    rc = bl_gpio_enable_input(SX1276_NRESET, 0, 0);
    assert(rc == 0);

    // Wait 6 ms
    os_cputime_delay_usecs(6000);
}

void
SX1276SetOpMode(uint8_t opMode)
{
#if SX1276_HAS_ANT_SW
    if (opMode == RF_OPMODE_SLEEP) {
        SX1276SetAntSwLowPower(true);
    } else {
        SX1276SetAntSwLowPower(false);
        if (opMode == RF_OPMODE_TRANSMITTER) {
            SX1276SetAntSw(1);
        } else {
            SX1276SetAntSw(0);
        }
    }
#endif
    SX1276Write(REG_OPMODE, (SX1276Read(REG_OPMODE) & RF_OPMODE_MASK) | opMode);
}

void
SX1276SetModem(RadioModems_t modem)
{

    if (SX1276.Settings.Modem == modem) {
        return;
    }

    SX1276.Settings.Modem = modem;
    switch (SX1276.Settings.Modem) {
    default:
    case MODEM_FSK:
        SX1276SetOpMode(RF_OPMODE_SLEEP);
        SX1276Write(REG_OPMODE, (SX1276Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);

        SX1276Write(REG_DIOMAPPING1, 0x00);
        SX1276Write(REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetOpMode(RF_OPMODE_SLEEP);
        SX1276Write(REG_OPMODE, (SX1276Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

        SX1276Write(REG_DIOMAPPING1, 0x00);
        SX1276Write(REG_DIOMAPPING2, 0x00);
        break;
    }
}

void
SX1276Write(uint16_t addr, uint8_t data)
{
    SX1276WriteBuffer(addr, &data, 1);
}

uint8_t
SX1276Read(uint16_t addr)
{
    uint8_t data;
    SX1276ReadBuffer(addr, &data, 1);
    return data;
}

void
SX1276WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    bl_gpio_output_set(RADIO_NSS, 0);

    hal_spi_tx_val(RADIO_SPI_IDX, addr | 0x80);
    for(i = 0; i < size; i++) {
        hal_spi_tx_val(RADIO_SPI_IDX, buffer[i]);
    }

    bl_gpio_output_set(RADIO_NSS, 1);
}

void
SX1276ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    bl_gpio_output_set(RADIO_NSS, 0);

    hal_spi_tx_val(RADIO_SPI_IDX, addr & 0x7f);
    for (i = 0; i < size; i++) {
        buffer[i] = hal_spi_tx_val(RADIO_SPI_IDX, 0);
    }

    bl_gpio_output_set(RADIO_NSS, 1);
}

void
SX1276WriteFifo(uint8_t *buffer, uint8_t size)
{
    SX1276WriteBuffer(0, buffer, size);
}

void
SX1276ReadFifo(uint8_t *buffer, uint8_t size)
{
    SX1276ReadBuffer(0, buffer, size);
}

void
SX1276SetMaxPayloadLength(RadioModems_t modem, uint8_t max)
{
    SX1276SetModem(modem);

    switch (modem) {
    case MODEM_FSK:
        if (SX1276.Settings.Fsk.FixLen == false) {
            SX1276Write(REG_PAYLOADLENGTH, max);
        }
        break;
    case MODEM_LORA:
        SX1276Write(REG_LR_PAYLOADMAXLENGTH, max);
        break;
    }
}

void
SX1276SetPublicNetwork(bool enable)
{
    printf("\r\nSX1276 %s public network\r\n", enable ? "enable" : "disable");
    SX1276SetModem(MODEM_LORA);
    SX1276.Settings.LoRa.PublicNetwork = enable;
    if (enable == true) {
        // Change LoRa modem SyncWord
        SX1276Write(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    } else {
        // Change LoRa modem SyncWord
        SX1276Write(REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }
}

uint32_t
SX1276GetWakeupTime(void)
{
    return SX1276GetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

/// Callback Function for Transmit, Receive and Sync Timeout
void SX1276OnTimeoutIrq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 %s timeout\r\n",
        (ev == &TxTimeoutTimer.ev) ? "transmit" :  //  Identify the timeout
        (ev == &RxTimeoutTimer.ev) ? "receive" :
        (ev == &RxTimeoutSyncWord.ev) ? "sync" :
        "unknown");
    switch (SX1276.Settings.State) {
    case RF_RX_RUNNING:
        if (SX1276.Settings.Modem == MODEM_FSK) {
            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1276Write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                       RF_IRQFLAGS1_PREAMBLEDETECT |
                                       RF_IRQFLAGS1_SYNCADDRESSMATCH);
            SX1276Write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

            if (SX1276.Settings.Fsk.RxContinuous == true) {
                // Continuous mode restart Rx chain
                SX1276Write(REG_RXCONFIG, SX1276Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                assert(rx_timeout_sync_delay != (uint32_t)-1);
                os_cputime_timer_stop(&RxTimeoutSyncWord);
                os_cputime_timer_relative(&RxTimeoutSyncWord, rx_timeout_sync_delay*1000);
            } else {
                SX1276.Settings.State = RF_IDLE;
                os_cputime_timer_stop(&RxTimeoutSyncWord);
            }
        }
        SX1276RxTimeout();
        break;
    case RF_TX_RUNNING:
        SX1276.Settings.State = RF_IDLE;
        SX1276TxTimeout();
        break;
    default:
        break;
    }
}

/// DIO0: Trigger for Packet Received and Packet Transmitted
void SX1276OnDio0Irq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 DIO0: Packet received / transmitted\r\n");
    int8_t snr;
    int16_t rssi;
    volatile uint8_t irqFlags = 0;

    switch (SX1276.Settings.State) {
    case RF_RX_RUNNING:
        //TimerStop(&RxTimeoutTimer);
        // RxDone interrupt
        switch (SX1276.Settings.Modem) {
        case MODEM_FSK:
            if (SX1276.Settings.Fsk.CrcOn == true) {
                irqFlags = SX1276Read(REG_IRQFLAGS2);
                if ((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK) {
                    // Clear Irqs
                    SX1276Write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                               RF_IRQFLAGS1_PREAMBLEDETECT |
                                               RF_IRQFLAGS1_SYNCADDRESSMATCH);
                    SX1276Write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                    os_cputime_timer_stop(&RxTimeoutTimer);

                    if (SX1276.Settings.Fsk.RxContinuous == false) {
                        os_cputime_timer_stop(&RxTimeoutSyncWord);
                        SX1276.Settings.State = RF_IDLE;
                    } else {
                        // Continuous mode restart Rx chain
                        SX1276Write(REG_RXCONFIG, SX1276Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                        assert(rx_timeout_sync_delay != (uint32_t)-1);
                        os_cputime_timer_stop(&RxTimeoutSyncWord);
                        os_cputime_timer_relative(&RxTimeoutSyncWord, rx_timeout_sync_delay*1000);
                    }

                    SX1276RxError();
                    SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                    SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                    SX1276.Settings.FskPacketHandler.NbBytes = 0;
                    SX1276.Settings.FskPacketHandler.Size = 0;
                    break;
                }
            }

            // Read received packet size
            if ((SX1276.Settings.FskPacketHandler.Size == 0) && (SX1276.Settings.FskPacketHandler.NbBytes == 0)) {
                if (SX1276.Settings.Fsk.FixLen == false) {
                    SX1276ReadFifo((uint8_t*)&SX1276.Settings.FskPacketHandler.Size, 1);
                } else {
                    SX1276.Settings.FskPacketHandler.Size = SX1276Read(REG_PAYLOADLENGTH);
                }
                SX1276ReadFifo(RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
                SX1276.Settings.FskPacketHandler.NbBytes += (SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
            } else {
                SX1276ReadFifo(RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
                SX1276.Settings.FskPacketHandler.NbBytes += (SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
            }

            if (SX1276.Settings.Fsk.RxContinuous == false) {
                SX1276.Settings.State = RF_IDLE;
                assert(rx_timeout_sync_delay != (uint32_t)-1);
                os_cputime_timer_stop(&RxTimeoutSyncWord);
                os_cputime_timer_relative(&RxTimeoutSyncWord, rx_timeout_sync_delay*1000);

            } else {
                // Continuous mode restart Rx chain
                SX1276Write(REG_RXCONFIG, SX1276Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
            }

            os_cputime_timer_stop(&RxTimeoutTimer);

            SX1276RxDone(RxTxBuffer, SX1276.Settings.FskPacketHandler.Size, SX1276.Settings.FskPacketHandler.RssiValue, 0);
            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;
            break;
        case MODEM_LORA:
            // Clear Irq
            SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

            irqFlags = SX1276Read(REG_LR_IRQFLAGS);
            if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR) {
                // Clear Irq
                SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                if (SX1276.Settings.LoRa.RxContinuous == false) {
                    SX1276.Settings.State = RF_IDLE;
                }
                os_cputime_timer_stop(&RxTimeoutTimer);

                SX1276RxError();
                break;
            }

            // The SNR sign bit is 1
            SX1276.Settings.LoRaPacketHandler.SnrValue = SX1276Read(REG_LR_PKTSNRVALUE);
            if (SX1276.Settings.LoRaPacketHandler.SnrValue & 0x80) {
                // Invert and divide by 4
                snr = ((~SX1276.Settings.LoRaPacketHandler.SnrValue + 1) & 0xFF) >> 2;
                snr = -snr;
            } else {
                // Divide by 4
                snr = (SX1276.Settings.LoRaPacketHandler.SnrValue & 0xFF) >> 2;
            }

            rssi = SX1276Read(REG_LR_PKTRSSIVALUE);
            if (snr < 0) {
                if (SX1276.Settings.Channel > RF_MID_BAND_THRESH) {
                    SX1276.Settings.LoRaPacketHandler.RssiValue =
                        RSSI_OFFSET_HF + rssi + (rssi >> 4) + snr;
                } else {
                    SX1276.Settings.LoRaPacketHandler.RssiValue =
                        RSSI_OFFSET_LF + rssi + (rssi >> 4) +snr;
                }
            } else {
                if (SX1276.Settings.Channel > RF_MID_BAND_THRESH) {
                    SX1276.Settings.LoRaPacketHandler.RssiValue =
                        RSSI_OFFSET_HF + rssi + (rssi >> 4);
                } else {
                    SX1276.Settings.LoRaPacketHandler.RssiValue =
                        RSSI_OFFSET_LF + rssi + (rssi >> 4);
                }
            }

            SX1276.Settings.LoRaPacketHandler.Size = SX1276Read(REG_LR_RXNBBYTES);
            SX1276Write(REG_LR_FIFOADDRPTR, SX1276Read(REG_LR_FIFORXCURRENTADDR));
            SX1276ReadFifo(RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size);

            if (SX1276.Settings.LoRa.RxContinuous == false) {
                SX1276.Settings.State = RF_IDLE;
            }
            os_cputime_timer_stop(&RxTimeoutTimer);

            SX1276RxDone(RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue);
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        os_cputime_timer_stop(&TxTimeoutTimer);
        // TxDone interrupt
        switch (SX1276.Settings.Modem) {
        case MODEM_LORA:
            // Clear Irq
            SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
            SX1276.Settings.State = RF_IDLE;
            SX1276TxDone();
            break;

        case MODEM_FSK:
        default:
            SX1276.Settings.State = RF_IDLE;
            SX1276TxDone();
            break;
        }
        break;
    default:
        break;
    }
}

/// DIO1: Trigger for Receive Timeout (Single Receive Mode only)
void SX1276OnDio1Irq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 DIO1: Receive timeout\r\n");
    switch (SX1276.Settings.State) {
    case RF_RX_RUNNING:
        switch (SX1276.Settings.Modem) {
        case MODEM_FSK:
            // FifoLevel interrupt
            // Read received packet size
            if ((SX1276.Settings.FskPacketHandler.Size == 0) && (SX1276.Settings.FskPacketHandler.NbBytes == 0)) {
                if (SX1276.Settings.Fsk.FixLen == false) {
                    SX1276ReadFifo((uint8_t*)&SX1276.Settings.FskPacketHandler.Size, 1);
                } else {
                    SX1276.Settings.FskPacketHandler.Size = SX1276Read(REG_PAYLOADLENGTH);
                }
            }

            if ((SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) > SX1276.Settings.FskPacketHandler.FifoThresh) {
                SX1276ReadFifo((RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes), SX1276.Settings.FskPacketHandler.FifoThresh);
                SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.FifoThresh;
            } else {
                SX1276ReadFifo((RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes), SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
                SX1276.Settings.FskPacketHandler.NbBytes += (SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
            }
            break;
        case MODEM_LORA:
            // Sync time out
            os_cputime_timer_stop(&RxTimeoutTimer);
            SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);
            SX1276.Settings.State = RF_IDLE;
            SX1276RxTimeout();
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        switch (SX1276.Settings.Modem) {
        case MODEM_FSK:
            // FifoEmpty interrupt
            if ((SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) > SX1276.Settings.FskPacketHandler.ChunkSize) {
                SX1276WriteFifo((RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes), SX1276.Settings.FskPacketHandler.ChunkSize);
                SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
            } else {
                // Write the last chunk of data
                SX1276WriteFifo(RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
                SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes;
            }
            break;
        case MODEM_LORA:
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

/// DIO2: Trigger for Change Channel (Spread Spectrum / Frequency Hopping)
void SX1276OnDio2Irq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 DIO2: Change channel\r\n");
    switch (SX1276.Settings.State) {
    case RF_RX_RUNNING:
        switch (SX1276.Settings.Modem) {
        case MODEM_FSK:
            if ((SX1276.Settings.FskPacketHandler.PreambleDetected == true) &&
                (SX1276.Settings.FskPacketHandler.SyncWordDetected == false)) {
                os_cputime_timer_stop(&RxTimeoutSyncWord);

                SX1276.Settings.FskPacketHandler.SyncWordDetected = true;

                SX1276.Settings.FskPacketHandler.RssiValue = -(SX1276Read(REG_RSSIVALUE) >> 1);

                SX1276.Settings.FskPacketHandler.AfcValue = (int32_t)(double)(((uint16_t)SX1276Read(REG_AFCMSB) << 8) |
                                                                       (uint16_t)SX1276Read(REG_AFCLSB)) *
                                                                       (double)FREQ_STEP;
                SX1276.Settings.FskPacketHandler.RxGain = (SX1276Read(REG_LNA) >> 5) & 0x07;
            }
            break;
        case MODEM_LORA:
            if (SX1276.Settings.LoRa.FreqHopOn == true) {
                // Clear Irq
                SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                if (RadioEvents.FhssChangeChannel != NULL) {
                    RadioEvents.FhssChangeChannel((SX1276Read(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                }
            }
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        switch (SX1276.Settings.Modem) {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            if (SX1276.Settings.LoRa.FreqHopOn == true) {
                // Clear Irq
                SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                if (RadioEvents.FhssChangeChannel != NULL) {
                    RadioEvents.FhssChangeChannel((SX1276Read(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                }
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

/// DIO3: Trigger for CAD Done.
/// CAD = Channel Activity Detection. We detect whether a Radio Channel 
/// is in use, by scanning very quickly for the LoRa Packet Preamble.
void SX1276OnDio3Irq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 DIO3: Channel activity detection\r\n");
    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if ((SX1276Read(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED) {
            // Clear Irq
            SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
            if (RadioEvents.CadDone != NULL) {
                RadioEvents.CadDone(true);
            }
        } else {
            // Clear Irq
            SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
            if (RadioEvents.CadDone != NULL) {
                RadioEvents.CadDone(false);
            }
        }
        break;
    default:
        break;
    }
}

/// DIO4: Unused (FSK only)
void SX1276OnDio4Irq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 DIO4: Unused\r\n");
    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        if (SX1276.Settings.FskPacketHandler.PreambleDetected == false) {
            SX1276.Settings.FskPacketHandler.PreambleDetected = true;
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

/// DIO5: Unused (FSK only)
void SX1276OnDio5Irq(struct ble_npl_event *ev)
{
    ////  This handler runs in the context of the FreeRTOS Background Application Task.
    ////  Previously this handler ran in the Interrupt Context.
    ////  So we are safe to call printf and SPI Functions now.
    printf("\r\nSX1276 DIO5: Unused\r\n");
    switch (SX1276.Settings.Modem) {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void SX1276RxDisable(void)
{
    printf("\r\nSX1276 Rx disabled\r\n");
    if (SX1276.Settings.Modem == MODEM_LORA) {
        /* Disable GPIO interrupts */
        SX1276RxIoIrqDisable();

        /* Disable RX interrupts */
        SX1276Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT_MASK       |
                                         RFLR_IRQFLAGS_RXDONE_MASK          |
                                         RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK |
                                         RFLR_IRQFLAGS_CADDONE_MASK         |
                                         RFLR_IRQFLAGS_CADDETECTED_MASK);

        /* Put radio into standby */
        SX1276SetStby();

        /* Clear any pending interrupts */
        SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT            |
                                     RFLR_IRQFLAGS_RXDONE               |
                                     RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK |
                                     RFLR_IRQFLAGS_CADDONE_MASK         |
                                     RFLR_IRQFLAGS_CADDETECTED_MASK);
        /* Enable GPIO interrupts */
        SX1276RxIoIrqEnable();
    }
}
