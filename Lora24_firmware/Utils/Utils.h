/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Display demo menus and manage touch sensor, header

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include <radio.h>
#include <array>

#ifndef UTILS_H
#define UTILS_H
#define DEBUG 0 /**<Enables debug mode on the serial stream*/
#if (DEVICE_ID == 0)
    #define VERBOSE 0  /**<Enables verbose mode on the serial stream*/
#else
    #define VERBOSE 0  /**<Enables verbose mode on the serial stream*/
#endif
#define PROFILING 0 /**<Enables timer stream on the serial output*/
#define debug_print(fmt, ...) \
            do { if (DEBUG) printf(fmt, ##__VA_ARGS__); } while (0)
#define verbose_print(fmt, ...) \
            do { if (VERBOSE) printf(fmt, ##__VA_ARGS__); } while (0)
#define timer_print(fmt, ...) \
            do { if (PROFILING) printf(fmt, ##__VA_ARGS__); } while (0)
/*!
 * \brief Used to display firmware version on TFT (Utilities menu)
 */
enum TestMode {
    NORMAL_SLAVE,
    ONE_FOR_ALL,
    RANDOM
};

#define SLAVE_MODE NORMAL_SLAVE

#define SERIAL_MAX_LENGTH 50
#define FIRMWARE_VERSION    ( ( char* )"v2.0.0" )
#define FIRMWARE_DATE       ( ( char* )"2020-01-14" )

/*!
 * \brief Define range of central frequency [Hz]
 */
#define DEMO_CENTRAL_FREQ_MIN       2400000000UL
#define DEMO_CENTRAL_FREQ_MAX       2483500000UL

/*!
 * \brief Define 3 preset central frequencies [Hz]
 */
#define DEMO_CENTRAL_FREQ_PRESET1   2402000000UL
#define DEMO_CENTRAL_FREQ_PRESET2   2450000000UL
#define DEMO_CENTRAL_FREQ_PRESET3   2480000000UL

/*!
 * \brief Define 5 preset ranging addresses
 */
#define DEMO_RNG_ADDR_1             0x00000001
#define DEMO_RNG_ADDR_2             0x00000002
#define DEMO_RNG_ADDR_3             0x00000003
#define DEMO_RNG_ADDR_4             0x00000004
#define DEMO_RNG_ADDR_5             0x00000005
#define DEMO_RNG_ADDR_6             0x00000006
#define DEMO_RNG_ADDR_7             0x00000007
#define DEMO_RNG_ADDR_8             0x00000008

/*!
 * \brief Define antenna selection for ranging
 */
#define DEMO_RNG_ANT_1              1
#define DEMO_RNG_ANT_0              2
#define DEMO_RNG_ANT_BOTH           0

/*!
 * \brief Define units for ranging distances
 */
#define DEMO_RNG_UNIT_CONV_M        1.0 // not used
#define DEMO_RNG_UNIT_CONV_YD       1.0936
#define DEMO_RNG_UNIT_CONV_MI       6.2137e-4
#define DEMO_RNG_UNIT_SEL_M         0
#define DEMO_RNG_UNIT_SEL_YD        1
#define DEMO_RNG_UNIT_SEL_MI        2

/*!
 * \brief Define min and max Tx power [dBm]
 */
#define DEMO_POWER_TX_MIN           -18
#define DEMO_POWER_TX_MAX           13


#define GUARD_TIME 0
/*!
 * \brief Time necessary for the slave to switch
 */
#define RANGING_SWITCH_TIME 10000
/*!
 * \brief Delay between the ranging slots of two slaves, in us
 */
#define INTER_SLAVE_DELAY 85000
/*!
 * \brief Default number of ranging request for each ranging protocol
 */
#define DEFAULT_RNG_REQUEST_COUNT 1
/*!
 * \brief Default size of the channel hop (e.g., if set to 3, proceeds directly to channel 7 after channel 4)
 */
#define DEFAULT_CHANNEL_HOP (CHANNELS /  DEFAULT_RNG_REQUEST_COUNT)

/*!
 * \brief Default spreading factor 
 */
#define DEFAULT_SF LORA_SF7

/*!
 * \brief Define min and max ranging channels count
 */
const uint16_t DEMO_RNG_CHANNELS_COUNT_MAX = 255;
const uint16_t DEMO_RNG_CHANNELS_COUNT_MIN = 10;

/*!
 * \brief Define min and max Z Score for ranging filtered results
 */
#define DEMO_RNG_ZSCORE_MIN         1
#define DEMO_RNG_ZSCORE_MAX         5

/*!
 * \brief Define min and max payload length for demo applications
 */
#define DEMO_MIN_PAYLOAD            12
#define DEMO_FLRC_MAX_PAYLOAD       127
#define DEMO_GFS_LORA_MAX_PAYLOAD   255

/*!
 * \brief Integer value of the ASCII char '0'. Used to convert char to integers
 */
#define ASCII_0 48

/*!
 * \brief Minimum spreading factor allowed int eh RF settings
 */
#define SF_MIN 5

/*!
 * \brief Maximum spreading factor allowed int eh RF settings
 */
#define SF_MAX 12

/*!
 * \brief Define current demo mode
 */
enum DemoMode
{
    MASTER = 0,
    SLAVE
};

/*!
 * \brief Status of ranging distance
 */
enum RangingStatus
{
    RNG_INIT = 0,
    RNG_PROCESS,
    RNG_VALID,
    RNG_TIMEOUT,
    RNG_PER_ERROR
};

enum Event_t
{
    NOTHING_PENDING,
    TRANSCEIVER_READY,
    TX_DONE,
    RX_DONE,
    REQUEST_RECEIVED,
    REQUEST_SENT,
    RX_FAILED,
    TX_FAILED,
    TX_TIMEOUT,
    RX_TIMEOUT,
    PROBING_INIT,
    PROBING_COMPLETED,
    PROBING_FAILED, 
    RANGING_TIMER,
    PROBING_DONE,
    RANGING_TIMEOUT,
    RANGING_COMPLETED, 
    EXIT_FSM
};

template<typename T, int sz>
inline constexpr int size(T(&)[sz])
{
    return sz;
}

/** Serial commands **/
/*!
 * \brief Class for serial commands; each command has a 1-char code sent as the header of the command. 
 */
class SerialCommand
{
public:
    SerialCommand(char code, const char *description = "") : code(code), description(description) {}
    char code;
    const char *description;
};

/** FSM **/
/*!
 * \brief Represents a Finite State Machine state.
 */
class FSMState
{
public:
    FSMState(const char *name, void (&func)(FSMState *)) : name(name), func(func) {}
    bool isTimerEnabled = false;
    const char *getName() const
    {
        return (this->name);
    }
    void (&func)(FSMState *);
    Event_t toggle() {
        this->func(this);
        return(this->lastEvent);
    }
    Event_t lastEvent = NOTHING_PENDING;
    void clear() {
        lastEvent = NOTHING_PENDING;
    }
    uint32_t getElapsedTime() 
    {
        uint32_t currentTime = HAL_GetTick(), lastTime = this->timer;
        this->timer = currentTime;
        return(currentTime - lastTime);
    }

    void resetTimer() {
        this->timer = HAL_GetTick();
    }
  
private:
    const char *name;
    uint32_t timer;  
};

class FSMTransition
{
public:
    FSMTransition(const Event_t onEvent, FSMState &from, FSMState &to) : onEvent(onEvent), from(from), to(to) {}
    const Event_t onEvent;
    FSMState &from;
    FSMState &to;
};

class FSM
{
public:
    FSM(FSMState &inputState, const FSMTransition * transitionsMatrix, const int transitionsMatrixSize, FSMState * const idle = NULL): inputState(inputState), transitionsMatrix(transitionsMatrix), transitionsMatrixSize(transitionsMatrixSize), idle(idle) {}
    FSMState &inputState;
    const FSMTransition * transitionsMatrix;
    const int transitionsMatrixSize;
    Event_t lastEvent;
    void run();
    bool findTransition(Event_t event, FSMState *from, FSMState*& to);
private:
    FSMState *currentState = &inputState;
    FSMState *idle;
};

/*!
 * \brief List of states for demo state machine
 */
enum DemoInternalStates {
    APP_IDLE, // nothing to do (or wait a radio interrupt)
    APP_RANGING_DONE,
    APP_RANGING_TIMEOUT,
    APP_RANGING_CONFIG,
    APP_RANGING_REQUEST,
    APP_RNG,
    SEND_PING_MSG,
    SEND_PONG_MSG,
    APP_MSG_RECEIVED, // Rx done
    APP_RX_TIMEOUT,   // Rx timeout
    APP_RX_ERROR,     // Rx error
    APP_ENABLE_RX,    // Tx done
    APP_TX_TIMEOUT,   // Tx error
    PER_TX_START,     // PER master
    PER_RX_START      // PER slave
};

/*!
 * \brief Demo Settings structure of Eeprom structure
 */
typedef struct
{
    uint8_t Entity;              // Master or Slave
    uint8_t HoldDemo;            // Put demo in hold status
    uint8_t AntennaSwitch;       // Witch antenna connected
    uint32_t Frequency;          // Demo frequency
    int8_t TxPower;              // Demo Tx power
    uint8_t RadioPowerMode;      // Radio Power Mode [0: LDO, 1:DC_DC]
    uint8_t PayloadLength;       // Demo payload length
    uint8_t ModulationType;      // Demo modulation type (LORA, GFSK, FLRC)
    uint8_t ModulationParam1;    // Demo Mod. Param1 (depend on modulation type)
    uint8_t ModulationParam2;    // Demo Mod. Param2 (depend on modulation type)
    uint8_t ModulationParam3;    // Demo Mod. Param3 (depend on modulation type)
    uint8_t PacketParam1;        // Demo Pack. Param1 (depend on packet type)
    uint8_t PacketParam2;        // Demo Pack. Param2 (depend on packet type)
    uint8_t PacketParam3;        // Demo Pack. Param3 (depend on packet type)
    uint8_t PacketParam4;        // Demo Pack. Param4 (depend on packet type)
    uint8_t PacketParam5;        // Demo Pack. Param5 (depend on packet type)
    uint8_t PacketParam6;        // Demo Pack. Param6 (depend on packet type)
    uint8_t PacketParam7;        // Demo Pack. Param7 (depend on packet type)
    uint32_t MaxNumPacket;       // Demo Max Num Packet for PingPong and PER
    uint16_t TimeOnAir;          // Computed time on air of the current packet
    uint32_t CntPacketTx;        // Tx packet transmitted
    uint32_t CntPacketRxOK;      // Rx packet received OK
    uint32_t CntPacketRxOKSlave; // Rx packet received OK (slave side)
    uint32_t CntPacketRxKO;      // Rx packet received KO
    uint32_t CntPacketRxKOSlave; // Rx packet received KO (slave side)
    uint16_t RxTimeOutCount;     // Rx packet received KO (by timeout)
    double RngDistance;          // Distance measured by ranging demo
    double RngRawDistance;       // Uncorrected measured distance [m]
    uint32_t RngAddress;         // Ranging Address
    uint16_t RngFullScale;       // Full range of measuring distance (Ranging)
    uint8_t RngRequestCount;     // Ranging Request Count
    uint8_t RngUnit;             // Ranging distance unit [m]/[mi]
    uint8_t RngStatus;           // Status of ranging distance
    double RngFei;               // Ranging Frequency Error Indicator
    uint8_t RngAntenna;          // Ranging antenna selection
    double RngFeiFactor;         // Ranging frequency correction factor
    uint16_t RngReqDelay;        // Time between ranging request
    uint16_t RngCalib;           // Ranging Calibration
    uint8_t RFU;                 // -------------------------
    int8_t RssiValue;            // Demo Rssi Value
    int8_t SnrValue;             // Demo Snr Value (only for LORA mod. type)
}DemoSettings_t;

/*!
 * \brief Define freq offset for config central freq in "Radio Config Freq" menu
 */
enum FreqBase
{
    FB1     = 1,            //   1 Hz
    FB10    = 10,           //  10 Hz
    FB100   = 100,          // 100 Hz
    FB1K    = 1000,         //   1 kHz
    FB10K   = 10000,        //  10 kHz
    FB100K  = 100000,       // 100 kHz
    FB1M    = 1000000,      //   1 MHz
    FB10M   = 10000000      //  10 MHz
};
/*!
 * \brief List of events for Finite State Machine transitions
 */
// enum Event_t 
// {
//     MSG_RECEIVED,
//     TX_DONE,
//     REQ_TIMEOUT,
//     RNG_TIMEOUT
// }
/*!
 * \brief Function to be executed on Radio Tx Done event
 */

/* globals */
extern int total_slaves;
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRangingDone( IrqRangingCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */

/*!
 * \brief Init RAM copy of Eeprom structure and init radio with it.
 */
void InitDemoApplication( void );

/*!
 * \brief Init vars of demo and fix APP_IDLE state to demo state machine.
 */
void StopDemoApplication( void );

/*!
 * \brief Run Demo in sleep mode.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoSleepMode( void );

/*!
 * \brief Run Demo in standby RC mode.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoStandbyRcMode( void );

/*!
 * \brief Run Demo in standby XOSC mode.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoStandbyXoscMode( void );

/*!
 * \brief Run Demo Tx in continuous mode without modulation.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoTxCw( void );

/*!
 * \brief Run Demo Tx in continuous modulation.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoTxContinuousModulation( void );

/*!
 * \brief Run demo PingPong.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoApplicationPingPong( void );

/*!
 * \brief Compute payload of Rx frame and update current counts and indicators.
 *
 * \param [in]  buffer        buffer with frame to compute
 * \param [in]  buffersize    size of frame data in the buffer
 */
void ComputePingPongPayload( uint8_t *buffer, uint8_t bufferSize );

/*!
 * \brief Run demo PER.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoApplicationPer( void );

/*!
 * \brief Compute payload of Rx frame and update current counts and indicators.
 *
 * \param [in]  buffer        buffer with frame to compute
 * \param [in]  buffersize    size of frame data in the buffer
 */
void ComputePerPayload( uint8_t *buffer, uint8_t bufferSize );

/*!
 * \brief Run ranging demo.
 *
 * \retval      demoStatusUpdate    page refresh status ( >0 : refresh)
 */
uint8_t RunDemoApplicationRanging( void );


void LedBlink( void );

/*!
 * \brief Writes 3 lines on display, with current radio parameters.
 *
 * \param [in]  page          Current page to choose what to display.
 */

/*!
 * \brief Return text with current frame type.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuRadioFrameType( void );

/*!
 * \brief Return text with current Radio Modulation Parameters1.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioModulationParameters1( void );

/*!
 * \brief Return text with current Radio Modulatio nParameters2.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioModulationParameters2( void );

/*!
 * \brief Return text with current Radio Modulation Parameters3.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioModulationParameters3( void );

/*!
 * \brief Return text with current Radio Frequency.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioFrequency( void );
/*!
 * \brief Update the radio frequency displayed on the screen.
 *
 * \param [in]  freq          freq in Hz
 */
void UpdateRadioFrequency( unsigned long freq );

/*!
 * \brief Return text with current Radio Frequency [GHz] format #.###.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioFrequencyGHz( void );

/*!
 * \brief Return text with current Radio Freq Base.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioFreqBase( void );

/*!
 * \brief Return text with Radio preset Frequency 1.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioFreqBasePS1( void );

/*!
 * \brief Return text with Radio preset Frequency 2.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioFreqBasePS2( void );

/*!
 * \brief Return text with Radio preset Frequency 3.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioFreqBasePS3( void );

/*!
 * \brief Return text with current Radio Tx Power.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioTxPower( void );

/*!
 * \brief Return text with current Radio Payload Length.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRadioPayloadLength( void );

/*!
 * \brief Return text with Max Num Packet.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoMaxNumPacket( void );

/*!
 * \brief Return text with current Demo Num Sent Packet.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoNumSentPacket( void );

/*!
 * \brief Return text with current Rx frame Ok count.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoRxOk( void );

/*!
 * \brief Return text with current Rx frame Ko count.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoRxKo( void );

/*!
 * \brief Return text with current Rx frame Ok (on slave side) count.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoRxOkSlave( void );

/*!
 * \brief Return text with current Rx frame Ko (on slave side) count.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoRxKoSlave( void );

/*!
 * \brief Return text with current Result PerCent1, format ###.##.
 *
 * \param [in]  value         value to compute in [%]
 * \param [in]  reference     reference value for % computation
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoResultPerCent1( uint32_t value, uint32_t reference );

/*!
 * \brief Return text with current Result PerCent2, format ###.##.
 *
 * \param [in]  value         value to compute in [%]
 * \param [in]  reference     reference value for % computation
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoResultPerCent2( uint32_t value, uint32_t reference );

/*!
 * \brief Return text with current Rssi.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoRssi( void );

/*!
 * \brief Return text with current Snr.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoSnr( void );

/*!
 * \brief Return text with current Antenna Setting.
 *
 * \retval      text          Pointer on text to display
 */
char* GetAntennaSetting( void );

/*!
 * \brief Return text with Total Packet for the test.
 *
 * \retval      text          Pointer on text to display
 */
char* GetTotalPackets( void );

/*!
 * \brief Return text with current GPS Time.
 *
 * \retval      text          Pointer on text to display
 */
char* GetGpsTime( void );

/*!
 * \brief Return text with current GPS Position.
 *
 * \retval      text          Pointer on text to display
 */
char* GetGpsPos( void );

/*!
 * \brief Return text with current Proximity Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetProximityValue( void );

/*!
 * \brief Return text with current Radio Power Mode Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetMenuDemoRadioPowerMode( void );

/*!
 * \brief Return text with current Frequency Error Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetFrequencyError( void );

/*!
 * \brief Return text with current Ranging Channels Successfully Done Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRngChannelsOk( void );

/*!
 * \brief Return text with current Ranging Request Count Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRangingRequestCount( void );

/*!
 * \brief Return text with current Ranging Address Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRangingAddress( void );

/*!
 * \brief Return text with current Ranging Antenna Value.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRangingAntenna( void );

/*!
 * \brief Return text with current Ranging Distance Unit.
 *
 * \retval      text          Pointer on text to display
 */
char* GetRangingUnit( void );

#endif // UTILS_H

