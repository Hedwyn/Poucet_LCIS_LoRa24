/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Display demo menus and manage touch sensor.

Maintainer: Gregory Cristian & Gilbert Menth
*/
#include <math.h>
#include <stdio.h>
#include <FreqLUT.h>

#include <RangingCorrection.h>
#include <sx1280-hal.h>
#include <Utils.h>
#include "mbed.h"
#include "Eeprom.h"
#include "main.h"
#include "app_config.h"


#if defined(HAS_GPS_SENSOR)
#include "GpsMax7.h"

/*!
 * \brief Pointer to GPS Data, will be updated when Max7GpsgetData( ) is called.
 */
GpsStruct *thisGps;
#endif

#if defined(HAS_PROXIMITY_SENSOR)
#include "SX9306.h"
#endif


/*!
 * \brief Maximum character count on the same line with function DrawText
 * This include needed space for the outline (at the begin and the end of line).
 * MAX_CHAR_PER_BTN is the same, but for a button.
 */
#define MAX_CHAR_PER_LINE       28

#define SCALE_LINE_COUNT        5
#define RANGING_FULLSCALE_MIN   10
#define RANGING_FULLSCALE_MAX   30000


/*!
 * \brief DrawText( ) need char* to display a text. It can display until
 * 3 different texts on the same line. To avoid recursive use of the same
 * ressource, 3 temporary string are declared.
 */
char StringText[MAX_CHAR_PER_LINE + 1];  // don't forget the /0 (end of string)
char StringText2[MAX_CHAR_PER_LINE + 1];
char StringText3[MAX_CHAR_PER_LINE + 1];


/*!
 * \brief In "Radio Config Freq", we can update the central frequency. To avoid
 * keyboard, we use (+) et (-) keys. CurrentFreqBase is the offset to be applied
 * when we increase or decrease the frequency.
 */
static FreqBase CurrentFreqBase = FB100K;
bool echoedOnce = false;

RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    &OnRangingDone,   // rangingDone
    NULL,             // cadDone
};
/*!
 * \brief Define IO and callbacks for radio
 * 			      mosi,miso,sclk,nss,busy,dio1,dio2,dio3,rst,callbacks
 */


#ifdef NUCLEO_L432KC
    SX1280Hal Radio( D11, D12, D13, D10, D7, D9, NC, NC, A0, &Callbacks ); // L432_KC
#else // NUCLEO_476RG
    SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, &Callbacks );
#endif

/*!
 * \brief Control the Antenna Diversity switch
 */
DigitalOut ANT_SW( A3 );

/*!
 * \brief Tx LED toggling on transmition success
 */
DigitalOut TX_LED( A4 );

/*!
 * \brief Rx LED toggling on reception success
 */
DigitalOut RX_LED( A5 );

/*!
 * \brief Mask of IRQs
 */
uint16_t IrqMask = 0x0000;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
ModulationParams_t ModulationParams;

/** Serial commands **/
array<SerialCommand, 3> commandsList = 
{
    SerialCommand('H', "Hold state. This device will go to sleep until further serial notice\r\n"),
    SerialCommand('S', "Switch Spreading Factor(SF).\r\n"),
    SerialCommand('h', "Serial manual. Displays help menu\r\n")
};

int channel_hop = DEFAULT_CHANNEL_HOP;
// int channel_hop = 1;


#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define SERIAL_LINE_MAX_LENGTH 50
/*!
 * \brief Contains explicit strings for each state that are printed in debug mode.
 * The size of the string array is indexed on the last state defined in the DemoInternalStates enum
 */
const char states[][SERIAL_LINE_MAX_LENGTH] =
{
    "Idle",
    "Ranging Done",
    "Ranging Timeout",
    "Ranging Config",
    "Ranging Request",
    "Ranging",
    "Sending Ping",
    "Sending Pong",
    "Message received",
    "Reception timeout",
    "Reception error",
    "Starting reception",
    "Tranmission timeout",
    "PER Transmission start"
    "Per Reception start"
};
char *previous_state = (char *) states;

void onSerialReadComplete(int code) 
{
    printf("Serial read complete %d", code);
}

void onSerialMsg() {
    uint8_t buffer[50];
    int c;
    char code[10];
    printf("Serial message received\r\n");
    // s.scanf("%d", &c);
    // s.read(buffer, 10, onSerialReadComplete, SERIAL_EVENT_RX_COMPLETE, '\n');
    // s.read(buffer, 3);
    printf("code %d\r\n", c);
}



void processSerial() {
    char msg;
    uint8_t buffer[SERIAL_MAX_LENGTH];
    int bytes;
    if (pc.readable()) {
        bytes = pc.read(buffer, SERIAL_MAX_LENGTH);
        printf("Received %d bytes: %s\r\n", bytes, (char *) buffer);
        auto it = commandsList.begin();
        bool commandFound = false;
        while (it != commandsList.end() )
        {
            
            if (it->code == buffer[0]) 
            {
                printf("%s\r\n", it->description);
            }
            it++;
        }
        // usbReceive();
        // printf("Received %c \r\n", msg);
        // s.read(buffer, 3)
        /* echoing */
        // pc.gets(msg, 10);
        // printf("msg %s \r\n", msg);
        // printf("_Received: %s %c\r\n", msg, msg[0] );
    //     bytes = strlen(msg);
    //     switch (msg[0] ) {
    //         case 'H':
    //             printf("Hold command recived. Going to sleep \r\n");
    //             break;
            
    //         case 'S':
    //             if ( (bytes > 1) && (msg[2] - ASCII_0 >= SF_MIN) && (msg[2] - ASCII_0 >= SF_MAX) ) 
    //             {
    //                 ModulationParams.Params.LoRa.SpreadingFactor = int_to_sf(msg[2] - ASCII_0);
    //                 printf("Spreading factor changed to SF%d", msg[2] - ASCII_0);
    //             }
    //             else 
    //             {
    //                 printf("Cannot change spreading factor: the submitted spreading factor code is unvalid. Please provide values between %d and %d.", SF_MIN, SF_MAX);
    //             }
    //     }
    }

}

void FSM::run()
{
    /* Starting from the input state */
    Event_t lastEvent = NOTHING_PENDING;
    FSMState *nextState = this->currentState;

    while (lastEvent != EXIT_FSM)
    {
        this->currentState = nextState;
        if (DEBUG && (Radio.GetIrqStatus()  != 0)) 
            debug_print("Pending IRQ: %d\r\n", Radio.GetIrqStatus() );
        Radio.ProcessIrqs();
        lastEvent = this->currentState->toggle();
        /* If an idle state has been defined, coming back to Idle state whenever nothing is pending */
        if ((this->idle != NULL) && (lastEvent == NOTHING_PENDING))
        {
            nextState = this->idle;
        }
        else
        {
            /* finding the transition corresponding to the submitted event */
            /* If the transition is not found, exitting FSM */
            if (!this->findTransition(lastEvent, this->currentState, nextState))
            {
                printf("Transition not found in FSM: %d on %s  Exitting \r\n", (int) lastEvent, this->currentState->getName());
                lastEvent = EXIT_FSM; 
            }
        }
        if (nextState != this->currentState)
        {
            verbose_print("Switching to {%s} after %u ms. Last occurence was %u ms ago\r\n",  nextState->getName(), this->currentState->getElapsedTime(), nextState->getElapsedTime());
            // verbose_print("Switching to %s\r\n", nextState->getName());
        }
        this->currentState->clear();
    }
}

bool FSM::findTransition(Event_t event, FSMState *from, FSMState*& to) {
    bool found = false;
    int i = 0;
    // verbose_print("Looking for transition from %s on %d\r\n", from->getName(), (int) event);
    while ( (i < this->transitionsMatrixSize) && !found) {
        if (this->transitionsMatrix[i].onEvent == event) {
            if ( &(this->transitionsMatrix[i].from) == from ) 
            {
                found = true;
                to = &(this->transitionsMatrix[i].to);
                debug_print("Found transition from %s to %s on %d\r\n", from->getName(), to->getName(), (int) event);
            }
        }
        i++;
    }
    
    return(found);
}

/* Master FSM states toggle functions */
void onRangingConfig(FSMState *state);
void onRangingRequest(FSMState *state);
void onRanging(FSMState *state);
void onRangingDone(FSMState *state);
void onMsgReceived(FSMState *state);
void onEnableRx(FSMState *state);
void onIdle(FSMState *state);

/* Slave FSM toggle functions */
void onWaitRangingRequest(FSMState *state);
void onRequestReceived(FSMState *state);
void onPrepareRanging(FSMState *state);
void onRangingRx(FSMState *state);

/* Master FSM states declaration */
FSMState stateRangingConfig =  FSMState("Ranging Config", onRangingConfig);
FSMState stateRangingRequest = FSMState("Ranging Request", onRangingRequest);
FSMState stateRanging =        FSMState("Ranging", onRanging);
FSMState stateRangingDone =    FSMState("Ranging done", onRangingDone);
FSMState stateMsgReceived =    FSMState("Message Received", onMsgReceived);
FSMState stateEnableRx =       FSMState("Enable Reception", onEnableRx);
FSMState stateIdle =           FSMState("Idle", onIdle);

/* Master transitions matrix */
const FSMTransition masterTransitionsMatrix[] = 
{
    FSMTransition(TRANSCEIVER_READY,     stateRangingConfig,        stateRangingRequest),
    FSMTransition(PROBING_DONE,          stateRangingDone,          stateRanging),
    FSMTransition(PROBING_INIT,          stateMsgReceived,          stateIdle),
    FSMTransition(RX_FAILED,             stateMsgReceived,          stateRangingConfig),
    FSMTransition(RANGING_COMPLETED,     stateRanging,              stateRangingRequest),
    FSMTransition(RX_TIMEOUT,            stateIdle,                 stateRangingRequest),
    FSMTransition(REQUEST_SENT,          stateIdle,                 stateEnableRx),
    FSMTransition(REQUEST_RECEIVED,      stateIdle,                 stateMsgReceived),
    FSMTransition(PROBING_COMPLETED,     stateIdle,                 stateRangingDone),
    FSMTransition(RANGING_TIMER,         stateIdle,                 stateRanging),
    FSMTransition(RX_FAILED,             stateIdle,                 stateRangingRequest),
    FSMTransition(RX_TIMEOUT,            stateIdle,                 stateRangingRequest),
    FSMTransition(RANGING_TIMEOUT,       stateIdle,                 stateRangingRequest),
    FSMTransition(TX_TIMEOUT,            stateIdle,                 stateRangingRequest),
};

/* Master FSM instanciation */
FSM fsmMaster = FSM(stateRangingConfig, masterTransitionsMatrix, (const int) size(masterTransitionsMatrix), &stateIdle);

/* Slave FSM states declaration */
FSMState stateWaitRangingRequest = FSMState("Waiting ranging request", onWaitRangingRequest);
FSMState stateRequestReceived = FSMState("Request received", onRequestReceived);
FSMState statePrepareRanging = FSMState("Preparing ranging", onPrepareRanging);
FSMState stateRangingRx = FSMState("Ranging", onRangingRx);

/* Slave transitions matrix */
const FSMTransition slaveTransitionsMatrix[] = 
{
    FSMTransition(TRANSCEIVER_READY,     stateRangingConfig,        stateWaitRangingRequest),
    FSMTransition(PROBING_DONE,          stateRangingDone,          stateRangingRx),
    FSMTransition(RANGING_COMPLETED,     stateRangingRx,            stateWaitRangingRequest),
    FSMTransition(RX_TIMEOUT,            stateIdle,                 stateWaitRangingRequest),
    FSMTransition(REQUEST_RECEIVED,      stateIdle,                 stateRequestReceived),
    FSMTransition(RX_FAILED,             stateRequestReceived,      stateWaitRangingRequest),
    FSMTransition(REQUEST_SENT,          stateIdle,                 statePrepareRanging),
    FSMTransition(PROBING_COMPLETED,     stateIdle,                 stateRangingDone),
    FSMTransition(RANGING_TIMER,         stateIdle,                 stateRangingRx),
    FSMTransition(RX_FAILED,             stateIdle,                 stateWaitRangingRequest),
    FSMTransition(RX_TIMEOUT,            stateIdle,                 stateWaitRangingRequest),
    FSMTransition(RANGING_TIMEOUT,       stateIdle,                 stateWaitRangingRequest),
};
/* Master FSM instanciation */
FSM fsmSlave = FSM(stateRangingConfig, slaveTransitionsMatrix, (const int) size(slaveTransitionsMatrix), &stateIdle);

/* FSM globals */
static time_t endwait;
static time_t start;
static uint32_t rangingClock;
static int ranging_counter = 0;

double t0 =       -0.016432807883697;                         // X0
double t1 =       0.323147003165358;                          // X1
double t2 =       0.014922061351196;                          // X1^2
double t3 =       0.000137832006285;                          // X1^3
double t4 =       0.536873856625399;                          // X2
double t5 =       0.040890089178579;                          // X2^2
double t6 =       -0.001074801048732;                         // X2^3
double t7 =       0.000009240142234;                          // X2^4

const time_t rx_timeout =  3.; // end loop after this time has elapsed
bool lastRangingOK = false;

double p[8] = { 0,
                -4.1e-9,
                1.03e-7,
                1.971e-5,
                -0.00107,
                0.018757,
                0.869171,
                3.072450 };

/*!
 * \brief Defines the local payload buffer size
 */
#define BUFFER_SIZE                     255

/*!
 * \brief Defines the size of the token defining message type in the payload
 *        cf. above.
 */
#define PINGPONG_SIZE                   4
#define PER_SIZE                        3

/*!
 * \brief Define time used in PingPong demo to synch with cycle
 * RX_TX_INTER_PACKET_DELAY is the free time between each cycle (time reserve)
 */
#define RX_TX_INTER_PACKET_DELAY        150  // ms
#define RX_TX_TRANSITION_WAIT           15   // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE            RADIO_TICK_SIZE_1000_US

#define RNG_TIMER_MS                    384 // ms / default : 384
#define RNG_COM_TIMEOUT                 80 // ms
#define SLAVE_POLL_TIME                 1000  

/*!
 * \brief Maximum length in bytes of a master or slave ID
 */
#define ID_MAX_LEN 8 

/*!
 * \brief Ranging raw factors
 *                                  SF5     SF6     SF7     SF8     SF9     SF10
 */
const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };


/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

static uint8_t CurrentChannel;
static uint16_t MeasuredChannels;
int RngResultIndex;
double RawRngResults[DEMO_RNG_CHANNELS_COUNT_MAX];
double RssiRng[DEMO_RNG_CHANNELS_COUNT_MAX];

// /** flags **/
// enum RXTX_flag {NOTHING_PENDING, TX_DONE, RX_DONE, RX_FAILED, TX_TIMEOUT, RX_TIMEOUT };
Event_t rangingFlag = NOTHING_PENDING;

// enum Ranging_flag {NO_RANGING, RANGING_COMPLETED, PROBING_FAILED, RANGING_TIMEOUT};


/*!
 * \brief Flag to indicate if the demo is already running
 */
static bool DemoRunning = false;

/*!
 * \brief Flag holding the current internal state of the demo application
 */
static uint8_t DemoInternalState = APP_IDLE;

/*!
 * \brief Ticker for master to synch Tx frames. Flags for PER and PingPong demo
 * for Synch TX in cycle.
 */
Ticker SendNextPacket;
static bool SendNext = false;

/*!
 * \brief Ticker for slave to synch Tx frames. Flags for PER and PingPong demo
 * for Synch RX in cycle.
 */
Ticker ReceiveNextPacket;
static bool ReceiveNext = false;

/*!
 * \brief Hold last Rx packet number to compute PER in PER and PingPong demo
 */
static uint32_t PacketRxSequence = 0;
static uint32_t PacketRxSequencePrev = 0;

const char MASTER_IDS[][ID_MAX_LEN] = 
{
    "01",
};

const char SLAVE_IDS[][ID_MAX_LEN] = {
    "01",
    "02",
    "03",
    "04",
};

void SetAntennaSwitch( void );
void LedBlink( void );
void InitializeDemoParameters( uint8_t modulation );
uint16_t GetTimeOnAir( uint8_t modulation );
void SendNextPacketEvent( void );
void ReceiveNextPacketEvent( void );
uint8_t CheckDistance( void );

void switchOffLeds () {
    RX_LED = 0;
    TX_LED = 0;
}


void getRangingParameters() {
    // s.attach(onSerialMsg);
    // memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR,      1 );
    // memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,           1 );
    // memcpy( &( ModulationParams.Params.LoRa.CodingRate ),      Eeprom.Buffer + MOD_RNG_CODERATE_EEPROM_ADDR,     1 );
    // memcpy( &( PacketParams.Params.LoRa.PreambleLength ),      Eeprom.Buffer + PAK_RNG_PREAMBLE_LEN_EEPROM_ADDR, 1 );
    memcpy( &( PacketParams.Params.LoRa.HeaderType ),          Eeprom.Buffer + PAK_RNG_HEADERTYPE_EEPROM_ADDR,   1 );
    memcpy( &( PacketParams.Params.LoRa.Crc ),                 Eeprom.Buffer + PAK_RNG_CRC_MODE_EEPROM_ADDR,     1 );
    memcpy( &( PacketParams.Params.LoRa.InvertIQ ),            Eeprom.Buffer + PAK_RNG_IQ_INV_EEPROM_ADDR,       1 );
}

void configureRangingRequest() {
    if (Eeprom.EepromData.DemoSettings.Entity == MASTER) 
    {
        slave_index = (slave_index + 1) % total_slaves;
        Eeprom.EepromData.DemoSettings.RngAddress = RNG_ADDR_LIST[slave_index];
        
        wait_us(INTER_SLAVE_DELAY);
        printf("Sending request to Slave %d \r\n", slave_index);
        lastRangingOK = true;
    }

    if ( (SLAVE_MODE == ONE_FOR_ALL) && (Eeprom.EepromData.DemoSettings.Entity == SLAVE) && echoedOnce )
    {
        Eeprom.EepromData.DemoSettings.RngAddress = RNG_ADDR_LIST[slave_index];
        slave_index = (slave_index + 1) % total_slaves;
        // wait_us(INTER_SLAVE_DELAY);
        printf("My index %d \r\n", slave_index);
        echoedOnce = false;
    }
    ModulationParams.PacketType = PACKET_TYPE_LORA;
    PacketParams.PacketType = PACKET_TYPE_LORA;
    Radio.SetPacketType(ModulationParams.PacketType);
    Radio.SetModulationParams(&ModulationParams);
    Radio.SetPacketParams(&PacketParams);
    Radio.SetRfFrequency(Eeprom.EepromData.DemoSettings.Frequency);
    Eeprom.EepromData.DemoSettings.RngStatus = RNG_INIT;
    /* Payload length is different on slave ans master */
    PacketParams.Params.LoRa.PayloadLength = (Eeprom.EepromData.DemoSettings.Entity == MASTER)?7:9;
}

void writeRangingAddress(uint32_t address, uint8_t *buffer) {
    buffer[0] = (address >> 24) & 0xFF;
    buffer[1] = (address >> 16) & 0xFF;
    buffer[2] = (address >> 8)  & 0xFF;
    buffer[3] =  address        & 0xFF;
}

bool checkRangingAddress(uint32_t address, uint8_t *buffer) {
    uint32_t receivedAddress = buffer[0] << 24 | buffer[1] << 16 | buffer[2] << 8 | buffer[3];
    // return (receivedAddress == address);
    return( (uint32_t) buffer[3] == address);
}

void displayRangingAddress(uint8_t *buffer) {
    printf("Ranging address: %X.%X.%X.%X\r\n", (int) buffer[0], (int) buffer[1], (int) buffer[2], (int) buffer[3]);
}

void computeRangingResult(double *results, int& index) {
    results[index] = Radio.GetRangingResult(RANGING_RESULT_RAW);
    results[index] += Sx1280RangingCorrection::GetRangingCorrectionPerSfBwGain(
        ModulationParams.Params.LoRa.SpreadingFactor,
        ModulationParams.Params.LoRa.Bandwidth,
        Radio.GetRangingPowerDeltaThresholdIndicator());
    index++;
    Eeprom.EepromData.DemoSettings.CntPacketRxOK++;
}

void onRangingConfig(FSMState *state)
{
    getRangingParameters();
    // DemoInternalState = APP_RANGING_REQUEST;
    state->lastEvent = TRANSCEIVER_READY;
}

void onRangingRequest(FSMState *state)
{
    processSerial();
    configureRangingRequest();
    
    /* setting watchdog timer */
    start = time(NULL);
    endwait = start + rx_timeout;    
    Eeprom.EepromData.DemoSettings.CntPacketTx++;

    /* resetting RX validity flags */
    Eeprom.EepromData.DemoSettings.CntPacketRxOK = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = 0;
    
    /* restarting channel cycle */
    MeasuredChannels = 0;
    CurrentChannel = 0;
    /* first four bytes of the frame contain the ranging address */
    writeRangingAddress(Eeprom.EepromData.DemoSettings.RngAddress, Buffer);
    Buffer[4] = CurrentChannel;                                 // set the first channel to use
    Buffer[5] = Eeprom.EepromData.DemoSettings.RngAntenna;      // set the antenna strategy
    Buffer[6] = Eeprom.EepromData.DemoSettings.RngRequestCount; // set the number of hops);
    TX_LED = 1;
    // displayRangingAddress(Buffer);
    Radio.SendPayload(Buffer, PacketParams.Params.LoRa.PayloadLength, (TickTime_t){RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT});
    // DemoInternalState = APP_IDLE;
}

void onRanging(FSMState *state)
{

    lastRangingOK = false;
    Radio.GetPayload(Buffer, &BufferSize, BUFFER_SIZE);
    if (checkRangingAddress(Eeprom.EepromData.DemoSettings.RngAddress, Buffer))
    {

        SendNext = false;
        MeasuredChannels++;
        // printf("Ranging channel: %d \r\n", MeasuredChannels);
        if (MeasuredChannels <= Eeprom.EepromData.DemoSettings.RngRequestCount)
        {
            verbose_print("Launching ranging request \r\n");
            Radio.SetRfFrequency(Channels[CurrentChannel]);
            // printf("Channel %d\r\n", (int) CurrentChannel);
            TX_LED = 1;
            /* If both antennas are used, probing with both antennas before changing channel */
            if ( (Eeprom.EepromData.DemoSettings.RngAntenna == DEMO_RNG_ANT_BOTH) && ANT_SW) 
            {
                Eeprom.EepromData.DemoSettings.AntennaSwitch = 1;
            } 
            else 
            /* proceeding to the next channel */
            {
                Eeprom.EepromData.DemoSettings.AntennaSwitch = (Eeprom.EepromData.DemoSettings.RngAntenna == DEMO_RNG_ANT_1);
                CurrentChannel = (CurrentChannel + channel_hop) % CHANNELS;
            }
            SetAntennaSwitch();
            // printf("Ranging address: \r\n");
            // displayRangingAddress(Buffer);
            Radio.SetTx((TickTime_t){RX_TIMEOUT_TICK_SIZE, 2 * Eeprom.EepromData.DemoSettings.RngReqDelay});
        }
        else
        {
            verbose_print("Ranging completed, computing distance \r\n");
            Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = CheckDistance();
            SendNextPacket.detach();
            SendNext = false;
            Eeprom.EepromData.DemoSettings.RngStatus = RNG_INIT;
            debug_print("Ranging Completed, CntPacketRxOK = %s\r\n", GetMenuDemoRxOk());
            state->lastEvent = RANGING_COMPLETED;
        }
    }
    
}

void onRangingDone(FSMState *state)
{
    switchOffLeds();
    if (Eeprom.EepromData.DemoSettings.Entity == MASTER)    
        computeRangingResult(RawRngResults, RngResultIndex);
    
    if (Eeprom.EepromData.DemoSettings.Entity == SLAVE) 
    {
        SendNextPacket.detach();
        // SendNextPacket.attach(&SendNextPacketEvent, chrono::milliseconds(Eeprom.EepromData.DemoSettings.RngReqDelay - GUARD_TIME));
        rangingFlag= RANGING_TIMER;
        // printf("Retriggered ticker \r\n");
    }
    Eeprom.EepromData.DemoSettings.CntPacketRxOK++;
}

void onMsgReceived(FSMState *state)
{
    RX_LED = 0;
    debug_print("Receiving data\r\n");
    Radio.GetPayload(Buffer, &BufferSize, BUFFER_SIZE);
    if (checkRangingAddress(Eeprom.EepromData.DemoSettings.RngAddress, Buffer))
    {
        verbose_print("Start Ranging Process\r\n");
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
        Eeprom.EepromData.DemoSettings.RngStatus = RNG_PROCESS;
        Eeprom.EepromData.DemoSettings.RngFei = (double)(((int32_t)Buffer[4] << 24) |
                                                         ((int32_t)Buffer[5] << 16) |
                                                         ((int32_t)Buffer[6] << 8) |
                                                         Buffer[7]);
        Eeprom.EepromData.DemoSettings.RssiValue = Buffer[8]; // for ranging post-traitment (since V3 only)
        ModulationParams.PacketType = PACKET_TYPE_RANGING;
        PacketParams.PacketType = PACKET_TYPE_RANGING;

        // memcpy(&(ModulationParams.Params.LoRa.SpreadingFactor), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR, 1);
        // memcpy(&(ModulationParams.Params.LoRa.Bandwidth), Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR, 1);
        // memcpy(&(ModulationParams.Params.LoRa.CodingRate), Eeprom.Buffer + MOD_RNG_CODERATE_EEPROM_ADDR, 1);
        // memcpy(&(PacketParams.Params.LoRa.PreambleLength), Eeprom.Buffer + PAK_RNG_PREAMBLE_LEN_EEPROM_ADDR, 1);
        memcpy(&(PacketParams.Params.LoRa.HeaderType), Eeprom.Buffer + PAK_RNG_HEADERTYPE_EEPROM_ADDR, 1);
        PacketParams.Params.LoRa.PayloadLength = 10;
        memcpy(&(PacketParams.Params.LoRa.Crc), Eeprom.Buffer + PAK_RNG_CRC_MODE_EEPROM_ADDR, 1);
        memcpy(&(PacketParams.Params.LoRa.InvertIQ), Eeprom.Buffer + PAK_RNG_IQ_INV_EEPROM_ADDR, 1);

        Radio.SetPacketType(ModulationParams.PacketType);

        Radio.SetModulationParams(&ModulationParams);
        Radio.SetPacketParams(&PacketParams);
        Radio.SetRangingRequestAddress(Eeprom.EepromData.DemoSettings.RngAddress);
        Radio.SetRangingCalibration(Eeprom.EepromData.DemoSettings.RngCalib);
        Radio.SetTxParams(Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US);

        MeasuredChannels = 0;
        RngResultIndex = 0;
        wait_us(RANGING_SWITCH_TIME);
        SendNextPacket.attach(&SendNextPacketEvent, chrono::milliseconds(Eeprom.EepromData.DemoSettings.RngReqDelay)); //Eeprom.EepromData.DemoSettings.RngReqDelay * 1000);
        // DemoInternalState = APP_RNG;
        state->lastEvent = PROBING_INIT;
    }
    else
    {
        debug_print("RX failed\r\n");
        // DemoInternalState = APP_RANGING_CONFIG;
        state->lastEvent = RX_FAILED;
    }
}

void onEnableRx(FSMState *state)
{
    TX_LED = 0;
    RX_LED = 1;
    Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT});
}

void onRxTimeout(FSMState *state)
{
    RX_LED = 0;
    Eeprom.EepromData.DemoSettings.RngStatus = RNG_TIMEOUT;
    state->lastEvent = RX_TIMEOUT;

}

void onTxTimeout(FSMState *state)
{
    TX_LED = 0;
    // DemoInternalState = APP_RANGING_REQUEST;
    state->lastEvent = TX_FAILED;
}

void onIdle(FSMState *state)
{
    start = time(NULL);
    if (start > endwait)
    {
        printf("Timeout during protocol\r\n");
        endwait = start + rx_timeout;
        StopDemoApplication();
        Eeprom.EepromData.DemoSettings.HoldDemo = true;
        state->lastEvent = RANGING_TIMEOUT;
        // DemoInternalState = APP_RANGING_CONFIG;
    }
    else 
    {
        state->lastEvent = rangingFlag;
        rangingFlag = NOTHING_PENDING;
    }
}

void onWaitRangingRequest(FSMState *state) 
{
    configureRangingRequest();
    printf("Waiting ranging request...\r\n");
    /* setting watchdog timer */
    start = time(NULL);
    endwait = start + rx_timeout; 
    RX_LED = 1;
    Radio.SetRx( (TickTime_t){RX_TIMEOUT_TICK_SIZE, SLAVE_POLL_TIME});
}

void onRequestReceived(FSMState *state)
{
    switchOffLeds();
    Radio.GetPayload(Buffer, &BufferSize, BUFFER_SIZE);
    Radio.GetPacketStatus(&PacketStatus);
    // printf("Request received for: ");
    // displayRangingAddress(Buffer);
    if (checkRangingAddress(Eeprom.EepromData.DemoSettings.RngAddress, Buffer))
    {
        printf("That's for me: %d\r\n", (int) Buffer[3] );
        echoedOnce = true;
        Eeprom.EepromData.DemoSettings.RngFei = Radio.GetFrequencyError();
        Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
        Eeprom.EepromData.DemoSettings.CntPacketTx++;
        CurrentChannel = Buffer[4];
        Eeprom.EepromData.DemoSettings.RngAntenna = Buffer[5];
        Eeprom.EepromData.DemoSettings.RngRequestCount = Buffer[6];
        wait_us(5000);
        Buffer[4] = (((int32_t)Eeprom.EepromData.DemoSettings.RngFei) >> 24) & 0xFF;
        Buffer[5] = (((int32_t)Eeprom.EepromData.DemoSettings.RngFei) >> 16) & 0xFF;
        Buffer[6] = (((int32_t)Eeprom.EepromData.DemoSettings.RngFei) >> 8) & 0xFF;
        Buffer[7] = (((int32_t)Eeprom.EepromData.DemoSettings.RngFei) & 0xFF);
        Buffer[8] = Eeprom.EepromData.DemoSettings.RssiValue;
        TX_LED = 1;
        Radio.SendPayload(Buffer, 9, (TickTime_t){RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT});
    }
    else
    {
        printf("That's not for me");
        displayRangingAddress(Buffer);
        state->lastEvent = RX_FAILED;
    }
}

void onPrepareRanging(FSMState *state)
{
    switchOffLeds();
    Eeprom.EepromData.DemoSettings.RngStatus = RNG_PROCESS;

    ModulationParams.PacketType = PACKET_TYPE_RANGING;
    PacketParams.PacketType = PACKET_TYPE_RANGING;
    PacketParams.Params.LoRa.PayloadLength = 10;

    Radio.SetPacketType(ModulationParams.PacketType);
    Radio.SetModulationParams(&ModulationParams);
    Radio.SetPacketParams(&PacketParams);
    Radio.SetDeviceRangingAddress(Eeprom.EepromData.DemoSettings.RngAddress);
    Radio.SetRangingCalibration(Eeprom.EepromData.DemoSettings.RngCalib);
    printf("%d\r\n", Eeprom.EepromData.DemoSettings.RngCalib);
    Radio.SetTxParams(Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US);
    Eeprom.EepromData.DemoSettings.CntPacketRxOK = 0;
    MeasuredChannels = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 0;
    SendNextPacket.attach(&SendNextPacketEvent, chrono::milliseconds(Eeprom.EepromData.DemoSettings.RngReqDelay) );
}

void onRangingRx(FSMState *state)
{
    lastRangingOK = false;
    SendNext = false;
    MeasuredChannels++;
    if (MeasuredChannels <= Eeprom.EepromData.DemoSettings.RngRequestCount)
    {
        // printf("Ranging channel: %d \r\n", MeasuredChannels);
        Radio.SetRfFrequency(Channels[CurrentChannel]);


        RX_LED = 1;
        /* If both antennas are used, probing with both antennas before changing channel */
        if ((Eeprom.EepromData.DemoSettings.RngAntenna == DEMO_RNG_ANT_BOTH) && ANT_SW)
        {
            Eeprom.EepromData.DemoSettings.AntennaSwitch = 1;
        }
        else
        /* proceeding to the next channel */
        {
            Eeprom.EepromData.DemoSettings.AntennaSwitch = (Eeprom.EepromData.DemoSettings.RngAntenna == DEMO_RNG_ANT_1);
            CurrentChannel = (CurrentChannel + channel_hop) % CHANNELS;
        }
        SetAntennaSwitch();
        printf("Switching to ranging RX\r\n");
        Radio.SetRx((TickTime_t){RX_TIMEOUT_TICK_SIZE, 2 * Eeprom.EepromData.DemoSettings.RngReqDelay});
    }
    else
    {
        verbose_print("Ranging completed \r\n");
        Radio.SetStandby(STDBY_RC);
        SendNextPacket.detach();
        Eeprom.EepromData.DemoSettings.RngStatus = RNG_VALID;
        state->lastEvent = RANGING_COMPLETED;
    }
}

int masterFSM () {
    while (true) {
    fsmMaster.run();
    }
}



int slaveFSM() {
    while (true) {
    fsmSlave.run();
    }
    return 0;
}

// ************************      Ranging Demo      *****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
uint8_t RunDemoApplicationRanging( void )
{
    Eeprom.EepromData.DemoSettings.HoldDemo = false;
    TX_LED = 0;
    RX_LED = 0;
    ANT_SW = 1;
    Eeprom.EepromData.DemoSettings.CntPacketTx = 0;
    Eeprom.EepromData.DemoSettings.RngFei = 0.0;
    Eeprom.EepromData.DemoSettings.RngStatus = RNG_INIT;
    Eeprom.EepromData.DemoSettings.ModulationParam2 = LORA_BW_1600;
    Eeprom.EepromData.DemoSettings.ModulationParam1 = LORA_SF8;
    Eeprom.EepromData.DemoSettings.ModulationParam3 = LORA_CR_4_8;

    ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF8;
    ModulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
    ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_8;
    PacketParams.Params.LoRa.PreambleLength = 8;

    Eeprom.EepromData.DemoSettings.RngReqDelay = 10;
    InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );
    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        verbose_print("Setting Master\r\n");
        Eeprom.EepromData.DemoSettings.TimeOnAir = RX_TX_INTER_PACKET_DELAY;
        Radio.SetDioIrqParams( IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT,
                                IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT,
                                IRQ_RADIO_NONE, IRQ_RADIO_NONE );
        Eeprom.EepromData.DemoSettings.RngDistance = 0.0;
        DemoInternalState = APP_RANGING_CONFIG;
    }
    else
    {
        // Radio.SetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
        Radio.SetDioIrqParams(IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_SLAVE_REQUEST_DISCARDED | IRQ_RANGING_SLAVE_RESPONSE_DONE ,
                              IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_SLAVE_REQUEST_DISCARDED | IRQ_RANGING_SLAVE_RESPONSE_DONE, 
                              IRQ_RADIO_NONE, 
                              IRQ_RADIO_NONE );
        DemoInternalState = APP_RANGING_CONFIG;
    }
    
    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        while (1) {
            masterFSM();
        }
    }
    else    // Slave
    {
        while (1) {
            slaveFSM();
        }
    }
}

// ************************        Utils            ****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************

void InitDemoApplication( void )
{
    // RX_LED = 1;
    // TX_LED = 1;
    // SetAntennaSwitch( );

    wait_us(1000000); // wait for on board DC/DC start-up time

    Radio.Init( );
    // Can also be set in LDO mode but consume more power
    // Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );
    Radio.SetStandby( STDBY_RC );
    Radio.SetRegulatorMode(USE_DCDC);


    memset( &Buffer, 0x00, BufferSize );
    RX_LED = 0;
    TX_LED = 0;

    PacketRxSequence = 0;
    PacketRxSequencePrev = 0;
    Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
    Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
}

void StopDemoApplication( void )
{
    if( DemoRunning == true )
    {
        __disable_irq( );    // Disable Interrupts

#ifdef PRINT_DEBUG
        printf( "StopDemoApplication\n\r" );
#endif

        if( Radio.GetOpMode( ) == MODE_SLEEP )
        {
            Radio.Wakeup( );
            InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );
        }
        RX_LED = 0;
        TX_LED = 0;
        DemoRunning = false;
        SendNext = false;
        ReceiveNext = false;
        PacketRxSequence = 0;
        PacketRxSequencePrev = 0;
        Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;

        Radio.SetAutoFs( false );
        DemoInternalState = APP_IDLE;
        Radio.SetStandby( STDBY_RC );
        Radio.ClearIrqStatus( IRQ_RADIO_ALL );
        SendNextPacket.detach( );
        ReceiveNextPacket.detach( );

        __enable_irq( );     // Enable Interrupts
    }
}

void InitializeDemoParameters( uint8_t modulation )
{
    Radio.SetStandby( STDBY_RC );

    // Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );
    Radio.SetRegulatorMode(USE_DCDC);

#ifdef PRINT_DEBUG
    printf("> InitializeDemoParameters\n\r");
#endif
    if( modulation == PACKET_TYPE_LORA )
    {
#ifdef PRINT_DEBUG
        printf("set param LORA for demo\n\r");
#endif
        ModulationParams.PacketType = PACKET_TYPE_LORA;
        PacketParams.PacketType     = PACKET_TYPE_LORA;

        ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )  Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.LoRa.Bandwidth       = ( RadioLoRaBandwidths_t )        Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.LoRa.CodingRate      = ( RadioLoRaCodingRates_t )       Eeprom.EepromData.DemoSettings.ModulationParam3;
        PacketParams.Params.LoRa.PreambleLength      =                                  Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.LoRa.HeaderType          = ( RadioLoRaPacketLengthsModes_t )Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.LoRa.PayloadLength       =                                  Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.LoRa.Crc                 = ( RadioLoRaCrcModes_t )          Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.LoRa.InvertIQ            = ( RadioLoRaIQModes_t )           Eeprom.EepromData.DemoSettings.PacketParam5;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.LoRa.PayloadLength;

        Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
        printf("Modulation loRa params: %s", GetRadioModulationParameters1());
    }
    else if( modulation == PACKET_TYPE_FLRC )
    {
#ifdef PRINT_DEBUG
        printf("set param FLRC for demo\n\r");
#endif
        ModulationParams.PacketType = PACKET_TYPE_FLRC;
        PacketParams.PacketType     = PACKET_TYPE_FLRC;

        ModulationParams.Params.Flrc.BitrateBandwidth  = ( RadioFlrcBitrates_t )       Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.Flrc.CodingRate        = ( RadioFlrcCodingRates_t )    Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.Flrc.ModulationShaping = ( RadioModShapings_t )        Eeprom.EepromData.DemoSettings.ModulationParam3;
        PacketParams.Params.Flrc.PreambleLength        = ( RadioPreambleLengths_t )    Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.Flrc.SyncWordLength        = ( RadioFlrcSyncWordLengths_t )Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.Flrc.SyncWordMatch         = ( RadioSyncWordRxMatchs_t )   Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.Flrc.HeaderType            = ( RadioPacketLengthModes_t )  Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.Flrc.PayloadLength         =                               Eeprom.EepromData.DemoSettings.PacketParam5;
        PacketParams.Params.Flrc.CrcLength             = ( RadioCrcTypes_t )           Eeprom.EepromData.DemoSettings.PacketParam6;
        PacketParams.Params.Flrc.Whitening             = ( RadioWhiteningModes_t )     Eeprom.EepromData.DemoSettings.PacketParam7;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.Flrc.PayloadLength;

        Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    }
    else if( modulation == PACKET_TYPE_GFSK )
    {
#ifdef PRINT_DEBUG
        printf("set param GFSK for demo\n\r");
#endif
        ModulationParams.PacketType = PACKET_TYPE_GFSK;
        PacketParams.PacketType     = PACKET_TYPE_GFSK;

        ModulationParams.Params.Gfsk.BitrateBandwidth  = ( RadioGfskBleBitrates_t )  Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.Gfsk.ModulationIndex   = ( RadioGfskBleModIndexes_t )Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.Gfsk.ModulationShaping = ( RadioModShapings_t )      Eeprom.EepromData.DemoSettings.ModulationParam3;
        PacketParams.Params.Gfsk.PreambleLength        = ( RadioPreambleLengths_t )  Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.Gfsk.SyncWordLength        = ( RadioSyncWordLengths_t )  Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.Gfsk.SyncWordMatch         = ( RadioSyncWordRxMatchs_t ) Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.Gfsk.HeaderType            = ( RadioPacketLengthModes_t )Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.Gfsk.PayloadLength         =                             Eeprom.EepromData.DemoSettings.PacketParam5;
        PacketParams.Params.Gfsk.CrcLength             = ( RadioCrcTypes_t )         Eeprom.EepromData.DemoSettings.PacketParam6;
        PacketParams.Params.Gfsk.Whitening             = ( RadioWhiteningModes_t )   Eeprom.EepromData.DemoSettings.PacketParam7;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.Gfsk.PayloadLength;

        Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    }
    if( modulation == PACKET_TYPE_RANGING )
    {
        Radio.SetBufferBaseAddresses( 0x00, 0x00 );
        Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );
        // memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR, 1 );
        // memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,      1 );
        ModulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.LoRa.Bandwidth = (RadioLoRaBandwidths_t)Eeprom.EepromData.DemoSettings.ModulationParam2;

        switch( ModulationParams.Params.LoRa.Bandwidth )
        {
            case LORA_BW_0400:
                Eeprom.EepromData.DemoSettings.RngCalib     = RNG_CALIB_0400[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngFeiFactor = ( double )RNG_FGRAD_0400[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngReqDelay  = RNG_TIMER_MS >> ( 0 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
                break;

            case LORA_BW_0800:
                Eeprom.EepromData.DemoSettings.RngCalib     = RNG_CALIB_0800[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngFeiFactor = ( double )RNG_FGRAD_0800[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngReqDelay  = RNG_TIMER_MS >> ( 1 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
                break;

            case LORA_BW_1600:
                Eeprom.EepromData.DemoSettings.RngCalib     = RNG_CALIB_1600[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngFeiFactor = ( double )RNG_FGRAD_1600[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngReqDelay  = RNG_TIMER_MS >> ( 2 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
                break;
        }
        // printf("Modulation params ranging: %s", GetRadioModulationParameters1());

        Radio.SetPollingMode( );
        Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    }
    else
    {
        Radio.SetStandby( STDBY_RC );
        Radio.SetPacketType( ModulationParams.PacketType );
        Radio.SetRfFrequency( Eeprom.EepromData.DemoSettings.Frequency );
        Radio.SetBufferBaseAddresses( 0x00, 0x00 );
        Radio.SetModulationParams( &ModulationParams );
        Radio.SetPacketParams( &PacketParams );
        // only used in GFSK, FLRC (4 bytes max) and BLE mode
//        Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );
        // only used in GFSK, FLRC
        uint8_t crcSeedLocal[2] = {0x45, 0x67};
        Radio.SetCrcSeed( crcSeedLocal );
        Radio.SetCrcPolynomial( 0x0123 );
        Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );
        Radio.SetPollingMode( );
    }
}

/*!
 * \brief Callback of ticker PerSendNextPacket
 */
void SendNextPacketEvent( void )
{
    SendNext = true;
    rangingFlag = RANGING_TIMER;
    // printf("Ticker !\r\n");
    if( Eeprom.EepromData.DemoSettings.RngStatus == RNG_PROCESS )
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave++;
    }
}

/*!
 * \brief Callback of ticker ReceiveNextPacket
 */
void ReceiveNextPacketEvent( void )
{
    ReceiveNext = true;
}

uint8_t CheckDistance( void )
{
    double displayRange = 0.0;

    uint16_t j = 0;
    uint16_t i;

    if( RngResultIndex > 0 )
    {
        for( i = 0; i < RngResultIndex; ++i )
        {
            RawRngResults[i] = RawRngResults[i] - ( Eeprom.EepromData.DemoSettings.RngFeiFactor * Eeprom.EepromData.DemoSettings.RngFei / 1000 );
        }

        for (int i = RngResultIndex - 1; i > 0; --i)
        {
            for (int j = 0; j < i; ++j)
            {
                if (RawRngResults[j] > RawRngResults[j+1])
                {
                    int temp = RawRngResults[j];
                    RawRngResults[j] = RawRngResults[j+1];
                    RawRngResults[j+1] = temp;
                }
            }
        }
        double median;


        if ((RngResultIndex % 2) == 0)
        {
            median = (RawRngResults[RngResultIndex/2] + RawRngResults[(RngResultIndex/2) - 1])/2.0;
        }
        else
        {
            median = RawRngResults[RngResultIndex/2];
        }

        if( median < 100 )
        {
            displayRange = Sx1280RangingCorrection::ComputeRangingCorrectionPolynome(
            ModulationParams.Params.LoRa.SpreadingFactor,
            ModulationParams.Params.LoRa.Bandwidth,
            median
            );

        }
        else
        {
            displayRange = median;
        }

        if( j < DEMO_RNG_CHANNELS_COUNT_MIN )
        {
            Eeprom.EepromData.DemoSettings.RngStatus = RNG_PER_ERROR;
        }
        else
        {
            Eeprom.EepromData.DemoSettings.RngStatus = RNG_VALID;
        }

        if( displayRange < 0 )
        {
            Eeprom.EepromData.DemoSettings.RngDistance = 0.0;
        }
        else
        {
            switch( Eeprom.EepromData.DemoSettings.RngUnit )
            {
                case DEMO_RNG_UNIT_SEL_M:
                    Eeprom.EepromData.DemoSettings.RngDistance = displayRange;
                    break;

                case DEMO_RNG_UNIT_SEL_YD:
                    Eeprom.EepromData.DemoSettings.RngDistance = displayRange * DEMO_RNG_UNIT_CONV_YD;
                    break;

                case DEMO_RNG_UNIT_SEL_MI:
                    Eeprom.EepromData.DemoSettings.RngDistance = displayRange * DEMO_RNG_UNIT_CONV_MI;
                    break;
            }
        }
    }

//    printf(".\n\r");


//    printf( "{\"type\":\"LoRa2.4\",\"frequency\":\"%s\",\"bw\":\"%s\",\"sf\":\"%s\",\"cr\":\"%s\",\"rssi\":\"%d\",\"zn\":\"%d\",\"distance\":\"%.1f\",\"fei\":\"%d\",\"initiator\":\"master\"}\n\r", GetRadioFrequencyGHz( ), GetRadioModulationParameters2( ), GetRadioModulationParameters1( ), GetRadioModulationParameters3( ), Eeprom.EepromData.DemoSettings.RssiValue, j, Eeprom.EepromData.DemoSettings.RngDistance, ( int32_t )Eeprom.EepromData.DemoSettings.RngFei);
//
//    printf( "{\"type\":\"LoRa2.4\",\"frequency\":\"%s\",\"bw\":\"%s\",\"sf\":\"%s\",\"cr\":\"%s\",\"rssi\":\"%d\",\"zn\":\"%d\",\"distance\":\"%.1f\",\"fei\":\"%d\",\"initiator\":\"master\",\"target\":\"%d\"}\n\r", GetRadioFrequencyGHz( ), GetRadioModulationParameters2( ), GetRadioModulationParameters1( ), GetRadioModulationParameters3( ), Eeprom.EepromData.DemoSettings.RssiValue, j, Eeprom.EepromData.DemoSettings.RngDistance, ( int32_t )Eeprom.EepromData.DemoSettings.RngFei, Eeprom.EepromData.DemoSettings.RngAddress );
    float uncertainty = 3 + (float) Eeprom.EepromData.DemoSettings.RngDistance * 0.08;
    int beacon_idx = (slave_index > 4)?slave_index + 6:slave_index + 1;
    printf( "{\"type\":\"LoRa2.4\",\"frequency\":\"%s\",\"bw\":\"%s\",\"sf\":\"%s\",\"cr\":\"%s\",\"rssi\":\"%d\",\"zn\":\"%d\",\"distance\":\"%d.%d\",\"uncertainty\":\"%d.%d\",\"fei\":\"%d\",\"initiator\":\"master\",\"target\":\"%d\"}\r\n", 
    GetRadioFrequencyGHz( ), 
    GetRadioModulationParameters2( ), 
    GetRadioModulationParameters1( ), 
    GetRadioModulationParameters3( ), 
    Eeprom.EepromData.DemoSettings.RssiValue, 
    j, 
    (int) INT(Eeprom.EepromData.DemoSettings.RngDistance), 
    (int) DEC(Eeprom.EepromData.DemoSettings.RngDistance), 
    (int) INT(uncertainty), 
    (int) DEC(uncertainty),  
    ( int32_t )Eeprom.EepromData.DemoSettings.RngFei,
    beacon_idx );

    // printf( "{\"type\":\"LoRa2.4\",\"sf\":\"%s\",\"rssi\":\"%d\",\"distance\":\"%d.%d\",\"uncertainty\":\"%d.%d\", \"fei\":\"%d.%d\"\"target\":\"", 
    //         GetRadioModulationParameters1( ), 
    //         Eeprom.EepromData.DemoSettings.RssiValue, 
    //         (int) INT(Eeprom.EepromData.DemoSettings.RngDistance), 
    //         (int) DEC(Eeprom.EepromData.DemoSettings.RngDistance),
    //         (int) INT(uncertainty),
    //         (int) DEC(uncertainty),
    //         (int) INT(Eeprom.EepromData.DemoSettings.RngFei),
    //         (int) DEC(Eeprom.EepromData.DemoSettings.RngDistance));
    // int beacon_idx = (slave_index > 4)?slave_index + 6:slave_index + 1;
    // printf("%d\"}\r\n", beacon_idx);
    // if (Eeprom.EepromData.DemoSettings.RngAddress == DEMO_RNG_ADDR_1){
    //  	printf("1");
    //  }
    //  else if (Eeprom.EepromData.DemoSettings.RngAddress == DEMO_RNG_ADsDR_2){
    //  	printf("2");
    //  }
    //  else if (Eeprom.EepromData.DemoSettings.RngAddress == DEMO_RNG_ADDR_3){
    //  	printf("3");
    //  }
    //  else{
    //  	printf("NC");
    //  }
    // printf("\"}\n\r");
    // printf(
    //     "*%s|%s|%d.%d|%d|%d.%d|%d\r\n", 
    //     MASTER_IDS[0],
    //     SLAVE_IDS[slave_index],
    //     (int) INT(Eeprom.EepromData.DemoSettings.RngDistance),
    //     (int) DEC(Eeprom.EepromData.DemoSettings.RngDistance),
    //     (int) Eeprom.EepromData.DemoSettings.TimeOnAir, 
    //     (int) INT(Eeprom.EepromData.DemoSettings.RngFei),  
    //     (int) DEC(Eeprom.EepromData.DemoSettings.RngFei),
    //     (int) Eeprom.EepromData.DemoSettings.RssiValue); 


    
    return j;
}

void LedBlink( void )
{
    if( ( TX_LED == 0 ) && ( RX_LED == 0 ) )
    {
        TX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 0 ) )
    {
        RX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 1 ) )
    {
        TX_LED = 0;
    }
    else
    {
        RX_LED = 0;
    }
}

void SetAntennaSwitch( void )
{
    if( Eeprom.EepromData.DemoSettings.AntennaSwitch == 0 )
    {
        ANT_SW = 1; // ANT1
    }
    else
    {
        ANT_SW = 0; // ANT0
    }
}

// ************************     Radio Callbacks     ****************************
// *                                                                           *
// * These functions are called through function pointer by the Radio low      *
// * level drivers                                                             *
// *                                                                           *
// *****************************************************************************
void OnTxDone( void )
{
    // DemoInternalState = APP_ENABLE_RX;
    rangingClock =  HAL_GetTick();
    verbose_print("TX done\r\n");
    rangingFlag = REQUEST_SENT;
}

void OnRxDone( void )
{
    verbose_print("RX done\r\n");
	Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
    displayRangingAddress(Buffer);
    if (checkRangingAddress(Eeprom.EepromData.DemoSettings.RngAddress, Buffer))
	{
        rangingFlag = REQUEST_RECEIVED;
    }
    else {
        /* improper formatting */
        rangingFlag = RX_FAILED;
    }
}

void OnTxTimeout( void )
{
    rangingFlag = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    rangingFlag = RX_TIMEOUT;
    printf("RX Timeout event\r\n");
}

void OnRxError( IrqErrorCode_t errorCode )
{
    rangingFlag = RX_FAILED;
}

void OnRangingDone( IrqRangingCode_t val )
{
    // printf("Ranging done \r\n");

    if( val == IRQ_RANGING_MASTER_VALID_CODE || val == IRQ_RANGING_SLAVE_VALID_CODE )
    {
        rangingFlag = PROBING_COMPLETED;
        lastRangingOK = true;
    }
    else if( val == IRQ_RANGING_MASTER_ERROR_CODE || val == IRQ_RANGING_SLAVE_ERROR_CODE )
    {
        /* errors are returned when ranging times out */
        rangingFlag = RANGING_TIMEOUT;
        printf("Ranging error: %d !\r\n", val);
        SendNextPacket.detach();
    }
}

void OnCadDone( bool channelActivityDetected )
{
}


char* GetMenuRadioFrameType( void )
{
    switch( Eeprom.EepromData.DemoSettings.ModulationType )
    {
        case PACKET_TYPE_FLRC:    return ( char* )" FLRC";
        case PACKET_TYPE_RANGING: return ( char* )"RANGING";
        case PACKET_TYPE_GFSK:    return ( char* )" GFSK";
        case PACKET_TYPE_BLE:     return ( char* )"  BLE";
        case PACKET_TYPE_LORA:
        default:                  return ( char* )" LORA";
    }
}

char* GetRadioModulationParameters1( void )
{
    if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_FLRC )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam1 )
        {
            case FLRC_BR_1_300_BW_1_2: return ( char* )"1.3 Mbps/BW 1.2 MHz";
            case FLRC_BR_1_040_BW_1_2: return ( char* )"1.0 Mbps/BW 1.2 MHz";
            case FLRC_BR_0_650_BW_0_6: return ( char* )"650 kbps/BW 600 kHz";
            case FLRC_BR_0_520_BW_0_6: return ( char* )"520 kbps/BW 600 kHz";
            case FLRC_BR_0_325_BW_0_3: return ( char* )"325 kbps/BW 300 kHz";
            case FLRC_BR_0_260_BW_0_3: return ( char* )"260 kbps/BW 300 kHz";
            default:                   return ( char* )"X";
        }
    }
    else if( ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_GFSK ) || \
             ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_BLE ) )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam1 )
        {
            case GFSK_BLE_BR_2_000_BW_2_4: return ( char* )"2.0 Mbps/BW 2.4 MHz";
            case GFSK_BLE_BR_1_600_BW_2_4: return ( char* )"1.6 Mbps/BW 2.4 MHz";
            case GFSK_BLE_BR_1_000_BW_2_4: return ( char* )"1.0 Mbps/BW 2.4 MHz";
            case GFSK_BLE_BR_1_000_BW_1_2: return ( char* )"1.0 Mbps/BW 1.2 MHz";
            case GFSK_BLE_BR_0_800_BW_2_4: return ( char* )"800 kbps/BW 2.4 MHz";
            case GFSK_BLE_BR_0_800_BW_1_2: return ( char* )"800 kbps/BW 1.2 MHz";
            case GFSK_BLE_BR_0_500_BW_1_2: return ( char* )"500 kbps/BW 1.2 MHz";
            case GFSK_BLE_BR_0_500_BW_0_6: return ( char* )"500 kbps/BW 600 kHz";
            case GFSK_BLE_BR_0_400_BW_1_2: return ( char* )"400 kbps/BW 1.2 MHz";
            case GFSK_BLE_BR_0_400_BW_0_6: return ( char* )"400 kbps/BW 600 kHz";
            case GFSK_BLE_BR_0_250_BW_0_6: return ( char* )"250 kbps/BW 600 kHz";
            case GFSK_BLE_BR_0_250_BW_0_3: return ( char* )"250 kbps/BW 300 kHz";
            case GFSK_BLE_BR_0_125_BW_0_3: return ( char* )"125 kbps/BW 300 kHz";
            default:                      return ( char* )"X";
        }
    }
    else if( ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_LORA ) || \
             ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_RANGING ) )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam1 )
        {
            case LORA_SF5:  return ( char* )"SF5";
            case LORA_SF6:  return ( char* )"SF6";
            case LORA_SF7:  return ( char* )"SF7";
            case LORA_SF8:  return ( char* )"SF8";
            case LORA_SF9:  return ( char* )"SF9";
            case LORA_SF10: return ( char* )"SF10";
            case LORA_SF11: return ( char* )"SF11";
            case LORA_SF12: return ( char* )"SF12";
            default:        return ( char* )"X";
        }
    }
    else
    {
        return ( char* )"";
    }
}

char* GetRadioModulationParameters2( void )
{
    if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_FLRC )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam2 )
        {
            case FLRC_CR_1_2: return ( char* )"CR 1/2";
            case FLRC_CR_3_4: return ( char* )"CR 3/4";
            case FLRC_CR_1_0: return ( char* )"CR 1";
            default:          return ( char* )"X";
        }
    }
    else if( ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_GFSK ) || \
             ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_BLE ) )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam2 )
        {
            case GFSK_BLE_MOD_IND_0_35: return ( char* )"Mod.i 0.35";
            case GFSK_BLE_MOD_IND_0_50: return ( char* )"Mod.i 0.5";
            case GFSK_BLE_MOD_IND_0_75: return ( char* )"Mod.i 0.75";
            case GFSK_BLE_MOD_IND_1_00: return ( char* )"Mod.i 1";
            case GFSK_BLE_MOD_IND_1_25: return ( char* )"Mod.i 1.25";
            case GFSK_BLE_MOD_IND_1_50: return ( char* )"Mod.i 1.5";
            case GFSK_BLE_MOD_IND_1_75: return ( char* )"Mod.i 1.75";
            case GFSK_BLE_MOD_IND_2_00: return ( char* )"Mod.i 2";
            case GFSK_BLE_MOD_IND_2_25: return ( char* )"Mod.i 2.25";
            case GFSK_BLE_MOD_IND_2_50: return ( char* )"Mod.i 2.50";
            case GFSK_BLE_MOD_IND_2_75: return ( char* )"Mod.i 2.75";
            case GFSK_BLE_MOD_IND_3_00: return ( char* )"Mod.i 3";
            case GFSK_BLE_MOD_IND_3_25: return ( char* )"Mod.i 3.25";
            case GFSK_BLE_MOD_IND_3_50: return ( char* )"Mod.i 3.5";
            case GFSK_BLE_MOD_IND_3_75: return ( char* )"Mod.i 3.75";
            case GFSK_BLE_MOD_IND_4_00: return ( char* )"Mod.i 4";
            default:                   return ( char* )"X";
        }
    }
    else if( ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_LORA ) || \
             ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_RANGING ) )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam2 )
        {
            case LORA_BW_0200: return ( char* )"BW 200 kHz";
            case LORA_BW_0400: return ( char* )"BW 400 kHz";
            case LORA_BW_0800: return ( char* )"BW 800 kHz";
            case LORA_BW_1600: return ( char* )"BW 1.6 MHz";
            default:           return ( char* )"X";
        }
    }
    else
	    {
	        return ( char* )"";
    }
}

char* GetRadioModulationParameters3( void )
{
    if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_FLRC )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam3 )
        {
            case RADIO_MOD_SHAPING_BT_OFF: return ( char* )"BT OFF";
            case RADIO_MOD_SHAPING_BT_1_0: return ( char* )"BT 1";
            case RADIO_MOD_SHAPING_BT_0_5: return ( char* )"BT 0.5";
            default:                       return ( char* )"X";
        }
    }
    else if( ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_GFSK ) || \
             ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_BLE ) )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam3 )
        {
            case RADIO_MOD_SHAPING_BT_OFF: return ( char* )"BT OFF";
            case RADIO_MOD_SHAPING_BT_1_0: return ( char* )"BT 1";
            case RADIO_MOD_SHAPING_BT_0_5: return ( char* )"BT 0.5";
            default:                       return ( char* )"X";
        }
    }
    else if( ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_LORA ) || \
             ( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_RANGING ) )
    {
        switch( Eeprom.EepromData.DemoSettings.ModulationParam3 )
        {
            case LORA_CR_4_5:    return ( char* )"CR 4/5";
            case LORA_CR_4_6:    return ( char* )"CR 4/6";
            case LORA_CR_4_7:    return ( char* )"CR 4/7";
            case LORA_CR_4_8:    return ( char* )"CR 4/8";
            case LORA_CR_LI_4_5: return ( char* )"CRLI 4/5";
            case LORA_CR_LI_4_6: return ( char* )"CRLI 4/6";
            case LORA_CR_LI_4_7: return ( char* )"CRLI 4/7";
            default:             return ( char* )"X";
        }
    }
    else
    {
        return ( char* )"";
    }
}

char* GetRadioFrequency( void )
{
    sprintf( StringText2, "%lu Hz", \
             ( unsigned long )Eeprom.EepromData.DemoSettings.Frequency );
    return StringText2;
}

void UpdateRadioFrequency( unsigned long freq )
{
    sprintf( StringText ,"f:%1d.%03d GHz, P:%s, %s", freq / 1000000000, ( freq / 1000000 ) % 1000, GetRadioTxPower( ), GetAntennaSetting( ) );
    return;
}

char* GetRadioFrequencyGHz( void )
{
    // quicker than using a float (which is not optimized in MBED)
    sprintf( StringText3, "%1d.%03d GHz", \
             Eeprom.EepromData.DemoSettings.Frequency / 1000000000, \
             ( Eeprom.EepromData.DemoSettings.Frequency / 1000000 ) % 1000 );
    return StringText3;
}

char* GetRadioFreqBase( void )
{
    switch( CurrentFreqBase )
    {
        case FB1:    return ( char* )"1 Hz";
        case FB10:   return ( char* )"10 Hz";
        case FB100:  return ( char* )"100 Hz";
        case FB1K:   return ( char* )"1 kHz";
        case FB10K:  return ( char* )"10 kHz";
        case FB100K: return ( char* )"100 kHz";
        case FB1M:   return ( char* )"1 MHz";
        case FB10M:  return ( char* )"10 MHz";
        default:     return ( char* )"X";
    }
}

char* GetRadioFreqBasePS1( void )
{
    sprintf( StringText, "%lu Hz", DEMO_CENTRAL_FREQ_PRESET1 );
    return StringText;
}

char* GetRadioFreqBasePS2( void )
{
    sprintf( StringText, "%lu Hz", DEMO_CENTRAL_FREQ_PRESET2 );
    return StringText;
}

char* GetRadioFreqBasePS3( void )
{
    sprintf( StringText, "%lu Hz", DEMO_CENTRAL_FREQ_PRESET3 );
    return StringText;
}

char* GetRadioTxPower( void )
{
    sprintf( StringText2, "%d dBm", Eeprom.EepromData.DemoSettings.TxPower );
    return StringText2;
}

char* GetRadioPayloadLength( void )
{
    if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_LORA )
    {
        sprintf( StringText2, "%d", Eeprom.EepromData.DemoSettings.PacketParam3 );
    }
    else // PACKET_TYPE_GFSK, PACKET_TYPE_FLRC
    {
        sprintf( StringText2, "%d", Eeprom.EepromData.DemoSettings.PacketParam5 );
    }
    return StringText2;
}

char* GetMenuDemoMaxNumPacket( void )
{
    if( Eeprom.EepromData.DemoSettings.MaxNumPacket == 0 )
    {
        return ( char* )"Infinite";
    }
    else
    {
        sprintf( StringText, "%6d", Eeprom.EepromData.DemoSettings.MaxNumPacket );
    }
    return StringText;
}

char* GetMenuDemoNumSentPacket( void )
{
    sprintf( StringText2, "%6d", Eeprom.EepromData.DemoSettings.CntPacketTx );
    return StringText2;
}

char* GetMenuDemoRxOk( void )
{
    sprintf( StringText2, "%6lu", \
            ( unsigned long )( Eeprom.EepromData.DemoSettings.CntPacketRxOK ) );
    return StringText2;
}

char* GetMenuDemoRxKo( void )
{
    sprintf( StringText2, "%6lu", \
             ( unsigned long )( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
             Eeprom.EepromData.DemoSettings.RxTimeOutCount ) );
    return StringText2;
}

char* GetMenuDemoRxOkSlave( void )
{
    sprintf( StringText3, "%6lu", \
       ( unsigned long )( Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave ) );
    return StringText3;
}

char* GetMenuDemoResultPerCent1( uint32_t value, uint32_t reference )
{
    // quicker than using a float (which is not optimized in MBED)
    sprintf( StringText2, "%3d.%02d", \
             ( ( value * 10000 ) / reference ) / 100, \
             ( ( value * 10000 ) / reference ) % 100 );
    return StringText2;
}

char* GetMenuDemoResultPerCent2( uint32_t value, uint32_t reference )
{
    // quicker than using a float (which is not optimized in MBED)
    sprintf( StringText3, "%3d.%02d", \
             ( ( value * 10000 ) / reference ) / 100, \
             ( ( value * 10000 ) / reference ) % 100 );
    return StringText3;
}

char* GetMenuDemoRxKoSlave( void )
{
    sprintf( StringText3, "%6lu", \
       ( unsigned long )( Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave ) );
    return StringText3;
}

char* GetMenuDemoRssi( void )
{
    sprintf( StringText2, "%6d", Eeprom.EepromData.DemoSettings.RssiValue );
    return StringText2;
}

char* GetMenuDemoSnr( void )
{
    if( Eeprom.EepromData.DemoSettings.SnrValue >= 0 )
    {
        sprintf( StringText2, "     /" );
    }
    else
    {
        sprintf( StringText2, "%6d", Eeprom.EepromData.DemoSettings.SnrValue );
    }
    return StringText2;
}

char* GetAntennaSetting( void )
{
    if( Eeprom.EepromData.DemoSettings.AntennaSwitch == 0 )
    {
        return ( char* )"ANT1";
    }
    else
    {
        return ( char* )"ANT0";
    }
}

char* GetTotalPackets( void )
{
    if( Eeprom.EepromData.DemoSettings.MaxNumPacket == 0 )
    {
        return ( char* )"Total:  Inf.";
    }
    else
    {
        sprintf( StringText, "Total: %5lu", \
                 ( unsigned long )( Eeprom.EepromData.DemoSettings.MaxNumPacket ) );
        return StringText;
    }
}

#if defined(HAS_GPS_SENSOR)
char* GetGpsTime( void )
{
    thisGps = Max7GpsgetData( );
    if( ( thisGps->Position.Fixed ) && ( thisGps->Time.Updated ) )
    {
        sprintf( StringText, "GPS:  %s.%s.%s %s:%s:%s", thisGps->Time.Year, \
                                                        thisGps->Time.Month, \
                                                        thisGps->Time.Day, \
                                                        thisGps->Time.Hour, \
                                                        thisGps->Time.Minute, \
                                                        thisGps->Time.Second );
        thisGps->Time.Updated = false;
        return StringText;
    }
    else
    {
        return ( char* )"GPS: Satellites searching..";
    }
}

char* GetGpsPos( void )
{
    thisGps = Max7GpsgetData( );
    if( thisGps->Position.Fixed )
    {
        sprintf( StringText,"%s, %s", thisGps->Position.Lat, \
                                      thisGps->Position.Long );
        return StringText;
    }
    else
    {
        return ( char* )"Pos: Satellites searching..";
    }
}
#else
char* GetGpsTime( void )
{
    return ( char* )"";
}

char* GetGpsPos( void )
{
    return ( char* )"GPS: Not supported";
}
#endif

#if defined(HAS_PROXIMITY_SENSOR)
char* GetProximityValue( void )
{
    sprintf( StringText,"Proximity : %06d, %06d", \
                        SX9306proximityGetReadValue( 1 ), \
                        SX9306proximityGetReadValue( 0 ) ); // Left then right
    return StringText;
}
#else
char* GetProximityValue( void )
{
    return ( char * )"Proximity: Not supported";
}
#endif

char* GetMenuDemoRadioPowerMode( void )
{
    if( Eeprom.EepromData.DemoSettings.RadioPowerMode == USE_LDO )
    {
        return ( char* )"  LDO";
    }
    else
    {
        return ( char* )"  DCDC";
    }
}

char* GetFrequencyError( void )
{
    sprintf( StringText2, "%6d", ( int32_t )Eeprom.EepromData.DemoSettings.RngFei );
    return StringText2;
}

char* GetRngChannelsOk( void )
{
    if( Eeprom.EepromData.DemoSettings.Entity == SLAVE )
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxOK /= 2;
    }
    sprintf( StringText2, "%03d/%03d", Eeprom.EepromData.DemoSettings.CntPacketRxOK, Eeprom.EepromData.DemoSettings.RngRequestCount );
    return StringText2;
}

char* GetRangingRequestCount( void )
{
    sprintf( StringText2, "%d", Eeprom.EepromData.DemoSettings.RngRequestCount );
    return StringText2;
}

char* GetRangingAddress( void )
{
    sprintf( StringText2, "0x%08x", Eeprom.EepromData.DemoSettings.RngAddress );
    return StringText2;
}

char* GetRangingAntenna( void )
{
    switch( Eeprom.EepromData.DemoSettings.RngAntenna )
    {
        case DEMO_RNG_ANT_1:    return ( char* )"ANT1";
        case DEMO_RNG_ANT_0:    return ( char* )"ANT0";
        case DEMO_RNG_ANT_BOTH: return ( char* )"BOTH";
        default:                return ( char* )"X";
    }
}

char* GetRangingUnit( void )
{
    switch( Eeprom.EepromData.DemoSettings.RngUnit )
    {
        case DEMO_RNG_UNIT_SEL_M:  return ( char* )"Meter";
        case DEMO_RNG_UNIT_SEL_YD: return ( char* )"Yard";
        case DEMO_RNG_UNIT_SEL_MI: return ( char* )"Mile";
        default:                   return ( char* )"X";
    }
}
