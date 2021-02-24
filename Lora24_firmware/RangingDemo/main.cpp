#include <stdio.h>
#include <stdlib.h>
#include "mbed.h"
#include "SX1280.h"
#include "SX1280-hal.h"
#include <math.h>
#include "chrono"
#include "FreqLUT.h"

/** 
 * Compile with
 * mbed compile --source C:\Users\pestourb\Documents\Poucet\Mbed-test\SX1280\BUILD\libraries\SX1280\NUCLEO_L432KC\GCC_ARM-RELEASE --source RangingDemo -NRangingDemo
**/

#define BAUDRATE 115200
#define BUFFER_SIZE 255

#define FLOAT_DECIMALS 4
#define FLOAT_MAX_DIGITS 20
#define FLOAT_EXPONENT pow(10, FLOAT_DECIMALS)
#define INT(_FLOAT) (trunc(_FLOAT))
#define DEC(_FLOAT) (FLOAT_EXPONENT * fabs(_FLOAT - INT(_FLOAT)) )

#define INTER_RANGING_DELAY 20
#define CHANNEL_SWITCH 0
#define RANGINGS_PER_ROTATION 100
#define MAX_TIMEOUTS 20

/** Physical layers default paramaters */
const int8_t defaultTxPower = 13;


const RadioLoRaSpreadingFactors_t defaultSf = LORA_SF9;
const RadioLoRaBandwidths_t defaultBw = LORA_BW_1600;
const RadioLoRaCodingRates_t defaultCr = LORA_CR_4_8;


#define DEFAULT_TX_POWER 13 /**<Default Tx power, in Dbm */
#define RANGING_ADDRESS_REGISTER_LAST_BYTE 0x919
#define ADDRESS_NUMBER_OF_BITS_REGISTER 0x931

#define WAIT_INPUT
#define DEBUG 0
#define VERBOSE 0

#define RNG_TIMER_MS 384
#define RNG_TIMEOUT 500
#define debug_print(fmt, ...) \
            do { if (DEBUG) printf(fmt, ##__VA_ARGS__); } while (0)
#define verbose_print(fmt, ...) \
            do { if (VERBOSE) printf(fmt, ##__VA_ARGS__); } while (0)

#define ADVANCED_RANGING_OPCODE 0x9A
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

/** function prototypes */
bool setPreambleLength(PacketParams_t params, int length);
void initRadio();
void ping();
void onTxTimeout();
void onReceptionTimeout();
void onRangingDone(IrqRangingCode_t code);
void onRxError(IrqErrorCode_t code);
void onRxDone();
void onTxDone();
void getRangingResults();

Timer rangingClock;
int rangingCounter = 0;


DigitalOut TX_LED( A4 );
enum DeviceType{
    MASTER_DEVICE,
    SLAVE_DEVICE
};
DeviceType myType; 

enum RangingEvent {
    NONE,
    TIMEOUT,
    SLAVE_DONE,
    MASTER_DONE
};
bool rangingDone = true;
RangingEvent lastEvent = TIMEOUT;


/* Radio buffer */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

/** Callbacks **/

RadioCallbacks_t Callbacks =
{
    &onTxDone,             // syncWordDone        // txDone
    &onRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &onTxTimeout,    // txTimeout
    &onReceptionTimeout,     // rxTimeout
    &onRxError,       // rxError
    &onRangingDone,   // rangingDone
    NULL,             // cadDone
};

/** Peripherals - Serial USB and SX1280 Radio **/
BufferedSerial pc( USBTX, USBRX );
SX1280Hal Radio( D11, D12, D13, D10, D7, D9, NC, NC, A0, &Callbacks );
ModulationParams_t modulationParams;


/** Radio physical layer paramaters **/
RadioLoRaSpreadingFactors_t currentSf = defaultSf;
RadioLoRaBandwidths_t currentBw = defaultBw;
// int requestDelay = 5 + (INTER_RANGING_DELAY) << ( (currentSf >> 4) - 5);
int requestDelay = INTER_RANGING_DELAY;
int channelIdx = 0;
uint8_t frameCounter = 0;
uint16_t calibration = 13215;
uint8_t pLength = 8;
bool resultPending = false;
int successiveTimeouts = 0;


// void setAdvancedRanging()
// {
//     uint8_t role = RADIO_RANGING_ROLE_SLAVE;
//     Radio.WriteCommand(RADIO_SET_RANGING_ROLE, &role, 1);
//     Radio.WriteCommand(ADVANCED_RANGING_OPCODE, &role, 1);
// }
const uint8_t RANGING_ADDR[] =
{
    0x00,
    0x01,
    0x02,
    0x03,
};

int getBW(RadioLoRaBandwidths_t bw)
{
    int bw_as_int;
    switch (bw)
    {
    case LORA_BW_0200:
        bw_as_int = 200;
        break;

    case LORA_BW_0400:
        bw_as_int = 400;
        break;

    case LORA_BW_0800:
        bw_as_int = 800;
        break;

    case LORA_BW_1600:
        bw_as_int = 1600;
        break;
    }

    return(bw_as_int);
}

int main() {
    uint8_t role; 
    pc.set_baud(BAUDRATE);

#if !defined(DEVICE_ID) || defined(WAIT_INPUT)
    bool ready = false;
    while (!ready) 
    {
        while (!pc.readable());
        ready = true;
        pc.read(&role, 1);
            switch (role)
            {
            case 'm':
                myType = MASTER_DEVICE;
                printf("Starting as Master \r\n");
                break;
            case 's':
                myType = SLAVE_DEVICE;
                printf("Starting as Slave \r\n");
                break;
            default:
                printf("Wrong command received. Please send 'm' for Master and 's' for Slave");
                ready = false;
                break;
            }
    }
#else
    myType = (DEVICE_ID > 0)?SLAVE_DEVICE:MASTER_DEVICE;
#endif
    initRadio();
    ping();
    return 0;
}


void initRadio()
{
    verbose_print("Starting radio init\r\n");
    thread_sleep_for(1000); // wait for on board DC/DC start-up time
    Radio.Init();    
    Radio.SetStandby( STDBY_RC );
    Radio.SetRegulatorMode(USE_LDO);

    /** Buffer init **/
    memset( &Buffer, 0x00, BufferSize );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    

    modulationParams.Params.LoRa.SpreadingFactor = defaultSf;
    modulationParams.Params.LoRa.Bandwidth = defaultBw;
    modulationParams.Params.LoRa.CodingRate = defaultCr;
    
    Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    Radio.SetInterruptMode();
    
    verbose_print("Init completed...\r\n");
    
}

void setRadio(ModulationParams_t modulation)
{
    Radio.SetPacketType(PACKET_TYPE_RANGING);
    modulation.PacketType = PACKET_TYPE_RANGING;
    Radio.SetModulationParams(&modulation);
    PacketParams_t params;
    params.PacketType = PACKET_TYPE_RANGING;
    params.Params.LoRa.PreambleLength = 8;
    params.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    params.Params.LoRa.PayloadLength = 1;
    params.Params.LoRa.Crc = LORA_CRC_OFF;
    params.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

    Radio.SetPacketParams(&params);
    Radio.SetRfFrequency(Channels[channelIdx]);
    Radio.SetTxParams(DEFAULT_TX_POWER, RADIO_RAMP_20_US );
    Radio.SetRangingCalibration(calibration);
    switch( modulation.Params.LoRa.Bandwidth )
    {
        case LORA_BW_0400:
            requestDelay = RNG_TIMER_MS >> ( 0 + 10 - ( modulation.Params.LoRa.SpreadingFactor >> 4 ) );
            break;

        case LORA_BW_0800:
            requestDelay  = RNG_TIMER_MS >> ( 1 + 10 - ( modulation.Params.LoRa.SpreadingFactor >> 4 ) );
            break;

        case LORA_BW_1600:
            requestDelay  = RNG_TIMER_MS >> ( 2 + 10 - ( modulation.Params.LoRa.SpreadingFactor >> 4 ) );
            break;
    }    
}

bool setPreambleLength(PacketParams_t params, int length)
{
    bool ret = true;
    int exponent = 0;
    if (length > 0)
    {
        while ( (length & 1) == 0)
        {
            length >>= 1;
            exponent += 1;
        }
        if (length > 15) 
        {
            ret = false;
            debug_print("Cannot set this preamble length. Length should be n * 2^e, with n < 16 \r\n");
        }
        else {
            params.Params.LoRa.PreambleLength = (exponent * 16) + (uint8_t) length;
        }
    }
    else if (length == 0)
    {
        params.Params.LoRa.PreambleLength = 0;
    }
    else 
    {
        ret = false;
        debug_print("Cannot set a negative preamble length \r\n");
    }
    return(ret);
}

void setRangingAddress(uint8_t address)
{
    if (myType == MASTER_DEVICE)
    {
        uint8_t buf[4];
        for (int i = 0; i < 3; i++) 
        {
            buf[i] = 0;
        }      
        buf[4] = address;
        Radio.WriteRegister(REG_LR_REQUESTRANGINGADDR, buf, 4);
    }
    else {
        Radio.WriteRegister(RANGING_ADDRESS_REGISTER_LAST_BYTE, &address,1);
        /* settings the number of address bits to check to 8 */
        Radio.WriteRegister(ADDRESS_NUMBER_OF_BITS_REGISTER, 0);
    }
}



void setIRQs() {
    Radio.SetDioIrqParams(0xFFFF,
                            IRQ_RANGING_SLAVE_REQUEST_DISCARDED | IRQ_RANGING_SLAVE_RESPONSE_DONE | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT | IRQ_RX_TX_TIMEOUT,
                            IRQ_RADIO_NONE, 
                            IRQ_RADIO_NONE );    
}


void ping()
{
    double rawRangingRes;
    uint8_t frameCounter = 0;
    DeviceType role = myType;
    rangingClock.start();
    setRadio(modulationParams);
    setRangingAddress(RANGING_ADDR[0]);
    setIRQs();
    lastEvent = TIMEOUT;
    while (true)
    {
        if (lastEvent != NONE)
        {
            /* role rotation */
            if (lastEvent == TIMEOUT)
            {
                /* always going back to the primary goal when facing timeouts to ensure service continuity */
                role = myType;
                if (successiveTimeouts > MAX_TIMEOUTS)
                {
                    /* restarting whole process */
                    channelIdx = 0;
                    Radio.SetRfFrequency(Channels[channelIdx]);
                    rangingCounter = 0;
                }
            }
            else
            {
                successiveTimeouts = 0;
                /* otherwise switching roles */
                role = (role==MASTER_DEVICE)?SLAVE_DEVICE:MASTER_DEVICE; 
                debug_print("Switching roles...\r\n");
                if (role == myType) 
                {
                    rangingCounter += 1;
                    /* Channel rotation */
                    if ( CHANNEL_SWITCH && ( rangingCounter % RANGINGS_PER_ROTATION == 0) )
                    {
                        printf("Switching channel \r\n");
                        channelIdx = (channelIdx + 1) % CHANNELS;
                        Radio.SetRfFrequency(Channels[channelIdx]);
                    }
                }
            }      

            if (role == MASTER_DEVICE) {
                debug_print("Setting Master...\r\n");
                thread_sleep_for(requestDelay);
                rangingClock.reset();     
                rangingClock.start();
                Radio.SetTx((TickTime_t) {RADIO_TICK_SIZE_4000_US, 0x0000});     
                // hal_sleep();
            }

            if (role == SLAVE_DEVICE) {  
                debug_print("Setting Slave...\r\n");                 
                Radio.SetRx((TickTime_t){RADIO_TICK_SIZE_1000_US, 500});
                // if (resultPending)
                // {
                //     getRangingResults();
                //     resultPending = false;
                // }

                // hal_sleep();
                
            }
            lastEvent = NONE;
        }
        
    }
}

void getRangingResults()
{
    PacketStatus_t p;
    Radio.GetPacketStatus(&p);
    double fei = Radio.GetFrequencyError();
    double rawRangingRes = Radio.GetRangingResult(RANGING_RESULT_RAW);
    int8_t rssi = p.LoRa.RssiPkt;
    int8_t snr = p.LoRa.SnrPkt;
    printf("*%d.%d|%d|%d|%d|%u|%u|%d|%d|%u|%llu\r\n", 
            (int) INT(rawRangingRes), 
            (int) DEC(rawRangingRes), 
            (int) INT(fei), 
            rssi, 
            snr, 
            calibration, 
            Channels[channelIdx], 
            currentSf >> 4,
            getBW(currentBw),
            pLength,
            rangingClock.elapsed_time().count());   

    // printf("{\"distance\":%d.%d,\"fei\":%d, \"rssi\":%d, \"snr\":%d, \"calibration\":%u, \"frequency\":%u,\"sf\":%d, \"bw\":%d\r\n",
    //         (int) INT(rawRangingRes), 
    //         (int) DEC(rawRangingRes), 
    //         (int) INT(fei), 
    //         rssi, snr, 
    //         calibration, 
    //         Channels[channelIdx], 
    //         currentSf >> 4,
    //         getBW(currentBw));
}


/** Handlers */
void onTxTimeout()
{
    printf("TX timed out\r\n");
    rangingDone = true;
}

void onReceptionTimeout()
{
    printf("Reception timeout \r\n");
    rangingDone = true;
}

void onRangingDone(IrqRangingCode_t code)
{   
    rangingClock.stop();
    debug_print("Ranging result: %d\r\n", code);
    if ( (code == IRQ_RANGING_SLAVE_ERROR_CODE) || (code == IRQ_RANGING_MASTER_ERROR_CODE) )
    {
        printf("Ranging timeout: %d; %d; %d\r\n", code, rangingCounter, channelIdx);  
        lastEvent = TIMEOUT;
        successiveTimeouts++;
    }
    else if (code == IRQ_RANGING_SLAVE_VALID_CODE)
    {
        debug_print("Ranging success: %d\r\n", code); 
        lastEvent = SLAVE_DONE;
    }
    else if (code == IRQ_RANGING_MASTER_VALID_CODE)
    {
        debug_print("Ranging success: %d\r\n", code); 
        verbose_print("Elapsed time: %llu; Counter: %d\r\n", rangingClock.elapsed_time().count(), rangingCounter);
        lastEvent = MASTER_DONE;
        resultPending = true;
        getRangingResults();
    }
    rangingDone = true;
    Radio.ClearIrqStatus(0xFFFF);    
}

void onRxError(IrqErrorCode_t code)
{
    printf("RX error\r\n");
    rangingDone = true;
}

void onRxDone()
{
    printf("RX done\r\n");
    rangingDone = true;
}

void onTxDone()
{
    printf("TX done\r\n");
    rangingDone = true;
}