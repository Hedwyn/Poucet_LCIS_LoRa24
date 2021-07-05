#include <stdio.h>
#include <stdlib.h>
#include "mbed.h"
#include "SX1280.h"
#include "SX1280-hal.h"
#include "RangingCorrection.h"
#include <math.h>
#include "chrono"
#include "FreqLUT.h"
#include "filters.h"


/** 
 * Compile with
 * mbed compile --source C:\Users\pestourb\Documents\Poucet\Mbed-test\SX1280\BUILD\libraries\SX1280\NUCLEO_L432KC\GCC_ARM-RELEASE --source RangingDemo -NRangingDemo
**/

#define BAUDRATE 115200
#define BUFFER_SIZE 255
// #define SPEED_OF_LIGHT 299705000.
#define FLOAT_DECIMALS 4
#define FLOAT_MAX_DIGITS 20
#define FLOAT_EXPONENT pow(10, FLOAT_DECIMALS)
#define INT(_FLOAT) (trunc(_FLOAT))
#define DEC(_FLOAT) (FLOAT_EXPONENT * fabs(_FLOAT - INT(_FLOAT)) )

#define INTER_RANGING_DELAY 20     /**<Master sleep time in-between each ranging*/
#define CHANNEL_SWITCH 0           /**<If enabled, the transceiver will switch channels after a given number of rangings*/
#define RANGINGS_PER_ROTATION 10    /**<Number of ranging protocols held before switching channels. Does nothing if channel rotation is off*/
#define MAX_TIMEOUTS 10              /**<After that number of timeouts the transceiver will go back to default parameters*/
#define CHANNEL_INCREMENT 1         /**<Value by which the channel index is incremented at each channel rotation. If superior to 1, some channels will be skipped */
#define CHANNEL_WIDTH 1000000       /**<Bandwidth of LoRa 2.4 Ghz channels */
#define ORDER_CHANNELS 1             /**<When enabled, the channel rotation goes linearily from lowest to highest frequency. Otherwise, the channel order is a random permutation of available channels */
#define BOTTOM_FREQUENCY 2400000000 /**<Center frequency of the lowest channel available for LoRa 2.4 GHz */
#define SYMBOL_LENGTH_SF5 197       /**<Symbol length for SF5 in microseconds. Increasing SF by 1 doubles that length */
// #define SPEED_OF_LIGHT 3E8          /**<Speed of light in m/s*/
#define ENABLE_FILTERING 0 
#define SINGLE_SLAVE_MODE   1         /**<If enabled, only one slave is defined. Node IDs above 1 will be started as passive slaves performing differential ranging */
#define DEFAULT_TX_POWER 6 /**<Default Tx power, in Dbm */
#define DEFAULT_LNA_GAIN 8
#define ADDRESS_NUMBER_OF_BITS_REGISTER 0x931
#define MOVING_MEDIAN_LENGTH 20
// #define WAIT_INPUT
#define DEBUG 0
#define VERBOSE 0

#define RNG_TIMER_MS 800 //384
#define RNG_TIMEOUT 400
#define debug_print(fmt, ...) \
            do { if (DEBUG) printf(fmt, ##__VA_ARGS__); } while (0)
#define verbose_print(fmt, ...) \
            do { if (VERBOSE) printf(fmt, ##__VA_ARGS__); } while (0)
#define ADVANCED_RANGING_ROLE 0x01
#define ACCUMULATOR_LENGTH 10
#ifndef TOTAL_SLAVES
    #define TOTAL_SLAVES 1
#endif

#define TOTAL_DEVICES TOTAL_SLAVES + 1
#ifndef DEVICE_ID
    #define DEVICE_ID 0
#endif 



/** Physical layers default paramaters */
const int8_t defaultTxPower = 10;
const float SPEED_OF_LIGHT = 299705000.;
const float PI = 3.14159;

/** index of the current slave address */
int slaveIndex, masterIndex;
bool myTurn = true;


const RadioLoRaSpreadingFactors_t defaultSf = LORA_SF7;
const RadioLoRaBandwidths_t defaultBw = LORA_BW_1600;
const RadioLoRaCodingRates_t defaultCr = LORA_CR_4_8;

/*!
 * \brief Ranging raw factors
 *                                  SF5     SF6     SF7     SF8     SF9     SF10
 */
const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13276,  13275,  13275,  13230,  13167  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };

uint16_t calibration;
double feiTable[TOTAL_DEVICES] = {};
MovingMedian<20> *filters[TOTAL_DEVICES];

class Stack
{
private:
    double accumulator[ACCUMULATOR_LENGTH];
    int index;
    bool full;
public:
    Stack() 
    {
        index = 0;
        full = false;
        for (int i =0; i < ACCUMULATOR_LENGTH; i++)
        {
            accumulator[i] = 0;
        }
    }
    void insert(double element)
    {
        accumulator[index] = element;
        index++;
        if (index == ACCUMULATOR_LENGTH)
        {
            full = true;
            index = 0;
        }
    }
    int getDepth()
    {
        int depth;
        if (full)
        {
            depth = ACCUMULATOR_LENGTH;
        }
        else
        {
            depth = index;
        }
        return(depth);
    }   

    double getMean()
    {
        double sum = 0;
        int depth = getDepth();
        for (int i = 0; i < getDepth(); i++)
        {
            sum += accumulator[i];
        }
        return(sum / depth);
    }
};
Stack distanceStack = Stack();
Stack correctedDistanceStack = Stack();

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
void getRangingResults(bool differential = false);
double convertFeiToPpm(double fei);
double correctFei(double distance, double fei);
void initAddresses();
void setMasterAddress(int masterIdx, int slaveIdx);
void setSlaveAddress(int slaveIdx);
void setBroadcast();
int getMasterIndex();
int getSlaveIndex();
int getBW(RadioLoRaBandwidths_t bw);
void acceptAllRequests();

const int WATCHDOG_TIMEOUT = 1000000; // us
Timer rangingClock;
Timer sysClock;
Timer watchdog;
bool clock_on = false;
int rangingCounter = 0;
int8_t txPower = DEFAULT_TX_POWER;
int8_t lnaGain = DEFAULT_LNA_GAIN;
int lnaGainValues[] = {0, 0, 6, 12, 18, 24, 30, 36, 42, 46, 48, 50, 52, 54};

enum DeviceType{
    MASTER_DEVICE,
    SLAVE_DEVICE,
    PASSIVE_SLAVE_DEVICE
};
DeviceType myType, role;



enum RangingEvent {
    NONE,
    TIMEOUT,
    REQUEST_IGNORED,
    SLAVE_DONE,
    PASSIVE_SLAVE_DONE,
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
#ifdef NUCLEO_L432KC
    SX1280Hal Radio( D11, D12, D13, D10, D7, D9, NC, NC, A0, &Callbacks ); // L432_KC
#elif defined(NUCLEO_L476RG)
    DigitalOut ANT_SW( A3 ); // antenna diversity switch
    SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, &Callbacks );
#elif defined(IMST282A)
    SX1280Hal Radio( PA_7, PA_6, PA_5, PA_4, PB_0, PB_1, NC, NC, PA_1, &Callbacks );
#endif
ModulationParams_t modulationParams;

const uint8_t RANGING_ADDR[] =
{
    0x01,
    0x02,
    0x03,
    0x04,
};

/** Radio physical layer paramaters **/
RadioLoRaSpreadingFactors_t currentSf = defaultSf;
RadioLoRaBandwidths_t currentBw = defaultBw;
// int requestDelay = 5 + (INTER_RANGING_DELAY) << ( (currentSf >> 4) - 5);
int requestDelay = INTER_RANGING_DELAY;
int channelIdx = 0;
uint8_t frameCounter = 0;
uint8_t pLength = 8;
bool resultPending = false;
int successiveTimeouts = 0;
uint32_t channelFreq = BOTTOM_FREQUENCY;

// #define DUMP_CORRECTION_VALUES
#ifdef DUMP_CORRECTION_VALUES
void dumpRssiCalibration()
{
    static const RadioLoRaSpreadingFactors_t SF_LIST[] = {LORA_SF5, LORA_SF6, LORA_SF7, LORA_SF8, LORA_SF9, LORA_SF10};
    static const RadioLoRaBandwidths_t BW_LIST[] = {LORA_BW_0400, LORA_BW_0800, LORA_BW_1600};
    double correction;
    for (int i = 0; i < 6; i++) // SF
    {
        for (int j = 0; j < 3; j++) // BW
        {
            for (int k = 0; k < 160; k++)
            {
                correction = Sx1280RangingCorrection::GetRangingCorrectionPerSfBwGain(
                    SF_LIST[i],
                    BW_LIST[j],
                    128
                );
                correction = RangingCorrectionPerSfBwGain[i][j][k];
                printf("{\"SF\":%d, \"BW\":%d, \"RSSI\":%d, \"distance\":%d.%d}\r\n",(int) SF_LIST[i] >> 4, getBW(BW_LIST[j]), k, (int) INT(correction), (int) DEC(correction));
                thread_sleep_for(5);
            }
        }
    }
    exit(0);
}
#endif

double correctLnaGain(double distance, int lnaGain)
{
    int gain = lnaGainValues[lnaGain];
    if (lnaGain >= 10)
    {
        distance -= 5.8;
    }
    else if (lnaGain >= 7)
    {
        distance -= 3;
    }
    else {
        distance -= 0.2 * (6 - lnaGain);
    }
    return(distance);
}

double convertFeiToPpm(double fei)
{
    double freq = Channels[channelIdx];
    /* converting fei to ppm */
    double ppm = (1E6 * fei / freq);
    return(ppm);
}

double computeChannelPhase(double distance, double frequency, double *reminder)
{
    double wavelength = SPEED_OF_LIGHT / frequency;
    double phaseReminder = distance;
    double phase;
    while (phaseReminder > wavelength)
    {
        phaseReminder -= wavelength;
    }
    *reminder = phaseReminder;
    phase = 2 * PI * (phaseReminder / wavelength); 
    return(phase);
}

double correctFei(double distance, double fei)
{
    double skew = convertFeiToPpm(-fei);
    /* time-of-flight error = (Reply time) * skew; reply time is 2 symbols; symbol length depends on SF */
    /* converting SF enum to actual value */
    int sf = currentSf >> 4;
    double halfReplyTime = pow(2, (sf - 5)) * SYMBOL_LENGTH_SF5; /* in us */
    double tofError = halfReplyTime * skew; /* tof error for a one-way trip in ps (us * ppm) */
    double distanceError = tofError * SPEED_OF_LIGHT * 1E-12; /* in m */
    // printf("distance: %d.%d, distance Error = %d.%d\r\n",  (int) INT(distance), (int) DEC(distance), (int) INT(distanceError), (int) DEC(distanceError));
    return(distance - distanceError);
}

int getMasterIndex(uint32_t receivedAddress)
{
    /* master address is the third byte of the received address */
    uint8_t masterAddress = (receivedAddress >> 8) & 0x000000FF;
    int i = 0, ret = -1;
    while ( (i < TOTAL_DEVICES) && ret == -1)
    {
        if (RANGING_ADDR[i] == masterAddress)
        {
            ret = i;
        }
        i++;
    }
    return(ret);
}

int getSlaveIndex(uint32_t receivedAddress)
{
    /* master address is the third byte of the received address */
    uint8_t slaveAddress = receivedAddress  & 0x000000FF;
    int i = 0, ret = -1;
    while ( (i < TOTAL_DEVICES) && ret == -1)
    {
        if (RANGING_ADDR[i] == slaveAddress)
        {
            ret = i;
        }
        i++;
    }
    return(ret);
}

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
#if defined(NUCLEO_L476RG)
    ANT_SW = 1;
#endif

#ifdef WAIT_INPUT
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
                #ifdef DUMP_CORRECTION_VALUES
                    dumpRssiCalibration();
                #endif
                printf("Starting as Master \r\n");
                break;
            case 's':
                myType = SLAVE_DEVICE;
                printf("Starting as Slave \r\n");
                break;
            case 'p':
                myType = PASSIVE_SLAVE_DEVICE;
                printf("Starting as Passive Slave \r\n");
                break;
            default:
                printf("Wrong command received. Please send 'm' for Master, 's' for Slave, 'p' for Passive Slave \r\n");
                ready = false;
                break;
            }
    }
#else
    #if DEVICE_ID == 0
        myType = MASTER_DEVICE;
    #elif (DEVICE_ID == 1) || !SINGLE_SLAVE_MODE
        myType = SLAVE_DEVICE;
    #else 
        myType = PASSIVE_SLAVE_DEVICE;
    #endif

#endif
    slaveIndex = (myType == MASTER_DEVICE)?TOTAL_SLAVES:0;
    role = myType;
    initRadio();
    ping();
    return 0;
}


void initRadio()
{
    printf("Starting radio init\r\n");
    printf("Sleep\r\n");

    thread_sleep_for(1000); // wait for on board DC/DC start-up time
    printf("Init\r\n");

    Radio.Init();
    printf("Init done\r\n");

    
    Radio.SetStandby( STDBY_RC );
    Radio.SetRegulatorMode(USE_LDO);
    /** Buffer init **/
    memset( &Buffer, 0x00, BufferSize );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );  
    printf("Buffer done\r\n");
    modulationParams.Params.LoRa.SpreadingFactor = currentSf;
    modulationParams.Params.LoRa.Bandwidth = defaultBw;
    modulationParams.Params.LoRa.CodingRate = defaultCr;
    
    Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    Radio.EnableManualGain();
    Radio.SetManualGainValue(lnaGain);

    Radio.SetInterruptMode();
    printf("IRQ Set\r\n");
    
    verbose_print("Init completed...\r\n");
    role = myType;
    initAddresses();
    
}

void setRadio(ModulationParams_t modulation)
{
    Radio.SetPacketType(PACKET_TYPE_RANGING);
    modulation.PacketType = PACKET_TYPE_RANGING;
    Radio.SetModulationParams(&modulation);
    PacketParams_t params;
    params.PacketType = PACKET_TYPE_RANGING;
    // setPreambleLength(params, 8);
    params.Params.LoRa.PreambleLength = 8;
    params.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    params.Params.LoRa.PayloadLength = 1;
    params.Params.LoRa.Crc = LORA_CRC_ON;
    params.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

    Radio.SetPacketParams(&params);
    if (ORDER_CHANNELS)
    {
        Radio.SetRfFrequency(channelFreq);
    }
    else 
    {
        Radio.SetRfFrequency(Channels[channelIdx]);
    }
    Radio.SetTxParams(DEFAULT_TX_POWER, RADIO_RAMP_20_US );
    int sfIndex = ((int) defaultSf >> 4) - 5;
    switch( modulation.Params.LoRa.Bandwidth )
    {
        case LORA_BW_0400:
            // requestDelay = RNG_TIMER_MS >> ( 0 + 10 - ( modulation.Params.LoRa.SpreadingFactor >> 4 ) );
            calibration = RNG_CALIB_0400[sfIndex];
            break;

        case LORA_BW_0800:
            // requestDelay  = RNG_TIMER_MS >> ( 1 + 10 - ( modulation.Params.LoRa.SpreadingFactor >> 4 ) );
            calibration = RNG_CALIB_0800[sfIndex];
            break;

        case LORA_BW_1600:
            // requestDelay  = RNG_TIMER_MS >> ( 2 + 10 - ( modulation.Params.LoRa.SpreadingFactor >> 4 ) );
            calibration = RNG_CALIB_1600[sfIndex];
            break;
    }    
    // calibration = 13230;
    requestDelay = INTER_RANGING_DELAY;
    if (myType == MASTER_DEVICE)
    {
        Radio.SetRangingCalibration(calibration);
    }
    else
    {
        Radio.SetRangingCalibration(calibration);
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
/**
 * @remark Note that setting 0x00000000 as address will lead the node to ignore the address, thus 0 should be avoided unless that's the wanted behavior 
 */
void setMasterAddress(int masterIdx, int slaveIdx)
{
    uint8_t buf[4] =  {};
    /* settings the number of address bits to check to 8 */
    /* Note that the bytes are checked from fourth to first byte of the buffer */
    buf[2] = RANGING_ADDR[masterIdx];
    buf[3] = RANGING_ADDR[slaveIdx];
    printf("Programmed adress: %02x%02x%02x%02x\r\n", buf[0], buf[1], buf[2], buf[3]);
    Radio.WriteRegister(REG_LR_REQUESTRANGINGADDR, buf, 4);
    masterIndex = masterIdx;
    slaveIndex = slaveIdx;
}

void setSlaveAddress(int slaveIdx)
{
    uint8_t buf[4] =  {};
    buf[3] = RANGING_ADDR[slaveIdx];
    Radio.WriteRegister(REG_LR_DEVICERANGINGADDR, buf, 4);
}

void acceptAllRequests()
{
    uint8_t buf[4] =  {0};
    // Radio.WriteRegister(ADDRESS_NUMBER_OF_BITS_REGISTER, 0);
    /* setting request address to 0 will accept all requests regardless of the address */
    Radio.WriteRegister(REG_LR_DEVICERANGINGADDR, buf, 4);

}

void setBroadcast()
{
    uint8_t buf[4] =  {};
    /* setting request address to 0 will accept all requests regardless of the address */
    Radio.WriteRegister(REG_LR_REQUESTRANGINGADDR, buf, 4);
}

void initAddresses()
{
    /* settings the number of address bits to check to 8 */
    /* Note that the bytes are checked from fourth to first byte of the buffer */
    Radio.WriteRegister(ADDRESS_NUMBER_OF_BITS_REGISTER, 0);
    switch (role)
    {
    case MASTER_DEVICE:
        printf("Init master address, %d, %d, %d\r\n", DEVICE_ID, TOTAL_SLAVES, RANGING_ADDR[1]);
        // acceptAllRequests();
        setSlaveAddress(0);
        setMasterAddress(0, 1);
        break;
    case SLAVE_DEVICE:
        printf("Init slave address, %d, %d, %d\r\n", DEVICE_ID, TOTAL_SLAVES, RANGING_ADDR[DEVICE_ID]);
        setMasterAddress(DEVICE_ID, 0);
        setSlaveAddress(DEVICE_ID);
        break;
    case PASSIVE_SLAVE_DEVICE:
        printf("Accepting all requests\r\n");
        setMasterAddress(DEVICE_ID, 0);
        setSlaveAddress(DEVICE_ID);
        // acceptAllRequests();
        break;
    }
}

void setIRQs() {
    if (role == PASSIVE_SLAVE_DEVICE)
    {
        /* In advanced ranging mode, the ranging completed interrrupt replaced preamble detected */
        uint16_t mask = IRQ_PREAMBLE_DETECTED | IRQ_RX_TX_TIMEOUT;
        Radio.SetDioIrqParams(  mask,
                                mask,
                                IRQ_RADIO_NONE, 
                                IRQ_RADIO_NONE );  
    } 
    else 
    {
        uint16_t mask = (IRQ_RANGING_SLAVE_REQUEST_DISCARDED | IRQ_RANGING_SLAVE_RESPONSE_DONE | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT | IRQ_RX_TX_TIMEOUT);
        Radio.SetDioIrqParams(  mask,
                                mask,
                                IRQ_RADIO_NONE, 
                                IRQ_RADIO_NONE );  
    }
}


void ping()
{
    double rawRangingRes;
    uint8_t frameCounter = 0;
    rangingClock.start();
    setRadio(modulationParams);
    setIRQs();
    lastEvent = TIMEOUT;
    int slaveCounter = 0;
    watchdog.start();
    while (true)
    {
        if (watchdog.elapsed_time().count() > WATCHDOG_TIMEOUT)
        {
            lastEvent = TIMEOUT;
            printf("Watchdog reached timeout\r\n\n");
            watchdog.reset();
            // compensating for the missed ranging
            rangingCounter++;
        }
        thread_sleep_for(1);
        // printf("Watchdog time %llu\r\n", watchdog.elapsed_time().count() );

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
                    txPower = DEFAULT_TX_POWER;
                    lnaGain = DEFAULT_LNA_GAIN;
                    rangingCounter = 0;
                }
            }
            else if (lastEvent == REQUEST_IGNORED)
            {
                role = SLAVE_DEVICE;
            }
            else
            {
                successiveTimeouts = 0;
                
                if (role == myType) 
                {
                    // rangingCounter += 1;
                    /* Channel rotation */
                    ANT_SW = !ANT_SW;
                    if ( CHANNEL_SWITCH && ( rangingCounter % RANGINGS_PER_ROTATION == 0) )
                    {
                        // lnaGain = (lnaGain == 13)?DEFAULT_LNA_GAIN:(lnaGain + 1); 
                        // txPower =  6 + lnaGainValues[DEFAULT_LNA_GAIN] - lnaGainValues[lnaGain];
                        // txPower = (txPower > 12)?7:txPower + 1;
                        // txPower = (txPower == -18)?6:txPower - 6;
                        // Radio.SetManualGainValue(lnaGain);
                        // Radio.SetTxParams(txPower, RADIO_RAMP_20_US );
                        // printf("Switching channel \r\n");
                        channelIdx = (channelIdx + CHANNEL_INCREMENT) % CHANNELS;
                        if (ORDER_CHANNELS)
                        {
                            channelFreq = BOTTOM_FREQUENCY + channelIdx * CHANNEL_WIDTH;
                            Radio.SetRfFrequency(channelFreq);
                        }
                        else
                        {
                            channelFreq = Channels[channelIdx];
                            Radio.SetRfFrequency(channelFreq);
                        }
                        // if (myType == MASTER_DEVICE)
                        // {
                        //     Radio.SetRangingCalibration(calibration);            
                        // }
                    }
                }
            }      

            if (role == MASTER_DEVICE) {
                feiTable[slaveIndex] = Radio.GetFrequencyError();
                debug_print("Saving fei: %f\r\n", feiTable[slaveIndex]);
                if (myType == MASTER_DEVICE)
                { 
                    if (!SINGLE_SLAVE_MODE && (TOTAL_SLAVES > 1) && (slaveCounter++ % 1 == 0) )
                    {
                        slaveIndex = (slaveIndex % TOTAL_SLAVES) + 1;
                        setMasterAddress(0, slaveIndex);
                    }
                }
                verbose_print("Setting Master...[%d]\r\n", DEVICE_ID);
                thread_sleep_for(requestDelay);
                rangingClock.reset();     
                rangingClock.start();
                Radio.SetTx((TickTime_t) {RADIO_TICK_SIZE_4000_US, 0x0000});     

            }

            else if (role == SLAVE_DEVICE) {  
                verbose_print("Setting Slave...\r\n");                 
                Radio.SetRx((TickTime_t){RADIO_TICK_SIZE_1000_US, ( 2 *TOTAL_DEVICES) *  requestDelay});
                // hal_sleep();            
            }

            else if (role == PASSIVE_SLAVE_DEVICE)
            {
                verbose_print("Setting passive Slave...\r\n");    
                setIRQs();
                acceptAllRequests();   
                thread_sleep_for(2);
                Radio.SetAdvancedRanging( (TickTime_t){RADIO_TICK_SIZE_1000_US, 0xFFFF});
               // hal_sleep();                
            }
            lastEvent = NONE;
        }
        
    }
}

void getRangingResults(bool differential)
{
    PacketStatus_t p;
    Radio.GetPacketStatus(&p);
    // double fei = Radio.GetFrequencyError();
    double fei = feiTable[slaveIndex];
    double rawRangingRes = Radio.GetRangingResult(RANGING_RESULT_RAW);
    int8_t rssi = p.LoRa.RssiPkt;
    int8_t snr = p.LoRa.SnrPkt;
    char header = differential?'^':'*';
    double skew = convertFeiToPpm(fei);
    double correctedDistance = correctFei(rawRangingRes, fei);
    correctedDistance = correctLnaGain(correctedDistance, lnaGain);
    double phase, phaseDistance;
    phase = computeChannelPhase(rawRangingRes, Channels[channelIdx], &phaseDistance);

    if (ENABLE_FILTERING)
    {
        correctedDistance = correctFei(rawRangingRes, fei);
        filters[slaveIndex]->append(correctedDistance);
        correctedDistance = filters[slaveIndex]->compute();

        // rawRangingRes =  filters[slaveIndex]->compute();
        // distanceStack.insert(rawRangingRes);
        // correctedDistanceStack.insert(correctedDistance);
        // rawRangingRes = distanceStack.getMean();
        // correctedDistance = correctedDistanceStack.getMean();
    }
    // printf("Ranging delta threshold: %d\r\n", Radio.GetRangingPowerDeltaThresholdIndicator());
    // printf("%c%d|%d|%d.%d|%d.%d|%d|%d.%d|%d|%d|%u|%u|%d|%d|%d|%u|%llu\r\n", 
    //         header,
    //         masterIndex,
    //         slaveIndex,
    //         (int) INT(rawRangingRes), 
    //         (int) DEC(rawRangingRes), 
    //         (int) INT(correctedDistance),
    //         (int) DEC(correctedDistance),
    //         (int) INT(fei), 
    //         (int) INT(skew),
    //         (int) DEC(skew),
    //         rssi, 
    //         snr, 
    //         calibration, 
    //         Channels[channelIdx], 
    //         currentSf >> 4,
    //         getBW(currentBw),
    //         txPower,
    //         pLength,
    //         rangingClock.elapsed_time().count());   
    uint8_t deltaThold = Radio.GetRangingPowerDeltaThresholdIndicator();
    // double rssiCorrection = Sx1280RangingCorrection::GetRangingCorrectionPerSfBwGain(
    //     currentSf,
    //     currentBw,
    //     Radio.GetRangingPowerDeltaThresholdIndicator());
    // double correctedDistanceRssi = correctedDistance + rssiCorrection;

    // printf("Corrected distance (FEI): %d.%d\r\n\n", (int) INT(correctedDistance), (int) DEC(correctedDistance));
    // printf("Delta threshold: %d, Corrected distance (RSSI): %d.%d\r\n\n", (int) deltaThold, (int) INT(correctedDistanceRssi), (int) DEC(correctedDistanceRssi));



    printf("{\"distance\":%d.%d, \"corrected_distance\":%d.%d,\"fei\":%d, \"rssi\":%d, \"delta_rssi\":%d, \"snr\":%d, \"frequency\":%u, \"txPower\":%d, \"LNA_gain\":%u, \"phase\":%d.%d, \"phase_equivalent_distance\":%d.%d}\r\n",
            (int) INT(rawRangingRes), 
            (int) DEC(rawRangingRes), 
            (int) INT(correctedDistance),
            (int) DEC(correctedDistance),
            (int) INT(fei), 
            rssi, 
            deltaThold,
            snr,
            // calibration, 
            channelFreq,
            txPower,
            lnaGain,
            (int) INT(phase),
            (int) DEC(phase),   
            (int) INT(phaseDistance),
            (int) DEC(phaseDistance)
            // currentSf >> 4,
            // getBW(currentBw)
            );
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
    watchdog.reset();
    sysClock.stop();
    debug_print("Elapsed time: %llu\n", sysClock.elapsed_time().count());
    sysClock.reset();
    sysClock.start();
    if (role == myType)
        rangingCounter += 1;

    debug_print("Ranging done %d\r\n", code);
    if (code == IRQ_ADVANCED_RANGING_DONE)
    {
        uint32_t receivedAddress;
        Radio.DisableAdvancedRanging();
        receivedAddress = Radio.GetAdvancedRangingAddress();
        printf("Received address: %08x\r\n", (int) receivedAddress);
        masterIndex = getMasterIndex(receivedAddress);
        slaveIndex = getSlaveIndex(receivedAddress);
        printf("Last addresses received: master: %d slave: %d\r\n", masterIndex, slaveIndex);     
        getRangingResults(true);
        if (masterIndex != -1)
        {
            feiTable[masterIndex] = Radio.GetFrequencyError();
        }
        else
        {
            printf("/!\\ Unknown address received\r\n");
        }    

        if (!SINGLE_SLAVE_MODE && (masterIndex > 0) && ( (masterIndex % TOTAL_SLAVES) + 1 == DEVICE_ID) )
        {
            printf("It's my Turn ! \r\n");
            role = SLAVE_DEVICE;
            setIRQs();
            initAddresses();
        }  
        lastEvent = PASSIVE_SLAVE_DONE;
    }
    else if ( (code == IRQ_RANGING_SLAVE_ERROR_CODE) || (code == IRQ_RANGING_MASTER_ERROR_CODE))
    {
        printf("Ranging timeout: %d; %d; %d, %d\r\n", code, rangingCounter, channelIdx, (int) code);  
        lastEvent = TIMEOUT;
        successiveTimeouts++;
        rangingCounter--;
    }

    else if (code == IRQ_RANGING_SLAVE_INVALID_ADDRESS)
    {
        printf("That message was not for me\r\n");
        lastEvent = REQUEST_IGNORED;
        setMasterAddress(DEVICE_ID, 0);
        setSlaveAddress(DEVICE_ID);
        rangingCounter--;

    }

    else if (code == IRQ_RANGING_MASTER_VALID_CODE)
    {
        verbose_print("Elapsed time: %llu; Counter: %d\r\n", rangingClock.elapsed_time().count(), rangingCounter);
        lastEvent = MASTER_DONE;
        resultPending = true;
        getRangingResults();
        if ( !SINGLE_SLAVE_MODE && (myType == SLAVE_DEVICE) && (TOTAL_SLAVES > 1))
        // if (DEVICE_ID == 2)
        {
            printf("Im device %d, going passive\r\n", DEVICE_ID);
            role = PASSIVE_SLAVE_DEVICE;
            // acceptAllRequests();
        }
        else
        {
            role = SLAVE_DEVICE;
        }
    }

    else if (code == IRQ_RANGING_SLAVE_VALID_CODE)
    {
        lastEvent = SLAVE_DONE;
        role = MASTER_DEVICE;
    }
    rangingDone = true;
    if (myType != PASSIVE_SLAVE_DEVICE)
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