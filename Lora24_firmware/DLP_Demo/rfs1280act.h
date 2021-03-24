SPI RadioSpi(PA_7, PA_6, PA_5);//SX1280: MOSI, MISO, SCK
AnalogOut dac(PA_4);
Ticker MyTicker;

AnalogIn   ain(PA_1);

DigitalIn bsy       ( PB_3 ); //SX1280: BUSY
DigitalIn ALL3IRQ   ( PB_4 ); //SX1280: DIO1 | DIO2 | DIO3
DigitalIn SW1       ( PB_8 ); 
DigitalIn SW2       ( PB_6 ); 
DigitalIn SW3       ( PC_0 ); 
DigitalIn SW4       ( PB_9 ); 
DigitalIn SW5       ( PC_1 ); 
DigitalIn SW6       ( PB_0 ); 

DigitalOut ANT_SEL  ( PC_7 );
DigitalOut RadioNss ( PA_8 );//SX1280: NSS              
DigitalOut LRS      ( PA_10 );
DigitalOut LCS      ( PB_5 );
DigitalOut LCLK     ( PA_9 );
DigitalOut LSI      ( PB_10 );

DigitalOut TEST_LED( PC_8 );//CN10 pin 2 on Nucleo 

DigitalInOut RadioReset( PA_0 );         

//#define FIRMWARE_VERSION    ( ( char* )"Firmware Version: PTTSF6v1.0" )//original January 2018
#define FIRMWARE_VERSION    ( ( char* )"Firmware Version: v1.2" )//Added Star network & Learn of multiple IDs 2-2-2018

#define EEPROM_B0              0 
#define EEPROM_B1              1 
#define EEPROM_B2              2 
#define EEPROM_B3              3 
#define EEPROM_B4              4 
#define EEPROM_B5              5 

#define EEPROM_C0              0 
#define EEPROM_C1              2 
#define EEPROM_C2              4 


#define XTAL_FREQ              52000000
#define FREQ_STEP              ( ( double )( XTAL_FREQ / pow( 2.0, 18.0 ) ) )
#define WaitOnBusy( )          while( bsy.read() == 1 ){ }
#define STDBY_RC               0x00 //set standby command parameter
#define STDBY_XOSC             0x01 //set standby command parameter

#define MY_DATA_EEPROM_BASE       ( ( uint32_t )0x08080000U ) 
#define EEPROM_MYID            0 //base address - 2 bytes long
#define EEPROM_YOURID          4 //base address - 2 bytes long
#define EEPROM_MODMODE         8 //base address - 1 byte long


#define RX_TIMEOUT_TICK_SIZE   RADIO_TICK_SIZE_1000_US
#define IRQ_RADIO_ALL          0xFFFF

#define RXTXDELAYCALOFFSET     13430
#define RANGINGSLAVE           0
#define RANGINGMASTER          1

#define RAWRESULT              0
#define AVERAGERESULT          1
#define DEBIASEDRESULT         2
#define FILTEREDRESULT         3

#define MYRXTIMEOUT_LONG       4000000 

#define GFSK                     0  
#define LORA                     1
#define RANGING                  2
#define FLRC                     3

#define DEFAULTSPREADINGFACTOR   12
#define RANGINGSPREADINGFACTOR6  6
#define RANGINGSPREADINGFACTOR10 10
#define PTTSPREADINGFACTOR       6
#define PTTPACKETSIZE            255

#define STARTDELAY             6500

#define TICKERSF6                0.006
#define TICKERSF10               0.050

#define RSSITHRESHOLD            -80

#define REG_LR_SYNCWORDBASEADDRESS1                 0x09CE
#define REG_LR_SYNCWORDBASEADDRESS2                 0x09D3
#define REG_LR_SYNCWORDBASEADDRESS3                 0x09D8
#define REG_LR_CRCSEEDBASEADDR                      0x09C8
#define REG_LR_CRCPOLYBASEADDR                      0x09C6


#define PLAY                   0
#define RECORD                 1
#define RANGINGCHCHANGE        2

#define LOW                    2402000000
#define MID                    2440000000
#define HIGH                   2480000000

#define DEFAULTMODGFSKP1       6 //Bandwidth (MHz DSB)         P86
#define DEFAULTMODGFSKP2       4  //Modindex  
#define DEFAULTMODGFSKP3       2  //BT                          P87

#define DEFAULTMODLORAP1       DEFAULTSPREADINGFACTOR //Spreading factor            P111
#define DEFAULTMODLORAP2       1  //Bandwidth [kHz]             P112 
#define DEFAULTMODLORAP3       1  //Coding rate

#define DEFAULTMODFLRCP1       3 //Bitrate (Mb/s)  Bandwidth (MHz DSB)  P32  P102                        
#define DEFAULTMODFLRCP2       1 //Coding rate                          P103
#define DEFAULTMODFLRCP3       2 //BT   Bandwidth-Time bit period product




#define DEFAULTPACKETGFSKP1    8 //Preamble length in bits       P87
#define DEFAULTPACKETGFSKP2    5 //Sync Word size in bytes       P88
#define DEFAULTPACKETGFSKP3    2 //Sync Word combination to use
#define DEFAULTPACKETGFSKP4    5 //Packet Length mode
#define DEFAULTPACKETGFSKP5    12 //Payload length in bytes
#define DEFAULTPACKETGFSKP6    4 //CRC type
#define DEFAULTPACKETGFSKP7    1 //Whitening mode

#define DEFAULTPACKETLORAP1    0 //Preamble length in symbols    P113
#define DEFAULTPACKETLORAP2    1 //Header mode
#define DEFAULTPACKETLORAP3    6 //PayloadLength
#define DEFAULTPACKETLORAP4    1 //CRC mode
#define DEFAULTPACKETLORAP5    1 //LoRa IQ swap
#define DEFAULTPACKETLORAP6    0 //N/A
#define DEFAULTPACKETLORAP7    0 //N/A

#define DEFAULTPACKETFLRCP1    8 //Preamble length in bits       P103       was 6
#define DEFAULTPACKETFLRCP2    3 //Sync Word size in bytes       P104  
#define DEFAULTPACKETFLRCP3    2 //Sync Word combination to use         
#define DEFAULTPACKETFLRCP4    2 //Packet Length mode            P105
#define DEFAULTPACKETFLRCP5    6 //Payload length in bytes - must be 6-127...
#define DEFAULTPACKETFLRCP6    4 //CRC type
#define DEFAULTPACKETFLRCP7    2 //In FLRC packet type, it is not possible to enable whitening. You must always set the value of packetParam7 to disabled.



typedef enum
{
    RADIO_TICK_SIZE_0015_US                 = 0x00,
    RADIO_TICK_SIZE_0062_US                 = 0x01,
    RADIO_TICK_SIZE_1000_US                 = 0x02,
    RADIO_TICK_SIZE_4000_US                 = 0x03,
}RadioTickSizes_t;

typedef struct TickTime_s
{
    RadioTickSizes_t Step;                                  //!< The step of ticktime
    /*!
     * \brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t NbSteps;
}TickTime_t;
	
//DLP-RFS1280ACT packet types 
#define     LOCATOR_PING_PACKET            0x57
#define     LEARN_BROADCAST_PACKET         0x58
#define     STAY_QUIET_PACKET              0x59
#define     LISTEN_PACKET                  0x5A
#define     SIMPLE_PING_PACKET             0x21
#define     PTT_AUDIO_PACKET               0x22
#define     RANGING_PACKET                 0x23



//Represents all possible opcode understood by the radio
typedef enum RadioCommands_u
{
    RADIO_GET_STATUS                        = 0xC0,
    RADIO_WRITE_REGISTER                    = 0x18,
    RADIO_READ_REGISTER                     = 0x19,
    RADIO_WRITE_BUFFER                      = 0x1A,
    RADIO_READ_BUFFER                       = 0x1B,
    RADIO_SET_SLEEP                         = 0x84,
    RADIO_SET_STANDBY                       = 0x80,
    RADIO_SET_FS                            = 0xC1,
    RADIO_SET_TX                            = 0x83,
    RADIO_SET_RX                            = 0x82,
    RADIO_SET_RXDUTYCYCLE                   = 0x94,
    RADIO_SET_CAD                           = 0xC5,
    RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
    RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    RADIO_SET_PACKETTYPE                    = 0x8A,
    RADIO_GET_PACKETTYPE                    = 0x03,
    RADIO_SET_RFFREQUENCY                   = 0x86,
    RADIO_SET_TXPARAMS                      = 0x8E,
    RADIO_SET_CADPARAMS                     = 0x88,
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    RADIO_SET_MODULATIONPARAMS              = 0x8B,
    RADIO_SET_PACKETPARAMS                  = 0x8C,
    RADIO_GET_RXBUFFERSTATUS                = 0x17,
    RADIO_GET_PACKETSTATUS                  = 0x1D,
    RADIO_GET_RSSIINST                      = 0x1F,
    RADIO_SET_DIOIRQPARAMS                  = 0x8D,
    RADIO_GET_IRQSTATUS                     = 0x15,
    RADIO_CLR_IRQSTATUS                     = 0x97,
    RADIO_CALIBRATE                         = 0x89,
    RADIO_SET_REGULATORMODE                 = 0x96,
    RADIO_SET_SAVECONTEXT                   = 0xD5,
    RADIO_SET_AUTOTX                        = 0x98,
    RADIO_SET_AUTORX                        = 0x9E,
    RADIO_SET_LONGPREAMBLE                  = 0x9B,
    RADIO_SET_UARTSPEED                     = 0x9D,
    RADIO_SET_RANGING_ROLE                  = 0xA3,
    RADIO_GET_SILICON_VERSION               = 0x14,
}RadioCommands_t;

#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB    0x954
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK   0x000FFFFF

//Ranging raw factor                 SF5     SF6     SF7     SF8     SF9     SF10
const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };

		
#define CHANNELS 40
#define READINGSPERCHANNEL 10

/*
 * Frequency look up table :
 * To avoid Wifi channels, 40 Bluetooth channels are defined below (they already
 * avoid Wifi common channels) : from 2402 MHz to 2480 MHz, step 2 MHz.
 * User can define channel count for Ranging run, and it is optimized to have
 * several frequencies in the largest band as possible. Also the 40 frequencies 
 * are generated by random sorting to preferate the 10 first in the largest band
 * as possible (10 is the shortest channel count the user can choose).
 */
const uint32_t Channels[] =
{
 2450000000,
 2476000000,
 2436000000,
 2430000000,
 2468000000,
 2458000000,
 2416000000,
 2424000000,
 2478000000,
 2456000000,
 2448000000,
 2462000000,
 2472000000,
 2432000000,
 2446000000,
 2422000000,
 2442000000,
 2460000000,
 2474000000,
 2414000000,
 2464000000,
 2454000000,
 2444000000,
 2404000000,
 2434000000,
 2410000000,
 2408000000,
 2440000000,
 2452000000,
 2480000000,
 2426000000,
 2428000000,
 2466000000,
 2418000000,
 2412000000,
 2406000000,
 2470000000,
 2438000000,
 2420000000,
 2402000000,
};



