/*
 _____       ______  _____   ______
|_   _|    .' ___  ||_   _|.' ____ \
  | |     / .'   \_|  | |  | (___ \_|
  | |   _ | |         | |   _.____`.
 _| |__/ |\ `.___.'\ _| |_ | \____) |
|________| `.____ .'|_____| \______.'
    (C)2019 LCIS

Description: Main program

Maintainer: Viken Kojakian
*/
#include "main.h"
#include <sx1280-hal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "Timers.h"
#include "Eeprom.h"
#include "app_config.h"
#include <Utils.h>


enum DeviceType{
    MASTER_DEVICE,
    SLAVE_DEVICE
};

#ifndef TOTAL_SLAVES
    #define TOTAL_SLAVES 1
#endif

/* if no ID is defined, defaulting to Master */
#ifndef DEVICE_ID
    #define DEVICE_ID 0
#endif

DeviceType myType = (DEVICE_ID > 0)?SLAVE_DEVICE:MASTER_DEVICE;




const int RNG_ADDR_LIST[] = {DEMO_RNG_ADDR_1, DEMO_RNG_ADDR_2, DEMO_RNG_ADDR_3, DEMO_RNG_ADDR_4, DEMO_RNG_ADDR_5, DEMO_RNG_ADDR_6, DEMO_RNG_ADDR_7, DEMO_RNG_ADDR_8};

/*  COmpile-time assertions. Checks for inconsistencies in the provided IDs and number of slaves */
static_assert( DEVICE_ID <= TOTAL_SLAVES, "You cannot define a slave ID superior to the the total number of slaves !");
static_assert( size(RNG_ADDR_LIST) >= TOTAL_SLAVES, "There are not enough ranging addresses defined for the required number of slaves. Please define more ranging addresses \r\n");

#define DEVICE_MASTER
#ifndef DEVICE_MASTER
    #define DEVICE_SLAVE1
    // #define DEVICE_SLAVE2
    // #define DEVICE_SLAVE3
    // #define DEVICE_SLAVE4
    // #define DEVICE_SLAVE5
    // #define DEVICE_SLAVE6
#endif

#define BAUDRATE 115200


/*!
 * \brief Define IO for Unused Pin
 */
DigitalOut F_CS( D6 );      // MBED description of pin
DigitalOut SD_CS( D8 );     // MBED description of pin

DigitalOut MODE1( A1 );
DigitalOut MODE2( A2 );
DigitalOut MODE3( A3 );

BufferedSerial pc( USBTX, USBRX );



int total_slaves = TOTAL_SLAVES;
int slave_index = TOTAL_SLAVES - 1;
void PrintCompileSupportMessage( void );

/*
 * \brief Specify serial datarate for UART debug outputftgl
 brdb§y(tgfvc  vTVDCX
 t÷µ¬im:pÒ≈π©©©©©©©©©©©π‘≈µ¬¬¬¬†µ÷π®ﬁ¬dl+o:÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷µ±
 v                     fvgvoid baud( int baudrate ){
*/


extern SX1280Hal Radio;


uint8_t serialBuffer[USB_SERIAL_MAX_LENGTH];



char* printFloat(float f)
{
    static char floatString[FLOAT_MAX_DIGITS];
    float integer_part = trunc(f);
    float decimal_part = fabs((f - integer_part) * FLOAT_EXPONENT);
    sprintf(floatString, "%d.%d", (int) integer_part, (int) decimal_part) ;
    return (floatString);
}

int main( ){
    pc.set_baud(BAUDRATE);
    double f1 = 98.7, f2 = 1584.965;
    printf("Test %d.%d", (int) INT(f1), (int) DEC(f1) );
    char buffer[4];
    char message;
    char MST_CASE[] = "mst";
    char SLV_CASE[] = "slv";
    char RNG_CASE[] = "rng";
    char RN2_CASE[] = "rn2";
    char RN3_CASE[] = "rn3";
    char INF_CASE[] = "inf";
    char LED_CASE[] = "led";
    char AD1_CASE[] = "ad1";
    char AD2_CASE[] = "ad2";
    char AD3_CASE[] = "ad3";
    char AD4_CASE[] = "ad4";

    F_CS = 1;
    SD_CS = 1;

    MODE1 = 0;
	MODE2 = 0;
	MODE3 = 0;

    printf( "\n\rStarting poucet-lcis %s (%s)\n\r", FIRMWARE_VERSION, FIRMWARE_DATE );
    PrintCompileSupportMessage();

    EepromInit( );

    printf("Main ... EepromInit\n\r");

    printf("Main ... Initializing Demo\n\r");
    InitDemoApplication( );
    printf("Main ... Done\n\r");

    printf( "Radio version: 0x%x\n\r", Radio.GetFirmwareVersion( ) );


    EepromSetRangingDefaultSettings();



    /* ranging settings */
    Eeprom.EepromData.DemoSettings.RngRequestCount = DEFAULT_RNG_REQUEST_COUNT;
    // Eeprom.EepromData.DemoSettings.RngReqDelay = 80; // default 24

	Eeprom.EepromData.DemoSettings.HoldDemo = false;
	StopDemoApplication();

    Eeprom.EepromData.DemoSettings.RngAddress = RNG_ADDR_LIST[DEVICE_ID - 1];


    if (myType == SLAVE_DEVICE) 
    {
        printf("This device will run as Slave %d \n\r", DEVICE_ID);
        Eeprom.EepromData.DemoSettings.Entity = SLAVE;
        Eeprom.EepromData.DemoSettings.HoldDemo = false;
        StopDemoApplication();

        printf("Starting slave ranging application \r\n");
        RunDemoApplicationRanging();

    }

    else 
    {
        printf("This device will run as Master \n\r");
        Eeprom.EepromData.DemoSettings.Entity = MASTER;
        Eeprom.EepromData.DemoSettings.HoldDemo = false;
        StopDemoApplication();

        Eeprom.EepromData.DemoSettings.RngAddress = RNG_ADDR_LIST[slave_index];
        slave_index = (slave_index + 1) % TOTAL_SLAVES;
        printf("Starting master ranging application. There are %d slaves \r\n", total_slaves);
        RunDemoApplicationRanging();
    }


//     while( 1 )
//     {
//         memset(buffer, 0, sizeof(buffer));

//         printf("\n\rshell> ");

//         for(int i=0;i<3;i++){
//             message = pc.getc();
//             if( message != '\r'){
//                 printf("%c", message);
//                 buffer[i]=message;
//             }
//             else{
//                 i--;
//             }
//         }
//         buffer[3] = '\0';

// //        printf("\r\nbuffer : ");
// //        printf(buffer);
// //        printf("\r\n");
// //        printf("size of buffer : %d\n\r",sizeof(buffer));

//         if(strcmp(buffer,MST_CASE) == 0){
//             printf("\n\rSetting Master Mode\n\r");
//             Eeprom.EepromData.DemoSettings.Entity = MASTER;
//         }
//         else if(strcmp(buffer,SLV_CASE) == 0){
//             printf("\n\rSetting Slave Mode\n\r");
//             Eeprom.EepromData.DemoSettings.Entity = SLAVE;
//         }
//         else if(strcmp(buffer,LED_CASE) == 0){
//             printf("\n\rBlinking LED\n\r");
//             LedBlink();
//         }
//         else if(strcmp(buffer,AD1_CASE) == 0){
//             printf("\n\rSetting Ranging Address 1\n\r");
//             Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
//         }
//         else if(strcmp(buffer,AD2_CASE) == 0){
//             printf("\n\rSetting Ranging Address 2\n\r");
//             Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_2;
//         }
//         else if(strcmp(buffer,AD3_CASE) == 0){
//             printf("\n\rSetting Ranging Address 3\n\r");
//             Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_3;
//         }
//         else if(strcmp(buffer,AD4_CASE) == 0){
//             printf("\n\rSetting Ranging Address 4\n\r");
//             Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_4;
//         }
//         else if(strcmp(buffer,INF_CASE) == 0){
//             printf("\n\rPrinting SX1280 Informations :\n\r");
//             if(Eeprom.EepromData.DemoSettings.Entity == MASTER){
//             	printf("\n\rSX1280 Entity : MASTER\n\r");
//             }
//             else{
//             	printf("\n\rSX1280 Entity : SLAVE\n\r");
//             }
//             printf("Frequency: %s, Tx Power: %s, Antenna Setting: %s\n\r", GetRadioFrequencyGHz( ), 
//                 GetRadioTxPower( ), GetAntennaSetting( ) );
//             printf("Radio Frame Type : %s\n\rModulation Parameters : %s\n\r", GetMenuRadioFrameType( ), 
//                 GetRadioModulationParameters1( ) );
//             printf("Modulation Parameters : %s\n\r", GetRadioModulationParameters2( ) );
//             printf("Modulation Parameters : %s\n\r", GetRadioModulationParameters3( ) );
//             printf("Ranging Unit : %s\n\r",GetRangingUnit());
//             printf("Ranging Adress : %s\n\r",GetRangingAddress());
//             printf("Ranging Request Count: %s\n\r",GetRangingRequestCount());
//             printf("Ranging Channels Ok: %s\n\r",GetRngChannelsOk());
//             printf("Total Sent Packets: %s\n\r",GetMenuDemoNumSentPacket());
//         }
//         else if(strcmp(buffer,RNG_CASE) == 0){
//             printf("\n\rLaunching Ranging App\n\r");
//             Eeprom.EepromData.DemoSettings.HoldDemo = false;
//             StopDemoApplication();
//             Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
// //            while(1){
// 			RunDemoApplicationRanging();
// //            }
// //            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_2;
// //            RunDemoApplicationRanging();
// //            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_3;
// //            RunDemoApplicationRanging();
//         }
//         else if(strcmp(buffer,RN2_CASE) == 0){
//             printf("\n\rLaunching Ranging App\n\r");
//             Eeprom.EepromData.DemoSettings.HoldDemo = false;
//             StopDemoApplication();
//             Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
//             while(1){
//             	RunDemoApplicationRanging();
//             }

//         }
// //        else if(strcmp(buffer,RN2_CASE) == 0){
// //            printf("\n\rLaunching 1000 Ranging App with Time measure\n\r");
// //            Eeprom.EepromData.DemoSettings.Entity = MASTER;
// //            Eeprom.EepromData.DemoSettings.HoldDemo = false;
// //            StopDemoApplication();
// //            Timer t;
// //            t.start();
// //            for(int cpt = 0; cpt < 1000 ; cpt++){
// //                RunDemoApplicationRanging();
// //            }
// //            t.stop();
// //            printf(" %d us\n\r", t.read_us());
// //            t.reset();
// //        }
//         else{
//             printf("\n\rMessage Received, try again\n\r");
//         }

//     }
}

void FactoryReset( void )
{
    EepromFactoryReset( );
    HAL_NVIC_SystemReset( );
}

void PrintCompileSupportMessage( void )
{
    printf( "\n\rCompile time configuration:\n\r" );
    #if defined(HAS_GPS_SENSOR)
    printf( " + gps\n\r" );
    #else
    printf( " - gps\n\r" );
    #endif
    #if defined(HAS_PROXIMITY_SENSOR)
    printf( " + proximity\n\r" );
    #else
    printf( " - proximity\n\r" );
    #endif
    printf("( '+ <FEAT>' means <FEAT> is enabled, '- <FEAT>' means <FEAT> is disabled)\n\r");
}

