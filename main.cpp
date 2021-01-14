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

#ifndef TOTAL_SLAVES
    #define TOTAL_SLAVES 7
#endif
/*!
 * \brief Define IO for Unused Pin
 */
DigitalOut F_CS( D6 );      // MBED description of pin
DigitalOut SD_CS( D8 );     // MBED description of pin

DigitalOut MODE1( A1 );
DigitalOut MODE2( A2 );
DigitalOut MODE3( A3 );

Serial pc( USBTX, USBRX );


const int RNG_ADDR_LIST[] = {DEMO_RNG_ADDR_1, DEMO_RNG_ADDR_2, DEMO_RNG_ADDR_3};
int total_slaves = TOTAL_SLAVES;
int slave_index = 0;
void PrintCompileSupportMessage( void );

/*
 * \brief Specify serial datarate for UART debug outputftgl
 brdb§y(tgfvc  vTVDCX
 t÷µ¬im:pÒ≈π©©©©©©©©©©©π‘≈µ¬¬¬¬†µ÷π®ﬁ¬dl+o:÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷÷µ±
 v                     fvgvoid baud( int baudrate ){
*/


extern SX1280Hal Radio;
UART_HandleTypeDef s_UARTHandle = UART_HandleTypeDef();

uint8_t serialBuffer[USB_SERIAL_MAX_LENGTH];

void usbReceive() {
    HAL_UART_Receive(&s_UARTHandle, serialBuffer, 10, USB_SERIAL_TIMEOUT );
    printf("Received: %s\r\n", (char *) serialBuffer);
}

void USART2_IRQHandler(void)
{
printf("USART2 handler\r\n");
HAL_UART_IRQHandler(&s_UARTHandle);
}

void configureUsbSerial() {
    s_UARTHandle.Instance        = USART2;
    s_UARTHandle.Init.BaudRate   = 115200;
    s_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
    s_UARTHandle.Init.StopBits   = UART_STOPBITS_1;
    s_UARTHandle.Init.Parity     = UART_PARITY_NONE;
    s_UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    s_UARTHandle.Init.Mode       = UART_MODE_TX_RX;
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_UART_Init(&s_UARTHandle);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&s_UARTHandle, serialBuffer, 30);
    
}
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) 
{
    printf("Callback 1 !!\r\n");
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    printf("Callback 2!!\r\n");
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    printf("Callback 3 !!\r\n");
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    printf("Callback 4 !!\r\n");
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    printf("Callback 5!!\r\n");
}


int main( ){
    // Serial s( USBTX, USBRX );
    pc.baud(BAUDRATE);
    uint8_t buf[] = {48, 49, 50, 51};

    configureUsbSerial();
    HAL_UART_Transmit(&s_UARTHandle, buf, 4, 1000);
    while (true){
        wait_ms(1000);
        printf("Waiting...\r\n");
    }
    // wait_ms(3000);
    // if (pc.readable())
    // {
    //     // usbReceive();
    //     // printf("Received: %s", serialBuffer);
    //     printf("OKAY !!\r\n");
    // }

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

//    Eeprom.EepromData.DemoSettings.ModulationParam1 = LORA_SF10;
//    Eeprom.EepromData.DemoSettings.ModulationParam2 = LORA_BW_1600;
//    Eeprom.EepromData.DemoSettings.ModulationParam3 = LORA_CR_4_8;
//

    /* ranging settings */
    Eeprom.EepromData.DemoSettings.RngRequestCount = 5;
    // Eeprom.EepromData.DemoSettings.RngReqDelay = 80; // default 24

	Eeprom.EepromData.DemoSettings.HoldDemo = false;
	StopDemoApplication();


#ifdef DEVICE_SLAVE1
	Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
    printf("DEVICE_SLAVE1 defined\n\r");
#endif
#ifdef DEVICE_SLAVE2
	Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_2;
    printf("DEVICE_SLAVE2 defined\n\r");
#endif
#ifdef DEVICE_SLAVE3
	Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_3;
    printf("DEVICE_SLAVE3 defined\n\r");
#endif
#ifdef DEVICE_SLAVE4
	Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_4;
    printf("DEVICE_SLAVE4 defined\n\r");
#endif
#ifdef DEVICE_SLAVE5
	Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_5;
    printf("DEVICE_SLAVE5 defined\n\r");
#endif
#ifdef DEVICE_SLAVE6
	Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_6;
    printf("DEVICE_SLAVE6 defined\n\r");
#endif

#if defined DEVICE_SLAVE1 || defined DEVICE_SLAVE2 || defined DEVICE_SLAVE3 || defined DEVICE_SLAVE4 || defined DEVICE_SLAVE5 || defined DEVICE_SLAVE6
	Eeprom.EepromData.DemoSettings.Entity = SLAVE;
	Eeprom.EepromData.DemoSettings.HoldDemo = false;
    StopDemoApplication();
    while(1){
	    RunDemoApplicationRanging();
	}
#endif

#ifdef DEVICE_MASTER
    printf("DEVICE_MASTER defined\n\r");
    printf("\n\rLaunching Ranging App\n\r");
	Eeprom.EepromData.DemoSettings.Entity = MASTER;
    Eeprom.EepromData.DemoSettings.HoldDemo = false;
    StopDemoApplication();
	while(1){
		debug_print("DEMO_RNG_ADDR_1\n\r");
		Eeprom.EepromData.DemoSettings.RngAddress = RNG_ADDR_LIST[slave_index];
        slave_index = (slave_index + 1) % TOTAL_SLAVES;
        RunDemoApplicationRanging();

//#ifdef DEBUG_MSG
//		printf("DEMO_RNG_ADDR_2\n\r");
//#endif
//		Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_2;
//		RunDemoApplicationRanging();
//#ifdef DEBUG_MSG
//		printf("DEMO_RNG_ADDR_3\n\r");
//#endif
//		Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_3;
//		RunDemoApplicationRanging();
	}
#endif

    while( 1 )
    {
        memset(buffer, 0, sizeof(buffer));

        printf("\n\rshell> ");

        for(int i=0;i<3;i++){
            message = pc.getc();
            if( message != '\r'){
                printf("%c", message);
                buffer[i]=message;
            }
            else{
                i--;
            }
        }
        buffer[3] = '\0';

//        printf("\r\nbuffer : ");
//        printf(buffer);
//        printf("\r\n");
//        printf("size of buffer : %d\n\r",sizeof(buffer));

        if(strcmp(buffer,MST_CASE) == 0){
            printf("\n\rSetting Master Mode\n\r");
            Eeprom.EepromData.DemoSettings.Entity = MASTER;
        }
        else if(strcmp(buffer,SLV_CASE) == 0){
            printf("\n\rSetting Slave Mode\n\r");
            Eeprom.EepromData.DemoSettings.Entity = SLAVE;
        }
        else if(strcmp(buffer,LED_CASE) == 0){
            printf("\n\rBlinking LED\n\r");
            LedBlink();
        }
        else if(strcmp(buffer,AD1_CASE) == 0){
            printf("\n\rSetting Ranging Address 1\n\r");
            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
        }
        else if(strcmp(buffer,AD2_CASE) == 0){
            printf("\n\rSetting Ranging Address 2\n\r");
            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_2;
        }
        else if(strcmp(buffer,AD3_CASE) == 0){
            printf("\n\rSetting Ranging Address 3\n\r");
            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_3;
        }
        else if(strcmp(buffer,AD4_CASE) == 0){
            printf("\n\rSetting Ranging Address 4\n\r");
            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_4;
        }
        else if(strcmp(buffer,INF_CASE) == 0){
            printf("\n\rPrinting SX1280 Informations :\n\r");
            if(Eeprom.EepromData.DemoSettings.Entity == MASTER){
            	printf("\n\rSX1280 Entity : MASTER\n\r");
            }
            else{
            	printf("\n\rSX1280 Entity : SLAVE\n\r");
            }
            printf("Frequency: %s, Tx Power: %s, Antenna Setting: %s\n\r", GetRadioFrequencyGHz( ), \
                GetRadioTxPower( ), GetAntennaSetting( ) );
            printf("Radio Frame Type : %s\n\rModulation Parameters : %s\n\r", GetMenuRadioFrameType( ), \
                GetRadioModulationParameters1( ) );
            printf("Modulation Parameters : %s\n\r", GetRadioModulationParameters2( ) );
            printf("Modulation Parameters : %s\n\r", GetRadioModulationParameters3( ) );
            printf("Ranging Unit : %s\n\r",GetRangingUnit());
            printf("Ranging Adress : %s\n\r",GetRangingAddress());
            printf("Ranging Request Count: %s\n\r",GetRangingRequestCount());
            printf("Ranging Channels Ok: %s\n\r",GetRngChannelsOk());
            printf("Total Sent Packets: %s\n\r",GetMenuDemoNumSentPacket());
        }
        else if(strcmp(buffer,RNG_CASE) == 0){
            printf("\n\rLaunching Ranging App\n\r");
            Eeprom.EepromData.DemoSettings.HoldDemo = false;
            StopDemoApplication();
            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
//            while(1){
			RunDemoApplicationRanging();
//            }
//            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_2;
//            RunDemoApplicationRanging();
//            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_3;
//            RunDemoApplicationRanging();
        }
        else if(strcmp(buffer,RN2_CASE) == 0){
            printf("\n\rLaunching Ranging App\n\r");
            Eeprom.EepromData.DemoSettings.HoldDemo = false;
            StopDemoApplication();
            Eeprom.EepromData.DemoSettings.RngAddress = DEMO_RNG_ADDR_1;
            while(1){
            	RunDemoApplicationRanging();
            }

        }
//        else if(strcmp(buffer,RN2_CASE) == 0){
//            printf("\n\rLaunching 1000 Ranging App with Time measure\n\r");
//            Eeprom.EepromData.DemoSettings.Entity = MASTER;
//            Eeprom.EepromData.DemoSettings.HoldDemo = false;
//            StopDemoApplication();
//            Timer t;
//            t.start();
//            for(int cpt = 0; cpt < 1000 ; cpt++){
//                RunDemoApplicationRanging();
//            }
//            t.stop();
//            printf(" %d us\n\r", t.read_us());
//            t.reset();
//        }
        else{
            printf("\n\rMessage Received, try again\n\r");
        }

    }
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

