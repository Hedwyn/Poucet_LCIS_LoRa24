#include "mbed.h"
#include "rfs1280act.h"

/*THIS DEMONSTRATION SOURCE CODE IS PROVIDED BY DLP DESIGN "AS IS" AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW.
IN NO EVENT  SHALL DLP DESIGN BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS DEMONSTRATION SOURCE CODE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


//*******  PROTOTYPES  ***********************************************************
void writecom(unsigned char d, unsigned char mode);
void write_string_to_LCD(char *strdata, unsigned char len);
void write_int_to_LCD(unsigned int x, unsigned char num_digits);
void write_float_to_LCD(float x);
void DisplayChipWhip(unsigned char chipwhip);
void DisplayLMH(unsigned char lmh);
void SpiInit(void);
uint8_t WriteTXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size );
uint8_t ReadRXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size );
uint8_t WriteCommand2( RadioCommands_t command, uint8_t *buffer, uint16_t size );
uint8_t ReadCommand2( RadioCommands_t command, uint8_t *buffer, uint16_t size );
uint8_t WriteRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size );
uint8_t ReadRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size );
void MyRadioReset(void);
void Wakeup(void);
void ClearIrqStatus(uint16_t irq);
void SetTx();
uint8_t SelectRegulator(uint8_t mode);
uint8_t	SetTypicalRegisterSettings(uint8_t mode, uint8_t spreading_factor);
void SetAllModulationParameters(uint8_t CurrentMode, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t spreading_factor);
void SetAllPacketParameters(uint8_t CurrentMode, uint8_t p1,uint8_t p2,uint8_t p3,uint8_t p4,uint8_t p5,uint8_t p6,uint8_t p7);
void GoToStandby(uint8_t mode);
void LoadTXBuf(void);
void SelectInterrupts(void);
void initLCD(uint8_t CurrentMode, uint8_t spreading_factor, uint16_t MyID, uint16_t YourID);
void EnterCWTestMode(void);
void EnterCWModTestMode(void);
void SetRfFrequency(uint32_t frequency);
void SetRx();
void SendMyID(uint16_t MyID);
void ReceiveYourID(uint16_t MyID, uint8_t CurrentMode);
void EnterPingTestMode(uint16_t MyID, uint16_t YourID);
void DisplayTR(unsigned char txrx);
void RangeTestMasterMode(uint16_t MyID, uint16_t YourID);
void RangeTestSlaveMode(void);
void SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord, uint8_t offset );
void SetCrcSeed( uint8_t *seed );
void SetCrcPolynomial( uint16_t polynomial );
void Set_FLRC_Sync_CRC(void);
void Set_GFSK_Sync_CRC(void);
int32_t complement2( const uint32_t num, const uint8_t bitCnt );
void EEPROM_Erase(uint32_t addr);
void EepromWrite( uint32_t addr, uint32_t data);
double calc_y1(double x, int32_t rssi, int8_t chipwhip);



//*******  GLOBALS  **************************************************************
TickTime_t timeout;
uint8_t spreading_factor;
uint8_t g_ant_int_ext;
uint8_t LMHsetting;
float v;
uint16_t sample_number;
uint8_t audio_txrx_buf[300];
uint8_t OK_to_transmit;	
uint8_t myinterruptmode;
uint8_t ranging_active;
uint8_t current_ranging_channel;
uint8_t time_to_change_channels;	
uint16_t itemp2;
uint16_t r_count;
uint8_t stay_quiet_mode;



//********************************************************************************
void int_acq()  // ACQ/DAC/RANGING interupt routine 
{	
	if(myinterruptmode == RANGINGCHCHANGE) //periodically change channels
		time_to_change_channels = 1;	
	
	if(myinterruptmode == RECORD)//acquire an audio packet and set transmit flag
	{
		v = ain.read();
		v *= (float)250.0;	
		audio_txrx_buf[sample_number++] = (uint8_t)v;
		
		if(sample_number>=PTTPACKETSIZE)
		{
			//sample_number=5;//start acquiring next buffer
			OK_to_transmit=1;	
		}
	}
	
	if(myinterruptmode == PLAY)//play audio packet over speaker
	{
		v = (float)audio_txrx_buf[sample_number];
		v /= (float)300.0;
		dac.write(v);
		
		sample_number++;
		if(sample_number > PTTPACKETSIZE-1)
		{
			sample_number = PTTPACKETSIZE-1;//waiting for next audio packet	
		}
	}
}




//*******************************************************************************
//*******************************************************************************
//**********  SX1280 Module L073RZ  *********************************************
//*******************************************************************************
//*******************************************************************************
int main() 
{
	uint16_t bloop;
	uint8_t  buf[260];
	volatile uint8_t  silicon_version;
	uint16_t YourID, MyID, masterID;
	uint16_t testID;
	uint8_t  CurrentMode, PURmodmode;
	uint32_t lSerNum;
	uint8_t  sertemp;
	int8_t   tmprssi;


	//set up back channel through Nucleo USB port to host PC terminal window
	BufferedSerial s( USBTX, USBRX );
  	s.set_baud( 115200 );
	printf( "Start SX1280DevKit : %s\n\r", FIRMWARE_VERSION );

	
	//build MyID from Unique device ID register @ Base address: 0x1FF80064 (STM32L073RXT6)
	lSerNum = *((uint32_t *)0x1FF80064);	
	sertemp = (uint8_t)lSerNum & 0xFF;             //save 8 LSbits
	MyID = (uint16_t)((lSerNum>>8) & 0xFF00);   //save 8 MSbits
	MyID |= sertemp;//                             //combine



	sample_number = 0;
	OK_to_transmit = 0;
	ranging_active=0;//50mS timer disabled
	stay_quiet_mode=0;

	//read EEPROM - if never initialized then do it here
	testID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MYID);//Read 32-bit word
	if(testID != MyID)
	{
		//init EEPROM storage to -not- paired and LORA 
		EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, MyID); //MyID
		EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, 0x0000); //YourID
		EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MODMODE, LORA); //PURmodmode
	}

	//read startup values for ID of paired module and current modulating mode
	MyID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MYID);//Read 32-bit word
	YourID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_YOURID);//Read 32-bit word
	PURmodmode = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MODMODE);//Read 32-bit word
	printf( "Eeprom done\r\n" );

	
 	SpiInit();//set SPI interface format and freq
	printf( "SPI done\r\n" );

  	MyRadioReset();//reset SX1280 and enable interrupts
	printf( "radio reset done\r\n" );

  	Wakeup();//read SX1280 status register
	SelectRegulator(0);//0:LDO   1:DC-DC converter
	
	//Set TX parameters - power and TX ramp time
	buf[0] = 31;    //-18+31 yields +13dBm (+12.5 max) Xmit power
	buf[1] = 0xE0;  //20uS ramp time - best for less out of band noise
	WriteCommand2( RADIO_SET_TXPARAMS, buf, 2 );//0x8E P72

	

	ReadCommand2( RADIO_GET_SILICON_VERSION, buf, 1 ); //returns 0x47 for new Silicon, 0x67 for original
	silicon_version = buf[0];
	printf( "Silicon Version: 0x%X\n\r", silicon_version);

	//Set initial freq
	SetRfFrequency( LOW );

	//set TX and RX buffer base address
	buf[0] = 0x00;  //TX Base Address
	buf[1] = 0x00;  //RX Base Address
	WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F

	LoadTXBuf();//load data to be transmitted into the TX buffer	

	SelectInterrupts();//enable TxDone, RxDone and Timeout
	
	SendMyID(MyID);//broadcast my ID using fixed parameters to any transceiver waiting to be paired

	GoToStandby(0);//go to standby mode

	//Set packet type - this MUST be called before modulation and packet parameters are selected
	spreading_factor=DEFAULTSPREADINGFACTOR;//init
	CurrentMode = PURmodmode;//stored in EEPROM memory so the RFS2380ACT starts up in last mode used before powering down
	buf[0] = CurrentMode;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
	WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
	SetTypicalRegisterSettings(CurrentMode, spreading_factor); //these setting can be also changed by the following 2 function calls...
	//SetAllModulationParameters(CurrentMode, 0,0,0, spreading_factor);//0x8B
	//SetAllPacketParameters(CurrentMode, 0,0,0,0,0,0,0);//0x8C


	//GFSK CRC and Sync 
	//Table 13-11: Sync Word Definition in GFSK Packet P89,90
	//Table 13-12: CRC Initialisation Registers P90
	//Table 13-13: CRC Polynomial Definition P90
	if(CurrentMode == GFSK)
		Set_GFSK_Sync_CRC();
	
	//FLRC 
	//Table 13-39: CRC Initialisation Registers P105
	//Table 13-40: CRC Polynomial Definition P106
	//Table 13-42: Sync Word Definition in FLRC Packet P106
	if(CurrentMode == FLRC)
		Set_FLRC_Sync_CRC();

	LMHsetting=1;
	initLCD(CurrentMode, spreading_factor, MyID, YourID);
	
	// //quick beep
	// for(bloop=0; bloop<200; bloop++)
	// {
	// 	dac=0.0;
	// 	wait_us( 300 );
	// 	dac=2.0;
	// 	wait_us( 300 );
	// }


	if(!SW4) //enter loop with fixed parameters waiting for ID from module that wants to pair - set packet type to CurrentMode when done
		ReceiveYourID(MyID, CurrentMode);


	//FCC TEST LAB MODES - entered from power up *********
	//enter and remain in CW (no Modulation) test mode - Use SW5 to 
	//select low (2.402GHz), mid (2.44GHz) and high (2.48GHz) high carrier freqs
	if(!SW1) 
		EnterCWTestMode();  //must reset micro to exit this mode ***********

	//enter and remain in CW (with Modulation) test mode - Use SW5 to 
	//select low (2.402GHz), mid (2.44GHz) and high (2.48GHz) high carrier freqs
	if(!SW3)
		EnterCWModTestMode();  //must reset micro to exit this mode ***********


	//RANGE AND RANGING MODES - entered from power up *******
	//enter and remain in Ping test mode (2 pings per second) - Use SW5 to 
	//select low (2.40GHz), mid (2.44GHz) and high (2.48GHz) high carrier freqs
	if(!SW5) 
		EnterPingTestMode(MyID, YourID);  //must reset micro to exit this mode **********

	//enter and remain in Ranging test mode 
	if(!SW6) 
		RangeTestMasterMode(MyID, YourID);  //must reset micro to exit this mode **********
	
	//enter receive mode.  
	ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
	SetRx();//stays in this mode until packet received or RESET button pressed
	
	while(1) // enter main loop waiting for incoming packet
	{
		//read the random number generator every pass for LOCATOR LEARN function
		r_count = rand();
		r_count &= 0x0000FFFF;
		
		if(ALL3IRQ)   //if packet received (interrupt generated)
		{                
			for(bloop=0; bloop<5; bloop++ )//init buffer
				buf[bloop]=0;

			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
			while(ALL3IRQ == 1);//wait for RX interrupt to go low

			//read status register for the RSSI byte
			ReadCommand2( RADIO_GET_PACKETSTATUS, buf, 1 );
			tmprssi = buf[0];
			
			ReadRXBuffer(0, buf, 5);//read buffer to get received packet
			testID = (buf[2] | (buf[3]<<8));  //Slave ID (me) LSB first



			//If Locator Learn packet received _AND_ broadcast ID was received then 
			//save master's ID, update LCD with master's ID,
			//wait randon amount of time and transmit back myID to update counter
			if((buf[4] == LEARN_BROADCAST_PACKET) && (testID == 0xFFFF))
			{
				masterID = (buf[0] | (buf[1]<<8));
				EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, masterID); //YourID (master - LOCATOR)
				
				//update LCD with master's ID
				writecom(0xC0, 0);//mode=0 instruction/command,    mode=1 data  position cursor to start of second row
				write_string_to_LCD("YU:", 3);
				write_int_to_LCD(masterID, 5);				

				//change to SF6 for FAST 2mS LORA packets during this Learn process
				GoToStandby(0);//go to standby mode
				spreading_factor=6;
				CurrentMode = LORA;//stored in EEPROM memory so the RFS2380ACT starts up in last mode used before powering down
				buf[0] = CurrentMode;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
				WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
				SetTypicalRegisterSettings(CurrentMode, spreading_factor); //these setting can be also changed by the following 2 function calls...
				//SetAllModulationParameters(CurrentMode, 0,0,0, spreading_factor);//0x8B
				//SetAllPacketParameters(CurrentMode, 0,0,0,0,0,0,0);//0x8C

				//make sure TX buffer has correct header for audio packets
				buf[0] = (uint8_t)(MyID&0xff);      //MyID LSB
				buf[1] = (uint8_t)((MyID>>8)&0xff);  //MyID MSB
				buf[2] = (uint8_t)YourID & 0x00FF;       //YourID ID - LSB first
				buf[3] = (uint8_t)((YourID>>8) & 0xff);  //YourID - MSB
				buf[4] = LEARN_BROADCAST_PACKET;
				WriteTXBuffer(0, buf, 5);//load buffer

				//do not send the reply if we had previously been instructed to 
				//be quiet by the LOCATOR master
				if(stay_quiet_mode == 0)
				{
					//wait random amount of time (0-65mS) before transmitting.
					//5 byte packet takes 2mS to Xmit in SF6
					r_count = rand();
					r_count &= 0x0000FFFF;
					wait_us(r_count);
					
					ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
					SetTx();//transmit
					while(ALL3IRQ == 0);//wait for TX interrupt to go high				
				}


				//go back to SF12
				GoToStandby(0);//go to standby mode
				spreading_factor=DEFAULTSPREADINGFACTOR;//init
				CurrentMode = LORA;//stored in EEPROM memory so the RFS2380ACT starts up in last mode used before powering down
				buf[0] = CurrentMode;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
				WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
				SetTypicalRegisterSettings(CurrentMode, spreading_factor); //these setting can be also changed by the following 2 function calls...
				//SetAllModulationParameters(CurrentMode, 0,0,0, spreading_factor);//0x8B
				//SetAllPacketParameters(CurrentMode, 0,0,0,0,0,0,0);//0x8C
			}


			//If Stay Quiet packet received _AND_ my ID was received then 
			//set quiet mode active (disabled upon reset)
			//put Q on display for quiet mode
			if((buf[4] == STAY_QUIET_PACKET) && (testID == MyID))
			{
				stay_quiet_mode=1;
				writecom(0xCE, 0);//position cursor
				write_string_to_LCD("Q", 1);
			}


			
			
			//If Listen packet received _AND_ Broadcast ID was received then 
			//disable quiet mode (respond to Learn mode packets)
			//do not send a reply
			if((buf[4] == LISTEN_PACKET) && (testID == 0xFFFF))
			{
				stay_quiet_mode=0;
				writecom(0xCE, 0);//position cursor
				write_string_to_LCD(" ", 1);
			}
			
			

			//If Locator Ping packet (sends a reply) received _AND_ my ID was received then 
			//send back the rssi for this incoming packet
			if((buf[4] == LOCATOR_PING_PACKET) && (testID == MyID))
			{
				//make sure TX buffer has correct header for audio packets
				buf[0] = (uint8_t)(MyID&0xff);      //MyID LSB
				buf[1] = (uint8_t)((MyID>>8)&0xff);  //MyID MSB
				buf[2] = (uint8_t)YourID & 0x00FF;       //YourID ID - LSB first
				buf[3] = (uint8_t)((YourID>>8) & 0xff);  //YourID - MSB
				buf[4] = LOCATOR_PING_PACKET;
				buf[5] = tmprssi;

				WriteTXBuffer(0, buf, 6);//load buffer
				ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
				SetTx();//transmit
				while(ALL3IRQ == 0);//wait for TX interrupt to go high				
			}
			
			
			
			//If Ping test packet received _AND_ my ID was received then beep speaker
			if((buf[4] == SIMPLE_PING_PACKET) && (testID == MyID))
			{
				//quick beep
				for(bloop=0; bloop<200; bloop++)
				{
					dac=0.0;
					wait_us( 300 );
					dac=2.0;
					wait_us( 300 );
				}
			}			

			//If audio packets received play audio over speaker
			if((buf[4] == PTT_AUDIO_PACKET) && (testID == MyID))
			{
					myinterruptmode = PLAY;
				
					ReadRXBuffer(0, audio_txrx_buf, PTTPACKETSIZE);//read buffer to get received packet
					sample_number=5;

					if(spreading_factor != PTTSPREADINGFACTOR)
					{
							//return to LORA mode
							GoToStandby(0);//go to standby mode and check current status
							spreading_factor=PTTSPREADINGFACTOR;
							buf[0] = LORA;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
							WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
							//SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
							//go to these functions so we can change the packet size to 250
							SetAllModulationParameters(CurrentMode, PTTSPREADINGFACTOR,DEFAULTMODLORAP2,DEFAULTMODLORAP3, spreading_factor);//0x8B
							SetAllPacketParameters(CurrentMode, DEFAULTPACKETLORAP1,DEFAULTPACKETLORAP2,PTTPACKETSIZE,DEFAULTPACKETLORAP4,DEFAULTPACKETLORAP5,DEFAULTPACKETLORAP6,DEFAULTPACKETLORAP7);//0x8C

							SetRfFrequency( LOW );

							//set TX and RX buffer base address
							buf[0] = 0x00;  //TX Base Address
							buf[1] = 0x00;  //RX Base Address
							WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F
			
							writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor
							write_string_to_LCD("LORA", 4);				
							writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
							write_int_to_LCD(spreading_factor, 2);

							SelectInterrupts();//enable TxDone and RxDone 
					}
					else
					{
						//send to DAC at 8K s/s using interrupt timer
						MyTicker.attach(&int_acq, 0.000125); // play rate is 8K s/s
					}
			}				
			
			//If Ranging packet received AND my ID was received then enter range test mode
			if((buf[4] == RANGING_PACKET) && (testID == MyID))
			{
				RangeTestSlaveMode();//need to add timeout to keep from getting locked into this mode

				//return to LORA mode
				GoToStandby(0);//go to standby mode and check current status
				spreading_factor=DEFAULTSPREADINGFACTOR;
				buf[0] = LORA;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
				WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
				SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
				//SetAllModulationParameters(CurrentMode, 0,0,0, spreading_factor);//0x8B
				//SetAllPacketParameters(CurrentMode, 0,0,0,0,0,0,0);//0x8C

				SetRfFrequency( LOW );

				LMHsetting=1;
				initLCD(CurrentMode, spreading_factor, MyID, YourID);
			}	
			
			//remain in RX mode
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
			SetRx();//stays in RX mode
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt

		}//end - if packet received (interrupt generated)

		
		
		if(!SW1) //toggle packet mode GFSK, LORA, RANGING, FLRC 0 this mode change is saved to EEPROM memory for next RESET/power up
		{
			while(!SW1);
			
			CurrentMode++;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
			if(CurrentMode>3) CurrentMode=0;

			if(CurrentMode==2) //skip ranging
				CurrentMode=3;
			
			GoToStandby(0);//go to standby mode and check current status
			
			buf[0] = CurrentMode;
			WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
			SetTypicalRegisterSettings(CurrentMode, spreading_factor); 

			//update EEPROM every time the mode is changed so system will power up in this mode
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, MyID); //MyID
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, YourID); //YourID
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MODMODE, CurrentMode); //PURmodmode

			if(CurrentMode == GFSK)
			{
				Set_GFSK_Sync_CRC();

				writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
				write_string_to_LCD("GFSK", 4);

				writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
				write_string_to_LCD("  ", 2);
				writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
				write_string_to_LCD("  ", 2);
				writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
			}	
			if(CurrentMode == FLRC)
			{
				Set_FLRC_Sync_CRC();
				
				writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
				write_string_to_LCD("FLRC", 4);

				writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
				write_string_to_LCD("  ", 2);//blank out SF
				writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
				write_string_to_LCD("  ", 2);//blank out spreading factor
				writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
			}	

			if(CurrentMode == LORA)
			{
				writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
				write_string_to_LCD("LORA", 4);

				writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
				write_string_to_LCD("SF", 2);
				writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
				write_int_to_LCD(spreading_factor, 2);
				writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
			}
			
			//enter and stay in receive mode.  
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
			SetRx();//stays in this mode until packet received or RESET button pressed
		}//end of change packet mode
			
		if(!SW2)//toggle antenna switch internal, external
		{
			while(!SW2);
			g_ant_int_ext^=1;
			ANT_SEL = g_ant_int_ext;
			DisplayChipWhip(g_ant_int_ext);//0chip  1whip				
		}
			
		if(!SW3) //increase spreading factor, wrapping from 12 back to 5
		{
			while(!SW3);

			GoToStandby(0);//go to standby mode and check current status
			
			spreading_factor++;   //5-12
			if(spreading_factor>12) 
				spreading_factor=5;
			CurrentMode = LORA;
			buf[0] = LORA;
			WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A

			//update EEPROM every time the mode is changed so system will power up in this mode
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, MyID); //MyID
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, YourID); //YourID
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MODMODE, CurrentMode); //PURmodmode

			//writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
			writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
			write_string_to_LCD("LORA", 4);
			writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
			write_string_to_LCD("SF", 2);
			writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
			write_int_to_LCD(spreading_factor, 2);
			writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home

			SetAllModulationParameters(CurrentMode, spreading_factor, DEFAULTMODLORAP2, DEFAULTMODLORAP3, spreading_factor);
			
			//enter and stay in receive mode.  
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
			SetRx();//stays in this mode until packet received or RESET button pressed
		}

		

		//if sample rate is 8000s/s then 
		//  1 byte of data represents 125uS of audio 	
		//  100 bytes of data represents 12.5mS of audio 	
		//  250 bytes of data represents 31.25mS of audio 	
		//***************************************************************************************************************
		//***************************************************************************************************************
		//***  FLRC packet take 740uS to transmit a 6 byte packet 
		//***  FLRC packet take 3.2mS to transmit a 105 byte packet 
		//***	
		//***  LORA packet SF5 take 1.2mS to transmit a 6 byte packet 
		//***
		//***  LORA packet SF5 take 5.0mS to transmit a 105 byte packet 
		//***  LORA packet SF6 take 8.4mS to transmit a 105 byte packet 
		//***  LORA packet SF7 take 14.4mS to transmit a 105 byte packet 
		//***  LORA packet SF8 take 25.2mS to transmit a 105 byte packet 
		//***
		//***  LORA packet SF5 take 10.9mS to transmit a 255 byte packet 
		//***  LORA packet SF6 take 18.2mS to transmit a 255 byte packet 
		//***  LORA packet SF7 take 31.2mS to transmit a 255 byte packet  *** GOAL ***
		//***  LORA packet SF8 take 55.2mS to transmit a 255 byte packet 
		//***  LORA packet SF9 take 97mS   to transmit a 255 byte packet 
		//***  LORA packet SF10 take 179mS to transmit a 255 byte packet 
		//***  LORA packet SF11 take 390mS to transmit a 255 byte packet 
		//***  LORA packet SF12 take 710mS to transmit a 255 byte packet 
		//***************************************************************************************************************
		//***************************************************************************************************************
		if(!SW4) //PTT: digitize audio (8Ksps) and transmit 255 byte packets (31.25mS of audio each)
		{
			printf("Called PTT\r\n");
			MyTicker.detach();//disable receive/play timer (if active)

			DisplayTR(0);//0-T 1-R  (Display T or R on LCD)

			//make sure TX buffer has correct header for audio packets
			audio_txrx_buf[0] = (uint8_t)(MyID&0xff);      //MyID LSB
			audio_txrx_buf[1] = (uint8_t)((MyID>>8)&0xff);  //MyID MSB
			audio_txrx_buf[2] = (uint8_t)YourID & 0x00FF;       //YourID ID - LSB first
			audio_txrx_buf[3] = (uint8_t)((YourID>>8) & 0xff);  //YourID - MSB
			audio_txrx_buf[4] = PTT_AUDIO_PACKET;//walktie talkie command 

			//send a audio packet with DEFAULTSPREADINGFACTOR to force the receiver to switch to PTTSPREADINGFACTOR
			GoToStandby(0);//go to standby mode and check current status
			spreading_factor=DEFAULTSPREADINGFACTOR;
			buf[0] = LORA;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
			WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
			//SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
			//go to these functions so we can change the packet size to 250
			SetAllModulationParameters(CurrentMode, PTTSPREADINGFACTOR,DEFAULTMODLORAP2,DEFAULTMODLORAP3, spreading_factor);//0x8B
			//only send first 5 bytes so its fast!
			SetAllPacketParameters(CurrentMode, DEFAULTPACKETLORAP1,DEFAULTPACKETLORAP2,5,DEFAULTPACKETLORAP4,DEFAULTPACKETLORAP5,DEFAULTPACKETLORAP6,DEFAULTPACKETLORAP7);//0x8C
			SetRfFrequency( LOW );
			//set TX and RX buffer base address
			buf[0] = 0x00;  //TX Base Address
			buf[1] = 0x00;  //RX Base Address
			WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F
			SelectInterrupts();//enable TxDone and RxDone 
			WriteTXBuffer(0, audio_txrx_buf, 5);//only send first 5 bytes so its fast!
			SetTx();//transmit
			while(ALL3IRQ == 0);//wait for TX interrupt to go high

			//switch to PTTSPREADINGFACTOR for Walktie Talkie mode
			GoToStandby(0);//go to standby mode and check current status
			//LORA mode
			spreading_factor=PTTSPREADINGFACTOR;
			buf[0] = LORA;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
			WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
			//SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
			//go to these functions so we can change the packet size to 250
			SetAllModulationParameters(CurrentMode, PTTSPREADINGFACTOR,DEFAULTMODLORAP2,DEFAULTMODLORAP3, spreading_factor);//0x8B
			SetAllPacketParameters(CurrentMode, DEFAULTPACKETLORAP1,DEFAULTPACKETLORAP2,PTTPACKETSIZE,DEFAULTPACKETLORAP4,DEFAULTPACKETLORAP5,DEFAULTPACKETLORAP6,DEFAULTPACKETLORAP7);//0x8C

			SetRfFrequency( LOW );

			//set TX and RX buffer base address
			buf[0] = 0x00;  //TX Base Address
			buf[1] = 0x00;  //RX Base Address
			WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F
			
			SelectInterrupts();//enable TxDone and RxDone 
			
			writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor
			write_string_to_LCD("LORA", 4);				
			writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
			write_int_to_LCD(spreading_factor, 2);

			sample_number=5;			
			myinterruptmode = RECORD;
			MyTicker.attach(&int_acq, 0.000125); // acquire at a rate of 8K s/s
			printf("Ready to send packets\r\n");
			while(!SW4)//while SW4 is held stay here transmitting complete audio packets
			{
				if(OK_to_transmit==1)//if full packet acquired then transmit
				{
					sample_number=5;//make sure to catch audio data at the next interrupt
					printf("TX\r\n");

					WriteTXBuffer(0, audio_txrx_buf, PTTPACKETSIZE);//load audio samples to TX buffer
					printf("TX done\r\n");

					OK_to_transmit=0;

					SetTx();//transmit
				}					
			}

			while(ALL3IRQ == 0);//wait for TX to complete
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt

			MyTicker.detach();
			DisplayTR(1);//0-T 1-R  (for TX or RX modes)
			
			//enter and stay in receive mode.  
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
			SetRx();//stays in this mode until packet received or RESET button pressed
    }



		
		if(!SW5) //toggle high/mid/low transmit frequency
		{
			while(!SW5);//wait for button release		
			
			LMHsetting++;
			if(LMHsetting>3)
				LMHsetting=1;
			
			if(LMHsetting==1)	SetRfFrequency( LOW );
			if(LMHsetting==2)	SetRfFrequency( MID );
			if(LMHsetting==3)	SetRfFrequency( HIGH );
			DisplayLMH(LMHsetting);//1-Low 2-Mid 3-High - shows current transmit channel
		}			
		
	}//end of while(1) loop
}//end of main


/*
	TEST_LED=1;
	wait_us(a);
	TEST_LED=0;
	wait_us(1000 *100);
*/



//Reg: 0x9CE, 5 bytes: DD, A0, 96, 69, DD, End.<LF><CR>
//Reg: 0x9C8, 2 bytes: 0, 45, End.<LF><CR>                       GFSK CRC init register   
//Reg: 0x9C6, 2 bytes: 1, 23, End.<LF><CR>                       GFSK CRC Polynomial def  
//**************************************************************************************
void Set_GFSK_Sync_CRC()
{
	static uint8_t syncWord[] = { 0xDD, 0xA0, 0x96, 0x69, 0xDD };
	uint8_t i, Buffer[130];
	
	SetSyncWord( 1, syncWord, 0 );

  uint8_t crcSeedLocal[3] = { 0x00, 0x45, 0x67 };
  SetCrcSeed( crcSeedLocal );
	
	SetCrcPolynomial( 0x0123 );
	
	//write 120 bytes of randon data to TX buffer
  for( i = 0; i < 120; i++ )
    Buffer[i] = ( uint8_t )rand( );
	WriteTXBuffer(0, Buffer, 120);//Load TX Buffer 120 byte payload
}



//Reg: 0x9CF, 4 bytes: DD, A0, 96, 69, End.<LF><CR>              GFSK sync word - 32 bits (no MSByte)  
//Reg: 0x9C8, 2 bytes: 0, 45, End.<LF><CR>                       GFSK CRC init register                   
//Reg: 0x9C6, 2 bytes: 1, 23, End.<LF><CR>                       GFSK CRC Polynomial def  
//**************************************************************************************
void Set_FLRC_Sync_CRC()
{
	uint8_t i, Buffer[130];
	static uint8_t syncWord[] = { 0xDD, 0xA0, 0x96, 0x69, 0xDD };
	
	SetSyncWord( 1, syncWord, 1 );

  uint8_t crcSeedLocal[3] = { 0x00, 0x45, 0x67 };
  SetCrcSeed( crcSeedLocal );
	
	SetCrcPolynomial( 0x0123 );
	
	//write 120 bytes of randon data to TX buffer
  for( i = 0; i < 120; i++ )
            Buffer[i] = ( uint8_t )rand( );
	WriteTXBuffer(0, Buffer, 120);//Load TX Buffer 120 byte payload
}



//**************************************************************************************
void SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord, uint8_t offset )
{         
  uint16_t addr;
	uint8_t syncwordSize;

	syncwordSize = 4;
	addr = REG_LR_SYNCWORDBASEADDRESS1 + offset; //+1=0x09CF  +0=0x9CE
 	WriteRegister_16bAddress(addr, syncWord, syncwordSize);
}


//**************************************************************************************
void SetCrcSeed( uint8_t *seed )
{
 	WriteRegister_16bAddress(REG_LR_CRCSEEDBASEADDR, seed, 2);
}


//**************************************************************************************
void SetCrcPolynomial( uint16_t polynomial )
{
  uint8_t val[2];

  val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
  val[1] = ( uint8_t )( polynomial  & 0xFF );
  
 	WriteRegister_16bAddress(REG_LR_CRCPOLYBASEADDR, val, 2);
}


//arrive here because I am Ranging master and need to send a LORA packet requesting 
//RANGING with the paired slave transceiver
//stay in this function sending Ranging packets and updating the LCD display until RESET
//**************************************************************************************
void RangeTestMasterMode(uint16_t MyID, uint16_t YourID)//Master mode for Ranging
{
	uint8_t	buf[260];
	volatile int32_t rssi1, rssi2;
	volatile uint16_t masterID, slaveID;
	volatile uint16_t big_index1, i, num_packets;
	volatile uint32_t tmp1, tmp2, tmp3;
	volatile double feet; 
	volatile uint32_t temp_result;
	volatile uint32_t rtimeout;
	volatile uint32_t tt;
	volatile int32_t tmp32median1;
  volatile double RngFei;
	volatile int32_t rawcounts1[410];
	volatile double dblrawcounts1[410];	
	volatile uint8_t RangingSetupSuccess;
	volatile uint8_t byte0, byte1;
	volatile uint16_t j, big_loop;
	volatile int32_t x1;
	volatile uint32_t temppp;
	volatile uint8_t tmp, intextsel;
	volatile uint16_t meters;


	while(!SW6);//wait for button release


	//default to whip antenna
	intextsel=1;//0-chip  1-whip
	ANT_SEL=1;//whip
	DisplayChipWhip(1);//0-chip  1-whip				


	//set TX and RX buffer base address
	buf[0] = 0x00;  //TX Base Address
	buf[1] = 0x00;  //RX Base Address
	WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F

	while(1)//until RESET
	{
			if(!SW2)
			{
				intextsel ^=1;//toggle selection of antenna (internal Vs external)
				ANT_SEL = intextsel;//0-chip  1-whip
			}
			if(intextsel==0)//chip
			{
				writecom(0x8C, 0);//position cursor top row 4 back from end
				write_string_to_LCD("CHIP", 4);
			}
			else
				{				
					writecom(0x8C, 0);//position cursor top row 4 back from end
					write_string_to_LCD("WHIP", 4);
				}
			while(!SW2);//wait here for button release
			
			RangingSetupSuccess=0;
			while(RangingSetupSuccess==0)//ping paired transceiver requesting Ranging function
			{                            //repeat until successful
				GoToStandby(0);//0-STDBY_RC  1-STDBY_XOSC
				
				//setup for LORA mode - **paired transceiver must also be in LORA mode**
				//Set packet type - LORA
				buf[0] = LORA;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
				WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
				spreading_factor = DEFAULTSPREADINGFACTOR;//the use of SF11 or SF12 are not permitted for Ranging P116- SF9 is best per Semtech App note
				SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
				//SetAllModulationParameters(LORA, DEFAULTMODLORAP1, DEFAULTMODLORAP12, DEFAULTMODLORAP13, spreading_factor);//0x8B
				//SetAllPacketParameters(LORA, DEFAULTPACKETLORAP1, DEFAULTPACKETLORAP2, DEFAULTPACKETLORAP3, DEFAULTPACKETLORAP4, DEFAULTPACKETLORAP5, DEFAULTPACKETLORAP6, DEFAULTPACKETLORAP7);//0x8C

				//load transmit data for RANGING - need to load IDs so this packet will be received.
				buf[0] = (uint8_t)MyID & 0x00FF;
				buf[1] = (uint8_t)((MyID>>8) & 0xff);
				buf[2] = (uint8_t)YourID & 0x00FF;
				buf[3] = (uint8_t)((YourID>>8) & 0xff);
				buf[4] = 0x23;//RANGING request ********************
				WriteTXBuffer(0, buf, 5);//TX Buffer offset=0, 5 byte payload
				
				//tell paired transceiver to start responding to to Ranging packets
				SetTx(); //transmit 
				while(ALL3IRQ == 0);//wait for TX interrupt to go high
				ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
				
				//read reply to get the RSSI value from the slave in packet position [5] and freq error
				SetRx();//stays in this mode until packet received or RESET button pressed
				rtimeout = MYRXTIMEOUT_LONG;
				while((ALL3IRQ == 0) && (rtimeout>0))//wait for TX interrupt to go high with timeout
					rtimeout--;
				ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
				
				if(rtimeout>0)//didn't timeout...
				{
					RangingSetupSuccess=1;
					
					//read status register for the RSSI byte
					ReadCommand2( RADIO_GET_PACKETSTATUS, buf, 1 );//only need first of 5 bytes - that where the RSSI value is located
					tmp = buf[0];
					rssi2 = -( tmp / 2 );	//rssi of packet I just received from the slave
				}
				else
				{ 
					writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
					writecom(0xC0, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
					write_string_to_LCD("...Retry...     ", 16);				
				}
					
			}

		ReadRXBuffer(0, buf, 6);//get rssi *sent* by slave 
		tmp = buf[5];	
		rssi1 = -( tmp / 2 );//rssi of packet received by the slave
			
			
			
    RngFei = (double)(((int32_t)buf[0] << 24) | ((int32_t)buf[1]<<16) | ((int32_t)buf[2]<<8) | buf[3]);//not used...
		//printf( "RSSI: %f   RngFei: %lf    \n\r", rssi1, RngFei);
		
		GoToStandby(0);//0-STDBY_RC  1-STDBY_XOSC
		
		//prep for Ranging
		//Set packet type - Ranging
		buf[0] = RANGING;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
		WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
		if(rssi1>-80)//SF6 for short distances (<100ft)			
			spreading_factor = RANGINGSPREADINGFACTOR6;
		else
			spreading_factor = RANGINGSPREADINGFACTOR10;//the use of SF11 or SF12 are not permitted for Ranging P116
		SetAllModulationParameters(LORA, DEFAULTMODLORAP1, DEFAULTMODLORAP2, DEFAULTMODLORAP3, spreading_factor);//0x8B
		SetAllPacketParameters(LORA, DEFAULTPACKETLORAP1, DEFAULTPACKETLORAP2, DEFAULTPACKETLORAP3, DEFAULTPACKETLORAP4, DEFAULTPACKETLORAP5, DEFAULTPACKETLORAP6, DEFAULTPACKETLORAP7);//0x8C

		
		//read master and slave IDs from Flash
		MyID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MYID);//Read 32-bit word
		YourID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_YOURID);//Read 32-bit word
		//this is the code for master, therefore I am master.  My address is MyID.
		//the other transceiver is slave (YourID) 
		//the shared 32-bit shared RANGING ID is always masterID<<8 + slaveID
		masterID = MyID;
		slaveID = YourID;

		//Load Shared Ranging Address 
		buf[0] = (uint8_t)((masterID>>8) & 0x00FF); //31:24  masterID MSByte
		WriteRegister_16bAddress(0x912, buf, 1);
		buf[0] = (uint8_t)(masterID & 0x00FF); //23:16  masterID LSByte
		WriteRegister_16bAddress(0x913, buf, 1);
		buf[0] = (uint8_t)((slaveID>>8) & 0x00FF); //15:8  slaveID MSByte
		WriteRegister_16bAddress(0x914, buf, 1);
		buf[0] = (uint8_t)(slaveID & 0x00FF); //7:0 slaveID LSByte
		WriteRegister_16bAddress(0x915, buf, 1);

		//Ranging Address Bit Definition - select 32-bit shared ranging address
		ReadRegister_16bAddress(0x0931, buf, 1);
		buf[0] = (buf[0] | 0xC0); //select 32 bit master/slave address - comprised of MyID and YourID
		WriteRegister_16bAddress(0x0931, buf, 1);

		//Setup Interrupts P80 P81  MASTER    0x4603
		//enable Ranging interrupts for Master
		buf[0] = 0x46;  //0100 0110 irqMask  15:8  enable RXTXTimeout, RangingResultValid9 and RangingTimeout10
		buf[1] = 0x03;  //0000 0011 irqMask   7:0  P80
		//**NOTE: DIO1, DIO2 and DIO3 are all OR'd together on the DLP-RFS1280ACT board
		buf[2] = 0x46;  //0100 0110 dio1Mask 15:8  RangingMasterResultValid9 and RangingMasterResultTimeout10 tied to DIO1
		buf[3] = 0x03;  //0000 0011 dio1Mask  7:0  DIO1  
		buf[4] = 0x00;  //0000 0000 dio2Mask 15:8  DIO2
		buf[5] = 0x00;  //0000 0000 dio2Mask  7:0  DIO2
		buf[6] = 0x00;  //0000 0000 dio3Mask 15:8  DIO3
		buf[7] = 0x00;  //0000 0000 dio3Mask  7:0  DIO3
		WriteCommand2( RADIO_SET_DIOIRQPARAMS, buf, 8 );//0x8D

		//Set TX parameters: power and TX ramp time
		buf[0] = 31;    //-18+31 yields +13dBm (+12.5 max) Xmit power
		buf[1] = 0xE0;  //20uS ramp time - best for less out of band noise
		WriteCommand2( RADIO_SET_TXPARAMS, buf, 2 );//0x8E P72

		//Write the RxTx delay offset calibration value (from the Semtech App note) - not used...
		buf[0] = (uint8_t)(RXTXDELAYCALOFFSET & 0xFF); //13430 for BW:1600 and SF=9
		WriteRegister_16bAddress(0x092D, buf, 1);//address used in Semtech demo code
		buf[0] = (uint8_t)((RXTXDELAYCALOFFSET>>8) & 0xFF);
		WriteRegister_16bAddress(0x092C, buf, 1);//address used in Semtech demo code


		big_index1=0;
		myinterruptmode = RANGINGCHCHANGE;
		current_ranging_channel=0;
		SetRfFrequency(Channels[current_ranging_channel]);//set TX freqq to first in hop list
	
		wait_us( STARTDELAY );//wait for slave to be ready - 
			
		//determine SF based on slave rssi	
		if(rssi1 > RSSITHRESHOLD)//SF6 for short distances ( < ~100ft )
		{
			MyTicker.attach(&int_acq, TICKERSF6); // mS tick for each reading: 10 batches of 40 channels = 400 readings 
			num_packets = 200;
		}
		else //SF10 for long distances ( > ~100ft )
		{
			MyTicker.attach(&int_acq, TICKERSF10); // mS tick for each reading: 10 batches of 40 channels = 400 readings 
			num_packets = 80;
		}

		
		for(big_loop=0; big_loop<num_packets; big_loop++) 
		{
						GoToStandby(0);//0-STDBY_RC  1-STDBY_XOSC

						//set my Ranging role - MASTER
						buf[0] = RANGINGMASTER;//1-master  0-slave
						WriteCommand2( RADIO_SET_RANGING_ROLE, buf, 1 );//0xA3

						//Set packet type - Ranging
						buf[0] = RANGING;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
						WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
						if(rssi1>-80)//SF6 for short distances (<100ft)			
							spreading_factor = RANGINGSPREADINGFACTOR6;
						else
							spreading_factor = RANGINGSPREADINGFACTOR10;//the use of SF11 or SF12 are not permitted for Ranging P116
						SetAllModulationParameters(LORA, DEFAULTMODLORAP1, DEFAULTMODLORAP2, DEFAULTMODLORAP3, spreading_factor);//0x8B
						SetAllPacketParameters(LORA, DEFAULTPACKETLORAP1, DEFAULTPACKETLORAP2, DEFAULTPACKETLORAP3, DEFAULTPACKETLORAP4, DEFAULTPACKETLORAP5, DEFAULTPACKETLORAP6, DEFAULTPACKETLORAP7);//0x8C
						
						//TRANSMIT RANGING PACKET - wait for interrupt and time to change to next RF channel
						time_to_change_channels=0;//clear flag
						ClearIrqStatus( IRQ_RADIO_ALL );//clear all interrupts
						SetTx(); //transmit
						while((ALL3IRQ == 0) && (time_to_change_channels==0));//wait for TX interrupt or channel change

						//read IRQ Status register to confirm RANGING successsful			
						ReadCommand2( RADIO_GET_IRQSTATUS, buf, 2 ); //0x15
						byte0 = buf[0];//Ranging Result Valid for Master - used below reading register 0x0961 to see if ranging result was valid
						byte1 = buf[1];

						ClearIrqStatus( IRQ_RADIO_ALL );//clear interrupts
						while(time_to_change_channels==0);//wait for channel change timer interrupt 
						ClearIrqStatus( IRQ_RADIO_ALL );//clear interrupts

						time_to_change_channels=0;//clear flag
						current_ranging_channel++;
						if(current_ranging_channel>39)
						{
							current_ranging_channel=0;
							SetRfFrequency(Channels[current_ranging_channel]);			
						}
						else			
							SetRfFrequency(Channels[current_ranging_channel]);			

						//Set the radio in Xoscilator mode 
						GoToStandby(1);//0-STDBY_RC  1-STDBY_XOSC

						//Enable clock in LoRa memory
						ReadRegister_16bAddress(0x097F, buf, 1);
						buf[0] = ( buf[0] | (1<<1) );
						WriteRegister_16bAddress(0x097F, buf, 1);
						
						//Set Ranging Result Type
						ReadRegister_16bAddress(0x0924, buf, 1);
						buf[0] = (buf[0] & 0xCF);//default to Raw result
						WriteRegister_16bAddress(0x0924, buf, 1);						

						ReadRegister_16bAddress(0x0961, buf, 3);//read 3 range bytes
						if(byte0 == 0x02)//2=Ranging Result Valid  4=timeout
						{
							temp_result = ( buf[0]<<16 | buf[1]<<8 | buf[2] );
							rawcounts1[big_index1++] = (int32_t)complement2(temp_result, 24);
						}			
		}

	MyTicker.detach(); //disable this timer

		
	tmp32median1 = 999999999;
	if(big_index1 > 5)//do not sort data if too few packets received
	{
		for (i=big_index1-1; i>0; --i) 
		{
				for (j=0; j<i; ++j) 
				{
						if (rawcounts1[j] > rawcounts1[j+1]) 
						{
								temppp = rawcounts1[j];
								rawcounts1[j] = rawcounts1[j+1];
								rawcounts1[j+1] = temppp;
						}
				}
		}
	
		//debug - so can see negative numbers during debug
		for (uint16_t i=0; i<big_index1; i++) 
			dblrawcounts1[i] = (double)rawcounts1[i];

		tmp32median1 = rawcounts1[big_index1/2];
	}
		
	x1 = (double)tmp32median1;   //median of raw data (2's comp of counts)
	tmp32median1 = calc_y1(x1, rssi1, intextsel);	 //Feet - calculated by polynomial(s)
		
	
	//display distance/counts, number of good samples and RSSI
	writecom(0x01, 0);//clear display
	writecom(0x02, 0);//position cursor row 1 home
	//write_int_to_LCD(x1, 5);          //counts   - for system calibration, raw data collection
	write_int_to_LCD(tmp32median1, 5);  //feet	
	write_string_to_LCD("ft", 2);
	writecom(0xC0, 0);//position cursor to start of second row
	meters = (uint16_t)((double)tmp32median1 * 0.3048); 
	write_int_to_LCD(meters, 5);
	write_string_to_LCD("m", 1);
	
	writecom(0x88, 0);//position cursor top row center
	write_int_to_LCD(big_index1, 3);//packets received out of 40 (channels)

	if(intextsel==0)//chip
	{
		writecom(0x8C, 0);//position cursor top row 4 back from end
		write_string_to_LCD("CHIP", 4);
	}
	else
		{				
			writecom(0x8C, 0);//position cursor top row 4 back from end
			write_string_to_LCD("WHIP", 4);
		}
	
	writecom(0xC7, 0);//position cursor 2nd row
	write_string_to_LCD("SF", 2);
	write_int_to_LCD(spreading_factor, 2);

	writecom(0xCC, 0);//position cursor 2nd row
	write_int_to_LCD(rssi1, 4);//rssi

	printf( "%d\t %d\t\r\n", x1, rssi1); //log range data to host PC via USB during raw data collection 

	SetRfFrequency( LOW );

	wait_us(1000 *1000);//wait 1 second for slave to transition back to LORA  mode	
		
	}//end of while(1)	

}//RangeTestMasterMode()




//***************************************************************************************************
double calc_y1(double x, int32_t rssi, int8_t chipwhip)//0-chip  1-whip
{
	volatile double y;
	
	if(chipwhip==1)
	{
		//WHIP ANTENNA
		if(rssi>-57)//SF6 - near targets
		{
			if(x<=-128)//less than 10ft
				y = -125.5105 - 3.018208*x - 0.02405039*x*x - 0.00008100349*x*x*x - 9.929779e-8*x*x*x*x;
			else // >10ft
				y = 9.359128 + 0.07299215*x + 0.0002869848*x*x - 0.000001927275*x*x*x + 4.244874e-9*x*x*x*x - 3.064832e-12*x*x*x*x*x;
		}
			else //SF10 - far targets (>50ft)
				y = 5.753966 + 0.07179256*x + 3.41401e-8*x*x - 1.146837e-13*x*x*x;
	}
	
	if(chipwhip==0)
	{
		//CHIP ANTENNA
		if(rssi>-80)//SF6 - near targets
		{
			if(x<=-142)//less than 10ft
				y = 25.62718 + 0.1427834*x + 0.0002415783*x*x + 7.792585e-8*x*x*x;
			else // >10ft
				y = 19.87852 + 0.06585959*x - 0.00001669755*x*x + 1.879339e-8*x*x*x;
		}
			else //SF10 - far targets (>50ft)
				y = 35.86462 + 0.07399442*x - 1.032333e-9*x*x;
	}
	
	return y;
}


//********************************************************************************************************
int32_t complement2( const uint32_t num, const uint8_t bitCnt )
{
	int32_t retVal = ( int32_t )num;
	if( num >= 2<<( bitCnt - 2 ) )
			retVal -= 2<<( bitCnt - 1 );

	return retVal;
}



//arrive here because the master I am paired with sent a LORA packet requesting RANGING
//*******************************************************************************
void RangeTestSlaveMode()//Slave mode for Ranging
{
	uint8_t tmprssi, buf[260];
	volatile int32_t rssi;
	uint16_t MyID, YourID, masterID, slaveID, big_loop, num_packets;
  uint8_t efeRaw[3];
  uint32_t efe = 0;
  double efeHz = 0.0;


	//chip antenna has many lobes
	//whip is flat (�1.5dB) radiating in all directions equally
	//g_ant_int_ext - current antenna switch setting
	//ANT_SEL=0;//Chip Antenna
	//ANT_SEL=1;//Whip
	//*** Do not change int/ext antenna selection here.  
	//*** Use the antenna that was active when Ranging began
		
	GoToStandby(0);//0-STDBY_RC  1-STDBY_XOSC

	//read status register for the RSSI byte
	ReadCommand2( RADIO_GET_PACKETSTATUS, buf, 1 );//only need first of 5 bytes - that where the RSSI value is located
	tmprssi = buf[0];
	rssi = -( tmprssi / 2 );	//rssi of packet received by the slave

	//not used with this method of ranging.  
	ReadRegister_16bAddress(REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, efeRaw, 3);  //0x954
	efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
	//efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK; //0x000FFFFF  
	efe &= 0x000FFFFF;
	//printf( "efe: 0x%X  ", efe);
	efeHz = 1.55 * ( double )complement2( efe, 20 ) / ( 1600.0 / 1625000.0 * 1000.0);
	//printf( "RSSI: %d   efeHz: %lf   \n\r", tmp/2*-1, efeHz);

	buf[0] = (((int32_t)efeHz)>>24 ) & 0xFF ;
	buf[1] = (((int32_t)efeHz)>>16 ) & 0xFF ;
	buf[2] = (((int32_t)efeHz)>>8 ) & 0xFF ;
	buf[3] = (((int32_t)efeHz)& 0xFF );

	//printf( "RSSI: %d,   EstFreqErr: 0x%X  efe: 0x%X  \n\r", rssi, efeHz, efe);
	
	//load transmit data for RSSI valuse return to master
	buf[4] = 0x23;//ranging request
	buf[5] = tmprssi;
	WriteTXBuffer(0, buf, 6);//TX Buffer offset=0, 6 byte payload
	SetTx(); //transmit  with spreading factor set to DEFAULTSPREADINGFACTOR
	while(ALL3IRQ == 0);//wait for TX interrupt to go high
	ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt


	//Set packet type - Ranging
	buf[0] = RANGING;   //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
	WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
	if(rssi > RSSITHRESHOLD)//SF6 for short distances (<100ft)
	{
		MyTicker.attach(&int_acq, TICKERSF6); // mS tick for each reading: 10 batches of 40 channels = 400 readings 
		spreading_factor = RANGINGSPREADINGFACTOR6;//the use of SF11 or SF12 are not permitted for Ranging P116
	}
	else //SF10 for long distances (>100ft)
	{
		MyTicker.attach(&int_acq, TICKERSF10); // mS tick for each reading: 10 batches of 40 channels = 400 readings 
		spreading_factor = RANGINGSPREADINGFACTOR10;//the use of SF11 or SF12 are not permitted for Ranging P116
	}
	SetAllModulationParameters(LORA, DEFAULTMODLORAP1, DEFAULTMODLORAP2, DEFAULTMODLORAP3, spreading_factor);//0x8B
	SetAllPacketParameters(LORA, DEFAULTPACKETLORAP1, DEFAULTPACKETLORAP2, DEFAULTPACKETLORAP3, DEFAULTPACKETLORAP4, DEFAULTPACKETLORAP5, DEFAULTPACKETLORAP6, DEFAULTPACKETLORAP7);//0x8C

	//set my Ranging role - SLAVE
	buf[0] = RANGINGSLAVE;
	WriteCommand2( RADIO_SET_RANGING_ROLE, buf, 1 );//0xA3
	
	//set display to indicate ranging mode active
  writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
	writecom(0x02, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
	write_string_to_LCD("Ranging mode on ", 16);

	//Set TX parameters - power and TX ramp time
	buf[0] = 31;    //-18+31 yields +13dBm (+12.5 max) Xmit power
	buf[1] = 0xE0;  //20uS ramp time - best for less out of band noise
	WriteCommand2( RADIO_SET_TXPARAMS, buf, 2 );//0x8E P72

	//read master and slave IDs from Flash
	MyID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MYID);//Read 32-bit word
	YourID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_YOURID);//Read 32-bit word
	//this is the code for slave, therefore I am slave.  My address is MyID.
	//the other transceiver is master (YourID) 
	//the shared 32-bit shared RANGING ID is always masterID<<8 + slaveID


	masterID = YourID;
	slaveID = MyID;

	//Load Shared Ranging Address 
	buf[0] = (uint8_t)((masterID>>8) & 0x00FF); //31:24  masterID MSByte
	WriteRegister_16bAddress(0x916, buf, 1);
	buf[0] = (uint8_t)(masterID & 0x00FF); //23:16  masterID LSByte
	WriteRegister_16bAddress(0x917, buf, 1);
	buf[0] = (uint8_t)((slaveID>>8) & 0x00FF); //15:8  slaveID MSByte
	WriteRegister_16bAddress(0x918, buf, 1);
	buf[0] = (uint8_t)(slaveID & 0x00FF); //7:0 slaveID LSByte
	WriteRegister_16bAddress(0x919, buf, 1);

	//Ranging Address Bit Definition
	ReadRegister_16bAddress(0x0931, buf, 1);
	buf[0] = (buf[0] | 0xC0); //select 32 bit master/slave address
	WriteRegister_16bAddress(0x0931, buf, 1);

	//Setup Interrupts P80 P81    SLAVE  0x4183
	//enable Ranging interrupts for Slave
	buf[0] = 0x41;  //0100 0001 irqMask  15:8  RangingReqDisc enabled 
	buf[1] = 0x83;  //1000 0011 irqMask   7:0  RangingRespDone enabled P80 
	//tied to the 3 DIO pins on the SX1280
	//**NOTE: DIO1, DIO2 and DIO3 are all OR'd together on the DLP-RFS1280ACT board
	buf[2] = 0x41;  //0100 0001 dio1Mask 15:8  tied to DIO1
	buf[3] = 0x83;  //1000 0011 dio1Mask  7:0  tied to DIO1.  
	buf[4] = 0x00;  //0000 0000 dio2Mask 15:8  
	buf[5] = 0x00;  //0000 0000 dio2Mask  7:0
	buf[6] = 0x00;  //0000 0000 dio3Mask 15:8
	buf[7] = 0x00;  //0000 0000 dio3Mask  7:0
	WriteCommand2( RADIO_SET_DIOIRQPARAMS, buf, 8 );//0x8D

	//Write the RxTx delay offset calibration value (from the Semtech App note)
	buf[0] = (uint8_t)(RXTXDELAYCALOFFSET & 0xFF); //13430 for BW:1600 and SF=9
	WriteRegister_16bAddress(0x092D, buf, 1);//address used in Semtech demo code
	buf[0] = (uint8_t)((RXTXDELAYCALOFFSET>>8) & 0xFF);
	WriteRegister_16bAddress(0x092C, buf, 1);//address used in Semtech demo code

	current_ranging_channel=0;
	SetRfFrequency(Channels[current_ranging_channel]);//set TX freq to first in hop list
	myinterruptmode = RANGINGCHCHANGE;	
	time_to_change_channels=0;//clear flag
	
	if(rssi > RSSITHRESHOLD)//SF6 for short distances (<100ft)
	{
		MyTicker.attach(&int_acq, TICKERSF6); // mS tick for each reading
		num_packets = 200;
	}
	else //SF10 for long distances (>100ft)
	{
		MyTicker.attach(&int_acq, TICKERSF10); // mS tick for each reading
		num_packets = 80;
	}
	
	
	
	if(!SW2)
	{
		g_ant_int_ext ^=1;//toggle selection of antenna (internal Vs external)
		ANT_SEL = g_ant_int_ext;//0-chip  1-whip
	}
	if(g_ant_int_ext==0)//chip
	{
		writecom(0xCC, 0);//position cursor 2nd row
		write_string_to_LCD("CHIP", 4);
	}
	else
		{				
			writecom(0xCC, 0);//position cursor 2nd row
			write_string_to_LCD("WHIP", 4);
		}

	
	for(big_loop=0; big_loop<num_packets; big_loop++)
	{
					ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
					SetRx();//stays in this mode until packet received or time for channel change
					while((ALL3IRQ == 0) && (time_to_change_channels==0));//wait for RX or channel change interrupts
					ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt

					while(time_to_change_channels==0);//wait for channel change interrupt - may already be there if packet not received
					ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt

					time_to_change_channels=0;//clear flag
					current_ranging_channel++;
					if(current_ranging_channel>39)
					{
						current_ranging_channel=0;//test
						SetRfFrequency(Channels[current_ranging_channel]);			
					}
					else			
						SetRfFrequency(Channels[current_ranging_channel]);			
		}
	
		GoToStandby(0);//0-STDBY_RC  1-STDBY_XOSC
		MyTicker.detach(); //disable this timer
}


//*******************************************************************************
void ReceiveYourID(uint16_t MyID, uint8_t CurrentMode)
{
	volatile uint8_t x;
	uint8_t buf[260], IDreceived;
	volatile uint16_t YourNewID, temp;
	uint16_t YourID;
	volatile uint8_t b0, b1;
		
	IDreceived=0;
	
	GoToStandby(0);//go to standby mode and check current status

	//Set packet type - GFSK
	buf[0] = LORA;   //0-GFSK, 1-LORA, 3-FLRC
	spreading_factor = 6;
	WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
	SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
	//SetAllModulationParameters(LORA, 13, 4, 2, spreading_factor);//0x8B
	//SetAllPacketParameters(LORA, 2, 2, 2, 1, 5, 2, 2);//0x8C

	//ANT_SEL=0;//Chip Antenna
	//ANT_SEL=1;//Whip
	ANT_SEL = 0;

	SetRfFrequency( LOW );

	//set TX and RX buffer base address
	buf[0] = 0x00;  //TX Base Address
	buf[1] = 0x00;  //RX Base Address
	WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F
	
	writecom(0xC0, 0);//mode=0 instruction/command,    mode=1 data  position cursor to start of second row
	write_string_to_LCD("YU:wait ", 8);

	SelectInterrupts();//enable TxDone and RxDone 
	while(IDreceived == 0)
	{
		SetRx();//stays in this mode until packet received or RESET button pressed
		while(ALL3IRQ == 0);//wait for TX interrupt to go high
		ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
		while(ALL3IRQ == 1);//wait for TX interrupt to go low

		ReadRXBuffer(0, buf, 5);

		if(buf[4] == 0x20)
			IDreceived=1;
	}

	YourNewID = (uint16_t)buf[0];
	temp = (uint16_t)buf[1];
	YourNewID |= (temp<<8);
	
	EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, MyID); //MyID
	EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, YourNewID); //Your new ID
	EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MODMODE, CurrentMode); //PURmodmode

	YourID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_YOURID);//Read 32-bit word
	
	//write to LCD
	writecom(0xC0, 0);//mode=0 instruction/command,    mode=1 data  position cursor to start of second row
	write_string_to_LCD("YU:", 3);
	write_int_to_LCD(YourID, 5);

	while(1);//lock up - reset required
}



//*******************************************************************************
void SendMyID(uint16_t MyID)
{
  uint8_t buf[260], spreading_factor;
	
	DisplayTR(0);//0-T 1-R  (for TX or RX modes)
	
	GoToStandby(0);//go to standby mode and check current status

	buf[0] = LORA;   //0-GFSK, 1-LORA, 3-FLRC
	spreading_factor = 6;
	WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A
	SetTypicalRegisterSettings(LORA, spreading_factor); //these setting can be also changed by the following 2 function calls...
	//SetAllModulationParameters(GFSK, 13, 4, 2, spreading_factor);//0x8B
	//SetAllPacketParameters(GFSK, 2, 2, 2, 1, 5, 2, 2);//0x8C


	//ANT_SEL=0;//Chip Antenna
	//ANT_SEL=1;//Whip
	ANT_SEL = 0;

	SetRfFrequency( LOW );
	
	//Set TX parameters - power and TX ramp time
	buf[0] = 31;    //-18+31 yields +13dBm (+12.5 max) Xmit power
	buf[1] = 0xE0;  //20uS ramp time - best for less out of band noise
	WriteCommand2( RADIO_SET_TXPARAMS, buf, 2 );//0x8E P72

	//set TX and RX buffer base address
	buf[0] = 0x00;  //TX Base Address
	buf[1] = 0x00;  //RX Base Address
	WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F

	//load transmit data for modulation tests
	buf[0] = (uint8_t)(MyID&0xff);      //MyID LSB
	buf[1] = (uint8_t)((MyID>>8)&0xff);  //MyID MSB
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x20;//pairing command 
	WriteTXBuffer(0, buf, 5);//TX Buffer offset=0, 5 byte payload

	SelectInterrupts();//enable TxDone and RxDone 

	SetTx(); //transmit with current settings
	while(ALL3IRQ == 0);//wait for TX interrupt to go high
	ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
	while(ALL3IRQ == 1);//wait for TX interrupt to go low

	DisplayTR(1);//0-T 1-R  (for TX or RX modes)
}





//*******************************************************************************
void EnterPingTestMode(uint16_t MyID, uint16_t YourID)
{
	uint8_t buf[260], CurrentMode;

	//set up transceiver to repeatedly send Ping packets (0x21) to paired transceiver 
	//every 500mS.
	//when paired transceiver receives this packet it will beep the speaker
	while(!SW5);//wait for button release		

	//set TX and RX buffer base address
	buf[0] = 0x00;  //TX Base Address
	buf[1] = 0x00;  //RX Base Address
	WriteCommand2( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );//0x8F

	//load transmit data for modulation tests
	buf[0] = (uint8_t)MyID & 0x00FF;
	buf[1] = (uint8_t)((MyID>>8) & 0xff);
	buf[2] = (uint8_t)YourID & 0x00FF;
	buf[3] = (uint8_t)((YourID>>8) & 0xff);
	buf[4] = 0x21;
	WriteTXBuffer(0, buf, 5);//TX Buffer offset=0, 5 byte payload

	while(1)
	{	
		SetTx(); //transmit with current settings
		while(ALL3IRQ == 0);//wait for TX interrupt to go high
		ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
		while(ALL3IRQ == 1);//wait for TX interrupt to go low
		
		if(!SW5)//change channels
		{
			while(!SW5);//wait for button release		
			LMHsetting++;
			if(LMHsetting>3)
				LMHsetting=1;
	
			if(LMHsetting==1)	SetRfFrequency( LOW );
			if(LMHsetting==2)	SetRfFrequency( MID );
			if(LMHsetting==3)	SetRfFrequency( HIGH );
			DisplayLMH(LMHsetting);//1-Low 2-Mid 3-High - shows current transmit channel
		}			
		
		if(!SW2)//change antenna (internal/external)
		{
			while(!SW2);
			g_ant_int_ext^=1;
			ANT_SEL = g_ant_int_ext;//toggle antenna
			DisplayChipWhip(g_ant_int_ext);//0chip  1whip				
		}

		if(!SW3) //increase spreading factor wrapping from 12 back to 5
		{
			while(!SW3);

			GoToStandby(0);//go to standby mode and check current status
			
			spreading_factor++;   //5-12
			if(spreading_factor>12) 
				spreading_factor=5;
			CurrentMode = LORA;
			buf[0] = LORA;
			WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A

			//update EEPROM every time the mode is changed so system will power up in this mode
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, MyID); //MyID
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, YourID); //YourID
			EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MODMODE, CurrentMode); //PURmodmode
			
			writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
			write_string_to_LCD("LORA", 4);
			writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
			write_string_to_LCD("SF", 2);
			writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
			write_int_to_LCD(spreading_factor, 2);
			writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home

			SetAllModulationParameters(CurrentMode, spreading_factor, DEFAULTMODLORAP2, DEFAULTMODLORAP3, spreading_factor);
			
			//enter and stay in receive mode.  
			ClearIrqStatus( IRQ_RADIO_ALL );//clear the interrupt
			SetRx();//stays in this mode until packet received or RESET button pressed
		}
	
		DisplayTR(0);//0-T 1-R  (for TX or RX modes)
		wait_us(1000 *300);//pause for half second	 ----  remove this wait to see the packets on the spectrum analyzer................................	
		DisplayTR(1);//0-T 1-R  (for TX or RX modes)
		wait_us(1000 *300);//pause for half second	 ----  remove this wait to see the packets on the spectrum analyzer................................	
	}	
}
	


//*******************************************************************************
void EnterCWModTestMode()
{
	uint8_t buf[5];


	while(!SW3);//wait for button release		

	DisplayTR(0);//0-T 1-R  (for TX or RX modes)

	if(LMHsetting==1)	SetRfFrequency( LOW );
	if(LMHsetting==2)	SetRfFrequency( MID );
	if(LMHsetting==3)	SetRfFrequency( HIGH );
	DisplayLMH(LMHsetting);//1-Low 2-Mid 3-High - shows current transmit channel

	while(1)
	{
		SetTx(); //transmit with current settings
		while(ALL3IRQ == 0);//wait for TX interrupt to go high
		ClearIrqStatus( IRQ_RADIO_ALL );//clear all interrupts
		while(ALL3IRQ == 1);//wait for TX interrupt to go low

		wait_us(1000 *1);
		
		if(!SW3)//change spreading factor
		{
			while(!SW3);//wait for button release		
			
			GoToStandby(0);//go to standby mode and check current status
			
			spreading_factor++;   //5-12
			if(spreading_factor>12) spreading_factor=5;
			buf[0] = LORA;
			WriteCommand2( RADIO_SET_PACKETTYPE, buf, 1 );//0x8A

			//writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
			writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
			write_string_to_LCD("LORA", 4);
			writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
			write_string_to_LCD("SF", 2);
			writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
			write_int_to_LCD(spreading_factor, 2);
			writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home

			SetAllModulationParameters(LORA, DEFAULTMODLORAP1, DEFAULTMODLORAP2, DEFAULTMODLORAP3, spreading_factor);
		}	
		
		if(!SW5)//change carrier freq
		{
			while(!SW5);//wait for button release		

			LMHsetting++;
			if(LMHsetting>3)
				LMHsetting=1;
	
			if(LMHsetting==1)	SetRfFrequency( LOW );
			if(LMHsetting==2)	SetRfFrequency( MID );
			if(LMHsetting==3)	SetRfFrequency( HIGH );
			DisplayLMH(LMHsetting);//1-Low 2-Mid 3-High - shows current transmit channel
		}			
		
		if(!SW2)//toggle internal / external antenna
		{
			while(!SW2);
			g_ant_int_ext^=1;
			ANT_SEL = g_ant_int_ext;//toggle antenna
			DisplayChipWhip(g_ant_int_ext);//0chip  1whip				
		}
	}//end while(1)
}

//*******************************************************************************
void EnterCWTestMode()//must reset micro to exit this mode
{
	uint8_t buf[10];
	

	while(!SW1);//wait for button release		

	DisplayTR(0);//0-T 1-R  (for TX or RX modes)

	if(LMHsetting==1)	SetRfFrequency( LOW );
	if(LMHsetting==2)	SetRfFrequency( MID );
	if(LMHsetting==3)	SetRfFrequency( HIGH );

	//transmit cont.
	buf[0]=0;
	WriteCommand2( RADIO_SET_TXCONTINUOUSWAVE, buf, 0 );//0xD1

	while(1)
		{
			if(!SW5)
			{
				while(!SW5);//wait for button release		
				LMHsetting++;
				if(LMHsetting>3)
					LMHsetting=1;
		
				if(LMHsetting==1)	SetRfFrequency( LOW );
				if(LMHsetting==2)	SetRfFrequency( MID );
				if(LMHsetting==3)	SetRfFrequency( HIGH );
				DisplayLMH(LMHsetting);//1-Low 2-Mid 3-High - shows current transmit channel
  		}			
					
			if(!SW2)//toggle antenna
			{
				while(!SW2);
				g_ant_int_ext^=1;
				ANT_SEL = g_ant_int_ext;
				DisplayChipWhip(g_ant_int_ext);//0chip  1whip				
			}
		}//end while(1)
}


//*******************************************************************************
void DisplayTR(unsigned char txrx)//0-T 1-R  (for TX or RX modes)
{
	writecom(0xCF, 0);//mode=0 instruction/command,    mode=1 data  position cursor
	
	if(txrx == 0)
	{
		write_string_to_LCD("T", 1);
	}
	if(txrx == 1)
	{
		write_string_to_LCD("R", 1);
	}
}

//*******************************************************************************
void DisplayLMH(unsigned char lmh)//1-Low 2-Mid 3-High
{
	writecom(0xCD, 0);//mode=0 instruction/command,    mode=1 data  position cursor
	
	if(lmh == 1)
	{
		write_string_to_LCD("L", 1);
	}
	if(lmh == 2)
	{
		write_string_to_LCD("M", 1);
	}
	if(lmh == 3)
	{
		write_string_to_LCD("H", 1);
	}
}

//*******************************************************************************
void DisplayChipWhip(unsigned char chipwhip)//0chip  1whip
{
	writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
	
	if(chipwhip)
	{
		write_string_to_LCD("W", 1);//W=Whip,   C=Chip
	}
	else
	{
		write_string_to_LCD("C", 1);//W=Whip,   C=Chip
	}
}





//***************************************************************************
void initLCD(uint8_t CurrentMode, uint8_t spreading_factor, uint16_t MyID, uint16_t YourID)
{
	wait_us( 5 );
	writecom(0x30,0); //wake up
	wait_us( 5 );
	writecom(0x30,0); //wake up
	wait_us( 5 );
	writecom(0x30,0); //wake up
	wait_us( 5 );
	writecom(0x39,0); //function set
	wait_us( 5 );
	writecom(0x14,0); //internal osc frequency
	wait_us( 5 );
	writecom(0x56,0); //power control
	wait_us( 5 );
	writecom(0x6D,0); //follower control
	wait_us( 5 );
	writecom(0x70,0); //contrast  was 70,0
	//writecom(0x76,0); //contrast  was 70,0
	//writecom(0x7B,0); //contrast  was 70,0
	//writecom(0x7F,0); //contrast  was 70,0
	wait_us( 5 );
	writecom(0x0C,0); //display on
	wait_us( 5 );
	writecom(0x06,0); //entry mode
	wait_us( 5 );
	writecom(0x01,0); //clear
	wait_us( 5 );
	
	writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
	writecom(0x02, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
	write_string_to_LCD("DLP Design      ", 16);
	writecom(0xC0, 0);//mode=0 instruction/command,    mode=1 data  position cursor to start of second row
	write_string_to_LCD("DLP-RFS1280 Demo", 16);

	wait_us(1000 *600);//quick display banner 

	writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
	writecom(0x02, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
	write_string_to_LCD("ME:", 3);
	write_int_to_LCD(MyID, 5);
	writecom(0xC0, 0);//mode=0 instruction/command,    mode=1 data  position cursor to start of second row
	write_string_to_LCD("YU:", 3);
	write_int_to_LCD(YourID, 5);

	if(CurrentMode == GFSK)
	{
		writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
		write_string_to_LCD("GFSK", 4);
	}	
	if(CurrentMode == FLRC)
	{
		writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
		write_string_to_LCD("FLRC", 4);
	}	

	if(CurrentMode == LORA)
	{
		//writecom(0x01, 0);//mode=0 instruction/command,    mode=1 data  clear display
		writecom(0x8C, 0);//mode=0 instruction/command,    mode=1 data  place cursor at row1 pos9
		write_string_to_LCD("LORA", 4);
		writecom(0x89, 0);//mode=0 instruction/command,    mode=1 data  position cursor
		write_string_to_LCD("SF", 2);
		writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
		write_int_to_LCD(spreading_factor, 2);
		writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
	}

	//write_string_to_LCD("C", 1);//W=Whip,   C=Chip	
	DisplayChipWhip(g_ant_int_ext);//0-chip  1-whip				
	DisplayLMH(LMHsetting);//1-Low 2-Mid 3-High - shows current transmit channel
	DisplayTR(1);//0-T 1-R  (for TX or RX modes)
}





//******************************************************************************
void writecom(unsigned char d, unsigned char mode)
{
	unsigned char serialcounter;

	LCS=0;
	if(mode==0) LRS=0;//RS=0 instruction/command
	if(mode==1) LRS=1;//RS-1 data

	wait_us( 5 );	

	for(serialcounter = 1; serialcounter <= 8; serialcounter++) //send 8 bits
	{
		if((d&0x80)==0x80) //if MSB is high
		{
			LSI=1;//SI high
		}
		   else
		{
			LSI=0;//SI low
			wait_us( 5 );
		}

		d=(d<<1); //shift data byte left
		LCLK=0;//clock low
		wait_us( 5 );
		LCLK=1;//clock high (idle)
		wait_us( 5 );
		LCLK=0;//clock low
		wait_us( 5 );
	}

	wait_us( 5 );
	LCS=1;
	wait_us( 5 );
}






//**************************************************************************************************
void write_string_to_LCD(char *strdata, unsigned char len)
{
	unsigned char loop;

	for(loop=0; loop<len; loop++)
		writecom(strdata[loop], 1);//mode=0 instruction/command,    mode=1 data  A
}



//**************************************************************************************************
void write_int_to_LCD(unsigned int x, unsigned char num_digits)
{
	char buf[13], y;

	if(num_digits==1)
		sprintf(buf,"%01d", x);
	if(num_digits==2)
		sprintf(buf,"%02d", x);
	if(num_digits==3)
		sprintf(buf,"%03d", x);
	if(num_digits==4)
		sprintf(buf,"%04d", x);
	if(num_digits==5)
		sprintf(buf,"%05d", x);
	if(num_digits==6)
		sprintf(buf,"%06d", x);
	if(num_digits==7)
		sprintf(buf,"%07d", x);
	if(num_digits==8)
		sprintf(buf,"%08d", x);
	for(y=0; y<num_digits; y++)
		writecom(buf[y], 1);//mode=0 instruction/command,    mode=1 data
}


//**************************************************************************************************
void write_float_to_LCD(float x)
{
	char buf[10];

	sprintf(buf,"%f", x) ;
	writecom(buf[0], 1);//mode=0 instruction/command,    mode=1 data  A
	writecom(buf[1], 1);//mode=0 instruction/command,    mode=1 data  A
	writecom(buf[2], 1);//mode=0 instruction/command,    mode=1 data  A
}







//********************************************************************************************
void SelectInterrupts()
{
  uint8_t buf[10];
	
	//P80 P81   0x4003
	//enable TX done interrupt - not using interrupts for this project in code - only watching the DIOx pin for TX/RX complete (polling)
	buf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
	buf[1] = 0x03;  //irqMask   7:0  enable TxDone and RxDone P80 interrupts
	
	buf[2] = 0x40;  //dio1Mask 15:8
	buf[3] = 0x03;  //dio1Mask  7:0  map TXDone and RxDone to DIO1. NOTE: DIO1, DIO2 and DIO3 are all OR'd together on the DLP-RFS1280ACT board
	buf[4] = 0x00;  //dio2Mask 15:8
	buf[5] = 0x00;  //dio2Mask  7:0
	buf[6] = 0x00;  //dio3Mask 15:8
	buf[7] = 0x00;  //dio3Mask  7:0
	WriteCommand2( RADIO_SET_DIOIRQPARAMS, buf, 8 );//0x8D
}


//********************************************************************************************
void LoadTXBuf()
{
  uint8_t buf[260];

	//load transmit data for modulation tests
	buf[0] = 0xAA;
	buf[1] = 0x55;
	buf[2] = 0xAA;
	buf[3] = 0x55;
	buf[4] = 0xAA;
	buf[5] = 0x55;
	buf[6] = 0xAA;
	buf[7] = 0x55;
	buf[8] = 0xAA;
	buf[9] = 0x55;
	buf[10] = 0xAA;
	buf[11] = 0x55;
	buf[12] = 0xAA;
	buf[13] = 0x55;
	buf[14] = 0xAA;
	buf[15] = 0x55;
	
	buf[0] = 0x00;
	buf[1] = 0xFF;
	buf[2] = 0x00;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	buf[5] = 0xFF;
	buf[6] = 0x00;
	buf[7] = 0xFF;
	buf[8] = 0x00;
	buf[9] = 0xFF;
	buf[10] = 0x00;
	buf[11] = 0xFF;
	buf[12] = 0x00;
	buf[13] = 0xFF;
	buf[14] = 0x00;
	buf[15] = 0xFF;

	WriteTXBuffer(0, buf, 16);//TX Buffer offset=0, 16 byte payload
}




//********************************************************************************************
uint8_t	SetTypicalRegisterSettings(uint8_t modmode, uint8_t spreading_factor)
{
	uint8_t status, p1, p2, p3, p4, p5, p6, p7;
	
	if(modmode == GFSK)
	{
		p1 = DEFAULTMODGFSKP1;
		p2 = DEFAULTMODGFSKP2;
		p3 = DEFAULTMODGFSKP3;
		SetAllModulationParameters(modmode, p1, p2, p3, spreading_factor); //spreading_factor onl used in LORA mode
		p1 = DEFAULTPACKETGFSKP1;
		p2 = DEFAULTPACKETGFSKP2;
		p3 = DEFAULTPACKETGFSKP3; 
		p4 = DEFAULTPACKETGFSKP4; 
		p5 = DEFAULTPACKETGFSKP5;
		p6 = DEFAULTPACKETGFSKP6; //4 byte option
		p7 = DEFAULTPACKETGFSKP7; 
		SetAllPacketParameters(modmode, p1, p2, p3, p4, p5, p6, p7);
	}
	if(modmode == LORA)
	{
		p1 = DEFAULTMODLORAP1;
		p2 = DEFAULTMODLORAP2;
		p3 = DEFAULTMODLORAP3;
		SetAllModulationParameters(modmode, p1, p2, p3, spreading_factor); //spreading_factor onl used in LORA mode
		p1 = DEFAULTPACKETLORAP1;
		p2 = DEFAULTPACKETLORAP2;
		p3 = DEFAULTPACKETLORAP3; 
		p4 = DEFAULTPACKETLORAP4; 
		p5 = DEFAULTPACKETLORAP5;
		p6 = DEFAULTPACKETLORAP6; 
		p7 = DEFAULTPACKETLORAP7; 
		SetAllPacketParameters(modmode, p1, p2, p3, p4, p5, p6, p7);
	}
	
	if(modmode == FLRC)
	{
		p1 = DEFAULTMODFLRCP1; 
		p2 = DEFAULTMODFLRCP2; 
		p3 = DEFAULTMODFLRCP3; 
		SetAllModulationParameters(modmode, p1, p2, p3, spreading_factor); //spreading_factor onl used in LORA mode
		p1 = DEFAULTPACKETFLRCP1; //   P103
		p2 = DEFAULTPACKETFLRCP2; //   P104
		p3 = DEFAULTPACKETFLRCP3; 
		p4 = DEFAULTPACKETFLRCP4; 
		p5 = DEFAULTPACKETFLRCP5; 
		p6 = DEFAULTPACKETFLRCP6; 
		p7 = DEFAULTPACKETFLRCP7; 
		SetAllPacketParameters(modmode, p1, p2, p3, p4, p5, p6, p7);
	}

	return status;
}





//********************************************************************************************
void GoToStandby(uint8_t mode)//0-STDBY_RC  1-STDBY_XOSC
{
	volatile uint8_t status;
  uint8_t buf[10];
	
	if(mode == 0)
		buf[0] = STDBY_RC;//0x00
	else
		buf[0] = STDBY_XOSC;//0x01		
  WriteCommand2( RADIO_SET_STANDBY, buf, 1 );//0x80
	
	//check status 
	status = ReadCommand2( RADIO_GET_STATUS, buf, 1 );//0xC0
	//status:
	//010x xxxx - 2 STDBY_RX
	//011x xxxx - 3 STCBY_XOSC
	//100x xxxx - 4 FS active  - here most of the time...
	//101x xxxx - 5 RX
	//110x xxxx - 6 TX
	//xxx0 01xx - 1 Command successfully processed
	//xxx0 10xx - 2 data ready to be read
	//xxx0 11xx - 3 Command timeout
	//xxx1 00xx - 4 Command processing error
	//xxx1 01xx - 5 Failure to execute command
	//xxx1 10xx - 6 Packet TX complete
}




//                GFSK/FLRC            LoRa/Ranging
//PacketParam0:   PreambleLength       PreambleLength
//PacketParam1:   SyncWordSize         HeaderType
//PacketParam2:   SyncWordMask         PayloadLength
//PacketParam3:   HeaderType           CRC
//PacketParam4:   PayloadLength        InvertIQ/chirp invert
//PacketParam5:   CrcType              not used
//PacketParam6:   WhiteningEnable      not used
//********************************************************************************************
void SetAllPacketParameters(uint8_t CurrentMode, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, uint8_t p7)
{
  uint8_t buf[20];

	if(CurrentMode == GFSK)
	{
		switch(p1) //Parameter 1 settings for GFSK
		{
			case 0: //                                           Preamble length in bits
			case 1:  buf[0] = 0x00;   //PREAMBLE_LENGTH_04_BITS  4
				break;	
			case 2:  buf[0] = 0x10;   //PREAMBLE_LENGTH_08_BITS  8
				break;	
			case 3:  buf[0] = 0x20;   //PREAMBLE_LENGTH_12_BITS  12
				break;	
			case 4:  buf[0] = 0x30;   //PREAMBLE_LENGTH_16_BITS  16
				break;	
			case 5:  buf[0] = 0x40;   //PREAMBLE_LENGTH_20_BITS  20
				break;	
			case 6:  buf[0] = 0x50;   //PREAMBLE_LENGTH_24_BITS  24
				break;	
			case 7:  buf[0] = 0x60;   //PREAMBLE_LENGTH_28_BITS  28
				break;	
			case 8:  buf[0] = 0x70;   //PREAMBLE_LENGTH_32_BITS  32
				break;	

			default: buf[0] = 0x10;
				break;
		}

		switch (p2) //Parameter 2 settings for GFSK
		{
			case 0: //                                         Sync Word size in bytes
			case 1:  buf[1] = 0x00;   //SYNC_WORD_LEN_1_B      1
				break;	
			case 2:  buf[1] = 0x02;   //SYNC_WORD_LEN_2_B      2
				break;	
			case 3:  buf[1] = 0x04;   //SYNC_WORD_LEN_3_B      3
				break;	
			case 4:  buf[1] = 0x06;   //SYNC_WORD_LEN_4_B      4
				break;	
			case 5:  buf[1] = 0x08;   //SYNC_WORD_LEN_5_B      5
				break;	

      default:  buf[1] = 0x00;
				break;	
		}

		switch (p3)  //Parameter 3 settings for GFSK
		{
			case 0: //                                                   Sync Word combination to use
			case 1:  buf[2] = 0x00;   //RADIO_RX_MATCH_SYNCWORD_OFF      Disable Sync Word
				break;	
			case 2:  buf[2] = 0x10;   //RADIO_RX_MATCH_SYNCWORD_1        SyncWord1
				break;	
			case 3:  buf[2] = 0x20;   //RADIO_RX_MATCH_SYNCWORD_2        SyncWord2
				break;	
			case 4:  buf[2] = 0x30;   //RADIO_RX_MATCH_SYNCWORD_1_2      SyncWord1 or SyncWord2
				break;	
			case 5:  buf[2] = 0x40;   //RADIO_RX_MATCH_SYNCWORD_3        SyncWord3
				break;	
			case 6:  buf[2] = 0x50;   //RADIO_RX_MATCH_SYNCWORD_1_3      SyncWord1 or SyncWord3
				break;	
			case 7:  buf[2] = 0x60;   //RADIO_RX_MATCH_SYNCWORD_2_3      SyncWord2 or SyncWord3
				break;	
			case 8:  buf[2] = 0x70;   //RADIO_RX_MATCH_SYNCWORD_1_2_3    SyncWord1, SyncWord2 or SyncWord3
				break;	

			default:  buf[2] = 0x00;
				break;
		}
		
		switch (p4)  //Parameter 4 settings for GFSK
		{
			case 0: //                                                  Packet Length mode
			case 1:  buf[3] = 0x00;   //RADIO_PACKET_FIXED_LENGTH       FIXED LENGTH MODE
				break;	
			case 2:  buf[3] = 0x20;   //RADIO_PACKET_VARIABLE_LENGTH    VARIABLE LENGTH MODE
				break;	

			default:  buf[3] = 0x00;
				break;
		}

		//Parameter 5 settings for GFSK
		{
			buf[4] = p5;              //Payload length in bytes 0-255
		}
		
		switch (p6)  //Parameter 6 settings for GFSK
		{
			case 0: //                                        CRC type
			case 1:  buf[5] = 0x00;   //RADIO_CRC_OFF         No CRC
				break;	
			case 2:  buf[5] = 0x10;   //RADIO_CRC_1_BYTES     CRC field used 1 byte
				break;	
			case 3:  buf[5] = 0x20;   //RADIO_CRC_2_BYTES     CRC field uses 2 bytes
				break;	
			case 4:  buf[5] = 0x30;   //RADIO_CRC_4_BYTES     CRC field uses 2 bytes - NOT IN DATASHEET!!!!!!!!
				break;	

			default:  buf[5] = 0x00;
				break;
		}

		switch (p7)  //Parameter 7 settings for GFSK
		{
			case 0: //                                        Whitening mode
			case 1:  buf[6] = 0x00;   //WHITENING_ENABLE      WHITENING ENABLE
				break;	
			case 2:  buf[6] = 0x08;   //WHITENING_DISABLE     WHITENING DISABLE
				break;	

			default:  buf[6] = 0x08;
				break;
		}		
	}
		
	
	
	
	
	
	if(CurrentMode == FLRC)
	{
		switch(p1) //Parameter 1 settings for FLRC
		{
			case 0: //                                           Preamble length in bits
			case 1:  buf[0] = 0x00;   //PREAMBLE_LENGTH_4_BITS  Reserved
				break;	
			case 2:  buf[0] = 0x10;   //PREAMBLE_LENGTH_8_BITS  8
				break;	
			case 3:  buf[0] = 0x20;   //PREAMBLE_LENGTH_12_BITS  12
				break;	
			case 4:  buf[0] = 0x30;   //PREAMBLE_LENGTH_16_BITS  16
				break;	
			case 5:  buf[0] = 0x40;   //PREAMBLE_LENGTH_20_BITS  20
				break;	
			case 6:  buf[0] = 0x50;   //PREAMBLE_LENGTH_24_BITS  24
				break;	
			case 7:  buf[0] = 0x60;   //PREAMBLE_LENGTH_28_BITS  28
				break;	
			case 8:  buf[0] = 0x70;   //PREAMBLE_LENGTH_32_BITS  32
  			break;	
	
			default: buf[0] = 0x50;   //PREAMBLE_LENGTH_24_BITS  24
				break;
		}

		switch (p2) //Parameter 2 settings for FLRC
		{
			case 0: //                                           Sync Word size in bytes
			case 1:  buf[1] = 0x00;   //FLRC_SYNC_NOSYNC         21 bits preamble
				break;	
			case 2:  buf[1] = 0x04;   //FLRC_SYNC_WORD_LEN_P32S  21 bits preamble + 32 bits Sync Word
				break;	

      default:  buf[1] = 0x04;   //FLRC_SYNC_WORD_LEN_P32S  21 bits preamble + 32 bits Sync Word
				break;	
		}

		switch (p3) //Parameter 3 settings for FLRC
		{
			case 0: //                                            Sync Word combination to use
			case 1:  buf[2] = 0x00;   //RX_DISABLE_SYNC_WORD      Disable Sync Word
				break;	
			case 2:  buf[2] = 0x10;   //RX_MATCH_SYNC_WORD_1      SyncWord1
				break;	
			case 3:  buf[2] = 0x20;   //RX_MATCH_SYNC_WORD_2      SyncWord2
				break;	
			case 4:  buf[2] = 0x30;   //RX_MATCH_SYNC_WORD_1_2    SyncWord1 or SyncWord2
				break;	
			case 5:  buf[2] = 0x40;   //RX_MATCH_SYNC_WORD_3      SyncWord3
				break;	
			case 6:  buf[2] = 0x50;   //RX_MATCH_SYNC_WORD_1_3    SyncWord1 or SyncWord3
				break;	
			case 7:  buf[2] = 0x60;   //RX_MATCH_SYNC_WORD_2_3    SyncWord2 or SyncWord3
				break;	
			case 8:  buf[2] = 0x70;   //RX_MATCH_SYNC_WORD_1_2_3  SyncWord1 or SyncWord2 or SyncWord3
				break;	

			default:  buf[2] = 0x10;   //RX_MATCH_SYNC_WORD_1      SyncWord1
				break;
		}
		
		switch (p4) //Parameter 4 settings for FLRC
		{
			case 0: //                                           Packet Length mode
			case 1:  buf[3] = 0x00;   //PACKET_FIXED_LENGTH      FIXED LENGTH MODE
				break;	
			case 2:  buf[3] = 0x20;   //PACKET_VARIABLE_LENGTH   VARIABLE LENGTH MODE
				break;	

			default:  buf[3] = 0x00;   //PACKET_FIXED_LENGTH      FIXED LENGTH MODE
				break;
		}
		
		buf[4] = p5;   //Payload length in bytes - must be 6-127... ***************************************************************************
		
		switch (p6) //Parameter 6 settings for FLRC
		{
			case 0: //                                          CRC type
			case 1:  buf[5] = 0x00;   //CRC_OFF                 No CRC
				break;	
			case 2:  buf[5] = 0x10;   //CRC_1_BYTE              CRC field used 1 byte
				break;	
			case 3:  buf[5] = 0x20;   //CRC_2_BYTE              CRC field uses 2 bytes
				break;	
			case 4:  buf[5] = 0x30;   //CRC_3_BYTE              CRC field uses 3 bytes
				break;	

			default:  buf[5] = 0x10;   //CRC_1_BYTE              CRC field used 1 byte
				break;
		}

		//Parameter 7 settings for FLRC
		buf[6] = 0x08;   //In FLRC packet type, it is not possible to enable whitening. You 
		                 //must always set the value of packetParam7 (buf[6]) to disable.
	}
		
		
	if(CurrentMode == LORA)
	{
		//Parameter 1 settings for LORA
		
		//packetParam1 defines the preamble length number expressed in LoRa symbols. Recommended value is 12 symbols.
		// P113           Preamble length in symbols
		//                preamble length =LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
		//mant = 3;
		//exp = 2;
		//buf[0] = mant | (exp<<4);
		//orig		buf[0] = 0x23;   //Recommended value is 12 symbols
		
		//used by Semtech Demo hardware:
		buf[0] = 0x0C;   
		
		switch (p2) //Parameter 2 settings for LORA
		{
			case 0: //                                           Header mode
			case 1:  buf[1] = 0x00;   //EXPLICIT_HEADER          EXPLICIT HEADER
				break;	
			case 2:  buf[1] = 0x80;   //IMPLICIT_HEADER          IMPLICIT HEADER
				break;	

      default:  buf[1] = 0x00;   //EXPLICIT_HEADER         EXPLICIT HEADER
				break;	
		}
 
		//Parameter 3 settings for LORA
		buf[2] = p3;   //PayloadLength

		switch (p4) //Parameter 4 settings for LORA
		{
			case 0: //                                           CRC mode
			case 1:  buf[3] = 0x20;   //LORA_CRC_ENABLE          CRC ENABLE
				break;	
			case 2:  buf[3] = 0x00;   //LORA_CRC_DISABLE         CRC DISABLE
				break;	

      default:  buf[3] = 0x20;   //LORA_CRC_ENABLE         CRC ENABLE
				break;	
		}
 
		switch (p5) //Parameter 5 settings for LORA
		{
			case 0: //                                           LoRa IQ swap
			case 1:  buf[4] = 0x40;   //LORA_IQ_STD              IQ as defined
				break;	
			case 2:  buf[4] = 0x00;   //LORAFLRC_IQ_INVERTED     IQ swapped
				break;	

			default:  buf[4] = 0x40;   //LORA_IQ_STD             IQ as defined
				break;
		}
	}

	//	if(CurrentMode == BLE){}//not inplemented

	WriteCommand2( RADIO_SET_PACKETPARAMS, buf, 7 );//0x8C
}



//Set the 3 modulation parameters
//ModParam1 BLE/GFSK/FLRC:BitrateBandwidth                  LORA/RANGING:SpreadingFactor
//ModParam2 BLE/GFSK: ModulationIndex      FLRC:CodingRate  LORA/RANGING:Bandwidth
//ModParam3 BLE/GFSK/FLRC:ModulationShaping                 LORA/RANGING:CodingRate
//********************************************************************************************
void SetAllModulationParameters(uint8_t CurrentMode, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t spreading_factor)//
{
	uint8_t buf[10], flag1, flag2, flag3;

	flag1=0;
	flag2=0;
	flag3=0;
	
	if(CurrentMode == GFSK)
	{
		switch(p1) //Parameter 1 settings for GFSK
		{
			case 0: //                                         Bitrate (Mb/s)  Bandwidth (MHz DSB)
			case 1:  buf[0] = 0x04;   //FSK_BR_2_000_BW_2_4        2               2.4
				break;	
			case 2:  buf[0] = 0x28;   //FSK_BR_1_600_BW_2_4        1.6             2.4
				break;	
			case 3:  buf[0] = 0x4C;   //FSK_BR_1_000_BW_2_4        1               2.4
				break;	
			case 4:  buf[0] = 0x45;   //FSK_BR_1_000_BW_1_2        1               1.2
				break;	
			case 5:  buf[0] = 0x70;   //FSK_BR_0_800_BW_2_4        0.8             2.4
				break;	
			case 6:  buf[0] = 0x69;   //FSK_BR_0_800_BW_1_2        0.8             1.2
				break;	
			case 7:  buf[0] = 0x8D;   //FSK_BR_0_500_BW_1_2        0.5             1.2
				break;	
			case 8:  buf[0] = 0x86;   //FSK_BR_0_500_BW_0_6        0.5             0.6
				break;	
			case 9:  buf[0] = 0xB1;   //FSK_BR_0_400_BW_1_2        0.4             1.2
				break;	
			case 10: buf[0] = 0xAA;   //FSK_BR_0_400_BW_0_6        0.4             0.6
				break;	
			case 11: buf[0] = 0xCE;   //FSK_BR_0_250_BW_0_6        0.25            0.6
				break;	
			case 12: buf[0] = 0xC7;   //FSK_BR_0_250_BW_0_3        0.25            0.3
				break;	
			case 13: buf[0] = 0xEF;   //FSK_BR_0_125_BW_0_3        0.125           0.3
				break;	

			default: buf[0] = 0x8D;   //FSK_BR_0_500_BW_1_2        0.5             1.2
				break;
		}

		switch (p2) //Parameter 2 settings for GFSK
		{
			case 0: //                                         Modindex
			case 1:  buf[1] = 0x00;   //MOD_IND_0_35             0.35   
				break;	
			case 2:  buf[1] = 0x01;   //MOD_IND_0_5              0.5
				break;	
			case 3:  buf[1] = 0x02;   //MOD_IND_0_75             0.75
				break;	
			case 4:  buf[1] = 0x03;   //MOD_IND_1_00             1
				break;	
			case 5:  buf[1] = 0x04;   //MOD_IND_1_25             1.25
				break;	
			case 6:  buf[1] = 0x05;   //MOD_IND_1_50             1.5
				break;	
			case 7:  buf[1] = 0x06;   //MOD_IND_1_75             1.75
				break;	
			case 8:  buf[1] = 0x07;   //MOD_IND_2_00             2
				break;	
			case 9:  buf[1] = 0x08;   //MOD_IND_2_25             2.25
				break;	
			case 10:  buf[1] = 0x09;   //MOD_IND_2_50            2.5
				break;	
			case 11:  buf[1] = 0x0A;   //MOD_IND_2_75            2.75
				break;	
			case 12:  buf[1] = 0x0B;   //MOD_IND_3_00            3
				break;	
			case 13:  buf[1] = 0x0C;   //MOD_IND_3_25            3.25
				break;	
			case 14:  buf[1] = 0x0D;   //MOD_IND_3_50            3.5
				break;	
			case 15:  buf[1] = 0x0E;   //MOD_IND_3_75            3.75
				break;	
			case 16:  buf[1] = 0x0F;   //MOD_IND_4_00            4
				break;	

      default:  buf[1] = 0x07;   //MOD_IND_2_00            2
				break;	
		}

		switch (p3)  //Parameter 3 settings for GFSK
		{
			case 0: //                                            BT
			case 1:  buf[2] = 0x00;   //BT_OFF                No filtering 
				break;	
			case 2:  buf[2] = 0x10;   //BT_1_0                   1 
				break;	
			case 3:  buf[2] = 0x20;   //BT_0_5                   0.5 
				break;	

			default:  buf[2] = 0x00;   //BT_OFF                No filtering 
				break;
		}
	}

	
	
	
	if(CurrentMode == FLRC)
	{
		switch(p1) //Parameter 1 settings for FLRC
		{
			case 0: //                                         Bitrate (Mb/s)  Bandwidth (MHz DSB)    P32
			case 1:  buf[0] = 0x45;   //FLRC_BR_1_300_BW_1_2   1.3             1.2
				break;	
			case 2:  buf[0] = 0x69;   //FLRC_BR_1_000_BW_1_2   1.04            1.2
				break;	
			case 3:  buf[0] = 0x86;   //FLRC_BR_0_650_BW_0_6   0.65            0.6
				break;	
			case 4:  buf[0] = 0xAA;   //FLRC_BR_0_520_BW_0_6   0.52            0.6
				break;	
			case 5:  buf[0] = 0xC7;   //FLRC_BR_0_325_BW_0_3   0.325           0.3
				break;	
			case 6:  buf[0] = 0xEB;   //FLRC_BR_0_260_BW_0_3   0.26            0.3
				break;	

			default: buf[0] = 0xAA;   //FLRC_BR_0_520_BW_0_6   0.52            0.6
				break;
		}

		switch (p2) //Parameter 2 settings for FLRC
		{
			case 0: //                                         Coding rate     P103
			case 1:  buf[1] = 0x00;   //FLRC_CR_1_2               � 1/2
				break;	
			case 2:  buf[1] = 0x02;   //FLRC_CR_3_4               � 3/4
				break;	
			case 3:  buf[1] = 0x04;   //FLRC_CR_1_0               1
				break;	
			case 4:  buf[1] = 0x05;   //                       Reserved
				break;	

      default:  buf[1] = 0x00;   //FLRC_CR_1_2              � 1/2
				break;	
		}

		switch (p3) //Parameter 3 settings for FLRC
		{
			case 0: //                                            BT
			case 1:  buf[2] = 0x00;   //BT_DIS                No filtering 
				break;	
			case 2:  buf[2] = 0x10;   //BT_1                      1 
				break;	
			case 3:  buf[2] = 0x20;   //BT_0_5                   0.5 
				break;	

			default:  buf[2] = 0x00;   //BT_DIS               No filtering 
				break;
		}
	}
	
	
	if(CurrentMode == LORA)
	{
		p1 = spreading_factor;

		switch(p1) //Parameter 1 settings for LORA
		{
			case 5:  buf[0] = 0x50;   //LORA_SF_5                      5
							 flag1=1;
							 spreading_factor=5;
				break;	
			case 6:  buf[0] = 0x60;   //LORA_SF_6                      6
							 flag1=1;
							 spreading_factor=6;
				break;	
			case 7:  buf[0] = 0x70;   //LORA_SF_7                      7
							 flag2=1;
							 spreading_factor=7;
				break;	
			case 8:  buf[0] = 0x80;   //LORA_SF_8                      8
							 flag2=1;
							 spreading_factor=8;
				break;	
			case 9:  buf[0] = 0x90;   //LORA_SF_9                      9
							 flag3=1;
							 spreading_factor=9;
				break;	
			case 10: buf[0] = 0xA0;   //LORA_SF_10                     10
							 flag3=1;
							 spreading_factor=10;
				break;	
			case 11: buf[0] = 0xB0;   //LORA_SF_11                     11
							 flag3=1;
							 spreading_factor=11;
				break;	
			case 12: buf[0] = 0xC0;   //LORA_SF_12                     12
							 flag3=1;
							 spreading_factor=12;
				break;	

			default: buf[0] = 0x80;   //LORA_SF_8                      8
							 flag2=1;
							 spreading_factor=8;
				break;
		}

		switch (p2) //Parameter 2 settings for LORA
		{
			case 0: //                                             Bandwidth (KHz)
			case 1:  buf[1] = 0x0A;   //LORA_BW_1600                  1625.0
				break;	
			case 2:  buf[1] = 0x18;   //LORA_BW_800                    812.5
				break;	
			case 3:  buf[1] = 0x26;   //LORA_BW_400                    406.25
				break;	
			case 4:  buf[1] = 0x34;   //LORA_BW_200                    203.125 
				break;	

      default:  buf[1] = 0x26;  //LORA_BW_400                    406.25
				break;	
		}
 
		switch (p3) //Parameter 3 settings for LORA
		{
			case 0: //                                             CodingRate
			case 1:  buf[2] = 0x01;   //LORA_CR_4_5                   4/5
				break;	
			case 2:  buf[2] = 0x02;   //LORA_CR_4_6                   4/6
				break;	
			case 3:  buf[2] = 0x03;   //LORA_CR_4_7                   4/7
				break;	
			case 4:  buf[2] = 0x04;   //LORA_CR_4_8                   4/8
				break;	
			case 5:  buf[2] = 0x05;   //LORA_CR_LI_4_5                4/5  Long Interleave
				break;	
			case 6:  buf[2] = 0x06;   //LORA_CR_LI_4_6                4/6  Long Interleave
				break;	
			case 7:  buf[2] = 0x07;   //LORA_CR_LI_4_7                4/8
				break;	

			default:  buf[2] = 0x04;  //LORA_CR_4_8                   4/8
				break;
		}

		//put spreading factor on the LCD display
		writecom(0xC9, 0);//mode=0 instruction/command,    mode=1 data  position cursor 
		//write_int_to_LCD(spreading_factor, 2);
		writecom(0xCC, 0);//mode=0 instruction/command,    mode=1 data  position cursor home home
	}

	//if(CurrentMode == BLE){}
	
	WriteCommand2( RADIO_SET_MODULATIONPARAMS, buf, 3 );//0x8B
	
	//Datasheet Note P112 - after SetModulationParams command:
	//If the Spreading Factor selected is SF5 or SF6, it is required to use WriteRegister( 0x925, 0x1E )
	if(flag1)
	{
		buf[0] = 0x1E;	
		WriteRegister_16bAddress(0x925, buf, 1);
	}

	//If the Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
	if(flag2)
	{
		buf[0] = 0x37;	
		WriteRegister_16bAddress(0x925, buf, 1);
	}
	//If the Spreading Factor is SF9, SF10, SF11 or SF12, then the command WriteRegister( 0x925, 0x32 ) must be used
	if(flag3)
	{
		buf[0] = 0x32;	
		WriteRegister_16bAddress(0x925, buf, 1);
	}
}





//********************************************************************************************
uint8_t SelectRegulator(uint8_t mode)//0-LDO 1-CD-CD
{
	uint8_t status;
  uint8_t buf[10];

	//                 1 DC-DC mode - consumes less power
	buf[0] = mode;   //0 LDO mode - consumes more power
	status = WriteCommand2( RADIO_SET_REGULATORMODE, buf, 1 );

	status = ReadCommand2( RADIO_GET_STATUS, buf, 1 );//0xC0
	
	return status;
}









//********************************************************************************************
void SetRx()//enter receive mode
{
	uint8_t buf[10];

	timeout.Step = RADIO_TICK_SIZE_0015_US; //15.625uS
	timeout.NbSteps = 0xFFFF;
	buf[0] = timeout.Step;
	buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
	buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

	
	ClearIrqStatus( IRQ_RADIO_ALL );
	WriteCommand2( RADIO_SET_RX, buf, 3 );//0x83
}




//********************************************************************************************
void SetTx()
{
	uint8_t buf[10];
	
	timeout.Step = RADIO_TICK_SIZE_0015_US; //15.625uS
	timeout.NbSteps = 0xFFFF;
	buf[0] = timeout.Step;
	buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
	buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

	ClearIrqStatus( IRQ_RADIO_ALL );
	WriteCommand2( RADIO_SET_TX, buf, 3 );//0x83
}




//****************************************************************************************
void ClearIrqStatus( uint16_t irq )
{
	uint8_t buf[10];

	buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
	buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
	WriteCommand2( RADIO_CLR_IRQSTATUS, buf, 2 );
}



//******************************************************************************
void SetRfFrequency( uint32_t frequency )
{
	uint8_t buf[10];
	uint32_t freq = 0;

	freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
	buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
	buf[1] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
	buf[2] = ( uint8_t )( freq & 0xFF );
	WriteCommand2( RADIO_SET_RFFREQUENCY, buf, 3 );
}



//***************************************************************************
uint8_t WriteCommand2( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint8_t status;
	
	WaitOnBusy( );

	RadioNss = 0;
	RadioSpi.write( ( uint8_t )command );
	for( uint16_t i = 0; i < size; i++ )
			status = RadioSpi.write( buffer[i] );
	RadioNss = 1;

	if( command != RADIO_SET_SLEEP )
			WaitOnBusy( );
	
	return status;
}


//***************************************************************************
uint8_t ReadCommand2( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint8_t status;

	WaitOnBusy( );

	RadioNss = 0;
	RadioSpi.write( ( uint8_t )command );
	status = RadioSpi.write( 0 );
	for( uint16_t i = 0; i < size; i++ )
			 buffer[i] = RadioSpi.write( 0 );
	RadioNss = 1;

	WaitOnBusy( );

	return status;
}

//***************************************************************************
uint8_t WriteRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint8_t status;

	WaitOnBusy( );

	RadioNss = 0;
	RadioSpi.write( RADIO_WRITE_REGISTER );//0x18 command for writing a block of bytes in a data memory space
	RadioSpi.write( ( address & 0xFF00 ) >> 8 );
	RadioSpi.write( address & 0x00FF );
	for( uint16_t i = 0; i < size; i++ )
	{
			status = RadioSpi.write( buffer[i] );
	}
	RadioNss = 1;

	if( address != RADIO_SET_SLEEP )
			WaitOnBusy( );
	
	return status;
}

//***************************************************************************
uint8_t ReadRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint8_t status;

	WaitOnBusy( );

	RadioNss = 0;
	RadioSpi.write( RADIO_READ_REGISTER );// send 0x19
	RadioSpi.write( ( address & 0xFF00 ) >> 8 );//send 00 C0 
	RadioSpi.write( address & 0x00FF );
	status = RadioSpi.write( 0 ); // send 00
	for( uint16_t i = 0; i < size; i++ )
	{
			buffer[i] = RadioSpi.write( 0 );
	}
	RadioNss = 1;

	WaitOnBusy( );
	
	return status;
}

//**********************************************************************************************
uint8_t WriteTXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size )
{
	uint8_t status;
	printf("Waiting busy...\r\n");
	WaitOnBusy( );
	// while (bsy.read() == 1) {};

	RadioNss = 0;
	printf("Writing ...\r\n");

	RadioSpi.write( RADIO_WRITE_BUFFER );//0x1A write the data payload to be transmitted
	printf("Writing done ...\r\n");


	status = RadioSpi.write( offset );
	for( uint16_t i = 0; i < size; i++ )
	{
			RadioSpi.write( buffer[i] );
	}
	RadioNss = 1;

	return status;
}

//**********************************************************************************************
uint8_t ReadRXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size )
{
	uint8_t status;
	
	WaitOnBusy( );

	RadioNss = 0;
	RadioSpi.write( RADIO_READ_BUFFER );//0x1B allows reading (n-3) bytes of payload received starting at offset
	RadioSpi.write( offset );
	status = RadioSpi.write( 0 );//write NOP, read status
	for( uint16_t i = 0; i < size; i++ )
	{
			 buffer[i] = RadioSpi.write( 0 );
	}
	RadioNss = 1;

	WaitOnBusy( );
				
	return status;
}




//**************************************************************************************************
void Wakeup( void )
{
	__disable_irq( );
	//Don't wait for BUSY here
	RadioNss = 0;
	wait_us(1000 *2);//wait in case SX1280 was sleeping
	RadioSpi.write( RADIO_GET_STATUS );
	RadioSpi.write( 0 );
	RadioNss = 1;
	// Wait for chip to be ready.
	WaitOnBusy( );
	__enable_irq( );
}


//**********************************************************************************************
void SpiInit( void )
{
	RadioNss = 1;
	RadioSpi.format( 8, 0 );
	RadioSpi.frequency( 8000000 );
	wait_us(1000 *100);
}

//**********************************************************************************************
void MyRadioReset( void )
{
	__disable_irq( );
	printf("IRQs disabled\r\n");
	wait_us( 20000 );
	printf("Sleeping\r\n");

	RadioReset.output( );
	printf("output \r\n");

	RadioReset = 0;
	wait_us(1000 * 50 );
	RadioReset = 1;
	RadioReset.input( ); // Using the internal pull-up
	printf("input\r\n");

	wait_us(1000 * 20 );
	__enable_irq( );
}


//Write 32-bit data to actual EEPROM memory
//**********************************************************************************
void EepromWrite( uint32_t addr, uint32_t data)
{
	//Unlock the data EEPROM and FLASH_PECR register
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	if((FLASH->PECR & FLASH_PECR_PELOCK) != 0)
	{
		FLASH->PEKEYR = FLASH_PEKEY1;
		FLASH->PEKEYR = FLASH_PEKEY2;
	}

	EEPROM_Erase(addr);
	
	//Write to data EEPROM
	//*(uint8_t *)(DATA_E2_ADDR+i) = DATA_BYTE;
	//*(uint16_t *)(DATA_E2_ADDR+j) = DATA_16B_WORD;
	*(uint32_t *)(addr) = data;
	//DATA_E2_ADDR is an aligned address in the data EEPROM area.
	//i can be any integer.
	//j must be an even integer.
	
	//Lock data EEPROM and FLASH_PECR register
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	FLASH->PECR |= FLASH_PECR_PELOCK;
}




//Erase EEPROM region of memory
//****************************************************************************************************
void EEPROM_Erase(uint32_t addr)
{
	FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_DATA;//enable page erasing
	*(__IO uint32_t *)addr = (uint32_t)0;//Write a 32-bit word to start erase
	__WFI();//wait for interrupt
	FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_DATA);//disable the page erase
}

