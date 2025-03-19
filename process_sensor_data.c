#include "includes.h"


uint8_t I2C_tx_data[2],I2C_rx_data[14];


struct nand_driver_data nand0;
volatile uint8_t WriteBuffer_GPS[PAGE_OOB_SIZE]; //2048+64

volatile uint8_t WriteBuffer_IMU[PAGE_OOB_SIZE]; //2048+64 = 2112
//volatile uint8_t TempGPSBuffer[64];
volatile uint8_t TempIMUBuffer[64];
volatile uint8_t ReadBuffer[PAGE_OOB_SIZE];
//volatile uint8_t USB_PrintBuffer[64];

volatile uint8_t BadBlockBackup[NBR_BLOCK];

//uint32_t DataCountIMU=0,DataCountGPS=0;

uint16_t block_index_gps,page_index_gps, now_page_index_gps;
uint16_t block_index_imu,page_index_imu, now_page_index_imu;
volatile uint32_t inqueue_index_gps=0,inqueue_index_imu=0;
int32_t retv;
uint8_t imuOverflow;


uint32_t bad_table_offset=0;		// every 1024 byte 저장,
uint32_t blockIndex_Badblock = 0;	// stable blcok is 0 block

//RTC_TimeTypeDef sTime;
//RTC_DateTypeDef sDate;

uint16_t batteryVal[1]={0x00};


uint16_t start_gps_block_index=0;
uint16_t start_gps_page_index=0;
uint16_t start_imu_block_index=0;
uint16_t start_imu_page_index=0;


uint8_t stage =0;
uint32_t nowTimeTick=0;
uint32_t imuTimeTick=0;
uint32_t secTimeTick=0;
uint32_t millTimeTick=0;
uint8_t btnPressTick;

uint8_t runImuFrequency;
extern uint8_t DockMode;
uint32_t command_menu_tick;
uint32_t command_menu_sec;


static uint8_t ledCounter= 0;
extern uint16_t gpioIntCnt;

volatile uint8_t millCnt=0;
volatile uint8_t preSec=0;



uint8_t gpsHour;
uint8_t gpsMin;
uint8_t gpsSec;
uint8_t gpsMili;
uint8_t operationMode = 1;

volatile uint8_t imuMiliCnt = 1;
volatile uint16_t motion_detect_counter = 0;




uint8_t gpsMessageBuff[90];
uint8_t messageLength = 0;

#define	KEYBOARD_CarryRet	0x0D
#define	KEYBOARD_LineFeed	0x0A
uint8_t startSaveBuffer = 0, powerOffWork = 0, currImuSec=0;


uint8_t nowSec, previousSec;
uint8_t FirstgpsMsgOk = 0;


uint16_t gpsError_nandPage = 0, imuError_nandPage = 0;


static uint8_t saved_sizeInformation(void);
static void WriteIMUToNandFlash(void);




float batteryConstChkBuf[20] = {4.2,4.2,4.2,4.2,4.2,
								4.2,4.2,4.2,4.2,4.2,
								4.2,4.2,4.2,4.2,4.2,
								4.2,4.2,4.2,4.2,4.2};

uint16_t current_voltage = 0;
static uint8_t low_batt_counter = 0;

uint16_t  ten_minute_led_counter = 60*3; //60*10;
uint8_t gps_data_overflow, gps_overflow_buffer[128];

//extern uint8_t UartReady;
static uint8_t wifi_send_data[22];
//static uint16_t esp32_status_tick=0;

static uint8_t motion_value_from_imu = 0x40;
static uint8_t wifi_send_batt_info;
static uint16_t average_gps_speed;

static uint32_t total_gps_speed;


#if (FEATURE_SAVE_HR_DEAT)
volatile uint8_t write_hr_data[PAGE_OOB_SIZE];
volatile uint32_t inqueue_index_hr=0;
static uint8_t hr_first_done = 0;
static uint8_t hr_data_overflow; // hr_overflow_buffer[128];
static uint8_t full_nand_flash = 0;

uint16_t block_index_hr,page_index_hr, now_page_index_hr;
uint16_t start_hr_block_index=0;
uint16_t start_hr_page_index=0;

static void WriteHeartRateToNandFlash(void);
#endif

static uint8_t run_imu_flag = 0;
static uint16_t start_imu_year = 0;
static uint8_t start_imu_month = 0;
static uint8_t start_imu_date = 0;
static uint8_t start_imu_hour = 0;
static uint8_t start_imu_minute = 0;




uint8_t digit_buff[5];
uint32_t send_data_counter = 0;
// 12*2 =24 in 1 second
static void make_esp32_transmit_data_no_coordinator(void)
{

	wifi_send_data[0] = '$'; 		// 0x24
	wifi_send_data[1] = myGPSmessage.utc.hour;
	wifi_send_data[2] = myGPSmessage.utc.min;
	wifi_send_data[3] = myGPSmessage.utc.sec;
	wifi_send_data[4] = myGPSmessage.utc.milliSec;
	wifi_send_data[5] = average_gps_speed;
	wifi_send_data[6] = average_gps_speed>>8;
	wifi_send_data[7] = 's'; //0x73
	wifi_send_data[8] = motion_value_from_imu;
	wifi_send_data[9] =  wifi_send_batt_info;
	wifi_send_data[10] = userData.deviceSerialId>>8;
	wifi_send_data[11] = userData.deviceSerialId;
	HAL_UART_Transmit(&huart1, (uint8_t*)&wifi_send_data[0],12,100);
}
static void make_esp32_transmit_data(void)
{

	wifi_send_data[0] = '$'; 		// 0x24
	wifi_send_data[1] = myGPSmessage.utc.hour;
	wifi_send_data[2] = myGPSmessage.utc.min;
	wifi_send_data[3] = myGPSmessage.utc.sec;
	wifi_send_data[4] = myGPSmessage.utc.milliSec;
	wifi_send_data[5] = myGPSmessage.latitude; // LSB
	wifi_send_data[6] = myGPSmessage.latitude>>8;
	wifi_send_data[7] = myGPSmessage.latitude>>16;
	wifi_send_data[8] = myGPSmessage.latitude>>24;
	wifi_send_data[9] = myGPSmessage.nshemi; // N/S
	wifi_send_data[10] = myGPSmessage.longitude; // LSB
	wifi_send_data[11] = myGPSmessage.longitude>>8;
	wifi_send_data[12] = myGPSmessage.longitude>>16;
	wifi_send_data[13] = myGPSmessage.longitude>>24;
	wifi_send_data[14] = myGPSmessage.ewhemi;
	wifi_send_data[15] = average_gps_speed;
	wifi_send_data[16] = average_gps_speed>>8;
	wifi_send_data[17] = 's'; //0x53
	wifi_send_data[18] = motion_value_from_imu;
	wifi_send_data[19] = wifi_send_batt_info;
	wifi_send_data[20] = userData.deviceSerialId>>8;
	wifi_send_data[21] = userData.deviceSerialId;

	HAL_UART_Transmit(&huart1, (uint8_t*)&wifi_send_data[0],22,100);
}


void userImuCalibration(uint8_t timeout)
{
	uint32_t timeTick = 0;
	uint8_t b;
	uint8_t i2cRxData[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t i2cTxData[2];
	uint8_t magData[6] = {0,0,0,0,0,0};
	int readCounter = 0;

	i2cTxData[0] = 59; //register xyz
	i2cTxData[1] = 0;

    //USB_Printf("userImuCalibration\r\n");

	WriteBuffer_IMU[inqueue_index_imu++] = 0x0D;
	WriteBuffer_IMU[inqueue_index_imu++] = 0x0A;
	WriteBuffer_IMU[inqueue_index_imu++] = 'B';
	WriteBuffer_IMU[inqueue_index_imu++] = 'I';
	WriteBuffer_IMU[inqueue_index_imu++] = 'A';
	WriteBuffer_IMU[inqueue_index_imu++] = 'S';

	timeTick = HAL_GetTick();	
	while (readCounter < 1000)
	{
		//led blinking
		if(clock_time_exceed(timeTick, 1000))
		{
			//power led on/off : gps scan
			//HAL_GPIO_TogglePin(GPIOB, POWER_ON_LED_Pin);
			HAL_GPIO_TogglePin(GPIOB, READY_LED_Pin);
			timeTick = HAL_GetTick();
		}

		if(inqueue_index_imu >= 2048)
		{
			imuOverflow = inqueue_index_imu - 2048;

			for(b=0;  b<imuOverflow; b++ )
			{
				TempIMUBuffer[b] = WriteBuffer_IMU[2048 + b];
			}

			inqueue_index_imu = 0;
			WriteIMUToNandFlash();

			for(b=0; b<imuOverflow; b++ )
			{
				WriteBuffer_IMU[b]=TempIMUBuffer[b];
			}
			inqueue_index_imu += imuOverflow;

		}
		//Read IMU Sensor
		I2C_tx_data[0] = 59; //register xyz
		I2C_tx_data[1] = 0;
		HAL_I2C_Master_Transmit(&hi2c2, MPU9250_ADDRESS, i2cTxData, 1, 50);
		HAL_I2C_Master_Receive(&hi2c2, MPU9250_ADDRESS, i2cRxData, 14, 50);

		//acc
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[0]; //accx
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[1];
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[2]; //accy
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[3];
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[4]; //accz
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[5];

		//gyro
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[8]; //gyrox
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[9];
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[10]; //gyroy
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[11];
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[12]; // gyroz
		WriteBuffer_IMU[inqueue_index_imu++] = i2cRxData[13];


		readMagData8bit(&magData[0]);
		WriteBuffer_IMU[inqueue_index_imu++] = magData[0];
		WriteBuffer_IMU[inqueue_index_imu++] = magData[1];
		WriteBuffer_IMU[inqueue_index_imu++] = magData[2];
		WriteBuffer_IMU[inqueue_index_imu++] = magData[3];
		WriteBuffer_IMU[inqueue_index_imu++] = magData[4];
		WriteBuffer_IMU[inqueue_index_imu++] = magData[5];

		HAL_Delay(10);
		readCounter++;

	}

	WriteBuffer_IMU[inqueue_index_imu++] = 'E';
	WriteBuffer_IMU[inqueue_index_imu++] = 'N';
	WriteBuffer_IMU[inqueue_index_imu++] = 'D';
	WriteBuffer_IMU[inqueue_index_imu++] = 0x0D;
	WriteBuffer_IMU[inqueue_index_imu++] = 0x0A;

	// variable initialization 
	imuOverflow = 0;
	for(int j=0; j<64; j++)
		TempIMUBuffer[j] = 0x00;


}

void alram_LedIndication(uint8_t timeout)
{
	uint32_t tick;
	tick = HAL_GetTick();
	 while (clock_time_exceed(tick, timeout * 1000) == 0)  // 1 min
	 {
		 HAL_GPIO_TogglePin(GPIOB, POWER_ON_LED_Pin|READY_LED_Pin|BATT_WARNING_LED_Pin|RUNING_LED_Pin);
		 HAL_Delay(500);
	 }
}

void getGpsMessage(uint8_t data)
{
	static uint32_t gsv_index=0;
	static uint32_t rcv_gps_counter = 0;
	static uint8_t rmc_first_done = 0;

	gpsMessageBuff[messageLength] = data;
	messageLength++;

	if (data == KEYBOARD_LineFeed) // ||data == KEYBOARD_LineFeed)
	{
		//display GPS message
		if (NMEA_GPRMC_Analysis(&myGPSmessage, &gpsMessageBuff[0]))
		{
			if(FirstgpsMsgOk == 0)
			{
				FirstgpsMsgOk = 1;
				
				WriteBuffer_GPS[0] = 's';
				WriteBuffer_GPS[1] = 't';
				WriteBuffer_GPS[2] = 'a';
				WriteBuffer_GPS[3] = 'r';
				WriteBuffer_GPS[4] = 't';
				inqueue_index_gps = 5;
			}
			gpsHour = myGPSmessage.utc.hour;
			gpsMin = myGPSmessage.utc.min;
			gpsSec = myGPSmessage.utc.sec;
			
			total_gps_speed += myGPSmessage.speed;
			if(use_esp32_power_on && rcv_gps_counter %5 == 0)
			{
				average_gps_speed = (total_gps_speed/5);
				total_gps_speed = 0;

				if(use_gps_coordinator)
				{
					make_esp32_transmit_data();

				}else
				{
					make_esp32_transmit_data_no_coordinator();
				}
			}
			rcv_gps_counter++;
#if (FEATURE_SAVE_HR_DEAT)
			if(rcv_hr_counter>= 60) 
			{
				if(hr_first_done == 0)
				{
					if((myGPSmessage.utc.year != 0x00) && (myGPSmessage.utc.month != 0x00) &&(myGPSmessage.utc.date != 0x00))
					{
						hr_first_done = 1;
						write_hr_data[0] = 's';
						write_hr_data[1] = 't';
						write_hr_data[2] = 'a';
						write_hr_data[3] = 'r';
						write_hr_data[4] = 't';
						write_hr_data[5] =  (uint8_t)(myGPSmessage.utc.year - 2000);
						write_hr_data[6] =  myGPSmessage.utc.month;
						write_hr_data[7] =  myGPSmessage.utc.date;
						write_hr_data[8] =  myGPSmessage.utc.hour;
						write_hr_data[9] =  myGPSmessage.utc.min;
						inqueue_index_hr = 10;
					}
				}
				if ((previousSec != myGPSmessage.utc.sec) && hr_first_done)
				{
					write_hr_data[inqueue_index_hr++] = myGPSmessage.utc.sec;
					write_hr_data[inqueue_index_hr++] = heart_rate_bpm;
				}
			}
#endif
			if(rmc_first_done == 0)
			{
				if((myGPSmessage.utc.year != 0x00) && (myGPSmessage.utc.month != 0x00) &&(myGPSmessage.utc.date != 0x00))
				{
					rmc_first_done = 1;
					run_imu_flag = 1;
					start_imu_year = myGPSmessage.utc.year; 	start_imu_month = myGPSmessage.utc.month;
					start_imu_date = myGPSmessage.utc.date;	    start_imu_hour =  myGPSmessage.utc.hour;
					start_imu_minute = myGPSmessage.utc.min;
				}
			}
			if((runImuFrequency == 10) && (run_imu_flag == 1)) // IMU:10hz
			{
				imuMiliCnt = myGPSmessage.utc.milliSec; // 1 ~ 9 
				InqueueIMU_RTC();
			}
			else if((runImuFrequency == 100) && (run_imu_flag == 1)) // IMU:100hz
			{
				if (previousSec != myGPSmessage.utc.sec)
				{
					imuMiliCnt = 0;
					HAL_TIM_Base_Start_IT(&htim7);
				}
				previousSec = gpsSec;
			}
			for(int j=0; j<messageLength; j++)
			{
				WriteBuffer_GPS[inqueue_index_gps++] = gpsMessageBuff[j];
			}
		}
		//gps SNR
		NMEA_GPGSV_Analysis(&myGPSmessage, &gpsMessageBuff[0]);
		if(gps_snr_value > 0)
		{
			itoa(gps_snr_value, gps_snr_string, 10);
		}
		//glonass SNR
		if(gnssModeSelect != ONLY_GPS_MODE)
		{
			NMEA_GLGSV_Analysis(&myGPSmessage, &gpsMessageBuff[0]);
			if(glonass_snr_value > 0)
			{
				itoa(glonass_snr_value, glonass_snr_string, 10);
			}
		}
		// use GSA message. save dop value
		if(NMEA_GPGSA_Analysis(&myGPSmessage, &gpsMessageBuff[0]))
		{
			 if(gsv_index %2 == 0)
			{
				itoa(pdop_value, pdop_string, 10);
				itoa(hdop_value, hdop_string, 10);
				itoa(vdop_value, vdop_string, 10);
				// 22byte.
				WriteBuffer_GPS[inqueue_index_gps++] = '$';
				WriteBuffer_GPS[inqueue_index_gps++] = pdop_string[0];
				WriteBuffer_GPS[inqueue_index_gps++] = pdop_string[1];
				WriteBuffer_GPS[inqueue_index_gps++] = pdop_string[2];
				WriteBuffer_GPS[inqueue_index_gps++] = ',';
				WriteBuffer_GPS[inqueue_index_gps++] = hdop_string[0];
				WriteBuffer_GPS[inqueue_index_gps++] = hdop_string[1];
				WriteBuffer_GPS[inqueue_index_gps++] = hdop_string[2];
				WriteBuffer_GPS[inqueue_index_gps++] = ',';
				WriteBuffer_GPS[inqueue_index_gps++] = vdop_string[0];
				WriteBuffer_GPS[inqueue_index_gps++] = vdop_string[1];
				WriteBuffer_GPS[inqueue_index_gps++] = vdop_string[2];

				WriteBuffer_GPS[inqueue_index_gps++] = ',';
				WriteBuffer_GPS[inqueue_index_gps++] = 'G';
				WriteBuffer_GPS[inqueue_index_gps++] = gps_snr_string[0];
				WriteBuffer_GPS[inqueue_index_gps++] = gps_snr_string[1];
				
				if(gnssModeSelect != ONLY_GPS_MODE)
				{
					WriteBuffer_GPS[inqueue_index_gps++] = ',';
					WriteBuffer_GPS[inqueue_index_gps++] = 'L';
					WriteBuffer_GPS[inqueue_index_gps++] = glonass_snr_string[0];
					WriteBuffer_GPS[inqueue_index_gps++] = glonass_snr_string[1];
				}

				WriteBuffer_GPS[inqueue_index_gps++] = '\r';
				WriteBuffer_GPS[inqueue_index_gps++] = '\n';
				//USB_Printf("index %d\r\n", inqueue_index_gps);
			}
			 
			if(gnssModeSelect != ONLY_GPS_MODE)
			{
				gsv_index++;
			}
		}

		// initialize buffer, length
		for (uint8_t i = 0; i < sizeof(gpsMessageBuff); i++)
		{
			gpsMessageBuff[i] = 0;
		}
		messageLength = 0;
		startSaveBuffer = 0;
	}
}



uint32_t  clock_time_exceed(uint32_t ref, uint32_t span_ms){
	return ((uint32_t)(HAL_GetTick()- ref) > span_ms );
}

#ifdef DEBUG
volatile uint8_t DebugBuffer[64];
#endif


static uint8_t checkReadNand_BlockAndPage(const uint32_t block,const uint32_t page);

uint8_t magneticData[6];  // change 8bit  // Stores the 16-bit signed magnetometer sensor output
char tempbuff[95];
void InqueueIMU_RTC(void)
{
	static uint8_t firstdone = 1;
	uint8_t len=0;


	if(inqueue_index_imu > PAGE_OOB_SIZE)
	{
		inqueue_index_imu = 0;
	}
	
	//Read IMU Sensor
	I2C_tx_data[0] = 59; //register xyz
	I2C_tx_data[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c2, MPU9250_ADDRESS, I2C_tx_data, 1, 50);
	HAL_I2C_Master_Receive(&hi2c2, MPU9250_ADDRESS, I2C_rx_data, 14, 50);
	if (firstdone)
	{
		// add start string
		WriteBuffer_IMU[inqueue_index_imu++] = '\r';
		WriteBuffer_IMU[inqueue_index_imu++] = '\n';

		WriteBuffer_IMU[inqueue_index_imu++] = 's';
		WriteBuffer_IMU[inqueue_index_imu++] = 't';
		WriteBuffer_IMU[inqueue_index_imu++] = 'a';
		WriteBuffer_IMU[inqueue_index_imu++] = 'r';
		WriteBuffer_IMU[inqueue_index_imu++] = 't';		
		WriteBuffer_IMU[inqueue_index_imu++] = '2';
		WriteBuffer_IMU[inqueue_index_imu++] = '0';
		WriteBuffer_IMU[inqueue_index_imu++] =  (uint8_t)(start_imu_year - 2000);//(uint8_t) (myGPSmessage.utc.year  - 2000); //year
		WriteBuffer_IMU[inqueue_index_imu++] =  start_imu_month; //myGPSmessage.utc.month;	//month
		WriteBuffer_IMU[inqueue_index_imu++] = start_imu_date;   // myGPSmessage.utc.date;	// date
		WriteBuffer_IMU[inqueue_index_imu++] = start_imu_hour;   // gpsHour;				// hour
		WriteBuffer_IMU[inqueue_index_imu++] = start_imu_minute; // gpsMin;					// minute

		firstdone = 0;
	}
	
	WriteBuffer_IMU[inqueue_index_imu++] = gpsSec; //myGPSmessage.utc.sec;
	WriteBuffer_IMU[inqueue_index_imu++] = imuMiliCnt; //gpsMili+imuMiliCnt;

	//acc
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[0]; //accx
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[1];
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[2]; //accy
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[3];
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[4]; //accz
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[5];

	//gyro
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[8]; //gyrox
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[9];
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[10]; //gyroy
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[11];
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[12]; // gyroz
	WriteBuffer_IMU[inqueue_index_imu++] = I2C_rx_data[13];



	readMagData8bit(&magneticData[0]);

	WriteBuffer_IMU[inqueue_index_imu++] = magneticData[0];
	WriteBuffer_IMU[inqueue_index_imu++] = magneticData[1];
	WriteBuffer_IMU[inqueue_index_imu++] = magneticData[2];
	WriteBuffer_IMU[inqueue_index_imu++] = magneticData[3];
	WriteBuffer_IMU[inqueue_index_imu++] = magneticData[4];
	WriteBuffer_IMU[inqueue_index_imu++] = magneticData[5];

	// IMU:100hz
	if(runImuFrequency == 100)
	{
		imuMiliCnt++;
		if (imuMiliCnt > 99)
		{
			motion_value_from_imu = checkWakeOnMotion(); //check motion by imu
			imuMiliCnt = 0;
			HAL_TIM_Base_Stop_IT(&htim7);
		}
	}}

static void WriteIMUToNandFlash(void)
{
	int32_t status = EIO;
	uint8_t retryWrite, goodblock;
	uint16_t i;

	goodblock = 1;

	if(operationMode == 0)
	{
		WriteBuffer_IMU[2000] = 'P';
		WriteBuffer_IMU[2001] = 'W';
		WriteBuffer_IMU[2002] = 'R';
		WriteBuffer_IMU[2003] = 'O';
		WriteBuffer_IMU[2004] = 'F';
	}


	for(retryWrite=0; retryWrite<3; retryWrite++)
	{
		nand_gpio_write_page_ecc(&nand0, block_index_imu, page_index_imu, WriteBuffer_IMU);
		status = nand_gpio_read_page_ecc_retry(&nand0 ,block_index_imu, page_index_imu, ReadBuffer);
		if(status < 0)
		{
			userData.imuReadError++;
			block_index_imu++;
			page_index_imu = 0;
			goodblock = 0;
		}
		//good
		else {
			//USB_Printf("imu good\r\n");
			goodblock = 1;
			break;
		}
	}

	//USB_Printf("Ok\r\n");
	now_page_index_imu = page_index_imu;
	
	if(operationMode==1 && goodblock == 1)
	{
		page_index_imu++;
	}

	if( page_index_imu >= nand0.info.pages_per_block )
	{
		page_index_imu = 0;
		block_index_imu++;
	}
	
	if(block_index_imu > IMU_BLOCK_END )
	{
		while(1)
		{
			eraseNandflashAndInternalflash();
			HAL_GPIO_WritePin(GPIOB, POWER_HOLD_Pin, GPIO_PIN_RESET);
		}
	}

}


#if (FEATURE_SAVE_HR_DEAT)
static void WriteHeartRateToNandFlash(void)
{
	int32_t status = EIO;
	uint8_t retryWrite, goodblock;
	uint16_t i;

	goodblock = 1;

//	USB_Printf("goodflag %d imu Enter block %d page %d\r\n", goodblock, block_index_imu,page_index_imu);
	/*
	if(block_index_imu%5 == 0)
	{
		USB_Printf("goodflag %d imu Enter block %d page %d\r\n", goodblock, block_index_imu,page_index_imu);
	}
	*/
	if(operationMode == 0)
	{
		WriteBuffer_IMU[2000] = 'P';
		WriteBuffer_IMU[2001] = 'W';
		WriteBuffer_IMU[2002] = 'R';
		WriteBuffer_IMU[2003] = 'O';
		WriteBuffer_IMU[2004] = 'F';
	}

	// flash read/write 시간 : 4.6msec 최대 bad block 20 이라면, 총 시간은 92msec.
	for(retryWrite=0; retryWrite<3; retryWrite++)	// 시간이 얼마나 걸릴까 ?
	{
		nand_gpio_write_page_ecc(&nand0, block_index_hr, page_index_hr, write_hr_data);
		// status > 0  성공 경우, read 체크 한다.
		//bad block 경우 read error 발생 함.
		status = nand_gpio_read_page_ecc_retry(&nand0 ,block_index_hr, page_index_hr, ReadBuffer);
			//USB_Printf("do read check\r\n");
		//read error 발생 :
		if(status < 0)
		{

			//USB_Printf("(hr bad): index %d block %d page %d\r\n", retryWrite, block_index_hr, page_index_hr);
			userData.imuReadError++;
			// bad block 처리 및 block_index_imu 증가,, Bad block 체크는 Erase Error 경우만 처리 한다.
			//bad_block_table[block_index_imu] = NAND_BLOCK_BAD;
			block_index_hr++;
			page_index_hr = 0;
			goodblock = 0;
		}
		//good
		else {
			//USB_Printf("(hr ok): index %d block %d page %d\r\n", retryWrite, block_index_hr, page_index_hr);
			goodblock = 1;
			userData.hr_save_data_cnt++;
			break;
		}
	}

	//USB_Printf("Ok\r\n");
	now_page_index_hr = page_index_hr;
	//v1.7: power off, no increase counter
	if(operationMode==1 && goodblock == 1)
	{
		page_index_hr++;
	}

	if( page_index_hr >= nand0.info.pages_per_block )
	{
		//USB_Printf("add: page:%d",page_index_imu);
		page_index_hr = 0;
		//increase block index hr
		block_index_hr++;
	}
	//if space is full, no save nand flash
	if(block_index_hr > HEART_RATE_BLOCK_END )
	{
		full_nand_flash = 1;
	}

}
#endif


uint8_t SendGPSDataToPC(void)
{
	uint32_t block,page;
	int32_t status =EIO;
	uint8_t msg[6]= {'G','P','S','E','N','D'};
	uint8_t errmsg[6]= {'G','P','S','E','R','R'};
	uint8_t idx=0;
	//uint16_t allGpssize=0;
	displayCounter = 0;
	uint16_t loop_cnt = 0;


	 if(userData.finish_gps_block_index >  GPS_BLOCK_END)
	 {
		 userData.finish_gps_block_index = GPS_BLOCK_END -1;
	 }
	 if(userData.finish_gps_page_index >  64)
	 {
		 userData.finish_gps_page_index  = 63;
	 }

	memset(&ReadBuffer[0], 0x00, PAGE_OOB_SIZE);

		for(block = GPS_BLOCK_START; block <= userData.finish_gps_block_index; block++ )
		{
			for(page=0; page < 64; page++)  //64
			{
				status = nand_gpio_read_page_ecc_retry(&nand0, block, page, &ReadBuffer[0]);
				if(status < 0)
				{

					while(CDC_Transmit_FS((uint8_t *)&errmsg[0],6) != USBD_OK )
					{
						HAL_Delay(1);
					}

					gpsError_nandPage++;
				}
				else
				{

					while(CDC_Transmit_FS((uint8_t *)&ReadBuffer[0],2048) != USBD_OK )
					{
						HAL_Delay(1);
					}

					if(loop_cnt %30 == 0)
					{
						 CommandMode_LedDisplay();
						 loop_cnt = 0;
					}
					loop_cnt++;
				}
				if(page >= userData.finish_gps_page_index && block >= userData.finish_gps_block_index)
				{

					for(idx =0; idx<3; idx++)
					{
						while(CDC_Transmit_FS((uint8_t *)&msg[0],6) != USBD_OK )
						{
							HAL_Delay(1);
						}
						HAL_Delay(10);
					}


					return 1;
				}
			} // page
			 // CommandMode_LedDisplay();
		} // block
	return 0;
}

uint8_t SendIMUDataToPC(void)
{
	uint32_t block,page;
	int32_t status =EIO;
	uint8_t msg[6]= {'I','M','U','E','N','D'};
	uint8_t errmsg[6]= {'I','M','U','E','R','R'};
	uint8_t idx=0;
	uint16_t loop_cnt=0;
	//uint16_t allImusize = 0;
	displayCounter = 0;

	//v1.7 lz4 compression
	// lz4CompressIint();



	if(userData.finish_imu_block_index >IMU_BLOCK_END)
	{
		userData.finish_imu_block_index = IMU_BLOCK_END -1;
	}
	if(userData.finish_imu_page_index >  64)
	{
		userData.finish_imu_page_index  = 63;
	}

	//allImusize = (userData.finish_imu_block_index - IMU_BLOCK_START)*64 + userData.finish_imu_page_index;
	//USB_Printf("allszie %d imu:block: %d page:%d\r\n", allImusize, userData.finish_imu_block_index,userData.finish_imu_page_index);
	memset(&ReadBuffer[0], 0x00, PAGE_OOB_SIZE);

		for(block = IMU_BLOCK_START; block <= userData.finish_imu_block_index; block++ )
		{
			for(page=0; page < 64; page++)
			{
				status = nand_gpio_read_page_ecc_retry(&nand0,block,page,&ReadBuffer[0]);
				if(status < 0)
				{

					while(CDC_Transmit_FS((uint8_t *)&errmsg[0],6) != USBD_OK )
					{
						HAL_Delay(1);
					}

					imuError_nandPage++;
				}
				else
				{
					while(CDC_Transmit_FS((uint8_t *)&ReadBuffer[0],2048) != USBD_OK )
					{
						HAL_Delay(1);
					}

					if(loop_cnt %30 == 0)
					{
					   CommandMode_LedDisplay();
					   loop_cnt = 0;
					}
					loop_cnt++;
				}

				if(page >= userData.finish_imu_page_index && block >= userData.finish_imu_block_index)
				{

					for(idx=0; idx<3; idx++)
					{
						while(CDC_Transmit_FS((uint8_t *)&msg[0],6) != USBD_OK )
						{
							HAL_Delay(1);
						}
						HAL_Delay(10);
					}
					HAL_GPIO_WritePin(GPIOB, POWER_ON_LED_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, RUNING_LED_Pin, GPIO_PIN_SET);

					return 1;
				}
			} // page
//			   CommandMode_LedDisplay();
		} // block
	return 0;
}
#if (FEATURE_SAVE_HR_DEAT)
uint8_t Send_HeartRate_DataToPC(void)
{
	uint32_t block,page;
	int32_t status =EIO;
	uint8_t msg[6]= {'H','R','D','E','N','D'};
	uint8_t errmsg[6]= {'H','R','D','E','R','R'};
	uint8_t idx=0;
	uint16_t loop_cnt=0;
	displayCounter = 0;
	if(userData.finish_hr_block_index >HEART_RATE_BLOCK_END)
	{
		userData.finish_hr_block_index = HEART_RATE_BLOCK_END -1;
	}
	if(userData.finish_hr_page_index >  64)
	{
		userData.finish_hr_page_index  = 63;
	}

	memset(&ReadBuffer[0], 0x00, PAGE_OOB_SIZE);

		for(block = HEART_RATE_BLOCK_START; block <= userData.finish_hr_block_index; block++ )
		{
			for(page=0; page < 64; page++)
			{
				status = nand_gpio_read_page_ecc_retry(&nand0,block,page,&ReadBuffer[0]);
				if(status < 0)
				{
					while(CDC_Transmit_FS((uint8_t *)&errmsg[0],6) != USBD_OK )
					{
						HAL_Delay(1);
					}
				}
				else
				{
					while(CDC_Transmit_FS((uint8_t *)&ReadBuffer[0],2048) != USBD_OK )
					{
						HAL_Delay(1);
					}

					if(loop_cnt %30 == 0)
					{
					   CommandMode_LedDisplay();
					   loop_cnt = 0;
					}
					loop_cnt++;
				}

				if(page >= userData.finish_hr_page_index && block >= userData.finish_hr_block_index)
				{

					for(idx=0; idx<3; idx++)
					{
						while(CDC_Transmit_FS((uint8_t *)&msg[0],6) != USBD_OK )
						{
							HAL_Delay(1);
						}
						HAL_Delay(10);
					}
					HAL_GPIO_WritePin(GPIOB, POWER_ON_LED_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, RUNING_LED_Pin, GPIO_PIN_SET);
					return 1;
				}
			} // page
		} // block
	return 0;
}
#endif








static uint8_t saved_sizeInformation(void)
{
	//v16: bad block.
	//int32_t status = EIO;
	//int32_t status2 = EIO;

	uint8_t idx;
	uint8_t sta=1;
//	uint32_t startGpsAllsize,finishGpsAllsize,startImuAllsize,finishImuAllsize;

//	 startGpsAllsize = startImuAllsize = startImuAllsize = finishImuAllsize = 0;
	 //gps data
	 userData.savedGpsCnt++;
	 userData.finish_gps_block_index = block_index_gps;
	 userData.finish_gps_page_index = now_page_index_gps;
	 // v1.7: gps data full indication
	 userData.gpsData_full_flag = 0x00;

	 //imu data
	 userData.savedImuCnt++;
	 userData.finish_imu_block_index = block_index_imu;
	 userData.finish_imu_page_index = now_page_index_imu;
	 // v1.7: imu data full indication
	 userData.imuData_full_flag = 0x00;

	 #if (FEATURE_SAVE_HR_DEAT)
	 	 userData.finish_hr_block_index = block_index_hr;
	 	 userData.finish_hr_page_index = now_page_index_hr;
	 #endif


	 for(idx = 0; idx<50; idx++)
	 {
		 sta = eraseInteralflash();
		 if(sta==0){
			 break;
		 }
		 HAL_Delay(10);
	 }
	 for(idx = 0; idx<50; idx++)
	 {
		 sta = writeInteralflash();
		 if(sta==0){
			 break;
		 }
		 HAL_Delay(10);
	 }

	 
	 if(sta==0)
	 {
		 HAL_GPIO_WritePin(GPIOB, POWER_HOLD_Pin, GPIO_PIN_RESET);
	}
}


void checkButtonPressed(void)
{
	static uint8_t firstdone =0 ;
	 if(HAL_GPIO_ReadPin(GPIOB, POWER_DETECT_Pin)==1)
	 {
		 // 2sec power off
		 if(btnPressTick > 1)
		 {
			 HAL_GPIO_WritePin(GPIOB, POWER_ON_LED_Pin, GPIO_PIN_RESET);
			prepare_PowerOff();
			nowTimeTick = HAL_GetTick();
			while(clock_time_exceed(nowTimeTick, 5*1000)==0);
			saved_sizeInformation();
		 }
	 }
	 else
	 {
		 btnPressTick = 0;
	 }
}

void gpsRun_LedIndication(void)
{
	if(clock_time_exceed(millTimeTick, 300) && ledCounter<4)
	{
		if(ledCounter%2==0){ HAL_GPIO_WritePin(GPIOB, RUNING_LED_Pin,GPIO_PIN_RESET);}
			else { HAL_GPIO_WritePin(GPIOB, RUNING_LED_Pin,GPIO_PIN_SET);}


		if(ten_minute_led_counter > 10)
		{
			if(ledCounter%2==0){ HAL_GPIO_WritePin(GPIOB, POWER_ON_LED_Pin,GPIO_PIN_RESET);}
			else { HAL_GPIO_WritePin(GPIOB, POWER_ON_LED_Pin,GPIO_PIN_SET);}
		}

		ledCounter++;
		millTimeTick = HAL_GetTick();
	}
}



void MainAlgorithm(void)
{
	uint8_t gpsData=0;
	uint8_t b=0;
	//float voltage = 4.2;
	//uint32_t tmpTimeTick=0;

	//v16: 버그 수정
	static uint8_t firstDoneSave =1;


	//inqueue_index_gps = 0;
	inqueue_index_imu = 0;
	imuOverflow =0;
	powerDetectItCnt = 0;

	//v16: bad block 처리
	getBadBlockTable();

	//X3 select gps, gps&glonass
	read_gnss_mode_select();
	
	//v1.7: main 함수에서 이동 함.
	ubloxSetup();
	initMPU9250();
	initAK8963(magCalibration);
	//checkNandFlash();
	readInteralflash();


	UART_CLEAR();
	//USB_Printf("gps start block:%d page:%d\r\n",block_index_gps, page_index_gps);
	//USB_Printf("imu start block:%d page:%d\r\n",block_index_imu, page_index_imu);
	nowTimeTick= HAL_GetTick();
	memset(&UART_Receive_data[0], 0x00, UART_BUFFER_SIZE);

	//cellX3 V1.0 : imu bias
	userImuCalibration(10);
	while(1)
	{
		// hang 관련 reset 코드
		gpioIntCnt = 0;
		if(uartDma_ready())
		{
			//WriteBuffer_GPS[inqueue_index_gps++] = uartDma_getData();
			//V1.7 gps 시간 정보 사용
			WriteBuffer_GPS[inqueue_index_gps] = uartDma_getData();
			gpsData = WriteBuffer_GPS[inqueue_index_gps];
			if(gpsData == '$')
			{
				startSaveBuffer = 1;
			}
			if(startSaveBuffer)
			{
				getGpsMessage(gpsData);
			}
		}
// save GPS data to NAND flash
		if(inqueue_index_gps >= nand0.info.page_size)
		{
			gps_data_overflow = inqueue_index_gps - nand0.info.page_size;
			for(int i=0; i<gps_data_overflow; i++)
			{
				gps_overflow_buffer[i] = WriteBuffer_GPS[nand0.info.page_size+i];
			}
			inqueue_index_gps =0;
			writeGpsDataToNandFlash();

			for(int i=0; i<gps_data_overflow; i++)
			{
				WriteBuffer_GPS[i] = gps_overflow_buffer[i];
			}
			inqueue_index_gps += gps_data_overflow;
		}

// save IMU data to NAND flash
		if(inqueue_index_imu >= nand0.info.page_size )
		{
			imuOverflow = inqueue_index_imu - nand0.info.page_size;
			//USB_Printf("inqueue_index_imu:%d over:%d\r\n",inqueue_index_imu,imuOverflow);
			for(b=0;  b<imuOverflow; b++ )
			{
				TempIMUBuffer[b] = WriteBuffer_IMU[nand0.info.page_size+b];
				//USB_Printf("%02x ",TempIMUBuffer[b]);
			}
			inqueue_index_imu = 0;
			WriteIMUToNandFlash();

			//USB_Printf("\r\n over \r\n");
			for(b=0; b<imuOverflow; b++ )
			{
				//WriteBuffer_IMU[inqueue_index_imu++]=TempIMUBuffer[b];
				WriteBuffer_IMU[b]=TempIMUBuffer[b];
				//USB_Printf("%02x ",WriteBuffer_IMU[inqueue_index_imu + b]);
			}

			inqueue_index_imu += imuOverflow;
		}
		// X4: save herat rate data
		#if (FEATURE_SAVE_HR_DEAT)
				if(full_nand_flash == 0)
				{
					if(inqueue_index_hr >= nand0.info.page_size)
					{
						hr_data_overflow = inqueue_index_hr - nand0.info.page_size;
						//USB_Printf("HR data over:%d\r\n",hr_data_overflow);

//						for(int i=0; i<hr_data_overflow; i++)
//						{
//							hr_overflow_buffer[i] = write_hr_data[nand0.info.page_size+i];
//						}
						inqueue_index_hr =0;
						WriteHeartRateToNandFlash();

//						for(int i=0; i<hr_data_overflow; i++)
//						{
//							write_hr_data[i] = hr_overflow_buffer[i];
//						}
//						inqueue_index_hr += hr_data_overflow;
					}
				}
		#endif
		// V1 : 실제 전압과 0.1V 차이남. 실제 3.3V 에서 cutoff 한다.
		if(clock_time_exceed(nowTimeTick, 1000)) // 1sec tick
		{
			if(use_esp32_power_on)
			{
				//X4 : setup RX UART interrupt for heart rate BPM
				if(has_ble_name)
				{
					HAL_UART_Receive_IT(&huart1, &esp32_uart_rcv[0], 1);
				}
				//USB_Printf("Heart Rate BPM %d\r\n", heart_rate_bpm);
			}
			Check_BatteryLevel(0);
			Establish_Battery_indicator(current_voltage);
			//LDO 3.0V(low voltage:3.2V)
			if(current_voltage <=320)
			{
				//v1.7: power off 이전에 gps, imu 데이터 저장 함.
				low_batt_counter++;
				if(low_batt_counter >= 10) // 10초 동안 동일 전압 측정.
				{
					// USB_Printf("low battery power off\r\n");
					prepare_PowerOff();
					saved_sizeInformation();
				}
			}
			//USB_Printf("val %d adc:%.2f avgVol:%.4f \r\n", voltage,batteryConstChkBuf[0],averagedVoltage);
			//USB_Printf("adc:%.2f avgVol:%.4f \r\n", batteryConstChkBuf[0],averagedVoltage);
			secTimeTick++;
			btnPressTick++;
			nowTimeTick = HAL_GetTick();

			if(ten_minute_led_counter > 1)
				ten_minute_led_counter--;
		}
		// led indication 3sec
		if(secTimeTick >=3)
		{
			millTimeTick = HAL_GetTick();
			secTimeTick = 0;
			ledCounter = 0;
		}
		gpsRun_LedIndication();
		checkButtonPressed();
		
		// 동작 모드 에서 도킹에 연결 될 경우 power off 코드 필요
		//v16:버그 3초 이내에 power off 버튼을 4번 연속 하면 off 된다.도킹에서 400msec 동안 high low 20번 전송 된다.
		if(clock_time_exceed(lastIttimeTick, 100))
		{
			powerDetectItCnt=0;
		}
		if(powerDetectItCnt>15)
		{
			//v16:버그수정 인터럽트로 여러번 호출 됨, 사이즈 정보 지워짐.
			if(firstDoneSave)
			{
				//v1.7: power off 이전에 gps, imu 데이터 저장 함.
				prepare_PowerOff();
				saved_sizeInformation();
				firstDoneSave = 0;
			}
		}
	}

}


extern uint8_t msgEnable;
void MainCommandMenu(void)
{
	uint8_t recv;
	uint32_t status;
	uint8_t tmp[8];
	uint8_t command;

	HAL_GPIO_WritePin(GPIOB, POWER_ON_LED_Pin, GPIO_PIN_RESET);

        command_menu_tick = HAL_GetTick();
	while(1)
	{
		//usb 통신 중에 usb 케이블 제거 할 경우 인터럽트에서 강제 power off 한다.
		gpioIntCnt = 0;

		//board_ledToggle(GREEN_LED);
		//HAL_Delay(100);
		if(clock_time_exceed(command_menu_tick, 1000)) // 1sec tick
		{
			command_menu_sec++;
			command_menu_tick = HAL_GetTick();
		}
		// docking 보드에서 low 신호 수신 시 power off 함.
		if(DockMode)
		{
			if(HAL_GPIO_ReadPin(GPIOB, POWER_DETECT_Pin)==0)
			{
				if(command_menu_sec > 3)
				{
					//USB_Printf("power off\r\n");
					HAL_GPIO_WritePin(GPIOB, POWER_HOLD_Pin, GPIO_PIN_RESET);
				}
			}else
			{
				command_menu_sec =0;
			}
		}
#if 1
		if(hostUartParser())
		{
			command = hostToMcu.messageCommand;
			switch(command)
			{
				case SYSCOMMAND_HW_INFORMATION:
					send_HW_information();
					break;

				case SYSCOMMAND_UPLOAD_ALL_GPS_AND_IMU_SIZE:
					send_AllGpsImuSize_Information();
					break;
				// no use
//				case SYSCOMMAND_UPLOAD_ALL_GAME_SIZE:
//					send_AllGameSize_Information();
//					break;

				case SYSCOMMAND_UPLOAD_GPS_START:
					send_uploadingGpsDataStart();
					break;

				case SYSCOMMAND_UPLOAD_IMU_START:
					send_uploadingImuDataStart();
					break;

				case SYSCOMMAND_ERASE_NAND_FLASH:
					set_EraseNand_Flash();
					break;

				case SYSCOMMAND_OLD_UPLOAD_GPS_DATA:
					old_uploadGPSDataToPC();
					break;
				case SYSCOMMAND_OLD_UPLOAD_IMU_DATA:
					old_uploadImuDataToPC();
					break;

				case SYSCOMMAND_SET_MCU_POWEROFF:
					set_McuPowerOff();
					break;
				case SYSCOMMAND_UPLOAD_SHORT_GPSDATA:
					SendgameShortGpsDataToPC();
					break;

				case SYSCOMMAND_REPORT_ERROR_INFO:
					reportErrorInfo();
					break;

				case SYSCOMMAND_SET_IMU_FREQUENCY:
					setup_imufrequency();
					break;
					
				case SYSCOMMAND_SET_IMU_TIMETICK_COUNUTER:
					imuTimeTickinfor();
					break;

				case SYSCOMMAND_SET_PRODUCTID:
					setup_productId();
					break;

				// 데이터 사이즈가 없는 경우 강제로 nand 끝까지 읽는다.
				// gps data
				case SYSCOMMAND_SET_FORCE_UPLOAD_GPS:
					send_Force_uploadingGpsData();
					break;
				// imu data
				case SYSCOMMAND_SET_FORCE_UPLOAD_IMU:
					send_Force_uploadingImuData();
					break;

				case SYSCOMMAND_SET_USB_DFU_MODE:
					enter_UsbDFUMode();
					break;
				case SYSCOMMAND_NANDFLASH_TEST:
					test_NandFlash();
					break;
				//no use
//				case SYSCOMMAND_NANDWRITE_TEST:
//					Test_NandFlashWrite();
//					break;

				//v16:bad block process
				case SYSCOMMAND_GET_BADBLOCK_NUMBER:
					get_badblockTable();
					break;
				case SYSCOMMAND_SET_BADBLOCK_ERASE:
					set_EraseBadblock();
					break;
				case SYSCOMMAND_SET_BADBLOCK_WRITE_READ:
					Test_BadBlockWriteAndRead();
					break;
				case SYSCOMMAND_PRINT_BADBLOCK_NUMBER:
					print_badblockTable();
					break;
#if 0
				case SYSCOMMAND_GET_BATTERY_VOLTAGE:
					get_BatteryRead();
					break;	
#endif
				//v1.8: imu bias: factory bias 생산 과정 
				case SYSCOMMAND_SET_WRITE_IMU_CAL:
					setup_imuCalibration();
					break;

				case SYSCOMMAND_SET_READ_IMU_CAL:
					read_imuCalibrationValue();
					break;
				// no save flash
				case SYSCOMMAND_SET_WRITE_IMU_CAL_MSG:
					msgEnable =1;
					user_Mpu9250_Calibration();
					break;

					// cellX3 V1.0:GPSERR, IMUERR 사용 PC 프로그램 전송
				case SYSCOMMAND_REPORT_GPS_IMU_ERR:
					send_GpsImuError_NandPage();
					break;

				case SYSCOMMAND_SET_MPU9250_SELF_TEST:
					run_mpu9250_SelfTest();
					break;
				//생산 테스트 진입 명령어 변경
				case SYSCOMMAND_SET_MESS_PRODUCTION:
					enter_production_mode();
					break;

				case SYSCOMMAND_SET_GNSS_MODE:
					setup_gnss_mode();
					break;
				//X4:write wifi configration
				case SYSCOMMAND_SET_WIFI_CONFIG:
					setup_wifi_configuration();
					break;

				case SYSCOMMAND_ERASE_WIFI_CONFIG_ESP32:
					erase_wifi_configuration_command();
					break;
				case SYSCOMMAND_ESP32_OTA_UPDATE:
					esp32_ota_update_command();
					break;

#if FEATURE_SAVE_HR_DEAT
				case SYSCOMMAND_UPLOAD_ALL_HR_DATA_SIZE:
					send_AllHeartRate_Data_Size_Information();
				break;

				case SYSCOMMAND_UPLOAD_HR_DATA:
					upload_heartRate_DataToPC();
				break;
#endif
				default:
				break;
			}
		}

#else
		recv= 'null';
		recv = VCP_getch();
		// 1. flash erase 
		if( recv == '1' )
		{
			extern void ClearBadBlockTable(void);
		//	ClearBadBlockTable();
			USB_Printf("\r\n Erase \r\n");
			for( int block=0; block < nand0.info.num_blocks ; block++ )
			{
				status = nand_gpio_erase(&nand0,block);
				if(status<0)
				{
					USB_Printf("error block %d\r\n",block);
				}
			}
			//ClearBadBlockTable();
			eraseInteralflash();
			USB_Printf("\r\n Erase Done!\r\n");

		}
		else if(recv == '2')
		{
			moduleTest_NandFlash();
		}
		else if(recv == '3')
		{
			moduleTest_GPS();
		}
		else if(recv == '4')
		{
			testIMUWhoamI();
			moduleTest_IMU();
		}
		else if(recv == '5')
		{
			moduleTest_RTC();
		}	
		else if(recv == '6')
		{
			//USB_Printf("\r\n Send GPS\r\n");
			gpsCompressflag =1;
			SendGPSDataToPC();
			//USB_Printf("\r\n Send GPS DONE\r\n");
		}
		else if(recv == '7')
		{
			//USB_Printf("\r\n Send GPS\r\n");
			SendIMUDataToPC();
			//USB_Printf("\r\n Send GPS DONE\r\n");
		}
		else if(recv == '8')
		{

		}
		else if(recv == 's')
		{
			USB_Printf("\r\n MCU power off\r\n");
			HAL_GPIO_WritePin(GPIOB, POWER_HOLD_Pin, GPIO_PIN_RESET);
			//continue;
		}
		else if(recv == 'h')
		{
			PrintMenu();
		}
		else if(recv == 'a')
		{
			//adcTest();
			moduleTest_NandFlashWrite();
		}
		else if(recv == 'b')
		{
			moduleTest_NandFlashRead2();
		}
		else if(recv == 'c')
		{
			SystemTimerTest();
		}
		else if(recv == 'd')
		{
			USB_Printf("Read Bad block\r\n");
			for(uint16_t idx=0; idx<1024; idx++)
			{
				USB_Printf("block %d val %d\r\n", idx,bad_block_table[idx]);
			}
		}
		else if(recv =='g')
		{
			GPIOH->ODR |= 0x00000001;
			HAL_Delay(100);
			GPIOH->ODR &= ~(0x00000001);
			HAL_Delay(100);

		}
		else if(recv == 'w')
		{
			testIMUWhoamI();
		}

		else if(recv == 'f')
		{
			internalflashtest();
		}
		else
		{

		}
#endif
	}

 }

