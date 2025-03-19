

#include "includes.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// GPS, IMU information
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_100   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_100 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */

// Device information 0x08032800
#define FLASH_USER_DI_START_ADDR   ADDR_FLASH_PAGE_101   /* Start @ of user Flash area */
#define FLASH_USER_DI_END_ADDR     ADDR_FLASH_PAGE_101 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */

// IMU frequency  0x08033000
#define FLASH_IMU_SETTING_START_ADDR   ADDR_FLASH_PAGE_102   /* Start @ of user Flash area */
#define FLASH_IMU_SETTING_END_ADDR     ADDR_FLASH_PAGE_102 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */

//v1.8: imu bias value 저장 0x08033800
#define FLASH_IMU_CAL_START_ADDR   ADDR_FLASH_PAGE_103   /* Start @ of user Flash area */
#define FLASH_IMU_CAL_END_ADDR     ADDR_FLASH_PAGE_103 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */

//X3: select gps, gps&glonass 0x08034000
#define FLASH_GNSS_MODE_SETTING_START_ADDR   ADDR_FLASH_PAGE_104   /* Start @ of user Flash area */
#define FLASH_GNSS_MODE_SETTING_END_ADDR     ADDR_FLASH_PAGE_104 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */


//X4 : wifi configuration 저장 0x08034800
#define FLASH_WIFI_CONFIG_START_ADDR   ADDR_FLASH_PAGE_105   /* Start @ of user Flash area */
#define FLASH_WIFI_CONFIG_END_ADDR     ADDR_FLASH_PAGE_105 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
/*Variable used for Erase procedure*/



static FLASH_EraseInitTypeDef EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
/* Private functions ---------------------------------------------------------*/

extern uint16_t block_index_gps,page_index_gps;
extern uint16_t block_index_imu,page_index_imu;
extern uint16_t start_gps_block_index;
extern uint16_t start_gps_page_index;
extern uint16_t start_imu_block_index;
extern uint16_t start_imu_page_index;

userflashDataTypeDef	userData;
imuConfigurationTypeDef		imuConfiguration;
//X4: WIFI CONFIG
wifi_config_flash_typedef	userWifiConfig;
static gnssModeSelectypeDef		gnss_mode_typedef;


// X4: save herat rate data
#if (FEATURE_SAVE_HR_DEAT)
extern uint16_t block_index_hr,page_index_hr;
extern uint16_t start_hr_block_index;
extern uint16_t start_hr_page_index;
#endif

uint8_t use_esp32_power_on = 1;
uint8_t use_gps_coordinator = 0;
uint8_t has_ble_name = 0;

void setup_default_value(void)
{
	// no save nand error, defualt setting
	if(userData.savedGpsCnt==0xffff && userData.savedImuCnt==0xffff)
	{
		userData.savedGpsCnt = userData.savedImuCnt=0;
		userData.gpsWriteError = userData.gpsReadError = 0;
		userData.imuWriteError = userData.imuReadError = 0;		
// game 1
//		userData.game1_all_gps_page_size = userData.game2_all_gps_page_size = 0;
//		userData.game3_all_gps_page_size = userData.game4_all_gps_page_size = 0;

		userData.game1_all_imu_page_size = userData.game2_all_imu_page_size = 0;
		userData.game3_all_imu_page_size = userData.game4_all_imu_page_size = 0;


#if 0
// game 2
		userData.game2_gps_block_size = userData.game2_gps_page_size = 0;
		userData.game2_imu_block_size = userData.game2_imu_page_size = 0;
// game 3
		userData.game3_gps_block_size = userData.game3_gps_page_size = 0;
		userData.game3_imu_block_size = userData.game3_imu_page_size = 0;
#endif
/*
		USB_Printf("game2:%d game1:%d\n\r",userData.game2_all_gps_page_size, userData.game1_all_gps_page_size);
		USB_Printf("game4:%d game3:%d\n\r",userData.game4_all_gps_page_size, userData.game3_all_gps_page_size);
		USB_Printf("imuReadError:%d imuWriteError:%d\n\r",userData.imuReadError, userData.imuWriteError);
		USB_Printf("gpsReadError:%d gpsWriteError:%d\n\r",userData.gpsReadError, userData.gpsWriteError);
*/
	}

}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}


/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}



uint8_t erase_ImuConfiguration(void)
{
	__disable_irq();

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_IMU_SETTING_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_IMU_SETTING_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_IMU_SETTING_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		you have to make sure that these data are rewritten before they are accessed during code
		execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		DCRST and ICRST bits in the FLASH_CR register. */
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	 {
	   /*
		 Error occurred while page erase.
		 User can add here some code to deal with this error.
		 PAGEError will contain the faulty page and then to know the code error on this page,
		 user can call function 'HAL_FLASH_GetError()'
	   */
		 //USB_Printf("Flash erase error\n\r");
		 return HAL_ERROR;
	 }
	 HAL_FLASH_Lock();
	__enable_irq();
	 return HAL_OK;
}


#define IMU_TIME_VAL		100
extern uint8_t runImuFrequency;

void read_ImuConfiguration(void)
{
	Address = FLASH_IMU_SETTING_START_ADDR;
	data32 = *(__IO uint32_t *)Address;

	//read imu frequency from flash
	imuConfiguration.run_frequency= data32;
    imuConfiguration.esp32_power_on = data32>>16;
	Address= Address+4;
	data32 = *(__IO uint32_t *)Address;
	imuConfiguration.gps_latitude = data32;

	//USB_Printf("read frequency %d power %d lati%d\r\n", imuConfiguration.run_frequency, imuConfiguration.esp32_power_on, imuConfiguration.gps_latitude);
	
	
	if(imuConfiguration.run_frequency != 0xffff)
	{
		if(imuConfiguration.run_frequency==10 || imuConfiguration.run_frequency==100)
		{
			runImuFrequency = imuConfiguration.run_frequency; 
	
		}
		else
		{
			// default value:
			runImuFrequency = IMU_TIME_VAL;
	
		}
	}
	// default value:
	else
		{
			runImuFrequency  = IMU_TIME_VAL;
	
		}

	
	if(imuConfiguration.esp32_power_on != 0xffff && imuConfiguration.gps_latitude != 0xffff)
	{

		use_esp32_power_on = imuConfiguration.esp32_power_on;
		use_gps_coordinator = imuConfiguration.gps_latitude;
	
	}
	// default value:
	else
	{
		use_esp32_power_on = 1;
		use_gps_coordinator = 0;
	
	}
}

uint8_t write_ImuConfiguration(uint8_t frequency, uint8_t esp32_pwr, uint8_t gps_coordi)
{
	uint64_t Data64bit;
	uint8_t status=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	//test data
	/*
	userData.savedGpsCnt = 0x01;
	userData.finish_gps_block_index = 0x02;
	userData.finish_gps_page_index = 0x03;
	userData.rev1 = 0x04;
	*/
	imuConfiguration.run_frequency = frequency;
	imuConfiguration.esp32_power_on = esp32_pwr;
	imuConfiguration.gps_latitude = gps_coordi;

	// make 64bit data
	Data64bit = imuConfiguration.rev1;
	Data64bit = imuConfiguration.gps_latitude|Data64bit<<16;
	Data64bit = imuConfiguration.esp32_power_on|Data64bit<<16;
	Data64bit = imuConfiguration.run_frequency|Data64bit<<16;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Address = FLASH_IMU_SETTING_START_ADDR;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("ok frequency %d,power %d, lat %d \n\r", frequency, esp32_pwr, gps_coordi);

	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return status;
}


uint8_t erase_gnss_mode_select(void)
{
	__disable_irq();

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_GNSS_MODE_SETTING_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_GNSS_MODE_SETTING_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_GNSS_MODE_SETTING_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		you have to make sure that these data are rewritten before they are accessed during code
		execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		DCRST and ICRST bits in the FLASH_CR register. */
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	 {
	   /*
		 Error occurred while page erase.
		 User can add here some code to deal with this error.
		 PAGEError will contain the faulty page and then to know the code error on this page,
		 user can call function 'HAL_FLASH_GetError()'
	   */
		 //USB_Printf("Flash erase error\n\r");
		 return HAL_ERROR;
	 }
	 HAL_FLASH_Lock();
	__enable_irq();
	 return HAL_OK;
}

void read_gnss_mode_select(void)
{
	Address = FLASH_GNSS_MODE_SETTING_START_ADDR;
	data32 = *(__IO uint32_t *)Address;

	//read imu frequency from flash
	gnss_mode_typedef.gnss_mode = data32;
	// flsh 메모리 저장된 값이 있으면, 그러나 그 값이 1,2 이면 저용 한다.
	if(gnss_mode_typedef.gnss_mode != 0xffff)
	{
		if( gnss_mode_typedef.gnss_mode ==1 || gnss_mode_typedef.gnss_mode==2)
		{
			gnssModeSelect  = gnss_mode_typedef.gnss_mode;
			//USB_Printf("gnss mode %d\r\n", gnssModeSelect);
		}
		else
		{
			// default value:
			gnssModeSelect = ONLY_GPS_MODE;	// only gps mode
			//USB_Printf("flash read is ONLY_GPS_MODE\r\n");
		}
	}
	// default value: // only gps mode
	else
		{
			gnssModeSelect  = ONLY_GPS_MODE;
			//USB_Printf("gnss mode is ONLY_GPS_MODE\r\n");
		}
	//USB_Printf("imu %d", runImuFrequency);
}

uint8_t write_gps_mode_select(uint8_t value)
{
	uint64_t Data64bit;
	uint8_t status=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	//test data
	/*
	userData.savedGpsCnt = 0x01;
	userData.finish_gps_block_index = 0x02;
	userData.finish_gps_page_index = 0x03;
	userData.rev1 = 0x04;
	*/
	gnss_mode_typedef.gnss_mode = value;

	// make 64bit data
	Data64bit = gnss_mode_typedef.rev1;
	Data64bit = gnss_mode_typedef.rev2|Data64bit<<16;
	Data64bit = gnss_mode_typedef.rev3|Data64bit<<16;
	Data64bit = gnss_mode_typedef.gnss_mode|Data64bit<<16;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Address = FLASH_GNSS_MODE_SETTING_START_ADDR;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK1\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return status;
}





//end







uint8_t erase_ProductId(void)
{
	__disable_irq();

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_USER_DI_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_USER_DI_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_USER_DI_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		you have to make sure that these data are rewritten before they are accessed during code
		execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		DCRST and ICRST bits in the FLASH_CR register. */
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	 {
	   /*
		 Error occurred while page erase.
		 User can add here some code to deal with this error.
		 PAGEError will contain the faulty page and then to know the code error on this page,
		 user can call function 'HAL_FLASH_GetError()'
	   */
		 //USB_Printf("Flash erase error\n\r");
		 return HAL_ERROR;
	 }
	 HAL_FLASH_Lock();
	__enable_irq();
	 return HAL_OK;
}


uint8_t write_ProductId(void)
{
	uint64_t Data64bit;
	uint8_t status=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	//test data
	/*
	userData.savedGpsCnt = 0x01;
	userData.finish_gps_block_index = 0x02;
	userData.finish_gps_page_index = 0x03;
	userData.rev1 = 0x04;
	*/
	// make 64bit data
	Data64bit = userData.deviceSerialId;
	Data64bit = userData.fwVersion |Data64bit<<16;
	Data64bit = userData.productId|Data64bit<<16;
	Data64bit = userData.versionId|Data64bit<<16;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Address = FLASH_USER_DI_START_ADDR;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK1\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
	//userData.noUsed = 0x0011;
	//userData.noUsed_1 = 0x0022;
	
	Address+= 8;
	Data64bit = userData.product_version;
	Data64bit = userData.factory_code|Data64bit<<16;
	Data64bit = userData.makeWeek|Data64bit<<16;
	Data64bit = userData.makeYear|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
	}

	//v1.8 생산 코드 추가 : (msb)A01 경우 userData.lot_number[3] = '1'; userData.lot_number[2] = '0'; userData.lot_number[1] = 'A'; userData.lot_number[0] = '0';
	Address+= 8;
	Data64bit = userData.lot_number[0];
	Data64bit = userData.lot_number[1]|Data64bit<<8;
	Data64bit = userData.lot_number[2]|Data64bit<<8;
	Data64bit = userData.lot_number[3]|Data64bit<<8;

	Data64bit = userData.ext_mess_reservd|Data64bit<<16;
	Data64bit = userData.ext_mess_reservd2|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
	}



	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return status;


}





uint8_t eraseInteralflash(void)
{
	__disable_irq();

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_USER_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_USER_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		you have to make sure that these data are rewritten before they are accessed during code
		execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		DCRST and ICRST bits in the FLASH_CR register. */
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	 {
	   /*
		 Error occurred while page erase.
		 User can add here some code to deal with this error.
		 PAGEError will contain the faulty page and then to know the code error on this page,
		 user can call function 'HAL_FLASH_GetError()'
	   */
		 //USB_Printf("Flash erase error\n\r");
		 return HAL_ERROR;
	 }
	 HAL_FLASH_Lock();
	__enable_irq();
	 return HAL_OK;
}


extern uint8_t debugPrint;
// 01,02,03,04 저장되면, read 경우 04,03,02,01 된다, // read 4byte
// read 경우 구조체 64byte 아래 변수 값 붙터 저장 한다.



void readInteralflash(void)
{

//Read Device information
	Address = FLASH_USER_DI_START_ADDR;
	data32 = *(__IO uint32_t *)Address;

	userData.versionId = data32;
	userData.productId = data32>>16;
	//USB_Printf("fwVersion:%04x deviceSerialId:%04x \n\r",userData.fwVersion,userData.deviceSerialId);
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.fwVersion= data32;
	userData.deviceSerialId= data32>>16;
	//USB_Printf("versionId:%04x productId:%04x \n\r",userData.versionId,userData.productId);


	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.makeYear= data32;		//RF channel 포함 4byte
	userData.makeWeek= data32>>16; 
//USB_Printf("noUsed:%04x noUsed_1:%04x \n\r",userData.noUsed,userData.noUsed_1);
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.factory_code= data32;		//RF channel 포함 4byte
	userData.product_version = data32>>16;
//	USB_Printf("makeWeek:%04x makeYear:%04x \n\r",userData.makeWeek,userData.makeYear);

	//v1.8 추가 생산 코드 : 32bit read
	Address = Address + 4;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.lot_number[0]= data32;
	userData.lot_number[1]= data32>>8;
	userData.lot_number[2]= data32>>16;
	userData.lot_number[3]= data32>>24;


// Read GPS IMU block page index
// gps data
	Address = FLASH_USER_START_ADDR;
	data32 = *(__IO uint32_t *)Address;
	// v1.7: gps data full indication
	userData.gpsData_full_flag = data32;
	userData.finish_gps_page_index = data32>>16;
	//USB_Printf("gameCount:%d gps_page_idx:%d \n\r",userData.gameCount,userData.finish_gps_page_index);

	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.finish_gps_block_index = data32;
	userData.savedGpsCnt = data32>>16;
	//USB_Printf("gps_block_idx:%d savedGpsCnt:%d\n\r",userData.finish_gps_block_index, userData.savedGpsCnt);

// imu data
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	// v1.7: gps data full indication
	userData.imuData_full_flag = data32;
	userData.finish_imu_page_index = data32>>16;
	//USB_Printf("gameCount2:%d imu_page_idx:%d\n\r",userData.gameCount2, userData.finish_imu_page_index);

	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.finish_imu_block_index = data32;
	userData.savedImuCnt = data32>>16;
	//USB_Printf("imu_block_idx:%d imu_page_idx:%d\n\r",userData.finish_imu_block_index, userData.savedImuCnt);

	// X4: save herat rate data
	#if (FEATURE_SAVE_HR_DEAT)
		Address = Address + 4;
		data32 = *(__IO uint32_t *)Address;
		userData.hr_save_data_cnt = data32;
		userData.hr_ReadError = data32>>16;
		//USB_Printf("game4_gps:%d game3_gps:%d\n\r",userData.game4_all_gps_page_size, userData.game3_all_gps_page_size);

		Address = Address + 4;
		data32 = *(__IO uint32_t *)Address;
		userData.finish_hr_page_index = data32;
		userData.finish_hr_block_index = data32>>16;
		//USB_Printf("game2_gps:%d game1_gps:%d\n\r",userData.game2_all_gps_page_size, userData.game1_all_gps_page_size);
	#endif



// game imu size information
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.game4_all_imu_page_size = data32;
	userData.game3_all_imu_page_size = data32>>16;
	//USB_Printf("game4_imu:%d game3_imu:%d\n\r",userData.game4_all_imu_page_size, userData.game3_all_imu_page_size);

	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.game2_all_imu_page_size = data32;
	userData.game1_all_imu_page_size = data32>>16;
	//USB_Printf("game2_imu:%d game1_imu:%d\n\r",userData.game2_all_imu_page_size, userData.game1_all_imu_page_size);

// nand error
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.imuReadError = data32;
	userData.imuWriteError = data32>>16;
	//USB_Printf("imuReadError:%d imuWriteError:%d\n\r",userData.imuReadError, userData.imuWriteError);

	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userData.gpsReadError = data32;
	userData.gpsWriteError = data32>>16;
	//USB_Printf("gpsReadError:%d gpsWriteError:%d\n\r",userData.gpsReadError, userData.gpsWriteError);

	// if gps block is saved, next start block ... page_index_gps 0 ~ 63.
	if(userData.finish_gps_block_index != 0xffff)
	{
		block_index_gps = userData.finish_gps_block_index;
		page_index_gps = userData.finish_gps_page_index+1;
		if(page_index_gps >= nand0.info.pages_per_block)  //64
		{
			//v16: 버그 수정 이전 power off 당시 page 63경우 page_index_gps 64 보다 크면 문제 발생. 초기화 필요
			page_index_gps = 0;
			block_index_gps++;
		}
	}
	// no save gps block, default setting
	else
	{
		block_index_gps = GPS_BLOCK_START;
		page_index_gps = 0;
	}
	start_gps_block_index = block_index_gps;
	start_gps_page_index = page_index_gps;

	if(userData.finish_imu_block_index != 0xffff)
	{
		block_index_imu = userData.finish_imu_block_index;
		page_index_imu = userData.finish_imu_page_index+1;
		if(page_index_imu >= nand0.info.pages_per_block)
		{
			//v16: 버그 수정 이전 power off 당시 page 63경우 page_index_imu 64 보다 크면 문제 발생. 초기화 필요
			page_index_imu = 0;
			block_index_imu++;
		}
	}
	// no save gps block, default setting
	else
	{
		block_index_imu = IMU_BLOCK_START;
		page_index_imu = 0;
	}
	start_imu_block_index = block_index_imu;
	start_imu_page_index = page_index_imu;

	// X4: save herat rate data
	#if (FEATURE_SAVE_HR_DEAT)
		//X4: if heart rate is saved,
		if(userData.finish_hr_block_index != 0xffff)
		{
			block_index_hr = userData.finish_hr_block_index;
			page_index_hr = userData.finish_hr_page_index+1;
			if(page_index_hr >= nand0.info.pages_per_block) //64
			{
				//v16: 버그 수정 이전 power off 당시 page 63경우 page_index_imu 64 보다 크면 문제 발생. 초기화 필요
				page_index_hr = 0;
				block_index_hr++;
			}
		}
		// no save gps block, default setting
		else
		{
			block_index_hr = HEART_RATE_BLOCK_START;
			page_index_hr = 0;
		}
		start_hr_block_index = block_index_hr;
		start_hr_page_index = page_index_hr;
	#endif


	userData.fwVersion= CELL_FW_MAJOR_VER << 8;
	userData.fwVersion |= CELL_FW_MINOR_VER;
	

	// no message
	if(0)
	{
		//USB_Printf("start gps_block:%d gps_page:%d\n\r",start_gps_block_index,start_gps_page_index);
		//USB_Printf("start imu_block:%d imu_page:%d\n\r",start_imu_block_index,start_imu_page_index);
		//USB_Printf("start hr_block:%d hr_page:%d\n\r",start_hr_block_index,start_hr_page_index);
		//USB_Printf("gameCount:%d gps_page_idx:%d \n\r",userData.gpsData_full_flag,userData.finish_gps_page_index);
		//USB_Printf("gps_block_idx:%d savedGpsCnt:%d\n\r",userData.finish_gps_block_index, userData.savedGpsCnt);

		//USB_Printf("gameCount2:%d imu_page_idx:%d\n\r",userData.imuData_full_flag, userData.finish_imu_page_index);
		//USB_Printf("imu_block_idx:%d imu_page_idx:%d\n\r",userData.finish_imu_block_index, userData.savedImuCnt);

		//gps game
		//USB_Printf("game4_gps:%d game3_gps:%d\n\r",userData.game4_all_gps_page_size, userData.game3_all_gps_page_size);
		//USB_Printf("game2_gps:%d game1_gps:%d\n\r",userData.game2_all_gps_page_size, userData.game1_all_gps_page_size);

		//imu game
		//USB_Printf("game4_imu:%d game3_imu:%d\n\r",userData.game4_all_imu_page_size, userData.game3_all_imu_page_size);
		//USB_Printf("game2_imu:%d game1_imu:%d\n\r",userData.game2_all_imu_page_size, userData.game1_all_imu_page_size);

		//USB_Printf("imuReadError:%d imuWriteError:%d\n\r",userData.imuReadError, userData.imuWriteError);
		//USB_Printf("gpsReadError:%d gpsWriteError:%d\n\r",userData.gpsReadError, userData.gpsWriteError);
	}

	// no save nand error, defualt setting
	setup_default_value();


}


uint8_t writeInteralflash(void)
{
	uint64_t Data64bit;
	uint8_t status=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	//test data
	/*
	userData.savedGpsCnt = 0x01;
	userData.finish_gps_block_index = 0x02;
	userData.finish_gps_page_index = 0x03;
	userData.rev1 = 0x04;
	*/
	// make 64bit data
	Data64bit = userData.savedGpsCnt;
	Data64bit = userData.finish_gps_block_index|Data64bit<<16;
	Data64bit = userData.finish_gps_page_index|Data64bit<<16;
	// v1.7: gps data full indication
	Data64bit = userData.gpsData_full_flag|Data64bit<<16;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Address = FLASH_USER_START_ADDR;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK1\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}

	// test data
	/*
	userData.savedImuCnt = 0x04;
	userData.finish_imu_block_index = 0x05;
	userData.finish_imu_page_index = 0x06;
	userData.rev2 = 0x07;
	*/
	Address+= 8;
	Data64bit = userData.savedImuCnt;
	Data64bit = userData.finish_imu_block_index|Data64bit<<16;
	Data64bit = userData.finish_imu_page_index|Data64bit<<16;
	// v1.7: imu data full indication
	Data64bit = userData.imuData_full_flag|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK2\n\r");

	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}

	// X4: save herat rate data
	#if (FEATURE_SAVE_HR_DEAT)
		Address+= 8;
		Data64bit = userData.finish_hr_block_index;	// heart rate size
		Data64bit = userData.finish_hr_page_index|Data64bit<<16; // heart rate size
		Data64bit = userData.hr_ReadError|Data64bit<<16;
		Data64bit = userData.hr_save_data_cnt|Data64bit<<16;

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
		{
			//USB_Printf("OK3\n\r");

		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			status =1;
			//USB_Printf("Flash write error\n\r");
		}
	#endif

// game imu size information
	Address+= 8;
	Data64bit = userData.game1_all_imu_page_size;
	Data64bit = userData.game2_all_imu_page_size|Data64bit<<16;
	Data64bit = userData.game3_all_imu_page_size|Data64bit<<16;
	Data64bit = userData.game4_all_imu_page_size|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK3\n\r");

	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}


#if 0
// game 2
	Address+= 8;
	Data64bit = userData.game2_gps_block_size;
	Data64bit = userData.game2_gps_page_size|Data64bit<<16;
	Data64bit = userData.game2_imu_block_size|Data64bit<<16;
	Data64bit = userData.game2_imu_page_size|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		USB_Printf("OK4\n\r");

	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		USB_Printf("Flash write error\n\r");
	}
// game 3
	Address+= 8;
	Data64bit = userData.game3_gps_block_size;
	Data64bit = userData.game3_gps_page_size|Data64bit<<16;
	Data64bit = userData.game3_imu_block_size|Data64bit<<16;
	Data64bit = userData.game3_imu_page_size|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		USB_Printf("OK5\n\r");

	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		USB_Printf("Flash write error\n\r");
	}
#endif
	/*
	userData.gpsWriteError = 0x08;
	userData.gpsReadError = 0x09;
	userData.imuWriteError = 0x0a;
	userData.imuReadError = 0x0b;
	*/
	Address+= 8;
	Data64bit = userData.gpsWriteError;
	Data64bit = userData.gpsReadError|Data64bit<<16;
	Data64bit = userData.imuWriteError|Data64bit<<16;
	Data64bit = userData.imuReadError|Data64bit<<16;

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK6\n\r");

	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return status;
}

void internalflashtest(void)
{
	eraseInteralflash();
	writeInteralflash();
	readInteralflash();
}

//v1.8: imu bias value 저장
uint8_t erase_ImuCalibration(void)
{
	__disable_irq();

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_IMU_CAL_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_IMU_CAL_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_IMU_CAL_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		you have to make sure that these data are rewritten before they are accessed during code
		execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		DCRST and ICRST bits in the FLASH_CR register. */
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	 {
	   /*
		 Error occurred while page erase.
		 User can add here some code to deal with this error.
		 PAGEError will contain the faulty page and then to know the code error on this page,
		 user can call function 'HAL_FLASH_GetError()'
	   */
		 //USB_Printf("Flash erase error\n\r");
		 return HAL_ERROR;
	 }
	 HAL_FLASH_Lock();
	__enable_irq();
	 return HAL_OK;
}


void read_ImuCalibration(void)
{
	uint8_t idx;
	//float f = 3.124;

// write 겨우 2word 단위 때문에 read [0],[2],[4]한다.
	Address = FLASH_IMU_CAL_START_ADDR;
	for(idx = 0; idx < 6; idx++)
	{
		data32 = *(__IO uint32_t *)Address;		//read
		accelBias[idx] =  *(float *)&data32;
		//USB_Printf("#1 address 0x%x, val %f\r\n", Address, accelBias[idx]);
		Address +=4;
	}

	for(idx = 0; idx < 6; idx++)
	{
		data32 = *(__IO uint32_t *)Address;		//read
		gyroBias[idx] =  *(float *)&data32;
		//USB_Printf("#2 address 0x%x, val %f\r\n", Address, gyroBias[idx]);
		Address +=4;
	}

	for(idx = 0; idx < 6; idx++)
	{
		data32 = *(__IO uint32_t *)Address;		//read
		magCalibration[idx] =  *(float *)&data32;
		//USB_Printf("#3 address 0x%x, val %f\r\n", Address, magCalibration[idx]);
		Address +=4;
	}
/*
	accelBias[0] = -1.969466; accelBias[1] = 0.541985; 		accelBias[2] = 2.282443;
	gyroBias[0] = 0.070984; 	gyroBias[1] = -1.146729; 	gyroBias[2] = 0.972900;
	magCalibration[0] = 1.167969; magCalibration[1] = 1.179688; magCalibration[2] = -1.132812;
*/
	//USB_Printf("cal %f,%f,%f,%f,%f,%f,%f,%f,%f\n\r",accelBias[0],accelBias[2],accelBias[4],gyroBias[0],gyroBias[2],gyroBias[4],magCalibration[0],magCalibration[2],magCalibration[4]);
	//USB_Printf("test %f", f);
}



uint8_t write_ImuCalibration(void)
{
	uint64_t Data64bit;
	uint8_t status=0, idx;
	uint32_t dummy = 0x0000;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	//test data
/*
	accelBias[0] = -1.969466; 		accelBias[1] = 0.541985; 			accelBias[2] = 2.282443;
	gyroBias[0] = 0.070984; 		gyroBias[1] = -1.146729; 			gyroBias[2] = 0.972900;
	magCalibration[0] = 1.167969; 	magCalibration[1] = 1.179688; 		magCalibration[2] = -1.132812;
*/
	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Address = FLASH_IMU_CAL_START_ADDR;
	for(idx = 0; idx <3; idx++)
	{
		//USB_Printf("#1 address 0x%x, val %f\r\n", Address, accelBias[idx]);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(uint32_t*)&accelBias[idx]) == HAL_OK)
		{
			//USB_Printf("OK1\n\r");
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			status =1;
			//USB_Printf("#1 Flash write error\n\r");
		}
		Address += 8;
	}

	for(idx = 0; idx <3; idx++)
	{
		//USB_Printf("#2 address 0x%x, val %f\r\n", Address, gyroBias[idx]);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(uint32_t*)&gyroBias[idx]) == HAL_OK)
		{
			//USB_Printf("OK1\n\r");
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			status =1;
			//USB_Printf("#2 Flash write error\n\r");
		}
		Address += 8;
	}
	for(idx = 0; idx <3; idx++)
	{
		//USB_Printf("#3 address 0x%x, val %f\r\n", Address, magCalibration[idx]);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(uint32_t*)&magCalibration[idx]) == HAL_OK)
		{
			//USB_Printf("OK1\n\r");
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			status =1;
			//USB_Printf("#3 Flash write error\n\r");
		}
		Address += 8;
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return status;
}

//X4 wifi configuration
uint8_t erase_wifi_configuration(void)
{
	__disable_irq();

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_WIFI_CONFIG_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_WIFI_CONFIG_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_WIFI_CONFIG_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		you have to make sure that these data are rewritten before they are accessed during code
		execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		DCRST and ICRST bits in the FLASH_CR register. */
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	 {
	   /*
		 Error occurred while page erase.
		 User can add here some code to deal with this error.
		 PAGEError will contain the faulty page and then to know the code error on this page,
		 user can call function 'HAL_FLASH_GetError()'
	   */
		 //USB_Printf("Flash erase error\n\r");
		 return HAL_ERROR;
	 }
	 HAL_FLASH_Lock();
	__enable_irq();
	 return HAL_OK;
}

void read_wifi_configuration(void)
{
	uint8_t zero_cnt = 0;
// WIFI AP SSID
	Address = FLASH_WIFI_CONFIG_START_ADDR;
	data32 = *(__IO uint32_t *)Address;

	userWifiConfig.ap_ssid[7] = data32;
	userWifiConfig.ap_ssid[6] = data32>>8;
	userWifiConfig.ap_ssid[5] = data32>>16;
	userWifiConfig.ap_ssid[4] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ssid[3] = data32;
	userWifiConfig.ap_ssid[2] = data32>>8;
	userWifiConfig.ap_ssid[1] = data32>>16;
	userWifiConfig.ap_ssid[0] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ssid[15] = data32;
	userWifiConfig.ap_ssid[14] = data32>>8;
	userWifiConfig.ap_ssid[13] = data32>>16;
	userWifiConfig.ap_ssid[12] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ssid[11] = data32;
	userWifiConfig.ap_ssid[10] = data32>>8;
	userWifiConfig.ap_ssid[9] = data32>>16;
	userWifiConfig.ap_ssid[8] = data32>>24;

// WIFI AP PASSWORD : no use
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_password[7] = data32;
	userWifiConfig.ap_password[6] = data32>>8;
	userWifiConfig.ap_password[5] = data32>>16;
	userWifiConfig.ap_password[4] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_password[3] = data32;
	userWifiConfig.ap_password[2] = data32>>8;
	userWifiConfig.ap_password[1] = data32>>16;
	userWifiConfig.ap_password[0] = data32>>24;

// WIFI AP IP ADDRESS
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ip_address[7] = data32;
	userWifiConfig.ap_ip_address[6] = data32>>8;
	userWifiConfig.ap_ip_address[5] = data32>>16;
	userWifiConfig.ap_ip_address[4] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ip_address[3] = data32;
	userWifiConfig.ap_ip_address[2] = data32>>8;
	userWifiConfig.ap_ip_address[1] = data32>>16;
	userWifiConfig.ap_ip_address[0] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ip_address[15] = data32;
	userWifiConfig.ap_ip_address[14] = data32>>8;
	userWifiConfig.ap_ip_address[13] = data32>>16;
	userWifiConfig.ap_ip_address[12] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ap_ip_address[11] = data32;
	userWifiConfig.ap_ip_address[10] = data32>>8;
	userWifiConfig.ap_ip_address[9] = data32>>16;
	userWifiConfig.ap_ip_address[8] = data32>>24;


// WIFI CELL IP ADDRESS
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.cell_ip_address[7] = data32;
	userWifiConfig.cell_ip_address[6] = data32>>8;
	userWifiConfig.cell_ip_address[5] = data32>>16;
	userWifiConfig.cell_ip_address[4] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.cell_ip_address[3] = data32;
	userWifiConfig.cell_ip_address[2] = data32>>8;
	userWifiConfig.cell_ip_address[1] = data32>>16;
	userWifiConfig.cell_ip_address[0] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.cell_ip_address[15] = data32;
	userWifiConfig.cell_ip_address[14] = data32>>8;
	userWifiConfig.cell_ip_address[13] = data32>>16;
	userWifiConfig.cell_ip_address[12] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.cell_ip_address[11] = data32;
	userWifiConfig.cell_ip_address[10] = data32>>8;
	userWifiConfig.cell_ip_address[9] = data32>>16;
	userWifiConfig.cell_ip_address[8] = data32>>24;

// BLE DEVICE ADDRESS
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ble_device_name[7] = data32;
	userWifiConfig.ble_device_name[6] = data32>>8;
	userWifiConfig.ble_device_name[5] = data32>>16;
	userWifiConfig.ble_device_name[4] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ble_device_name[3] = data32;
	userWifiConfig.ble_device_name[2] = data32>>8;
	userWifiConfig.ble_device_name[1] = data32>>16;
	userWifiConfig.ble_device_name[0] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ble_device_name[15] = data32;
	userWifiConfig.ble_device_name[14] = data32>>8;
	userWifiConfig.ble_device_name[13] = data32>>16;
	userWifiConfig.ble_device_name[12] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ble_device_name[11] = data32;
	userWifiConfig.ble_device_name[10] = data32>>8;
	userWifiConfig.ble_device_name[9] = data32>>16;
	userWifiConfig.ble_device_name[8] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ble_device_name[23] = data32;
	userWifiConfig.ble_device_name[22] = data32>>8;
	userWifiConfig.ble_device_name[21] = data32>>16;
	userWifiConfig.ble_device_name[20] = data32>>24;
	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.ble_device_name[19] = data32;
	userWifiConfig.ble_device_name[18] = data32>>8;
	userWifiConfig.ble_device_name[17] = data32>>16;
	userWifiConfig.ble_device_name[16] = data32>>24;

	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.reserve_2 = data32;
	userWifiConfig.reserve_1 = data32>>16;

	Address = Address + 4;
	data32 = *(__IO uint32_t *)Address;
	userWifiConfig.reserve = data32;
	userWifiConfig.connect_flag = data32>>16;
// no ble name, userWifiConfig.ble_device_name[] = {0x00,0x00,0x00,0x00 ....}
	for(int i=0; i<24; i++)
	{
		if(userWifiConfig.ble_device_name[i] == 0x00)
		{
			zero_cnt++;
		}
	}
	if(zero_cnt <= 10) // 만약 0x00 개수가 10 보다 작으면 즉 데이터 존재 한다.
	{
		has_ble_name = 1;
	}
	//USB_Printf("ble name %d\r\n", has_ble_name);

//	USB_Printf("ssid \r\n");
//	for(int i=0; i<16; i++)
//	{
//		USB_Printf("%c ", userWifiConfig.ap_ssid[i]);
//		USB_Printf("%x ", userWifiConfig.ap_ssid[i]);
//	}
//	USB_Printf("\r\n");
//
////	for(int i=0; i<8; i++)
////	{
////		USB_Printf("%c", userWifiConfig.ap_password[i]);
////	}
//
//	USB_Printf("AP ip \r\n");
//	for(int i=0; i<16; i++)
//	{
//		USB_Printf("%c ", userWifiConfig.ap_ip_address[i]);
//		//USB_Printf("%x ", userWifiConfig.ap_ip_address[i]);
//	}
//	USB_Printf("\r\n");
//
//	USB_Printf("cell ip \r\n");
//	for(int i=0; i<16; i++)
//	{
//		USB_Printf("%c ", userWifiConfig.cell_ip_address[i]);
//		//USB_Printf("%x ", userWifiConfig.cell_ip_address[i]);
//	}
//	USB_Printf("\r\n");
//
//	USB_Printf("ble name \r\n");
//	for(int i=0; i<24; i++)
//	{
//		USB_Printf("%c ", userWifiConfig.ble_device_name[i]);
//		USB_Printf("%x ", userWifiConfig.ble_device_name[i]);
//	}
//	USB_Printf("\r\n");


//	USB_Printf("data %x %x %x %x\r\n", userWifiConfig.connect_flag, userWifiConfig.reserve, userWifiConfig.reserve_1,userWifiConfig.reserve_2);


}


uint8_t write_wifi_configuration(void)
{
	uint64_t Data64bit;
	uint8_t status=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	// test data
/*	for(int i=0; i<16; i++)
	{
		userWifiConfig.ap_ssid[i] = 0x10+i;
	}
	for(int i=0; i<8; i++)
	{
		userWifiConfig.ap_password[i] = 0x20+i;
	}
	for(int i=0; i<4; i++)
	{
		userWifiConfig.ap_ip_address[i] = 0x30+i;
	}

	for(int i=0; i<4; i++)
	{
		userWifiConfig.cell_ip_address[i] = 0x40+i;
	}
	 userWifiConfig.connect_flag = 0xe1;
	 userWifiConfig.reserve = 0xe2;
	 userWifiConfig.reserve_1 = 0xe3;
	 userWifiConfig.reserve_2 = 0xe4;
*/


	Address = FLASH_WIFI_CONFIG_START_ADDR;

	Data64bit = userWifiConfig.ap_ssid[0];
	Data64bit = userWifiConfig.ap_ssid[1]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[2]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[3]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[4]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[5]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[6]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[7]|Data64bit<<8;
	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK1\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
	Address +=8;
	Data64bit = userWifiConfig.ap_ssid[8];
	Data64bit = userWifiConfig.ap_ssid[9]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[10]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[11]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[12]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[13]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[14]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ssid[15]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK2\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
// AP SSID END

	Address +=8;
	Data64bit = userWifiConfig.ap_password[0];
	Data64bit = userWifiConfig.ap_password[1]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_password[2]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_password[3]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_password[4]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_password[5]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_password[6]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_password[7]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK3\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
// AP PassWord END

	Address +=8;
	Data64bit = userWifiConfig.ap_ip_address[0];
	Data64bit = userWifiConfig.ap_ip_address[1]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[2]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[3]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[4]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[5]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[6]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[7]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK4\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}

	Address +=8;
	Data64bit = userWifiConfig.ap_ip_address[8];
	Data64bit = userWifiConfig.ap_ip_address[9]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[10]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[11]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[12]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[13]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[14]|Data64bit<<8;
	Data64bit = userWifiConfig.ap_ip_address[15]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK5\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
////////////////////////////////////////////// AP IP Address END //////////////////////////////////////////////

	Address +=8;
	Data64bit = userWifiConfig.cell_ip_address[0];
	Data64bit = userWifiConfig.cell_ip_address[1]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[2]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[3]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[4]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[5]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[6]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[7]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK6\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}

	Address +=8;
	Data64bit = userWifiConfig.cell_ip_address[8];
	Data64bit = userWifiConfig.cell_ip_address[9]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[10]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[11]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[12]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[13]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[14]|Data64bit<<8;
	Data64bit = userWifiConfig.cell_ip_address[15]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK7\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
////////////////////////////////////////////// CELL IP Address END //////////////////////////////////////////////

	Address +=8;
	Data64bit = userWifiConfig.ble_device_name[0];
	Data64bit = userWifiConfig.ble_device_name[1]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[2]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[3]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[4]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[5]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[6]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[7]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK7\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}


	Address +=8;
	Data64bit = userWifiConfig.ble_device_name[8];
	Data64bit = userWifiConfig.ble_device_name[9]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[10]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[11]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[12]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[13]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[14]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[15]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK7\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}


	Address +=8;
	Data64bit = userWifiConfig.ble_device_name[16];
	Data64bit = userWifiConfig.ble_device_name[17]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[18]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[19]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[20]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[21]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[22]|Data64bit<<8;
	Data64bit = userWifiConfig.ble_device_name[23]|Data64bit<<8;

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK7\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}
////////////////////////////////////////////// BLE DEVICE Address END //////////////////////////////////////////////

	Address +=8;
	Data64bit = userWifiConfig.connect_flag;
	Data64bit = userWifiConfig.reserve|Data64bit<<16;
	Data64bit = userWifiConfig.reserve_1|Data64bit<<16;
	Data64bit = userWifiConfig.reserve_2|Data64bit<<16;
	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)Data64bit) == HAL_OK)
	{
		//USB_Printf("OK8\n\r");
	}
	else
	{
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		status =1;
		//USB_Printf("Flash write error\n\r");
	}

	/* Lock the Flash to disable the flash control register access (recommended
		 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return status;
}




















void testSavefloatingVal(void)
{
	int ret;

	ret = erase_ImuCalibration();
	write_ImuCalibration();
	read_ImuCalibration();
}

void testProductId(void)
{
	int ret;
	//USB_Printf("testProductId\r\n");

	ret = erase_ProductId();
	write_ProductId();
	readInteralflash();
}


