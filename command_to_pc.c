tSerialMessageFormat		hostToMcu;	//Uart In(Send Commend)
tSerialMessageFormat		mcuToHost;	//Uart Out(ACK)

uint8_t gpsCompressflag =0;
uint8_t setImuFrequency =10;



#if 0
//v1.7:lz4 compression
unsigned char Txbuff[1024*4];
uint16_t sendTx_length = 0, overflow_length=0;

/* Compression */
// We'll store some text into a variable pointed to by *src to be compressed later.
//const char* const src = "123456789012345678901234567890";

//v1.7:lz4 compression
void lz4CompressIint(void)
{
	sendTx_length = 0;
	overflow_length = 0;
	memset(Txbuff, 0x00, sizeof(Txbuff));
}

uint8_t lastpage=0;
/* Introduction */
// Below we will have a Compression and Decompression section to demonstrate.
// There are a few important notes before we start:
//   1) The return codes of LZ4_ functions are important.
//      Read lz4.h if you're unsure what a given code means.
//   2) LZ4 uses char* pointers in all LZ4_ functions.
//      This is baked into the API and not going to change, for consistency.
//      If your program uses different pointer types,
//      you may need to do some casting or set the right -Wno compiler flags to ignore those warnings (e.g.: -Wno-pointer-sign).

int lz4Compress(const char* const src, const int src_size)
{

	uint16_t len=0;
	uint16_t size,j;

	size = src_size;

  // LZ4 provides a function that will tell you the maximum size of compressed output based on input data via LZ4_compressBound().
  const int max_dst_size = LZ4_compressBound(size);


  // We will use that size for our destination boundary when allocating space.
   char* compressed_data = malloc(max_dst_size);

   if (compressed_data == NULL)
   {
	   //USB_Printf("Failed to allocate memory for *compressed_data.\r\n");
   }

   const int compressed_data_size = LZ4_compress_default(src, compressed_data, size, max_dst_size);

  // That's all the information and preparation LZ4 needs to compress *src into *compressed_data.
  // Invoke LZ4_compress_default now with our size values and pointers to our memory locations.
  // Save the return value for error checking.
  // Check return_value to determine what happened.
  if (compressed_data_size <= 0)
  {

  }
  else
  {
#if 1
	  Txbuff[sendTx_length++] = '&';
	  Txbuff[sendTx_length++] = '&';
	  Txbuff[sendTx_length++] = compressed_data_size >> 8;
	  Txbuff[sendTx_length++] = compressed_data_size;
	  len = sendTx_length;


	  for(uint16_t j = 0; j<compressed_data_size; j++)
	  {
		  Txbuff[len+j] = compressed_data[j];
	  }
	  sendTx_length += compressed_data_size;


	  if(sendTx_length >= 2048)
	  {
#if 1
		  overflow_length = sendTx_length - 2048;
		  char* overflowBuff = malloc(overflow_length);


		  for(uint16_t i = 0; i < overflow_length; i++)
		  {
			  overflowBuff[i] = Txbuff[2048+i];
		  }
		  sendTx_length = 0;
		  // send data by UART
		  while(CDC_Transmit_FS((uint8_t *)&Txbuff[0],2048) != USBD_OK )
		  {
				HAL_Delay(1);
		  }
		  for(uint16_t i = 0; i < overflow_length; i++)
		  {
			  Txbuff[i] = overflowBuff[i];
		  }

		  sendTx_length += overflow_length;
		  free(overflowBuff);
#endif
	  }

	  free(compressed_data);   /* no longer useful */
#endif
  	}



   if(lastpage)
   {
	  lastpage = 0;
	  while(CDC_Transmit_FS((uint8_t *)&Txbuff[0],2048) != USBD_OK )
	  {
	 	HAL_Delay(1);
	  }
	  return 1;
   }

// debug :decompression
#if 0

  /* Decompression */
  // Now that we've successfully compressed the information from *src to *compressed_data, let's do the opposite!
  // The decompression will need to know the compressed size, and an upper bound of the decompressed size.
  // In this example, we just re-use this information from previous section,
  // but in a real-world scenario, metadata must be transmitted to the decompression side.
  // Each implementation is in charge of this part. Oftentimes, it adds some header of its own.
  // Sometimes, the metadata can be extracted from the local context.

  // First, let's create a *new_src location of size src_size since we know that value.
  char* const regen_buffer = unCompressbuff2;

  // The LZ4_decompress_safe function needs to know where the compressed data is, how many bytes long it is,
  // where the regen_buffer memory location is, and how large regen_buffer (uncompressed) output will be.
  // Again, save the return_value.

  const int decompressed_size = LZ4_decompress_safe(compressed_data, regen_buffer, compressed_data_size, src_size);

  if (decompressed_size < 0)
	  USB_Printf("A negative result from LZ4_decompress_safe indicates a failure trying to decompress the data.  See exit code (echo $?) for value returned.\r\n");

  if (decompressed_size >= 0)
	  USB_Printf("We successfully decompressed some data! size %d\r\n", decompressed_size);
  // Not only does a positive return value mean success,
  // value returned == number of bytes regenerated from compressed_data stream.
  if (decompressed_size != src_size)
	  USB_Printf("Decompressed data is different from original! \r\n");

  /* Validation */
  // We should be able to compare our original *src with our *new_src and be byte-for-byte identical.
  if (memcmp(src, regen_buffer, src_size) != 0)
  {
	  USB_Printf("Validation failed.  *src and *new_src are not identical.\r\n");
  }


  for(j=0; j<2048; j++)
  {
	  if(src[j] != regen_buffer[j])
	  {
		  USB_Printf("validation failed %d\r\n", j);
	  }
  }

  USB_Printf("validation successful\r\n");
  /*
  while(CDC_Transmit_FS((uint8_t *)&regen_buffer[0],2048)!= USBD_OK)
  {
 	  HAL_Delay(1);
  }
*/
#endif

  return 0;
}
#endif

void SendgameShortGpsDataToPC(void)
{
	uint32_t block,page;
	int32_t status =EIO;
	uint8_t msg[6]= {'G','P','S','S','H','O','R','T'};


	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
//	USB_Printf("gps:block: %d page:%d\r\n", userData.finish_gps_block_index,userData.finish_gps_page_index);
	memset(&ReadBuffer[0], 0x00, PAGE_OOB_SIZE);

	for(block=GPS_BLOCK_START+1; block<=GPS_BLOCK_START+2; block++ )
	{
		for(page=0; page<nand0.info.pages_per_block; page++)  //64
		{
			status = nand_gpio_read_page_ecc(&nand0,block,page,ReadBuffer);
			if(status < 0)
			{
				USB_Printf("status:%d block:%d page:%d\r\n", status,block,page);
				//break; //this block is bad block. or Empty block. next block.
			}
			else
			{
				if(gpsCompressflag)
				{
					compressGpsData(&ReadBuffer[0]);
				}
				else
				{
					while(CDC_Transmit_FS((uint8_t *)&ReadBuffer[0],2048) != USBD_OK )
					{
						HAL_Delay(1);
					}
				}
			}
		}
	}
	while(CDC_Transmit_FS((uint8_t *)&msg[0],8) != USBD_OK )
	{
		HAL_Delay(1);
	}

}


void send_GpsDataToPc(void)
{
	uint8_t keyIn;
	int32_t status;
	static int16_t block,page;
	uint8_t msg[6]= {'G','P','S','E','N','D'};
	uint8_t errmsg[6]= {'G','P','S','E','R','R'};

	static uint8_t isOn=0;
	USB_Printf("save :block %d page %d\r\n",userData.finish_gps_block_index, userData.finish_gps_page_index);

	 
	if(userData.finish_gps_block_index >  GPS_BLOCK_END)
	{
		userData.finish_gps_block_index = GPS_BLOCK_END -1;
	
	}
	if(userData.finish_gps_page_index >  64)
	{
		userData.finish_gps_page_index  = 63;
		//USB_Printf("page %d\r\n",userData.finish_gps_page_index);
	}
	memset(&ReadBuffer[0], 0x00, PAGE_OOB_SIZE);
	block = GPS_BLOCK_START;
	page = 0;
 	while(1)
 	{
 		keyIn = '0';
 		if(VCP_Ready())
 		{
 			keyIn = VCP_getch();
 		}
 		if(keyIn =='q')
 		{
 			return;
 		}
 		if(keyIn=='p')
 		{
 			  USB_Printf("\r\n block:%d page:%d\r\n", block,page);
 			  status = nand_gpio_read_page_ecc_retry(&nand0, block, page, &ReadBuffer[0]);
			  if(status<0)
			  {
				  while(CDC_Transmit_FS((uint8_t *)&errmsg[0],6) != USBD_OK )
				  {
						HAL_Delay(1);
				  }
			  }
			  else
			  {
				  if(gpsCompressflag)
				  {
					  // lz4Compress(ReadBuffer, 2048);
				  }else
				  {
					  while(CDC_Transmit_FS((uint8_t *)&ReadBuffer[0],2048)!= USBD_OK)
					  {
						  HAL_Delay(1);
					  }
				  }
			  }
			  HAL_Delay(5);

			  if(page >= userData.finish_gps_page_index && block >= userData.finish_gps_block_index)
			  {

				  while(CDC_Transmit_FS((uint8_t *)&msg[0],6) != USBD_OK )
				  {
						HAL_Delay(1);
				  }
				  USB_Printf("\r\n finish block:%d page:%d\r\n",block, page);
				  HAL_Delay(10);
				  return;
			  }
			  page++;

			  if( page == nand0.info.pages_per_block)
			  {
			  	  page=0;
			  	  block++;
			  }
 		 }
	  }
 }
void send_ImuDataToPc(void)
{
	uint8_t keyIn;
	int32_t status;
	static int16_t block,page;
	uint8_t msg[6]= {'I','M','U','E','N','D'};
	uint8_t errmsg[6]= {'I','M','U','E','R','R'};

	USB_Printf("save :block %d page %d\r\n",userData.finish_imu_block_index, userData.finish_imu_page_index);
	//양산 버전:전원 off 하지 않는 상태  userData.finish_imu_block_index 없는 경우 최대값을 설정 한다.
	if(userData.finish_imu_block_index >IMU_BLOCK_END)
	{
	 userData.finish_imu_block_index = IMU_BLOCK_END -1;
	}
	if(userData.finish_imu_page_index >  64)
	{
	 userData.finish_imu_page_index  = 63;
	}

	memset(&ReadBuffer[0], 0x00, PAGE_OOB_SIZE);
	block = IMU_BLOCK_START;
	page = 0;

//	lz4CompressIint();

 	while(1)
 	{
 		keyIn = '0';
 		if(VCP_Ready())
 		{
 			keyIn = VCP_getch();
 		}
 		if(keyIn =='q')
 	 	{
 	 		return;
 	 	}

 		if(keyIn=='p')
 		{
 			USB_Printf("\r\n block %d page %d\r\n", block,page);
 			// nand read:2.4m vcp:2.3m, Total :4.7m
			  //status = nand_gpio_read_page_ecc(&nand0,block,page,&ReadBuffer[0]);
			  status = nand_gpio_read_page_ecc_retry(&nand0,block,page,&ReadBuffer[0]);
			  if(status<0)
			  {
				  // USB 통신 끝어지지 않게 ERR 문자 전송
				  while(CDC_Transmit_FS((uint8_t *)&errmsg[0],6) != USBD_OK )
				  {
						HAL_Delay(1);
				  }
			  }
			  else
			  {
				  if(gpsCompressflag)
				  {
					  // lz4Compress(ReadBuffer, 2048);
				  }else
				  {
					  while(CDC_Transmit_FS((uint8_t *)&ReadBuffer[0],2048)!= USBD_OK)
					  {
						  HAL_Delay(1);
					  }
				  }
			  }

			  if(page >= userData.finish_imu_page_index && block == userData.finish_imu_block_index)
			  {
				  USB_Printf("\r\n finish block:%d page:%d\r\n",block, page);

				  while(CDC_Transmit_FS((uint8_t *)&msg[0],6) != USBD_OK )
				  {
				  		HAL_Delay(1);
				  }
				  HAL_Delay(10);
				  return;
			  }

			  page++;
				  if( page == nand0.info.pages_per_block)
				  {
					  page=0;
					  block++;
				  }
 		 }
	 }
}


uint8_t hostUartParser(void)	//Host to Front Parser(Uart Parser)
{
	static	uint8_t	State = 0;
	static	uint8_t	*pDst;
	static	uint8_t	BufCnt;
	uint8_t	u8KeyIn;
	uint8_t	ParseOK;

	if(VCP_Ready() == 0)
	return	0;

	ParseOK = 0;
	u8KeyIn = VCP_getch();
	switch(State)
	{
		case 0 :
			if(u8KeyIn == SYNC_HOST_TO_FRONT)
			{
				State = 1;	//Sync. Byte(Host --> RF4CE Controller)
				hostToMcu.checkSum= u8KeyIn;
			}
			break;
		case 1 :
			if(u8KeyIn == SYNC_HOST_TO_FRONT2)
			{
				hostToMcu.messageSyncByte2 = u8KeyIn;
				hostToMcu.checkSum ^= u8KeyIn;
				// Header Parser
				State = 2;
			}
			else
			{
				State = 0;
			}
			break;
		case 2 :
			hostToMcu.messageBodySize = u8KeyIn;
			// X4:WIFI change length
			if(hostToMcu.messageBodySize > 100)
			{
				State = 0;
				break;
			}
			hostToMcu.checkSum ^= u8KeyIn;
			pDst = (uint8_t *)&hostToMcu.messageCommand;
			BufCnt = 0;
			State = 3;
			break;
		case 3 :
			*pDst = u8KeyIn;
			pDst++;
			hostToMcu.checkSum ^= u8KeyIn;
			BufCnt++;
			if(BufCnt == hostToMcu.messageBodySize)
			{
				State = 4;
			}
			break;
		case 4:
			if( (u8KeyIn==0) || (u8KeyIn== hostToMcu.checkSum) )
			{
				ParseOK = 1;
			}
			else
			{
			}
			State = 0;
			break;
		default	:
			State = 0;
			break;
	}
	return	ParseOK;
}


void setup_imuCalibration(void)
{
	uint8_t ret =0xff,len=0;
	uint8_t tempbuff[128] = {0x00,};

	if(hostToMcu.messageBody[0]==0) 
	{
		vertical_direction_bias = 0; 
		//USB_Printf("horizontal direction\r\n");
	}
	else if(hostToMcu.messageBody[0]==1)
	{
		vertical_direction_bias = 1; 
		//USB_Printf("vertical direction\r\n");
	}

	ret = user_Mpu9250_Calibration();
	// save data into flash
	if(ret == 1) //success
	{
		erase_ImuCalibration();
		write_ImuCalibration();

		sprintf((char*) tempbuff,"%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",accelBias[0],accelBias[1],accelBias[2],gyroBias[0],gyroBias[1],gyroBias[2],magCalibration[0],magCalibration[1],magCalibration[2]);
		len= strlen((char*)tempbuff);
	}

	mcuToHost.messageSyncByte = SYNC_FRONT_TO_HOST;
	mcuToHost.messageSyncByte2 = SYNC_FRONT_TO_HOST2;
	mcuToHost.messageBodySize = (3+len);
	mcuToHost.messageCommand = SYSCOMMAND_SET_WRITE_IMU_CAL;
	mcuToHost.messageBody[0] = 1;
	mcuToHost.messageBody[2] = ret;	// success:1, fail:0


	for(uint8_t idx = 0; idx<len; idx++)
	{
		mcuToHost.messageBody[3+idx] = tempbuff[idx];
	}
	sendData_McuTohost(&mcuToHost,mcuToHost.messageBodySize,mcuToHost.messageBodySize+3);

}



uint8_t imuBuff[128] = {0x00,};
void read_imuCalibrationValue(void)
{
	uint8_t len, imufrequency = 10;

	//test
	/*
	accelBias[0] = -1.969466; 		accelBias[1] = 0.541985; 			accelBias[2] = 2.282443;
	gyroBias[0] = 0.070984; 		gyroBias[1] = -1.146729; 			gyroBias[2] = 0.972900;
	magCalibration[0] = 1.167969; 	magCalibration[1] = 1.179688; 		magCalibration[2] = -1.132812;
	 */
	memset(&imuBuff[0], 0x00, 128);

	if(runImuFrequency == 100)	// 100hz
	{
	    imufrequency = 100;
	}

	read_ImuCalibration();
	sprintf((char*) imuBuff,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",imufrequency,accelBias[0],accelBias[2],accelBias[4],gyroBias[0],gyroBias[2],gyroBias[4],magCalibration[0],magCalibration[2],magCalibration[4]);

	len= strlen((char*)imuBuff);

	mcuToHost.messageSyncByte = SYNC_FRONT_TO_HOST;
	mcuToHost.messageSyncByte2 = SYNC_FRONT_TO_HOST2;
	mcuToHost.messageBodySize = (2+len);
	mcuToHost.messageCommand = SYSCOMMAND_SET_READ_IMU_CAL;
	mcuToHost.messageBody[0] = 1;

	for(uint8_t idx = 0; idx<len; idx++)
	{
		mcuToHost.messageBody[1+idx] = imuBuff[idx];
	}
	sendData_McuTohost(&mcuToHost,mcuToHost.messageBodySize,mcuToHost.messageBodySize+3);

}

extern uint16_t gpsError_nandPage, imuError_nandPage;
void send_GpsImuError_NandPage(void)
{

	mcuToHost.messageSyncByte = SYNC_FRONT_TO_HOST;
	mcuToHost.messageSyncByte2 = SYNC_FRONT_TO_HOST2;
	mcuToHost.messageBodySize = 6;
	mcuToHost.messageCommand = SYSCOMMAND_REPORT_GPS_IMU_ERR;
	mcuToHost.messageBody[0] = 1;
	mcuToHost.messageBody[1] = gpsError_nandPage>>8;  //msb
	mcuToHost.messageBody[2] = gpsError_nandPage;
	mcuToHost.messageBody[3] = imuError_nandPage>>8;
	mcuToHost.messageBody[4] = imuError_nandPage;

	sendData_McuTohost(&mcuToHost,mcuToHost.messageBodySize,mcuToHost.messageBodySize+3);
}

void run_mpu9250_SelfTest(void)
{
	uint8_t ret =0xff,len=0;
	float destinationData[6];


	memset(&imuBuff[0], 0x00, 128);

	MPU9250_SelfTest(&destinationData[0]);

	sprintf((char*) imuBuff,"%f,%f,%f,%f,%f,%f\r\n",destinationData[0],destinationData[1],destinationData[2],destinationData[3],destinationData[4],destinationData[5]);
	len= strlen((char*)imuBuff);


	mcuToHost.messageSyncByte = SYNC_FRONT_TO_HOST;
	mcuToHost.messageSyncByte2 = SYNC_FRONT_TO_HOST2;
	mcuToHost.messageBodySize = (3+len);
	mcuToHost.messageCommand = SYSCOMMAND_SET_MPU9250_SELF_TEST;
	mcuToHost.messageBody[0] = 1;
	mcuToHost.messageBody[1] = 2;	// success:1, fail:0

	for(uint8_t idx = 0; idx<len; idx++)
	{
		mcuToHost.messageBody[2+idx] = imuBuff[idx];
	}
	sendData_McuTohost(&mcuToHost,mcuToHost.messageBodySize,mcuToHost.messageBodySize+3);

}
