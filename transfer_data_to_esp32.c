
static uint8_t rcv_length;
uint8_t rcv_data_buffer[20];
uint8_t debug_rx_buff[20];
static uint8_t esp32_response_ok = 0;
static uint8_t hr_stage=0;
uint8_t esp32_uart_rcv[2];
uint8_t esp32_rsp_ok;

static uint8_t stage_connect=0;

uint8_t heart_rate_bpm = 0;
uint16_t rcv_hr_counter = 0;

uint16_t wifi_connection_status = 0;


static void check_hr_data(uint8_t indata)
{
	switch(hr_stage)
	{
	case 0:
		if(indata == 'H')
		{
			hr_stage = 1;
		}
		break;
	case 1:
		if(indata == 'R')
		{
			hr_stage = 2;
		}
		else
			hr_stage = 0;
		break;
	case 2:
		heart_rate_bpm = indata;
		hr_stage = 0;
		if(heart_rate_bpm != 0)
		{
			rcv_hr_counter++;
		}
		break;
	default:
		break;
	}

}
uint8_t debug_sta, debug_data;
uint16_t debug_uartrx_cnt;
//CONNETED
#if 0
static void check_wifi_status(uint8_t data_in)
{
	uint8_t rcv_data = 0x00;
	rcv_data = data_in;
	debug_data = rcv_data;
	switch(stage_connect)
	{
	case 0:
		if(rcv_data == 'C') //C
		{
			stage_connect = 1;
			debug_sta = stage_connect;

		}
		break;
	case 1:
		if(rcv_data == 'O') //O
		{
			stage_connect = 2;
			debug_sta = stage_connect;
		}
		break;
	case 2:
		if(rcv_data == 'N') //N
		{

			stage_connect = 3;
			debug_sta = stage_connect;
		}
		break;
	case 3:
		if(rcv_data == 'N') //N
		{
			debug_sta = stage_connect;
			stage_connect = 4;
		}
		break;

	case 4:
		if(rcv_data == 'E') //N
		{
			debug_sta = stage_connect;
			stage_connect = 5;
		}
		break;

	case 5:
		if(rcv_data == 'C') //N
		{
			debug_sta = stage_connect;
			stage_connect = 6;
		}
		break;
	case 6:
		if(rcv_data == 'T') //N
		{
			debug_sta = stage_connect;
			stage_connect = 0;
			wifi_connection_status++;
		}
		break;

//	case 4:
//		if(rcv_data == '-')
//		{
//			stage_connect = 5;
//			debug_sta = stage_connect;
//		}
//		break;
//	case 5:
//		if(rcv_data == '+')
//		{
//			stage_connect = 0;
//			wifi_connection_status++;
//			debug_sta = stage_connect;
//		}
//		break;

	default:
		break;
	}

}
#endif

static uint8_t make_crc_and_sendData_esp32(tSerialMessageFormat *rsp, uint8_t bodysize, uint8_t size)
{
	uint32_t timeout_tick=0;
	uint8_t state=0;
	uint8_t idx, *pSrc;
	rsp->messageBody[bodysize-1]=0x00;
	rsp->messageBody[bodysize-1] ^= rsp->messageSyncByte;
	rsp->messageBody[bodysize-1] ^= rsp->messageSyncByte2;
	rsp->messageBody[bodysize-1] ^= rsp->messageBodySize;


	pSrc = (uint8_t *)&rsp->messageCommand;
	for(idx=0; idx<rsp->messageBodySize;idx++)
	{
		rsp->messageBody[bodysize-1] ^= *pSrc;
		pSrc++;
	}

	state = HAL_UART_Transmit(&huart1, (uint8_t*)rsp, (size+1),100);
	if(state == 0)
	{
	}

	timeout_tick = HAL_GetTick();
	while((clock_time_exceed(timeout_tick, 200) == 0)) //100 msec
	{
		if(esp32_response_ok)
		{
			return 1;
		}
	}
	return 0;
}


void send_erase_command_to_esp32(void)
{
	static uint8_t uart_data_buff[5] = {'E','R','A','S','E'};
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_data_buff,5,10);
}

void send_ota_update_command_to_esp32(void)
{
	static uint8_t uart_data_buff[9] = {'O','T','A','U','P','D','A','T','E'};
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_data_buff,9,10);
}

void esp32_init(void)
{
	HAL_UART_Receive_IT(&huart1, &esp32_uart_rcv[0], 1);
	// wifi enable
	HAL_GPIO_WritePin(GPIOB, WIFI_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

//	USB_Printf("send erase command\r\n");
//	HAL_Delay(2000);
//	send_erase_command_to_esp32();
//	HAL_Delay(3000);
}

//#########################################################################################################
// HR9\n : 4byte
// ESP32OK\n : 45 53 50 33 32 4F 4B 0A : response 10.
// CONNECTED\n : response 5.
// CR LF	0D 0A	\r\n
uint8_t esp32_receive_data(uint8_t rcv_data)
{
	char* p1=NULL, *p2=NULL;
	static uint8_t find_header=0;

	check_hr_data(rcv_data);

	if(rcv_length == 0)
	{
		if(rcv_data == 'E'&& esp32_response_ok==0)
		{
			find_header = 1;
			rcv_data_buffer[rcv_length++] = rcv_data;
			return 0;
		}
		if(rcv_data == 'C')
		{
			debug_uartrx_cnt++;
			find_header = 1;
			rcv_data_buffer[rcv_length++] = rcv_data;
			return 0;
		}
	}

	if(find_header)
	{
		rcv_data_buffer[rcv_length++] = rcv_data;
		//if(esp32_response_ok ==0)
		{
			if(rcv_length==10)
			{
				p1 = (char*)strstr((const char*)rcv_data_buffer, "ESP");
				if (p1 != NULL)
				{
					esp32_response_ok = 1;
				}
				p2 = (char*)strstr((const char*)rcv_data_buffer, "CON");
				if (p2 != NULL)
				{
					wifi_connection_status++;
				}
				rcv_length = 0;
				find_header = 0;
				memcpy(debug_rx_buff, rcv_data_buffer, 10);
				memset(rcv_data_buffer, 0x00, 20);
			}
		}
	}
}


void send_wifi_configuration_to_esp32(uint8_t timeout)
{

	uint32_t timeout_tick,led_toggle_tick;

	mcuToHost.messageSyncByte = SYNC_HOST_TO_FRONT;
	mcuToHost.messageSyncByte2 = SYNC_HOST_TO_FRONT2;
	mcuToHost.messageBodySize = 73; //len = command + messageBody = 1 + 72
	mcuToHost.messageCommand = SYSCOMMAND_SEND_WIFI_CONFIG_TO_ESP32;

	// server ip
	for(int i=0; i<16; i++)
	{
		mcuToHost.messageBody[i] = userWifiConfig.ap_ip_address[i];
	}
	// client ip
	for(int i=0; i<16; i++)
	{
		mcuToHost.messageBody[i+16] = userWifiConfig.cell_ip_address[i];
	}
	// ap ssid
	for(int i=0; i<16; i++)
	{
		mcuToHost.messageBody[i+32] = userWifiConfig.ap_ssid[i];
	}
	// ble device name
	for(int i=0; i<24; i++)
	{
		mcuToHost.messageBody[i+48] = userWifiConfig.ble_device_name[i];
	}


	timeout_tick = HAL_GetTick();
	led_toggle_tick = HAL_GetTick();
	while (clock_time_exceed(timeout_tick, timeout * 1000) == 0)
	{
		HAL_UART_Receive_IT(&huart1, &esp32_uart_rcv[0], 1);
		esp32_rsp_ok = make_crc_and_sendData_esp32(&mcuToHost,mcuToHost.messageBodySize,mcuToHost.messageBodySize+3);
		if (esp32_rsp_ok)
		{

			HAL_Delay(100);
			HAL_UART_Receive_IT(&huart1, &esp32_uart_rcv[0], 1);
			break;
		}
		else{
			//USB_Printf(".");
		}
	}
}




void get_wifi_connection_info_from_esp32(void)
{
	static uint8_t uart_data_buff[6] = {'S','T','A','T','U','S'};
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_data_buff, 6,10);
}

void user_erase_wifi_configration(void)
{

	production_mode_usb_stop();

	esp32_init();
	HAL_Delay(3000);
	send_erase_command_to_esp32();
	HAL_Delay(1000 * 10);
}
extern uint16_t gpioIntCnt;

void user_esp32_ota_update(void)
{
	uint32_t tick_counter=0;

	production_mode_usb_stop();

	read_wifi_configuration();
	esp32_init();
	HAL_Delay(3000);
	send_erase_command_to_esp32();
	HAL_Delay(5000);

	send_wifi_configuration_to_esp32(10); 	//10 sec
	HAL_Delay(3000);
	send_ota_update_command_to_esp32();

	tick_counter = HAL_GetTick();
	while(1)
	{

		gpioIntCnt = 0;

		if(clock_time_exceed(tick_counter, 3*60*1000)) // 1sec tick
		{
			HAL_GPIO_WritePin(GPIOB, POWER_HOLD_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_TogglePin(GPIOB, POWER_ON_LED_Pin);
		HAL_Delay(100);
	}
}


