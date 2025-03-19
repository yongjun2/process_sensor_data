#include "includes.h"
             
volatile uint16_t GPSMSG_FLAG = 0;                           
volatile uint16_t GPSMSG_INDEX = 0;
volatile char GPS_DEBUG = 0; 

uint8_t pdop[4], hdop[4], vdop[4];
int gps_snr_value,glonass_snr_value;
char gps_snr_string[2], glonass_snr_string[2];
int pdop_value,hdop_value,vdop_value;
char pdop_string[3], hdop_string[3], vdop_string[3];


//X3 select gps, gps&glonass : 1, only gps mode
uint8_t gnssModeSelect =ONLY_GPS_MODE;
                          
char comma_position(char* buf,char cx)
{               
    char *p=buf;
    while(cx)
    {        
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;    
}

uint32_t scale_power(char m,char n)
{
    uint32_t result=1;    
    while(n--)result*=m;    
    return result;
}

int string_to_number(char* buf,char* dx)
{
    char *p=buf;
    uint32_t ires=0,fres=0;
    char ilen=0,flen=0,i;
    char mask=0;
    int res;
    while(1)
    {
        if(*p=='-'){mask|=0X02;p++;}
        if(*p==','||(*p=='*'))break;
        if(*p=='.'){mask|=0X01;p++;}
        else if(*p>'9'||(*p<'0'))   
        {   
            ilen=0;
            flen=0;
            break;
        }   
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++;
    for(i=0;i<ilen;i++)
    {  
        ires+=scale_power(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;   
    *dx=flen;           
    for(i=0;i<flen;i++) 
    {  
        fres+=scale_power(10,flen-1-i)*(buf[ilen+1+i]-'0');
    } 
    res=ires*scale_power(10,flen)+fres;
    if(mask&0X02)res=-res;         
    return res;
}     

//GPRMC
//void gprmc_parser(nmea_msg *gpsx,char *buf)
int gprmc_parser(nmea_msg *gpsx,uint8_t *buf)
{
    char *p1,*p2, *data_prt, dx;
    char posx;     
    uint32_t temp;      
    //float rs;
    double rs;
    uint8_t ret =1;
    
    // GPRMC , GNRMC ??�떆 泥섎??
    p2=(char*)strstr((const char *)buf,"GPRMC");
    p1=(char*)strstr((const char *)buf,"GNRMC");
    if(p1 == NULL && p2 == NULL) {
    	return 0;
    }

    data_prt = (char*)buf;
    p1 = data_prt;

    posx=comma_position(p1,1);
    if(posx!=0XFF)
    {
        temp=string_to_number(p1+posx,&dx);
        gpsx->utc.hour=temp/1000000;
        gpsx->utc.min=(temp/10000)%100;
        gpsx->utc.sec=(temp/100)%100;
     	gpsx->utc.milliSec =(temp/10)%10;
    }

    posx=comma_position(p1,3);
    if(posx!=0XFF)
    {
        temp=string_to_number(p1+posx,&dx);
        gpsx->latitude=temp/scale_power(10,dx+2);
        rs=temp%scale_power(10,dx+2);
        gpsx->latitude=gpsx->latitude*scale_power(10,7)+(rs*scale_power(10,7-dx))/60; //Should divide this with now 10^8.
    }
    posx=comma_position(p1,4);
    if(posx!=0XFF)gpsx->nshemi=*(p1+posx);
    posx=comma_position(p1,5);
    if(posx!=0XFF)
    {
        temp=string_to_number(p1+posx,&dx);
        gpsx->longitude=temp/scale_power(10,dx+2);
        rs=temp%scale_power(10,dx+2);
        gpsx->longitude=gpsx->longitude*scale_power(10,7)+(rs*scale_power(10,7-dx))/60; //Should divide this with now 10^8.
    }

    posx=comma_position(p1,6);
    if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);
    //speed
    posx = comma_position(p1, 7);
	if (posx != 0XFF)
	{
		gpsx->speed = string_to_number(p1 + posx, &dx);
	}

    posx=comma_position(p1,9);
    if(posx!=0XFF)
    {
        temp=string_to_number(p1+posx,&dx);
        gpsx->utc.date=temp/10000;
        gpsx->utc.month=(temp/100)%100;
        gpsx->utc.year=2000+temp%100;
    }
    return 1;
}

//GPGGA
void gpgga_parser(nmea_msg *gpsx,char *buf)
{
    char *p1,dx;           
    char posx;    
    p1=(char*)strstr((const char *)buf,"$GPGGA");
    posx=comma_position(p1,6);                      
    if(posx!=0XFF)gpsx->gpssta=string_to_number(p1+posx,&dx);   
    posx=comma_position(p1,7);                          
    if(posx!=0XFF)gpsx->posslnum=string_to_number(p1+posx,&dx); 
    posx=comma_position(p1,9);                          
    if(posx!=0XFF)gpsx->altitude=string_to_number(p1+posx,&dx);  
}


// gps parser code
int8_t getDelimeter(uint8_t *buf) {
    char *p1 = (char *)strstr((const char *)buf, "$GNRMC");
    if (p1 == NULL) return -1;
    return (uint8_t)p1 + 5;
}

int lengthOfCharPointer(char *p) {
    int ret = 0;
    while(*p != '\0') {
        *p++;
        ret++;
    }
    return ret;
}

char *xstrtok (char *lineSrc, char *delims)
{
    int cpLen = 0;
    static char *saveline = NULL;
    char line[200];

    if (lineSrc != NULL) {
        cpLen = lengthOfCharPointer(lineSrc);
        for(int i=0;i<cpLen;i++) {
            line[i] = *lineSrc++;
        }
        line[cpLen] = '\0';
        saveline = line;
    }

    char *p;
    int n;

    if (saveline == NULL || *saveline == '\0')
        return (NULL);

    n = strcspn (saveline, delims);
    p = saveline;

    saveline += n;

    if (*saveline != '\0')
        *saveline++ = '\0';

    return (p);
}

GPSStream parseGPS(char *data) {
    GPSStream ret;
    char buff[10];

    char *posx;
    posx = buff;

    posx = xstrtok(data,",");
    posx = xstrtok(NULL, ",");
    if(posx!=0xff)
    {

        double temp = atof(posx) * 10.0f;
        int temp2 = (int)temp;
        ret.time1 = (uint8_t)(temp2 / 10000);
        ret.time2 = (uint8_t)((temp2 / 100) % 100);
        ret.time3 = (uint8_t)(temp2 % 100);
        //USB_Printf("time\r\n");
        //USB_Printf("%d,%d,%d", ret.time1, ret.time2,ret.time3);
    }
    posx = xstrtok(NULL, ",");
    posx = xstrtok(NULL, ",");
    if(posx!=0xff)
    {

        double temp = atof(posx) * 100000.0f;
        ret.latitude=(uint32_t)temp;
    }
    posx = xstrtok(NULL, ",");
    if(posx!=0xff && *posx == 'N') {

        ret.latitude += 900000000;
    }
    posx = xstrtok(NULL, ",");
    if(posx!=0xff)
    {
        double temp = atof(posx) * 100000.0f;
        ret.longitude = (uint32_t)temp;
    }
    posx = xstrtok(NULL, ",");
    if(posx!=0xff && *posx == 'E') {

        ret.longitude += 1800000000;
    }

    posx = xstrtok(NULL, ",");
    if(posx!=0xff) {

        float temp = atof(posx) * 1000.0f;
        if ((int)temp > 65000) temp = 65000;
        ret.speed = (unsigned short int)temp;
    }

    posx = xstrtok(NULL, ",");
    if(posx!=0xff)
    {
        int temp=atoi(posx);
        int tag = 0;

        if (temp%100 > 33 || temp%100 < 19) ret.tag_year = 0;
        else ret.tag_year = (uint8_t)(tag * 16 + ((temp%100) - 18));

        ret.month = (uint8_t)((temp/100)%100);
        ret.day = (uint8_t)(temp/10000);
    }
    return ret;
}




uint16_t currPos,prePos;
uint8_t gpsShortBuf[80];
uint8_t preNandData[2048];
uint8_t compressed[2048+64];
uint16_t outIdx;
uint8_t overBuff[64];
GPSStream gs;



void compressGpsData(uint8_t *pNandData)
{
 uint8_t len, data, jj,kk,hh,printlen,overflow;
 uint16_t pageidx;
 static uint8_t doWork=0;

 static uint8_t addString=0;
 uint8_t addflag=0;


 for(pageidx =0; pageidx<2048; pageidx++)
 {
	  data = pNandData[pageidx];

	  if(data == 's')
	  {
		  addString = 1;
	  }


	  if(data == '$')
	 
		  currPos = pageidx;
		  if(currPos != prePos && doWork)
		  {
			  len = currPos - prePos;
			  printlen = len;
			  
			  if(currPos < prePos)
			  {
				  len = (2048 - prePos) + currPos;
				  printlen = len;
				  jj=0;
				  for(;prePos<2048;)
				  {
					  gpsShortBuf[jj] = preNandData[prePos];
					  prePos++;
					  jj++;
					  len--;
				  }

				  for(kk=0; kk<len; kk++)
				  {
					  gpsShortBuf[jj] = pNandData[kk];
					  jj++;
				  }
				  gs = parseGPS(&gpsShortBuf[0]);
				  memcpy(&compressed[outIdx], (unsigned char *)&gs,sizeof(GPSStream));
				  outIdx+=20;
			  }
			  else
			  {
				  memcpy(&gpsShortBuf[0],&pNandData[prePos],len);
				  gs = parseGPS(&gpsShortBuf[0]);
				  memcpy(&compressed[outIdx], (unsigned char *)&gs,sizeof(GPSStream));
				  outIdx+=20;

				  if(addString)
				  {
					  compressed[outIdx++]='s';
					  compressed[outIdx++]='t';
					  compressed[outIdx++]='r';
					  addString =0;
				  }
				  else
				  {
					  compressed[outIdx++]=0xff;
					  compressed[outIdx++]=0xff;
					  compressed[outIdx++]=0xff;
				  }

			  }

		}
		doWork =1;
		prePos = currPos;
		if(outIdx >= 2048)
		{

			overflow = outIdx - 2048;
			for(hh = 0; hh<overflow; hh++)
			{
				overBuff[hh] = compressed[2048+hh];
			}
			outIdx = 0;
			while(CDC_Transmit_FS((uint8_t *)&compressed[0],2048) != USBD_OK )
			{
				HAL_Delay(1);
			}

			for(hh=0; hh<overflow; hh++)
			{
				compressed[outIdx++] = overBuff[hh];
			}

		  }
		} 
 	 } 

 memcpy(&preNandData[0], &pNandData[0], 2048);
}



