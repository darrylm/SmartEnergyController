#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "secl9.h"
#include "sendtob70.h"

#define ZIGBEE_CHANNEL 0x01
#define Dbg2() printf("%s\r\n",msg)
#define BAD     255

const char emHexChars[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

//char doBase64Decode(char *inBuffer, char *outBuffer, char inDatalength);
#define nibbleToHexChar(val) (emHexChars[((val) & 0x0f)])

char tempByte = 0;
int Temperature=0;
int Light=0;
char Motion=0;
long timeToWait = 0;
char DebugEnabled=0;
char msg[256];
char EWAPIMsg[256];
int controller_pin=65535;
int EWAPIMessageAck=0;
int EWAPIMessageSent=0;

long BAUD_RATE = 19200;
//long Time;

long CLOSEST_SPBRG_VALUE = 0;
long BAUD_ACTUAL = 0;
long BAUD_ERROR = 0;
long BAUD_ERROR_PRECENT = 0;
long CLOSEST_UBRG_VALUE = 0;

#define MAX_CHANNELS 1000;
#define ZIGBEE_RX_BUFFER_SIZE 256
#define ZIGBEE_TX_BUFFER_SIZE 256
#define TEMP_BUFFER_SIZE 256
#define EWAPI_BUFFER_LENGTH 256

volatile char zigbeeRxBuffer[ZIGBEE_RX_BUFFER_SIZE] = {0};
volatile char zigbeeTxBuffer[ZIGBEE_TX_BUFFER_SIZE] = {0};
//char tempBuffer[TEMP_BUFFER_SIZE] = {0};	

volatile int zigbeeRxBufferReadPtr = 0;
volatile int zigbeeRxBufferWritePtr = 0;
volatile int zigbeeTxBufferReadPtr = 0;
volatile int zigbeeTxBufferWritePtr = 0;

char connected = 0;
char reconnect = 0;

union {
	char b[8];
	struct {
		unsigned long l;
		unsigned long h;
		} hl;
	unsigned long long ll;
	} gateway_eui;


char fastToggle = 0;
char ZigbeeRxBufferDataAvailable(void);

static char signbit;
static char IsPulseMeter;
static char MeterType=1; // set to a 5 way / 16 channel device
static union {
	unsigned long ul;
	char b[4];
	} Pulses;

char CRC8Table[256] = 
        {
            0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
            0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
            0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
            0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
            0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
            0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
            0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
            0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
            0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
            0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
            0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
            0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
            0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
            0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
            0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
            0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
            0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
            0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
            0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
            0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
            0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
            0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
            0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
            0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
            0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
            0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
            0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
            0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
            0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,         
            0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
            0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
            0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
        };

const char base64val[] = {
     BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD,
     BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD,
     BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD, BAD,BAD,BAD,BAD, BAD,62 ,BAD,BAD,
      52, 53, 54, 55,  56, 57, 58, 59,  60, 61,BAD,BAD, BAD,BAD,BAD,BAD,
     BAD,  0,  1,  2,   3,  4,  5,  6,   7,  8,  9, 10,  11, 12, 13, 14,
      15, 16, 17, 18,  19, 20, 21, 22,  23, 24, 25,BAD, BAD,BAD,BAD, 63,
     BAD, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51,BAD, BAD,BAD,BAD,BAD
 };
 #define DECODE64(c)  (isascii(c) ? base64val[c] : BAD)
#define isascii(x) 1
 
const char base64digits[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
 

 
char to64frombits(char *out, const char *in,int inlen)
 {
	 char count=0;
	 
         for (; inlen >= 3; inlen -= 3)
         {
                 *out++ = base64digits[in[0] >> 2];
                 *out++ = base64digits[((in[0] << 4) & 0x30) | (in[1] >> 4)];
                 *out++ = base64digits[((in[1] << 2) & 0x3c) | (in[2] >> 6)];
                 *out++ = base64digits[in[2] & 0x3f];
                 in += 3;
				 count += 4;
         }
 
         if (inlen > 0)
         {
                 char fragment;
 
                 *out++ = base64digits[in[0] >> 2];
                 fragment = (in[0] << 4) & 0x30;
 
                 if (inlen > 1)
                         fragment |= in[1] >> 4;
 
                 *out++ = base64digits[fragment];
                 *out++ = (inlen < 2) ? '=' : base64digits[(in[1] << 2) & 0x3c];
                 *out++ = '=';
				 count += 4;
         }
         
         *out = '\0';
		 return count;
 }
 



char doBase64Encode(char *inBuffer, char *outBuffer, char inDatalength)
{
  char outDataLength=0;

  outDataLength = to64frombits(outBuffer,inBuffer,inDatalength);

  return outDataLength;
}




 int from64tobits(char *out, char *in, char inlen)
 {
         int len = 0;
         char digit1, digit2, digit3, digit4;
 
         if (in[0] == '-' && in[1] == ' ')
                 in += 2;
         if (inlen == 0)
                 return(0);
 
         do {
                 digit1 = in[0];
                 if (DECODE64(digit1) == BAD)
                         return(0);
                 digit2 = in[1];
                 if (DECODE64(digit2) == BAD)
                         return(0);
                 digit3 = in[2];
                 if (digit3 != '=' && DECODE64(digit3) == BAD)
                         return(0);
                 digit4 = in[3];
                 if (digit4 != '=' && DECODE64(digit4) == BAD)
                         return(0);
                 in += 4;
				 inlen -= 4;
                 *out++ = (DECODE64(digit1) << 2) | (DECODE64(digit2) >> 4);
                 ++len;
                 if (digit3 != '=')
                 {
                         *out++ = ((DECODE64(digit2) << 4) & 0xf0) | (DECODE64(digit3) >> 2);
                         ++len;
                         if (digit4 != '=')
                         {
                                 *out++ = ((DECODE64(digit3) << 6) & 0xc0) | DECODE64(digit4);
                                 ++len;
                         }
                 }
         } while (*in && inlen && digit4 != '=');
 
         return (len);
} 



char asciiHexToHexConv(char char1, char char0)
{
  if(char1 >= 0x30 && char1 <= 0x39)
    char1 -= 0x30;
  else if(char1 >= 0x41 && char1 <= 0x46)
    char1 -= 0x37;
  else if(char1 >= 0x61 && char1 <= 0x66)
    char1 -= 0x57;
  else
    char1 = 0x00;
    
  if(char0 >= 0x30 && char0 <= 0x39)
    char0 -= 0x30;
  else if(char0 >= 0x41 && char0 <= 0x46)
    char0 -= 0x37;
  else if(char0 >= 0x61 && char0 <= 0x66)
    char0 -= 0x57;
  else
    char0 = 0x00;
 return((char1 << 4) | char0);
}


char doBase64Decode(char *inBuffer, char *outBuffer, char inDatalength)
{
  char outDataLength=0;
 int len=0;

  
  len = from64tobits(outBuffer,inBuffer,(int)inDatalength);
  if(len > 0){
	  outDataLength = len;
  }

  return outDataLength;
}
	
void DecodeCore(char *tempBuffer)
{
	static char count = 0, c;
	char Str[80];
	
	static unsigned int i,j, k;
	static unsigned long val;
	static char CC;
	//static char offset = 0;
	static char receptionStatus;
	static char decoded_length;
	static unsigned long address=256ul;
	static char done;

	static float fInput=344.9876; // Pick a number
	static long lWhole=0; // Stores digits left of decimal
	static unsigned long ulPart=0; // Stores digits right of decimal

	#define MULTIPLIER 1000

	static union {
		char b[8];
		struct {
			unsigned long l;
			unsigned long h;
			} hl;
		unsigned long long ll;
		} eui;
	static unsigned long energy;
	static union {
		unsigned long ul;
		float f;
		char b[4];
		} e2;
		
	static float power;
	static float power2;
	
	static char PartCode[2];
	
	static union {
		char b[4];
		float f;
		} mulfac;
	static char state;
	static unsigned long powerl;
	static union {
		char b[4];
		unsigned long ul;
		float f;
		} powerl2;
	static unsigned long long euil=0;
	static char channelb;
	static char found;
	static float seconds;
	char EWAPISequenceNumber=0;

	static char DecodedMsg[60];
	char ProcessEWAPI=0, NeedsEWAPIAck=0;
static union {
		long dt;
		unsigned long tick_get;
		char b[4];
		} dt;
static union {
	unsigned long ul;
	char b[4];
	} Vval;
char Vtype = ' ';
time_t RawTime;
struct tm *Time;

static long ReadyToReceiveTimeout =0ul;

time(&RawTime);
Time=localtime(&RawTime);
Time->tm_mon++;

count=strlen(tempBuffer);
while ((count>0) && ((tempBuffer[count] ==13) || (tempBuffer[count] ==10)))
	tempBuffer[count--]=0;

		//printf("Decoding %s\r\n",tempBuffer);
		// delete any junk at the start of the line.  All messages start with a 'W'
		while ((tempBuffer[0] != 'W') && (tempBuffer[0] != 0)) {
			for (i=0; i<69; i++) {
				tempBuffer[i]=tempBuffer[i+1];
				}
			tempBuffer[69]=0;
			}			
		
		if (DebugEnabled > 0) {
			msg[0]='D';
			msg[1]='B';
			msg[2]='G';
			msg[3]=':';
        	strncpy(&msg[4], &tempBuffer[0], 70);
			Dbg2();
			}	
					
// ======================================================================================================================				    
// ======================================================================================================================				    
// ======================================================================================================================				    

		//DebugEnabled = 1;
		//sprintf(msg, "Char0|1=|%u|%c| and |%u|%c",tempBuffer[0], tempBuffer[0], tempBuffer[1], tempBuffer[1]); Dbg2();

		if (((tempBuffer[0] != 'W') || (tempBuffer[1] != 'R') || (tempBuffer[2] != '=')) && (DebugEnabled == 1) ) { // this will be printed out later in th CDBG: section
			tempBuffer[count]  = 0;
        	strncpy(&msg[0], &tempBuffer[0], 70);
			Dbg2();
			}	

// ======================================================================================================================				    
// ======================================================================================================================				    
// ======================================================================================================================				    

		if(	(tempBuffer[0] == 'W') && (tempBuffer[1] == 'D') && (tempBuffer[2] == '=')) {
			tempBuffer[count]  = 0;

			//sprintf(msg, "DBG|%s|%c|%c|%c|", tempBuffer, tempBuffer[20],tempBuffer[21],tempBuffer[22]); Dbg2();
			if ((tempBuffer[3]=='P') && (tempBuffer[4]=='I') && (tempBuffer[5]=='N')) {
				j=7;
				i=tempBuffer[j]-'0';
				j = j + 1;
	
				i *= 10;
				k=tempBuffer[j]-'0';
				i = i + k;
				j = j + 1;
	
				i *= 10;
				k=tempBuffer[j]-'0';
				i = i + k;
				j = j + 1;
	
				i *= 10;
				k=tempBuffer[j]-'0';
				i = i + k;
				j = j + 1;
	
				i *= 10;
				k=tempBuffer[j]-'0';
				i = i + k;
				controller_pin =i; // used to report back to server
			//	sprintf(msg, "Got new PIN |%u|", controller_pin ); Dbg2();
				}	

			if ((tempBuffer[20]=='A') && (tempBuffer[21]=='c') && (tempBuffer[22]=='k')) {
				j=24;
				i=tempBuffer[j]-'0';
				j = j + 1;
	
				i *= 10;
				k=tempBuffer[j]-'0';
				i = i + k;
				EWAPIMessageAck =i; // used to report back to server
			//	sprintf(msg, "Got new PIN |%u|", controller_pin ); Dbg2();
			//	sprintf(msg, "Status:LastAck=%u", EWAPIMessageAck);	Dbg2();
				Vval.ul=EWAPIMessageAck; Vtype='j';
				//goto StoreEWAPICommand;
				}	
				
			if ((tempBuffer[20]=='S') && (tempBuffer[21]=='e') && (tempBuffer[22]=='n') && (tempBuffer[23]=='t')) {
				j=25;
				i=tempBuffer[j]-'0';
				j = j + 1;
	
				i *= 10;
				k=tempBuffer[j]-'0';
				i = i + k;
				EWAPIMessageSent =i; // used to report back to server
				sprintf(msg, "Status:LastSent=%u",EWAPIMessageSent); Dbg2();
			//	sprintf(msg, "Got new PIN |%u|", controller_pin ); Dbg2();
				Vval.ul=EWAPIMessageSent; Vtype='k';
				//goto StoreEWAPICommand;
				}	

			msg[0]='C';
			msg[1]='o';
			msg[2]='r';
			msg[3]='e';
			msg[4]=':';
						
		    strncpy(&msg[5], &tempBuffer[3],70);
		    for (i=0;i<strlen(msg);i++) 
				if ((msg[i]==13) || (msg[i]==10)) msg[i]=32;
			Dbg2();
			}

// ======================================================================================================================				    
// ======================================================================================================================				    
// ======================================================================================================================				    

	   	//BinaryToDate(SNTPGetUTCSeconds(),&Time);
		if(	(tempBuffer[0] == 'W') && (tempBuffer[1] == 'A') && (tempBuffer[2] == '=')) {
			
			// EWAPI ASCII message coming in
			
			// find end of string
			i=3;
			CC=0;
			while ((i <= TEMP_BUFFER_SIZE) && (tempBuffer[i] != '~')) { // starting at 3 skips the WA= part of the message
				//sprintf(msg,"Got at %u val=%u, |%c|%u|", i,tempBuffer[i],tempBuffer[i],CC); Dbg2();
				CC += tempBuffer[i];
				i++;
				}
			tempBuffer[TEMP_BUFFER_SIZE-31]=0; // just in case.  Remebering to leave room below for date time etc
			// tempBuffer[i] should now point to a :
			c=asciiHexToHexConv(tempBuffer[i+1],tempBuffer[i+2]);
			//c=((tempBuffer[i+1]-'0')*10) + tempBuffer[i+2] -'0';
			//sprintf(msg,"Got CC-PIC=%u, CC-ETRX2=%u |%c%c|", CC,c, tempBuffer[i+1],tempBuffer[i+2]); Dbg2();
			
			// Good old fashioned EWAPI message (yippee!) comes in here
			//sprintf(msg, "D1:S:%s", &tempBuffer[3]);	
			//Dbg2();
			if (CC != c) {
				tempBuffer[100]=0; // just in case the string is not terminated
				sprintf(msg,"*** Checksum fail"); Dbg2(); 
				return;
				}	
			
			tempBuffer[i+3]=0; // terminate the string AFTER the :
			
			k=1; // the message upload type

			j=0;
			while (j<(i)) {
				if ((tempBuffer[j] == ',') && (tempBuffer[j+1] == 'F') && (tempBuffer[j+2] == 'T')) {
					// FAST TWITCH
					k=6;
					}	
				j++;
				}	

			//sprintf(msg,"******** k = %u",k); Dbg2();		
			sprintf(msg,"D%u:W:%i/%i/%i/%i/%i/%i;S:%s",k,Time->tm_mday, Time->tm_mon, Time->tm_year+1900, Time->tm_hour, Time->tm_min, Time->tm_sec, &tempBuffer[3]);
			if (k==6) strncpy(EWAPIMsg, msg, EWAPI_BUFFER_LENGTH-1);
			sprintf(msg,"D%u:W:%i/%i/%i/%i/%i/%i;S:%s",k,Time->tm_mday, Time->tm_mon, Time->tm_year+1900, Time->tm_hour, Time->tm_min, Time->tm_sec, &tempBuffer[3]);
			StoreLine(msg);			
			}				    

// ======================================================================================================================				    
// ======================================================================================================================				    
// ======================================================================================================================				    

		if(	(tempBuffer[0] == 'W') && (tempBuffer[1] == 'R') && (tempBuffer[2] == '=') && (DebugEnabled == 1) ) {
			tempBuffer[count]  = 0;
			msg[0]='C';
			msg[1]='D';
			msg[2]='b';
			msg[3]='g';
			msg[4]=':';
			
	        strncpy(&msg[5], &tempBuffer[3], 70);
			Dbg2();
			}

		IsPulseMeter=0;
		if(	(tempBuffer[0] == 'W') && (tempBuffer[1] == 'F') && (tempBuffer[2] == '=')) {
			eui.b[7] = asciiHexToHexConv(tempBuffer[5],tempBuffer[6]);
			eui.b[6] = asciiHexToHexConv(tempBuffer[7],tempBuffer[8]);
			eui.b[5] = asciiHexToHexConv(tempBuffer[9],tempBuffer[10]);
			eui.b[4] = asciiHexToHexConv(tempBuffer[11],tempBuffer[12]);
			eui.b[3] = asciiHexToHexConv(tempBuffer[13],tempBuffer[14]);
			eui.b[2] = asciiHexToHexConv(tempBuffer[15],tempBuffer[16]);
			eui.b[1] = asciiHexToHexConv(tempBuffer[17],tempBuffer[18]);
			eui.b[0] = asciiHexToHexConv(tempBuffer[19],tempBuffer[20]);
			MeterType = asciiHexToHexConv(tempBuffer[24],tempBuffer[25]);
			Pulses.b[3] = asciiHexToHexConv(tempBuffer[29],tempBuffer[30]);
			Pulses.b[2] = asciiHexToHexConv(tempBuffer[31],tempBuffer[32]);
			Pulses.b[1] = asciiHexToHexConv(tempBuffer[33],tempBuffer[34]);
			Pulses.b[0] = asciiHexToHexConv(tempBuffer[35],tempBuffer[36]);
			sprintf(Str, "D3:S:%.8lX%.8lX:%u:%lu", eui.hl.h, eui.hl.l, MeterType, Pulses.ul); 
			StoreLine(Str);
			}


// ======================================================================================================================				    
// ======================================================================================================================				    
// ======================================================================================================================				    
					
		if(	(tempBuffer[0] == 'W') && (tempBuffer[1] == 'Z') && (tempBuffer[2] == '=')) {
//				if (screen_display == 5) {sprintf(msg, "Got Rx2"); Dbg2();}
            	decoded_length = doBase64Decode(&tempBuffer[3],&DecodedMsg[0],count);
            	//sprintf(msg, "DBG: Decode length1 = %u", decoded_length ); Dbg2();
				gateway_eui.b[0] = DecodedMsg[0];
				gateway_eui.b[1] = DecodedMsg[1];
				gateway_eui.b[2] = DecodedMsg[2];
				gateway_eui.b[3] = DecodedMsg[3];
				gateway_eui.b[4] = DecodedMsg[4];
				gateway_eui.b[5] = DecodedMsg[5];
				gateway_eui.b[6] = DecodedMsg[6];
				gateway_eui.b[7] = DecodedMsg[7];

			msg[0]='G';
			msg[1]='a';
			msg[2]='t';
			msg[3]='e';
			msg[4]='w';
			msg[5]='a';
			msg[6]='y';
			msg[7]=':';

	           	strncpy(&msg[8], &DecodedMsg[8],70); Dbg2();
				
				sprintf(msg, "Gateway:%.8lX%.8lX", gateway_eui.hl.h,  gateway_eui.hl.l); Dbg2();
			}

// ======================================================================================================================				    
// ======================================================================================================================				    
// ======================================================================================================================				    

		if((tempBuffer[0] == 'W') && (tempBuffer[1] == 'S') && (tempBuffer[2] == '=')) {  
	            //remove '\r'
	            char temp = 0;
	            //char spaceAvailable = 0;
	            //char tempData[6] = {0};
				char i;

		// calculate checksum
            CC = 0;
            for (i = 3; i <= count-4; i++)
            {
                j = tempBuffer[i];
                j = CC ^ j;
                j = j & 0xFF;
                CC = CRC8Table[j];
            }
            i = CC & 0x0F;
            if (i < 10) 
            	i = i + '0';
            else
            	i = i + 'A' - 10;
//			sprintf(msg, "Checksum 1=%.2X, %c", i,i); Dbg2();
            
            j=0;
			if (i != tempBuffer[count-2]) j++;
	
			i=CC /16;
            if (i < 10) 
            	i = i + '0';
            else
            	i = i + 'A' - 10;            
//			sprintf(msg, "Checksum 2=%.2X, %c", i,i); Dbg2();
			
			if (i != tempBuffer[count-3]) j++;

			if (j !=0) {
				sprintf(msg, "*** Checksum fail (2)"); Dbg2();
		     	count = 0;
				return;
				}	

//			sprintf(msg, "Checksum %.2X: tempBuffer[count-3]=%.2X, %c", CC, tempBuffer[count-3], tempBuffer[count-3]); Dbg2();
//			sprintf(msg, "Checksum %.2X: tempBuffer[count-2]=%.2X, %c", CC, tempBuffer[count-2], tempBuffer[count-2]); Dbg2();

		// ----------------------


				//if (screen_display == 5) {sprintf(msg, "Got Rx3"); Dbg2();}

	            count = count-1;


	            //Move received temp data <LL> + <MASTID> + <CH>  = 12 bytes towards MSBytes
	            for(temp=0; temp<count; temp++) {
		            char tempByte = 0x00;
		            tempByte = tempBuffer[temp+12+6];//save temporarily
	            	tempBuffer[temp+12+6] = tempBuffer[temp%18];
	            	tempBuffer[temp%18] = tempByte;//copy temporarily
	            }	

	            
	            
	            //<LL> <MASTID> <CH><TTTTTT><Payload> <CC><CR> 
	            //Re-format and send
	            
	            //Calculate <LL>
	            count = 8 + 2 + count+6; //mastid, ch, ,tttttt, payload
	            
				//Copy <LL>
	            tempBuffer[0] = nibbleToHexChar(count>>4);
	            tempBuffer[1] = nibbleToHexChar(count);
	            
	            //Copy <MASTID>
	            for(temp=0; temp<4; temp++)
	            {
//	            	tempBuffer[2+2*temp] = nibbleToHexChar(mastId[temp]>>4);
//	            	tempBuffer[2+2*temp+1] = nibbleToHexChar(mastId[temp]&0x0F);
	            	tempBuffer[2+2*temp] = nibbleToHexChar(0);
	            	tempBuffer[2+2*temp+1] = nibbleToHexChar(0);
	            }	
	            
	            //Copy <CH>
	            tempBuffer[10] = nibbleToHexChar(ZIGBEE_CHANNEL>>4);
	            tempBuffer[11] = nibbleToHexChar(ZIGBEE_CHANNEL);
	            	            
	            //<Payload> already copied

				// =================================================================================================

				//sprintf(msg, "here2"); Dbg2();
				//dt.tick_get = TickGet();

				decoded_length = doBase64Decode(&tempBuffer[21],&DecodedMsg[0],count);
/*
					if (DebugEnabled > 0) {
					sprintf(msg, "DBG: Byte 12 and 13 = %u %u", DecodedMsg[12], DecodedMsg[13] ); Dbg2();
					}	
*/
				eui.b[0] = DecodedMsg[0];
				eui.b[1] = DecodedMsg[1];
				eui.b[2] = DecodedMsg[2];
				eui.b[3] = DecodedMsg[3];
				eui.b[4] = DecodedMsg[4];
				eui.b[5] = DecodedMsg[5];
				eui.b[6] = DecodedMsg[6];
				eui.b[7] = DecodedMsg[7];
/*				sprintf(msg, "EUI: %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X ", 	
					eui.b[7],
					eui.b[6],
					eui.b[5],
					eui.b[4],
					eui.b[3],
					eui.b[2],
					eui.b[1],
					eui.b[0]
					); Dbg2();
				sprintf(msg, "Got EUI: %.8lX%.8lX", eui.hl.h,  eui.hl.l); Dbg2();
*/
//				sprintf(lcd, "EUI: %.8lX%.8lX", eui.hl.h,  eui.hl.l); 
//				dogl_display_str(0,48,&lcd[0]);

				#define DKM_MEASUREMENT_REPORT 1
				#define DKM_BUTTON_REPORT 3

				if (DecodedMsg[13] == DKM_BUTTON_REPORT) {
					sprintf(msg,"D4:W:%i/%i/%i/%i/%i/%i;S:%.8lX%.8lX:B=1:",Time->tm_mday, Time->tm_mon, Time->tm_year+1900, Time->tm_hour, Time->tm_min, Time->tm_sec, eui.hl.h,eui.hl.l);
					//sprintf(msg, "D4:S:%.8lX%.8lX:B=1:", eui.hl.h,  eui.hl.l); 
					StoreLine(msg);
					}	

				if (DecodedMsg[13] != DKM_MEASUREMENT_REPORT) {
					sprintf(msg,"D5:W:%i/%i/%i/%i/%i/%i;S:%.8lX%.8lX:",Time->tm_mday, Time->tm_mon, Time->tm_year+1900, Time->tm_hour, Time->tm_min, Time->tm_sec, eui.hl.h,eui.hl.l);
					//sprintf(msg, "D5:S:%.8lX%.8lX:", eui.hl.h,  eui.hl.l); 
					if (decoded_length < 20)
						j = decoded_length;
					else
						j=20;
					for (i=13;i<j; i++) {
						sprintf(&msg[strlen(msg)],"%.2X,",DecodedMsg[i]);
						}	
					StoreLine(msg);
					
/*					sprintf(msg, "EWAPI:%.8lX%.8lX:Z=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u:", eui.hl.h,  eui.hl.l, 
					DecodedMsg[8],
					DecodedMsg[9],
					DecodedMsg[10],
					DecodedMsg[11],
					DecodedMsg[12],
					DecodedMsg[13],
					DecodedMsg[14],
					DecodedMsg[15],
					DecodedMsg[16],
					DecodedMsg[17],
					DecodedMsg[18],
					DecodedMsg[19],
					DecodedMsg[20],
					DecodedMsg[21]
					); Dbg2();
*/					return;
					}	

				energy = DecodedMsg[19] * 255 * 255 * 255;
				energy += DecodedMsg[20] * 255 * 255;
				energy += DecodedMsg[21] * 255;
				energy += DecodedMsg[22];

				powerl = DecodedMsg[23];
				powerl *= 256;
				powerl += DecodedMsg[24];
				powerl *= 256;
				powerl += DecodedMsg[25];
				powerl *= 256;
				powerl += DecodedMsg[26];
				power = powerl;

				fInput=power;

				mulfac.b[3] = DecodedMsg[27];
				mulfac.b[2] = DecodedMsg[28];
				mulfac.b[1] = DecodedMsg[29];
				mulfac.b[0] = DecodedMsg[30];

			    signbit = mulfac.b[0] & 0x80;
     			mulfac.b[0] = mulfac.b[0] <<1;  // shift exponet over
     			mulfac.b[1] = (mulfac.b[1]& 0x7f) | signbit;


				power2=power;

				state = DecodedMsg[31];			
				channelb = DecodedMsg[32];
				channelb=1;
				
				if ((state >=10) && (state <= 25)) {
					
					if (DebugEnabled > 0) {
						fInput=power2;
						lWhole=(long)((float)fInput);
						ulPart=(long)((float)fInput*MULTIPLIER)-lWhole*MULTIPLIER;
						sprintf(msg,"*** Power: %li.%li",lWhole,ulPart); Dbg2();
	
						fInput=mulfac.f;
						lWhole=(long)((float)fInput);
						ulPart=(long)((float)fInput*MULTIPLIER)-lWhole*MULTIPLIER;
						sprintf(msg,"*** Mulfac: %li.%li",lWhole,ulPart); Dbg2();
						}	

					channelb=state-10+1; // this is the 16 channel cludge	
					power = sqrt(power2);
					power2=power / 1000.0;

					if (DebugEnabled > 0) {
						fInput=power;
						lWhole=(long)((float)fInput);
						ulPart=(long)((float)fInput*MULTIPLIER)-lWhole*MULTIPLIER;
						sprintf(msg,"*** PWRSQRT: %li.%li",lWhole,ulPart); Dbg2();
	
						fInput=power2;
						lWhole=(long)((float)fInput);
						ulPart=(long)((float)fInput*MULTIPLIER)-lWhole*MULTIPLIER;
						sprintf(msg,"*** Power2: %li.%li",lWhole,ulPart); Dbg2();
						}	

					}	
				else
					power2 = power2 * 1.8; // Don't ask me why the 1.8 is in the code.  The numpty who coded v1 of this hardcoded this into the Pro3 & Pro4 systems and we haven't been able to get rid of it since. Only affects 5 ways and plugins etc.


				power2 = mulfac.f * power2;
				
				if (DebugEnabled > 0) {
					fInput=power2;
					lWhole=(long)((float)fInput);
					ulPart=(long)((float)fInput*MULTIPLIER)-lWhole*MULTIPLIER;
					sprintf(msg,"Calibration3: %li.%li",lWhole,ulPart); Dbg2();
					}	

				if ((power2 > 1000000.0) || (power2 < 0.0)) {
					power2=0.0;
					sprintf(msg,"Power error (power > 1000000), resetting power to 0.0:"); Dbg2();
					}	
				if (state >=10) channelb=state-9;	
				
				powerl2.f = power2;
				fInput=power2;
				lWhole=(long)((float)fInput);
				ulPart=(long)((float)fInput*MULTIPLIER)-lWhole*MULTIPLIER;				
				sprintf(msg,"D2:W:%i/%i/%i/%i/%i/%i;S:%.8lX%.8lX:P=%li.%lu;C=%u;R=%u;",Time->tm_mday, Time->tm_mon, Time->tm_year+1900, Time->tm_hour, Time->tm_min, Time->tm_sec, eui.hl.h,  eui.hl.l, lWhole, ulPart, channelb, state);
				StoreLine(msg);
								
//				sprintf(msg,"2:W:%i/%i/%i/%i/%i/%i;S:%.8lX%.8lX:P:%li.%lu;C:%u;R:%u;",Time->tm_mday, Time->tm_mon, Time->tm_year+1900, Time->tm_hour, Time->tm_min, Time->tm_sec, eui.hl.h,  eui.hl.l, lWhole, ulPart, channelb, state);
				

//				sprintf(msg, "State %u",	state); Dbg2();
//				sprintf(msg, "Channel %u",	channelb); Dbg2();

//				sprintf(lcd,"Power: %li.%liW %u %u",lWhole,ulPart, state, channelb); 
//				dogl_display_str(0,56,&lcd[0]);

// ============================================================================

	} //'WS'
}

void str2ram(char *ptrRam, char *ptrRom)
{
	while( (*ptrRam++ = *ptrRom++) != '\0' )
		;
}	

void rom2ram(char *ptrRam, char *ptrRom, char length)
{
	char index = 0;
	for(index = 0; index<length; index++)
		*(ptrRam+index) = *(ptrRom+index);

}

char crc8(char *data_in, int number_of_bytes_to_read)
{
    char crc = 0;
    while(number_of_bytes_to_read > 0)
    {
//    	crc = crc8table[(crc ^ *data_in) & 0xFF];
    	crc = CRC8Table[(crc ^ *data_in) & 0xFF];
    	//sprintf(msg,"CRC8 %u = %u|%c|\r\n", i++, *data_in, *data_in); Dbg2();
    	data_in++;
    	number_of_bytes_to_read--;
    }
    //sprintf(msg,"CRC8 was %u\r\n", crc); Dbg2();
    return (crc);
}


