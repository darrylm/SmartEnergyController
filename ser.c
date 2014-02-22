	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <unistd.h>
	#include <time.h>
	#include "secl9.h"
	#include "WiMesh.h"
	#include "mac.h"
	#include "lcd.h"

	char MAC[20];
	char IP[30];
	int fd;
	int OldSeconds=0;
  
void DecodeCore(char *tempBuffer);

  /* baudrate settings are defined in <asm/termbits.h>, which is
  included by <termios.h> */
  #define BAUDRATE B115200
  /* change this definition for the correct port */
  #define MODEMDEVICE "/dev/ttyAMA0"
  #define _POSIX_SOURCE 1 /* POSIX compliant source */

  #define FALSE 0
  #define TRUE 1

  volatile int STOP=FALSE;

 // ============================================================
 // ============================================================
 // ============================================================
  
void DisplayTime(void)
{
	char TimeStr[100];
	char msg[100];
	time_t RawTime;
	struct tm *Time;
	
	time(&RawTime);
	Time=localtime(&RawTime);
	Time->tm_mon++;
	
	if (OldSeconds != Time->tm_sec) {
		sprintf(TimeStr,"%02i:%02i:%02i",Time->tm_hour, Time->tm_min,Time->tm_sec);
		LcdPos(2,8);
		LcdStr(TimeStr);
		OldSeconds = Time->tm_sec;
		}
}
  
// ============================================================
// ============================================================
// ============================================================

void CheckMAC(void)
{

if  (
	(strcmp(MAC,"B827EB283B6B")  != 0)  &&
	(strcmp(MAC,"801F0260FD56")   !=0)
	) {
	LcdCls();
	LcdPos(2,0);
	LcdStr("Please contact");
	sleep(2);
	LcdPos(2,0);
	LcdStr("www.Enistic.com");
	sleep(2);
	LcdPos(2,0);
	LcdStr("No network     ");
	printf("Please contact www.Enistic.com and quote reference 04318\r\n");
	sleep(7);
//	system("init 0");
	exit(1);
	}

} 
 
 // ============================================================
 // ============================================================
 // ============================================================
 
  
int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// ============================================================
 // ============================================================
 // ============================================================
 
  main()
  {
    int c, res;
    struct termios oldtio,newtio;
    char buf[255];
    char WiMesh[20];
    int key,i,j;
    char Line[500];
    char DecodedLine[300];
//	time_t RawTime;
//	struct tm *Time;

    
    GetMac(MAC);
    printf("MAC address is |%s|\r\n",MAC);
    GetIPAddress(IP);
    printf("IP address is |%s|\r\n",IP);

    printf("o=EWAPI timeout=2 minutes\r\n");
    printf("p=EWAPI timeout=20 minutes\r\n");
    printf("5=Reboot\r\n");
    printf("6=set channel 15\r\n");
    printf("7=set channel 20\r\n");
    printf("8=set channel 25\r\n");
    printf("9=set channel 26\r\n");
    printf("0=set channel to auto\r\n");
	
//	time(&RawTime);
//	Time=localtime(&RawTime);
//	Time->tm_mon++;
			    
  /*
    Open modem device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */
   fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_ASYNC  );
//   fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_ASYNC | O_NONBLOCK );
//   fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
   if (fd <0) {perror(MODEMDEVICE); return(-1); }

   tcgetattr(fd,&oldtio); /* save current serial port settings */
   bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

  /*
    BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
    CRTSCTS : output hardware flow control (only used if the cable has
              all necessary lines. See sect. 7 of Serial-HOWTO)
    CS8     : 8n1 (8bit,no parity,1 stopbit)
    CLOCAL  : local connection, no modem contol
    CREAD   : enable receiving characters
  */
   newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
//   newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

  /*
    IGNPAR  : ignore bytes with parity errors
    ICRNL   : map CR to NL (otherwise a CR input on the other computer
              will not terminate input)
    otherwise make device raw (no other input processing)
  */
   newtio.c_iflag = IGNPAR | ICRNL;

  /*
   Raw output.
  */
   newtio.c_oflag = 0;

  /*
    ICANON  : enable canonical input
    disable all echo functionality, and don't send signals to calling program
  */
   newtio.c_lflag = ICANON;

  /*
    initialize all control characters
    default values can be found in /usr/include/termios.h, and are given
    in the comments, but we don't need them here
  */
  
/*  
   newtio.c_cc[VINTR]    = 0;     // Ctrl-c 
   newtio.c_cc[VQUIT]    = 0;     // Ctrl-\ 
   newtio.c_cc[VERASE]   = 0;     // del 
   newtio.c_cc[VKILL]    = 0;     // @ 
   newtio.c_cc[VEOF]     = 4;     // Ctrl-d 
   newtio.c_cc[VTIME]    = 0;     // inter-character timer unused 
   newtio.c_cc[VMIN]     = 1;     // blocking read until 1 character arrives 
   newtio.c_cc[VSWTC]    = 0;     // '\0' 
   newtio.c_cc[VSTART]   = 0;     // Ctrl-q 
   newtio.c_cc[VSTOP]    = 0;     // Ctrl-s 
   newtio.c_cc[VSUSP]    = 0;     // Ctrl-z 
   newtio.c_cc[VEOL]     = 0;     // '\0' 
   newtio.c_cc[VREPRINT] = 0;     // Ctrl-r 
   newtio.c_cc[VDISCARD] = 0;     // Ctrl-u 
   newtio.c_cc[VWERASE]  = 0;     // Ctrl-w 
   newtio.c_cc[VLNEXT]   = 0;     // Ctrl-v 
   newtio.c_cc[VEOL2]    = 0;     // '\0' 
*/
  /*
    now clean the modem line and activate the settings for the port
  */
   tcflush(fd, TCIFLUSH);
   tcsetattr(fd,TCSANOW,&newtio);

  /*
    terminal settings done, now handle input
    In this example, inputting a 'z' at the beginning of a line will
    exit the program.
  */
	LcdInit();
	LcdCls();
	
	LcdPos(0,0);
	LcdStr("Enistic SEC");
	LcdPos(1,0);
	LcdStr("V1.0");
	sleep(3);

	LcdPos(0,0);
	LcdStr(IP);
	LcdPos(1,0);
	LcdStr(MAC);
	LcdPos(2,0);
	LcdStr("                ");
	
	CheckMAC();

   while (STOP==FALSE) {     // loop until we have a terminating condition 
   
	DisplayTime();
   
	key = getkey();
	//printf("Key was %u\r\n",key);
	if (key == '1') {
	   WiMesh[7]=0x00;
	   WiMesh[6]=0x0D;
	   WiMesh[5]=0x6F;
	   WiMesh[4]=0x00;
	   WiMesh[3]=0x00;
	   WiMesh[2]=0x26;
	   WiMesh[1]=0xB1;
	   WiMesh[0]=0xC5;
	   
	   WiMesh[8]=0x77;
	   WiMesh[9]=0x02;
	   WiMesh[10]=0x04;
	   WiMesh[11]=0x01;
	   WiMesh[12]=0x00; // 0=off, 1=no
	   WiMesh[13]=0x0f;
	   WiMesh[14]=0xbe;
	   DoMessage(WiMesh,15,'S',fd);
	}

	if (key == 'o') {
		WiMesh[0]='W';
		WiMesh[1]='E';
		WiMesh[2]='=';
		WiMesh[3]='0';
		WiMesh[4]='2';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}
	if (key == 'p') {
		WiMesh[0]='W';
		WiMesh[1]='E';
		WiMesh[2]='=';
		WiMesh[3]='2';
		WiMesh[4]='0';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}
	if (key == '5') {
		WiMesh[0]='W';
		WiMesh[1]='C';
		WiMesh[2]='=';
		WiMesh[3]='1';
		WiMesh[4]='5';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}
	if (key == '6') {
		WiMesh[0]='W';
		WiMesh[1]='B';
		WiMesh[2]='=';
		WiMesh[3]='1';
		WiMesh[4]='5';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}

	if (key == '7') {
		WiMesh[0]='W';
		WiMesh[1]='B';
		WiMesh[2]='=';
		WiMesh[3]='2';
		WiMesh[4]='0';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}

	if (key == '8') {
		WiMesh[0]='W';
		WiMesh[1]='B';
		WiMesh[2]='=';
		WiMesh[3]='2';
		WiMesh[4]='5';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}

	if (key == '9') {
		WiMesh[0]='W';
		WiMesh[1]='B';
		WiMesh[2]='=';
		WiMesh[3]='2';
		WiMesh[4]='6';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}

	if (key == '0') {
		WiMesh[0]='W';
		WiMesh[1]='B';
		WiMesh[2]='=';
		WiMesh[3]='0';
		WiMesh[4]='0';
		WiMesh[5]=13;
		WiMesh[6]=10;
		WiMesh[7]=' ';
		write(fd,WiMesh,7);
		}

	if (key == '2') {
	   WiMesh[7]=0x00;
	   WiMesh[6]=0x0D;
	   WiMesh[5]=0x6F;
	   WiMesh[4]=0x00;
	   WiMesh[3]=0x00;
	   WiMesh[2]=0x26;
	   WiMesh[1]=0xB1;
	   WiMesh[0]=0xC5;
	   
	   WiMesh[8]=0x77;
	   WiMesh[9]=0x02;
	   WiMesh[10]=0x04;
	   WiMesh[11]=0x01;
	   WiMesh[12]=0x01; // 0=off, 1=no
	   WiMesh[13]=0x0f;
	   WiMesh[14]=0xbe;
	   DoMessage(WiMesh,15,'S',fd);
	}

	if (key == '3') { //000D6F0000244D32
		sprintf(WiMesh,"WA000D6F0000244D32REL=2221-:\r\n");
		printf("Sending EWAPI %s",WiMesh);
		write(fd,WiMesh,strlen(WiMesh)+1);
		}
	
	if (key == '4') { //000D6F0000244D32
		sprintf(WiMesh,"WA000D6F0000244D32REL=2220-:\r\n");
		printf("Sending EWAPI %s",WiMesh);
		write(fd,WiMesh,strlen(WiMesh)+1);
		}
		
	res = read(fd,buf,255);
	buf[res]=0;             // set end of string, so we can printf 
	if (buf[0] == 'W') {
		//printf("%s\r\n",buf);
		DecodeCore(buf);
		}

   
   /* restore the old port settings */
   tcsetattr(fd,TCSANOW,&oldtio);
  }
}

// ============================================================
// ============================================================
// ============================================================
 
