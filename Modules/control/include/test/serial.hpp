//串口读取库函数（用于读取超声波数据）
//主要功能：读取串口数据
//输入：需要先设置波特率、端口号等
//输出：串口数据

#ifndef _SERIAL_H
#define _SERIAL_H


#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>

using namespace std;

class Serial{
public: 
	Serial();
	~Serial();
public:
	static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
	static int open_portUSB(int comport);
        static int open_port_ultrasonic();
        static int nwrite(int serialfd, unsigned char *data, int datalength);
        static int nread(int fd,  unsigned char *data, int datalength);

};


Serial::Serial( )  {  
}  
  
Serial::~Serial()  {   
}  
 

int Serial::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
    struct termios newtio,oldtio;  
    if  ( tcgetattr( fd,&oldtio)  !=  0) {   
        perror("SetupSerial 1");  
        return -1;  
    }  
    bzero(&newtio, sizeof(newtio));  
    newtio.c_cflag  |=  CLOCAL | CREAD;   
    newtio.c_cflag &= ~CSIZE;   
 
    switch( nBits )  
    {  
    case 7:  
        newtio.c_cflag |= CS7;  
        break;  
    case 8:  
        newtio.c_cflag |= CS8;  
        break;  
    }  
  
    switch( nEvent )  
    {  
    case 'O':  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag |= PARODD;  
        newtio.c_iflag |= (INPCK | ISTRIP);  
        break;  
    case 'E':   
        newtio.c_iflag |= (INPCK | ISTRIP);  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag &= ~PARODD;  
        break;  
    case 'N':    
        newtio.c_cflag &= ~PARENB;  
        break;  
    }  
  
    switch( nSpeed )  
    {  
    case 2400:  
        cfsetispeed(&newtio, B2400);  
        cfsetospeed(&newtio, B2400);  
        break;  
    case 4800:  
        cfsetispeed(&newtio, B4800);  
        cfsetospeed(&newtio, B4800);  
        break;  
    case 9600:  
        cfsetispeed(&newtio, B9600);  
        cfsetospeed(&newtio, B9600);  
        break;  
    case 115200:  
        cfsetispeed(&newtio, B115200);  
        cfsetospeed(&newtio, B115200);  
        break;  
    default:  
        cfsetispeed(&newtio, B9600);  
        cfsetospeed(&newtio, B9600);  
        break;  
    }  
    if( nStop == 1 )  
        newtio.c_cflag &=  ~CSTOPB;  
    else if ( nStop == 2 )  
    newtio.c_cflag |=  CSTOPB;  
    newtio.c_cc[VTIME]  = 0;  
    newtio.c_cc[VMIN] = 0;  
    tcflush(fd,TCIFLUSH);  
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
    {  
        perror("com set error");  
        return -1;  
    }  
    return 0;  
}  
  
int Serial::open_portUSB(int comport)                 //通过参数，打开相应的串口  
{  
    string serial_port_adr("/dev/ttyUSB"+ to_string(comport));
    cout << serial_port_adr << endl;
    int fd = open( serial_port_adr.c_str() , O_RDWR|O_NOCTTY);   
    if (-1 == fd)
    {  
        perror("Can't Open Serial Port!!");  
    }
    return fd;
}  


int Serial::open_port_ultrasonic()                //通过参数，打开相应的串口
{
    string serial_port_adr("/dev/ultrasonic");
    cout << serial_port_adr << endl;
    int fd = open( serial_port_adr.c_str() , O_RDWR|O_NOCTTY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port!!");
    }
    return fd;
}


int Serial::nwrite (int serialfd,unsigned char *data, int datalength )
{  
    return write(serialfd, data, datalength);     
}  
  
int Serial::nread(int fd, unsigned char *data,int datalength)   {
    int readlen = read(fd,data,datalength);  
    cout << "readlen: " << readlen<<endl;
    if( readlen <= 0 )  
    {  
        printf("read  error!!");  
    }  
    return readlen;  
}  

#endif
