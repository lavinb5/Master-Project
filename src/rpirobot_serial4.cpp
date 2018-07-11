#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include<pthread.h>
#include<iostream>
using namespace std;

#include "PathPlanner.h"

#define BUADRATE B9600
#define MODEMDEVICE "/dev/ttyAMA0"

PathPlanner path1;

int fd, res;
struct termios oldtio, newtio;
char buf[255];

fd_set set;
struct timeval timeout;
int rv;

int serial_reader()
{
	FD_ZERO(&set);
	FD_SET(fd, &set);
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	rv = select(fd + 1, &set, NULL, NULL, &timeout);
	if(rv == -1) {}
	else if(rv == 0){}
	else
	{
		res = read(fd, buf,255);
		buf[res] = 0;
	}
	return buf[0];
}

void *thread(void *ptr)
{

	int type = (int) ptr;
	fprintf(stderr,"Thread - %d\n",type);

	fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd<0) { perror(MODEMDEVICE); }
	else{ printf("port open\n"); }

	tcgetattr(fd, &oldtio);
	bzero(&newtio, sizeof(newtio));

	//newtio.c_cflag = BUADRATE | CRTSCTS | CS8 | CREAD;
	newtio.c_cflag = BUADRATE | CS8 | CREAD | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	while(1){
		if(serial_reader() == 1)
		{
			printf("Frame received\n");
			unsigned char frame_type = serial_reader();
			unsigned char pay_len = serial_reader();
			unsigned char pay[255];
			for(int i=0; i< pay_len; i++)
			{
				unsigned char payload = serial_reader();
				pay[i] = payload;
			}
			if(serial_reader() == (1 + pay_len) )
			{
				printf("Frame ok: Frame Type = %c\n", frame_type);
				path1.received_frame(frame_type, (int)pay_len, pay);
			}
			else printf("Frame error: Checksum\n");

		}
		buf[0] = 0; // clear the buffer
	}

	tcsetattr(fd, TCSANOW, &oldtio);
	return  ptr;
}

int main()
{
	pthread_t thread1;
	int thr = 1;
	pthread_create(&thread1, NULL, *thread, (void *) thr);

	pthread_join(thread1,NULL);
	return 0;
}
