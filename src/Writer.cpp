/*
 * Writer.cpp
 *
 *  Created on: 10 Jul 2018
 *      Author: bryan
 */

#include<iostream>
using namespace std;

#include "Writer.h"

#define BUADRATE B9600
#define MODEMDEVICE "/dev/ttyAMA0"

Writer::Writer() {
	// TODO Auto-generated constructor stub
	fd=open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0 )perror("open_port: Unable to open /dev/ttyAMA0 – ");
	else printf("Port 1 has been sucessfully opened\n");

	struct termios oldtio, newtio;
	tcgetattr(fd, &oldtio);
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = BUADRATE| CRTSCTS | CS8 | CLOCAL | CREAD;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

}

Writer::~Writer() {
	// TODO Auto-generated destructor stub
}

void Writer::write_byte(unsigned char byte) {
	//cout << "Writer: Sending byte" << endl;
	usleep(50000);
	write(fd, &byte, 1);
}

void Writer::write_frame(unsigned char frame_type, unsigned char pay_len, unsigned char *pay) {
	unsigned char delimiter = 1;
	cout << "Writer: Payload length = " << (int)pay_len << endl;
	write_byte(delimiter);
	write_byte(frame_type);
	write_byte(pay_len);
	for(int i=0; i<(int)pay_len; i++)
	{
		write_byte(pay[i]);
	}
	write_byte(pay_len+delimiter);
}
