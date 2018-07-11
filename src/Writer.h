/*
 * Writer.h
 *
 *  Created on: 10 Jul 2018
 *      Author: bryan
 */

#ifndef WRITER_H_
#define WRITER_H_

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

class Writer {
public:
	Writer();
	virtual ~Writer();
	int fd;
	struct termios oldtio, newtio;
	char buf[255];

	void write_byte(unsigned char);
	void write_frame(unsigned char, unsigned char , unsigned char*);
};

#endif /* WRITER_H_ */
