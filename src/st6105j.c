/*
 * Copyright (c) 2009-2014, David Overton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include "st6105j.h"

static unsigned char st6105j_status[] = {
	0xc1, 0x0b, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x47, 0x98 /* CRC */
};

static unsigned char st6105j_ident[] = {
	0xc2, 0x0b, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0xb5
};

static unsigned char st6105j_temperature[] = {
	0xc5, 0x0c, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x01,      /* Variable index */
	0x0e, 0x49 /* CRC */
};

static int st6105j_put(struct st6105j *sensor, unsigned char *r, int size)
{
	int opins;

	usleep(ST6105J_DELAY);

	while (1) {
		if (write(sensor->fd, r, size) < 0) {
			if (errno == EAGAIN || errno == EINTR) {
				usleep(0);
				continue;
			}
			sensor->err = "Error writing request";
			return -1;
		}
		break;
	}

	ioctl(sensor->fd, TIOCMGET, &opins);

	return 0;
}

static int st6105j_get(struct st6105j *sensor, unsigned char *b, int size)
{
	int offset = 0;
	int r;
	struct timeval start;
	struct timeval now;

	if (gettimeofday(&start, NULL) < 0) {
		sensor->err = "Cannot get current time";
		return -1;
	}

	while (offset < size) {
		if ((r = read(sensor->fd, b + offset, size - offset)) < 0) {
			if (errno == EAGAIN || errno == EINTR) {
				if (gettimeofday(&now, NULL) < 0) {
					sensor->err = "Cannot get current time";
					return -1;
				}
				if (now.tv_sec - start.tv_sec > sensor->timeout) {
					sensor->err = "Sensor is not responding";
					return -1;
				}
				usleep(0);
				continue;
			} else {
				sensor->err = "Error reading response";
				return -1;
			}
		}
		offset += r;
	}

	return 0;
}

int st6105j_identify(struct st6105j *sensor)
{
	unsigned char ident[64];

	if (st6105j_put(sensor, st6105j_ident, sizeof(st6105j_ident)) < 0)
		return -1;
	if (st6105j_get(sensor, ident, sizeof(ident)) < 0)
		return -1;

	return 0;
}

int st6105j_check_status(struct st6105j *sensor)
{
	unsigned char status[6];

	if (st6105j_put(sensor, st6105j_status, sizeof(st6105j_status)) < 0)
		return -1;
	if (st6105j_get(sensor, status, sizeof(status)) < 0)
		return -1;

	if ((status[0] != 0x94 && status[0] != 0x90) || status[1] != 0x06) {
		sensor->err = "Invalid status from sensor";
		errno = EINVAL;
		return 1;
	}

	return 0;
}

int st6105j_get_temperature(struct st6105j *sensor, int *cels, int *dcel)
{
	unsigned char temp[7];

	if (st6105j_check_status(sensor) != 0)
		return -1;
	if (st6105j_put(sensor, st6105j_temperature, sizeof(st6105j_temperature)) < 0)
		return -1;
	if (st6105j_get(sensor, temp, sizeof(temp)) < 0)
		return -1;

	if (temp[0] != 0x90 || temp[1] != 0x07) {
		sensor->err = "Invalid temperature reading from sensor";
		errno = EINVAL;
		return 1;
	}

	*cels = temp[3] & 0xff;
	if (temp[4] != 0x00)
		*cels -= 256;
	*dcel = (*cels & 0x01) ? 5 : 0;
	*cels >>= 1;

	return 0;
}

int st6105j_close(struct st6105j *sensor)
{
	int rc = 0;

	if (tcsetattr(sensor->fd, TCSANOW, &sensor->orig) < 0) {
		sensor->err = "Failed to restore device attributes";
		rc = -1;
	}

	close(sensor->fd);
	sensor->fd = -1;

	return rc;
}

const char *st6105j_get_error(struct st6105j *sensor)
{
	const char *err = sensor->err;
	sensor->err = NULL;
	return err;
}

int st6105j_get_timeout(struct st6105j *sensor)
{
	return sensor->timeout;
}

int st6105j_set_timeout(struct st6105j *sensor, int timeout)
{
	if (timeout < 0) {
		sensor->err = "Invalid timeout";
		errno = EINVAL;
		return -1;
	}

	sensor->timeout = timeout;
	return 0;
}


int st6105j_open(const char *name, struct st6105j *sensor)
{
	struct termios tio;
	int flags;
	int opins;
	int done = 0;

	sensor->err = NULL;
	sensor->timeout = ST6105J_TIMEOUT;

	if ((sensor->fd = open(name, O_RDWR)) < 0) {
		sensor->err = name;
		return -1;
	}

	do {
		if (!isatty(sensor->fd)) {
			sensor->err = "Not a terminal device";
			break;
		}
		if (ioctl(sensor->fd, TIOCMGET, &opins) < 0) {
			sensor->err = "Failed to get status lines";
			break;
		}
		if (tcgetattr(sensor->fd, &sensor->orig) < 0) {
			sensor->err = "Failed to read device attributes";
			break;
		}
		if (tcflush(sensor->fd, TCIOFLUSH) < 0) {
			sensor->err = "Failed to flush device";
			break;
		}

		memcpy(&tio, &sensor->orig, sizeof(struct termios));

		cfmakeraw(&tio);
		if (cfsetspeed(&tio, B1200) < 0) {
			sensor->err = "Failed to set baud rate";
			break;
		}

		tio.c_cflag &= ~(PARENB | PARODD); /* No parity */
		tio.c_cflag |= CS8; /* 8 data bits */
		tio.c_cflag &= ~(CSTOPB); /* 1 Stop bit */

		tio.c_cc[VEOF] = 0x1a;
		tio.c_cc[VSTART] = 0x11;
		tio.c_cc[VSTOP] = 0x13;
		tio.c_cc[VMIN] = 1;
		tio.c_cc[VTIME] = 0;

		if (tcsetattr(sensor->fd, TCSANOW, &tio) < 0) {
			sensor->err = "Failed to set device attributes";
			break;
		}

		opins |= TIOCM_DTR;
		opins |= TIOCM_RTS;

		if (ioctl(sensor->fd, TIOCMSET, &opins) < 0) {
			sensor->err = "Failed to set status lines";
			break;
		}
		if ((flags = fcntl(sensor->fd, F_GETFL, 0)) < 0) {
			sensor->err = "Cannot get descriptor flags";
			break;
		}
		if (fcntl(sensor->fd, F_SETFL, flags | O_NONBLOCK) < 0) {
			sensor->err = "Cannot set non-blocking mode";
			break;
		}

		done = 1;
	} while (0);

	if (!done) {
		int e = errno;
		close(sensor->fd);
		sensor->fd = -1;
		errno = e;
		return -1;
	}

	return 0;
}

