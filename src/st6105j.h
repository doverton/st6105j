/*
 * Copyright (c) 2009-2013, David Overton
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
#if !defined(__ST6105J_H_)
# define __ST6105J_H_

# include <termios.h>
# include <unistd.h>

struct st6105j {
	int fd;
	struct termios orig;
	const char *err;
	int timeout;
};

int st6105j_open(const char *name, struct st6105j *sensor);
const char *st6105j_get_error(struct st6105j *sensor);
int st6105j_identify(struct st6105j *sensor);
int st6105j_get_timeout(struct st6105j *sensor);
int st6105j_set_timeout(struct st6105j *sensor, int timeout);
int st6105j_check_status(struct st6105j *sensor);
int st6105j_get_temperature(struct st6105j *sensor, int *cels, int *dcel);
int st6105j_close(struct st6105j *sensor);

# define ST6105J_TIMEOUT 3
# define ST6105J_DELAY 500000

#endif

