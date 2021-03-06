/*
 * (C) Copyright 2003
 * Gerry Hamel, geh@ti.com, Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include "config.h"
#include "sys/sys.h"
#include "common/circbuf.h"
#include "ulp/debug.h"
#include <string.h>

int buf_init (circbuf_t * buf, int size)
{
	assert (buf != NULL);
	buf->size = 0;
	if(size >= 0) {
		buf->totalsize = size;
		buf->data = NULL;
		if(size > 0) {
			buf->data = (char *) sys_malloc (sizeof (char) * size);
			assert(buf -> data != NULL);
		}
	}

	buf->top = buf->data;
	buf->tail = buf->data;
	buf->end = buf->data + buf->totalsize;
	return 0;
}

int buf_free (circbuf_t * buf)
{
	assert (buf != NULL);
	if(buf -> data != NULL) {
		sys_free (buf->data);
	}
	memset (buf, 0, sizeof (circbuf_t));
	return 0;
}

int buf_pop (circbuf_t * buf, void *vpdest, int len)
{
	unsigned int i;
	char *p = buf->top;
	char *dest = vpdest;

	assert (buf != NULL);
	assert (dest != NULL);
	assert (p != NULL);

	/* Cap to number of bytes in buffer */
	if (len > buf->size)
		len = buf->size;

	for (i = 0; i < len; i++) {
		dest[i] = *p++;
		/* Bounds check. */
		if (p == buf->end) {
			p = buf->data;
		}
	}

	/* Update 'top' pointer */
	buf->top = p;
	buf->size -= len;

	return len;
}

int buf_push (circbuf_t * buf, const void *vpsrc, int len)
{
	/* NOTE:  this function allows push to overwrite old data. */
	unsigned int i;
	char *p = buf->tail;
	const char *src = vpsrc;

	assert (buf != NULL);
	assert (src != NULL);
	assert (p != NULL);

	for (i = 0; i < len; i++) {
		*p++ = src[i];
		if (p == buf->end) {
			p = buf->data;
		}
		/* Make sure pushing too much data just replaces old data */
		if (buf->size < buf->totalsize) {
			buf->size++;
		} else {
			buf->top++;
			if (buf->top == buf->end) {
				buf->top = buf->data;
			}
		}
	}

	/* Update 'tail' pointer */
	buf->tail = p;

	return len;
}
