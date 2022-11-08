/*
 * data_buffer.c
 *
 *  Created on: Nov 7, 2022
 *      Author: mrmikeroot
 */

#include "data_buffer.h"

#include <string.h>

struct DATA_BUFFER
{
	uint8_t * buffers[3];
	size_t buffer_size;
	size_t bytes_in_buffer;
	size_t buf_idx;
	void (*buffers_full_cb)(void*, size_t);
};

DATA_BUFFER_HND_T data_buffer_create_new(size_t size, void (*buffer_full_cb)(void *, size_t))
{
	struct DATA_BUFFER * p_buffer_struct = malloc(sizeof(struct DATA_BUFFER));
	p_buffer_struct->buffers[0] = malloc(size * 3);
	p_buffer_struct->buffers[1] = p_buffer_struct->buffers[0] + size;
	p_buffer_struct->buffers[2] = p_buffer_struct->buffers[1] + size;
	p_buffer_struct->buffer_size = size;
	p_buffer_struct->bytes_in_buffer = 0;
	p_buffer_struct->buffers_full_cb = buffer_full_cb;
	p_buffer_struct->buf_idx = 0;

	return p_buffer_struct;
}

void * data_buffer_get_write_ptr(DATA_BUFFER_HND_T hnd)
{
	struct DATA_BUFFER * p_buffer_struct = hnd;

	return &p_buffer_struct->buffers[p_buffer_struct->buf_idx][p_buffer_struct->bytes_in_buffer];
}

void data_buffer_commit_bytes(DATA_BUFFER_HND_T hnd, size_t count)
{
	struct DATA_BUFFER * p_buffer_struct = hnd;

	p_buffer_struct->bytes_in_buffer += count;
	if (p_buffer_struct->bytes_in_buffer >= p_buffer_struct->buffer_size)
	{
		p_buffer_struct->buffers_full_cb(p_buffer_struct->buffers[p_buffer_struct->buf_idx], p_buffer_struct->buffer_size);

		p_buffer_struct->bytes_in_buffer -= p_buffer_struct->buffer_size;
		p_buffer_struct->buf_idx++;

		if (p_buffer_struct->buf_idx == 2)
		{
			p_buffer_struct->buf_idx = 0;
			memcpy(p_buffer_struct->buffers[0], p_buffer_struct->buffers[2], p_buffer_struct->bytes_in_buffer);
		}
	}
}
