/*
 * data_buffer.h
 *
 *  Created on: Nov 7, 2022
 *      Author: mrmikeroot
 */

#ifndef INC_DATA_BUFFER_H_
#define INC_DATA_BUFFER_H_

#include <stdlib.h>
#include <stdint.h>

typedef void * DATA_BUFFER_HND_T;

// creates a triple buffer
DATA_BUFFER_HND_T data_buffer_create_new(size_t size, void (*buffer_full_cb)(void *, size_t));

void * data_buffer_get_write_ptr(DATA_BUFFER_HND_T hnd);
void data_buffer_commit_bytes(DATA_BUFFER_HND_T hnd, size_t count);

#endif /* INC_DATA_BUFFER_H_ */
