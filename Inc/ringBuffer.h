/*
 * ringBuffer.h
 *
 *  Created on: 4 mars 2019
 *      Author: erlin
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_


#include <stdint.h>


#define RING_BUFFER_LENGTH 100

typedef struct {
	uint8_t buf[RING_BUFFER_LENGTH];
#if RING_BUFFER_LENGTH < 255
	uint8_t head, tail;
#else
	uint16_t head, tail;
#endif
} RingBuffer;

typedef enum {
	RING_BUFFER_OK = 0x0,
	RING_BUFFER_FULL,
	RING_BUFFER_NO_SUFFICIENT_SPACE
} RingBuffer_Status;

uint16_t RingBuffer_GetDataLength(RingBuffer *buf);
uint16_t RingBuffer_GetFreeSpace(RingBuffer *buf);
void RingBuffer_Init(RingBuffer *buf);
uint16_t RingBuffer_Read(RingBuffer *buf, uint8_t *data, uint16_t len);
uint8_t RingBuffer_Write(RingBuffer *buf, uint8_t *data, uint16_t len);


#endif /* RINGBUFFER_H_ */
