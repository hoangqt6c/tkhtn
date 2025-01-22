/*
 * queue.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Hoang
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_


#define QUEUE_SIZE 10

// Define the Queue structure
typedef struct {
    int data[QUEUE_SIZE];  // Array to hold the queue elements
    int front;             // Index of the front element
    int rear;              // Index of the rear element
    int size;              // Current size of the queue
} Queue;

// Function declarations
void initQueue(Queue *q);
int isFull(Queue *q);
int isEmpty(Queue *q);
void push(Queue *q, int value);
int pop(Queue *q);
void printQueue(Queue *q);
#endif /* INC_QUEUE_H_ */
