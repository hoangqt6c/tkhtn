#include <stdio.h>
#include "queue.h"

// Initialize the queue
void initQueue(Queue *q) {
    q->front = 0;
    q->rear = -1;
    q->size = 0;
}

// Check if the queue is full
int isFull(Queue *q) {
    return q->size == QUEUE_SIZE;
}

// Check if the queue is empty
int isEmpty(Queue *q) {
    return q->size == 0;
}

// Push an element into the queue (0 or 1)
void push(Queue *q, int value) {
    if (isFull(q)) {
        printf("Queue is full, cannot push element.\n");
        return;
    }
    q->rear = (q->rear + 1) % QUEUE_SIZE;  // Circular increment
    q->data[q->rear] = value;               // Insert the element
    q->size++;                              // Increase the size of the queue
    printf("Pushed %d into the queue.\n", value);
}

// Pop an element from the queue
int pop(Queue *q) {
    if (isEmpty(q)) {
        printf("Queue is empty, cannot pop element.\n");
        return -1;  // Return -1 to indicate an error (empty queue)
    }
    int poppedValue = q->data[q->front];   // Get the front element
    q->front = (q->front + 1) % QUEUE_SIZE; // Circular increment
    q->size--;                             // Decrease the size of the queue
    return poppedValue;
}

// Print the current state of the queue (for debugging)
void printQueue(Queue *q) {
    printf("Queue: ");
    for (int i = 0; i < q->size; i++) {
        int index = (q->front + i) % QUEUE_SIZE;
        printf("%d ", q->data[index]);
    }
    printf("\n");
}
