/*
 * filter.c
 *
 *  Created on: Nov 1, 2024
 *      Author: rahul
 */
#include <stdint.h>
#include "filter.h"
#include "../../ltc2959/ltc2959.h"

// Structure to hold the moving average filter state
typedef struct {
    int32_t* buffer;        // Pointer to circular buffer
    int32_t  sum;          // Running sum
    uint16_t bufferSize;   // Size of circular buffer
    uint16_t count;        // Number of samples in buffer
    uint16_t position;     // Current position in buffer
    bool     initialized;  // Initialization status
} MovingAvgFilter_t;

// Initialize the moving average filter
bool MovingAvg_Init(MovingAvgFilter_t* filter, int32_t* buffer, uint16_t bufferSize) {
    if (!filter || !buffer || bufferSize == 0) {
        return false;
    }

    filter->buffer = buffer;
    filter->bufferSize = bufferSize;
    filter->sum = 0;
    filter->count = 0;
    filter->position = 0;
    filter->initialized = true;

    // Zero out the buffer
    for (uint16_t i = 0; i < bufferSize; i++) {
        buffer[i] = 0;
    }

    return true;
}

// Process new sample and return filtered value
int32_t MovingAvg_Process(MovingAvgFilter_t* filter, int32_t newSample) {
    // Check for valid initialization
    if (!filter || !filter->initialized) {
        return newSample;  // Return raw value if filter is invalid
    }

    // Protect against integer overflow in sum calculation
    filter->sum -= filter->buffer[filter->position];
    filter->sum += newSample;

    // Store new sample
    filter->buffer[filter->position] = newSample;

    // Update position
    filter->position++;
    if (filter->position >= filter->bufferSize) {
        filter->position = 0;
    }

    // Update count of samples
    if (filter->count < filter->bufferSize) {
        filter->count++;
    }

    // Calculate average, avoiding division by zero
    return (filter->count > 0) ? (filter->sum / filter->count) : newSample;
}

// Example usage with the existing Get_Current_Filtered function
#define FILTER_SIZE 20

int32_t Get_Current_Filtered(int32_t currentReading) {
    static int32_t filterBuffer[FILTER_SIZE] = {0};
    static MovingAvgFilter_t filter = {0};
    static bool isInitialized = false;

    // One-time initialization
    if (!isInitialized) {
        isInitialized = MovingAvg_Init(&filter, filterBuffer, FILTER_SIZE);
        if (!isInitialized) {
            return currentReading;  // Return raw value if initialization fails
        }
    }

    return MovingAvg_Process(&filter, currentReading);
}
