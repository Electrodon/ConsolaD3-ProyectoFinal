#include <samples.h>
#include <stdint.h>
#define NUM_ELEMENTS 16
const uint8_t sampleS6[NUM_ELEMENTS] = {
    #include "../ArregloS6.C"
};
const uint32_t SAMPLE_S6_LEN = NUM_ELEMENTS;

