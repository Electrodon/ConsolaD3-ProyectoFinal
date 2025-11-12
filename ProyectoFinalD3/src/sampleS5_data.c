#include <samples.h>
#include <stdint.h>
#define NUM_ELEMENTS 31956
const uint8_t sampleS5[NUM_ELEMENTS] = {
    #include "../ArregloS5.C"
};
const uint32_t SAMPLE_S5_LEN = NUM_ELEMENTS;
