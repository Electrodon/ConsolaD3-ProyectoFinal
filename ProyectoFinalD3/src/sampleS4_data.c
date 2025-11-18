#include <samples.h>
#include <stdint.h>
#define NUM_ELEMENTS 48836

const uint8_t sampleS4[NUM_ELEMENTS] = {
    #include "../ArregloS4.C"
};
const uint32_t SAMPLE_S4_LEN = NUM_ELEMENTS;
