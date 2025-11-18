#include <samples.h>
#include <stdint.h>
#define NUM_ELEMENTS 56398

const uint8_t sampleS3[NUM_ELEMENTS] = {
    #include "../ArregloS3.C"
};
const uint32_t SAMPLE_S3_LEN = NUM_ELEMENTS;
