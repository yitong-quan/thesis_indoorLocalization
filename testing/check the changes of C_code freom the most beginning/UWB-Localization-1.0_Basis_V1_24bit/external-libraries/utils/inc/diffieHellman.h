#ifndef DIFFIEHELLMAN_H_
#define DIFFIEHELLMAN_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//public variable representing the shared secret key.
uint32_t k;
//prime number
const uint32_t prime = 2147483647;
//generator
const uint32_t generator = 16807;

uint32_t mul_mod(uint32_t a, uint32_t b, uint32_t m);
uint32_t pow_mod(uint32_t b, uint32_t e, uint32_t m);
uint32_t diffie_random(uint32_t howsmall, uint32_t howbig);
int bitRead(uint32_t b, int bitPos);


#endif /* DIFFIEHELLMAN_H_ */
