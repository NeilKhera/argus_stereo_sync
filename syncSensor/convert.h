#ifndef CONVERT_H
#define CONVERT_H

#include <cuda.h>

extern float convert(CUsurfObject surface, unsigned int width,
		    unsigned int height, uint8_t* oBuffer);

#endif // CONVERT_H
