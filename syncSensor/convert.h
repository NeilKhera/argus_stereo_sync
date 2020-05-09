#ifndef CONVERT_H
#define CONVERT_H

#include <cuda.h>

extern float convert(CUsurfObject surface1, CUsurfObject surface2,
		     unsigned int width, unsigned int height,
		     uint8_t* oBuffer);

#endif // CONVERT_H
