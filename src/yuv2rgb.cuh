#ifndef __YUV2RGB_CUH__
#define __YUV2RGB_CUH__

void gpuConvertYUV420toRGB(const unsigned char *Y, const unsigned char *CbCr, unsigned char *RGB, unsigned int width, unsigned int height);

#endif
