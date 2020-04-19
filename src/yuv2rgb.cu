#include <cuda_runtime.h>
#include "yuv2rgb.cuh"

__device__ inline int clamp(int val, int mn, int mx) {
  return (val >= mn)? ((val <= mx)? val : mx) : mn;
}

__global__ void gpuConvertYUV420toRGB_kernel(unsigned char *Y, unsigned char *CbCr, unsigned char *RGB, unsigned int width, unsigned int height) {
  int idx1 = blockIdx.x * blockDim.x + threadIdx.x;

  int r2 = ((int) (idx1 / width)) / 2;
  int c2 = ((int) ((idx1 % width) / 2)) * 2;
  int idx2 = r2 * (width / 2) + c2;

  int y = Y[idx1];
  int cb = CbCr[idx2 + 1] - 128;
  int cr = CbCr[idx2 + 0] - 128;

  RGB[3 * idx1 + 0] = clamp((int) (y + 1.402 * cr), 0, 255);
  RGB[3 * idx1 + 1] = clamp((int) (y - 0.344 * cb - 0.714 * cr), 0, 255);
  RGB[3 * idx1 + 2] = clamp((int) (y + 1.772 * cb), 0, 255);
}

void gpuConvertYUV420toRGB(const unsigned char *Y, const unsigned char *CbCr, unsigned char *RGB, unsigned int width, unsigned int height) {
  size_t planeSize = width * height * sizeof(unsigned char);

  unsigned char *d_Y = NULL;
  unsigned char *d_CbCr = NULL;
  unsigned char *d_RGB = NULL;

  cudaMalloc(&d_Y, planeSize);
  cudaMalloc(&d_CbCr, planeSize / 2);
  cudaMalloc(&d_RGB, 3 * planeSize);   
 
  cudaMemcpy(d_Y, Y, planeSize, cudaMemcpyHostToDevice);
  cudaMemcpy(d_CbCr, CbCr, planeSize / 2, cudaMemcpyHostToDevice);

  unsigned int blockSize = 129600;
  unsigned int numBlocks = (width * height) / blockSize;
  gpuConvertYUV420toRGB_kernel<<<numBlocks, blockSize>>>(d_Y, d_CbCr, d_RGB, width, height);
  cudaStreamAttachMemAsync(NULL, RGB, 0, cudaMemAttachHost);
  cudaStreamSynchronize(NULL);

  cudaMemcpy(RGB, d_RGB, 3 * planeSize, cudaMemcpyDeviceToHost);
  cudaFree(d_Y);
  cudaFree(d_CbCr);
  cudaFree(d_RGB);
}
