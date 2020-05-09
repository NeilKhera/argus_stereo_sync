#include <stdio.h>
#include "convert.h"

// First-pass histogram kernel (binning into privatized counters)
/*template <
    int         NUM_PARTS,
    int         NUM_BINS>
__global__ void histogram_smem_atomics(
    CUsurfObject surface,
    unsigned int width,
    unsigned int height,
    unsigned int *out)
{
    // global position and size
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int nx = blockDim.x * gridDim.x;
    int ny = blockDim.y * gridDim.y;

    // threads in workgroup
    int t = threadIdx.x + threadIdx.y * blockDim.x; // thread index in workgroup, linear in 0..nt-1
    int nt = blockDim.x * blockDim.y; // total threads in workgroup

    // group index in 0..ngroups-1
    int g = blockIdx.x + blockIdx.y * gridDim.x;

    // initialize smem
    __shared__ unsigned int smem[NUM_BINS];
    for (int i = t; i < NUM_BINS; i += nt)
        smem[i] = 0;

    // process pixels (updates our group's partial histogram in smem)
    for (int col = x; col < width; col += nx)
    {
        for (int row = y; row < height; row += ny)
        {
            uchar1 data;
            surf2Dread(&data, surface, col, row);

            atomicAdd(&smem[((unsigned int)data.x) % NUM_BINS], 1);
        }
    }

    __syncthreads();

    // move to our workgroup's slice of output
    out += g * NUM_PARTS;

    // store local output to global
    for (int i = t; i < NUM_BINS; i += nt)
    {
        out[i] = smem[i];
    }
}*/

__global__ void convert_kernel(CUsurfObject surface, unsigned int width, unsigned int height, uint8_t* out) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int nx = blockDim.x * gridDim.x;
  int ny = blockDim.y * gridDim.y;

  for (int col = x; col < width; col += nx) {
    for (int row = y; row < height; row += ny) {
      uchar1 data;
      surf2Dread(&data, surface, col, row);
      out[row * width + col] = data.x;
    }
  }
}

float run_smem_atomics(CUsurfObject surface, unsigned int width, unsigned int height, uint8_t* oBuffer) {
  enum {
    NUM_PARTS = 1024
  };
  cudaError_t err = cudaSuccess;
  dim3 block(32, 4);
  dim3 grid(16, 16);
  //int total_blocks = grid.x * grid.y;

  uint8_t* d_buffer;
  err = cudaMalloc(&d_buffer, width * height * sizeof(uint8_t));
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to allocate device buffer (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
    
  /*uint8_t* d_part_buffer;
  err = cudaMalloc(&d_part_buffer, total_blocks * NUM_PARTS * sizeof(unsigned int));
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to allocate device part buffer (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  dim3 block2(128);
  dim3 grid2((SIZE + block.x - 1) / block.x);*/

  cudaEvent_t start;
  cudaEvent_t stop;

  cudaEventCreate(&stop);
  cudaEventCreate(&start);

  cudaEventRecord(start, 0);

  convert_kernel<<<grid, block>>>(surface, width, height, d_buffer);
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to launch kernel (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  /*convert_kernel<NUM_PARTS, NUM_BINS><<<grid2, block2>>>(d_part_hist, total_blocks, d_hist);
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to launch convert_smem_accum kernel (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }*/

  cudaEventRecord(stop, 0);

  cudaEventSynchronize(stop);
  float elapsed_millis;
  cudaEventElapsedTime(&elapsed_millis, start, stop);

  cudaEventDestroy(start);
  cudaEventDestroy(stop);

  err = cudaMemcpy(oBuffer, d_buffer, width * height * sizeof(uint8_t), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to copy into host buffer (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  /*err = cudaFree(d_part_buffer);
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to free device buffer part (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }*/

  err = cudaFree(d_buffer);
  if (err != cudaSuccess) {
    fprintf(stderr, "Failed to free device buffer (%s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  return elapsed_millis;
}

float convert(CUsurfObject surface, unsigned int width,
	     unsigned int height, uint8_t* oBuffer) {
  return run_smem_atomics(surface, width, height, oBuffer);
}
