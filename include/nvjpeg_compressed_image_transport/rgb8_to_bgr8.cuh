#ifndef NVJPEG_COMPRESSED_IMAGE_TRANSPORT_RGB8_TO_BGR8
#define NVJPEG_COMPRESSED_IMAGE_TRANSPORT_RGB8_TO_BGR8

#include <cuda_runtime_api.h>

namespace nvjpeg_compressed_image_transport
{
inline __device__ __host__ int iDivUp(int a, int b) {return (a % b != 0) ? (a / b + 1) : (a / b);}

__global__ void RGB8ToBGR8(unsigned char * input, int width, int height, int step);

cudaError_t cudaRGB8ToBGR8(unsigned char * input, int width, int height, int step);
} // namespace nvjpeg_compressed_image_transport

#endif  // NVJPEG_COMPRESSED_IMAGE_TRANSPORT_RGB8_TO_BGR8