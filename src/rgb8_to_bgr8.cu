// Copyright 2022 Daisuke Nishimatsu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nvjpeg_compressed_image_transport/rgb8_to_bgr8.cuh"

namespace nvjpeg_compressed_image_transport
{
__global__ void RGB8ToBGR8(unsigned char * input, int width, int height, int step)
{
  //2D Index of current thread
  const int x_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int y_index = blockIdx.y * blockDim.y + threadIdx.y;

  //Only valid threads perform memory I/O
  if ((x_index < width) && (y_index < height)) {
    //Location of colored pixel in input
    const int color_tid = y_index * step + (3 * x_index);
    const unsigned char t = input[color_tid + 0];
    input[color_tid + 0] = input[color_tid + 2];
    input[color_tid + 2] = t;
  }
}

cudaError_t cudaRGB8ToBGR8(unsigned char * input, int width, int height, int step)
{
  if (!input) {
    return cudaErrorInvalidDevicePointer;
  }

  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));

  RGB8ToBGR8<<<gridDim, blockDim>>>(input, width, height, step);

  return cudaGetLastError();
}

}  // namespace nvjpeg_compressed_image_transport
