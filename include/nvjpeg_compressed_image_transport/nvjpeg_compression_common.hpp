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

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef NVJPEG_COMPRESSED_IMAGE_TRANSPORT_COMPRESSION_COMMON
#define NVJPEG_COMPRESSED_IMAGE_TRANSPORT_COMPRESSION_COMMON

#include <cuda_runtime_api.h>
#include <nvjpeg.h>

#include <memory>

namespace nvjpeg_compressed_image_transport
{

constexpr int DEFAULT_JPEG_QUALITY = 95;

struct StreamDeleter
{
  void operator()(cudaStream_t * stream)
  {
    if (stream) {
      cudaStreamDestroy(*stream);
      delete stream;
    }
  }
};

using StreamUniquePtr = std::unique_ptr<cudaStream_t, StreamDeleter>;

inline StreamUniquePtr makeCudaStream(const uint32_t flags = cudaStreamDefault)
{
  StreamUniquePtr stream(new cudaStream_t, StreamDeleter());
  if (cudaStreamCreateWithFlags(stream.get(), flags) != cudaSuccess) {
    stream.reset(nullptr);
  }
  return stream;
}

struct NvjpegHandleDeleter
{
  void operator()(nvjpegHandle_t * handle)
  {
    if (handle) {
      nvjpegDestroy(*handle);
      delete handle;
    }
  }
};

using NvjpegHandleUniquePtr = std::unique_ptr<nvjpegHandle_t, NvjpegHandleDeleter>;

inline NvjpegHandleUniquePtr makeNvjpegHandle()
{
  NvjpegHandleUniquePtr handle(new nvjpegHandle_t, NvjpegHandleDeleter());
  if (nvjpegCreateSimple(handle.get()) != NVJPEG_STATUS_SUCCESS) {
    handle.reset(nullptr);
  }
  return handle;
}

struct NvjpegEncoderStateDeleter
{
  void operator()(nvjpegEncoderState_t * state)
  {
    if (state) {
      nvjpegEncoderStateDestroy(*state);
      delete state;
    }
  }
};

using NvjpegEncoderStateUniquePtr =
  std::unique_ptr<nvjpegEncoderState_t, NvjpegEncoderStateDeleter>;

inline NvjpegEncoderStateUniquePtr makeNvjpegEncoderState(
  nvjpegHandle_t handle,
  cudaStream_t stream)
{
  NvjpegEncoderStateUniquePtr state(new nvjpegEncoderState_t, NvjpegEncoderStateDeleter());
  if (nvjpegEncoderStateCreate(handle, state.get(), stream) != NVJPEG_STATUS_SUCCESS) {
    state.reset(nullptr);
  }
  return state;
}

struct NvjpegEncoderParamsDeleter
{
  void operator()(nvjpegEncoderParams_t * params)
  {
    if (params) {
      nvjpegEncoderParamsDestroy(*params);
      delete params;
    }
  }
};

using NvjpegEncoderParamsUniquePtr = std::unique_ptr<nvjpegEncoderParams_t,
    NvjpegEncoderParamsDeleter>;

inline NvjpegEncoderParamsUniquePtr makeNvjpegEncoderParams(
  nvjpegHandle_t handle,
  cudaStream_t stream)
{
  NvjpegEncoderParamsUniquePtr params(new nvjpegEncoderParams_t, NvjpegEncoderParamsDeleter());
  if (nvjpegEncoderParamsCreate(handle, params.get(), stream) != NVJPEG_STATUS_SUCCESS) {
    params.reset(nullptr);
  }
  return params;
}

} // namespace nvjpeg_compressed_image_transport

#endif
