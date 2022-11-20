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

#include "nvjpeg_compressed_image_transport/nvjpeg_compression_common.hpp"
#include "nvjpeg_compressed_image_transport/nvjpeg_compressed_publisher.hpp"
#include "nvjpeg_compressed_image_transport/rgb8_to_bgr8.cuh"


#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sstream>
#include <vector>

namespace nvjpeg_compressed_image_transport
{

namespace enc = sensor_msgs::image_encodings;

void NvjpegCompressedPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;
  using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  jpeg_quality_param_name_ = param_base_name + ".jpeg_quality";
  rcl_interfaces::msg::ParameterDescriptor jpeg_quality_description;
  jpeg_quality_description.name = "jpeg_quality";
  jpeg_quality_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  jpeg_quality_description.description = "Image quality for JPEG format";
  jpeg_quality_description.read_only = false;
  rcl_interfaces::msg::IntegerRange jpeg_range;
  jpeg_range.from_value = 1;
  jpeg_range.to_value = 100;
  jpeg_range.step = 1;
  jpeg_quality_description.integer_range.push_back(jpeg_range);
  try {
    config_.jpeg_quality = node->declare_parameter(
      jpeg_quality_param_name_, DEFAULT_JPEG_QUALITY, jpeg_quality_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", jpeg_quality_param_name_.c_str());
    config_.jpeg_quality = node->get_parameter(jpeg_quality_param_name_).get_value<int64_t>();
  }
  optimized_huffman_param_name_ = param_base_name + ".optimized_huffman";
  rcl_interfaces::msg::ParameterDescriptor optimized_huffman_description;
  optimized_huffman_description.name = "optimized_huffman";
  optimized_huffman_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  optimized_huffman_description.description = "Use huffman optimization or not";
  optimized_huffman_description.read_only = false;
  try {
    config_.optimized_huffman = node->declare_parameter(
      optimized_huffman_param_name_, false, optimized_huffman_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", optimized_huffman_param_name_.c_str());
    config_.optimized_huffman =
      node->get_parameter(optimized_huffman_param_name_).get_value<bool>();
  }
  stream_ = makeCudaStream();
  if (!stream_) {
    RCLCPP_ERROR(logger_, "failed to create cuda stream");
    shutdown();
    return;
  }
  handle_ = makeNvjpegHandle();
  if (!handle_) {
    RCLCPP_ERROR(logger_, "failed to create nvjpeg handle");
    shutdown();
    return;
  }
  state_ = makeNvjpegEncoderState(*handle_, *stream_);
  if (!state_) {
    RCLCPP_ERROR(logger_, "failed to create nvjpeg encorder state");
    shutdown();
    return;
  }
  params_ = makeNvjpegEncoderParams(*handle_, *stream_);
  if (!params_) {
    RCLCPP_ERROR(logger_, "failed to create nvjpeg encorder params");
    shutdown();
    return;
  }
  if (nvjpegEncoderParamsSetQuality(
      *params_, config_.jpeg_quality, *stream_) != NVJPEG_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(logger_, "failed to set jpeg quality");
  }
  if (nvjpegEncoderParamsSetOptimizedHuffman(
      *params_, config_.optimized_huffman, *stream_) != NVJPEG_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(logger_, "failed to set optimized huffman");
  }
  if (nvjpegEncoderParamsSetSamplingFactors(
      *params_, NVJPEG_CSS_444, *stream_) != NVJPEG_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(logger_, "failed to set sampling factors");
  }
}

void NvjpegCompressedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Compressed image message
  sensor_msgs::msg::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);

  // Update ros message format header
  compressed.format += "; jpeg compressed ";

  // Check input format

  if ((bitDepth == 8) || (bitDepth == 16)) {
    nvjpegInputFormat_t input_format{};
    if (message.encoding != "rgb8" && message.encoding != "bgr8") {
      RCLCPP_ERROR(
        logger_,
        "%s is invalid encording format! Not supportted encording type.",
        message.encoding.c_str()
      );
      return;
    } else {
      input_format = NVJPEG_INPUT_BGRI;
      compressed.format += "bgr8";
    }

    nv_image_.pitch[0] = message.step;

    if (image_length_ < message.data.size()) {
      if (cudaMalloc((void **)&(nv_image_.channel[0]), message.data.size()) != cudaSuccess) {
        RCLCPP_ERROR(logger_, "failed to allocate device memory");
        return;
      }
      image_length_ = message.data.size();
    }

    if (cudaMemcpy(
        nv_image_.channel[0], &message.data[0], message.data.size(),
        cudaMemcpyHostToDevice) != cudaSuccess)
    {
      RCLCPP_ERROR(logger_, "failed to copy host memory to device memory");
      return;
    }
    if (message.encoding == "rgb8")
    {
      if (cudaRGB8ToBGR8(
          nv_image_.channel[0], message.width, message.height, message.step) != cudaSuccess)
      {
        RCLCPP_ERROR(logger_, "failed to convert rgb8 to bgr8");
        return;
      }
    }

    if (nvjpegEncodeImage(
        *handle_, *state_, *params_,
        &nv_image_, input_format, message.width, message.height, *stream_) != NVJPEG_STATUS_SUCCESS)
    {
      RCLCPP_ERROR(logger_, "failed to encode image");
      return;
    }

    size_t buffer_length{};
    if (nvjpegEncodeRetrieveBitstream(
        *handle_, *state_, nullptr, &buffer_length,
        *stream_) != NVJPEG_STATUS_SUCCESS)
    {
      RCLCPP_ERROR(logger_, "failed to retrieve bitstream");
      return;
    }

    if (cudaStreamSynchronize(*stream_) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "failed to synchronize stream");
      return;
    }

    compressed.data.resize(buffer_length);

    if (nvjpegEncodeRetrieveBitstream(
        *handle_, *state_,
        compressed.data.data(), &buffer_length, 0) != NVJPEG_STATUS_SUCCESS)
    {
      RCLCPP_ERROR(logger_, "failed to retrieve bitstream");
      return;
    }

    if (cudaStreamSynchronize(*stream_) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "failed to synchronize stream");
      return;
    }

    // Publish message
    publish_fn(compressed);
  } else {
    RCLCPP_ERROR(
      logger_,
      "Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)",
      message.encoding.c_str());
  }
}
} // namespace compressed_image_transport
