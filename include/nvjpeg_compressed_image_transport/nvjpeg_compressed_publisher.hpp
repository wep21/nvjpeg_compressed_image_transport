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

#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

#include "nvjpeg_compressed_image_transport/nvjpeg_compression_common.hpp"

namespace nvjpeg_compressed_image_transport
{

using CompressedImage = sensor_msgs::msg::CompressedImage;

class NvjpegCompressedPublisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  NvjpegCompressedPublisher()
  : logger_(rclcpp::get_logger("NvjpegCompressedPublisher")) {}

  ~NvjpegCompressedPublisher()
  {
    if (cudaFree(nv_image_.channel[0]) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "failed to free device memory");
    }
  }

  std::string getTransportName() const override
  {
    return "compressed";
  }

protected:
  // Overridden to set up reconfigure server
  void advertiseImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions options) override;

  void publish(
    const sensor_msgs::msg::Image & message,
    const PublishFn & publish_fn) const override;

  struct Config
  {
    // JPEG Quality from 0 to 100 (higher is better quality).
    // Default to OpenCV default of 95.
    int jpeg_quality;

    bool optimized_huffman;
  };

  Config config_;
  std::string jpeg_quality_param_name_;
  std::string optimized_huffman_param_name_;
  rclcpp::Logger logger_;
  rclcpp::Node * node_;
  StreamUniquePtr stream_;
  NvjpegHandleUniquePtr handle_;
  NvjpegEncoderStateUniquePtr state_;
  NvjpegEncoderParamsUniquePtr params_;
  mutable nvjpegImage_t nv_image_;
  mutable size_t image_length_{};
};

} // namespace nvjpeg_compressed_image_transport
