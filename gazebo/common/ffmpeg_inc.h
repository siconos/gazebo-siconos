/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_FFMPEG_INC_HH_
#define _GAZEBO_FFMPEG_INC_HH_

#include <gazebo/gazebo_config.h>

#pragma GCC system_header

#ifdef HAVE_FFMPEG
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>

#if defined(__linux__) && defined(HAVE_AVDEVICE)
#include <libavdevice/avdevice.h>
#endif
}

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \brief Helper function to avoid deprecation warnings.
    GZ_COMMON_VISIBLE
    AVFrame *AVFrameAlloc(void);

    /// \brief Helper function to avoid deprecation warnings.
    GZ_COMMON_VISIBLE
    void AVFrameUnref(AVFrame *_frame);

    /// \brief Helper function to avoid deprecation warnings.
    /// \param[in] _packet AVPacket structure that stores compressed data
    GZ_COMMON_VISIBLE
    void AVPacketUnref(AVPacket *_packet);
  }
}
// ifdef HAVE_FFMPEG
#endif

// ifndef _GAZEBO_FFMPEG_INC_HH
#endif
