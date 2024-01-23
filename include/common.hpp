/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <set>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>  
#include <thread> 
#include <fstream> 
//#include <memory>

extern "C" {
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <dirent.h>
#include <string.h>
#include <glib-unix.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
}

typedef enum _GstCamLogLevel 
{
    GstCamLogLevelError  = 0,
    GstCamLogLevelWarn   = 1,
    GstCamLogLevelInfo   = 2,
    GstCamLogLevelDebug  = 3,
} GstCamLogLevel;
#define DEFAULT_LOGS_LEVEL   2
#define LOGLEVEL_S           DEFAULT_LOGS_LEVEL

#define GSTCAM_DEBUG(fmt, args...) do {                                    \
    if(LOGLEVEL_S >= GstCamLogLevelDebug)                                    \
        g_print("[DEBUG] %s %d:" fmt, __FUNCTION__, __LINE__, ##args);     \
} while (0)

#define GSTCAM_INFO(fmt, args...) do {                                     \
    if(LOGLEVEL_S >= GstCamLogLevelInfo)                                     \
        g_print("[INFO] %s %d:" fmt, __FUNCTION__, __LINE__, ##args);      \
} while (0)

#define GSTCAM_WARNING(fmt, args...) do {                                  \
    if(LOGLEVEL_S >= GstCamLogLevelWarn)                                     \
        g_print("[WARNING] %s %d:" fmt, __FUNCTION__, __LINE__, ##args);   \
} while (0)

#define GSTCAM_ERROR(fmt, args...) do {                                    \
    if(LOGLEVEL_S >= GstCamLogLevelError)                                    \
        g_print("[ERROR] %s %d:" fmt, __FUNCTION__, __LINE__, ##args);     \
} while (0)

#define MAKE_AND_ADD(var, pipe, name, label, elem_name)                       \
G_STMT_START {                                                                \
  if (G_UNLIKELY (!(var = (gst_element_factory_make (name, elem_name))))) {   \
    g_print ("Could not create element %s", name);                            \
    goto label;                                                               \
  }                                                                           \
  if (G_UNLIKELY (!gst_bin_add (GST_BIN_CAST (pipe), var))) {                 \
    g_print ("Could not add element %s", name);                               \
    goto label;                                                               \
  }                                                                           \
} G_STMT_END


