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
#include "common.hpp"
#include "RosGstCamServer.hpp"
#include "MavrosGstCamServer.hpp"
#include "RosGstCamPublishStream.hpp"
#include "mavros/mavros_router.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mavros/utils.hpp"

using namespace mavros::cam;

RosGstCamServer::RosGstCamServer()
{
    ;
}

RosGstCamServer::~RosGstCamServer()
{
    Deinit();
}

RosGstCamServer* RosGstCamServer::CreateInstance()
{
    static RosGstCamServer* mCamServer = NULL;

    if (mCamServer)
    {
        return mCamServer;
    }

    mCamServer = new RosGstCamServer();
    if (mCamServer == NULL)
    {
        GSTCAM_ERROR("RosGstCamServer CreateInstance fail! \n");
        return NULL;
    }

    if (mCamServer->Init())
    {
        mCamServer->Destroy();
        mCamServer = NULL;
    }

    return  mCamServer;
}

int32_t RosGstCamServer::Init()
{
    GSTCAM_INFO("RosGstCamServer init...\n");
    return 0;
}

void RosGstCamServer::Run()
{
    int32_t ret = -1;
    int32_t config_index_;
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    rclcpp::NodeOptions options;
    //std::lock_guard<std::mutex> lock(m_lock);

    gst_init(nullptr, nullptr);
    
    GSTCAM_INFO("RosGstCamServer start run...\n");
    auto node = std::make_shared<rclcpp::Node>("Ros2GstCamServerNode",options);
    exec.add_node(node);
    
    node->declare_parameter("config_index", 0);
    node->get_parameter("config_index", config_index_);
    RCLCPP_INFO(node->get_logger(), "Starting Ros2GstCamServerNode container");

    ret = GstCamConfig::PreLoadCamConfigure(&mCamConfig, config_index_);
    if(ret || mCamConfig == NULL)
    {
        GSTCAM_INFO("RosGstCamServer: load cam config fail... , Perhaps configs files not exits \n");
        return;   
    }

    CamPublisher_node = std::make_shared<campublishstream::CamPublishStream>(options, mCamConfig);
    if(CamPublisher_node == NULL)
    {
        GSTCAM_ERROR("RosGstCamServer new camera componment fail! \n");
        return;

    }
    exec.add_node(CamPublisher_node);

    RCLCPP_INFO(mavroscam_node->get_logger(), "mavroscam_node Start heartbeat run... \n");
    
    exec.spin();        
    return;
}

void RosGstCamServer::Destroy()
{
    delete this;
}

void RosGstCamServer::Deinit()
{
    //std::lock_guard<std::mutex> lock(m_lock);

    GSTCAM_INFO("ROS2 CAMERA NODE CamServer Deinit......\n");
    
    mCamConfig->Destroy();
    mCamConfig = NULL;
    gst_deinit();
}
