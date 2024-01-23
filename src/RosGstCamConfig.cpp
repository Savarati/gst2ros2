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
#include <unistd.h>
#include "RosGstCamConfig.hpp"
#include "yaml-cpp/yaml.h"

GstCamConfig::GstCamConfig(GstRos2CamConfig *camcfg)
{
    GSTCAM_INFO("********** GstCamConfig : create config pipeline %d ...*************\n", camcfg->pipelineId);
}

GstCamConfig::~GstCamConfig()
{
    Deinit();
}

void GstCamConfig::Destroy()
{
    delete this;
}

void GstCamConfig::Deinit()
{
    
}

int32_t GstCamConfig::PreLoadCamConfigure(GstCamConfig **camcfg, int32_t index)
{
    GstRos2CamConfig m_camCfg;
    YAML::Node Config;

    if (camcfg == NULL)
    {
        GSTCAM_ERROR("GstCamConfig: PreLoadCamConfigure invalid params");
        return -1;
    }

    char fileName[256];
    snprintf(fileName, sizeof(fileName),
        GSTROS2CAM_CONFIG_DIRECTORY"camera_%d.yaml", index);
    if(access(fileName, R_OK | W_OK))
    {
        GSTCAM_ERROR("GstCamConfig : :%s file not exit \n", fileName);
        return -1;
    }

    GSTCAM_INFO("GstCamConfig: PreLoadCamConfigure filename %s \n",fileName);
    try{
        Config = YAML::LoadFile(fileName);
    } catch(YAML::BadFile &e) {
        GSTCAM_ERROR("config is NULL \n");
    }

    if(Config["Camid"])
    {
        m_camCfg.pipelineId = Config["Camid"].as<int>();
        m_camCfg.cameraId = Config["Camid"].as<int>();
    }
    else
    {
        m_camCfg.pipelineId = 0;
        m_camCfg.cameraId = 0;
    }
    if(Config["StreamSource"])
        m_camCfg.cameraSource = Config["StreamSource"].as<std::string>();
    else
        m_camCfg.cameraSource = "qtiqmmfsrc";
    
    /* load CamPubListerStream stream configuration */
    if(Config["CamPubListerStream"]["mWidth"])
        m_camCfg.mWidth = Config["CamPubListerStream"]["mWidth"].as<int>();
    else
        m_camCfg.mWidth = 1920;
    if(Config["CamPubListerStream"]["mHeight"])
        m_camCfg.mHeight = Config["CamPubListerStream"]["mHeight"].as<int>();
    else
        m_camCfg.mHeight = 1080;
    if(Config["CamPubListerStream"]["rWidth"])
        m_camCfg.rWidth = Config["CamPubListerStream"]["rWidth"].as<int>();
    else
        m_camCfg.rWidth = 4192;
    if(Config["CamPubListerStream"]["rHeight"])
        m_camCfg.rHeight = Config["CamPubListerStream"]["rHeight"].as<int>();
    else
        m_camCfg.rHeight = 3104;
    if(Config["CamPubListerStream"]["Format"])
        m_camCfg.format = Config["CamPubListerStream"]["Format"].as<std::string>();
    else
        m_camCfg.format = "NV12";
    if(Config["CamPubListerStream"]["FPS"])
        m_camCfg.fps = Config["CamPubListerStream"]["FPS"].as<int>();
    else
        m_camCfg.fps = 30;
    if(Config["CamPubListerStream"]["AfMode"])
        m_camCfg.afmode = Config["CamPubListerStream"]["AfMode"].as<int>();
    else
        m_camCfg.afmode = 3;
    if(Config["CamPubListerStream"]["image_encoding"])
        m_camCfg.image_encoding = Config["CamPubListerStream"]["image_encoding"].as<std::string>();
    else
        m_camCfg.image_encoding = "rgb8";

    /* load rtsp stream configuration file */
    if(Config["RtspStream"]["RtspHost"])
        m_camCfg.rtspHost = Config["RtspStream"]["RtspHost"].as<std::string>();
    else 
        m_camCfg.rtspHost = "127.0.0.1";
    if(Config["RtspStream"]["RtspPort"])
        m_camCfg.rtspPort = Config["RtspStream"]["RtspPort"].as<std::string>();
    else
        m_camCfg.rtspPort = "8900";
    if(Config["RtspStream"]["RtspMount"])
        m_camCfg.rtspMount = Config["RtspStream"]["RtspMount"].as<std::string>();
    else
        m_camCfg.rtspMount = "/live";
    /*if(Config["RtspStream"]["PipelineStr"])
    {
        m_camCfg.rtspPipelineStr = Config["RtspStream"]["PipelineStr"].as<std::string>();
    }
    else
    {
        m_camCfg.rtspPipelineStr = "";
    }*/

    /* mavlink server conf file */
    if(Config["MavLinkServer"]["MavTgtSystem"])
        m_camCfg.MavTgtSystem = Config["MavLinkServer"]["MavTgtSystem"].as<int>();
    else
        m_camCfg.MavTgtSystem = 1;
    if(Config["MavLinkServer"]["MavTgtComponent"])
        m_camCfg.MavTgtComponent = Config["MavLinkServer"]["MavTgtComponent"].as<int>();
    else
        m_camCfg.MavTgtComponent = 1;
    if(Config["MavLinkServer"]["MavSourceSystem"])
        m_camCfg.MavSourceSystem = Config["MavLinkServer"]["MavSourceSystem"].as<int>();
    else
        m_camCfg.MavSourceSystem = 1;
    if(Config["MavLinkServer"]["MavSourceComp"])
        m_camCfg.MavSourceComp = Config["MavLinkServer"]["MavSourceComp"].as<int>();
    else
        m_camCfg.MavSourceComp = 100;

    *camcfg = GstCamConfig::CreateInstance(&m_camCfg);
    if(*camcfg)
    {
        (*camcfg)->PrintConfig();
    }
    else
    {
        GSTCAM_ERROR("GstCamConfig: CreateInstance fail! \n");
        return -1;
    }

    return 0;
}

void GstCamConfig::PrintConfig()
{
    GSTCAM_DEBUG( "\n"
                  "pipelineId:   %d \n"
                  "cameraId:     %d \n"
                  "cameraSource: %s \n"
                  "mWidth:       %d \n"
                  "mHeight:      %d \n"
                  "rWidth:       %d \n"
                  "rHeight:      %d \n"
                  "format:       %s \n"
                  "fps:          %d \n"
                  "afmode:       %d \n"
                  "image_encoding:        %s \n"
                  "rtspHost:     %s \n"
                  "rtspPort:     %s \n"
                  "rtspMount:    %s \n"
                  "rtspPipelineStr:       %s \n"
                  "MavTgtSystem:          %d \n"
                  "MavTgtComponent:       %d \n"
                  "MavSourceSystem:       %d \n"
                  "MavSourceComp:         %d \n"
                  ,
        m_camCfg.pipelineId,
        m_camCfg.cameraId,
        m_camCfg.cameraSource.c_str(),
        m_camCfg.mWidth,
        m_camCfg.mHeight,
        m_camCfg.rWidth,
        m_camCfg.rHeight,
        m_camCfg.format.c_str(),
        m_camCfg.fps,
        m_camCfg.afmode,
        m_camCfg.image_encoding.c_str(),
        m_camCfg.rtspHost.c_str(),
        m_camCfg.rtspPort.c_str(),
        m_camCfg.rtspMount.c_str(),
        m_camCfg.rtspPipelineStr.c_str(),
        m_camCfg.MavTgtSystem,
        m_camCfg.MavTgtComponent,
        m_camCfg.MavSourceSystem,
        m_camCfg.MavSourceComp);
}

GstCamConfig* GstCamConfig::CreateInstance(GstRos2CamConfig *camcfg)
{
    GstCamConfig* mCamConfig = NULL;

    mCamConfig = new GstCamConfig(camcfg);
    if (mCamConfig == NULL)
    {
        GSTCAM_ERROR("GstCamConfig: CreateInstance fail! \n");
        return NULL;
    }

    if (mCamConfig->Init(camcfg))
    {
        mCamConfig->Destroy();
        mCamConfig = NULL;
    }

    return  mCamConfig;
}

int32_t GstCamConfig::Init(GstRos2CamConfig *camCfg)
{
        if(camCfg == NULL)
        {
            return -1;
        }
        m_camCfg.pipelineId              = camCfg->pipelineId;
        m_camCfg.cameraId                = camCfg->cameraId;
        m_camCfg.cameraSource            = camCfg->cameraSource;
        m_camCfg.mWidth                  = camCfg->mWidth;
        m_camCfg.mHeight                 = camCfg->mHeight;
        m_camCfg.rWidth                  = camCfg->rWidth;
        m_camCfg.rHeight                 = camCfg->rHeight;
        m_camCfg.format                  = camCfg->format;
        m_camCfg.fps                     = camCfg->fps;
        m_camCfg.afmode                  = camCfg->afmode;
        m_camCfg.image_encoding          = camCfg->image_encoding;

        m_camCfg.rtspHost                = camCfg->rtspHost;
        m_camCfg.rtspPort                = camCfg->rtspPort;
        m_camCfg.rtspMount               = camCfg->rtspMount;
        m_camCfg.rtspPipelineStr         = camCfg->rtspPipelineStr;

        m_camCfg.MavTgtSystem            = camCfg->MavTgtSystem;
        m_camCfg.MavTgtComponent         = camCfg->MavTgtComponent;
        m_camCfg.MavSourceSystem         = camCfg->MavSourceSystem;
        m_camCfg.MavSourceComp           = camCfg->MavSourceComp;
        return 0;
}


