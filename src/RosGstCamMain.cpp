/**
 * @brief MAVROS CAMERA NODE
 * @file Cam_Main.cpp
 * @author Simon.SZ.Huang
 
 */
#include "common.hpp"
#include "RosGstCamServer.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * 
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    GSTCAM_INFO("ROS2 CAMERA NODE FOR QGC SERVER START ......\n");

    RosGstCamServer *pRosGstCamServer = RosGstCamServer::CreateInstance();
    if(pRosGstCamServer)
    {
        pRosGstCamServer->Run();
    }
    else
    {
        GSTCAM_ERROR("GET ROS2 CAMERA NODE FAIL!\n");
    }

    if(pRosGstCamServer)
    {
        GSTCAM_INFO("ROS2 CAMERA NODE pCamServer Destroy......\n");
        pRosGstCamServer->Destroy();
        pRosGstCamServer = NULL;
    }

    rclcpp::shutdown();
    GSTCAM_INFO("ROS2 CAMERA NODE FOR QGC SERVER EXIT ......\n");

    return 0;
}
 


