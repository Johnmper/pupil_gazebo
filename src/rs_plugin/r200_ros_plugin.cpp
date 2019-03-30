#include "rs_plugin/r200_ros_plugin.h"
#include <sensor_msgs/fill_image.h>


namespace gazebo
{
    // Register the plugin
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

    /////////////////////////////////////////////////
    GazeboRosRealsense::GazeboRosRealsense()
    {
    }

    /////////////////////////////////////////////////
    GazeboRosRealsense::~GazeboRosRealsense()
    {
        ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
    }

    /////////////////////////////////////////////////
    void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()){
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        ROS_INFO("Realsense Gazebo ROS plugin loading.");

        RealSensePlugin::Load(_model, _sdf);

        // RGB camera info message
        image_info_.width = 1920;
        image_info_.height = 1080;
        image_info_.K[0] = 1415;
        image_info_.K[4] = 1415;
        image_info_.K[2] = image_info_.width * 0.5;
        image_info_.K[5] = image_info_.height * 0.5;
        image_info_.K[8] = 1.;

        image_info_.P[0] = image_info_.K[0];    // fx
        image_info_.P[5] = image_info_.K[4];    // fy
        image_info_.P[2] = image_info_.K[2];    // cx
        image_info_.P[6] = image_info_.K[5];    // cy
        image_info_.P[10] = 1;

        // Depth camera info message
        depth_info_.width = 640;
        depth_info_.height = 480;
        depth_info_.K[0] = 590;
        depth_info_.K[4] = 590;
        depth_info_.K[2] = depth_info_.width * 0.5;
        depth_info_.K[5] = depth_info_.height * 0.5;
        depth_info_.K[8] = 1.;

        depth_info_.P[0] = depth_info_.K[0];    // fx
        depth_info_.P[5] = depth_info_.K[4];    // fy
        depth_info_.P[2] = depth_info_.K[2];    // cx
        depth_info_.P[6] = depth_info_.K[5];    // cy
        depth_info_.P[10] = 1;


        this->rosnode_ = new ros::NodeHandle();
        // ROS Image transport
        this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

        this->color_pub_ = this->itnode_->advertiseCamera("/pupil/world/image_raw", 1);
        this->depth_pub_ = this->itnode_->advertiseCamera("/pupil/depth/image_raw", 1);
    }

        /////////////////////////////////////////////////
    void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub)
    {
        common::Time current_time = this->world->GetSimTime();

        // identify camera
        std::string camera_id = cam->Name();
        image_transport::CameraPublisher* image_pub;
        if (camera_id.find(COLOR_CAMERA_NAME) != std::string::npos){
            camera_id = COLOR_CAMERA_NAME;
            image_pub = &(this->color_pub_);
        }
        else{
            ROS_ERROR("Invalid camera name\n");
            return;
        }

        // copy data into image
        this->image_msg_.header.seq++;
        this->image_msg_.header.frame_id = "rgb_optical_frame";
        this->image_msg_.header.stamp.sec = current_time.sec;
        this->image_msg_.header.stamp.nsec = current_time.nsec;
        this->image_info_.header = this->image_msg_.header;

        // set image encoding
        std::string pixel_format = cam->ImageFormat();
        if(pixel_format == "RGB_INT8"){
            pixel_format = sensor_msgs::image_encodings::RGB8;
        }
        else{
            ROS_ERROR("Unsupported Gazebo ImageFormat\n");
            pixel_format = sensor_msgs::image_encodings::BGR8;
        }

        // copy from simulation image to ROS msg
        fillImage(this->image_msg_,
            pixel_format,
            cam->ImageHeight(), cam->ImageWidth(),
            cam->ImageDepth()*cam->ImageWidth(),
            reinterpret_cast<const void*>(cam->ImageData()));


        // publish to ROS
        image_pub->publish(this->image_msg_, this->image_info_);
    }

    /////////////////////////////////////////////////
    void GazeboRosRealsense::OnNewDepthFrame()
    {
        // get current time
        common::Time current_time = this->world->GetSimTime();

        RealSensePlugin::OnNewDepthFrame();

        // copy data into image
        this->depth_msg_.header.seq++;
        this->depth_msg_.header.frame_id = "depth_optical_frame";
        this->depth_msg_.header.stamp.sec = current_time.sec;
        this->depth_msg_.header.stamp.nsec = current_time.nsec;
        this->depth_info_.header = this->depth_msg_.header;

        // set image encoding
        std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

        // copy from simulation image to ROS msg
        fillImage(this->depth_msg_,
            pixel_format,
            this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
            2 * this->depthCam->ImageWidth(),
            reinterpret_cast<const void*>(this->depthMap.data()));

        // publish to ROS
        this->depth_pub_.publish(this->depth_msg_, this->depth_info_);
    }

}
