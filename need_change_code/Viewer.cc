/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
// TTT
// ROS publishing for visualization
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// matrix / pose publish
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, bool bReuse):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mbReuse = bReuse;
}

void Viewer::Run()
{
    ros::NodeHandle nh;
    // Subscriber to external localization mode topic
    ros::Subscriber localizationModeSub = nh.subscribe<std_msgs::Bool>("/SLAM/localizationMode", 1, &Viewer::LocalizationModeCallback, this);
    
    featurePointQuantityMsg.data.resize(2);
    ros::Publisher featurePointQuantity = nh.advertise<std_msgs::UInt32MultiArray>("SLAM/FeaturePoint/Quantity", 0);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher featurePointImage = it.advertise("SLAM/FeaturePoint/Image", 0);
    
    // TTT
    // Publishers for external visualization
    ros::Publisher mapPointPub = nh.advertise<sensor_msgs::PointCloud2>("SLAM/MapPoints", 1);
    ros::Publisher keyframePub = nh.advertise<visualization_msgs::MarkerArray>("SLAM/KeyFrames", 1);
    // Publish current camera OpenGL 4x4 matrix (column-major as in pangolin::OpenGlMatrix)
    ros::Publisher cameraMatrixPub = nh.advertise<std_msgs::Float64MultiArray>("SLAM/CameraOpenGLMatrix", 1);
    // Publish camera pose (translation + quaternion) as PoseStamped in frame "map"
    ros::Publisher cameraPosePub = nh.advertise<geometry_msgs::PoseStamped>("SLAM/CameraPose", 1);
    // Also publish a simple PointStamped for RViz Point display convenience
    ros::Publisher cameraPointPub = nh.advertise<geometry_msgs::PointStamped>("SLAM/CameraPoint", 1);
    // TF broadcaster to publish a camera frame (map -> camera)
    tf::TransformBroadcaster tf_broadcaster;
    // TTT


    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuse,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = mbReuse;

    // Ensure initial external flag matches member
    {
        unique_lock<mutex> lock(mLocalizationMutex);
        // initialize if not set yet
        // mExternalLocalizationMode default false until callback sets it
    }

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        // First, process incoming ROS messages so callbacks update external flag
        ros::spinOnce();

        // If an external request was received, update the Pangolin UI var accordingly
        {
            unique_lock<mutex> lock(mLocalizationMutex);
            // Only update Pangolin control when external differs from current menu state
            // Note: Pangolin Var<bool> can be assigned like a regular bool
            if(mExternalLocalizationMode != static_cast<bool>(menuLocalizationMode))
            {
                menuLocalizationMode = mExternalLocalizationMode;
            }
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        auto featureQuantity = mpFrameDrawer->GetFeaturePointsQuantity();
        featurePointQuantityMsg.data[0] = featureQuantity.mnTracked;
        featurePointQuantityMsg.data[1] = featureQuantity.mnTrackedVO;
        featurePointQuantity.publish(featurePointQuantityMsg);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
        featurePointImage.publish(msg);

        // TTT
        // 发布 MapPoints 和 KeyFrames（节流，每 N 帧）
        static int pub_count = 0;
        const int PUB_EVERY_N = 10;
        pub_count++;
        if(pub_count >= PUB_EVERY_N)
        {
            pub_count = 0;
            // MapPoints -> PointCloud2
            std::vector<cv::Mat> vPWs = mpMapDrawer->GetAllMapPointPositions();
            if(!vPWs.empty())
            {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                cloud.points.reserve(vPWs.size());
                for(const cv::Mat &pw : vPWs)
                {
                    float x=0,y=0,z=0;
                    if(pw.type() == CV_32F)
                    {
                        x = pw.at<float>(0);
                        y = pw.at<float>(1);
                        z = pw.at<float>(2);
                    }
                    else
                    {
                        x = static_cast<float>(pw.at<double>(0));
                        y = static_cast<float>(pw.at<double>(1));
                        z = static_cast<float>(pw.at<double>(2));
                    }
                    cloud.push_back(pcl::PointXYZ(x,y,z));
                }
                sensor_msgs::PointCloud2 pcmsg;
                pcl::toROSMsg(cloud, pcmsg);
                pcmsg.header.stamp = ros::Time::now();
                pcmsg.header.frame_id = "map";
                mapPointPub.publish(pcmsg);
            }

            // KeyFrames -> MarkerArray (one marker per keyframe as a small sphere)
            std::vector<cv::Mat> vKFs = mpMapDrawer->GetAllKeyFramePoses();
            visualization_msgs::MarkerArray ma;
            int id = 0;
            for(const cv::Mat &pw : vKFs)
            {
                visualization_msgs::Marker m;
                m.header.frame_id = "map";
                m.header.stamp = ros::Time::now();
                m.ns = "keyframes";
                m.id = id++;
                m.type = visualization_msgs::Marker::SPHERE;
                m.action = visualization_msgs::Marker::ADD;
                float x=0,y=0,z=0;
                if(pw.type() == CV_32F)
                {
                    x = pw.at<float>(0);
                    y = pw.at<float>(1);
                    z = pw.at<float>(2);
                }
                else
                {
                    x = static_cast<float>(pw.at<double>(0));
                    y = static_cast<float>(pw.at<double>(1));
                    z = static_cast<float>(pw.at<double>(2));
                }
                m.pose.position.x = x;
                m.pose.position.y = y;
                m.pose.position.z = z;
                m.pose.orientation.w = 1.0;
                m.scale.x = 0.05;
                m.scale.y = 0.05;
                m.scale.z = 0.05;
                m.color.r = 0.0;
                m.color.g = 0.0;
                m.color.b = 1.0;
                m.color.a = 1.0;
                ma.markers.push_back(m);
            }
            if(!ma.markers.empty())
                keyframePub.publish(ma);
            
            // Publish current camera OpenGL matrix and Pose
            // Twc is a pangolin::OpenGlMatrix (column-major 4x4), elements accessible via Twc.m[]
            // We'll publish as Float64MultiArray with 16 elements in row-major order for easier consumption.
            std_msgs::Float64MultiArray mat_msg;
            mat_msg.data.resize(16);
            // pangolin::OpenGlMatrix stores internally in m (float m[16]) column-major.
            // Convert to row-major: mat[row*4 + col] = Twc.m[col*4 + row]
            for(int r=0;r<4;++r)
                for(int c=0;c<4;++c)
                    mat_msg.data[r*4 + c] = static_cast<double>(Twc.m[c*4 + r]);
            mat_msg.layout.dim.resize(2);
            mat_msg.layout.dim[0].label = "rows"; mat_msg.layout.dim[0].size = 4; mat_msg.layout.dim[0].stride = 16;
            mat_msg.layout.dim[1].label = "cols"; mat_msg.layout.dim[1].size = 4; mat_msg.layout.dim[1].stride = 4;
            mat_msg.layout.data_offset = 0;
            mat_msg.data.shrink_to_fit();
            cameraMatrixPub.publish(mat_msg);

            // Extract translation and rotation (as quaternion) from Twc
            // Twc corresponds to camera pose in world: OpenGL matrix mapping camera to world
            // Extract translation from the last column (in our conversion above it's at indices (0,3),(1,3),(2,3))
            double tx = mat_msg.data[3];
            double ty = mat_msg.data[7];
            double tz = mat_msg.data[11];
            // Rotation matrix R is upper-left 3x3 in mat_msg (row-major)
            // Build tf::Matrix3x3 and convert to quaternion
            tf::Matrix3x3 R(
                mat_msg.data[0], mat_msg.data[1], mat_msg.data[2],
                mat_msg.data[4], mat_msg.data[5], mat_msg.data[6],
                mat_msg.data[8], mat_msg.data[9], mat_msg.data[10]
            );
            tf::Quaternion q;
            R.getRotation(q);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = tx;
            pose_msg.pose.position.y = ty;
            pose_msg.pose.position.z = tz;
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            cameraPosePub.publish(pose_msg);
            // TTT

            // Publish PointStamped (same position) for RViz Point display
            geometry_msgs::PointStamped point_msg;
            point_msg.header = pose_msg.header;
            point_msg.point = pose_msg.pose.position;
            cameraPointPub.publish(point_msg);

            // Broadcast TF from "map" -> "camera"
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(tx, ty, tz));
            transform.setRotation(q);
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera"));
        }
        // TTT
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::LocalizationModeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    unique_lock<mutex> lock(mLocalizationMutex);
    mExternalLocalizationMode = msg->data;
}

}
