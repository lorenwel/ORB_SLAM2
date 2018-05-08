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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

std::vector<std::string> file_paths = {
"/mnt/drive_c/datasets/beth_bags/2018_04_27_strickhof/asphalt_2018-04-27-11-23-40.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_27_strickhof/asphalt_2018-04-27-14-51-19.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_27_strickhof/asphalt_grass_dirt_2018-04-27-15-25-54.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_27_strickhof/dirt_2018-04-27-12-03-34.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_27_strickhof/dirt_2018-04-27-14-58-04.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_30_glattpark/asphalt_2018-04-30-11-38-56.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_30_glattpark/grass_2018-04-30-11-31-22.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_30_glattpark/grass_asphalt_2018-04-30-12-01-02.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_04_30_glattpark/gravel_2018-04-30-11-45-00.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_01_forest/forest_2018-05-01-14-27-05.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_01_forest/forest_2018-05-01-14-51-07.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_01_forest/forest_2018-05-01-15-01-59.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_glattpark/asphalt_2018-05-03-17-35-50.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_glattpark/grass_2018-05-03-17-04-33.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_glattpark/gravel_2018-05-03-17-26-37.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_strickhof/asphalt_2018-05-03-11-49-04.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_strickhof/asphalt_grass_2018-05-03-12-15-45.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_strickhof/dirt_2018-05-03-11-56-12.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_03_strickhof/grass_2018-05-03-12-08-13.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_04_glattpark/asphalt_sand_2018-05-04-15-05-34.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_04_glattpark/gravel_2018-05-04-14-52-25.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_04_glattpark/sand_2018-05-04-14-41-12.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_04_glattpark/sand_2018-05-04-14-46-14.bag", 
"/mnt/drive_c/datasets/beth_bags/2018_05_04_glattpark/sand_grass_2018-05-04-14-58-43.bag", 
};

void slamThread(char** argv, const std::string& path) {
    // Declare topics for ROSbag. 
    std::vector<std::string> topics= {"/realsense_zr300/ir/image_raw", 
                                      "/realsense_zr300/ir2/image_raw"};

    // Alter output file.
    std::string out_file = path;
    const auto str_size = out_file.length();
    out_file[str_size - 4] = '_';
    out_file[str_size - 3] = 's';
    out_file[str_size - 2] = 't';
    out_file[str_size - 1] = 'e';
    out_file.append("reo_");
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);

    ImageGrabber igb(&SLAM);

    // Read ROS bag.
    rosbag::Bag bag;
    std::cout << "Opening bag..." << std::endl;
    bag.open(path, rosbag::bagmode::Read);

    // Set up synchronizer.
    ros::NodeHandle nh(path);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, topics[0], 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, topics[1], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    const auto begin_time = view.getBeginTime();
    const auto end_time = view.getEndTime();
    const auto bag_duration = end_time - begin_time;

    // Process the bag. 
    for (const auto& msg: view) {
      std::cout << "Processing timestamp " << msg.getTime() - begin_time
                << "/" << bag_duration << "     \r";
      if (msg.getTopic() == topics[0]) {
        sync.add<0>(msg.instantiate<sensor_msgs::Image>());
      } else if (msg.getTopic() == topics[1]) {
        sync.add<1>(msg.instantiate<sensor_msgs::Image>());
      }
      // ros::WallDuration(0.1).sleep();
      if (!ros::ok()) break;
    }

    const auto tracking_state = SLAM.GetTrackingState();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (tracking_state > 1) {
        SLAM.SaveKeyFrameTrajectoryTUM(out_file + "keyframe.txt");
        SLAM.SaveTrajectoryTUM(out_file + "frame.txt");
    } else {
        std::cout << "Could not initialize ORB_SLAM and reached end of bag." << std::endl;
    }

    bag.close();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "StereoBag");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create vector with one thread per bag. 
    std::vector<std::thread> threads(file_paths.size());

    unsigned int counter = 0;
    for (const auto& path: file_paths) {
        threads[counter++] = std::thread(slamThread, argv, path);
    }

    // Join all threads. 
    for (auto&& thread: threads) {
        thread.join();
    }


    ros::shutdown();

    return 0;
}



void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}


