// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk
// kittiHelper 将kitti数据集转化为ROS的形式
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

std::vector<float> read_lidar_data(const std::string lidar_data_path) //读取雷达数据
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper"); //初始化ros node
    ros::NodeHandle n("~"); //将命名空间重新定义为～
    std::string dataset_folder, sequence_number, sequence_number; //初始化dataset_folder sequence_number sequence_number 的变量类型，变量类型为string
    
    //使用ROS的获取参数函数，获取dataset和sequence——num的参数
    n.getParam("dataset_folder", dataset_folder); 
    n.getParam("sequence_number", sequence_number);

    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    
    //初始化一个逻辑变量 to_bag
    bool to_bag;
    n.getParam("to_bag", to_bag); //获取参数
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);


    int publish_delay; //发布延时
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay; //判断型语句，如果publish_delay<=0,则将publish_delay置为0,否则不变

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2); //初始化点云发布话题，话题名为/velodyne_points. 发布频率为2

    image_transport::ImageTransport it(n);//初始化图像发布节点ImageTransport,在ROS的cpp中，一般使用ImageTransport进行图像的发布
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2); //初始化左侧图像话题
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2); //初始化右侧图像话题

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5); //初始化里程计真值
    nav_msgs::Odometry odomGT; //初始化真值变量类型
    odomGT.header.frame_id = "/camera_init"; //进行frameid和子frameid的定义
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5); //定义轨迹发布话题
    nav_msgs::Path pathGT; //初始化Path pathGT的数据类型
    pathGT.header.frame_id = "/camera_init"; //定义frame id

    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt"; //读取时间戳文件路径
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in); //读取文件

    std::string ground_truth_path = "results/" + sequence_number + ".txt";//初始化地面真值信息的数据类型
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in); //读取地面真值消息

    rosbag::Bag bag_out; //初始化rosbag类型
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    
    Eigen::Matrix3d R_transform; //初始化旋转矩阵 Rotation matrix 类型
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0; //赋值
    Eigen::Quaterniond q_transform(R_transform); //？这里不知道含义

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay); //根据publish_delay设置发布频率
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line); //将string类型转化为 float类型， stof() string->float 
        std::stringstream left_image_path, right_image_path; 
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png"; //读取图片
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE); //转化为灰度图像
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

        std::getline(ground_truth_file, line); //获取地面真值
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose; //初始化gt_pose
        
        //对真值矩阵中的每一个元素使用读取的pose_stream进行赋值
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }

        //使用相机的变换矩阵对相机的图像参数进行标定
        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;//获取图像的最终变换矩阵
        q.normalize();//进行归一化
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>(); //进行平移


        //初始化odomGT的ROS参数
        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT); //发布地面真值话题


        //初始化poseGT话题
        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT); //发布poseGT
        pubPathGT.publish(pathGT); //发布pathGT

        // read lidar point cloud， 读取点云
        std::stringstream lidar_data_path; //初始化点云路径
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin"; //读取点云路径
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str()); //读取点云
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n"; //输出一下点云信息，对点云信息归一化一下


        //这里为什么要设立lidar_points和lidar_intensities两个变量来表示点云之后再初始化一个pcl的点云类型呢？
        std::vector<Eigen::Vector3d> lidar_points; //初始化一个三维空间向量，表示点云的点的位置
        std::vector<float> lidar_intensities; //雷达的反射强度，根据雷达的反射强度，可以进行滤波等操作
        pcl::PointCloud<pcl::PointXYZI> laser_cloud; //初始化点云类型
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]); //点云泪痣
            lidar_intensities.push_back(lidar_data[i+3]); //点云强度

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg; //进行ROS发布的初始化
        pcl::toROSMsg(laser_cloud, laser_cloud_msg); //从pcl转化到ROS类型
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp); //获取时间戳
        laser_cloud_msg.header.frame_id = "/camera_init"; 
        pub_laser_cloud.publish(laser_cloud_msg); //发布点云话题


        //发布左侧和右侧的图像话题
        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);

        if (to_bag)
        {
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/image_right", ros::Time::now(), image_right_msg);
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
        }

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n"; //发布结束


    return 0;
}