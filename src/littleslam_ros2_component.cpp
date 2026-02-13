#include "littleslam_ros2/littleslam_ros2_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

namespace littleslam_ros2
{

Littleslam::Littleslam()
: Node("littleslam")
{
    use_odom_ = this->declare_parameter("use_odom", false);
    // get_parameter("use_odom", use_odom_);
    source_frame_ = this->declare_parameter("source_frame", "base_link");
    target_frame_ = this->declare_parameter("target_frame", "odom");
    topic_scan_ = this->declare_parameter("topic_scan", "scan");

    RCLCPP_INFO(this->get_logger(), "use_odom: %d", use_odom_);

    fc_.setSlamFrontEnd(sf_);
    fc_.makeFramework();
    if(use_odom_){
        fc_.customizeI();
    }
    else fc_.customizeG();
    auto scan_callback =
        [this](const typename sensor_msgs::msg::LaserScan::SharedPtr msg) -> void
        {
            Scan2D scan2d;
            if (make_scan2d(scan2d, msg)) sf_->process(scan2d);
        };

    laser_sub_  =
            create_subscription<sensor_msgs::msg::LaserScan>(topic_scan_, 100,
                scan_callback);

    icp_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("icp_map", 10);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);

    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);

    timer_ = create_wall_timer(100ms, [this]() { broadcast_littleslam(); });


    tfbuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*tfbuffer);
}

bool Littleslam::make_scan2d(Scan2D &scan2d, const sensor_msgs::msg::LaserScan::SharedPtr scan)
{

    if(use_odom_){
        // tf2_ros::Buffer tfbuffer(this->get_clock());
        // tf2_ros::TransformListener listener(tfbuffer);

        tf2::Stamped<tf2::Transform> tr;

        try{
            builtin_interfaces::msg::Time time_stamp = scan->header.stamp;
            tf2::TimePoint time_point = tf2::TimePoint(
                std::chrono::seconds(time_stamp.sec) +
                std::chrono::nanoseconds(time_stamp.nanosec));
            tf2::TimePoint time_out;

            // JW: debug
            double seconds = std::chrono::duration<double>(time_point.time_since_epoch()).count();
            std::cout<<"time_point: "<<seconds<<std::endl;
            std::cout<<"target_frame_: "<<target_frame_<<std::endl;
            std::cout<<"source_frame_: "<<source_frame_<<std::endl;

            std::cout<<"All frames: "<<tfbuffer->allFramesAsString()<<std::endl;

            if (tfbuffer->canTransform(target_frame_, source_frame_, time_point, tf2::durationFromSec(0.1))) {
                geometry_msgs::msg::TransformStamped tf = tfbuffer->lookupTransform(
                    target_frame_, source_frame_, time_point);
                
                

                tf2::fromMsg(tf, tr);
            }
            else{
                RCLCPP_WARN(this->get_logger(),"Cannot transform %s to %s at time %f",
                    source_frame_.c_str(), target_frame_.c_str(),
                    std::chrono::duration<double>(time_point.time_since_epoch()).count());
                return false;
            }
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
            return false;
        }
        scan2d.pose.tx = tr.getOrigin().x();
        scan2d.pose.ty = tr.getOrigin().y();
        scan2d.pose.th = RAD2DEG(tf2::impl::getYaw(tr.getRotation()));
        scan2d.pose.calRmat();
    }
    else{
        if(map_ == nullptr){
            sf_->process(scan2d);
            map_ = sf_->getPointCloudMap();
            }
        Pose2D curPose = map_->getLastPose();
        scan2d.pose = curPose;
    }

    scan2d.lps.clear();
    for(size_t i=0; i< scan->ranges.size(); ++i) {
        LPoint2D lp;
        double th = scan->angle_min + scan->angle_increment * i;
        double r = scan->ranges[i];
        if (scan->range_min < r && r < scan->range_max) {
            lp.x = r * cos(th); lp.y = r * sin(th);
            scan2d.lps.push_back(lp);
        }
    }

    return true;

}

void Littleslam::broadcast_littleslam()
{
    map_ = sf_->getPointCloudMap();

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

    msg->header.frame_id = target_frame_; //"map";
    msg->height = msg->width = 1;
    for (auto lp: map_->globalMap) msg->points.push_back(pcl::PointXYZ(lp.x, lp.y, 0));
    msg->width = msg->points.size();

    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg (*msg, cloud);
    icp_map_pub_->publish(cloud);

    nav_msgs::msg::Path path;
    path.header.frame_id = target_frame_; //"map";
    for(auto pos : map_->poses) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = pos.tx;
        pose.pose.position.y = pos.ty;
        pose.pose.position.z = 0;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, DEG2RAD(pos.th));
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg = tf2::toMsg(quat_tf);
        pose.pose.orientation = quat_msg;
        path.poses.push_back(pose);
        if(pos.tx == map_->lastPose.tx &&
                pos.ty == map_->lastPose.ty &&
                pos.th == map_->lastPose.th){
                pose.header.frame_id = target_frame_; //"map";
                current_pose_pub_->publish(pose);
            }
    }
    path_pub_->publish(path);
}

} // namespace littleslam_ros2

CLASS_LOADER_REGISTER_CLASS(littleslam_ros2::Littleslam, rclcpp::Node)
