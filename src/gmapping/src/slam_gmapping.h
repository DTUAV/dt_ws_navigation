/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "../../openslam_gmapping/include/gmapping/gridfastslam/gridslamprocessor.h"
#include "../../openslam_gmapping/include/gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>


#define   GMAPPING_FREE         (0)
#define   GMAPPING_UNKNOWN      (-1)
#define   GMAPPING_OCC          (100)

class SlamGMapping
{
  public:
    //构造与析构函数
    SlamGMapping();
    SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);//--
    SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer);//--
    ~SlamGMapping();
    //函数接口
    void init();
    //开始实时SLAM
    void startLiveSlam();
    void startReplay(const std::string & bag_fname, std::string scan_topic);
    //发布TF
    void publishTransform();
    //回调函数：订阅/scan
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);

  private:
    ros::NodeHandle node_;
    ros::Publisher entropy_publisher_;//熵发布
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    tf::TransformListener tf_;
    //利用消息滤波器去做时间同步
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;//坐标和激光雷达的数据时间要同步
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    tf::TransformBroadcaster* tfB_;//坐标变换发布器

    GMapping::GridSlamProcessor* gsp_;//slam处理器
    GMapping::RangeSensor* gsp_laser_;//激光雷达处理器
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;//激光雷达的数据
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    tf::Stamped<tf::Pose> centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;//翻转激光的范围，从左到右读取或者从右到左读取
    unsigned int gsp_laser_beam_count_;//激光点的个数
    GMapping::OdometrySensor* gsp_odom_;//里程计数据

    bool got_first_scan_; //是否获取第一帧激光雷达

    bool got_map_;//是否获取到地图
    nav_msgs::GetMap::Response map_;//地图的服务，返回地图

    ros::Duration map_update_interval_;//地图更新的时间
    tf::Transform map_to_odom_;//地图到里程计的坐标变换
    boost::mutex map_to_odom_mutex_;//线程互斥锁
    boost::mutex map_mutex_;//线程互斥锁

    int laser_count_; //激光雷达的个数? 3D?
    int throttle_scans_;//??

    boost::thread* transform_thread_;//坐标变换的线程, 用于发布坐标变换

    std::string base_frame_; //机器人底盘的坐标系统
    std::string laser_frame_;//激光雷达设备的坐标系统
    std::string map_frame_;//地图的坐标系统，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致
    std::string odom_frame_;//里程计的坐标系统
    /*
     * odom和map坐标系是不是重合的？ 机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。
     * 那map-->odom的tf怎么得到?就是在一些校正传感器合作校正的package比如amcl会给出一个位置估计（localization），
     * 这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。
     * 所以，如果你的odom计算没有错误，那么map-->odom的tf就是0.
    */

    void updateMap(const sensor_msgs::LaserScan& scan);//订阅激光雷达的数据去更新地图
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);//是否获取到里程计的位置
    bool getMapPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);//获取到地图的位置
    bool initMapper(const sensor_msgs::LaserScan& scan);//利用激光雷达的数据去初始化地图
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);//添加当前位置对应的激光雷达扫描信息
    double computePoseEntropy();
    
    // Parameters used by GMapping
    double maxRange_;  // 激光雷达的最大范围
    double maxUrange_; // 使用的激光雷达最大范围，要小于等于激光雷达的最大范围
    //double maxrange_;  //好像没有用到
    double minimum_score_; //最小的得分
    double sigma_;         //scan matching过程中的标准差(cell)
    int kernelSize_;       //scan matching过程中的搜索窗口大小
    double lstep_;         //scan matching过程中的位置增量
    double astep_;         //scan matching过程中的角度增量
    int iterations_;       //scan matching过程中的迭代优化次数
    double lsigma_;        //scan matching过程中的计算似然的标准差
    double ogain_;         //平滑似然的增益
    int lskip_;            //取激光束来计算匹配的起点，0代表从1开始取，也就是取全部 //跳过多少个激光点
    double srr_;           //由平移造成的平移误差//表示运动对定位和建图的影响
    double srt_;           //由平移造成的角度误差//表示运动对定位和建图的影响
    double str_;           //由旋转造成的平移误差//表示运动对定位和建图的影响
    double stt_;           //由旋转造成的角度误差//表示运动对定位和建图的影响
    double linearUpdate_;  //小车每走过linearUpdate,处理一次激光数据
    double angularUpdate_; //小车旋转angularUpdate_ / 12度，处理一次激光数据
    double temporalUpdate_;//小车保持不动的情况下，处理一次激光数据的时间间隔
    double resampleThreshold_;//有效粒子数阈值
    int particles_;           //粒子滤波器的粒子数
    double xmin_;             //地图X轴最小值
    double ymin_;             //地图Y轴最小值
    double xmax_;             //地图X轴最大值
    double ymax_;             //地图Y轴最大值
    double delta_;            //地图分辨率//一像素代表实际距离多少米
    double occ_thresh_;       //判定为障碍物的阈值
    double llsamplerange_;    //线性采样范围
    double llsamplestep_;     //线性采样步长
    double lasamplerange_;    //角度采样范围
    double lasamplestep_;     //角度采样步长
    
    ros::NodeHandle private_nh_;//私有句柄
    
    unsigned long int seed_; //随机数的种子
    
    double transform_publish_period_;//坐标变换的发布的频率
    double tf_delay_;//坐标变换的延迟
};
