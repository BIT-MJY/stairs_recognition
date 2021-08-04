#include <cmath>
#include <vector>
#include <string>
#include "stairs_recognition/common.h"
#include "stairs_recognition/tic_toc.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/passthrough.h> 
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ceres/ceres.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include<vector>

#include<mutex>

#define PI 3.1415926

using std::atan2;
using std::cos;
using std::sin;

double lidar_z_height = -1;




ros::Publisher pubForwardCloud;

struct LineFitCost
{
    LineFitCost(double x,double y):_x(x),_y(y){}
    template<typename T> 
    bool operator()(const T* const ab,T* residual)const
    {
        //y=ax+b
        residual[0]=T(_y)-(ab[0]*T(_x)+ab[1]);
        return true;

    }
    const double _x,_y;

};


void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{


    TicToc t_whole;
    TicToc t_prepare;


    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInROI(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

    for (int i=0; i<laserCloudIn->points.size(); i++)
    {
        double x = laserCloudIn->points[i].x;
        double y = laserCloudIn->points[i].y;
        double z = laserCloudIn->points[i].z;
        double r = std::sqrt(x*x+y*y+z*z);
        
        double theta = atan(y/x);
        
        if (theta < 0 && y > 0)
        {
            theta = theta + PI;
        }
        else if (theta > 0 && y < 0)
        {
            theta = theta - PI;
        }

        if (theta < 15*PI/180 && theta>-15*PI/180)
        // if (theta < 105*PI/180 && theta>75*PI/180)
        {
            laserCloudInROI->points.push_back(laserCloudIn->points[i]);
        }

    }

    sensor_msgs::PointCloud2 forwardCloudMsg;
    pcl::toROSMsg(*laserCloudInROI, forwardCloudMsg);
    forwardCloudMsg.header.stamp = laserCloudMsg->header.stamp;
    forwardCloudMsg.header.frame_id = "/rslidar";
    pubForwardCloud.publish(forwardCloudMsg);

    std::vector<double> x_data,y_data;
    for(int i=0;i<laserCloudInROI->points.size();++i)
    {
        x_data.push_back(laserCloudInROI->points[i].x);
        y_data.push_back(laserCloudInROI->points[i].z);
    }

    double ab[2]={};

    ceres::Problem problem;
    for(int i=0;i<laserCloudInROI->points.size();++i)
    {
        ceres::CostFunction *costfunction=new ceres::AutoDiffCostFunction<LineFitCost,1,2>(new LineFitCost(x_data[i],y_data[i]));

        problem.AddResidualBlock(costfunction,nullptr,ab);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";
    // cout<<"a,b:"<<endl;
    cout<<"final cost:"<<summary.final_cost<<endl;
    // for(auto i:ab)
    // {
    //     cout<<i<<endl;
    // }
    if (summary.final_cost < 10 && ab[0] <1 && ab[0] >0)
    {
        std::cout<< "Find the stairs"<<std::endl;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "collect_datasets");
    ros::NodeHandle nh;

    
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, laserCloudHandler);
    pubForwardCloud = nh.advertise<sensor_msgs::PointCloud2>("/forward_points", 1);


    ros::MultiThreadedSpinner spinner(1); // Use x threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return 0;
}
