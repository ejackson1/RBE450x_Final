#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

ros::Publisher pub;

// The following four functions were adapted from a tutorial.
// https://github.com/methylDragon/pcl-ros-tutorial/blob/master/PCL%20Reference%20with%20ROS.md#5-putting-it-together-cylinder-segmentation-example-integration-with-moveit-
// Modifications were made:
// 1. passThroughFilter accepts extra arguments, allowing for min and max z values to be parameterized
// 2. removePlaneSurface was modified to look for a normal plane instead of a cylinder
// 3. cloudCB was modified to use only a subset of the PCL processing functions
void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double z_min, double z_max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud);
}

void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);

    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(25);
    ne.compute(*cloud_normals);
}

void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
    // Find Plane
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);

    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    segmentor.setAxis(axis);
    segmentor.setMaxIterations(1000);
    segmentor.setDistanceThreshold(0.155);
    segmentor.setEpsAngle(0.09);
    segmentor.setNormalDistanceWeight(0.1);

    // Set input cloud and cloud normals
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Output plane
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);

    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);

    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
}

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    tf::TransformListener tf_listener;
    ros::Duration(2.0).sleep();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);

    tf::StampedTransform transform;

    tf_listener.lookupTransform("world", input->header.frame_id, ros::Time(0), transform);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    pcl_ros::transformPointCloud(*cloud, *out, transform);

    // Run Pass Through Filter
    passThroughFilter(out, 0.15, 1.5);

    // Find Normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(out, cloud_normals);

    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

    // Remove Plane Surface
    removePlaneSurface(out, cloud_normals, inliers_plane);

    // Return error and remove planning scene cylinder if no cylinder found
    // else process the found cylinder cloud
    if (out->points.empty())
    {
        ROS_ERROR_STREAM_NAMED("obj_segment", "Can't find the pudding box!");
        return;
    }
    else
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*out, cloud_msg);

        cloud_msg.header.frame_id = "world";

        pub.publish(cloud_msg);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "segment");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/voxel_grid/output", 10, cloudCB);

    pub = nh.advertise<pcl::PCLPointCloud2>("/segmented", 10);

    ros::spin();
}