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

// Extract normals by index
void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
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

void extractPrism(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_prism,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
    // Create the segmentation object for prism segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);

    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.1);

    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(10000);

    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.025);

    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0.01, 0.05);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
}

void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;

    outlier_filter.setInputCloud(cloud);
    outlier_filter.setMeanK(10);
    outlier_filter.setStddevMulThresh(2.5);
    outlier_filter.filter(*cloud);
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

    // // Grab normals of non-plane points
    // extractNormals(cloud_normals, inliers_plane);

    // Grab Cylinder
    // pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    // extractCylinder(cloud, coefficients_cylinder, cloud_normals);

    // Remove Outliers
    // removeOutliers(cloud);

    // Return error and remove planning scene cylinder if no cylinder found
    // else process the found cylinder cloud
    if (out->points.empty())
    {
        ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylinder!");
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

    pub = nh.advertise<pcl::PCLPointCloud2>("/pcl_test", 10);

    ros::spin();
}