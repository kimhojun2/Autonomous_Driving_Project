#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/common/centroid.h> // For compute3DCentroid
#include <pcl/common/common.h>   // For basic PCL types
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class SCANCluster {
  public:
    SCANCluster() {
        this->nh = ros::NodeHandle();
        this->cluster_pub =
            nh.advertise<geometry_msgs::PoseArray>("clusters", 10);
        this->scan_sub =
            nh.subscribe("/velodyne_points", 10, &SCANCluster::callback, this);
        this->roi_pub =
            nh.advertise<sensor_msgs::PointCloud2>("velodyne_points_roi", 10);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &input) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_removed(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*input, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        vg.filter(*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-10, 10);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-20, 100);

        pass.filter(*cloud_passthrough_filtered);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.3);
        seg.setInputCloud(cloud_passthrough_filtered);
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_passthrough_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_plane_removed);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_plane_removed, output);
        output.header.frame_id = input->header.frame_id;
        output.header.stamp = ros::Time::now();
        roi_pub.publish(output);

        // Clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_plane_removed);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.2); // 20cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_plane_removed);
        ec.extract(cluster_indices);

        geometry_msgs::PoseArray cluster_centers;
        cluster_centers.header.frame_id = input->header.frame_id;
        cluster_centers.header.stamp = ros::Time::now();

        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &idx : it->indices)
                cloud_cluster->push_back((*cloud_plane_removed)[idx]);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);

            geometry_msgs::Pose pose;
            pose.position.x = centroid[0];
            pose.position.y = centroid[1];
            pose.position.z = centroid[2];
            cluster_centers.poses.push_back(pose);
        }

        cluster_pub.publish(cluster_centers);
    }

  private:
    ros::NodeHandle nh;
    ros::Publisher cluster_pub;
    ros::Publisher roi_pub;
    ros::Subscriber scan_sub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_clustering");
    SCANCluster scanCluster;
    ros::spin();
    return 0;
}
