#include <chrono>
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

        // auto start_time = std::chrono::high_resolution_clock::now();

        // 다운 샘플링 (voxel)
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*cloud_voxel_filtered);

        // x 축 필터링
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_voxel_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-5, 50);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pass_x.filter(*cloud_x_filtered);

        // y 축 필터링
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_x_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-1, 1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pass_y.filter(*cloud_y_filtered);

        // z 축 필터링
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud_y_filtered);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-1.7, 2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pass_z.filter(*cloud_z_filtered);

        // 지면 지우기 (RANSAC)
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.4);
        seg.setInputCloud(cloud_z_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            return;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_z_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_plane_removed);

        if (cloud_plane_removed->points.empty()) {
            return;
        }

        // 시각화 테스트용
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
        ec.setClusterTolerance(0.3);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(1000);
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

        // cluster publish
        cluster_pub.publish(cluster_centers);

        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration =
        // std::chrono::duration_cast<std::chrono::milliseconds>(
        //     end_time - start_time);
        // ROS_INFO("%lld milliseconds", duration.count());
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

    ros::Rate rate(20);
    ros::spin();
    return 0;
}
