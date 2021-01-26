#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <experiment_1_msgs/InfoPieces.h>

ros::Publisher pub, pub1, pub2;

void planeCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ros::NodeHandle nh("processing_node");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    /*--------------------------------------------------------------------------
                                  Filtering
    --------------------------------------------------------------------------*/
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_filter.setInputCloud (cloud);
    double leafSize;
    nh.getParam("voxel_leaf_size", leafSize);
    voxel_filter.setLeafSize ( leafSize, leafSize, leafSize);
    voxel_filter.filter (*cloud_voxel);

    //CropBox filter should replace your passthrough filters

    pcl::PointCloud<pcl::PointXYZRGB> xyz_filtered_cloud;
    pcl::CropBox<pcl::PointXYZRGB> crop;
    double x_filter_max, x_filter_min, y_filter_max, y_filter_min, z_filter_max, z_filter_min;
    nh.getParam("x_filter_max", x_filter_max);
    nh.getParam("x_filter_min", x_filter_min);
    nh.getParam("y_filter_max", y_filter_max);
    nh.getParam("y_filter_min", y_filter_min);
    nh.getParam("z_filter_max", z_filter_max);
    nh.getParam("z_filter_min", z_filter_min);

    crop.setInputCloud(cloud_voxel);
    Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    crop.filter(xyz_filtered_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>(xyz_filtered_cloud));


    /*--------------------------------------------------------------------------

                                 Segmentation
    --------------------------------------------------------------------------*/
    //Given the point normals and point indices, extract the normals for the indices.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_crop);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    // Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(1000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);
    segmentor.setInputCloud(cloud_crop);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    /*std::cerr << "Model plane coefficients: " << coefficients_plane->values[0] << " "
                                      << coefficients_plane->values[1] << " "
                                      << coefficients_plane->values[2] << " "
                                      << coefficients_plane->values[3] << std::endl;*/

    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices.setInputCloud(cloud_crop);
    extract_indices.setIndices(inliers_plane);
    /*extract the planar inliers*/
    extract_indices.setNegative(false);
    extract_indices.filter(*cloud_planar);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *cloud_planar, output);

    // Publish the data
    pub.publish(output);
}

void cylinderCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ros::NodeHandle nh("processing_node");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    /*--------------------------------------------------------------------------
                                  Filtering
    --------------------------------------------------------------------------*/
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_filter.setInputCloud (cloud);
    double leafSize;
    nh.getParam("voxel_leaf_size", leafSize);
    voxel_filter.setLeafSize ( leafSize, leafSize, leafSize);
    voxel_filter.filter (*cloud_voxel);

    //CropBox filter should replace your passthrough filters

    pcl::PointCloud<pcl::PointXYZRGB> xyz_filtered_cloud;
    pcl::CropBox<pcl::PointXYZRGB> crop;
    double x_filter_max, x_filter_min, y_filter_max, y_filter_min, z_filter_max, z_filter_min;
    nh.getParam("x_filter_max", x_filter_max);
    nh.getParam("x_filter_min", x_filter_min);
    nh.getParam("y_filter_max", y_filter_max);
    nh.getParam("y_filter_min", y_filter_min);
    nh.getParam("z_filter_max", z_filter_max);
    nh.getParam("z_filter_min", z_filter_min);

    crop.setInputCloud(cloud_voxel);
    Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    crop.filter(xyz_filtered_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>(xyz_filtered_cloud));


    /*--------------------------------------------------------------------------

                                 Segmentation
    --------------------------------------------------------------------------*/
    //Given the point normals and point indices, extract the normals for the indices.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_crop);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    // Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(1000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);
    segmentor.setInputCloud(cloud_crop);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    /*std::cerr << "Model plane coefficients: " << coefficients_plane->values[0] << " "
                                      << coefficients_plane->values[1] << " "
                                      << coefficients_plane->values[2] << " "
                                      << coefficients_plane->values[3] << std::endl;*/

    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_non_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices.setInputCloud(cloud_crop);
    extract_indices.setIndices(inliers_plane);
    /*remove the planar inliers, extract everything else*/
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud_non_planar);

    /*--------------------------------------------------------------------------

                                Color Segmentation
    --------------------------------------------------------------------------*/
    //filter to extract blue game pieces
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blue_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);
    int r_max, r_min, g_max, g_min, b_max, b_min;
    nh.getParam("r_max", r_max);
    nh.getParam("r_min", r_min);
    nh.getParam("g_max", g_max);
    nh.getParam("g_min", g_min);
    nh.getParam("b_max", b_max);
    nh.getParam("b_min", b_min);
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, r_max)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, r_min)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, g_max)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, g_min)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, b_max)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, b_min)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
    condrem.setInputCloud (cloud_non_planar);
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter (*cloud_blue_cylinder);

    /*--------------------------------------------------------------------------

                                 Euclidean Cluster Extraction
    --------------------------------------------------------------------------*/
    int min_cluster_size, max_cluster_size;
    double cluster_tolerance;
    nh.getParam("min_cluster_size", min_cluster_size);
    nh.getParam("max_cluster_size", max_cluster_size);
    nh.getParam("cluster_tolerance", cluster_tolerance);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_blue_cylinder);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (cluster_tolerance); // meters
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree1);
    ec.setInputCloud (cloud_blue_cylinder);
    ec.extract (cluster_indices);

    std::cerr << "Cluster indices size : " << cluster_indices.size() << std::endl;

    int j = 0;
    Eigen::Matrix<double, 4, 1> centroid;
    std::vector<Eigen::Matrix<double, 4, 1>> centroids;
    std::string frame_id;
    nh.getParam("frame_id", frame_id);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);
    experiment_1_msgs::InfoPieces pieces_positions;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_blue_cylinder)[*pit]);
      cloud_cluster->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster->header.stamp);
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      *cloud_output += *cloud_cluster;
      cloud_output->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster->header.stamp);

      // calculate centroid of each, cluster
      pcl::compute3DCentroid(*cloud_cluster, centroid);
      centroids.push_back(centroid);
      experiment_1_msgs::List1 data;
      data.data.push_back((float)centroid[0]);
      data.data.push_back((float)centroid[1]);
      pieces_positions.poses.data.push_back(data);
      //std::cerr << "Cluster Number: " << j << std::endl;
      //std::cerr << "Content: " << centroids[j] << std::endl;
      j++;
    }

    pub2.publish(pieces_positions);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *cloud_output, output);

    // Publish the data
    pub1.publish(output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "filtering_node");
    ros::NodeHandle nh("processing_node");
    std::string cloud_topic;
    nh.getParam("cloud_topic", cloud_topic);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (cloud_topic, 1, planeCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output_plane", 1);

    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2> (cloud_topic, 1, cylinderCallback);
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output_cylinders", 1);

    pub2 = nh.advertise<experiment_1_msgs::InfoPieces> ("pieces_positions", 1);

    // Spin
    ros::spin ();
}
