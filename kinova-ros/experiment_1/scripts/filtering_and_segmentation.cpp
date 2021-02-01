#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <experiment_1_msgs/InfoPieces.h>
#include <experiment_1_msgs/InfoBoard.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Publisher pub, pub1, pub2, pub3;

ros::Publisher clust1, clust2, clust3, clust4, clust5;

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


    /*// Filter isolated points according to the number of neighbors
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror_filter ;
    float radius_search;
    nh.getParam("radius_search", radius_search);
    ror_filter.setInputCloud(cloud_cylinder);
    ror_filter.setMinNeighborsInRadius(1);     // at least one neigbor
    ror_filter.setRadiusSearch(radius_search);  // search radius
    ror_filter.filter(*cloud_output);*/


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
    int k_nearest_neighbors;
    nh.getParam("k_nearest_neighbors", k_nearest_neighbors);
    ne.setKSearch(k_nearest_neighbors);
    ne.compute(*cloud_normals);

    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    // Create the segmentation object for the planar model and set all the parameters
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    // Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
    // create a SAC segmenter without using normals
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(1000);
    segmentor.setDistanceThreshold(0.005);
    segmentor.setInputCloud(cloud_crop);
    segmentor.setInputNormals(cloud_normals);
    segmentor.segment(*inliers_plane, *coefficients_plane);

    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices.setInputCloud(cloud_crop);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(false);
    extract_indices.filter(*cloud_plane);


    /*--------------------------------------------------------------------------

                                Color Segmentation
    --------------------------------------------------------------------------*/
    //filter to extract blue game pieces
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_board(new pcl::PointCloud<pcl::PointXYZRGB>);
    int r_max, r_min, g_max, g_min, b_max, b_min;
    nh.getParam("r_max_b", r_max);
    nh.getParam("r_min_b", r_min);
    nh.getParam("g_max_b", g_max);
    nh.getParam("g_min_b", g_min);
    nh.getParam("b_max_b", b_max);
    nh.getParam("b_min_b", b_min);
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, r_max)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, r_min)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, g_max)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, g_min)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, b_max)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, b_min)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
    condrem.setInputCloud (cloud_plane);
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter (*cloud_board);

    /*--------------------------------------------------------------------------

                                Centroid computation
    --------------------------------------------------------------------------*/
    Eigen::Vector4f centroid;
    std::string frame_id;
    nh.getParam("frame_id", frame_id);
    experiment_1_msgs::InfoBoard board_positions;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform("/world", frame_id, ros::Time(), ros::Duration(5.0));
      listener.lookupTransform("/world", frame_id, ros::Time(), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_WARN("Transform unavailable %s", ex.what());
    }
    pcl::compute3DCentroid( *cloud_board, centroid);
    float x = centroid.coeff(0,0);
    float y = centroid.coeff(1,0);
    float z = centroid.coeff(2,0);
    tf::Vector3 point(x, y, z);
    tf::Vector3 point_final = transform * point;
    float w = 0.1;
    float x_m[9] = {w, 0, -w, w, 0, -w, w, 0, -w};
    float y_m[9] = {-w, -w, -w, 0, 0, 0, w, w, w};
    x = point_final.getX();
    y = point_final.getY();
    for (int i = 0; i < 9; i++)
    {
      float x_tmp, y_tmp;
      experiment_1_msgs::Coordinates data;
      x_tmp = x + x_m[i];
      y_tmp = y + y_m[i];
      data.x = x_tmp;
      data.y = y_tmp;
      board_positions.coord.push_back(data);

    }
    pub3.publish(board_positions);

    //std::cerr << "Centroid from pcl : " << centroid << std::endl;
    //std::cerr << "Centroid : " << board_positions << std::endl;

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *cloud_board, output);

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

                          Segmentation - Background extraction
    --------------------------------------------------------------------------*/
    /*//Given the point normals and point indices, extract the normals for the indices.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_crop);
    // Set the number of k nearest neighbors to use for the feature estimation.
    int k_nearest_neighbors;
    nh.getParam("k_nearest_neighbors", k_nearest_neighbors);
    ne.setKSearch(k_nearest_neighbors);
    ne.compute(*cloud_normals);*/

    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor_plane;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor_plane.setOptimizeCoefficients(true);
    segmentor_plane.setModelType(pcl::SACMODEL_PLANE);
    segmentor_plane.setMethodType(pcl::SAC_RANSAC);
    segmentor_plane.setMaxIterations(1000);
    segmentor_plane.setDistanceThreshold(0.001);
    segmentor_plane.setInputCloud(cloud_crop);
    segmentor_plane.segment(*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_non_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices.setInputCloud(cloud_crop);
    extract_indices.setIndices(inliers_plane);
    //remove the planar inliers, extract everything else
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

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output1;
    pcl::toROSMsg( *cloud_blue_cylinder, output1);
    //pub.publish(output1);

    /*--------------------------------------------------------------------------

                                 Euclidean Cluster Extraction
    --------------------------------------------------------------------------*/
    int min_cluster_size, max_cluster_size;
    double cluster_tolerance;
    nh.getParam("min_cluster_size", min_cluster_size);
    nh.getParam("max_cluster_size", max_cluster_size);
    nh.getParam("cluster_tolerance", cluster_tolerance);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree1->setInputCloud (cloud_blue_cylinder);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (cluster_tolerance); // meters
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree1);
    ec.setInputCloud (cloud_blue_cylinder);
    ec.extract (cluster_indices);


    Eigen::Vector4f centroid;
    std::string frame_id;
    nh.getParam("frame_id", frame_id);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    experiment_1_msgs::InfoPieces pieces_positions;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_blue_cylinder)[*pit]);
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster->header.stamp);
      cloud_cluster->header.frame_id = frame_id;
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster);

      //to visualize
      *cloud_output += *cloud_cluster;
      cloud_output->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_output->header.stamp);


      tf::TransformListener listener;
      tf::StampedTransform transform;
      try
      {
        listener.waitForTransform("/world", frame_id, ros::Time(), ros::Duration(5.0));
        listener.lookupTransform("/world", frame_id, ros::Time(), transform);
      }
      catch(tf::TransformException ex)
      {
        ROS_WARN("Transform unavailable %s", ex.what());
      }
      // calculate centroid of each, cluster
      pcl::compute3DCentroid( *cloud_cluster, centroid);
      float x = centroid.coeff(0,0);
      float y = centroid.coeff(1,0);
      float z = centroid.coeff(2,0);
      tf::Vector3 point(x, y, z);
      tf::Vector3 point_final = transform * point;
      experiment_1_msgs::List1 data;
      data.data.push_back(point_final.getX());
      data.data.push_back(point_final.getY());
      pieces_positions.poses.data.push_back(data);
    }

    pub2.publish(pieces_positions);

    /*
    sensor_msgs::PointCloud2 cluster_out1;
    pcl::toROSMsg( *clusters[0], cluster_out1);
    clust1.publish(cluster_out1);
    sensor_msgs::PointCloud2 cluster_out2;
    pcl::toROSMsg( *clusters[1], cluster_out2);
    clust2.publish(cluster_out2);
    sensor_msgs::PointCloud2 cluster_out3;
    pcl::toROSMsg( *clusters[2], cluster_out3);
    clust3.publish(cluster_out3);
    sensor_msgs::PointCloud2 cluster_out4;
    pcl::toROSMsg( *clusters[3], cluster_out4);
    clust4.publish(cluster_out4);
    sensor_msgs::PointCloud2 cluster_out5;
    pcl::toROSMsg( *clusters[4], cluster_out5);
    clust5.publish(cluster_out5);
    */
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *cloud_output, output);

    // Publish the data
    pub1.publish(output);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "piece_detection_node");
    ros::NodeHandle nh("processing_node");
    std::string cloud_topic;
    nh.getParam("cloud_topic", cloud_topic);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (cloud_topic, 1, planeCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("output_plane",1);
    pub3 = nh.advertise<experiment_1_msgs::InfoBoard> ("board_positions", 1);

    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2> (cloud_topic, 1, cylinderCallback);

    pub2 = nh.advertise<experiment_1_msgs::InfoPieces> ("pieces_positions", 1);
    //ros::Duration(1).sleep();//to make sure no message gets lost

    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output_cylinders", 1);
    /*
    clust1 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_1",1);
    clust2 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_2",1);
    clust3 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_3",1);
    clust4 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_4",1);
    clust5 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_5",1);*/

    // Spin
    ros::spin ();
}
