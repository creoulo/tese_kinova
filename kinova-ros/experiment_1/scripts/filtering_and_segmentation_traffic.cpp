#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
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
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_geometry/pinhole_camera_model.h>
ros::Publisher pub_, pub_1, pub_2, pub_3, pub_4, pub_5, pub_6;

ros::Publisher clust_1, clust_2, clust_3, clust_4, clust_5;

void cylinder_Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    ros::NodeHandle nh("processing_node_traffic");
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
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_voxel;
    pcl::toROSMsg( *cloud_crop, output_voxel);
    clust_4.publish(output_voxel);

    /*--------------------------------------------------------------------------

                          Segmentation - Background extraction
    --------------------------------------------------------------------------*/

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

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_non;
    pcl::toROSMsg( *cloud_non_planar, output_non);
    clust_5.publish(output_non);
    /*--------------------------------------------------------------------------

                                Color Segmentation - green
    --------------------------------------------------------------------------*/
    //filter to extract blue game pieces
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_g (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_green_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);
    int r_max_g, r_min_g, g_max_g, g_min_g, b_max_g, b_min_g;
    nh.getParam("r_max_g", r_max_g);
    nh.getParam("r_min_g", r_min_g);
    nh.getParam("g_max_g", g_max_g);
    nh.getParam("g_min_g", g_min_g);
    nh.getParam("b_max_g", b_max_g);
    nh.getParam("b_min_g", b_min_g);
    color_cond_g->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, r_max_g)));
    color_cond_g->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, r_min_g)));
    color_cond_g->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, g_max_g)));
    color_cond_g->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, g_min_g)));
    color_cond_g->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, b_max_g)));
    color_cond_g->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, b_min_g)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_g(color_cond_g);
    condrem_g.setInputCloud (cloud_non_planar);
    condrem_g.setKeepOrganized(false);
    // apply filter
    condrem_g.filter (*cloud_green_cylinder);


    /*--------------------------------------------------------------------------

                                 Euclidean Cluster Extraction - green
    --------------------------------------------------------------------------*/
    int min_cluster_size_g, max_cluster_size_g;
    double cluster_tolerance_g;
    nh.getParam("min_cluster_size_g", min_cluster_size_g);
    nh.getParam("max_cluster_size_g", max_cluster_size_g);
    nh.getParam("cluster_tolerance_g", cluster_tolerance_g);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_g (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree_g->setInputCloud (cloud_green_cylinder);
    std::vector<pcl::PointIndices> cluster_indices_g;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_g;
    ec_g.setClusterTolerance (cluster_tolerance_g); // meters
    ec_g.setMinClusterSize (min_cluster_size_g);
    ec_g.setMaxClusterSize (max_cluster_size_g);
    ec_g.setSearchMethod (tree_g);
    ec_g.setInputCloud (cloud_green_cylinder);
    ec_g.extract (cluster_indices_g);


    Eigen::Vector4f centroid_g;
    std::string frame_id;
    nh.getParam("frame_id", frame_id);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output_g (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_g;
    experiment_1_msgs::InfoPieces pieces_positions_g;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_g.begin (); it != cluster_indices_g.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_g (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster_g->push_back ((*cloud_green_cylinder)[*pit]);
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster_g->header.stamp);
      cloud_cluster_g->header.frame_id = frame_id;
      cloud_cluster_g->width = cloud_cluster_g->size ();
      cloud_cluster_g->height = 1;
      cloud_cluster_g->is_dense = true;

      clusters_g.push_back(cloud_cluster_g);

      //to visualize
      *cloud_output_g += *cloud_cluster_g;
      cloud_output_g->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_output_g->header.stamp);


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
      pcl::compute3DCentroid( *cloud_cluster_g, centroid_g);
      float x_g = centroid_g.coeff(0,0);
      float y_g = centroid_g.coeff(1,0);
      float z_g = centroid_g.coeff(2,0);
      tf::Vector3 point_g(x_g, y_g, z_g);
      tf::Vector3 point_final_g = transform * point_g;
      experiment_1_msgs::List1 data_g;
      data_g.data.push_back(point_final_g.getX());
      data_g.data.push_back(point_final_g.getY());
      pieces_positions_g.poses.data.push_back(data_g);
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 outputg;
    pcl::toROSMsg( *cloud_output_g, outputg);
    clust_1.publish(outputg);
    pub_2.publish(pieces_positions_g);

    /*--------------------------------------------------------------------------

                                Color Segmentation - yellow
    --------------------------------------------------------------------------*/
    //filter to extract blue game pieces
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_y (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_yellow_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);
    int r_max_y, r_min_y, g_max_y, g_min_y, b_max_y, b_min_y;
    nh.getParam("r_max_y", r_max_y);
    nh.getParam("r_min_y", r_min_y);
    nh.getParam("g_max_y", g_max_y);
    nh.getParam("g_min_y", g_min_y);
    nh.getParam("b_max_y", b_max_y);
    nh.getParam("b_min_y", b_min_y);
    color_cond_y->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, r_max_y)));
    color_cond_y->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, r_min_y)));
    color_cond_y->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, g_max_y)));
    color_cond_y->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, g_min_y)));
    color_cond_y->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, b_max_y)));
    color_cond_y->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, b_min_y)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_y(color_cond_y);
    //condrem_y.setCondition(color_cond_y);
    condrem_y.setInputCloud (cloud_non_planar);
    condrem_y.setKeepOrganized(false);
    // apply filter
    condrem_y.filter (*cloud_yellow_cylinder);

    /*--------------------------------------------------------------------------

                                 Euclidean Cluster Extraction - yellow
    --------------------------------------------------------------------------*/
    int min_cluster_size_y, max_cluster_size_y;
    double cluster_tolerance_y;
    nh.getParam("min_cluster_size_y", min_cluster_size_y);
    nh.getParam("max_cluster_size_y", max_cluster_size_y);
    nh.getParam("cluster_tolerance_y", cluster_tolerance_y);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_y (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree_y->setInputCloud (cloud_yellow_cylinder);
    std::vector<pcl::PointIndices> cluster_indices_y;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_y;
    ec_y.setClusterTolerance (cluster_tolerance_y); // meters
    ec_y.setMinClusterSize (min_cluster_size_y);
    ec_y.setMaxClusterSize (max_cluster_size_y);
    ec_y.setSearchMethod (tree_y);
    ec_y.setInputCloud (cloud_yellow_cylinder);
    ec_y.extract (cluster_indices_y);


    Eigen::Vector4f centroid_y;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output_y (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_y;
    experiment_1_msgs::InfoPieces pieces_positions_y;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_y.begin (); it != cluster_indices_y.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_y (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster_y->push_back ((*cloud_yellow_cylinder)[*pit]);
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster_y->header.stamp);
      cloud_cluster_y->header.frame_id = frame_id;
      cloud_cluster_y->width = cloud_cluster_y->size ();
      cloud_cluster_y->height = 1;
      cloud_cluster_y->is_dense = true;

      clusters_y.push_back(cloud_cluster_y);

      //to visualize
      *cloud_output_y += *cloud_cluster_y;
      cloud_output_y->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_output_y->header.stamp);


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
      pcl::compute3DCentroid( *cloud_cluster_y, centroid_y);
      float x_y = centroid_y.coeff(0,0);
      float y_y = centroid_y.coeff(1,0);
      float z_y = centroid_y.coeff(2,0);
      tf::Vector3 point_y(x_y, y_y, z_y);
      tf::Vector3 point_final_y = transform * point_y;
      experiment_1_msgs::List1 data_y;
      data_y.data.push_back(point_final_y.getX());
      data_y.data.push_back(point_final_y.getY());
      pieces_positions_y.poses.data.push_back(data_y);
    }

    pub_1.publish(pieces_positions_y);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 outputy;
    pcl::toROSMsg( *cloud_output_y, outputy);
    clust_2.publish(outputy);
    /*--------------------------------------------------------------------------

                                Color Segmentation - red
    --------------------------------------------------------------------------*/
    //filter to extract blue game pieces
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_r (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);
    int r_max_r, r_min_r, g_max_r, g_min_r, b_max_r, b_min_r;
    nh.getParam("r_max_r", r_max_r);
    nh.getParam("r_min_r", r_min_r);
    nh.getParam("g_max_r", g_max_r);
    nh.getParam("g_min_r", g_min_r);
    nh.getParam("b_max_r", b_max_r);
    nh.getParam("b_min_r", b_min_r);
    color_cond_r->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, r_max_r)));
    color_cond_r->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, r_min_r)));
    color_cond_r->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, g_max_r)));
    color_cond_r->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, g_min_r)));
    color_cond_r->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, b_max_r)));
    color_cond_r->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, b_min_r)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_r(color_cond_r);
    //condrem_r.setCondition(color_cond_r);
    condrem_r.setInputCloud (cloud_non_planar);
    condrem_r.setKeepOrganized(false);
    // apply filter
    condrem_r.filter (*cloud_red_cylinder);


    /*--------------------------------------------------------------------------

                                 Euclidean Cluster Extraction - red
    --------------------------------------------------------------------------*/
    int min_cluster_size_r, max_cluster_size_r;
    double cluster_tolerance_r;
    nh.getParam("min_cluster_size_r", min_cluster_size_r);
    nh.getParam("max_cluster_size_r", max_cluster_size_r);
    nh.getParam("cluster_tolerance_r", cluster_tolerance_r);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_r (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree_r->setInputCloud (cloud_red_cylinder);
    std::vector<pcl::PointIndices> cluster_indices_r;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_r;
    ec_r.setClusterTolerance (cluster_tolerance_r); // meters
    ec_r.setMinClusterSize (min_cluster_size_r);
    ec_r.setMaxClusterSize (max_cluster_size_r);
    ec_r.setSearchMethod (tree_r);
    ec_r.setInputCloud (cloud_red_cylinder);
    ec_r.extract (cluster_indices_r);


    Eigen::Vector4f centroid_r;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output_r (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_r;
    experiment_1_msgs::InfoPieces pieces_positions_r;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_r.begin (); it != cluster_indices_r.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_r (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster_r->push_back ((*cloud_red_cylinder)[*pit]);
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster_r->header.stamp);
      cloud_cluster_r->header.frame_id = frame_id;
      cloud_cluster_r->width = cloud_cluster_r->size ();
      cloud_cluster_r->height = 1;
      cloud_cluster_r->is_dense = true;

      clusters_r.push_back(cloud_cluster_r);

      //to visualize
      *cloud_output_r += *cloud_cluster_r;
      cloud_output_r->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_output_r->header.stamp);


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
      pcl::compute3DCentroid( *cloud_cluster_r, centroid_r);
      float x_r = centroid_r.coeff(0,0);
      float y_r = centroid_r.coeff(1,0);
      float z_r = centroid_r.coeff(2,0);
      tf::Vector3 point_r(x_r, y_r, z_r);
      tf::Vector3 point_final_r = transform * point_r;
      experiment_1_msgs::List1 data_r;
      data_r.data.push_back(point_final_r.getX());
      data_r.data.push_back(point_final_r.getY());
      pieces_positions_r.poses.data.push_back(data_r);
    }

    pub_6.publish(pieces_positions_r);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 outputr;
    pcl::toROSMsg( *cloud_output_r, outputr);
    clust_3.publish(outputr);

}

bool compare_rect(const cv::RotatedRect& a, const cv::RotatedRect& b)
{
    if(std::abs(a.center.x-b.center.x) < 1 )
      return a.center.y > b.center.y;
    if(std::abs(a.center.y-b.center.y) < 1 )
      return a.center.x > b.center.x;
    return (a.center.x > b.center.x) && (a.center.y > b.center.y);
}

void board_Callback (const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ros::NodeHandle nh("processing_node_traffic");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    std::string frame_id;
    nh.getParam("frame_id", frame_id);
    tf::StampedTransform transform;
    tf::TransformListener tf_listener;
    try
    {
      tf_listener.waitForTransform( "/world", frame_id, ros::Time(), ros::Duration(5.0));
     }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
    }

    cv_bridge::CvImagePtr image_cv;
    cv::Mat src, src_gray, canny_out;
    cv::RNG rng(12345);
    try
    {
      image_cv = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    src = image_cv->image;

    cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );
    //blur( src_gray, src_gray, cv::Size(3,3) );

    int thresh;
    nh.getParam("thresh", thresh);
    cv::Canny( src_gray, canny_out, 0, thresh);
    cv::dilate(canny_out, canny_out, cv::Mat(), cv::Point(-1,-1));
    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contours_new;
    cv::findContours( canny_out, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    cv::Mat corners_out;
    int blockSize, apertureSize;
    double k;
    nh.getParam("blockSize", blockSize);
    nh.getParam("apertureSize", apertureSize);
    nh.getParam("k", k);
    cv::cornerHarris( src_gray, corners_out, blockSize, apertureSize, k );
    std::vector<cv::RotatedRect> minRect;
    std::vector<cv::Point> approx;
    double area;
    for( size_t i = 0; i < contours.size(); i++ )
    {
        double arcLength;
        nh.getParam("arcLength", arcLength);
        cv::approxPolyDP(contours[i], approx, arcLength*cv::arcLength(contours[i], true), true);
        if( approx.size() == 4 && cv::isContourConvex(cv::Mat(approx)) )//&& fabs(contourArea(approx)) > 1000)//4 points detected in contours detect a square
        {
          minRect.push_back(cv::minAreaRect(approx));//returns (center, size, angle)
          contours_new.push_back(approx);
        }
    }
    //remove outside square
    int pop;
    float size_0 = minRect[0].size.area();
    for (int i = 1; i < minRect.size() - 1; i++)
    {
        if (size_0 > minRect[i].size.area())
          pop = 0;
        else
          pop = i;
    }
    minRect.erase(minRect.begin()+pop);
    contours_new.erase(contours_new.begin()+pop);
    std::sort(minRect.begin(), minRect.end(), compare_rect);


    cv::Mat drawing = cv::Mat::zeros( canny_out.size(), CV_8UC3 );
    experiment_1_msgs::InfoBoard board_positions;
    for( size_t i = 0; i< contours_new.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        // contour
        cv::drawContours( drawing, contours_new, (int)i, color);
        //transform pixel into coordinates in the camera frame
        pcl::PointXYZRGB tmp = cloud->at(minRect[i].center.x, minRect[i].center.y);

        //identify each square in image
        cv::putText(drawing, std::to_string(i), minRect[i].center, 0, 0.5, color);
        //transform to world coordinates
        geometry_msgs::PointStamped point_tmp, final_point;
        point_tmp.header.stamp = ros::Time();
        point_tmp.header.frame_id = frame_id;
        point_tmp.point.x = tmp.x;
        point_tmp.point.y = tmp.y;
        point_tmp.point.z = tmp.z;
        tf_listener.transformPoint( "/world", point_tmp, final_point);

        experiment_1_msgs::Coordinates data;
        data.x = final_point.point.x;
        data.y = final_point.point.y;
        board_positions.coord.push_back(data);

        //std::cerr << "index: " << i << std::endl;
        //std::cerr << "center : " << final_point.point << std::endl;
    }
    pub_3.publish(board_positions);
    sensor_msgs::ImageConstPtr image_out;
    image_out =  cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
    pub_4.publish(image_out);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "piece_detection_node");
    ros::NodeHandle nh("processing_node_traffic");
    std::string cloud_topic;
    nh.getParam("cloud_topic", cloud_topic);
    std::string image_topic;
    nh.getParam("image_topic", image_topic);
    std::string cameraInfo_topic;
    nh.getParam("cameraInfo_topic", cameraInfo_topic);

    ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2> (cloud_topic, 1, cylinder_Callback);

    message_filters::Subscriber<sensor_msgs::Image> image_msg_(nh, image_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_msg_(nh, cloud_topic, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::PointCloud2> sync_( image_msg_, cloud_msg_, 10);
    sync_.registerCallback(boost::bind(&board_Callback, _1, _2));

    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output_plane",1);
    pub_3 = nh.advertise<experiment_1_msgs::InfoBoard> ("board_positions", 1);

    pub_4 = nh.advertise<sensor_msgs::Image> ("board_square_selection",1);
    pub_5 = nh.advertise<sensor_msgs::Image> ("boardCanny",1);

    pub_2 = nh.advertise<experiment_1_msgs::InfoPieces> ("pieces_positions_green", 1);
    pub_1 = nh.advertise<experiment_1_msgs::InfoPieces> ("pieces_positions_yellow", 1);
    pub_6 = nh.advertise<experiment_1_msgs::InfoPieces> ("pieces_positions_red", 1);

    clust_1 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_green",1);
    clust_2 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_yellow",1);
    clust_3 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_red",1);

    clust_4 = nh.advertise<sensor_msgs::PointCloud2> ("voxel_grid",1);
    clust_5 = nh.advertise<sensor_msgs::PointCloud2> ("non_planar",1);

    // Spin
    ros::spin ();
}
