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
ros::Publisher pub, pub1, pub2, pub3, pub4, pub5;

ros::Publisher clust1, clust2, clust3, clust4, clust5;

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

    sensor_msgs::PointCloud2 output_crop;
    pcl::toROSMsg( *cloud_crop, output_crop);
    clust1.publish(output_crop);
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

    sensor_msgs::PointCloud2 output_non;
    pcl::toROSMsg( *cloud_non_planar, output_non);
    clust2.publish(output_non);
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
    clust3.publish(output1);

    /*--------------------------------------------------------------------------

                                 Euclidean Cluster Extraction
    --------------------------------------------------------------------------*/
    int min_cluster_size, max_cluster_size;
    double cluster_tolerance;
    nh.getParam("min_cluster_size", min_cluster_size);
    nh.getParam("max_cluster_size", max_cluster_size);
    nh.getParam("cluster_tolerance", cluster_tolerance);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_blue_cylinder);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (cluster_tolerance); // meters
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
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

bool compare_rect(const cv::RotatedRect& a, const cv::RotatedRect& b)
{
    if(std::abs(a.center.x-b.center.x) < 1 )
      return a.center.y > b.center.y;
    if(std::abs(a.center.y-b.center.y) < 1 )
      return a.center.x > b.center.x;
    return (a.center.x > b.center.x) && (a.center.y > b.center.y);
}

void boardCallback (const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ros::NodeHandle nh("processing_node");
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
    sensor_msgs::ImageConstPtr image_o;
    image_o =  cv_bridge::CvImage(std_msgs::Header(), "mono8", canny_out).toImageMsg();
    pub5.publish(image_o);
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
    //remove biggest square
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
    pub3.publish(board_positions);
    sensor_msgs::ImageConstPtr image_out;
    image_out =  cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
    pub4.publish(image_out);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "piece_detection_node");
    ros::NodeHandle nh("processing_node");
    std::string cloud_topic;
    nh.getParam("cloud_topic", cloud_topic);
    std::string image_topic;
    nh.getParam("image_topic", image_topic);
    std::string cameraInfo_topic;
    nh.getParam("cameraInfo_topic", cameraInfo_topic);

    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2> (cloud_topic, 1, cylinderCallback);

    message_filters::Subscriber<sensor_msgs::Image> image_msg(nh, image_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_msg(nh, cloud_topic, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::PointCloud2> sync( image_msg, cloud_msg, 10);
    sync.registerCallback(boost::bind(&boardCallback, _1, _2));

    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output_plane",1);
    pub3 = nh.advertise<experiment_1_msgs::InfoBoard> ("board_positions", 1);

    pub4 = nh.advertise<sensor_msgs::Image> ("board_square_selection",1);
    pub5 = nh.advertise<sensor_msgs::Image> ("boardCanny",1);

    pub2 = nh.advertise<experiment_1_msgs::InfoPieces> ("pieces_positions", 1);
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output_cylinders", 1);


    clust1 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_crop",1);
    clust2 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_non_planar",1);
    clust3 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_blue",1);

    // Spin
    ros::spin ();
}
