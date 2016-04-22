#include "ros/ros.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/PointIndices.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <math.h>
#include <vector>

using namespace std;
using namespace message_filters;

static const string OimageColorEdgeDetectionPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW = "Image window";
double HEIGHT = 480.0;
double WIDTH  = 640.0;

cv::Mat imageAcuting(cv_bridge::CvImagePtr cv_ptr);
vector<cv::Vec4i> edgeLineDetection(cv::Mat img);
void sidewalkEdgePicking(vector<cv::Vec4i> lines);
cv_bridge::CvImagePtr showImageWithMark(cv_bridge::CvImagePtr cv_ptr);
int inSideWalk(int index);
void imageColorEdgeDetection( const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& point_msg );

ros::Publisher image_pub_;
ros::Publisher pub_depth_in;
ros::Publisher pub_depth_out;

double lp_x1, lp_y1, lp_x2, lp_y2;
double rp_x1, rp_y1, rp_x2, rp_y2;

// imageColorEdgeDetection is using edge detection to detect the sidewalk
// the disadvantage is that if the edge is curve, it might not work well
void imageColorEdgeDetection( const sensor_msgs::ImageConstPtr& msg, 
                              const sensor_msgs::PointCloud2ConstPtr& point_msg ) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::RGB8 );
    }
    catch ( cv_bridge::Exception& e ) {
        ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
    }

    // acute the egde in the image
    cv::Mat imgResult = imageAcuting(cv_ptr);
    // detect the lines in the image
    vector<cv::Vec4i> lines = edgeLineDetection(imgResult);
    // choose the line which is most likely to be the sidewalk edge
    sidewalkEdgePicking(lines);
    // After filter, two line is obtained, represented by
    // left : lp_x1, lp_y1, lp_x2, lp_y2;
    // right: rp_x1, rp_y1, rp_x2, rp_y2;
    cv_ptr = showImageWithMark(cv_ptr);
    // publish processed image raw data
    image_pub_.publish( cv_ptr->toImageMsg() );

    // now deal with the point cloud
    sensor_msgs::PointCloud2 msg_in;
    sensor_msgs::PointCloud2 msg_out;
    // convert form sensor_msgs::PointCloud2 to PCLPointCloud2
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*point_msg, pcl_pc);
    // convert form PCLPointCloud2 to PCLPointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_in;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_out;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices ());
    for (int i = 0; i < WIDTH * HEIGHT * 0.75 * 0.75; i++) {
        if (inSideWalk(i) == 1) {
            inliers->indices.push_back(i);
        } else {
            outliers->indices.push_back(i);
        }
    }
    extract_in.setInputCloud(cloud);
    extract_in.setIndices(inliers);
    extract_in.setNegative(false);
    extract_in.filter(*cloud_in);
    extract_out.setInputCloud(cloud);
    extract_out.setIndices(outliers);
    extract_out.setNegative(false);
    extract_out.filter(*cloud_out);
    // // convert form PCLPointCloud to sensor_msgs::PointCloud2
    pcl::toROSMsg(*cloud_in, msg_in);
    pcl::toROSMsg(*cloud_out, msg_out);
    // publich topic
    pub_depth_in.publish(msg_in);
    pub_depth_out.publish(msg_out);
}

cv::Mat imageAcuting(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
        1,  1, 1,
        1, -8, 1,
        1,  1, 1);
    // do the laplacian filtering to acute the edge
    cv::Mat imgLaplacian;
    cv::Mat sharp = cv_ptr->image;
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    cv_ptr->image.convertTo(sharp, CV_32F);
    cv::Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    return imgResult;
}

vector<cv::Vec4i> edgeLineDetection(cv::Mat img) {
    cv::Mat gray, edge;
    cv::cvtColor( img, gray, CV_RGB2GRAY );
    // cv::blur( gray, edge, cv::Size(5,5) );          
    cv::Canny( gray, edge, 50, 150, 3);
    vector<cv::Vec4i> lines;
    cv::HoughLinesP( edge, lines, 1, CV_PI/180, 80, 20, 10 );
    return lines;
}

void sidewalkEdgePicking(vector<cv::Vec4i> lines) {
    double lmin = WIDTH/2, rmax = WIDTH/2;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        // if (lines[i][2] == lines[i][0]) continue;
        double alpha = atan2((lines[i][3]-lines[i][1]), (lines[i][2]-lines[i][0]));
        double b = lines[i][3] - lines[i][2] * tan(alpha);           
        if ( lines[i][0] > 540 && lines[i][2] > 540 ) { // right part
            if ( (alpha < -0.15*CV_PI/2) && (alpha > -0.85*CV_PI/2) ) {
                double x11, y11;
                double x21 = (HEIGHT/2 - b) / tan(alpha);
                double y21 = HEIGHT /2 ;
                if (x21 < rmax) {
                    continue;
                }
                rmax = x21;
                rp_x2 = x21;
                rp_y2 = y21;
                if (tan(alpha) * WIDTH + b <= HEIGHT) {
                    rp_x1 = WIDTH;
                    rp_y1 = tan(alpha) * rp_x1 + b;
                } else {
                    rp_x1 = (HEIGHT - b) / tan(alpha);
                    rp_y1 = HEIGHT;
                }
            }
        } else if ( lines[i][0] < 100 && lines[i][2] < 100 ) { // left part
            if ( (alpha > 0.15*CV_PI/2) && (alpha < 0.85*CV_PI/2) ) {
                double x12, y12;
                double x22 = (HEIGHT/2 - b) / tan(alpha);
                double y22 = HEIGHT/2;
                if (x22 > lmin) {
                    continue;
                }
                lmin = x22;
                lp_x2 = x22;
                lp_y2 = y22;
                if (b <= HEIGHT) {
                    lp_x1 = 0.0;
                    lp_y1 = b;
                } else {
                    lp_x1 = (HEIGHT - b) / tan(alpha);
                    lp_y1 = HEIGHT;
                }
            }
        }
    }
}

cv_bridge::CvImagePtr showImageWithMark(cv_bridge::CvImagePtr cv_ptr) {
    cv::Point polygon[1][6];
    polygon[0][0] = cv::Point(0.0, lp_y1);
    polygon[0][1] = cv::Point(lp_x2, lp_y2);
    polygon[0][2] = cv::Point(rp_x2, rp_y2);
    polygon[0][3] = cv::Point(WIDTH, rp_y1);
    polygon[0][4] = cv::Point(WIDTH, 0);            
    polygon[0][5] = cv::Point(0.0, 0.0);
    const cv::Point* ppt[1] = { polygon[0] };
    int npt[] = { 6 };
    cv::fillPoly(cv_ptr->image, ppt, npt, 1, cv::Scalar(0,0,255));

    cv::flip( cv_ptr->image, cv_ptr->image, -1 );
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
    return cv_ptr;
}

int inSideWalk(int index) {
    index = index / (0.75 * 0.75);
    double alphal = atan2((lp_y2 - lp_y1), (lp_x2 - lp_x1));
    double bl = lp_y2 - lp_x2 * tan(alphal);
    double alphar = atan2((rp_y2 - rp_y1), (rp_x2 - rp_x1));
    double br = rp_y2 - rp_x2 * tan(alphar);
    uint32_t x = index % (int)WIDTH;
    uint32_t y = index / (int)WIDTH;
    if ( (y < lp_y2) && (y > 0) && (x < WIDTH) && (x > 0)
        && (y < tan(alphal)*x + bl) && (y < tan(alphar)*x + br) ) {
        return 1; // it means this point is in sidewalk
    }
    return 0; // it means this point is not in sidewalk
}


        // imageSegment is using image segmentation to detect sidewalk
        // now still not work well, still need to test
/*        void imageSegment( const sensor_msgs::ImageConstPtr& msg ) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::RGB8 );
            }
            catch ( cv_bridge::Exception& e ) {
                ROS_ERROR( "cv_bridge exception: %s", e.what() );
                return;
            }
            cv::flip( cv_ptr->image, cv_ptr->image, -1 );
            // Create a kernel that we will use for accuting/sharpening our image
            cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                1,  1, 1,
                1, -8, 1,
                1,  1, 1);
            // do the laplacian filtering
            cv::Mat imgLaplacian;
            cv::Mat sharp = cv_ptr->image; // copy source image to another temporary one
            filter2D(sharp, imgLaplacian, CV_32F, kernel);
            cv_ptr->image.convertTo(sharp, CV_32F);
            cv::Mat imgResult = sharp - imgLaplacian;
            // convert back to 8bits gray scale
            imgResult.convertTo(imgResult, CV_8UC3);
            imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
            // cv::imshow( OPENCV_WINDOW, imgResult );
            cv_ptr->image = imgResult;

            // Create binary image from source image
            cv::Mat bw;
            cv::cvtColor(cv_ptr->image, bw, CV_RGB2HSV);
            std::vector<cv::Mat> channels;
            cv::split(bw, channels);
            bw = channels[0];
            // cv::cvtColor(cv_ptr->image, bw, CV_RGB2GRAY);
            cv::threshold(bw, bw, 20, 120, CV_THRESH_BINARY | CV_THRESH_OTSU);
            // cv::imshow( OPENCV_WINDOW, bw );
            
            // Perform the distance transform algorithm
            cv::Mat dist;
            cv::distanceTransform(bw, dist, CV_DIST_L2, 3);
            // Normalize the distance image for range = {0.0, 1.0}
            // so we can visualize and threshold it
            cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
            // cv::imshow(OPENCV_WINDOW, dist);
            // Threshold to obtain the peaks
            // This will be the markers for the foreground objects
            cv::threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
            // Dilate a bit the dist image
            cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
            cv::dilate(dist, dist, kernel1);
            // cv::imshow(OPENCV_WINDOW, dist);
            // Create the CV_8U version of the distance image
            // It is needed for findContours()
            cv::Mat dist_8u;
            dist.convertTo(dist_8u, CV_8U);
            // Find total markers
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            // Create the marker image for the watershed algorithm
            cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32SC1);
            // Draw the foreground markers
            for (size_t i = 0; i < contours.size(); i++)
                cv::drawContours(markers, contours, static_cast<int>(i), cv::Scalar::all(static_cast<int>(i)+1), -1);
            // Draw the background marker
            cv::circle(markers, cv::Point(5,5), 3, CV_RGB(255,255,255), -1);
            // cv::imshow(OPENCV_WINDOW, markers*10000);

            // Perform the watershed algorithm
            cv::watershed(cv_ptr->image, markers);
            cv::Mat mark = cv::Mat::zeros(markers.size(), CV_8UC1);
            markers.convertTo(mark, CV_8UC1);
            cv::bitwise_not(mark, mark);
            // Generate random colors
            std::vector<cv::Vec3b> colors;
            for (size_t i = 0; i < contours.size(); i++)
            {
                int b = cv::theRNG().uniform(0, 255);
                int g = cv::theRNG().uniform(0, 255);
                int r = cv::theRNG().uniform(0, 255);
                colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
            }
            // Create the result image
            cv::Mat dst = cv::Mat::zeros(markers.size(), CV_8UC3);
            // Fill labeled objects with random colors
            for (int i = 0; i < markers.rows; i++)
            {
                for (int j = 0; j < markers.cols; j++)
                {
                    int index = markers.at<int>(i,j);
                    if (index > 0 && index <= static_cast<int>(contours.size()))
                        dst.at<cv::Vec3b>(i,j) = colors[index-1];
                    else
                        dst.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
                }
            }
            // Visualize the final image
            imshow(OPENCV_WINDOW, dst);

            // cv::imshow( OPENCV_WINDOW, cv_ptr->image );  
            cv::waitKey(3);
            image_pub_.publish( cv_ptr->toImageMsg() );
        }  */
// };

int main( int argc, char **args ) {
    // node : sidewalk_detection
    ros::init( argc, args, "sidewalk_detection" );
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> sub_image(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points(nh, "/camera/depth/points", 1);

    image_pub_ = nh.advertise<sensor_msgs::Image>( "/sidewalk_detector/color/image_raw", 1 );
    pub_depth_in = nh.advertise<sensor_msgs::PointCloud2>("/sidewalk_detector/depth/points_in", 1);
    pub_depth_out = nh.advertise<sensor_msgs::PointCloud2>("/sidewalk_detector/depth/points_out", 1);
    typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_points);
    // TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, sub_point, 10);
    sync.registerCallback(boost::bind(&imageColorEdgeDetection, _1, _2));
    ros::spin();
    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();
    return 0;
}