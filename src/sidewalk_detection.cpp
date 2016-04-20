#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";
static const double HEIGHT = 480.0;
static const double WIDTH  = 640.0;

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
        ImageConverter(): it_( nh_ ) {
            // Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe( "/camera/color/image_raw", 1000, &ImageConverter::imageCb, this );
            image_pub_ = it_.advertise( "/sidewalk_detector/color/image_raw", 1000 );
            cv::namedWindow( OPENCV_WINDOW );
        }
    
        ~ImageConverter() {
            cv::destroyWindow( OPENCV_WINDOW );
        }

        void imageCb( const sensor_msgs::ImageConstPtr& msg ) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::RGB8 );
            }
            catch ( cv_bridge::Exception& e ) {
                ROS_ERROR( "cv_bridge exception: %s", e.what() );
                return;
            }

            cv::flip( cv_ptr->image, cv_ptr->image, -1 );

            cv::Mat gray, edge, draw;
            cvtColor( cv_ptr->image, gray, CV_RGB2GRAY );
            blur( gray, edge, cv::Size(5,5) );          
            Canny( gray, edge, 50, 150, 3);

            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP( edge, lines, 1, CV_PI/180, 80, 20, 10 );
            double lp_x1, lp_y1, lp_x2, lp_y2;
            double rp_x1, rp_y1, rp_x2, rp_y2;
            double lmin = WIDTH/2, rmax = WIDTH/2;
            for( size_t i = 0; i < lines.size(); i++ )
            {
                // if (lines[i][2] == lines[i][0]) continue;
                double alpha = atan2((lines[i][3]-lines[i][1]), (lines[i][2]-lines[i][0]));
                double b = lines[i][3] - lines[i][2] * tan(alpha);             
                if ( lines[i][0] > 540 && lines[i][2] > 540 ) { // right part
                    if ( (alpha > 0.15*CV_PI/2) && (alpha < 0.85*CV_PI/2) ) {
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
                    if ( (alpha < -0.15*CV_PI/2) && (alpha > -0.85*CV_PI/2) ) {
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
            // After filter, two line is obtained, represented by
            // left : lp_x1, lp_y1, lp_x2, lp_y2;
            // right: rp_x1, rp_y1, rp_x2, rp_y2;
            cv::Point polygon[1][6];
            polygon[0][0] = cv::Point(0.0, lp_y1);
            polygon[0][1] = cv::Point(lp_x2, lp_y2);
            polygon[0][2] = cv::Point(rp_x2, rp_y2);
            polygon[0][3] = cv::Point(WIDTH, rp_y1);
            polygon[0][4] = cv::Point(WIDTH, HEIGHT);            
            polygon[0][5] = cv::Point(0.0, HEIGHT);
            const cv::Point* ppt[1] = { polygon[0] };
            int npt[] = { 6 };
            cv::fillPoly(cv_ptr->image, ppt, npt, 1, cv::Scalar(0,0,255));

            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);
            // Output modified video stream
            image_pub_.publish( cv_ptr->toImageMsg() );
        }
};

int main( int argc, char **args ) {
    // node : sidewalk_detection
    ros::init( argc, args, "sidewalk_detection" );
    ImageConverter ic;
    ros::spin();
    return 0;
}