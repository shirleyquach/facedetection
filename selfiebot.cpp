#include "ros/ros.h"
#include <ctime>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>

#define COOLDOWN_TIMER 10
#define COUNTDOWN_TIMER 3

using namespace std;
using namespace cv;

/** Global variables */
static const std::string OPENCV_WINDOW = "Image window";
static time_t timer = 0;
int cooldown;
Mat frame;
String face_cascade_name = "/home/harveyu/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "/home/harveyu/opencv/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("camera/visible/image", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("image", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    bool hasFace()
    {
        std::vector<Rect> faces;
        Mat frame_gray;
        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
        return faces.size() > 0;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat outputImage, ret;
        frame = cv_ptr->image;
        frame.copyTo(ret);
        //-- 1. Load the cascades
        if( !face_cascade.load( face_cascade_name ) ){ ROS_ERROR("--(!)Error loading\n"); return; };
        if( !eyes_cascade.load( eyes_cascade_name ) ){ ROS_ERROR("--(!)Error loading\n"); return; };

        //-- 2. Apply the classifier to the frame
        if( !frame.empty() )
        {
            if ( cooldown )
            {
                if ( difftime(time(NULL), timer) >= COOLDOWN_TIMER )
                {
                    cooldown = 0;
                }
            }
            else if ( hasFace() )
            {
                if ( !timer )
                {
                    time(&timer);
                }
                else if ( difftime(time(NULL), timer) >= COUNTDOWN_TIMER )
                {
                    ROS_INFO("CHEEEEEEEEEEEEEEEEEEEESE!!!");
                    cvtColor( frame, outputImage, CV_BGR2RGB );
                    imwrite("/home/harveyu/catkin_ws/src/selfiebot/picture.jpg", outputImage);
                    time(&timer);
                    cooldown = 1;
                }
            }
            else
            {
                timer = 0;
            }
        }
        else
        { ROS_ERROR(" --(!) No captured frame -- Break!"); return;}

        cv_bridge::CvImage out;
        out.header = msg->header;
        out.encoding = msg->encoding;
        out.image = ret;
        image_pub_.publish(out.toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "selfiebot");
    ros::NodeHandle n;
    ImageConverter ic;

    ros::spin();
}
