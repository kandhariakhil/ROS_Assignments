//get images from topic "simple_camera/image_raw"; remap, as desired;
//search for red pixels;
// convert (sufficiently) red pixels to white, all other pixels black
// compute centroid of red pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <math.h>

//Include Eigen libraries for performing math operations for transformations.
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

int g_redratio; //threshold to decide if a pixel qualifies as dominantly "red"

const double BLOCK_HEIGHT=0.035; // hard-coded top surface of block relative to world frame
const  double scale_factor = 0.003; //Scale factor
const double init_x = 0.548; //320-> x_center
const double init_y = 0.315; //240 -> y_center
const double theta_cam = -0.196; //in radians


class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("block_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = BLOCK_HEIGHT;
        //block_pose_.pose.position.x = 0.5; //not true, but legal
        //block_pose_.pose.position.y = 0.0; //not true, but legal
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Eigen::Matrix4d T_cam_robot, T_block_cam, T_block_robot;
        Eigen::VectorXf red_pixels_x, red_pixels_y;
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int redval, blueval, greenval, testval;
        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel
        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = redval / (blueval + greenval);
                //if red (enough), paint this white:
                if (testval > g_redratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    
                    //Save all i and j values as vector matrices to find covariance:
                    red_pixels_x.conservativeResize(npix+1);
                    red_pixels_y.conservativeResize(npix+1);
                    red_pixels_x(npix)=i;
                    red_pixels_y(npix)=j;                 
                    //Declare the covariance space here
                    //Find covariance using math
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                    npix++; //note that found another red pixel
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }

        
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid, i_centroid_mean, j_centroid_mean;
        if (npix > 0) {
            i_centroid_mean = red_pixels_x.mean();
            j_centroid_mean = red_pixels_y.mean();
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);
            for(int i=0;i<npix;i++) {
                red_pixels_x(i) = red_pixels_x(i)-i_centroid_mean;
                red_pixels_y(i) = red_pixels_y(i)-j_centroid_mean;
            }
            Eigen::MatrixXf point_cloud(npix,2), Covariance_matrix(2,2);
            point_cloud.col(0)<<red_pixels_x;
            point_cloud.col(1)<<red_pixels_y;
            Covariance_matrix = point_cloud.transpose()*point_cloud;
            //cout<<Covariance_matrix<<endl;

            //Now to find eigenvectors
            Eigen::EigenSolver<Eigen::Matrix2f> es2f(Covariance_matrix);

            //cout<<"The eigen values of the covariance matrix are: "<<es2f.eigenvalues() <<endl;
            //cout<<"The eigen vectors of the covariance matrix are: "<<es2f.eigenvectors() <<endl<<endl;
            //cout<<es2f<<endl;
            double major_axis, minor_axis;
            ros::Duration(5.0).sleep();
            Eigen::MatrixXf real_eigenvec(2,1);
            real_eigenvec = es2f.eigenvectors().col(0).real();
            //cout<<real_eigenvec<<endl;
            if(real_eigenvec(1,0)>real_eigenvec(0,0)) {
                major_axis = real_eigenvec(1,0);
                minor_axis = real_eigenvec(0,0);
            }
            else {
                major_axis = real_eigenvec(0,0);
                minor_axis = real_eigenvec(1,0);
            }
            double rel_orientation = atan2(minor_axis,major_axis);
            double orientation = -(rel_orientation+.196-1.57);
            ROS_INFO("Orientation of block is %f",orientation);

            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());
        
        //block_pose_.pose.position.x = i_centroid; //not true, but legal
        //block_pose_.pose.position.y = j_centroid; //not true, but legal
        //double theta=0;
        
        //Define transformation between camera wrt robot
        T_cam_robot.row(0)<<cos(theta_cam), -sin(theta_cam), 0, 0.543; //0.543 is the position of x_cam wrt to robot
        T_cam_robot.row(1)<<sin(theta_cam), cos(theta_cam), 0, 0.321; //0.321 is the position of y_cam wrt to robot
        T_cam_robot.row(2)<<0, 0, 1, 1.75;
        T_cam_robot.row(3)<<0, 0, 0, 1;

        //Define transformation between block wrt camera
        T_block_cam.row(0)<<1, 0, 0, scale_factor*(x_centroid-320);
        T_block_cam.row(1)<<0, -1, 0, -scale_factor*(y_centroid-240);
        T_block_cam.row(2)<<0, -1, 0, -1.75+BLOCK_HEIGHT/2;
        T_block_cam.row(3)<<0, 0, 0, 1;

        //Transformation of block wrt robot 
        T_block_robot = T_cam_robot*T_block_cam;

        //Assign pose of block for robot to understand
        //cout<<T_block_robot<<endl;
        block_pose_.pose.position.x = T_block_robot(0,3);
        block_pose_.pose.position.y = T_block_robot(1,3);
        block_pose_.pose.position.z = T_block_robot(2,3);

        //Next is orientation, using eigenvectors.

        //ROS_INFO("Using eigen centroid = %f, %f",i_centroid_mean,j_centroid_mean);
        //ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
        //ROS_INFO("Block x: %f; Block y: %f",block_pose_.pose.position.x,block_pose_.pose.position.y);
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        //block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(theta); //not true, but legal
        block_pose_publisher_.publish(block_pose_);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    g_redratio= 10; //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
