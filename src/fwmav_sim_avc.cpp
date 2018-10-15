#include "ros/ros.h"
#include "fwmav_of_sim/MsgOAOF.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <iostream>
#include <fstream>
#include <ctype.h>
#include <cmath>

#define S_TIME      0.1

#define ALPHA       0.00003

#define PI          3.141592

#define O_WIDTH     800
#define O_HEIGHT    600

#define	WIDTH       720
#define HEIGHT	    480

#define N_TEXTONS   30
#define PATCH_L     5
#define PATCH_SIZE  PATCH_L*PATCH_L
#define S_TEXTONS   200

#define D_SET       0.1

#define INIT_P_X    0
#define INIT_P_Y    0
#define INIT_P_Z    1.5

using namespace cv;
using namespace std;

// ---------------------- //
// -- Grobal Variables -- //
// ---------------------- //

unsigned char Img_data[O_WIDTH*O_HEIGHT*3];
int count_loop;
double pose_p_x_c = INIT_P_X;
double pose_p_y_c = INIT_P_Y;
double pose_p_z_c = INIT_P_Z;
double pose_o_qx_c = 0;
double pose_o_qy_c = 0;
double pose_o_qz_c = 0;
double pose_o_qw_c = 1;
double pose_o_ex_c = 0;
double pose_o_ey_c = 0;
double pose_o_ez_c = 0;

double pose_p_x_t = INIT_P_X;
double pose_p_y_t = INIT_P_Y;
double pose_p_z_t = INIT_P_Z;
double pose_o_qx_t = 0;
double pose_o_qy_t = 0;
double pose_o_qz_t = 0;
double pose_o_qw_t = 1;
double pose_o_ex_t = 0;
double pose_o_ey_t = 0;
double pose_o_ez_t = 0;

double d_vel_lx = 0;    // Desired linear velocity to x direction
double d_vel_ly = 0;    // Desired linear velocity to y direction
double d_vel_lz = 0;    // Desired linear velocity to z direction
double d_vel_az = 0;    // Desired angular velocity to z direction(yaw angle)

// ----------------------- //
// -- General Functions -- //
// ----------------------- //

int min_index(double _dists[PATCH_SIZE]);
unsigned int GetTickCount();
void get_texton_distribution(uchar _arr_gray_curr[WIDTH][HEIGHT], double _dictionary[N_TEXTONS][PATCH_SIZE], double _distribution[N_TEXTONS]);
double get_entropy(double _distribution[N_TEXTONS]);

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //

void msgCallback_img(const sensor_msgs::Image::ConstPtr& Img);
void msgCallback_int(const std_msgs::Int32::ConstPtr& num_c);

// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
    ros::init(argc, argv, "fwmav_of_sim");
    ros::NodeHandle nh, nh_mavros, nh_image, nh_c;

    ros::Publisher oa_of_pub = nh.advertise<fwmav_of_sim::MsgOAOF>("fwmav_of_sim_msg",100);
    ros::Publisher d_Position_pub = nh_mavros.advertise<geometry_msgs::PoseStamped>("fwmav/command/pose",100);
    ros::Publisher d_Velocity_pub = nh_mavros.advertise<geometry_msgs::TwistStamped>("fwmav/command/twist",100);
    //ros::Subscriber oa_of_sub_pos = nh_mavros.subscribe("gazebo/model_states", 10, msgCallback);
    ros::Subscriber oa_of_sub_image = nh_image.subscribe("fwmav/camera/image_raw", 10, msgCallback_img);
    //ros::Subscriber oa_of_sub_image = nh_image.subscribe("fwmav/camera/image_raw", 10, msgCallback_img);
    ros::Subscriber oa_of_sub_dir = nh_c.subscribe("num_c", 10, msgCallback_int);
    //ros::Subscriber oa_of_sub_time = nh_dir.subscribe("/count_number", 10, msgCallback_int);

    ros::ServiceClient pose_client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

    gazebo_msgs::GetModelState srv_pose;

    //std::string str_model_name = "FWMAV";
    //std::string str_world_name = "world";

    srv_pose.request.model_name = "FWMAV";
    srv_pose.request.relative_entity_name = "world";

    ros::Rate loop_rate(1/S_TIME);

    ofstream file_pose_data("pose_data.txt");
    ofstream file_image_data("image_data.txt");

    Mat mat_original;
    Mat mat_grey;
    Mat mat_resized;

    char keypressed;
    //int count_loop_s = 1;
    uchar arr_gray_curr[WIDTH][HEIGHT];

    double cy = 0;
    double sy = 0;
    double cr = 0;
    double sr = 0;
    double cp = 0;
    double sp = 0;

    double dictionary[N_TEXTONS][PATCH_SIZE] =
            {{0.23083,0.23125,0.22872,0.22559,0.22426,0.34793,0.35655,0.36352,0.35426,0.34266,0.43039,0.46028,0.47906,0.45539,0.42323,0.38777,0.41883,0.4457,0.42278,0.39689,0.25986,0.29579,0.32888,0.30641,0.27847}
            ,{0.49395,0.49082,0.4906,0.48976,0.49012,0.50598,0.50669,0.50371,0.49876,0.49854,0.51191,0.50993,0.50588,0.50501,0.50516,0.50889,0.50854,0.5077,0.50885,0.50917,0.53692,0.53824,0.54141,0.54215,0.53988}
            ,{0.82636,0.82816,0.83017,0.83111,0.83194,0.80233,0.80401,0.80503,0.80575,0.80568,0.67791,0.67724,0.67722,0.67718,0.6741,0.56249,0.56388,0.56475,0.56581,0.5675,0.53593,0.53853,0.54126,0.54227,0.54379}
            ,{0.64085,0.56321,0.50505,0.45824,0.41532,0.63817,0.5575,0.49047,0.42827,0.38194,0.63489,0.55967,0.48508,0.41988,0.36941,0.63485,0.56699,0.49725,0.42744,0.37463,0.63334,0.56863,0.50163,0.43692,0.38408}
            ,{0.67536,0.66639,0.65779,0.65365,0.6485,0.56924,0.58096,0.57457,0.56822,0.55714,0.48497,0.49907,0.50553,0.50421,0.49066,0.38051,0.38167,0.38861,0.39389,0.38745,0.27973,0.27578,0.27314,0.27349,0.27504}
            ,{0.44513,0.41483,0.36331,0.29644,0.23887,0.46209,0.43671,0.38377,0.30541,0.23596,0.47501,0.4548,0.39712,0.31368,0.24011,0.48758,0.45793,0.39377,0.32329,0.25754,0.47189,0.43293,0.38466,0.33334,0.27974}
            ,{0.29357,0.29383,0.29047,0.2911,0.29237,0.37145,0.3722,0.37285,0.37761,0.3823,0.47282,0.47383,0.47784,0.47751,0.47543,0.54194,0.54132,0.54327,0.54273,0.54167,0.56938,0.5695,0.57121,0.57083,0.56777}
            ,{0.38812,0.35931,0.33505,0.3263,0.33011,0.54076,0.53219,0.50719,0.47078,0.43743,0.71316,0.7166,0.69956,0.65944,0.60902,0.80665,0.81938,0.81647,0.7894,0.75657,0.84542,0.86523,0.8726,0.85757,0.83858}
            ,{0.58951,0.59072,0.59078,0.58927,0.58972,0.59307,0.59643,0.59773,0.59521,0.59281,0.62058,0.62255,0.62328,0.6211,0.61921,0.59086,0.59279,0.59653,0.59784,0.59829,0.30028,0.30519,0.31144,0.31362,0.3162}
            ,{0.054143,0.055319,0.054013,0.052965,0.051613,0.11655,0.11436,0.1125,0.10854,0.10654,0.27413,0.27126,0.26684,0.2616,0.25693,0.4127,0.41094,0.40769,0.40509,0.40351,0.52342,0.52138,0.51795,0.51839,0.5165}
            ,{0.86889,0.86772,0.86609,0.86633,0.86692,0.86802,0.86689,0.86506,0.86552,0.8666,0.86709,0.86807,0.86782,0.86607,0.86539,0.86669,0.8674,0.86772,0.86623,0.86592,0.86703,0.86761,0.86742,0.86718,0.86598}
            ,{0.73465,0.73431,0.73551,0.73825,0.72814,0.76397,0.7623,0.76408,0.76868,0.76246,0.78088,0.78205,0.7822,0.78559,0.78078,0.79213,0.7935,0.79312,0.79447,0.79092,0.7986,0.79992,0.79916,0.79922,0.79743}
            ,{0.73465,0.73431,0.73551,0.73825,0.72814,0.76397,0.7623,0.76408,0.76868,0.76246,0.78088,0.78205,0.7822,0.78559,0.78078,0.79213,0.7935,0.79312,0.79447,0.79092,0.7986,0.79992,0.79916,0.79922,0.79743}
            ,{0.22097,0.38058,0.6455,0.78424,0.79769,0.21911,0.36485,0.63003,0.77621,0.79387,0.22607,0.34935,0.59993,0.75681,0.78281,0.22782,0.33934,0.57824,0.7345,0.76185,0.23083,0.3359,0.56602,0.71951,0.74547}
            ,{0.56369,0.73444,0.8673,0.91229,0.91555,0.54916,0.73548,0.87932,0.92913,0.93217,0.54305,0.73354,0.88395,0.93879,0.94099,0.53983,0.7252,0.87855,0.93709,0.94412,0.53243,0.71377,0.87213,0.93449,0.94438}
            ,{0.6197,0.61914,0.6181,0.62281,0.62843,0.32108,0.31706,0.32003,0.33204,0.34046,0.1699,0.16129,0.16623,0.17823,0.18914,0.16564,0.15509,0.16116,0.17626,0.18445,0.16809,0.15864,0.15423,0.16736,0.18397}
            ,{0.15932,0.16262,0.16628,0.16451,0.16401,0.15067,0.15285,0.1559,0.14921,0.15172,0.14722,0.14521,0.14821,0.14601,0.14513,0.14129,0.13493,0.14086,0.14189,0.13851,0.13173,0.11927,0.12494,0.12703,0.1251}
            ,{0.24973,0.24414,0.24142,0.25377,0.27319,0.23615,0.23638,0.23382,0.23708,0.24317,0.21492,0.2161,0.21171,0.21797,0.21635,0.18418,0.18616,0.18861,0.19011,0.18946,0.1758,0.17507,0.17714,0.17466,0.1748}
            ,{0.32936,0.25639,0.21991,0.19806,0.17779,0.46604,0.34639,0.24646,0.18406,0.16946,0.61731,0.50013,0.3375,0.20631,0.16568,0.68661,0.62303,0.47421,0.29149,0.17936,0.71192,0.70146,0.59862,0.41267,0.24311}
            ,{0.47006,0.53156,0.57578,0.63902,0.70871,0.45745,0.52349,0.56899,0.63988,0.71741,0.45258,0.51944,0.56967,0.64241,0.72393,0.44628,0.51161,0.56615,0.64074,0.72055,0.44418,0.49723,0.55275,0.62276,0.7053}
            ,{0.14981,0.13006,0.14902,0.24917,0.39609,0.14968,0.12705,0.14882,0.24941,0.39567,0.14854,0.127,0.148,0.25646,0.39986,0.14735,0.12776,0.14643,0.24668,0.39856,0.15142,0.13252,0.15041,0.24099,0.39319}
            ,{0.068166,0.069235,0.070468,0.071292,0.071365,0.10853,0.11085,0.11525,0.11471,0.11391,0.15062,0.15285,0.15712,0.15397,0.1546,0.18547,0.18733,0.18769,0.18617,0.1876,0.22129,0.21976,0.21968,0.21909,0.22123}
            ,{0.79467,0.68717,0.5096,0.35631,0.28246,0.80073,0.69485,0.51355,0.35229,0.28142,0.80534,0.70048,0.51524,0.34858,0.27909,0.80042,0.69642,0.51457,0.34858,0.28295,0.794,0.69182,0.51615,0.35712,0.29491}
            ,{0.42961,0.31815,0.20633,0.13246,0.14631,0.43654,0.3223,0.2004,0.13082,0.13968,0.44757,0.33184,0.20301,0.1287,0.1408,0.44737,0.33604,0.20909,0.12734,0.14126,0.44515,0.33851,0.21494,0.13263,0.14231}
            ,{0.57351,0.57327,0.57307,0.57107,0.56636,0.56442,0.56339,0.56841,0.57278,0.57164,0.37058,0.37671,0.38193,0.39013,0.39304,0.15742,0.15953,0.16257,0.1695,0.17058,0.11081,0.11112,0.11027,0.11282,0.11342}
            ,{0.27688,0.28218,0.29013,0.29322,0.29313,0.096704,0.098958,0.10312,0.10505,0.10668,0.058684,0.055242,0.053489,0.053524,0.057361,0.053717,0.054061,0.055986,0.054969,0.054583,0.057423,0.059006,0.058964,0.057779,0.056245}
            ,{0.21777,0.22519,0.22849,0.22714,0.2274,0.2507,0.2553,0.25799,0.25972,0.25979,0.26666,0.26385,0.26626,0.26753,0.27037,0.32964,0.32811,0.32853,0.33314,0.33734,0.44766,0.45428,0.45897,0.46503,0.46448}
            ,{0.38719,0.38727,0.39164,0.39387,0.39163,0.35166,0.34772,0.34791,0.34927,0.34545,0.3839,0.37937,0.37902,0.382,0.37935,0.45135,0.44703,0.44583,0.44318,0.44635,0.48519,0.48976,0.48919,0.48329,0.48235}
            ,{0.89925,0.86554,0.76152,0.57748,0.43277,0.90414,0.87434,0.77096,0.5749,0.41741,0.90411,0.87694,0.77488,0.57894,0.4126,0.88944,0.8679,0.7713,0.58523,0.41837,0.87743,0.85339,0.76319,0.59108,0.43458}
            ,{0.24578,0.28024,0.3918,0.51118,0.55325,0.23998,0.26262,0.37819,0.50801,0.55843,0.23394,0.24586,0.35816,0.51462,0.5909,0.23249,0.24009,0.33913,0.4962,0.59381,0.23756,0.2451,0.33226,0.47482,0.56931}};

    double distribution[N_TEXTONS];
    double H = 0;

    double count = 0;
	while (ros::ok()){
    	//if(count_loop == 0){
        if(1){
            if(pose_client.call(srv_pose)){
                //ROS_INFO("pose_client has been successfully called!!");
                pose_p_x_c = srv_pose.response.pose.position.x;
                pose_p_y_c = srv_pose.response.pose.position.y;
                pose_p_z_c = srv_pose.response.pose.position.z;
                pose_o_qx_c = srv_pose.response.pose.orientation.x;
                pose_o_qy_c = srv_pose.response.pose.orientation.y;
                pose_o_qz_c = srv_pose.response.pose.orientation.z;
                pose_o_qw_c = srv_pose.response.pose.orientation.w;

                double siny = +2.0 * (pose_o_qw_c * pose_o_qz_c + pose_o_qx_c * pose_o_qy_c);
                double cosy = +1.0 - 2.0 * (pose_o_qy_c * pose_o_qy_c + pose_o_qz_c * pose_o_qz_c);
                pose_o_ez_c = atan2(siny, cosy);
            }

            mat_original = Mat(O_HEIGHT, O_WIDTH, CV_8UC3, &Img_data);

            cvtColor(mat_original, mat_grey, CV_RGB2GRAY);
    		resize(mat_grey, mat_resized, Size(WIDTH, HEIGHT));

    		// -------------------- //
    		// -- Save New Image -- //
    		// -------------------- //

            for(int i=0; i<(WIDTH); i++){
                for(int j=0; j<(HEIGHT); j++){
                    arr_gray_curr[i][j] = mat_resized.at<uchar>(j,i);
                }
            }

            if(count>=1){
                // --------------------- //
                // -- Main Controller -- //
                // --------------------- //

                get_texton_distribution(arr_gray_curr, dictionary, distribution);
                H = get_entropy(distribution);

                // ---------------------- //
                // -- Desired Velocity -- //
                // ---------------------- //

                d_vel_lx = D_SET;
                d_vel_ly = 0;
                d_vel_lz = 0;
                d_vel_az = 0;

                // ---------------------------- //
                // -- Target Pose Generation -- //
                // ---------------------------- //

                pose_o_ex_t = 0;
                pose_o_ey_t = 0;
                pose_o_ez_t = pose_o_ez_c + d_vel_az;
                pose_p_x_t = pose_p_x_c + D_SET*cos(pose_o_ez_t);
                pose_p_y_t = pose_p_y_c + D_SET*sin(pose_o_ez_t);
                pose_p_z_t = pose_p_z_c + d_vel_lz;

                // ------------------------- //
                // -- Euler to Quaternion -- //
                // ------------------------- //

                cy = cos(pose_o_ez_t * 0.5);
                sy = sin(pose_o_ez_t * 0.5);
                cr = cos(pose_o_ey_t * 0.5);
                sr = sin(pose_o_ey_t * 0.5);
                cp = cos(pose_o_ex_t * 0.5);
                sp = sin(pose_o_ex_t * 0.5);

                pose_o_qw_t = cy * cr * cp + sy * sr * sp;
                pose_o_qx_t = cy * sr * cp - sy * cr * sp;
                pose_o_qy_t = cy * cr * sp + sy * sr * cp;
                pose_o_qz_t = sy * cr * cp - cy * sr * sp;
            }

    		keypressed = (char)waitKey(10);
    		if(keypressed == 27)
    			break;

            fwmav_of_sim::MsgOAOF msg;
            geometry_msgs::PoseStamped msg_setposition;
            geometry_msgs::TwistStamped msg_d_velocity;

    		msg.data = count;
    		msg_setposition.pose.position.x = 1;
    		msg_setposition.pose.position.y = 0;
    		msg_setposition.pose.position.z = 1.5;
    		msg_setposition.pose.orientation.x = 0;
    		msg_setposition.pose.orientation.y = 0;
    		msg_setposition.pose.orientation.z = 0;
    		msg_setposition.pose.orientation.w = 1;
            msg_d_velocity.twist.linear.x = 0.1;
            msg_d_velocity.twist.linear.y = 0;
            msg_d_velocity.twist.linear.z = 1.5;
            msg_d_velocity.twist.angular.x = 0;
            msg_d_velocity.twist.angular.y = 0;
            msg_d_velocity.twist.angular.z = 0;

    		//msg_setposition.pose.position.x = pose_p_x_t;
    		//msg_setposition.pose.position.y = pose_p_y_t;
    		//msg_setposition.pose.position.z = pose_p_z_t;
    		//msg_setposition.pose.orientation.x = pose_o_qx_t;
    		//msg_setposition.pose.orientation.y = pose_o_qy_t;
    		//msg_setposition.pose.orientation.z = pose_o_qz_t;
    		//msg_setposition.pose.orientation.w = pose_o_qw_t;
            //msg_d_velocity.twist.linear.x = d_vel_lx;
            //msg_d_velocity.twist.linear.y = d_vel_ly;
            //msg_d_velocity.twist.linear.z = d_vel_lz;
            //msg_d_velocity.twist.angular.x = 0;
            //msg_d_velocity.twist.angular.y = 0;
            //msg_d_velocity.twist.angular.z = d_vel_az;

    		oa_of_pub.publish(msg);
    		d_Position_pub.publish(msg_setposition);
            d_Velocity_pub.publish(msg_d_velocity);

            // --------------- //
            // -- Data Save -- //
            // --------------- //

            //// Pose Data save
            file_pose_data << count << ", " << pose_p_x_t << ", " << pose_p_y_t << ", " << pose_p_z_t << ", " << pose_o_ex_t << ", " << pose_o_ey_t << ", " << pose_o_ey_t << ", ";
            file_pose_data << pose_p_x_c << ", " << pose_p_y_c << ", " << pose_p_z_c << ", " << pose_o_ex_c << ", " << pose_o_ey_c<< ", " << pose_o_ez_c << endl;

            //// Image Data Save
            file_image_data << count << ", ";
            for(int i=0; i<(WIDTH); i++){
                for(int j=0; j<(HEIGHT); j++){
                    file_image_data << (int)arr_gray_curr[i][j] << ", ";
                }
            }
            file_image_data << endl;

            // ------------- //
            // -- Display -- //
            // ------------- //

            namedWindow("Image_original",WINDOW_NORMAL);
            imshow("Image_original",mat_original);
            //namedWindow("Image_grey",WINDOW_NORMAL);
            //imshow("Image_grey",mat_grey);
            namedWindow("Image_resized",WINDOW_NORMAL);
            imshow("Image_resized",mat_resized);

            ROS_INFO(" ");
            ROS_INFO("-------------------------------");
    		ROS_INFO("Send msg = %f", count);
    		ROS_INFO("P_X = %f", pose_p_x_c);
    		ROS_INFO("P_Y = %f", pose_p_y_c);
    		ROS_INFO("P_Z = %f", pose_p_z_c);
            ROS_INFO("O_Z = %f", pose_o_ez_c);
    		ROS_INFO("-------------------------------");
            ROS_INFO("Distribution: %f, %f, %f, %f, %f, %f", distribution[0], distribution[1], distribution[2], distribution[3], distribution[4], distribution[5]);
            ROS_INFO("Distribution: %f, %f, %f, %f, %f, %f", distribution[6], distribution[7], distribution[8], distribution[9], distribution[10], distribution[11]);
            ROS_INFO("Distribution: %f, %f, %f, %f, %f, %f", distribution[12], distribution[13], distribution[14], distribution[15], distribution[16], distribution[17]);
            ROS_INFO("Distribution: %f, %f, %f, %f, %f, %f", distribution[18], distribution[19], distribution[20], distribution[21], distribution[22], distribution[23]);
            ROS_INFO("Distribution: %f, %f, %f, %f, %f, %f", distribution[24], distribution[25], distribution[26], distribution[27], distribution[28], distribution[29]);
            ROS_INFO("H = %f", H);
            ROS_INFO("-------------------------------");

    		count = count + S_TIME;
            count_loop = 1;
    	}//if
        ros::spinOnce();
        loop_rate.sleep();
    }//while

	file_pose_data.close();
	file_image_data.close();

	return 0;
}

// ----------------------- //
// -- General Functions -- //
// ----------------------- //

int min_index(double _dists[PATCH_SIZE]){
    int _min_idx = 0;
    double _min_dist = _dists[0];

    for(int i=0; i<(PATCH_SIZE-1); i++){
        if(_min_dist > _dists[i+1]){
            _min_idx = i+1;
            _min_dist = _dists[i+1];
        }
    }
    return _min_idx;
}

unsigned int GetTickCount()
{
    struct timeval gettick;
    unsigned int tick;
    int ret;
    gettimeofday(&gettick, NULL);

    tick = gettick.tv_sec*1000 + gettick.tv_usec/1000;

    return tick;
}

void get_texton_distribution(uchar _arr_gray_curr[WIDTH][HEIGHT], double _dictionary[N_TEXTONS][PATCH_SIZE], double _distribution[N_TEXTONS]){
    int x = 0;
    int y = 0;
    double sample_texton[PATCH_SIZE];
    double dists[N_TEXTONS];
    int min_idx = 0;

    for(int i=0; i<N_TEXTONS; i++){
        _distribution[i] = 0;
    }
    for(int sample_idx=0; sample_idx<S_TEXTONS; sample_idx++){
        srand(GetTickCount());
        x = rand() % (WIDTH - PATCH_L);
        y = rand() % (HEIGHT - PATCH_L);
        for(int i=0; i<PATCH_L; i++){
            for(int j=0; j<PATCH_L; j++){
                sample_texton[i*PATCH_L+j] = ((double)_arr_gray_curr[x+i][y+j])/255;
            }
        }
        for(int i=0; i<N_TEXTONS; i++){
            dists[i] = 0;
            for(int j=0; j<PATCH_SIZE; j++){
                dists[i] = dists[i] + sqrt(pow(_dictionary[i][j] - sample_texton[j],2));
            }
        }
        min_idx = min_index(dists);
        _distribution[min_idx] = _distribution[min_idx] + 1;
    }
    for(int i=0; i<N_TEXTONS; i++){
        _distribution[i] = _distribution[i]/S_TEXTONS;
    }
}

double get_entropy(double _distribution[N_TEXTONS]){
    double _H = 0;
    for(int i=0; i<N_TEXTONS; i++){
        if(_distribution[i] != 0){
            _H = _H - _distribution[i] * log2(_distribution[i]);
        }
    }
    return _H;
}

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //

void msgCallback_img(const sensor_msgs::Image::ConstPtr& Img){
    for(int i = 0; i<O_WIDTH*O_HEIGHT*3; i++){
        Img_data[i] = Img->data[i];
    }
}

void msgCallback_int(const std_msgs::Int32::ConstPtr& num_c){
	count_loop = num_c->data;
}
