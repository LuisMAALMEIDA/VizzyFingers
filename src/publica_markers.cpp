#include "ros/ros.h"
#include "std_msgs/String.h" // Para eliminar
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdint.h>
#include <iostream>
#include <vizzy_fingers/MarkerArray.h> // Inclusão deste header serve para reconhecer a mensagem (MARKER) 
#include <tf/transform_broadcaster.h> // Este Header é necessario para se puder usar a descrição da tf no codigo

using namespace cv;
using namespace std;

namespace{
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       |    Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, aruco::DetectorParameters &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params.adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params.adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params.adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params.adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params.minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params.maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params.polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params.minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params.minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params.minMarkerDistanceRate;
    fs["cornerRefinementWinSize"] >> params.cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params.cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params.cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params.markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params.perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params.perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params.maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params.minOtsuStdDev;
    fs["errorCorrectionRate"] >> params.errorCorrectionRate;
    return true;
}

/**
 * This function retrives a quaternion of a rotation matrix from a Marker
 * This is useful to send a Ros message for a topic 
**/
vector<double>   getQuaternion(Mat R)
{
    vector<double> Q;
    (Q).resize(4);
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
    return Q;
}


cv::Mat DoubleMatFromVec3b(cv::Vec3b in)
{
    cv::Mat mat(3,1, CV_64FC1);
    mat.at <double>(0,0) = in [0];
    mat.at <double>(1,0) = in [1];
    mat.at <double>(2,0) = in [2];

    return mat;
};


/**
 * This function broadcast a tf of each marker to a topic
 * (so we can see it in rviz)
 * */
void tf_broadcaster_msg( vizzy_fingers::Marker * tem )
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(tem->pose.pose.position.x, tem->pose.pose.position.y, tem->pose.pose.position.z) );
    tf::Quaternion q;
    q.setX(tem->pose.pose.orientation.x);
    q.setY(tem->pose.pose.orientation.y);
    q.setZ(tem->pose.pose.orientation.z);
    q.setW(tem->pose.pose.orientation.w);

    transform.setRotation(q);
    //Publishes a transfrom tf
    stringstream ss;
    ss << tem->id;
    string id_str = ss.str();
    string name_frame = "son_frame_"+ id_str;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name_frame));

    //Putting a tf in a marker
    tem->transform.rotation.x = tem->pose.pose.orientation.x;
    tem->transform.rotation.y = tem->pose.pose.orientation.y;
    tem->transform.rotation.z = tem->pose.pose.orientation.z;
    tem->transform.rotation.w = tem->pose.pose.orientation.w;

    tem->transform.translation.x = tem->pose.pose.position.x;
    tem->transform.translation.y = tem->pose.pose.position.y;
    tem->transform.translation.z = tem->pose.pose.position.z;
}


/**
 * This funtion uses the getQuaternion(Mat R, double Q[]) to generates a quaternion (or more if there are more than one Marker)
 * Then sends a information to a topic (on ROS_MASTER)
*/
void Publish_Markers(vector< Vec3d > rvecs, vector< Vec3d > tvecs, vector<int> ids, ros::Publisher * chatter_pub)
{
    ros::Time curr_stamp(ros::Time::now());

    vizzy_fingers::MarkerArray::Ptr marker_array = vizzy_fingers::MarkerArray::Ptr(new vizzy_fingers::MarkerArray());
    marker_array->markers.clear();
    marker_array->markers.resize(ids.size());
    marker_array->header.stamp = curr_stamp;
    marker_array->header.seq++;
    marker_array->header.frame_id = "child";
    // For each Marker detected, it will find its quaternion and put them in a MarkerArray[]
     for(int i = 0; i<ids.size();i++)
     {
        //Declaration of variables
        Mat R;
        Mat rvec;
        vector<double> Q;
        vizzy_fingers::Marker tem;

        // Creation of Markers menssages
        tem.id = (uint32_t)ids[i];
        tem.pose.pose.position.x = tvecs[i][0];
        tem.pose.pose.position.y = tvecs[i][1];
        tem.pose.pose.position.z = tvecs[i][2];

        rvec = DoubleMatFromVec3b(rvecs[i]);
    
        cv::Rodrigues(rvec, R); // R is 3x3
        Q = getQuaternion( R);

        tem.pose.pose.orientation.w = Q[3];
        tem.pose.pose.orientation.x = Q[0];
        tem.pose.pose.orientation.y = Q[1];
        tem.pose.pose.orientation.z = Q[2]; 
        
        // Creates a tf in each marker (and broadcast it)
        tf_broadcaster_msg( &tem );

        marker_array->markers.at(i) = tem;        
    }

    chatter_pub->publish(marker_array); //Publica no topico
}

/**
 * 
 *  THE MAIN FUNCTION
 * 
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    // Ros node publisher
    ros::init(argc, argv, "Markers_publisher"); 
    ros::NodeHandle n; // Ros node handler
    ros::Publisher chatter_pub = n.advertise<vizzy_fingers::MarkerArray>("Markers_chatter", 1000);
    

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    //Aruco parameters
    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    cv::aruco::DetectorParameters detectorParams;
    const cv::Ptr<cv::aruco::DetectorParameters>& _detectorParams =&detectorParams;

    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    
    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    // obtem-se o dicionario predefinido DICT_7x7_50
    const cv::Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // Matrix da camara e os coeficientes de distorção
    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    //Abertura do video
    VideoCapture inputVideo;
   
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }
    cout << inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT,1080)<<endl;
    cout << inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 1920)<<endl;
    
    double totalTime = 0;
    int totalIterations = 0;


    // Where magic haapens
    while(inputVideo.grab()) {
        //cout << inputVideo.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
    	//cout << inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
    
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
       cv::aruco::detectMarkers(image, dictionary , corners, ids, _detectorParams, rejected);
        if(estimatePose && ids.size() > 0)
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                             tvecs);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++)
                {
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }
                // Function that publishes on the ROS topic
                Publish_Markers(rvecs, tvecs,  ids, &chatter_pub);
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

    return 0;
}
