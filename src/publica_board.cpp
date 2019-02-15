#include "ros/ros.h"
#include "std_msgs/String.h" // Para eliminar
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <vizzy_fingers/MarkerArray.h> // Inclusão deste header serve para reconhecer a mensagem (MARKER) 
#include <tf/transform_broadcaster.h> // Este Header é necessario para se puder usar a descrição da tf no codigo

using namespace cv;
using namespace std;


namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side lenght (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
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
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
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


cv::Mat DoubleMatFromVec3b(cv::Vec3d in)
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
void Publish_Markers(vector< Vec3d > rvecs, vector< Vec3d > tvecs, vector<int> markersOfBoardDetected, ros::Publisher * chatter_pub)
{
    ros::Time curr_stamp(ros::Time::now());

    vizzy_fingers::MarkerArray::Ptr marker_array = vizzy_fingers::MarkerArray::Ptr(new vizzy_fingers::MarkerArray());

    //Find the number of boards present in the frame
    int n_boards=0;
    for (int i =0; i<4 ; i++){
        if(markersOfBoardDetected[i] != 0)
            n_boards++;
    }

    marker_array->markers.clear();
    marker_array->markers.resize(n_boards);
    marker_array->header.stamp = curr_stamp;
    marker_array->header.seq++;
    marker_array->header.frame_id = "child";
    // For each Marker detected, it will find its quaternion and put them in a MarkerArray[]
    int index_marker=-1;
     for(int i = 0; i<n_boards;i++)
     {
        index_marker++;
        if ( markersOfBoardDetected[i]==0){
            continue;
        }

        //Declaration of variables
        Mat R;
        Mat rvec;
        vector<double> Q;
        vizzy_fingers::Marker tem;

        // Creation of Markers menssages
        tem.id = (uint32_t)index_marker;
        tem.pose.pose.position.x = tvecs[i][0];
        tem.pose.pose.position.y = tvecs[i][1];
        tem.pose.pose.position.z = tvecs[i][2];

        rvec = DoubleMatFromVec3b(rvecs[i]);
        cv::Rodrigues(rvec, R); // R is 3x3
        Q = getQuaternion(R);

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
    
    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int markersX = parser.get<int>("w");
    int markersY = parser.get<int>("h");
    float markerLength = parser.get<float>("l");
    float markerSeparation = parser.get<float>("s");
    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool refindStrategy = parser.has("rs");
    int camId = parser.get<int>("ci");
    bool estimatePose = true;

    
    

   Mat camMatrix, distCoeffs;
    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }
    
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);

    // create board object
    //fisrt board
    Ptr<aruco::GridBoard> gridboard1 = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary,0);
    Ptr<aruco::Board> board1 = gridboard1.staticCast<aruco::Board>();
    //secound board
    Ptr<aruco::GridBoard> gridboard2 = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary,4);
    Ptr<aruco::Board> board2 = gridboard2.staticCast<aruco::Board>();
    //Third board
    Ptr<aruco::GridBoard> gridboard3 = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary,8);
    Ptr<aruco::Board> board3 = gridboard3.staticCast<aruco::Board>();
    //Fourth board
    Ptr<aruco::GridBoard> gridboard4 = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary,12);
    Ptr<aruco::Board> board4 = gridboard4.staticCast<aruco::Board>();
    
    // Redimensiona a dimensão de cada imagem
    inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    
    double totalTime = 0;
    int totalIterations = 0;


    // Where magic haapens
    while(inputVideo.grab()) {
    
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();
        
        //Declaration of variables
        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;
        vector<Vec3d> rvecs,tvecs;
        //Dummies values
        rvecs.push_back(Vec3d(0,0,0));tvecs.push_back(Vec3d(0,0,0));
        rvecs.push_back(Vec3d(0,0,0));tvecs.push_back(Vec3d(0,0,0));
        rvecs.push_back(Vec3d(0,0,0));tvecs.push_back(Vec3d(0,0,0));
        rvecs.push_back(Vec3d(0,0,0));tvecs.push_back(Vec3d(0,0,0));
        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy){
            if (std::find(ids.begin(), ids.end(),0)!=ids.end())
                aruco::refineDetectedMarkers(image, board1, corners, ids, rejected, camMatrix,distCoeffs);
            if (std::find(ids.begin(), ids.end(),4)!=ids.end())
                aruco::refineDetectedMarkers(image, board2, corners, ids, rejected, camMatrix,distCoeffs);
            if (std::find(ids.begin(), ids.end(),8)!=ids.end())
                aruco::refineDetectedMarkers(image, board3, corners, ids, rejected, camMatrix,distCoeffs); 
            if (std::find(ids.begin(), ids.end(),12)!=ids.end())
                aruco::refineDetectedMarkers(image, board4, corners, ids, rejected, camMatrix,distCoeffs);       
        }
        // estimate board pose
        vector<int> markersOfBoardDetected(4,0);
        if(ids.size() > 0 && std::find(ids.begin(), ids.end(),0)!=ids.end()){
            markersOfBoardDetected[0] =aruco::estimatePoseBoard(corners, ids, board1, camMatrix, distCoeffs, rvec, tvec);
            rvecs[0]=rvec; tvecs[0]=(tvec);
        }
        if(ids.size() > 0 && std::find(ids.begin(), ids.end(),4)!=ids.end()){
            markersOfBoardDetected[1] =aruco::estimatePoseBoard(corners, ids, board2, camMatrix, distCoeffs, rvec, tvec);
            rvecs[1]=rvec; tvecs[1]=(tvec);
        }
        if(ids.size() > 0 && std::find(ids.begin(), ids.end(),8)!=ids.end()){
            markersOfBoardDetected[2] =aruco::estimatePoseBoard(corners, ids, board3, camMatrix, distCoeffs, rvec, tvec);
            rvecs[2]=rvec; tvecs[2]=(tvec);
        }       
        if(ids.size() > 0 && std::find(ids.begin(), ids.end(),12)!=ids.end()){
            markersOfBoardDetected[3] =aruco::estimatePoseBoard(corners, ids, board4, camMatrix, distCoeffs, rvec, tvec);
            rvecs[3]=rvec; tvecs[3]=(tvec);
        }
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        // draw results
        if(ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                // Function that publishes on the ROS topic
                Publish_Markers(rvecs, tvecs,  markersOfBoardDetected, &chatter_pub);
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));


       
        for(int i=0; i<4;i++){
            if(markersOfBoardDetected[i] !=0){
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], axisLength);
            }
        }
        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }




               

     
    return 0;
}
