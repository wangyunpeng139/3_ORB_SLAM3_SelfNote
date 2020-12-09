/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef FRAME_H
#define FRAME_H

//#define SAVE_TIMES

#include<vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include <mutex>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Destructor
    // ~Frame();

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose. (Imu pose is not modified!)
    void SetPose(cv::Mat Tcw);
    void GetPose(cv::Mat &Tcw);

    // Set IMU velocity
    void SetVelocity(const cv::Mat &Vwb);

    // Set IMU pose and velocity (implicitly changes camera pose)
    void SetImuPoseVelocity(const cv::Mat &Rwb, const cv::Mat &twb, const cv::Mat &Vwb);


    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    cv::Mat GetImuPosition();
    cv::Mat GetImuRotation();
    cv::Mat GetImuPose();

    void SetNewBias(const IMU::Bias &b);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    cv::Mat inRefCoordinates(cv::Mat pCw);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1, const bool bRight = false) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

    //? 什么玩意？？答：紧耦合对象指针，内部保留着边缘化后的Hession矩阵，具体可参考 PoseInertialOptimizationLastFrame
    ConstraintPoseImu* mpcpi;   

    bool imuIsPreintegrated();
    void setIntegrated();

    cv::Mat mRwc;
    cv::Mat mOw;
public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;        ///<x轴方向焦距
    static float fy;        ///<y轴方向焦距
    static float cx;        ///<x轴方向光心偏移
    static float cy;        ///<y轴方向光心偏移
    static float invfx;     ///<x轴方向焦距的逆
    static float invfy;     ///<x轴方向焦距的逆
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    //相机的基线长度,单位为米
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    //mvpMapPoints的大小与当前帧提取的特征点大小一致，目的是为了关联当前帧的特征点索引与三维地图点坐标
    std::vector<MapPoint*> mvpMapPoints;  
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    int mnCloseMPs;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];


    // Camera pose.
    //相机位姿势，也会在IMU预测函数中，通过 Frame::SetImuPoseVelocity()进行更新
    cv::Mat mTcw; 

    // IMU linear velocity
    //IMU线速度，在函数 PredictStateIMU() 预测本帧的状态量后，会通过函数SetImuPoseVelocity()直接将速度状态量部分直接赋值给下面的变量
    cv::Mat mVw;   

    //下面这三个都是IMU坐标系下的量（参考13页公式里面的i->j就知道了）
    //预积分预测的本关键帧的状态量（ PredictStateIMU ）
    cv::Mat mPredRwb, mPredtwb, mPredVwb;
    IMU::Bias mPredBias;

    // IMU bias
    IMU::Bias mImuBias;

    // Imu calibration
    IMU::Calib mImuCalib;  //里面存储的主要是一些协方差矩阵，以及相机与imu之间的相对变换

    // Imu preintegration from last keyframe
    //! 这个是一直进行预积分的对象
    //! 所有的关键帧共用这一个mpImuPreintegrated（因为所有的关键帧都是获取的跟踪线程的指针，所以是共用的指针）。
    //! 所以不难理解,先前帧就是时间上刚刚进行的帧率。上一帧就是该帧的上一帧图像
    IMU::Preintegrated* mpImuPreintegrated;   
    KeyFrame* mpLastKeyFrame;

    // Pointer to previous frame
    Frame* mpPrevFrame;  //该帧的上一帧
    IMU::Preintegrated* mpImuPreintegratedFrame;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    //该帧处理后，出现的第一个关键帧，也就是最近的关键帧（这个关键帧也有可能会是自己构造的关键帧）
    //但是在Tracking::UpdateLocalKeyFrames 局部关键帧更新中，设置为与当前帧共视程度最强的关键帧
    KeyFrame* mpReferenceKF;  

    /**
     * @name 图像金字塔信息
     * 
     */
    // Scale pyramid info.
    int mnScaleLevels;                  ///<图像金字塔的层数
    float mfScaleFactor;                ///<图像金字塔的尺度因子
    float mfLogScaleFactor;             ///<图像金字塔的尺度因子的对数值？
                                        ///@todo 为什么要计算存储这个，有什么实际意义吗
    vector<float> mvScaleFactors;		///<图像金字塔每一层的缩放因子
    vector<float> mvInvScaleFactors;	///<以及上面的这个变量的倒数
    vector<float> mvLevelSigma2;    // = sigma^2：每层缩放系数的平方 ///@todo 目前在frame.c中没有用到，无法下定论
    vector<float> mvInvLevelSigma2;		///<上面变量的倒数

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    //记录了跟踪线程中，进行局部地图跟踪的时候，视野范围内的局部地图点投影到当前帧像素坐标系上的坐标。<地图点ID，像素坐标>
    map<long unsigned int, cv::Point2f> mmProjectPoints;    
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

    string mNameFile;

    int mnDataset;   //按照这是第几个数据集进行理解

    double mTimeStereoMatch;
    double mTimeORB_Ext;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    //==mtwc

    //? 这个标志位会产生什么效果，好像是进行过预积分后，该标志位置位，表示已经预积分过了
    bool mbImuPreintegrated;

    std::mutex *mpMutexImu;

public:
    //? 下面这两个什么意思？？是不同的相机模型吗？？
    GeometricCamera* mpCamera, *mpCamera2;

    //Number of KeyPoints extracted in the left and right images
      //这个只有双目才有的信息，对于单目两个永远都是 -1，
      //当地图点ID大于Nleft的时候，表明是右目的地图点的ID
    int Nleft, Nright;    
    //Number of Non Lapping Keypoints
    int monoLeft, monoRight;

    //For stereo matching
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    //For stereo fisheye matching
    static cv::BFMatcher BFmatcher;

    //Triangulated stereo observations using as reference the left camera. These are
    //computed during ComputeStereoFishEyeMatches
    std::vector<cv::Mat> mvStereo3Dpoints;

    //Grid for the right image
    std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    cv::Mat mTlr, mRlr, mtlr, mTrl;

    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, cv::Mat& Tlr,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    //Stereo fisheye
    void ComputeStereoFishEyeMatches();

    bool isInFrustumChecks(MapPoint* pMP, float viewingCosLimit, bool bRight = false);

    cv::Mat UnprojectStereoFishEye(const int &i);

    cv::Mat imgLeft, imgRight;

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (Nleft != -1) ? Nleft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i] && !mvbOutlier[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
    }
};

}// namespace ORB_SLAM

#endif // FRAME_H
