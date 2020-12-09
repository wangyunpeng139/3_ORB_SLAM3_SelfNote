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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Atlas.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);
    
    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);
    // cv::Mat GrabImageImuMonocular(const cv::Mat &im, const double &timestamp);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();
public:

/**
 * V-D
 * 在单纯的视觉SLAM或VO系统中，短暂的摄像机遮挡和快速的运动会导致视觉元素失去轨迹，导致系统丢失。
 * ORB-SLAM率先使用了基于单词包位置识别的快速重定位技术，但它们被证明不足以解决EuRoC数据集中的复杂序列。
 * 当跟踪点数小于15时，系统进入视觉lost状态，系统实现了两阶段鲁棒性：  
 * 
 * 短期丢失:
 * 当前的body状态是根据IMU读数估计的，地图点被投影到估计的相机姿态中，
 * 并在一个大的图像窗口中搜索匹配。结果匹配包含在视觉惯性优化中。
 * 在大多数情况下，这允许恢复视觉跟踪。否则，5秒后，我们进入下一阶段（Lost）。
 * 
 * 长期丢失:
 * 一个新的视觉惯性map被初始化，并成为active map。
 * 
 * 
*/
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,    //系统没有准备好的状态,一般就是在启动后加载配置文件和词典文件时候的状态
        NO_IMAGES_YET=0,        //当前无图像
        NOT_INITIALIZED=1,      //有图像但是没有完成初始化
        OK=2,                   //正常时候的工作状态（已经完成了初始化）

        /**
         * 个人认为，这个状态其实就是相当于原先的LOST，
         * 在这里，如果激活了IMU则通过IMU数据进行预测，也就是 calculate current  pose by IMU data。
         * 
         * 但如果丢失时间超过五秒，则由当前state = RECENTLY_LOST切换为LOST；
         * 如果是非IMU，即纯视觉状态，则直接进行重定位Relocalization()；
        */
        RECENTLY_LOST=3,        //! 当前地图中的KF>10,且丢失时间<5秒,短期丢失

        /**
         * LOST: 这个状态就是ORBSLAM3的一个新加的状态（虽然名字与之前的一样），
         * 因为新增了atlas，如果当前地图中的关键帧数量<10个，可以认为当前地图中没有重要信息，
         * 直接ResetActiveMap(),这个思路相当于之前的Reset()， 
         * 即如果初始化刚刚成功就丢失，可以认为当前map并不重要，直接重新初始化，这里也是一样的思路，
         * 这个部分最终发挥作用是在函数Tracking::ResetActiveMap()中。
         * 但如果当前的地图已经有很多关键帧(大于10帧)，则调用CreateNewMap() 暂存该地图，再新建一个空的地图。
         * 
        */
        LOST=4,                 //!系统已经跟丢了的状态,也就是论文中的长期丢失状态
        OK_KLT=5
    };

    //当前线程的状态
    eTrackingState mState;
    //当前线程上一次的状态
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    //! 下面这三个只有在跟踪线程跟踪后，状态位正常跟踪或者短期丢失状态下才会被更新
    list<cv::Mat> mlRelativeFramePoses;    //存储帧之间的相对位姿
    list<KeyFrame*> mlpReferences;            //存储跟踪成功的每一帧
    list<double> mlFrameTimes;                  //存储时间戳
    list<bool> mlbLost;                                     //是否跟踪失败

    // frames with estimated pose
    int mTrackedFr;   //没有什么用处
    bool mbStep;   //在Pangolin上每次点击step一次, mbStep = true,就处理一次图像帧率

    // True if local mapping is deactivated and we are performing only localization
    //true : 开始定位模式（只定位，提取特征点，但是不创建关键帧）
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization


    vector<MapPoint*> GetLocalMapMPS();


    //TEST--
    bool mbNeedRectify;
    //cv::Mat M1l, M2l;
    //cv::Mat M1r, M2r;

    bool mbWriteStats;

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateNewMapPoints();
    cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    bool TrackLocalMap_old();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();
    void ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz);
    void ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz);

    //=true的时候，表示刚刚产生了新的地图
    bool mbMapUpdated;

    // Imu preintegration from last frame
    //并不是一直在积分，而是每一次产生新的关键帧的时候就会重新指向，从头开始进行积分
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;  

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;   // 存储两幅图像之间的imu数据

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;   //当前帧与上一帧之间的imu序列，这些会用来进行预积分
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    ///当进行纯定位时才会有的一个变量,为false表示该帧匹配了很多的地图点,跟踪是正常的;如果少于10个则为true,表示快要完蛋了
     // 在函数 TrackWithMotionModel 和  TrackReferenceKeyFrame 内部会根据跟踪点的个数进行更新
    // 在函数TrackWithMotionModel函数中,图像的内点中被匹配的点少于10个就会 mbVO=true
    bool mbVO;     

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    //正常运行时候的提取器
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    //初始化时候的提取器提取的特征点比较多
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;
    bool mbSetInit;

    //Local Map
    // 代码里面将跟踪线程里面的参考关键帧赋值为最新产生的关键帧（注意了，这是跟踪线程的关键帧，不是图像帧的参考帧）
    KeyFrame* mpReferenceKF;    //每一次图像传入跟踪线程后，在单目相机进行初始化的时候会产生该关键帧，产生新的关键帧的时候也会为其赋值
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;  //可视化相关 v3增加，打开后（点击 Step By Step），在Pangolin上每次点击step一次，就处理一帧图像

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;                     //内参模型，跟踪对象创建的时候会读取配置文件获取相机内参
    //k1、k2、p1、p2、k3
    cv::Mat mDistCoef;      //去畸变系统，也是在跟踪对象创建的时候会读取配置文件获取
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;     //产生关键帧需要至少间隔的帧数目（目前看 = 0）
    int mMaxFrames;     //在跟踪线程对象创建的时候被设定为了相机帧率大小

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;   //被赋值为 mMaxFrames（而mMaxFrames被赋值为图像的帧率fps）

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;    //上一次重定位的帧
    double mTimeStampLost;  //当前帧ID相比较上一帧，已经过去了mMaxFrames帧（初始化的时候设置为帧率大小），那就记录当前帧时间为丢失帧时间
    //判定跟踪失败的时间阈值，短期丢失（KF>10，丢失时间<5s）,允许恢复出位姿，但是时间超过5s的时候就会出现失败。
    double time_recently_lost;  //预设为5s

    
    unsigned int mnFirstFrameId;           //第一个地图的第一个图像帧的ID,牵涉到图像和imu的对齐，并不一定为0
    unsigned int mnInitialFrameId;        //每一个新地图的产生后的第一个图像帧
    unsigned int mnLastInitFrameId;    //每一个地图的最后一个图像帧

    //是否创建新的地图的标志位
    bool mbCreatedMap;


    //Motion Model
    cv::Mat mVelocity;     // 其实就是 Tcl  上一帧率到当前帧的变换

    //Color order (true RGB, false BGR, ignored if grayscale)
    //判断传入的图像时哪一种格式
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    //?  第几个数据集的 ？？？
    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    //图像帧ID
    int initID, lastID; //初始化帧ID，上一帧ID   ，在初始化的时候赋值为0

    cv::Mat mTlr;

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
