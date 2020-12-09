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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;  //<KF，该KF对应的sim3变换>   （sim3变换一般指世界坐标系到该KF） 

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    Viewer* mpViewer;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                vector<MapPoint*> &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints);
    void SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void CheckObservations(set<KeyFrame*> &spKFsMap1, set<KeyFrame*> &spKFsMap2);
    void printReprojectionError(set<KeyFrame*> &spLocalWindowKFs, KeyFrame* mpCurrentKF, string &name);

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    //保存来至于局部建图线程的KF，非空的时候启动闭环检测线程
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpLastCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    Map* mpLastMap;

    //!  ======================    相同地图内的闭环   ===========================
    //检测到了闭环
    bool mbLoopDetected;
    //mnLoopNumCoincidences  位置识别阶段，在当前帧共视图中验证  闭环匹配sim3  通过的关键帧数(0~3)（该关键帧是来至于当前帧的共视图）
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    KeyFrame* mpLoopLastCurrentKF;

    // 从世界坐标系到上一帧率的Sim3变换 
    g2o::Sim3 mg2oLoopSlw;     // mg2oLoopSlw 会在后面进行更新，直接在后面获取 世界坐标系w  -->> 当前c  的sim3变换作为下一次使用的值
    // 从世界坐标系到当前帧的Sim3变换 
    g2o::Sim3 mg2oLoopScw;
    
    KeyFrame* mpLoopMatchedKF;
    //位置识别阶段，在闭环候选帧局部窗口中的地图点
    std::vector<MapPoint*> mvpLoopMPs;   
    //位置识别阶段，在闭环候选帧局部窗口中，投影到当前帧中并匹配上的点（未进行冗余处理）
    std::vector<MapPoint*> mvpLoopMatchedMPs;

    //!  ======================    不同地图间的匹配   ===========================
    //检测到了地图匹配
    bool mbMergeDetected;
    // 同上面 mnLoopNumCoincidences ：位置识别阶段，在当前帧共视图中验证  闭环匹配sim3  通过的关键帧数(0~3)（该关键帧是来至于当前帧的共视图）
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    KeyFrame* mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;    // 得到世界坐标系到匹配帧的sim3变换
    g2o::Sim3 mg2oMergeScw;     //世界坐标系到当前帧的sim3变换
    //匹配帧（从候选匹配帧中得到的最佳帧）
    KeyFrame* mpMergeMatchedKF;
    //位置识别阶段，在闭环候选帧局部窗口中的地图点
    std::vector<MapPoint*> mvpMergeMPs;
    //位置识别阶段，在闭环候选帧局部窗口中，投影到当前帧中并匹配上的点（未进行冗余处理）
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    //与上面匹配点mvpMergeMatchedMPs相关联的KF（上面的地图点，来至于下面的KF，下面的KF属于候选匹配帧的共视图）
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;   //初始化为false，在CorrectLoop函数中置位 true
    bool mbFinishedGBA;
    bool mbStopGBA;                     // 由当前线程调用,请求停止当前正在进行的全局BA
    std::mutex mMutexGBA;           // 在对和全局线程标志量有关的操作的时候使用的互斥量
    std::thread* mpThreadGBA;    // 全局BA线程句柄

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;   //似乎只有纯粹单目不需要固定尺度，单目_imu也需要固定尺度


    bool mnFullBAIdx;

    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
