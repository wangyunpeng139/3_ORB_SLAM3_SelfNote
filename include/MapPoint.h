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


#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
    template<class Archive>
    void serializeMatrix(Archive &ar, cv::Mat& mat, const unsigned int version)
    {
        int cols, rows, type;
        bool continuous;

        if (Archive::is_saving::value) {
            cols = mat.cols; rows = mat.rows; type = mat.type();
            continuous = mat.isContinuous();
        }

        ar & cols & rows & type & continuous;
        if (Archive::is_loading::value)
            mat.create(rows, cols, type);

        if (continuous) {
            const unsigned int data_size = rows * cols * mat.elemSize();
            ar & boost::serialization::make_array(mat.ptr(), data_size);
        } else {
            const unsigned int row_size = cols*mat.elemSize();
            for (int i = 0; i < rows; i++) {
                ar & boost::serialization::make_array(mat.ptr(i), row_size);
            }
        }
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Variables used by the tracking
        ar & mTrackProjX;
        ar & mTrackProjY;
        ar & mTrackDepth;
        ar & mTrackDepthR;
        ar & mTrackProjXR;
        ar & mTrackProjYR;
        ar & mbTrackInView;
        ar & mbTrackInViewR;
        ar & mnTrackScaleLevel;
        ar & mnTrackScaleLevelR;
        ar & mTrackViewCos;
        ar & mTrackViewCosR;
        ar & mnTrackReferenceForFrame;
        ar & mnLastFrameSeen;

        // Variables used by local mapping
        ar & mnBALocalForKF;
        ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        ar & mnLoopPointForKF;
        ar & mnCorrectedByKF;
        ar & mnCorrectedReference;
        serializeMatrix(ar,mPosGBA,version);
        ar & mnBAGlobalForKF;
        ar & mnBALocalForMerge;
        serializeMatrix(ar,mPosMerge,version);
        serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        serializeMatrix(ar,mWorldPos,version);
        //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mNormalVector,version);
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;
        ar & mnVisible;
        ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    MapPoint();

    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);

    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,int idx);
    void EraseObservation(KeyFrame* pKF);

    std::tuple<int,int> GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void SetNormalVector(cv::Mat& normal);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void PrintObservations();

    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    //在跟踪线程的局部地图跟踪里面，对某一帧Fcurr正在跟踪，在isInFrustum里面会计算出来地图点投影到Fcurr像素坐标系上的点坐标。
    //下面的这两行就是计算出来的投影后的像素坐标，
    float mTrackProjX; 
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    // NOTICE mbTrackInView==false的点有几种：
    // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
    // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影   //? 为什么已经是内点了之后就不需要再进行投影了呢?  答：参考函数 SearchLocalPoints
    // c 不在当前相机视野中的点（即未通过isInFrustum判断）     //?  答：参考函数 SearchLocalPoints
    bool mbTrackInView ;
    bool mbTrackInViewR;   //?
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;  //记录参与了哪一KF的局部BA
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    cv::Mat mPosMerge;   //地图拼接过程中，相对于 mWorldPos 的拼接纠正坐标
    cv::Mat mNormalVectorMerge;    //对上面归一化处理后的坐标


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    KeyFrame* mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     //三个元素分别是： 关键帧，左目ID，右目ID。对于左右目不存在的情况下赋值为 -1
     std::map<KeyFrame*,std::tuple<int,int> > mObservations;   //能够观察到该特征点的所有关键帧
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     // 该MapPoint平均观测方向（所有能够观测到该店的关键帧到改点的方向的平均值）
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;  //?怎么判断的？是根据得分记票吗？
     long unsigned int mBackupRefKFId;

     // Tracking counters
/**     对于  mnVisible 
在局部地图中地图点投影到当前帧图像上，然后进行匹配获取匹配点对。 
在投影之前需要进行视野判断（通过函数isInFrustum进行）：
  V-D 1) 将MapPoint投影到当前帧, 并判断是否在图像内
  V-D 3) 计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内（这里所说的尺度变化是指地图点到相机中心距离的一段范围，
              如果计算出的地图点到相机中心距离不在这个范围的话就认为这个点在当前帧相机位姿下不能够得到正确、有效、可靠的观测，就要跳过.  ）
  V-D 2) 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回     //?  没看懂
  V-D 4) 根据深度预测尺度（对应特征点在一层）
 */
     int mnVisible;       //? 在帧中的可视次数(比如深度不能为负数什么的)
     int mnFound;      //被找到的次数 和上面的相比要求能够匹配上

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     //TODO 最大最小距离怎么确定的？？是根据金字塔层数计算出来的，还是？
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
