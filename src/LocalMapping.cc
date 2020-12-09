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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"

#include<mutex>
#include<chrono>

namespace ORB_SLAM3
{

LocalMapping::LocalMapping(
    System* pSys, 
    Atlas *pAtlas, 
    const float bMonocular,         //是否单目
    bool bInertial,                             //是否有imu
    const string &_strSeqName
    ):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mbNewInit(false), mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), mIdxIteration(0), infoInertial(Eigen::MatrixXd::Zero(9,9))
{
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling=0;

    //DEBUG: times and data from LocalMapping in each frame

    strSequence = "";//_strSeqName;

    //f_lm.open("localMapping_times" + strSequence + ".txt");
    /*f_lm.open("localMapping_times.txt");

    f_lm << "# Timestamp KF, Num CovKFs, Num KFs, Num RecentMPs, Num MPs, processKF, MPCulling, CreateMP, SearchNeigh, BA, KFCulling, [numFixKF_LBA]" << endl;
    f_lm << fixed;*/
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

//!  局部建图线程
void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        //! 1 标记不接收关键帧，设置为忙碌状态，后面开始处理关键帧了
        //这就是最基本的概念，进来后就开始处理新帧了，处理之前改变为 false，处理完毕（函数结尾）标记true允许接收新帧
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        //! 2  检查序列是否有跟踪线程插入进来的关键帧，并且无坏的imu数据，若不满足条件，本次循环也就相当于结束了
        //? 判断imu是否是坏的标准时什么？？？
        if(CheckNewKeyFrames() && !mbBadImu)
        {
            // std::cout << "LM" << std::endl;
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

            // BoW conversion and insertion in Map
            //! 3 处理列表中的关键帧,(更新共视图，插入关键帧)
            ProcessNewKeyFrame();
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Check recent MapPoints
            //! 4 剔除 ProcessNewKeyFrame 函数中引入的不合格MapPoints
            MapPointCulling();
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            // Triangulate new MapPoints
            //! 5 相机运动过程中与相邻关键帧通过三角化恢复出一些MapPoints
            //对应 3.New points creation
            CreateNewMapPoints();
            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

            // Save here:
            // # Cov KFs
            // # tot Kfs
            // # recent added MPs
            // # tot MPs
            // # localMPs in LBA
            // # fixedKFs in LBA

            //?  终止BA的标志(v3对该位置转移到了这里～，为了防止什么)
            mbAbortBA = false;

            //!  6 已经处理完队列中的最后的一个关键帧，检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
            if(!CheckNewKeyFrames())
            {
                //  Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point t5 = t4, t6 = t4;
            // mbAbortBA = false;

            //DEBUG--
            /*f_lm << setprecision(0);
            f_lm << mpCurrentKeyFrame->mTimeStamp*1e9 << ",";
            f_lm << mpCurrentKeyFrame->GetVectorCovisibleKeyFrames().size() << ",";
            f_lm << mpCurrentKeyFrame->GetMap()->GetAllKeyFrames().size() << ",";
            f_lm << mlpRecentAddedMapPoints.size() << ",";
            f_lm << mpCurrentKeyFrame->GetMap()->GetAllMapPoints().size() << ",";*/
            //--
            int num_FixedKF_BA = 0;     //?  
            // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
            //? 一般什么情况下闭环检测线程会请求停止呢？？
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                //! 7 如果当前地图的关键帧有超过两个的时候，就会进行局部地图的BA
                if(mpAtlas->KeyFramesInMap()>2)
                {
                    //VIO 当前帧所在地图已经完成了IMU初始化
                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        //前两帧到前一帧距离 +  前一帧到当前帧距离
                        float dist = cv::norm(mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()) +  //前一KF   ---->>  当前KF
                                //前一KF的前一KF  ---->>  前一KF距离
                                cv::norm(mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter());  

                        //前一KF到当前KF的时间间隔
                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;  

                        //这部分会导致Eroc数据集之前地图复位,注释掉很有效果，但是双目会不会有问题呀？单目存在尺度问题吗？
#if  1
                        //! 7-1 VIO 当前帧所在地图没有完成BA,并且在10s以内，距离小于2cm。那就进行地图复位
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            // 10s以内，距离小于2cm，进行地图复位
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }
#endif
                        //单目（VIO也算）传感器匹配的内点大于75个，或者非单目传感器匹配内点大于100个， bLarge = true
                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);   //TODO 手动阈值
                        //! 7-2 VIO 进行VIO BA优化
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                    }  //VIO 当前帧所在地图已经完成了IMU初始化
                    //! 7-2 VO 如果是 vo-orb,开始进行局部BA
                    //? 多地图的情况下会有固定帧的存在吗？？
                    else
                    {
                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA);
                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                    }
                }    // 7 如果当前地图的关键帧有超过两个的时候，就会进行局部地图的BA

                t5 = std::chrono::steady_clock::now();

                //!   Initialize IMU here
                //? 为什么在这里启动imu初始化
                //! 8 如果IMU还没有进行初始化，那就进行IMU初始化
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)
                        InitializeIMU(1e2, 1e10, true);     //? 为什么这样赋值，难道是为了给一个初始化种子吗？0这个种子不好？
                    else
                        InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                // VI-E local keyframes culling
                // 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                // 剔除的标准是：该关键帧的90%的MapPoints可以被其它关键帧观测到
                //!  trick! 
                // Tracking中先把关键帧交给LocalMapping线程
                // 并且在Tracking中InsertKeyFrame函数的条件比较松，交给LocalMapping线程的关键帧会比较密
                // 在这里再删除冗余的关键帧
                // 也是ORB_SLAM2的创新点之一吧(guoqing)
                // Check redundant local Keyframes
                //! 9 冗余关键帧剔除
                KeyFrameCulling();

                t6 = std::chrono::steady_clock::now();

                //!  10 VIO： 当前地图在不到100s的时候。
                if ((mTinit<100.0f) && mbInertial)
                {
                    //地图完成了初始化，跟踪线程跟踪正常
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
                    {
                        //! 10-1 如果当前KF地图没有BA1优化，在经过足够时间后（5s）开始进行BA1优化（imu初始化）
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true); // 1.f, 1e5
                                else
                                    InitializeIMU(1.f, 1e5, true); // 1.f, 1e5

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        //else if (mbNotBA2){
                        //! 10-2 当前地图经过15s后，并且完成了BA1，那就开始BA2
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){ // 15.0f
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true); // 0.f, 0.f
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                            }
                        }

                        // scale refinement
                        //! 10-3 尺度精细优化（只针对单目），每次间隔10s就连续执行0.5秒
                        if (((mpAtlas->KeyFramesInMap())<=100) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            cout << "start scale ref" << endl;
                            if (mbMonocular)
                                ScaleRefinement();          //TODO 2020-12-08   太累了，不干了～～
                            cout << "end scale ref" << endl;
                        }
                    }
                }   // imu部分 ～～跳过
            }  // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping

            std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
            //!  关键帧插入到LoopClosing线程当中的关键帧队列里，LoopClosing线程执行操作
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();

            //???  这一堆干嘛的？？？
            double t_procKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
            double t_MPcull = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            double t_CheckMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t3 - t2).count();
            double t_searchNeigh = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t4 - t3).count();
            double t_Opt = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t5 - t4).count();
            double t_KF_cull = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t6 - t5).count();
            double t_Insert = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t8 - t7).count();

            //DEBUG--
            /*f_lm << setprecision(6);
            f_lm << t_procKF << ",";
            f_lm << t_MPcull << ",";
            f_lm << t_CheckMP << ",";
            f_lm << t_searchNeigh << ",";
            f_lm << t_Opt << ",";
            f_lm << t_KF_cull << ",";
            f_lm << setprecision(0) << num_FixedKF_BA << "\n";*/
            //--

        }     //检查序列是否有跟踪线程插入进来的关键帧，并且无坏的imu数据
        //? 下面这个if里面的imu该怎么理解
        else if(Stop() && !mbBadImu)   // 当要终止当前线程的时候，结束前的处理
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                // cout << "LM: usleep if is stopped" << endl;
                // 如果还没有结束利索,那么等
                usleep(3000);
            }
            // 然后确定终止了就跳出这个线程的主循环,   //?   这个线程一旦跳出就终止这个线程了呀～～ 什么事情会导致 break ??
            if(CheckFinish())
                break;
        }

        // 查看是否有复位线程的请求
        ResetIfRequested();  // 执行复位操作:清空关键帧缓冲区,清空待cull的地图点缓冲

        // Tracking will see that Local Mapping is busy
        //一个keyframe已经处理完了，需要Tracking线程插入下一个KeyFrame进来
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        // cout << "LM: normal usleep" << endl;
        usleep(3000);
    }

    //f_lm.close();
    // 设置线程已经终止
    SetFinish();
}

//在单目初始化和跟踪线程里面的 CreateNewKeyFrame 函数里面会被调用
void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    //cout << "ProcessNewKeyFrame: " << mlNewKeyFrames.size() << endl;
    //! 1 弹出一帧KF开始处理
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    //! 2 计算该关键帧特征点的Bow映射关系
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    //! 3 当前帧在跟踪局部地图过程中新匹配上的MapPoints和当前关键帧绑定
    // 在TrackLocalMap函数中将局部地图中的MapPoints与当前帧进行了匹配，
    // 但没有对这些匹配上的MapPoints与当前帧进行关联
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    //! 3-1 对当前处理的这个关键帧中的所有的地图点展开遍历
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                //! 3-2 如果观测到该MapPoint的关键帧列表中,没有mpCurrentKeyFrame这个关键帧
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    //! 3-3 添加观测
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    //! 3-4 获得该点的平均观测方向和观测距离范围
                    pMP->UpdateNormalAndDepth();
                    //! 3-5 加入当前关键帧后，更新3d点的最佳描述子
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 当前帧生成的MapPoints
                    // 将双目或RGBD跟踪过程中新插入的MapPoints放入mlpRecentAddedMapPoints，等待检查
                    // CreateNewMapPoints函数中通过三角化也会生成MapPoints
                    // 这些MapPoints都会经过MapPointCulling函数的检验
                    mlpRecentAddedMapPoints.push_back(pMP);  // 认为这些由当前关键帧生成的地图点不靠谱,将其加入到待检查的地图点列表中
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    //! 4 更新关键帧间的连接关系，Covisibility图和Essential图(tree)
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    //! 5 将该关键帧插入到当前地图中
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

//  剔除 ProcessNewKeyFrame 和 CreateNewMapPoints 函数中引入的质量不好的 MapPoints
void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    // 观测阈值
    //! 这里有手动阈值
    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3; // MODIFICATION_STEREO_IMU here 3
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    // 遍历等待检查的MapPoints
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        //! 步骤1：已经是坏点的MapPoints直接从检查链表中删除
        if(pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if(pMP->GetFoundRatio()<0.25f)
        {
            // ORB_SLAM2 步骤2：将不满足VI-B条件的MapPoint剔除
            // VI-B 条件1：
            //! 步骤2：跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
            // IncreaseFound / IncreaseVisible < 25%，注意不一定是关键帧。
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // 步骤3：将不满足VI-B条件的MapPoint剔除
            // VI-B 条件2：从该点建立开始，到现在已经过了不小于2个关键帧
            //! 步骤3：但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            //! 步骤4：从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
            // 因此没有SetBadFlag()，仅从队列中删除，放弃继续对该MapPoint的检测
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
            borrar--;
        }
    }
    //cout << "erase MP: " << borrar << endl;
}

// 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    // 不同传感器下要求不一样,单目的时候要求更高一些
    //TODO 这里有手动阈值
    int nn = 10;
    // For stereo inertial case
    if(mbMonocular)
        nn=20;
    //!  1：在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻帧vpNeighKFs
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    //! 2:  VIO 共视帧不够时，在当前帧的前一帧存在的情况下，递归获取前一帧直到满足数量要求或者前一帧不存在
    if (mbInertial)
    {
        KeyFrame* pKF = mpCurrentKeyFrame;
        int count=0;
        //递归获取前一帧直到满足数量要求或者前一帧不存在
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
        {
            vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    //TODO 手动阈值
    float th = 0.6f;

    ORBmatcher matcher(th,false);

    //! 3：获取当前帧的旋转和位移
    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    //获取当前帧相机内参
    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    //?  用于后面的点深度的验证;这里为什么选1.5?
    //TODO 这里有手动阈值
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    // Search matches with epipolar restriction and triangulate
    //! 4 遍历获取的相邻帧
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        //? 在这里检查是否又来了新的关键帧，如果有新的关键帧则退出此函数，去处理新的关键帧
        if(i>0 && CheckNewKeyFrames())// && (mnMatchesInliers>50))
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];  //临接关键帧

        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        //邻接的关键帧在世界坐标系中的坐标
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        // 基线向量，两个关键帧间的相机中心位移
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        //! 4-1 判断相机基线是不是足够长
        // 判定条件是什么？？
        if(!mbMonocular)    //双目
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else  // 单目情况
        {
            // 邻接关键帧的场景深度中值
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            // baseline与景深的比例
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            // 如果特别远(比例特别小)，那么不考虑当前邻接的关键帧，不生成3D点
            // 导致这个结果有两个因素:一个还是两个关键帧之间的基线太短,另外一个就是相邻关键帧看到的空间的尺度非常大
            //? 这些是什么原理在支撑？？
            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        //! 4-2 ：根据两个关键帧的位姿计算它们之间的基础矩阵
        //TODO 基础必会知识点
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        //! 4-3 ：通过极线约束限制匹配时的搜索范围，进行特征点匹配
        //TODO 基础必会知识点
        vector<pair<size_t,size_t> > vMatchedIndices;
        //? 
        /**
         * 赋值为 true 需要必须为VIO系统，并且满足下面两个条件之一
         *  1： 必须为VIO系统，并且VO部分完成了BA优化，但是VIO部分还没有完成BA优化
         *  2：跟踪线程处于短暂丢失状态
        */
        bool bCoarse = mbInertial &&   //必须为VIO系统
                //VO部分完成了BA优化，但是VIO部分还没有完成BA优化
                ((!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && mpCurrentKeyFrame->GetMap()->GetIniertialBA1())||
                //或者跟踪线程处于短暂跟踪丢失情况
                 mpTracker->mState==Tracking::RECENTLY_LOST);
        // 用对极约束来约束匹配时的搜索范围，对满足对极约束的特征点进行特征点匹配，vMatchedIndices为匹配结果
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false,bCoarse);  //TODO 对极约束没看

        //! 4-4：获取临接关键帧的旋转和位移
        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        //获取临接KF的相机内参
        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        //! 4-5：对每对匹配点通过三角化生成3D点（和 Triangulate函数差不多）
        const int nmatches = vMatchedIndices.size();
        //遍历匹配点对
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            //! 4-5-1：取出匹配特征点
            // 当前匹配对在当前关键帧中的索引
            const int &idx1 = vMatchedIndices[ikp].first;
            // 当前匹配对在邻接关键帧中的索引
            const int &idx2 = vMatchedIndices[ikp].second;
            // 当前匹配在当前关键帧中的特征点
            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]    //单目
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]  //双目的左目
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];   //双目的右目
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0); //当前帧为双目
            //判断匹配的特征点是否为右目产生
            //idx1为单目，或者为双目的左目是，该变量为 false，这是一个记录是否为右目的特征点的变量。
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                               : true;
            // 当前匹配在邻接关键帧中的特征点
            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];      // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);        //临接关键帧为双目
            //判断匹配的特征点是否为右目产生
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            //?????  这一部分是什么呀？？是不同相机模型的三角花策略有所不同吗？？这一部分先跳过，后头再解决这一部分  2020-11-27
            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    Rcw2 = pKF2->GetRightRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation();
                    Tcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    Rcw2 = pKF2->GetRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation();
                    Tcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    Rcw2 = pKF2->GetRightRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation();
                    Tcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    Rcw1 = mpCurrentKeyFrame->GetRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    Rcw2 = pKF2->GetRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation();
                    Tcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
            }

            // Check parallax between rays
            /**
             * 检查光线之间的视差
             * 这里的kp1.pt.x为kp1这个关键点在图像中的x轴坐标，cx1为像素坐标中原点的移动距离，invfx1为对应fx的倒数
             * (kp1.pt.x-cx1)*invfx1：  像素的x坐标--->> 相机归一化坐标系下的 x 坐标
             * (kp1.pt.y-cy1)*invfy1：  像素的y坐标--->> 相机归一化坐标系下的 y 坐标
             * 
             * xn1为kp1这个关键点在归一化平面的像素坐标向量
            */

            //! 4-5-2：利用匹配点反投影到世界坐标系，然后求解出视差角
            //xn1 xn2 为相机坐标系归一化坐标    参考slam14第二版 式子（5.5）
            //从像素坐标系，变换到归一化相机坐标系
            cv::Mat xn1 = pCamera1->unprojectMat(kp1.pt);
            cv::Mat xn2 = pCamera2->unprojectMat(kp2.pt);

            // 由相机坐标系--->>  世界坐标系(得到的是那条反投影射线的一个同向向量在世界坐标系下的表示,还是只能够表示方向，但是也能够得到视差角余弦值）
            //从相机坐标系，变换到世界坐标系
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            // 这个就是求向量之间角度公式
            //（向量a × 向量b）/（向量a的模 × 向量b的模）= 夹角余弦值
            // 向量乘积：a * b = |a|*|b|*cos<a,b> 推出：cos<a,b> = (a * b)/(|a|*|b|)
            //求解匹配点在对应在世界坐标系下，向量之间的角度
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            // 加1是为了让cosParallaxStereo随便初始化为一个很大的值;不懂的话先向下看就好了
            //一般相邻的关键帧应该不会有这么大的视角吧,90°多,不过感觉这个也不好说啊;不过下面的程序中认为如果视差角的余弦值大于1是不正常的
            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            //! 4-5-3：对于双目，利用双目得到视差角      //参考 slam14V2 图5.6
            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            //??   得到双目观测的视差角
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            //! 4-5-4：三角化恢复出3D点   
            cv::Mat x3D;  //计算出来的目前还是相机坐标系，之后会利用该变量计算出来世界坐标系坐标
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
               (cosParallaxRays<0.9998 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
            {
                // Linear Triangulation Method
                //参考博客   https://www.cnblogs.com/fanglai-you/p/11276148.html
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                // 归一化之前的检查
                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                // Euclidean coordinates 归一化成为齐次坐标,然后提取前面三个维度作为欧式坐标
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            //TODO 双目先跳过
             //??  视差大的时候使用双目信息来恢复 - 直接反投影了
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            // 又转换成为了行向量了...
            cv::Mat x3Dt = x3D.t();

            if(x3Dt.empty()) continue;
            //Check triangulation in front of cameras
            //! 4-5-5：检测生成的3D点是否在相机前方,不在的话就放弃这个点
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            //! 4-5-6：计算3D点在当前关键帧下的重投影误差，误差太大就放弃该点
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;
                // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）自由度2
                //? 这里改怎么理解？？ 误差较大的情况下，放弃改点，sigmaSquare1起到什么作用？？？
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            //? 双目，跳过
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            //! 4-5-7 计算3D点在另一个关键帧（临接KF）下的重投影误差，误差过大也丢弃改点
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            //! 4-5-8：检查尺度连续性
            // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                continue;

            const float ratioDist = dist2/dist1;
            //??    金字塔尺度因子的比例??有啥用？？
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            //? ratioDist*ratioFactor < ratioOctave 或 ratioDist/ratioOctave > ratioFactor表明尺度变化是连续的
            //? 还不是非常明白,感觉这里的意思大致应该就是, 深度值的比例和图像金字塔的比例不应该差太多
            //? ratioDist < (ratioOctave/ratioFactor) , ratioDist > (ratioOctave*ratioFactor) ,中间那一段不行
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            //! 4-5-9：三角化生成3D点成功，构造成MapPoint
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpAtlas->GetCurrentMap());

            //! 4-5-10：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            // b.该MapPoint的描述子
            // c.该MapPoint的平均观测方向和深度范围
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);

            //! 4-5-11：将新产生的点放入检测队列mlpRecentAddedMapPoints， 这些MapPoints都会送往MapPointCulling函数进行检验
            mlpRecentAddedMapPoints.push_back(pMP);
        }  //遍历匹配点对
    }   //遍历获取的相邻帧
}

//  Find more matches in neighbor keyframes and fuse point duplications
// 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
void LocalMapping::SearchInNeighbors()
{
    //! 1：获得当前关键帧在covisibility图中权重排名前nn的邻接关键帧(一级相邻关键帧)
    // 找到当前帧一级相邻与二级相邻关键帧       
    /**skylor  关于一级 和 二级的概念：
     * 一级：与当前帧共视程度最靠前的 nn 个
     * 二级： 注意，这里对象变成一级相邻，而不是当前帧了。而是是与一级帧的共视程度最靠前的 5 个
     * 这里并不是确定是这些，nn和5是理想程度下的最大值，因为这些值里面包括 已经标记坏掉的帧 和 自身  //帧是通过什么判断坏掉的～～
    */

    // 单目情况要20个邻接关键帧，双目或者RGBD则要10个
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)  nn=20;

    //候选的一级相邻关键帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    // 筛选之后的一级相邻关键帧以及二级相邻关键帧
    vector<KeyFrame*> vpTargetKFs;
    // 开始对所有候选的一级关键帧展开遍历：
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    //! 2 添加二级关键帧（与一级关键帧共视程度最强的20个）
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
    {
        //取出1级关键帧的20个共视关键帧（二级关键帧）
        const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);        //TODO 手动阈值
        //对二级关键帧进行遍历
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            //如果KF是坏的，或者与当前帧发生了融合过程，再或者为当前帧的话。结束本次循环
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            //放入筛选后的容器中
            vpTargetKFs.push_back(pKFi2);
            //标记该二级KF与当前帧发生过融合
            pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
            break;
    }

    // Extend to temporal neighbors
    //! 3 VIO：在筛选的关键帧少于20个，那就迭代添加当前帧的前一KF的前一KF，直到满足数量要求，或者不存在前一KF
    if(mbInertial)
    {
        //获取当前帧的前一关键帧
        KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
        //如果筛选的关键帧少于20个，并且当前KF的前一KF存在，那就迭代添加前一KF的前一KF
        while(vpTargetKFs.size()<20 && pKFi)
        {
            //前一KF坏掉，或者已经参与了与当前帧的融合，那就直接开始迭代下一次的
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)
            {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            //放入筛选后的容器中
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
            pKFi = pKFi->mPrevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    // 使用默认参数, 最优和次优比例0.6,匹配时检查特征点的旋转
    ORBmatcher matcher;
    //! 4 ：将当前帧的MapPoints分别与一级二级相邻帧(的MapPoints)进行融合 -- 正向
    //获取当前帧的MapPoint
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        // 投影当前帧的MapPoints到相邻关键帧pKFi中，并判断是否有重复的MapPoints
        // 1.如果MapPoint能匹配关键帧的特征点，并且该点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
        // 2.如果MapPoint能匹配关键帧的特征点，并且该点没有对应的MapPoint，那么为该点添加MapPoint
        // 注意这个时候对地图点融合的操作是立即生效的
        matcher.Fuse(pKFi,vpMapPointMatches);
        //如果是双目的话，右目也要进行相应的操作    //? 说双目是不是不太合适？ 这个应该是特殊传感器，比如双目鱼眼相机？？
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true);
    }

    if (mbAbortBA)
        return;

    // Search matches by projection from target KFs in current KF
    // 用于进行存储要融合的一级邻接和二级邻接关键帧所有MapPoints的集合
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    //! 5 ：将一级二级相邻帧的MapPoints分别与当前帧（的MapPoints）进行融合 -- 反向
    // 遍历每一个一级邻接和二级邻接关键帧
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints,找出需要进行融合的并且加入到集合中 vpFuseCandidates 
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)  //右半部分是避免重复添加
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    // 进行融合操作,其实这里的操作和上面的那个融合操作是完全相同的,
    //不过上个是"每个关键帧和当前关键帧的地图点进行融合",而这里的是"当前关键帧和所有邻接关键帧的地图点进行融合"
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
    //如果是双目，右目也要进行相关操作
    if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,true);

    // Update points
    //! 6：更新当前帧MapPoints的描述子，深度，观测主方向等属性
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // 在所有找到pMP的关键帧中，获得最佳的描述子
                pMP->ComputeDistinctiveDescriptors();
                // 更新平均观测方向和观测距离
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    //! 7：更新当前帧的MapPoints后更新与其它帧的连接关系
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mpCamera->toK();
    const cv::Mat &K2 = pKF2->mpCamera->toK();


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

//可以继续建图和产生关键帧了
void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

//设置是否允许接受关键帧
void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    const int Nd = 21; // MODIFICATION_STEREO_IMU 20 This should be the same than that one from LIBA
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // Compoute last KF from optimizable window:
    unsigned int last_ID;
    if (mbInertial)
    {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }



    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFrame* pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;
                        const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>thObs)
                                    break;
                            }
                        }
                        if(nObs>thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial)
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                    continue;

                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && (cv::norm(pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition())<0.02) && (t<3))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                }
            }
            else
            {
                pKF->SetBadFlag();
            }
        }
        if((count > 20 && mbAbortBA) || count>100) // MODIFICATION originally 20 for mbabortBA check just 10 keyframes
        {
            break;
        }
    }
}


cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Reseting Atlas in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested=false;
            mbResetRequestedActiveMap = false;

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mIdxInit=0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting Local Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

//! IMU初始化函数
void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)   // bFIBA = true
{
    if (mbResetRequested)
        return;

    //设定需要的最小KF个数
    float minTime;
    int nMinKF;
    if (mbMonocular)
    {
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF = 10;
    }


    if(mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    //  temporal 可以理解递归向前添加的KF
    // Retrieve all keyframe in temporal order
    //! 1  存储递归向前添加的KF,添加干净
    list<KeyFrame*> lpKF;       
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)
        return;

    //获取除了当前帧以外时间上最新的KF
    mFirstTs=vpKF.front()->mTimeStamp;
    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)
        return;

    //我去，对象元素变量，你为啥不用m开头
    bInitializing = true;

    //这个时候如果有新的关键帧，先去处理新的关键帧（可能后面的处理比较耗时），处理完毕后并将它添加进来
    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0,0,0,0,0,0);            //无用行，而且对象里面也没有静态对象，暂时保留这里吧

    // Compute and KF velocities mRwg estimation
    //如果当前帧所在的地图还没有完成imu初始化



    //! 如果当前帧所在地图还没有完成imu初始化
    //?  IMU初始化B : 尺度和重力估计（尺度在后面直接预设估计为1.0）
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    {
        cv::Mat cvRwg;
        cv::Mat dirG = cv::Mat::zeros(3,1,CV_32F);
        //遍历VIO KF
        for(vector<KeyFrame*>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)
        {
            //VIO KF 必须已经完成了预计分，并且必须具备 前一KF存在
            if (!(*itKF)->mpImuPreintegrated)
                continue;
            if (!(*itKF)->mPrevKF)
                continue;

            //? IMU初始化 D : 速度估计？？
            dirG -= (*itKF)->mPrevKF->GetImuRotation()*(*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            //计算速度，两帧率位移除以时间，得到速度
            cv::Mat _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        //! 2 IMU初始化：C，对应论文ORBVIO公式14，计算重力的偏差角度 R_wi
        dirG = dirG/cv::norm(dirG);                         //? g_w ?
        cv::Mat gI = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);       //? g_i
        cv::Mat v = gI.cross(dirG);     //Mat::cross  计算叉乘
        const float nv = cv::norm(v);
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);       //计算角度信息 deta = ang
        cv::Mat vzg = v*ang/nv;                 //角度大小乘以方向信息 = 带有方向的角度 = 论文ORBVIO里面的公式14里面的   v^  * theta 
        cvRwg = IMU::ExpSO3(vzg);         //论文ORBVIO中的公式14里面的 R_wi
        mRwg = Converter::toMatrix3d(cvRwg);        //同上， ORBVI论文中公式(14)最上面一行公式，这里得出的就是 R_wi
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
    }   //当前帧所在地图还没有完成imu初始化
    //如果没有完成初始化，那就在初始化的时候利用当前帧的bias作为最初的迭代种子
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        mbg = Converter::toVector3d(mpCurrentKeyFrame->GetGyroBias());
        mba = Converter::toVector3d(mpCurrentKeyFrame->GetAccBias());
    }

    mScale=1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    //! 3 IMU初始化C，计算出bias和精细化的重力方向和尺度（内部的雅克比矩阵和跟踪线程的VIO优化90%相似，但是多了尺度和重力偏差旋转量R_wi）
    //false,false : 不固定速度顶点，不使用高斯牛顿
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    /*cout << "scale after inertial-only optimization: " << mScale << endl;
    cout << "bg after inertial-only optimization: " << mbg << endl;
    cout << "ba after inertial-only optimization: " << mba << endl;*/

    //! 4 优化的尺度太小，抛弃优化后的尺度，退出函数
    if (mScale<1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }



    // Before this line we are not changing the map

    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        mpTracker->UpdateFrameIMU(mScale,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    // Check if initialization OK
    //?? 是不是写错了，这里多加了一个感叹号？？？  
    //? 答：没写错，表示如果Map没有初始化，那就说明还没有对KF标记，那就先标记一下完成了初始化了
    if (!mpAtlas->isImuInitialized())       
        for(int i=0;i<N;i++)
        {
            KeyFrame* pKF2 = vpKF[i];
            pKF2->bImu = true;
        }

    /*cout << "Before GIBA: " << endl;
    cout << "ba: " << mpCurrentKeyFrame->GetAccBias() << endl;
    cout << "bg: " << mpCurrentKeyFrame->GetGyroBias() << endl;*/

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    if (bFIBA)
    {
        //! 5 又来一次VIO KF的局部窗口优化（里面雅克比部分和跟踪线程里面的VIO优化没啥区别，只不过跟踪线程仅仅对相邻的两个KF优化）
        if (priorA!=0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, true, priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, false);
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    // If initialization is OK
    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    //已经进行了imu初始化过了，如果还没有标记，那就标记一下
    if (!mpAtlas->isImuInitialized())
    {
        cout << "IMU in Map " << mpAtlas->GetCurrentMap()->GetId() << " is initialized" << endl;
        mpAtlas->SetImuInitialized();
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;
        mpCurrentKeyFrame->bImu = true;
    }


    mbNewInit=true;
    mnKFs=vpKF.size();
    mIdxInit++;

    //? IMU初始化后就开始剔除掉mlNewKeyFrames中的记录
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    mpTracker->mState=Tracking::OK;
    bInitializing = false;


    /*cout << "After GIBA: " << endl;
    cout << "ba: " << mpCurrentKeyFrame->GetAccBias() << endl;
    cout << "bg: " << mpCurrentKeyFrame->GetGyroBias() << endl;
    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    double t_update = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    double t_viba = std::chrono::duration_cast<std::chrono::duration<double> >(t5 - t4).count();
    cout << t_inertial_only << ", " << t_update << ", " << t_viba << endl;*/

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

 //! 优化,换汤不换药，同IMU初始化，使用的还是考虑重力方向和尺度信息的VIO KF优化，不注释了～
void LocalMapping::ScaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested)
        return;

    // Retrieve all keyframes in temporal order
    //! 只要出现 temporal 就按照共视帧窗口里面的KF
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();

    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    //! 优化,换汤不换药，同IMU初始化，使用的还是考虑重力方向和尺度信息的VIO KF优化，不注释了～
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }

    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();

    // To perform pose-inertial opt w.r.t. last keyframe
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}



bool LocalMapping::IsInitializing()
{
    return bInitializing;
}


double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
        return 0.0;
}

KeyFrame* LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

} //namespace ORB_SLAM
