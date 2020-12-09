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


#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM3
{

LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0), mnMergeNumCoincidences(0),
    mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0)
{
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF = static_cast<KeyFrame*>(NULL);
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

//! 闭环检测线程
void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        //NEW LOOP AND MERGE DETECTION ALGORITHM
        //----------------------------
        //! 1  检测是否插入进来关键帧
        if(CheckNewKeyFrames())
        {
            //?  为什么一进来就清空他？？
            if(mpLastCurrentKF)
            {
                mpLastCurrentKF->mvpLoopCandKFs.clear();
                mpLastCurrentKF->mvpMergeCandKFs.clear();
            }
            //参考论文  4-A :  Place Recognition
            //! 2 位置识别，会标记是否检测到闭环或者匹配
            if(NewDetectCommonRegions())
            {
                //! 3 如果检测到不同地图匹配,那就执行地图融合
                if(mbMergeDetected)
                {
                    //!  VI-orb
                    if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                        (!mpCurrentKF->GetMap()->isImuInitialized()))
                    {
                        cout << "IMU is not initilized, merge is aborted" << endl;
                    }
                    //! vo-orb
                    else
                    {
                        Verbose::PrintMess("*Merged detected", Verbose::VERBOSITY_QUIET);
                        Verbose::PrintMess("Number of KFs in the current map: " + to_string(mpCurrentKF->GetMap()->KeyFramesInMap()), Verbose::VERBOSITY_DEBUG);
                        // 得到世界坐标系到匹配帧的变换
                        cv::Mat mTmw = mpMergeMatchedKF->GetPose();
                        // 得到世界坐标系到匹配帧的sim3变换
                        g2o::Sim3 gSmw2(Converter::toMatrix3d(mTmw.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTmw.rowRange(0, 3).col(3)),1.0);
                        //得到世界坐标系到当前帧的变换
                        cv::Mat mTcw = mpCurrentKF->GetPose();
                        //? 得到世界坐标系到当前帧的sim3变换
                        g2o::Sim3 gScw1(Converter::toMatrix3d(mTcw.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcw.rowRange(0, 3).col(3)),1.0);
                        
                        
                        //? 得到世界坐标系到当前帧的sim3变换,下面的命名是不是有点怪？？
                        g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                        g2o::Sim3 gSw1m = mg2oMergeSlw;
                        mSold_new = (gSw2c * gScw1);


                        //!  vi 
                        if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                        {
                            if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                                   mpCurrentKF->GetMap()->GetIniertialBA1()) // TODO, maybe with GetIniertialBA1
                            {
                                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                phi(0)=0;
                                phi(1)=0;
                                mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
                            }
                        }   // vi系统


//                        cout << "tw2w1: " << mSold_new.translation() << endl;
//                        cout << "Rw2w1: " << mSold_new.rotation().toRotationMatrix() << endl;
//                        cout << "Angle Rw2w1: " << 180*LogSO3(mSold_new.rotation().toRotationMatrix())/3.14 << endl;
//                        cout << "scale w2w1: " << mSold_new.scale() << endl;

                        /**
                         * gSmw2: =    mTmw       世界坐标系到匹配帧的sim3变换
                         * gSw2c：=  mg2oMergeSlw.inverse()  =  gScw.inverse()  = gSwc
                         * gScw1 :  mTcw    （当前帧的位姿）
                         * 
                         * 
                        */
                         // 得到世界坐标系到匹配帧的sim3变换
                        mg2oMergeSmw = gSmw2 * gSw2c * gScw1;
                        //世界坐标系到当前帧的sim3变换
                        mg2oMergeScw = mg2oMergeSlw;

                        // TODO UNCOMMENT
                        //! vi
                        //如果跟踪线程使用到了imu那就执行 MergeLocal2() 的匹配融合
                        if (mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO)
                            MergeLocal2();
                        //! vo 的匹配融合
                        //否则为纯视觉，使用MergeLocal()的匹配融合
                        else
                            MergeLocal();  //地图融合
                    }   //vi

                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(1);

                    // Reset all variables
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                    mbMergeDetected = false;

                    //! 到这里本次匹配处理完毕了，但是为什么初始化闭环的状态，而不是匹配的初始化？
                    //我的理解是，每一次闭环只处理一个，避免相互影响，其中以不同地图的匹配作为做高处理优先级
                    if(mbLoopDetected)
                    {
                        // Reset Loop variables
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }

                } //如果检测到匹配

                //! 4 如果检测到闭环,那就执行本地图的闭环矫正
                if(mbLoopDetected)
                {
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(0);


                    Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

                    mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
                    if(mpCurrentKF->GetMap()->IsInertial())
                    {
                        cv::Mat Twc = mpCurrentKF->GetPoseInverse();
                        g2o::Sim3 g2oTwc(Converter::toMatrix3d(Twc.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(Twc.rowRange(0, 3).col(3)),1.0);
                        g2o::Sim3 g2oSww_new = g2oTwc*mg2oLoopScw;

                        Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                        //cout << "tw2w1: " << g2oSww_new.translation() << endl;
                        //cout << "Rw2w1: " << g2oSww_new.rotation().toRotationMatrix() << endl;
                        //cout << "Angle Rw2w1: " << 180*phi/3.14 << endl;
                        //cout << "scale w2w1: " << g2oSww_new.scale() << endl;

                        if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)
                        {
                            if(mpCurrentKF->GetMap()->IsInertial())
                            {
                                // If inertial, force only yaw
                                if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                                        mpCurrentKF->GetMap()->GetIniertialBA2()) // TODO, maybe with GetIniertialBA1
                                {
                                    phi(0)=0;
                                    phi(1)=0;
                                    g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
                                    mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
                                }
                            }

                            mvpLoopMapPoints = mvpLoopMPs;//*mvvpLoopMapPoints[nCurrentIndex];
                            CorrectLoop();
                        }
                        else
                        {
                            cout << "BAD LOOP!!!" << endl;
                        }
                    }
                    else
                    {
                        mvpLoopMapPoints = mvpLoopMPs;
                        CorrectLoop();
                    }

                    // Reset all variables
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                    mbLoopDetected = false;
                }  //如果检测到匹配

            }   //如果位置识别检测到共同区域

            mpLastCurrentKF = mpCurrentKF;
        }

        ResetIfRequested();

        if(CheckFinish()){
            // cout << "LC: Finish requested" << endl;
            break;
        }

        usleep(5000);
    }

    //ofstream f_stats;
    //f_stats.open("PlaceRecognition_stats" + mpLocalMapper->strSequence + ".txt");
    //f_stats << "# current_timestamp, matched_timestamp, [0:Loop, 1:Merge]" << endl;
    //f_stats << fixed;
    //for(int i=0; i< vdPR_CurrentTime.size(); ++i)
    //{
    //    f_stats  << 1e9*vdPR_CurrentTime[i] << "," << 1e9*vdPR_MatchedTime[i] << "," << vnPR_TypeRecogn[i] << endl;
    //}

    //f_stats.close();

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}


//参考论文  4-A :  Place Recognition
bool LoopClosing::NewDetectCommonRegions()
{
    {
        //!step 1  弹出一个来自于局部建图的KF
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        //!  Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
        mpCurrentKF->mbCurrentPlaceRecognition = true;   //?  

        //!step 2  取出所属的地图
        mpLastMap = mpCurrentKF->GetMap();
    }

    //?  属于VI-orb，并且没有进行过BA  ????     跳过
    if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA1())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    //? 双目处理
    if(mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    //! step 3 地图关键帧小于12的话，KF添加到数据库并退出处理，太少没进行位置识别必要
    //?  为什么必须小于12个呀？？ 
    if(mpLastMap->GetAllKeyFrames().size() < 12)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    //Check the last candidates with geometric validation    //使用几何验证检查最后的候选帧
    // Loop candidates
    bool bLoopDetectedInKF = false;
    bool bCheckSpatial = false;

    //! step 4相同地图，如果存在通过闭环匹配sim3  通过的关键帧，那就开始引导sim3闭环匹配细化
    //（看函数的结构一般在上一次执行的时候才会在本次满足条件吧）
    if(mnLoopNumCoincidences > 0)   //初始化为0，换句话来说在未检测到闭环之前来不了这里，而检测闭环偏偏就在这个函数的最下面几行～
    {
        bCheckSpatial = true;
        // Find from the last KF candidates
        //计算上一帧到当前帧的相对变换
        cv::Mat mTcl = mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse();
        //转化为sim3形式
        g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcl.rowRange(0, 3).col(3)),1.0);
        // 从世界坐标系到当前帧的Sim3变换   
        // mg2oLoopSlw 会在后面进行更新，直接在后面获取 世界坐标系w  -->> 当前c  的sim3变换作为下一次使用的值
        g2o::Sim3 gScw = gScl * mg2oLoopSlw;
        int numProjMatches = 0;
        vector<MapPoint*> vpMatchedMPs;
        //精细化sim3变换
        //! step 4 : 引导匹配细化？？
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);
        if(bCommonRegion)
        {

            bLoopDetectedInKF = true;

            mnLoopNumCoincidences++;
            mpLoopLastCurrentKF->SetErase();
            mpLoopLastCurrentKF = mpCurrentKF;
            mg2oLoopSlw = gScw;
            mvpLoopMatchedMPs = vpMatchedMPs;


            mbLoopDetected = mnLoopNumCoincidences >= 3;
            mnLoopNumNotFound = 0;

            if(!mbLoopDetected)
            {
                //f_succes_pr << mpCurrentKF->mNameFile << " " << "8"<< endl;
                //f_succes_pr << "% Number of spatial consensous: " << std::to_string(mnLoopNumCoincidences) << endl;
                cout << "PR: Loop detected with Reffine Sim3" << endl;
            }
        }
        else
        {
            bLoopDetectedInKF = false;
            /*f_succes_pr << mpCurrentKF->mNameFile << " " << "8"<< endl;
            f_succes_pr << "% Number of spatial consensous: " << std::to_string(mnLoopNumCoincidences) << endl;*/

            mnLoopNumNotFound++;
            if(mnLoopNumNotFound >= 2)
            {
                /*for(int i=0; i<mvpLoopLastKF.size(); ++i)
                {
                    mvpLoopLastKF[i]->SetErase();
                    mvpLoopCandidateKF[i]->SetErase();
                    mvpLoopLastKF[i]->mbCurrentPlaceRecognition = true;
                }

                mvpLoopCandidateKF.clear();
                mvpLoopLastKF.clear();
                mvg2oSim3LoopTcw.clear();
                mvnLoopNumMatches.clear();
                mvvpLoopMapPoints.clear();
                mvvpLoopMatchedMapPoints.clear();*/

                mpLoopLastCurrentKF->SetErase();
                mpLoopMatchedKF->SetErase();
                //mg2oLoopScw;
                mnLoopNumCoincidences = 0;
                mvpLoopMatchedMPs.clear();
                mvpLoopMPs.clear();
                mnLoopNumNotFound = 0;
            }

        }
    }

    //Merge candidates
    bool bMergeDetectedInKF = false;
    //! step 5不同地图，如果存在通过闭环匹配sim3  通过的关键帧，那就开始引导sim3闭环匹配细化
    if(mnMergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        //上一帧到当前帧的变换
        cv::Mat mTcl = mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse();
        //上一帧到当前帧的sim3变换
        g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcl.rowRange(0, 3).col(3)),1.0);
        //得到g2o优化后的  世界坐标系到当前帧的sim3变换
        g2o::Sim3 gScw = gScl * mg2oMergeSlw;
        int numProjMatches = 0;
        vector<MapPoint*> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
        if(bCommonRegion)
        {
            //cout << "BoW: Merge reffined Sim3 transformation suscelful" << endl;
            bMergeDetectedInKF = true;

            mnMergeNumCoincidences++;
            mpMergeLastCurrentKF->SetErase();
            mpMergeLastCurrentKF = mpCurrentKF;
            //更新 g2o优化后的  世界坐标系到当前帧的sim3变换
            mg2oMergeSlw = gScw;
            mvpMergeMatchedMPs = vpMatchedMPs;

            mbMergeDetected = mnMergeNumCoincidences >= 3;
        }
        else
        {
            //cout << "BoW: Merge reffined Sim3 transformation failed" << endl;
            mbMergeDetected = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            if(mnMergeNumNotFound >= 2)
            {
                /*cout << "+++++++Merge detected failed in KF" << endl;

                for(int i=0; i<mvpMergeLastKF.size(); ++i)
                {
                    mvpMergeLastKF[i]->SetErase();
                    mvpMergeCandidateKF[i]->SetErase();
                    mvpMergeLastKF[i]->mbCurrentPlaceRecognition = true;
                }

                mvpMergeCandidateKF.clear();
                mvpMergeLastKF.clear();
                mvg2oSim3MergeTcw.clear();
                mvnMergeNumMatches.clear();
                mvvpMergeMapPoints.clear();
                mvvpMergeMatchedMapPoints.clear();*/

                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
            }


        }
    }

    //! step 6上面两步细化匹配检测到闭环或者地图匹配那就返回true
    if(mbMergeDetected || mbLoopDetected)
    {
        //f_time_pr << "Geo" << " " << timeGeoKF_ms.count() << endl;
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }

    //-------------
    //TODO: This is only necessary if we use a minimun score for pick the best candidates
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    /*float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }*/
    //-------------

    // Extract candidates from the bag of words
    vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;
    //cout << "LC: bMergeDetectedInKF: " << bMergeDetectedInKF << "   bLoopDetectedInKF: " << bLoopDetectedInKF << endl;
    if(!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
        // Search in BoW
        //TODO 第一次进来该函数的时候先进入的这个函数，上面的那些都没有被执行（因为 mnLoopNumCoincidences = 0 ，应该需要完整执行完毕闭环一次～）
        //! step7  [PR_1] : DBoW2 candidate keyframes   分别检索三个最相似的关键帧放入闭环候选帧和匹配候选帧
        // vpLoopBowCand : 同地图 闭环候选帧
        // vpMergeBowCand：不同地图，匹配候选帧
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);
    }

    // Check the BoW candidates if the geometric candidate list is empty
    //Loop candidates
/*#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeStartGeoBoW = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeStartGeoBoW = std::chrono::monotonic_clock::now();
#endif*/

    //! step8 [PR_2-5] 同一地图中，第一次进来，使用局部窗口和精细匹配找到闭环相同区域
    if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {
        //mnLoopNumCoincidences  在当前帧共视图中验证  闭环匹配sim3  通过的关键帧数(0~3)（该关键帧是来至于当前帧的共视图）
        mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
    }
    // Merge candidates

    //cout << "LC: Find BoW candidates" << endl;
   //!step9  [PR_2-5]  不同地图中，第一次进来，使用局部窗口和精细匹配找到匹配相同区域
    if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
    }

    //cout << "LC: add to KFDB" << endl;
    //! step10 将当前帧加入关键帧数据库
    mpKeyFrameDB->add(mpCurrentKF);

    //! step11 如果上面匹配成功了返回true，否则返回false
    if(mbMergeDetected || mbLoopDetected)
    {
        return true;
    }

    //cout << "LC: erase KF" << endl;
    //都到这一步了，说明已经失败了，并没有找到闭环的地方
    mpCurrentKF->SetErase();    //?  
    mpCurrentKF->mbCurrentPlaceRecognition = false;

    return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
    cout << "REFFINE-SIM3: Projection from last KF with " << nNumProjMatches << " matches" << endl;


    int nProjMatches = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    /*if(mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO)
    {
        nProjMatches = 50;
        nProjOptMatches = 50;
        nProjMatchesRep = 80;
    }*/

    if(nNumProjMatches >= nProjMatches)
    {
        cv::Mat mScw = Converter::toCvMat(gScw);
        cv::Mat mTwm = pMatchedKF->GetPoseInverse();
        g2o::Sim3 gSwm(Converter::toMatrix3d(mTwm.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTwm.rowRange(0, 3).col(3)),1.0);
        g2o::Sim3 gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale = mbFixScale;       // TODO CHECK; Solo para el monocular inertial
        if(mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale=false;
        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);
        cout << "REFFINE-SIM3: Optimize Sim3 from last KF with " << numOptMatches << " inliers" << endl;



        if(numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(Converter::toMatrix3d(mScw.rowRange(0, 3).colRange(0, 3)),
                           Converter::toVector3d(mScw.rowRange(0, 3).col(3)),1.0);

            vector<MapPoint*> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
            //cout << "REFFINE-SIM3: Projection with optimize Sim3 from last KF with " << nNumProjMatches << " matches" << endl;
            if(nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}



/**Local window  :
 * 每Km我们定义一个局部窗口，其中包括Km，它的最佳共视关键帧，以及所有这些关键帧所观察到的地图点。
 * DBoW2直接索引提供了Ka中的关键点和本地窗口关键帧中的关键点之间的一组假定匹配。
 * 每一次的这些2D-2D匹配，我们也有相应地图点之间的3D-3D匹配。
 * 
 * 很多流程类似于：   bool LoopClosing::ComputeSim3()
*/
//! 此函数是关于论文中地图融合中的位置识别那一块  （VI-A）
bool LoopClosing::DetectCommonRegionsFromBoW(
            std::vector<KeyFrame*> &vpBowCand,          //[in]候选帧
            KeyFrame* &pMatchedKF2, 
            KeyFrame* &pLastCurrentKF, 
            g2o::Sim3 &g2oScw,                                              //[in] 世界坐标系到当前帧的sim3变换
            int &nNumCoincidences,                                    //[out]  在当前帧共视图中验证  闭环匹配sim3  通过的关键帧数(0~3)（该关键帧是来至于当前帧的共视图）
            std::vector<MapPoint*> &vpMPs,                    //局部窗口中的地图点
            std::vector<MapPoint*> &vpMatchedMPs   //局部窗口中与当前帧有匹配关系的地图点（未进行冗余处理）
            )
{

    //TODO 调参获取最优秀
    //在统计当前帧与候选帧的匹配关系的时候，设定的阈值（当前帧匹配上的地图点个数需要满足阈值）
    int nBoWMatches = 20;
    int nBoWInliers = 15;               //构造Sim3求解器时候使用
    int nSim3Inliers = 20;                 //g2o优化后得到的内点个数
    int nProjMatches = 50;           //投影匹配地图点阈值（没有g20优化之前的）
    int nProjOptMatches = 80;   //利用g2o优化后的sim3变换再匹配，这个时候肯定更多了，所以阈值也就相对更高
    /*if(mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO)
    {
        nBoWMatches = 20;
        nBoWInliers = 15;
        nSim3Inliers = 20;
        nProjMatches = 35;
        nProjOptMatches = 50;
    }*/

//!  一个约定
/**
 *    当前帧 ： mpCurrentKF
 *    当前帧的候选匹配帧：          【一级KF】           vpBowCand            (其中取一称为： pKFi)
 *   候选匹配帧的共视关键帧：  【二级KF】           vpCovKFi
 * 
*/


    //! 获取当前关键帧的共视图
    set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 5;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);
    int nNumGuidedMatching = 0;

    // Varibles to select the best numbe
    KeyFrame* pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<MapPoint*> vpBestMapPoints;
    std::vector<MapPoint*> vpBestMatchedMapPoints;

    //获取候选KF的数量
    int numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    //! [PR] step 2 : Local window 
    //! 遍历候选KF【一级KF】 
    for(KeyFrame* pKFi : vpBowCand)
    {
        //cout << endl << "-------------------------------" << endl;
        if(!pKFi || pKFi->isBad())
            continue;


        // Current KF against KF with covisibles version
        //! 取出与该候选帧共视程度最强的 nNumCovisibles 个关键帧（该候选帧pKFi的共视图关键帧vpCovKFi）  【二级KF】 
        std::vector<KeyFrame*> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovKFi.push_back(vpCovKFi[0]);
        vpCovKFi[0] = pKFi;

        // vvpMatchedMPs中每个vector都存储了当前帧上MapPoint的匹配对象，如果某个MapPoint=nullptr，则表示该候选帧没有与当前帧上该点匹配的对象
        std::vector<std::vector<MapPoint*> > vvpMatchedMPs;
        //存储了每个候选帧与当前帧的假定匹配（地图点）
        //答： 按照匹配对象进行，如果没有匹配上，对应的指针就为nullptr
        vvpMatchedMPs.resize(vpCovKFi.size());   
        std::set<MapPoint*> spMatchedMPi;
        int numBoWMatches = 0;

        //取出该候选帧
        KeyFrame* pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;


        // vpMatchedPoints 记录vpCovKFi  【二级KF】与当前帧上地图点匹配的对象，若索引对象=nullptr，说明是未匹配上的点。
        // vpMatchedPoints[k]表示与当前帧第k个地图点匹配的地图点
        std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
        // 找到vpCovKFi  【二级KF】与当前帧上地图点已经匹配上的地图点，并把它们插入到spMatchedMPi，
        //并在    vpKeyFrameMatchedMP    中记录匹配点对应的vpCovKFi  【二级KF】
        std::vector<KeyFrame*> vpKeyFrameMatchedMP = std::vector<KeyFrame*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));

        int nIndexMostBoWMatchesKF=0;  //记录匹配最多KF的下标
        //! 遍历    vpCovKFi  【二级KF】 ,获取匹配点和最大匹配KF
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;
            // 获取一组假定匹配(将当前帧mpCurrentKF与闭环候选关键帧匹配)
            //!  通过bow加速得到mpCurrentKF与    vpCovKFi  【二级KF】     之间的匹配特征点
            // vvpMatchedMPs 中每个vector都存储了当前帧的MapPoint的匹配对象，如果某个MapPoint=nullptr，则表示该候选帧没有与当前帧上该店匹配的对象
            int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            //cout << "BoW: " << num << " putative matches with KF " << vpCovKFi[j]->mnId << endl;
            //! 记录最大匹配值，并记录下此KF  在 vpCovKFi【二级KF】中的下标  nIndexMostBoWMatchesKF = j 
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        bool bAbortByNearKF = false;    //默认允许在临近找候选帧，后面会有禁止的
        //! 找到当前帧中与vpCovKFi  【二级KF】已经匹配上地图点，并把它们插入到spMatchedMPi，
        //!并在    vpKeyFrameMatchedMP    中记录与该地图点匹配对应的vpCovKFi  【二级KF】
        //遍历vpCovKFi  【二级KF】
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            //如果在当前帧的共视图中找到了该 候选帧，那就直接跳出     //?  这是直接找到的意思了吗？？
            if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;    //禁止候选KF临近当前帧
                //cout << "BoW: Candidate KF aborted by proximity" << endl;
                break;
            }

            //cout << "Matches: " << num << endl;
            //遍历有与当前帧有匹配关系的地图点
            for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPoint* pMPi_j = vvpMatchedMPs[j][k];
                if(!pMPi_j || pMPi_j->isBad())
                    continue;

                // 如果还没有该点，那就插入进来
                if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;  //记录与vpCovKFi  【二级KF】匹配上的地图点个数

                    vpMatchedPoints[k]= pMPi_j;   //记录与当前帧第k个地图点匹配的点
                    //! 下面这句话的意思是： 当前帧的第k个特征点与候选帧 vpCovKFi[j]  有匹配
                    //如果我没记错的话，特征点下标k是当前帧地图点的下标，如果候选帧率匹配的对应点为nullptr，说明当前帧的该点无匹配对象
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }    // 如果还没有该点，那就插入进来
            }   //遍历有与当前帧有匹配关系的地图点
        }   //遍历vpCovKFi  【二级KF】

        //cout <<"BoW: " << numBoWMatches << " independent putative matches" << endl;
        //!  如果： 禁止临近找候选关键帧，并且当前帧中与vpCovKFi  【二级KF】匹配上的地图点满足阈值要求（函数开头设定的20）
        if(!bAbortByNearKF && numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            /*cout << "-------------------------------" << endl;
            cout << "Geometric validation with " << numBoWMatches << endl;
            cout << "KFc: " << mpCurrentKF->mnId << "; KFm: " << pMostBoWMatchesKF->mnId << endl;*/
            // Geometric validation

            //是否需要固定尺度（似乎只有纯粹单目不需要固定尺度）
            bool bFixedScale = mbFixScale;
            if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale=false;


            //! [PR_3] step 3 : 3D aligning transformation
            /**论文： 译
             * 我们使用RANSAC计算变换Tam可以更好地将Km局部窗口中的映射点与Ka的映射点对齐.
             * 在单目或单目惯性情况下，当地图还不成熟时，计算{T}_{a m} ={sim}(3)，否则计算{T}_{a m} ={SE}(3)。
             * 在这两种情况下，我们使用Horn算法[80]使用三个3D-3D匹配的最小集合来找到Tam的每个假设。
             * 假设的匹配，通过Tam对Ka中的地图点进行变换后，在Ka中实现了低于阈值的重投影错误，给予假设积极的投票。
             * 拥有更多选票的假设被选中，前提是票数超过了一个阈值。
            */

            // 构造Sim3求解器
            /**
             * 参数的输入 ： 
             * mpCurrentKF  ： 当前帧
             * pMostBoWMatchesKF：本次遍历的候选帧【一级KF】
             * vpMatchedPoints ： vpCovKFi  【二级KF】与当前帧匹配的地图点
             * bFixedScale ： 是否固定尺度（单目=不固定尺度）  // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
             * vpKeyFrameMatchedMP ：      在    vpKeyFrameMatchedMP    中记录与该当前帧匹配地图点对应的vpCovKFi  【二级KF】
            */
            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            //?  下面怎么理解
            solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers，300次迭代

            bool bNoMore = false;         //没有更多
            vector<bool> vbInliers;     // 标记经过RANSAC sim3 求解后,vvpMapPointMatches中的哪些作为内点     //TODO  什么是内点？？
            int nInliers;
            bool bConverge = false;      //收敛
            cv::Mat mTcm;
            while(!bConverge && !bNoMore)
            {
                // 最多迭代20次，返回的mTcm是候选帧pKF到当前帧mpCurrentKF的Sim3变换（T12）
                //(用RANSAC方法求解SIM3，一共迭代五次，可以提高优化结果准确性)
                mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);            //TODO  未看
            }

            //cout << "Num inliers: " << nInliers << endl;

            //!====================================   下面全程懵逼    ========================================
            //!====================================   下面全程懵逼    ========================================
            //!====================================   下面全程懵逼    ========================================


            //TODO    这一个if是干嘛的？？应该是得到了一个比较理想的值了，所以在这个if下面开始下一步的处理了
            if(bConverge)
            {
                //cout <<"BoW: " << nInliers << " inliers in Sim3Solver" << endl;

                // Match by reprojection
                //int nNumCovisibles = 5;
                vpCovKFi.clear();
                //! 【PR_step 2】 Local window （重复执行的步骤2）
                //! 我们将每个需要进行位置识别的匹配候选对象称为Km,每Km我们定义一个局部窗口，
                //! 其中包括Km，它的最佳共视关键帧(vpCovKFi【二级KF】)，以及所有这些关键帧所观察到的地图点。
                // pMostBoWMatchesKF =  本次遍历的候选帧,也就是论文里面说的Km
                //获取自身以及自身最佳的 nNumCovisibles 个共视关键帧
                //![2.1] 获取局部窗口里面的关键帧集合 vpCovKFi
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);   // nNumCovisibles = 5
                int nInitialCov = vpCovKFi.size();
                vpCovKFi.push_back(pMostBoWMatchesKF);   //将候选帧KF自身也包含进来
                set<KeyFrame*> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                set<MapPoint*> spMapPoints;
                vector<MapPoint*> vpMapPoints;   //存储局部窗口里面的地图点，局部窗口包括闭环候选帧以及其共视关键帧
                vector<KeyFrame*> vpKeyFrames; //存储局部窗口里面的KF，局部窗口包括闭环候选帧以及其共视关键帧
                //! [2.1] 获取局部窗口里面的地图点（vpCovKFi【二级KF】所观察到的）
                for(KeyFrame* pCovKFi : vpCovKFi)
                {
                    for(MapPoint* pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if(!pCovMPij || pCovMPij->isBad())
                            continue;

                        if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }
                //cout << "Point cloud: " << vpMapPoints.size() << endl;

                //?  这些要做什么的？？ 要达到什么效果？？
                //? 匹配帧到当前帧
                g2o::Sim3 gScm(Converter::toMatrix3d(solver.GetEstimatedRotation()),Converter::toVector3d(solver.GetEstimatedTranslation()),solver.GetEstimatedScale());
                //? 世界坐标系到匹配候选帧？
                g2o::Sim3 gSmw(Converter::toMatrix3d(pMostBoWMatchesKF->GetRotation()),Converter::toVector3d(pMostBoWMatchesKF->GetTranslation()),1.0);
                //? 世界坐标系到当前帧？
                g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                cv::Mat mScw = Converter::toCvMat(gScw);


                //! 【PR_step 4】Guided matching reﬁnement（引导匹配细化）
                /**论文： 译
                 *   将局部窗口中的所有映射点用Tam进行变换，以找到更多与Ka中的关键点匹配的地图点。
                 * 搜索也是反向的，在本地窗口的所有关键帧中寻找关键帧Ka地图点的匹配。
                 * 利用所有找到的匹配，通过非线性优化优化Tam，其中目标函数为双向重投影误差，
                 * 利用Huber影响函数对伪匹配提供鲁棒性。如果优化后的inliers数量超过阈值，
                 * 则启动第二次迭代引导匹配和非线性细化，使用更小的图像搜索窗口。
                 */
                vector<MapPoint*> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                vector<KeyFrame*> vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
                // !  第一次匹配搜索：将闭环匹配上关键帧以及相连关键帧的vpPoints根据mScw 投影到当前关键帧进行投影匹配，搜索范围是 8
                int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
                cout <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.size() << " points with coarse Sim3" << endl;

                //第一次投影匹配（未优化）到的地图点需要满足阈值
                if(numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = mbFixScale;
                    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale=false;

                     // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
                     //!   优化mpCurrentKF与 pKFi 对应的MapPoints间的Sim3，得到优化后的量  gScm  （未看）
                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);
                    //cout <<"BoW: " << numOptMatches << " inliers in the Sim3 optimization" << endl;
                    //cout << "Inliers in Sim3 optimization: " << numOptMatches << endl;
                    //! g2o优化后得到的内点个数足够多(>=20)
                    if(numOptMatches >= nSim3Inliers)
                    {
                        //cout <<"BoW: " << numOptMatches << " inliers in Sim3 optimization" << endl;
                        // 得到从世界坐标系到该候选帧【一级KF】的Sim3变换，Scale=1
                        g2o::Sim3 gSmw(Converter::toMatrix3d(pMostBoWMatchesKF->GetRotation()),Converter::toVector3d(pMostBoWMatchesKF->GetTranslation()),1.0);
                        //!  得到g2o优化后  从世界坐标系到当前帧的Sim3变换
                        g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        cv::Mat mScw = Converter::toCvMat(gScw);

                        vector<MapPoint*> vpMatchedMP;    //局部窗口中与当前帧匹配的点
                        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                        //! 已经优化过一次了，利用优化过的sim3变换，再匹配一次
                        // !  将闭环匹配上关键帧以及相连关键帧的vpPoints根据mScw 投影到当前关键帧进行投影匹配，搜索范围是 5
                        int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);
                        //cout <<"BoW: " << numProjOptMatches << " matches after of the Sim3 optimization" << endl;
                        //! 第二次投影匹配（利用优化后的sim3）,应该满足更大的匹配点阈值
                        if(numProjOptMatches >= nProjOptMatches)
                        {
                            cout << "BoW: Current KF " << mpCurrentKF->mnId << "; candidate KF " << pKFi->mnId << endl;
                            cout << "BoW: There are " << numProjOptMatches << " matches between them with the optimized Sim3" << endl;
                            int max_x = -1, min_x = 1000000;
                            int max_y = -1, min_y = 1000000;
                            for(MapPoint* pMPi : vpMatchedMP)
                            {
                                if(!pMPi || pMPi->isBad())
                                {
                                    continue;
                                }

                                tuple<size_t,size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if(index >= 0)
                                {
                                    //范围约束
                                    int coord_x = pKFi->mvKeysUn[index].pt.x;
                                    if(coord_x < min_x)
                                    {
                                        min_x = coord_x;
                                    }
                                    if(coord_x > max_x)
                                    {
                                        max_x = coord_x;
                                    }
                                    int coord_y = pKFi->mvKeysUn[index].pt.y;
                                    if(coord_y < min_y)
                                    {
                                        min_y = coord_y;
                                    }
                                    if(coord_y > max_y)
                                    {
                                        max_y = coord_y;
                                    }
                                }
                            }   //遍历匹配的地图点
                            //cout << "BoW: Coord in X -> " << min_x << ", " << max_x << "; and Y -> " << min_y << ", " << max_y << endl;
                            //cout << "BoW: features area in X -> " << (max_x - min_x) << " and Y -> " << (max_y - min_y) << endl;


                            //![PR_5] : 5 Veriﬁcation in three covisible keyframes（在三个可共视关键帧中验证sim3变换）
                            /**
                             * 为了验证位置识别，我们在地图的活动部分中搜索两个与Ka共可视的关键帧
                             * （这两个关键帧与局部窗口中的点匹配的数量超过阈值）。如果没有找到关键帧，
                             * 则使用新的关键帧进一步尝试验证，而不需要再次启动bag-of-words。
                             * 验证一直持续，直到有三个关键帧验证Tam，或者连续两个关键帧验证失败。 
                             * 
                            */
                            int nNumKFs = 0;   // 统计验证通过的 当前帧共视帧 个数
                            //vpMatchedMPs = vpMatchedMP;
                            //vpMPs = vpMapPoints;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            //获取 nNumCovisibles 个当前关键帧的共视关键帧
                            vector<KeyFrame*> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                            //cout << "---" << endl;
                            //cout << "BoW: Geometrical validation" << endl;
                            int j = 0; 
                            while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                            //for(int j=0; j<vpCurrentCovKFs.size(); ++j)
                            {
                                //获取当前帧的共视图j
                                KeyFrame* pKFj = vpCurrentCovKFs[j];
                                //计算当前帧到共视帧j的变换
                                cv::Mat mTjc = pKFj->GetPose() * mpCurrentKF->GetPoseInverse();
                                //计算当前帧到共视帧j的sim3变换
                                g2o::Sim3 gSjc(Converter::toMatrix3d(mTjc.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTjc.rowRange(0, 3).col(3)),1.0);
                                //计算世界坐标系到当前帧的共视帧j的sim3变换
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPoint*> vpMatchedMPs_j;
                                //! 计算  当前帧的共视帧  与  候选闭环帧  的匹配，验证sim3是否有效（这两个关键帧与局部窗口中的点匹配的数量超过阈值）
                                bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                if(bValid)
                                {
                                    //cout << "BoW: KF " << pKFj->mnId << " has " << numProjMatches_j << " matches" << endl;
                                    cv::Mat Tc_w = mpCurrentKF->GetPose();
                                    cv::Mat Tw_cj = pKFj->GetPoseInverse();
                                    cv::Mat Tc_cj = Tc_w * Tw_cj;
                                    cv::Vec3d vector_dist =  Tc_cj.rowRange(0, 3).col(3);
                                    cv::Mat Rc_cj = Tc_cj.rowRange(0, 3).colRange(0, 3);
                                    double dist = cv::norm(vector_dist);
                                    cout << "BoW: KF " << pKFi->mnId << " to KF " << pKFj->mnId << " is separated by " << dist << " meters" << endl;
                                    cout << "BoW: Rotation between KF -> " << Rc_cj << endl;
                                    vector<float> v_euler = Converter::toEuler(Rc_cj);
                                    v_euler[0] *= 180 /3.1415;
                                    v_euler[1] *= 180 /3.1415;
                                    v_euler[2] *= 180 /3.1415;
                                    cout << "BoW: Rotation in angles (x, y, z) -> (" << v_euler[0] << ", " << v_euler[1] << ", " << v_euler[2] << ")" << endl;
                                    //! 统计验证通过的 当前帧共视帧 个数
                                    nNumKFs++;
                                    /*if(numProjMatches_j > numProjOptMatches)
                                    {
                                        pLastCurrentKF = pKFj;
                                        g2oScw = gSjw;
                                        vpMatchedMPs = vpMatchedMPs_j;
                                    }*/
                                }

                                j++;
                            }

                            //如果验证sim变换通过的的KF少于3个
                            if(nNumKFs < 3)
                            {
                                vnStage[index] = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            //记录最优匹配的效果
                            if(nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj = numProjOptMatches;
                                nBestNumCoindicendes = nNumKFs;       //[out]  在当前帧共视图中验证  闭环匹配sim3  通过的关键帧数(0~3)
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }


                        }   //第二次投影匹配（利用优化后的sim3）满足匹配阈值

                    }  // g2o优化后得到的内点个数足够多(>=20)

                }   //第一次投影匹配（未优化）到的地图点需要满足阈值
            }
        }
        index++;
    }

    //有最好的匹配，就更新各个状态量
    if(nBestMatchesReproj > 0)
    {
        pLastCurrentKF = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;   //[out]  在当前帧共视图中验证  闭环匹配sim3  通过的关键帧数(0~3)
        pMatchedKF2 = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
        g2oScw = g2oBestScw;
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        //在当前地图上（实际上就是当前帧的共视图）找到了三个验证通过的KF，表示这次匹配效果很成功
        return nNumCoincidences >= 3;
    }
    //没有一个验证通过的～～
    else
    {
        int maxStage = -1;
        int maxMatched;
        for(int i=0; i<vnStage.size(); ++i)
        {
            if(vnStage[i] > maxStage)
            {
                maxStage = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }
//        f_succes_pr << mpCurrentKF->mNameFile << " " << std::to_string(maxStage) << endl;
//        f_succes_pr << "% NumCand: " << std::to_string(numCandidates) << "; matches: " << std::to_string(maxMatched) << endl;
    }
    return false;
}


//参考论文：VI-A-(5)    Veriﬁcation in three covisible keyframes
//计算  当前帧的共视帧  与  候选闭环帧  的匹配，验证sim3是否有效（这两个关键帧与局部窗口中的点匹配的数量超过阈值）
bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
    //cout << "Projection from last KF with " << nNumProjMatches << " matches" << endl;

    int nProjMatches = 30;
    if(nNumProjMatches >= nProjMatches)
    {
        /*cout << "PR_From_LastKF: KF " << pCurrentKF->mnId << " has " << nNumProjMatches << " with KF " << pMatchedKF->mnId << endl;

        int max_x = -1, min_x = 1000000;
        int max_y = -1, min_y = 1000000;
        for(MapPoint* pMPi : vpMatchedMPs)
        {
            if(!pMPi || pMPi->isBad())
            {
                continue;
            }

            tuple<size_t,size_t> indexes = pMPi->GetIndexInKeyFrame(pMatchedKF);
            int index = get<0>(indexes);
            if(index >= 0)
            {
                int coord_x = pCurrentKF->mvKeysUn[index].pt.x;
                if(coord_x < min_x)
                {
                    min_x = coord_x;
                }
                if(coord_x > max_x)
                {
                    max_x = coord_x;
                }
                int coord_y = pCurrentKF->mvKeysUn[index].pt.y;
                if(coord_y < min_y)
                {
                    min_y = coord_y;
                }
                if(coord_y > max_y)
                {
                    max_y = coord_y;
                }
            }
        }*/
        //cout << "PR_From_LastKF: Coord in X -> " << min_x << ", " << max_x << "; and Y -> " << min_y << ", " << max_y << endl;
        //cout << "PR_From_LastKF: features area in X -> " << (max_x - min_x) << " and Y -> " << (max_y - min_y) << endl;


        return true;
    }

    return false;
}

int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                         set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                         vector<MapPoint*> &vpMatchedMapPoints)
{
    int nNumCovisibles = 5;
    vector<KeyFrame*> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame*> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame*> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    for(int i=0; i<nInitialCov; ++i)
    {
        vector<KeyFrame*> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
        int nInserted = 0;
        int j = 0;
        while(j < vpKFs.size() && nInserted < nNumCovisibles)
        {
            if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
            {
                spCheckKFs.insert(vpKFs[j]);
                ++nInserted;
            }
            ++j;
        }
        vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
    }
    set<MapPoint*> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for(KeyFrame* pKFi : vpCovKFm)
    {
        for(MapPoint* pMPij : pKFi->GetMapPointMatches())
        {
            if(!pMPij || pMPij->isBad())
                continue;

            if(spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }
    //cout << "Point cloud: " << vpMapPoints.size() << endl;

    cv::Mat mScw = Converter::toCvMat(g2oScw);

    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
    int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    return num_matches;
}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();
    mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue


    // If a Global Bundle Adjustment is running, abort it
    cout << "Request GBA abort" << endl;
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            cout << "GBA running... Abort!" << endl;
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    cout << "start updating connections" << endl;
    assert(mpCurrentKF->GetMap()->CheckEssentialGraph());
    mpCurrentKF->UpdateConnections();
    assert(mpCurrentKF->GetMap()->CheckEssentialGraph());

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oLoopScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    Map* pLoopMap = mpCurrentKF->GetMap();

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oLoopScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        cout << "LC: start correcting KeyFrames" << endl;
        cout << "LC: there are " << CorrectedSim3.size() << " KFs in the local window" << endl;
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();
            // cout << "scale for loop-closing: " << s << endl;

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Correct velocity according to orientation correction
            if(bImuInit)
            {
                Eigen::Matrix3d Rcor = eigR.transpose()*g2oSiw.rotation().toRotationMatrix();
                pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity());
            }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }
        // TODO Check this index increasement
        mpAtlas->GetCurrentMap()->IncreaseChangeIndex();
        cout << "LC: end correcting KeyFrames" << endl;


        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        cout << "LC: start replacing duplicated" << endl;
        for(size_t i=0; i<mvpLoopMatchedMPs.size(); i++)
        {
            if(mvpLoopMatchedMPs[i])
            {
                MapPoint* pLoopMP = mvpLoopMatchedMPs[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
        cout << "LC: end replacing duplicated" << endl;
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    //cout << "LC: start SearchAndFuse" << endl;
    SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);
    //cout << "LC: end SearchAndFuse" << endl;


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    //cout << "LC: start updating covisibility graph" << endl;
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }
    //cout << "LC: end updating covisibility graph" << endl;

    // Optimize graph
    //cout << "start opt essentialgraph" << endl;
    bool bFixedScale = mbFixScale;
    // TODO CHECK; Solo para el monocular inertial
    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
        bFixedScale=false;


    //cout << "Optimize essential graph" << endl;
    if(pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
    {
        //cout << "With 4DoF" << endl;
        Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
    }
    else
    {
        //cout << "With 7DoF" << endl;
        Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
    }


    //cout << "Optimize essential graph finished" << endl;
    //usleep(5*1000*1000);

    mpAtlas->InformNewBigChange();

    // Add loop edge
    mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
    if(!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1))
    {
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;

        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
}

//! vo系统的多地图匹配融合
void LoopClosing::MergeLocal()
{
    Verbose::PrintMess("MERGE-VISUAL: Merge Visual detected!!!!", Verbose::VERBOSITY_NORMAL);
    //mpTracker->SetStepByStep(true);

    int numTemporalKFs = 15; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFrame* pNewChild;
    KeyFrame* pNewParent;

    vector<KeyFrame*> vpLocalCurrentWindowKFs;
    vector<KeyFrame*> vpMergeConnectedKFs;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;   //  不发起  全局BA线程

    Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
    // If a Global Bundle Adjustment is running, abort it
    //!  1  如果全局BA在执行中，那就停掉它
    if(isRunningGBA())    //初始化为false，在CorrectLoop函数中置位 true
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        //!  1.1 如果存在全局线程，那就子线程脱离，让他自己运行结束后停止
        if(mpThreadGBA)
        {
            //子线程脱离，后台运行，不再受主线程管控。
            //mpThreadGBA将不会再和之前创建的全局BA线程绑定（但是子线程自己有检测停止的机制，会自己停止的）
            mpThreadGBA->detach();
            //删除管控句柄
            delete mpThreadGBA;
        }
        //!  1.2  打开  启用全局BA标志位 开关
        // enn，应该是为例多地图部分添加的～
        bRelaunchBA = true; 
    }

    Verbose::PrintMess("MERGE-VISUAL: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
    //! 2 地图拼接过程，禁止插入关键帧进入活跃地图（请求停止局部建图线程）
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    //! 3 等待局部建图线程的停止
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    Verbose::PrintMess("MERGE-VISUAL: Local Map stopped", Verbose::VERBOSITY_DEBUG);

    //! 4 将局部线程内的KF（来至于跟踪线程的插入）处理完毕（这里实际上相当于将部分局部线程关闭，暂时接替局部建图的KF处理任务，但是没有插入闭环线程这一步）
    //? 但是这里有一个问题，那就是全局优化的过程中还一直在插入当前地图KF，会不会导致一直不停的优化下去？？
    //根据论文里面的说法，是将当前地图合并到Mm地图
    mpLocalMapper->EmptyQueue();

    // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
    // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking

    /**译：
     * 匹配地图和他局部窗口里面的元素（共视帧及其共视帧上的地图点），会成为新的活跃地图，
     * 然后当前地图的元素会倒换到匹配地图中，以便于下一步的继续跟踪～
     * 补充：
     * 【1】：由于Ma可能包含许多元素，合并它们可能需要很长时间，合并被分为两个步骤。
     * 第一步：拼接过程发生在一个焊接窗口（ welding window）中（这个窗口由关键帧Ka和Km定义）
     * 第二步：通过位置图优化将校正传播到合并映射的其余部分。
    */
   //! 5 获取当前地图和局部地图句柄
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    Verbose::PrintMess("MERGE-VISUAL: Initially there are " + to_string(pCurrentMap->KeyFramesInMap()) + " KFs and " + to_string(pCurrentMap->MapPointsInMap()) + " MPs in the active map ", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE-VISUAL: Initially there are " + to_string(pMergeMap->KeyFramesInMap()) + " KFs and " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the matched map ", Verbose::VERBOSITY_DEBUG);
    //vector<KeyFrame*> vpMergeKFs = pMergeMap->GetAllKeyFrames();


    // Ensure current keyframe is updated
    //! 6 更新当前帧的连接关系
    mpCurrentKF->UpdateConnections();    // 更新关键帧之间的连接关系


    //!  7【STEP MM_1】 分别获取当前帧和匹配帧的局部窗口

    //! =======================   匹配帧的局部窗口  =============================
    //!  7【STEP MM_1】1.1  获取当前帧的局部窗口
    // Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
    // 用于获取当前帧以及他的相邻帧（VIO系统会多包涵几个 KF）
    set<KeyFrame*> spLocalWindowKFs;
    // Get MPs in the welding area from the current map
    // 用于得到焊接窗口中来至于当前地图的地图点
    set<MapPoint*> spLocalWindowMPs;

    // VIO  部分，先跳过
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        KeyFrame* pKFi = mpCurrentKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;

            set<MapPoint*> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }

        pKFi = mpCurrentKF->mNextKF;
        while(pKFi)
        {
            spLocalWindowKFs.insert(pKFi);

            set<MapPoint*> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }
    }
    //!  VO 系统
    else
    {
        //获取当前帧
        spLocalWindowKFs.insert(mpCurrentKF);
    }

    //获取当前帧的相邻帧（共视图）（共视图个数是函数开头的预设值 numTemporalKFs =  15  ）
    vector<KeyFrame*> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    Verbose::PrintMess("MERGE-VISUAL: Initial number of KFs in local window from active map: " + to_string(spLocalWindowKFs.size()), Verbose::VERBOSITY_DEBUG);
    
    const int nMaxTries = 3;
    int nNumTries = 0;
    //! 如果spLocalWindowKFs 获取的KF个数没有达到预设值  numTemporalKFs，并且迭代次数少于nMaxTries，那就继续迭代推入下一层共视帧
    while(spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFrame*> vpNewCovKFs;
        vpNewCovKFs.empty();
        //! 遍历当前帧的共视图（包括当前帧），以及共视图的共视图，将这些关键帧一同记录到spLocalWindowKFs中
        for(KeyFrame* pKFi : spLocalWindowKFs)
        {
            //  遍历  当前帧共视图的共视图， 并将局部窗口spLocalWindowKFs 中没有的添加进入vpNewCovKFs
            vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(KeyFrame* pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }
        //将  当前帧共视帧的共视帧（vpNewCovKFs），和当前帧及其共视帧（spLocalWindowKFs），一起合并到spLocalWindowKFs
        spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        //统计迭代次数
        nNumTries++;
    }
    Verbose::PrintMess("MERGE-VISUAL: Last number of KFs in local window from the active map: " + to_string(spLocalWindowKFs.size()), Verbose::VERBOSITY_DEBUG);

    //TODO TEST
    //vector<KeyFrame*> vpTestKFs = pCurrentMap->GetAllKeyFrames();
    //spLocalWindowKFs.insert(vpTestKFs.begin(), vpTestKFs.end());

    //遍历当前帧的局部窗口，将局部窗口中的地图点推入 spLocalWindowMPs
    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        set<MapPoint*> spMPs = pKFi->GetMapPoints();
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
    }
    Verbose::PrintMess("MERGE-VISUAL: Number of MPs in local window from active map: " + to_string(spLocalWindowMPs.size()), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE-VISUAL: Number of MPs in the active map: " + to_string(pCurrentMap->GetAllMapPoints().size()), Verbose::VERBOSITY_DEBUG);

    Verbose::PrintMess("-------", Verbose::VERBOSITY_DEBUG);

    //! =======================   匹配帧的局部窗口  =============================
    //!  【STEP MM_1】1.2  获取匹配帧的局部窗口
    set<KeyFrame*> spMergeConnectedKFs;
    //  vio 系统，跳过
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        KeyFrame* pKFi = mpMergeMatchedKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;
        }

        pKFi = mpMergeMatchedKF->mNextKF;
        while(pKFi)
        {
            spMergeConnectedKFs.insert(pKFi);
        }
    }
    // VO 系统
    else
    {
        //插入匹配帧
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
    }
    //插入匹配帧的局部地图
    vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    Verbose::PrintMess("MERGE-VISUAL: Initial number of KFs in the local window from matched map: " + to_string(spMergeConnectedKFs.size()), Verbose::VERBOSITY_DEBUG);
    
    nNumTries = 0;  //统计迭代次数
    //! 如果spMergeConnectedKFs 获取的KF个数没有达到预设值  numTemporalKFs，并且迭代次数少于nMaxTries，那就继续迭代推入下一层共视帧
    while(spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFrame*> vpNewCovKFs;
        for(KeyFrame* pKFi : spMergeConnectedKFs)
        {
            //  遍历  匹配帧共视图的共视图， 并将局部窗口 spMergeConnectedKFs 中没有的添加进入  vpKFiCov
            vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(KeyFrame* pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }
        //将  匹配帧共视帧的共视帧（vpNewCovKFs），和匹配帧及其共视帧（spMergeConnectedKFs），一起合并到spLocalWindowKFs
        spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }
    Verbose::PrintMess("MERGE-VISUAL: Last number of KFs in the localwindow from matched map: " + to_string(spMergeConnectedKFs.size()), Verbose::VERBOSITY_DEBUG);

    //遍历匹配帧的局部窗口，将局部窗口中的地图点推入 spLocalWindowMPs
    set<MapPoint*> spMapPointMerge;
    for(KeyFrame* pKFi : spMergeConnectedKFs)
    {
        set<MapPoint*> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
    }


    //! 8【STEP MM_2】1 Welding window assembly ： 进行两个地图Ma和Mm的对齐变换

    // 深度copy匹配帧局部窗口中的地图点到  vpCheckFuseMapPoint
    vector<MapPoint*> vpCheckFuseMapPoint;
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    //获取当前帧的变换 Twc
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat twc = Twc.rowRange(0,3).col(3);
    //获取g2o矫正之前的 sim3 变换  g2oNonCorrectedSwc 和  g2oNonCorrectedScw
    g2o::Sim3 g2oNonCorrectedSwc(Converter::toMatrix3d(Rwc),Converter::toVector3d(twc),1.0);
    g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
    //获取g2o矫正后的 sim3变换   g2oCorrectedScw (世界坐标系到当前帧的sim3变换)
    g2o::Sim3 g2oCorrectedScw = mg2oMergeScw; //TODO Check the transformation

    KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
    //记录该KFg2o优化前后的sim3变换
    vCorrectedSim3[mpCurrentKF]=g2oCorrectedScw;
    vNonCorrectedSim3[mpCurrentKF]=g2oNonCorrectedScw;


    //TODO Time test
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeStartTransfMerge = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeStartTransfMerge = std::chrono::monotonic_clock::now();
#endif
    //! 【STEP MM_2】 1.1 遍历当前帧的局部窗口
    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        //异常检测：该KF is bad
        if(!pKFi || pKFi->isBad())
        {
            Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
            continue;
        }

        //异常检测：该KF不属于当前帧，这不该发生（因为遍历的该KF就是当前地图下的当前帧的共视图）
        if(pKFi->GetMap() != pCurrentMap)
            Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

        g2o::Sim3 g2oCorrectedSiw;

       //! 【STEP MM_2】 1.1.1  非当前帧的话，取出g2o纠正前后，世界坐标系到该共视帧的sim3变换
        if(pKFi!=mpCurrentKF)
        {
            //取出该KF的坐标变换（世界坐标系到 该 共视帧）
            cv::Mat Tiw = pKFi->GetPose();
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            vNonCorrectedSim3[pKFi]=g2oSiw;

            //取出当前帧到该共视帧的变换
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2oCorrectedSiw = g2oSic*mg2oMergeScw;
            vCorrectedSim3[pKFi]=g2oCorrectedSiw;
        }
        //! 【STEP MM_2】 1.1.2  当前帧的话，只取出g2o纠正后的 世界坐标系到当前帧的变换
        else
        {
            g2oCorrectedSiw = g2oCorrectedScw;
        }
        //! 【STEP MM_2】1.1.3  自身向自身传递拼接的位姿 Tcw -> mTcwMerge（估计是怕merge地图前后位姿搞混乱了）
        pKFi->mTcwMerge  = pKFi->GetPose();

        //!   Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        //! 【STEP MM_2】1.1.4 更新该帧的sim3变换（世界坐标系到该KF，并且是已经经历过g2o优化过了）
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        //?  去尺度，归一化？？？
        pKFi->mfScale = s;
        eigt *=(1./s); //[R t/s;0 1]

        //cout << "R: " << mg2oMergeScw.rotation().toRotationMatrix() << endl;
        //cout << "angle: " << 180*LogSO3(mg2oMergeScw.rotation().toRotationMatrix())/3.14 << endl;
        //cout << "t: " << mg2oMergeScw.translation() << endl;

        //  sim3纠正矩阵
        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);
        //! 【STEP MM_2】1.1.5 更新该帧sim3(g2o优化过的sim3)纠正后的位姿
        pKFi->mTcwMerge = correctedTiw;

        //pKFi->SetPose(correctedTiw);

        // Make sure connections are updated
        //pKFi->UpdateMap(pMergeMap);
        //pMergeMap->AddKeyFrame(pKFi);
        //pCurrentMap->EraseKeyFrame(pKFi);

        //cout << "After -> Map current: " << pCurrentMap << "; New map: " << pKFi->GetMap() << endl;

        if(pCurrentMap->isImuInitialized())
        {
            Eigen::Matrix3d Rcor = eigR.transpose()*vNonCorrectedSim3[pKFi].rotation().toRotationMatrix();
            pKFi->mVwbMerge = Converter::toCvMat(Rcor)*pKFi->GetVelocity();
            //pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity()); // TODO: should add here scale s
        }

        //TODO DEBUG to know which are the KFs that had been moved to the other map
        //pKFi->mnOriginMapId = 5;
    }   // 遍历当前帧的局部窗口

    //! 【STEP MM_2】 1.2 遍历当前帧局部窗口中的地图点
    for(MapPoint* pMPi : spLocalWindowMPs)
    {
        if(!pMPi || pMPi->isBad())
            continue;

       //! 【STEP MM_2】 1.2.1 获取该地图点的参考帧
        KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
        //! 【STEP MM_2】 1.2.2 获取参考帧的sim3变换
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];


        // Project with non-corrected pose and project back with corrected pose
        //? 使用未矫正的pose投影过去，使用校正后的pose投影回来 ？？？想干嘛？，没看懂
        //答：这句话最好按照坐标系进行理解，先从当前的世界坐标系投影到相机坐标系，然后再投影回纠正后的世界坐标系～～
        //获取地图点
        //! 【STEP MM_2】 1.2.3  对该地图点的坐标进行两次投影（先从当前的世界坐标系投影到相机坐标系，然后再投影回纠正后的世界坐标系）
        cv::Mat P3Dw = pMPi->GetWorldPos();
        //转化格式
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        //  纠正后的世界坐标系  <<---- 相机坐标系  <<---世界坐标系
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(eigP3Dw));
        Eigen::Matrix3d eigR = g2oCorrectedSwi.rotation().toRotationMatrix();
        Eigen::Matrix3d Rcor = eigR * g2oNonCorrectedSiw.rotation().toRotationMatrix();

        //重新转化为cv::mat格式
        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);

        //! 【STEP MM_2】 1.2.4  更新纠正后的地图点，也不算更新，这里只是换做在mPosMerge中存储，原来的坐标还保留在 mWorldPos
        pMPi->mPosMerge = cvCorrectedP3Dw;
        //cout << "Rcor: " << Rcor << endl;
        //cout << "Normal: " << pMPi->GetNormal() << endl;
        pMPi->mNormalVectorMerge = Converter::toCvMat(Rcor) * pMPi->GetNormal();
        //pMPi->SetWorldPos(cvCorrectedP3Dw);
        //pMPi->UpdateMap(pMergeMap);
        //pMergeMap->AddMapPoint(pMPi);
        //pCurrentMap->EraseMapPoint(pMPi);
        //pMPi->UpdateNormalAndDepth();
    }
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeFinishTransfMerge = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeFinishTransfMerge = std::chrono::monotonic_clock::now();
#endif
    std::chrono::duration<double,std::milli> timeTransfMerge = timeFinishTransfMerge - timeStartTransfMerge; // Time in milliseconds
    Verbose::PrintMess("MERGE-VISUAL: TRANSF ms: " + to_string(timeTransfMerge.count()), Verbose::VERBOSITY_DEBUG);


    //TODO Time test
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeStartCritMerge = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeStartCritMerge = std::chrono::monotonic_clock::now();
#endif
    //!  9【STEP MM_2】2 Merging maps.： 地图Ma和Mm被融合在一起成为新的主动地图，并去除重复地图点
    /**二手论文翻译官：
     * 地图Ma和Mm被融合在一起成为新的主动地图。为了去除重复点，在Mm关键帧中主动搜索Ma点。
     * 对于每一个匹配，Ma中的点被删除，Mm中的点不断累积被删除点的所有观察值。
     * 由于 new mid-term point associations found（中期数据关联，这个应该指的是局部建图线程），
     * 共视图和基本图[2]通过添加 连接来自Mm和Ma的关键帧 的边来更新。
     * 
    */
    //! 【STEP MM_2】2.1  地图Ma和Mm被融合在一起成为新的主动地图.
    {
         //! 【STEP MM_2】2.1.1  锁住地图拼接的两个地图对象，避免两个地图更新
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

         //! 【STEP MM_2】2.1.2  遍历当前KF的局部窗口,更新拼接过程中的变量，以及地图指向改变为匹配地图
        for(KeyFrame* pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
            {
                //cout << "Bad KF in correction" << endl;
                continue;
            }

             // 更新拼接过程中的变量，以及地图指向改变为匹配地图
            pKFi->mTcwBefMerge = pKFi->GetPose();
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            pKFi->SetPose(pKFi->mTcwMerge);

            // Make sure connections are updated
            // 更新当前帧中的地图指向，全部指向匹配地图
            pKFi->UpdateMap(pMergeMap);
            pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
            //匹配地图添加该KF
            pMergeMap->AddKeyFrame(pKFi);
            //从当前地图中擦除自己的存在了
            pCurrentMap->EraseKeyFrame(pKFi);

            if(pCurrentMap->isImuInitialized())
            {
                pKFi->SetVelocity(pKFi->mVwbMerge);
            }
        }

        //! 【STEP MM_2】2.1.3   遍历当前局部窗口中的地图点，更新拼接过程中的变量,以及地图指向改变为匹配地图
        for(MapPoint* pMPi : spLocalWindowMPs)
        {
            if(!pMPi || pMPi->isBad())
                continue;

             // 更新拼接过程中的变量,以及地图指向改变为匹配地图
            pMPi->SetWorldPos(pMPi->mPosMerge);
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            pMPi->UpdateMap(pMergeMap);
            //匹配地图中添加该地图点
            pMergeMap->AddMapPoint(pMPi);
            //从当前地图中擦除自己的存在了
            pCurrentMap->EraseMapPoint(pMPi);
            //pMPi->UpdateNormalAndDepth();
        }

        mpAtlas->ChangeMap(pMergeMap);
        mpAtlas->SetMapBad(pCurrentMap);
        pMergeMap->IncreaseChangeIndex();
        //TODO for debug
        pMergeMap->ChangeId(pCurrentMap->GetId());
    }   //【STEP MM_2】2.1  地图Ma和Mm被融合在一起成为新的主动地图.

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeFinishCritMerge = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeFinishCritMerge = std::chrono::monotonic_clock::now();
#endif
    std::chrono::duration<double,std::milli> timeCritMerge = timeFinishCritMerge - timeStartCritMerge; // Time in milliseconds
    Verbose::PrintMess("MERGE-VISUAL: New current map: " + to_string(pMergeMap->GetId()), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE-VISUAL: CRITICAL ms: " + to_string(timeCritMerge.count()), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE-VISUAL: LOCAL MAPPING number of KFs: " + to_string(mpLocalMapper->KeyframesInQueue()), Verbose::VERBOSITY_DEBUG);


    //Rebuild the essential graph in the local window
    //! 【STEP MM_2】2.2   在本地窗口中重建局部窗口的基本图模型
    pCurrentMap->GetOriginKF()->SetFirstConnection(false);      //???  
    pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF

    // 旧的子元素，现在它将是它自己的父元素的父元素(我们需要从它的旧父元素的子元素列表中删除这个KF)
    pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpCurrentKF->ChangeParent(mpMergeMatchedKF);   //当前帧的父元素改变为当前帧的匹配帧
    while(pNewChild /*&& spLocalWindowKFs.find(pNewChild) != spLocalWindowKFs.end()*/)
    {
        //! 我们需要从它的旧父元素的子元素列表中删除这个KF（而这个新孩子元素就是原来的旧父元素）
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFrame * pOldParent = pNewChild->GetParent();

        pNewChild->ChangeParent(pNewParent);
        //cout << "The new parent of KF " << pNewChild->mnId << " was " << pNewChild->GetParent()->mnId << endl;

        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }

    //Update the connections between the local window
    //! 现在都在一个窗口和地图了，再更新一下匹配帧的共视关系吧
    //?  为什么不是当前帧呢？
    // 论文里面的意识是这样的，将当前地图融入匹配帧的地图，然后在某种程度上匹配帧地图就“被激活了”
    mpMergeMatchedKF->UpdateConnections();
    //cout << "MERGE-VISUAL: Essential graph rebuilded" << endl;


    //std::copy(spMapPointCurrent.begin(), spMapPointCurrent.end(), std::back_inserter(vpCheckFuseMapPoint));
    //获取匹配帧的共视图（包括自身）
    vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    vpMergeConnectedKFs.push_back(mpMergeMatchedKF);

    //将匹配帧的共视图深度copy进入 vpCheckFuseMapPoint   
    //?  深度copy上面已经进行过一次了，怎么还来？另外，使用vector不担心重复推入吗？
    //?  答： 难道是因为上面刚刚更新了共视关系（但是更新的只是KF变化呀～）
    //BUG  答： 别瞎想了，应该是代码bug，重复执行了两次
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));


    //TODO Time test
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeStartFuseMerge = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeStartFuseMerge = std::chrono::monotonic_clock::now();
#endif

    // Project MapPoints observed in the neighborhood of the merge keyframe
    // into the current keyframe and neighbors using corrected poses.
    /**
     * 二手翻译官：
     * 将那些在   匹配帧共视图中被观察到的地图点   投影到当前帧和共视帧（?当前帧的共视帧），投影过程使用纠正后的位姿
     * 
    */
    // Fuse duplications.   
    //! 【STEP MM_2】2.3  剔除合并后的冗余地图点（Ma中的点被删除，Mm中的点不断累积被删除点的所有观察值）
    //冗余点剔除,进行MapPoints检查与替换
    SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);        //!未看

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point timeFinishFuseMerge = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point timeFinishFuseMerge = std::chrono::monotonic_clock::now();
#endif
    std::chrono::duration<double,std::milli> timeFuseMerge = timeFinishFuseMerge - timeStartFuseMerge; // Time in milliseconds
    Verbose::PrintMess("MERGE-VISUAL: FUSE DUPLICATED ms: " + to_string(timeFuseMerge.count()), Verbose::VERBOSITY_DEBUG);

    //! 【STEP MM_2】2.4  Update connectivity
    Verbose::PrintMess("MERGE-VISUAL: Init to update connections in the welding area", Verbose::VERBOSITY_DEBUG);
    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(KeyFrame* pKFi : spMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    //CheckObservations(spLocalWindowKFs, spMergeConnectedKFs);

    Verbose::PrintMess("MERGE-VISUAL: Finish to update connections in the welding area", Verbose::VERBOSITY_DEBUG);

    //!  10【STEP MM_3】3  Welding bundle adjustment ：  焊接窗口BA优化
    /**二手论文翻译官：
     * 局部BA优化在焊接窗口(图3a)中的所有来自Ma和Mm的关键帧。
     * 为了固定尺度，与Mm中关键帧共视的关键帧是固定的。
     * 优化完成后，可将焊接区域所包含的所有关键帧用于摄像机跟踪，实现地图Mm的快速、准确重用。
     * 
    */
    bool bStop = false;
    Verbose::PrintMess("MERGE-VISUAL: Start local BA ", Verbose::VERBOSITY_DEBUG);
    vpLocalCurrentWindowKFs.clear();
    vpMergeConnectedKFs.clear();
    std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
    std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
    if (mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO)
    {
        Verbose::PrintMess("MERGE-VISUAL: Visual-Inertial", Verbose::VERBOSITY_DEBUG);
        Optimizer::MergeInertialBA(mpLocalMapper->GetCurrKF(),mpMergeMatchedKF,&bStop, mpCurrentKF->GetMap(),vCorrectedSim3);
    }
    else
    {
        Verbose::PrintMess("MERGE-VISUAL: Visual", Verbose::VERBOSITY_DEBUG);
        Verbose::PrintMess("MERGE-VISUAL: Local current window->" + to_string(vpLocalCurrentWindowKFs.size()) + "; Local merge window->" + to_string(vpMergeConnectedKFs.size()), Verbose::VERBOSITY_DEBUG);
        Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs,&bStop);
    }

    //!  Loop closed. Release Local Mapping.
    //!  11  闭环结束释放局部线程
    mpLocalMapper->Release();


    //return;
    Verbose::PrintMess("MERGE-VISUAL: Finish the LBA", Verbose::VERBOSITY_DEBUG);


    //Update the non critical area from the current map to the merged map
    vector<KeyFrame*> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
    vector<MapPoint*> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

    if(vpCurrentMapKFs.size() == 0)
    {
        Verbose::PrintMess("MERGE-VISUAL: There are not KFs outside of the welding area", Verbose::VERBOSITY_DEBUG);
    }
    else
    {
        Verbose::PrintMess("MERGE-VISUAL: Calculate the new position of the elements outside of the window", Verbose::VERBOSITY_DEBUG);
        //Apply the transformation
        {
            if(mpTracker->mSensor == System::MONOCULAR)
            {
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

                for(KeyFrame* pKFi : vpCurrentMapKFs)
                {
                    if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }

                    g2o::Sim3 g2oCorrectedSiw;

                    cv::Mat Tiw = pKFi->GetPose();
                    cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
                    cv::Mat tiw = Tiw.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
                    //Pose without correction
                    vNonCorrectedSim3[pKFi]=g2oSiw;

                    cv::Mat Tic = Tiw*Twc;
                    cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                    cv::Mat tic = Tic.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSim(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                    g2oCorrectedSiw = g2oSim*mg2oMergeScw;
                    vCorrectedSim3[pKFi]=g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                    Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                    double s = g2oCorrectedSiw.scale();

                    pKFi->mfScale = s;
                    eigt *=(1./s); //[R t/s;0 1]

                    cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

                    pKFi->mTcwBefMerge = pKFi->GetPose();
                    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                    pKFi->SetPose(correctedTiw);

                    if(pCurrentMap->isImuInitialized())
                    {
                        Eigen::Matrix3d Rcor = eigR.transpose()*vNonCorrectedSim3[pKFi].rotation().toRotationMatrix();
                        pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity()); // TODO: should add here scale s
                    }

                }
                for(MapPoint* pMPi : vpCurrentMapMPs)
                {
                    if(!pMPi || pMPi->isBad()|| pMPi->GetMap() != pCurrentMap)
                        continue;

                    KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
                    g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                    // Project with non-corrected pose and project back with corrected pose
                    cv::Mat P3Dw = pMPi->GetWorldPos();
                    Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                    Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(eigP3Dw));

                    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                    pMPi->SetWorldPos(cvCorrectedP3Dw);

                    pMPi->UpdateNormalAndDepth();
                }
            }
        }
        Verbose::PrintMess("MERGE-VISUAL: Apply transformation to all elements of the old map", Verbose::VERBOSITY_DEBUG);

        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }
        Verbose::PrintMess("MERGE-VISUAL: Local Map stopped", Verbose::VERBOSITY_DEBUG);

        //!  12 【STEP MM_4】4   Pose-graph optimization ：
        /**二手论文翻译官：
         * 利用融合图的基本图进行姿态图优化，使焊接区域的关键帧保持固定。
         * 这个优化将修正从焊接窗口传播到地图的其余部分。
         * 
        */

        // Optimize graph (and update the loop position for each element form the begining to the end)
        if(mpTracker->mSensor != System::MONOCULAR)
        {
            Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
        }


        {
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            Verbose::PrintMess("MERGE-VISUAL: There are " + to_string(pMergeMap->KeyFramesInMap()) + " KFs in the map", Verbose::VERBOSITY_DEBUG);
            Verbose::PrintMess("MERGE-VISUAL: It will be inserted " + to_string(vpCurrentMapKFs.size()) + " KFs in the map", Verbose::VERBOSITY_DEBUG);

            for(KeyFrame* pKFi : vpCurrentMapKFs)
            {
                if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }

                // Make sure connections are updated
                pKFi->UpdateMap(pMergeMap);
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);
            }
            Verbose::PrintMess("MERGE-VISUAL: There are " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the map", Verbose::VERBOSITY_DEBUG);
            Verbose::PrintMess("MERGE-VISUAL: It will be inserted " + to_string(vpCurrentMapMPs.size()) + " MPs in the map", Verbose::VERBOSITY_DEBUG);

            for(MapPoint* pMPi : vpCurrentMapMPs)
            {
                if(!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }
            Verbose::PrintMess("MERGE-VISUAL: There are " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the map", Verbose::VERBOSITY_DEBUG);
        }

        Verbose::PrintMess("MERGE-VISUAL: Optimaze the essential graph", Verbose::VERBOSITY_DEBUG);
    }



    mpLocalMapper->Release();


    Verbose::PrintMess("MERGE-VISUAL: Finally there are " + to_string(pMergeMap->KeyFramesInMap()) + "KFs and " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the complete map ", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE-VISUAL:Completed!!!!!", Verbose::VERBOSITY_DEBUG);

    //在视觉惯性情况下，只有在**关键帧数量低于阈值时才执行全局BA，以避免巨大的计算代价**。
    if(bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1)))
    {
        //!  13  发起一个新的线程执行全局BA优化(只在vio系统才会发起，并且需要KFs数量有限)
        Verbose::PrintMess("Relaunch Global BA", Verbose::VERBOSITY_DEBUG);
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this, pMergeMap, mpCurrentKF->mnId);
    }

    mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
    mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

    pCurrentMap->IncreaseChangeIndex();
    pMergeMap->IncreaseChangeIndex();

    mpAtlas->RemoveBadMaps();

}



void LoopClosing::printReprojectionError(set<KeyFrame*> &spLocalWindowKFs, KeyFrame* mpCurrentKF, string &name)
{
    string path_imgs = "./test_Reproj/";
    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        //cout << "KF " << pKFi->mnId << endl;
        cv::Mat img_i = cv::imread(pKFi->mNameFile, CV_LOAD_IMAGE_UNCHANGED);
        //cout << "Image -> " << img_i.cols << ", " << img_i.rows << endl;
        cv::cvtColor(img_i, img_i, CV_GRAY2BGR);
        //cout << "Change of color in the image " << endl;

        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        int num_points = 0;
        for(int j=0; j<vpMPs.size(); ++j)
        {
            MapPoint* pMPij = vpMPs[j];
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            cv::KeyPoint point_img = pKFi->mvKeysUn[j];
            cv::Point2f reproj_p;
            float u, v;
            bool bIsInImage = pKFi->ProjectPointUnDistort(pMPij, reproj_p, u, v);
            if(bIsInImage){
                //cout << "Reproj in the image" << endl;
                cv::circle(img_i, point_img.pt, 1/*point_img.octave*/, cv::Scalar(0, 255, 0));
                cv::line(img_i, point_img.pt, reproj_p, cv::Scalar(0, 0, 255));
                num_points++;
            }
            else
            {
                //cout << "Reproj out of the image" << endl;
                cv::circle(img_i, point_img.pt, point_img.octave, cv::Scalar(0, 0, 255));
            }

        }
        //cout << "Image painted" << endl;
        string filename_img = path_imgs +  "KF" + to_string(mpCurrentKF->mnId) + "_" + to_string(pKFi->mnId) +  name + "points" + to_string(num_points) + ".png";
        cv::imwrite(filename_img, img_i);
    }

}

//!  具备imu的系统进行调用
void LoopClosing::MergeLocal2()
{
    cout << "Merge detected!!!!" << endl;

    int numTemporalKFs = 11; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFrame* pNewChild;
    KeyFrame* pNewParent;

    vector<KeyFrame*> vpLocalCurrentWindowKFs;
    vector<KeyFrame*> vpMergeConnectedKFs;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    cout << "Check Full Bundle Adjustment" << endl;
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }


    cout << "Request Stop Local Mapping" << endl;
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    cout << "Local Map stopped" << endl;

    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    {
        float s_on = mSold_new.scale();
        cv::Mat R_on = Converter::toCvMat(mSold_new.rotation().toRotationMatrix());
        cv::Mat t_on = Converter::toCvMat(mSold_new.translation());

        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

        cout << "KFs before empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;
        mpLocalMapper->EmptyQueue();
        cout << "KFs after empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "updating active map to merge reference" << endl;
        cout << "curr merge KF id: " << mpCurrentKF->mnId << endl;
        cout << "curr tracking KF id: " << mpTracker->GetLastKeyFrame()->mnId << endl;
        bool bScaleVel=false;
        if(s_on!=1)
            bScaleVel=true;
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(R_on,s_on,bScaleVel,t_on);
        mpTracker->UpdateFrameIMU(s_on,mpCurrentKF->GetImuBias(),mpTracker->GetLastKeyFrame());

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    }

    const int numKFnew=pCurrentMap->KeyFramesInMap();

    if((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO)&& !pCurrentMap->GetIniertialBA2()){
        // Map is not completly initialized
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::InertialOptimization(pCurrentMap,bg,ba);
        IMU::Bias b (ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        mpTracker->UpdateFrameIMU(1.0f,b,mpTracker->GetLastKeyFrame());

        // Set map initialized
        pCurrentMap->SetIniertialBA2();
        pCurrentMap->SetIniertialBA1();
        pCurrentMap->SetImuInitialized();

    }


    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    // Load KFs and MPs from merge map
    cout << "updating current map" << endl;
    {
        // Get Merge Map Mutex (This section stops tracking!!)
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map


        vector<KeyFrame*> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
        vector<MapPoint*> vpMergeMapMPs = pMergeMap->GetAllMapPoints();


        for(KeyFrame* pKFi : vpMergeMapKFs)
        {
            if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->UpdateMap(pCurrentMap);
            pCurrentMap->AddKeyFrame(pKFi);
            pMergeMap->EraseKeyFrame(pKFi);
        }

        for(MapPoint* pMPi : vpMergeMapMPs)
        {
            if(!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue;

            pMPi->UpdateMap(pCurrentMap);
            pCurrentMap->AddMapPoint(pMPi);
            pMergeMap->EraseMapPoint(pMPi);
        }

        // Save non corrected poses (already merged maps)
        vector<KeyFrame*> vpKFs = pCurrentMap->GetAllKeyFrames();
        for(KeyFrame* pKFi : vpKFs)
        {
            cv::Mat Tiw=pKFi->GetPose();
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            NonCorrectedSim3[pKFi]=g2oSiw;
        }
    }

    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    cout << "end updating current map" << endl;

    // Critical zone
    bool good = pCurrentMap->CheckEssentialGraph();
    /*if(!good)
        cout << "BAD ESSENTIAL GRAPH!!" << endl;*/

    cout << "Update essential graph" << endl;
    // mpCurrentKF->UpdateConnections(); // to put at false mbFirstConnection
    pMergeMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);
    while(pNewChild)
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFrame * pOldParent = pNewChild->GetParent();
        pNewChild->ChangeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }


    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    cout << "end update essential graph" << endl;

    good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 1!!" << endl;

    cout << "Update relationship between KFs" << endl;
    vector<MapPoint*> vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
    vector<KeyFrame*> vpCurrentConnectedKFs;



    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vector<KeyFrame*> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mvpMergeConnectedKFs.size()>6)
        mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin()+6,mvpMergeConnectedKFs.end());
    /*mvpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);*/

    mpCurrentKF->UpdateConnections();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);
    /*vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);*/
    aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
    if (vpCurrentConnectedKFs.size()>6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin()+6,vpCurrentConnectedKFs.end());

    set<MapPoint*> spMapPointMerge;
    for(KeyFrame* pKFi : mvpMergeConnectedKFs)
    {
        set<MapPoint*> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
        if(spMapPointMerge.size()>1000)
            break;
    }

    cout << "vpCurrentConnectedKFs.size() " << vpCurrentConnectedKFs.size() << endl;
    cout << "mvpMergeConnectedKFs.size() " << mvpMergeConnectedKFs.size() << endl;
    cout << "spMapPointMerge.size() " << spMapPointMerge.size() << endl;


    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));
    cout << "Finished to update relationship between KFs" << endl;

    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 2!!" << endl;

    cout << "start SearchAndFuse" << endl;
    SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);
    cout << "end SearchAndFuse" << endl;

    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 3!!" << endl;

    cout << "Init to update connections" << endl;


    for(KeyFrame* pKFi : vpCurrentConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(KeyFrame* pKFi : mvpMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    cout << "end update connections" << endl;

    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 4!!" << endl;

    // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
    if (numKFnew<10){
        mpLocalMapper->Release();
        return;
    }

    good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 5!!" << endl;

    // Perform BA
    bool bStopFlag=false;
    KeyFrame* pCurrKF = mpTracker->GetLastKeyFrame();
    cout << "start MergeInertialBA" << endl;
    Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap,CorrectedSim3);
    cout << "end MergeInertialBA" << endl;

    good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 6!!" << endl;

    // Release Local Mapping.
    mpLocalMapper->Release();


    return;
}

void LoopClosing::CheckObservations(set<KeyFrame*> &spKFsMap1, set<KeyFrame*> &spKFsMap2)
{
    cout << "----------------------" << endl;
    for(KeyFrame* pKFi1 : spKFsMap1)
    {
        map<KeyFrame*, int> mMatchedMP;
        set<MapPoint*> spMPs = pKFi1->GetMapPoints();

        for(MapPoint* pMPij : spMPs)
        {
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            map<KeyFrame*, tuple<int,int>> mMPijObs = pMPij->GetObservations();
            for(KeyFrame* pKFi2 : spKFsMap2)
            {
                if(mMPijObs.find(pKFi2) != mMPijObs.end())
                {
                    if(mMatchedMP.find(pKFi2) != mMatchedMP.end())
                    {
                        mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                    }
                    else
                    {
                        mMatchedMP[pKFi2] = 1;
                    }
                }
            }

        }

        if(mMatchedMP.size() == 0)
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
        }
        else
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with " << mMatchedMP.size() << " KF from the other map" << endl;
            for(pair<KeyFrame*, int> matchedKF : mMatchedMP)
            {
                cout << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
            }
        }
    }
    cout << "----------------------" << endl;
}



    /**
     * Project MapPoints observed in the neighborhood of the merge keyframe into the current keyframe and neighbors using corrected poses.
     * 二手翻译官：
     * 将那些在   匹配帧共视图中被观察到的地图点   投影到当前帧和共视帧（?当前帧的共视帧），投影过程使用纠正后的位姿
     * 
    */

//! 冗余点剔除,进行MapPoints检查与替换
/**
 *  CorrectedPosesMap  :  <KF，该KF对应的sim3变换>   （sim3变换一般指世界坐标系到该KF） 
 *  vpMapPoints : 要检查的地图点，来至于 匹配帧的局部窗口（匹配帧和匹配帧的共视图）
*/
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    cout << "FUSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
    cout << "FUSE: Intially there are " << CorrectedPosesMap.size() << " KFs" << endl;
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        int num_replaces = 0;
        KeyFrame* pKFi = mit->first;
        Map* pMap = pKFi->GetMap();

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),static_cast<MapPoint*>(NULL));
        int numFused = matcher.Fuse(pKFi,cvScw,vpMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {


                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);

            }
        }

        total_replaces += num_replaces;
    }
    cout << "FUSE: " << total_replaces << " MPs had been fused" << endl;
}


void LoopClosing::SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
    cout << "FUSE-POSE: Intially there are " << vConectedKFs.size() << " KFs" << endl;
    for(auto mit=vConectedKFs.begin(), mend=vConectedKFs.end(); mit!=mend;mit++)
    {
        int num_replaces = 0;
        KeyFrame* pKF = (*mit);
        Map* pMap = pKF->GetMap();
        cv::Mat cvScw = pKF->GetPose();

        vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,vpMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
        cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << " MPs fused" << endl;
        total_replaces += num_replaces;
    }
    cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
}



void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::RequestResetActiveMap(Map *pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset = pMap;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;  //TODO old variable, it is not use in the new algorithm
        mbResetRequested=false;
        mbResetActiveMapRequested = false;
    }
    else if(mbResetActiveMapRequested)
    {

        for (list<KeyFrame*>::const_iterator it=mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
        {
            KeyFrame* pKFi = *it;
            if(pKFi->GetMap() == mpMapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        mLastLoopKFid=mpAtlas->GetLastInitKFid(); //TODO old variable, it is not use in the new algorithm
        mbResetActiveMapRequested=false;

    }
}

void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF)
{
    Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

    const bool bImuInit = pActiveMap->isImuInitialized();

    if(!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false);
    else
        Optimizer::FullInertialBA(pActiveMap,7,false,nLoopKF,&mbStopGBA);


    int idx =  mnFullBAIdx;
    // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!bImuInit && pActiveMap->isImuInitialized())
            return;

        if(!mbStopGBA)
        {
            Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
            // cout << "LC: Update Map Mutex adquired" << endl;

            //pActiveMap->PrintEssentialGraph();
            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),pActiveMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                //cout << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                //cout << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                cv::Mat Twc = pKF->GetPoseInverse();
                //cout << "Twc: " << Twc << endl;
                //cout << "GBA: Correct KeyFrames" << endl;
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(!pChild || pChild->isBad())
                        continue;

                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        //cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                        //cout << " child id: " << pChild->mnId << endl;
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        //cout << "Child pose: " << Tchildc << endl;
                        //cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;

                        cv::Mat Rcor = pChild->mTcwGBA.rowRange(0,3).colRange(0,3).t()*pChild->GetRotation();
                        if(!pChild->GetVelocity().empty()){
                            //cout << "Child velocity: " << pChild->GetVelocity() << endl;
                            pChild->mVwbGBA = Rcor*pChild->GetVelocity();
                        }
                        else
                            Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                        //cout << "Child bias: " << pChild->GetImuBias() << endl;
                        pChild->mBiasGBA = pChild->GetImuBias();


                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                //cout << "-------Update pose" << endl;
                pKF->mTcwBefGBA = pKF->GetPose();
                //cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                pKF->SetPose(pKF->mTcwGBA);
                /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
                cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
                double dist = cv::norm(trasl);
                cout << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" << endl;
                double desvX = 0;
                double desvY = 0;
                double desvZ = 0;
                if(pKF->mbHasHessian)
                {
                    cv::Mat hessianInv = pKF->mHessianPose.inv();

                    double covX = hessianInv.at<double>(3,3);
                    desvX = std::sqrt(covX);
                    double covY = hessianInv.at<double>(4,4);
                    desvY = std::sqrt(covY);
                    double covZ = hessianInv.at<double>(5,5);
                    desvZ = std::sqrt(covZ);
                    pKF->mbHasHessian = false;
                }
                if(dist > 1)
                {
                    cout << "--To much distance correction: It has " << pKF->GetConnectedKeyFrames().size() << " connected KFs" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected KF with 50 common matches or more" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" << endl;

                    cout << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ << endl;


                    string strNameFile = pKF->mNameFile;
                    cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);

                    cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);

                    vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
                    int num_MPs = 0;
                    for(int i=0; i<vpMapPointsKF.size(); ++i)
                    {
                        if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                        {
                            continue;
                        }
                        num_MPs += 1;
                        string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                        cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                        cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    }
                    cout << "--It has " << num_MPs << " MPs matched in the map" << endl;

                    string namefile = "./test_GBA/GBA_" + to_string(nLoopKF) + "_KF" + to_string(pKF->mnId) +"_D" + to_string(dist) +".png";
                    cv::imwrite(namefile, imLeft);
                }*/


                if(pKF->bImu)
                {
                    //cout << "-------Update inertial values" << endl;
                    pKF->mVwbBefGBA = pKF->GetVelocity();
                    if (pKF->mVwbGBA.empty())
                        Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                    assert(!pKF->mVwbGBA.empty());
                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);                    
                }

                lpKFtoCheck.pop_front();
            }

            //cout << "GBA: Correct MapPoints" << endl;
            // Correct MapPoints
            const vector<MapPoint*> vpMPs = pActiveMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    if(pRefKF->mTcwBefGBA.empty())
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            // TODO Check this update
            // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

            mpLocalMapper->Release();

            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    // cout << "LC: Finish requested" << endl;
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
