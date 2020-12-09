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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Initializer.h"
#include"G2oTypes.h"
#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include<chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>


using namespace std;

namespace ORB_SLAM3
{


Tracking::Tracking(
    System *pSys, 
    ORBVocabulary* pVoc, 
    FrameDrawer *pFrameDrawer, 
    MapDrawer *pMapDrawer, 
    Atlas *pAtlas, 
    KeyFrameDatabase* pKFDB, 
    const string &strSettingPath,           //EuRoC.yaml 等配置文件
    const int sensor, 
    const string &_nameSeq
    ):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

 //   cv::Mat DistCoef = cv::Mat::zeros(4,1,CV_32F);  //去畸变系统

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
    {
        std::cout << "*Error with the camera parameters in the config file*" << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb)
    {
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    initID = 0; lastID = 0;

    // Load IMU parameters
    bool b_parse_imu = true;
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if(!b_parse_imu)
        {
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        mnFramesToResetIMU = mMaxFrames;
    }

    mbInitWith3KFs = false;


    //Rectification parameters
    /*mbNeedRectify = false;
    if((mSensor == System::STEREO || mSensor == System::IMU_STEREO) && sCameraName == "PinHole")
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        fSettings["RIGHT.K"] >> K_r;
        fSettings["LEFT.P"] >> P_l;
        fSettings["RIGHT.P"] >> P_r;
        fSettings["LEFT.R"] >> R_l;
        fSettings["RIGHT.R"] >> R_r;
        fSettings["LEFT.D"] >> D_l;
        fSettings["RIGHT.D"] >> D_r;
        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty()
                || rows_l==0 || cols_l==0 || rows_r==0 || cols_r==0)
        {
            mbNeedRectify = false;
        }
        else
        {
            mbNeedRectify = true;
            // M1r y M2r son los outputs (igual para l)
            // M1r y M2r son las matrices relativas al mapeo correspondiente a la rectificación de mapa en el eje X e Y respectivamente
            //cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
            //cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        }
    }
    else
    {
        int cols = 752;
        int rows = 480;
        cv::Mat R_l = cv::Mat::eye(3, 3, CV_32F);
    }*/

    mnNumDataset = 0;

    if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        try
        {
            throw -1;
        }
        catch(exception &e)
        {

        }
    }

    //f_track_stats.open("tracking_stats"+ _nameSeq + ".txt");
    /*f_track_stats.open("tracking_stats.txt");
    f_track_stats << "# timestamp, Num KF local, Num MP local, time" << endl;
    f_track_stats << fixed ;*/

#ifdef SAVE_TIMES
    f_track_times.open("tracking_times.txt");
    f_track_times << "# ORB_Ext(ms), Stereo matching(ms), Preintegrate_IMU(ms), Pose pred(ms), LocalMap_track(ms), NewKF_dec(ms)" << endl;
    f_track_times << fixed ;
#endif
}

Tracking::~Tracking()
{
    //f_track_stats.close();
#ifdef SAVE_TIMES
    f_track_times.close();
#endif
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;
    string sCameraName = fSettings["Camera.type"];
    // //针孔相机模型
    // if(sCameraName == "PinHole"){
    //     float fx = fSettings["Camera.fx"];
    //     float fy = fSettings["Camera.fy"];
    //     float cx = fSettings["Camera.cx"];
    //     float cy = fSettings["Camera.cy"];

    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        //!  创建相机模型
        mpCamera = new Pinhole(vCamCalib);
        //!   atlas关联相机模型
        mpAtlas->AddCamera(mpCamera);

        // DistCoef.at<float>(0) = fSettings["Camera.k1"];
        // DistCoef.at<float>(1) = fSettings["Camera.k2"];
        // DistCoef.at<float>(2) = fSettings["Camera.p1"];
        // DistCoef.at<float>(3) = fSettings["Camera.p2"];


        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;


    }
    // // KannalaBrandt8 相机模型
    // if(sCameraName == "KannalaBrandt8"){
    //     float fx = fSettings["Camera.fx"];
    //     float fy = fSettings["Camera.fy"];
    //     float cx = fSettings["Camera.cx"];
    //     float cy = fSettings["Camera.cy"];

    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        // float K1 = fSettings["Camera.k1"];
        // float K2 = fSettings["Camera.k2"];
        // float K3 = fSettings["Camera.k3"];
        // float K4 = fSettings["Camera.k4"];

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


        // std::cout << "K1 = " << K1 << std::endl;
        // std::cout << "K2 = " << K2 << std::endl;
        // std::cout << "K3 = " << K3 << std::endl;
        // std::cout << "K4 = " << K4 << std::endl;

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // mpCamera = new KannalaBrandt8(vCamCalib);

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // mpAtlas->AddCamera(mpCamera);
        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if(!b_miss_params)
        {
            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);

            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

        // if(sensor==System::STEREO || sensor==System::IMU_STEREO){
        //     //Right camera
        //     fx = fSettings["Camera2.fx"];
        //     fy = fSettings["Camera2.fy"];
        //     cx = fSettings["Camera2.cx"];
        //     cy = fSettings["Camera2.cy"];

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            // K1 = fSettings["Camera2.k1"];
            // K2 = fSettings["Camera2.k2"];
            // K3 = fSettings["Camera2.k3"];
            // K4 = fSettings["Camera2.k4"];


            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }



            // cout << endl << "Camera2 Parameters: " << endl;
            // cout << "- fx: " << fx << endl;
            // cout << "- fy: " << fy << endl;
            // cout << "- cx: " << cx << endl;
            // cout << "- cy: " << cy << endl;

            // vector<float> vCamCalib2{fx,fy,cx,cy,K1,K2,K3,K4};

            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            // mpCamera2 = new KannalaBrandt8(vCamCalib2);
           int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }


            // mpAtlas->AddCamera(mpCamera2);

            node = fSettings["Tlr"];
            if(!node.empty())
            {
                mTlr = node.mat();
                if(mTlr.rows != 3 || mTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }



            if(!b_miss_params)
            {
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

            static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
            static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << mTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

        mpAtlas->AddCamera(mpCamera);
        mpAtlas->AddCamera(mpCamera2);
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    //获取相机帧率
    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(mSensor==System::RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }


    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }
    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}


bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat Tbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    cout << endl;

    cout << "Left camera to Imu Transform (Tbc): " << endl << Tbc << endl;

    float freq, Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        freq = node.operator int();
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }


    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();

    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(freq);
    cout << endl;
    cout << "IMU frequency: " << freq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;
    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}



cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    Track();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*cout << "trracking time: " << t_track << endl;
    f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;


    Track();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif

    return mCurrentFrame.mTcw.clone();
}


// 输入左目RGB或RGBA图像
// 1、将图像转为mImGray并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageMonocular(
    const cv::Mat &im,          //单目图像
    const double &timestamp,    //当前图像的时间戳
    string filename             // ""
    )
{
    mImGray = im;
    //! step 1 ：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    //!step2 ：构造Frame
    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
        //LNOTICE 这个使用的是初始化的时候的ORB特征点提取器，
        //初始化的mpIniORBextractor 比下面 mpORBextractorLeft 多提取一倍的特征点
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
        //NOTICE 当程序正常运行的时候使用的是正常的ORB特征点提取器
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    //?  IMU部分暂时跳过，慢慢来??
    /**
     * 印象中，首先是视觉部分完成初始化，大概是经历10帧左右，然后开始IMU部分的初始化
     * 
    */
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            //如果尚未完成初始化，特征点提取器使用初始化特征点提取器mpIniORBextractor（在单目情况下提取特征点提取是正常跟踪的5倍）
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            //初始化已经完成了，就按照正常的提取方式进行提取
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    //更新上一帧的ID
    lastID = mCurrentFrame.mnId;
    //!step3 跟踪
    Track();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif
    //!step4返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

// 获取两幅图像之间的imu数据
//  imuMeasurement  :  存储两幅图像之间的imu数据
void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}




/**
 * IMU预积分，在跟踪线程里面被函数Track()调用
 *  具体参考 Forster的论文
 * 
*/

void Tracking::PreintegrateIMU()
{
    //cout << "start preintegration" << endl;

    //! 1 判断是否存在上一帧，如果没有的话就退出该函数
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        //? 下面这一步是做什么的？？
        mCurrentFrame.setIntegrated();
        return;
    }

    // cout << "start loop. Total meas:" << mlQueueImuData.size() << endl;
    //清空缓存
    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    //! 2  如果两幅图像之间不存在imu数据，那么也退出该函数
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }
    //! 3 对两幅图像之间的imu序列进行挑选处理
    while(true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            //如果imu序列非空
            if(!mlQueueImuData.empty())
            {
                IMU::Point* m = &mlQueueImuData.front();
                //输出浮点数小数点后17位（存在四舍五入）      参考：https://blog.csdn.net/huangchijun11/article/details/72934222
                cout.precision(17);
                //防止异常处理，剔除掉时间上早于  该关键帧上一帧 的imu，值保留两幅图像之间的imu序列，这里设定的比较阈值是1ms
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-0.001l)
                {
                    mlQueueImuData.pop_front();
                }
                //! 3-1  将满足时间要求的两幅图像之间的imu序列转移到容器   mvImuFromLastFrame   中
                else if(m->t<mCurrentFrame.mTimeStamp-0.001l)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    //! 3-2 为了保险起见，在时间超出后额外吞掉一个，但是原有的容器并不弹出来
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            //! 3-3 如果imu序列空了，退出while循环
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }    //while预积分处理死循环


    //? 为什么会减1，直接获取总的imu序列不行吗?
    const int n = mvImuFromLastFrame.size()-1;

    //! 4 创建预积分对象，使用的是上一帧的imu bias和当前帧的协方差矩阵相关
    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

    //! 5 遍历两幅图像之间的imu序列,进行预积分，更新dR dP dV
    for(int i=0; i<n; i++)
    {
        float tstep;
        cv::Point3f acc, angVel;    //加速度、角速度
        /**
         * 下面使用：
         * 第一帧 表示：当前帧的前一帧
         * 第二帧 表示：当前帧
         * 
        */
       //! 5-1 计算imu之间的加速度、陀螺仪和时间间隔
       //? 看不懂，大意就是边缘imu处理
        if((i==0) && (i<(n-1)))
        {
            //获取两帧的时间差
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            //第1个imu与第一帧图像的时间差
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            //?  使用的是比值的方法计算的加速度,不像～  看不懂～
            //是不是这样，担心加速度不是恒等变量，又因为imu是高频率运行，所以短时间内假设imu是匀速均等改变的
            //那么只需要使用   【（第i+1个imu）- （第一帧图像）】/2
            // 等价于 【（第i+1个imu）-（第i个imu）】+【（第i个imu）-（第一帧图像）】 再除以2
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            //第i+1个imu与第一帧图像的时间差
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            //加速度，采用的是中值
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            //获取陀螺仪数值
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            //第i+1个imu时间与第i个imu时间的差值
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        //? 看不懂，大意就是边缘处理
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
        //如果两幅图像之间只有一个imu序列，不用算了，直接使用上一帧图像的数据。
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        //? 这里为什么使用两个   答：一个是针对关键帧的，一个是针对 图像帧的。关键帧是从图像帧中选取的。
        //! 注意理解概念： 先前帧就是时间上刚刚进行的帧率。上一帧就是该帧的上一帧图像。不是同一个概念。
        //! 5-2 传入加速度、陀螺仪角度、imu之间的时间间隔进行预积分
        //这一步预积分会计算出“预积分测量值”，但是没有减去“白噪声相关的数值”
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);   //传入变量： 加速度   陀螺仪  IMU之间的时间间隔
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }   // 遍历两幅图像之间的imu序列,进行预积分更新dR dP dV

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;
    //! 6 标记当前帧已经进行过了预积分
    mCurrentFrame.setIntegrated();

    Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}


/**
 * IMU预测
 * 注意：这个时候已经完成了预积分部分的处理，可以获取dR  dP  dV
 * 
*/
bool Tracking::PredictStateIMU()
{
    //! 1 如果当前帧没有上一帧，那也就自然没有预积分值，直接退出
    //当产生新的地图的时候，也很有可能会没有上一帧～
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

   //! 2 如果刚刚产生了新地图，就需要按照上一个的关键帧进行预积分预测
    if(mbMapUpdated && mpLastKeyFrame)
    {
        //下面都是在imu坐标系下进行的（IMU坐标系，参考cxq13页的公式，从i->j就知道了），进行完毕之后再更新到世界坐标系下
        //TODO 这里面 1：表示上一时刻的状态量。 2：表示当前时刻的状态量
        //获取世界坐标系下传感器IMU的位移的逆
        const cv::Mat twb1 = mpLastKeyFrame->GetImuPosition();  
        //获取上一时刻的旋转量的逆，b代表本体坐标系下的IMU，需要注意的是获取方式是利用刚性变换以及相机图像帧的旋转
        const cv::Mat Rwb1 = mpLastKeyFrame->GetImuRotation();   //! IMU坐标系
        //获取线速度的逆
        const cv::Mat Vwb1 = mpLastKeyFrame->GetVelocity();
        //创建3行1列的向量，赋值为   (  0,  0,  -9.81)
        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);    //  0  0  9.81
        //关键帧之间的预积分数值累积时间
        //需要注意了，这里的积分已经是 关键帧到当前帧的积分了，前面的积分执行流程已经完成了，并不是每来一个imu就积分一次，而是每来一个图像就积分一群imu
        const float t12 = mpImuPreintegratedFromLastKF->dT;   

        //! 2-1 累加增量（还是没有“白噪声相关部分”），更新P  V  Q
        //旋转量
        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        //位移量
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        //速度两
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        //! 2-2 将预测的姿态值更新到该图像帧类上
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        //更新本关键帧被imu预测的状态量
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        //bias暂时使用上一帧率的
        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    //! 3 如果本次没有产生新的地图，那么就正常处理，不再使用关键帧而是相连的连续帧进行预积分预测
    else if(!mbMapUpdated)
    {
        //不再注释，基本同上
        const cv::Mat twb1 = mLastFrame.GetImuPosition();
        const cv::Mat Rwb1 = mLastFrame.GetImuRotation();
        const cv::Mat Vwb1 = mLastFrame.mVw;
        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        //! 3-1 累加增量（还是没有“白噪声相关部分”），更新P  V  Q
        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        //! 3-2 将预测的姿态值更新到该图像帧类上
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        mCurrentFrame.mImuBias =mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}


void Tracking::ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz)
{
    const int N = vpFs.size();
    vector<float> vbx;
    vbx.reserve(N);
    vector<float> vby;
    vby.reserve(N);
    vector<float> vbz;
    vbz.reserve(N);

    cv::Mat H = cv::Mat::zeros(3,3,CV_32F);
    cv::Mat grad  = cv::Mat::zeros(3,1,CV_32F);
    for(int i=1;i<N;i++)
    {
        Frame* pF2 = vpFs[i];
        Frame* pF1 = vpFs[i-1];
        cv::Mat VisionR = pF1->GetImuRotation().t()*pF2->GetImuRotation();
        cv::Mat JRg = pF2->mpImuPreintegratedFrame->JRg;
        cv::Mat E = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaRotation().t()*VisionR;
        cv::Mat e = IMU::LogSO3(E);
        assert(fabs(pF2->mTimeStamp-pF1->mTimeStamp-pF2->mpImuPreintegratedFrame->dT)<0.01);

        cv::Mat J = -IMU::InverseRightJacobianSO3(e)*E.t()*JRg;
        grad += J.t()*e;
        H += J.t()*J;
    }

    cv::Mat bg = -H.inv(cv::DECOMP_SVD)*grad;
    bwx = bg.at<float>(0);
    bwy = bg.at<float>(1);
    bwz = bg.at<float>(2);

    for(int i=1;i<N;i++)
    {
        Frame* pF = vpFs[i];
        pF->mImuBias.bwx = bwx;
        pF->mImuBias.bwy = bwy;
        pF->mImuBias.bwz = bwz;
        pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        pF->mpImuPreintegratedFrame->Reintegrate();
    }
}

void Tracking::ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz)
{
    const int N = vpFs.size();
    const int nVar = 3*N +3; // 3 velocities/frame + acc bias
    const int nEqs = 6*(N-1);

    cv::Mat J(nEqs,nVar,CV_32F,cv::Scalar(0));
    cv::Mat e(nEqs,1,CV_32F,cv::Scalar(0));
    cv::Mat g = (cv::Mat_<float>(3,1)<<0,0,-IMU::GRAVITY_VALUE);

    for(int i=0;i<N-1;i++)
    {
        Frame* pF2 = vpFs[i+1];
        Frame* pF1 = vpFs[i];
        cv::Mat twb1 = pF1->GetImuPosition();
        cv::Mat twb2 = pF2->GetImuPosition();
        cv::Mat Rwb1 = pF1->GetImuRotation();
        cv::Mat dP12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaPosition();
        cv::Mat dV12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaVelocity();
        cv::Mat JP12 = pF2->mpImuPreintegratedFrame->JPa;
        cv::Mat JV12 = pF2->mpImuPreintegratedFrame->JVa;
        float t12 = pF2->mpImuPreintegratedFrame->dT;
        // Position p2=p1+v1*t+0.5*g*t^2+R1*dP12
        J.rowRange(6*i,6*i+3).colRange(3*i,3*i+3) += cv::Mat::eye(3,3,CV_32F)*t12;
        J.rowRange(6*i,6*i+3).colRange(3*N,3*N+3) += Rwb1*JP12;
        e.rowRange(6*i,6*i+3) = twb2-twb1-0.5f*g*t12*t12-Rwb1*dP12;
        // Velocity v2=v1+g*t+R1*dV12
        J.rowRange(6*i+3,6*i+6).colRange(3*i,3*i+3) += -cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*(i+1),3*(i+1)+3) += cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*N,3*N+3) -= Rwb1*JV12;
        e.rowRange(6*i+3,6*i+6) = g*t12+Rwb1*dV12;
    }

    cv::Mat H = J.t()*J;
    cv::Mat B = J.t()*e;
    cv::Mat x(nVar,1,CV_32F);
    cv::solve(H,B,x);

    bax = x.at<float>(3*N);
    bay = x.at<float>(3*N+1);
    baz = x.at<float>(3*N+2);

    for(int i=0;i<N;i++)
    {
        Frame* pF = vpFs[i];
        //?  这里似乎是唯一为mVw赋值的地方
        x.rowRange(3*i,3*i+3).copyTo(pF->mVw);
        if(i>0)
        {
            pF->mImuBias.bax = bax;
            pF->mImuBias.bay = bay;
            pF->mImuBias.baz = baz;
            pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        }
    }
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}


//!  图像跟踪
void Tracking::Track()
{
#ifdef SAVE_TIMES
    mTime_PreIntIMU = 0;
    mTime_PosePred = 0;
    mTime_LocalMapTrack = 0;
    mTime_NewKF_Dec = 0;
#endif

      //可视化相关 v3增加，打开后，在Pangolin上每次点击step一次，就处理一帧图像
    if (bStepByStep) 
    {
        while(!mbStep)  //在Pangolin上每次点击step一次, mbStep = true
            usleep(500);
        mbStep = false;
    }

    //?  这里什么时候置位？
    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }
    //! 1 获取当前地图
    Map* pCurrentMap = mpAtlas->GetCurrentMap();

    //! 2 时间戳不满足要求的异常处理
    //当前有图像存在的话判断时间是否混乱，如果有imu的话，就判断是否完成了BA，如果完成了BA那就产生新地图，否则那就直接复位当前地图就完事了
    if(mState!=NO_IMAGES_YET)
    {
        //! 2-1 两帧时间戳混乱,那就清空imu数据序列
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        //! 2-2 时间上相差至少1s，两帧图像时间戳相差太远，判定为不正常状态，进行异常处理
        //? 这里是不是说1s内处理的图像如果只有一张的话，说明当前环境不好进行跟踪，所以会产生新地图
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            //! 2-2-1 VIO：IMU完成了初始化以及当前地图完成了BA，那就产生新的地图，否则那就复位当前地图
            if(mpAtlas->isInertial())
            {
                //如果IMU完成了初始化
                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    // 如果该地图没有执行BA优化的话，那就直接复位吧，如果执行过BA那就产生新的地图
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        //地图复位
                        mpSystem->ResetActiveMap();
                    }
                    //当前地图完成了BA，那就说明地图比较好了，可以直接放心的产生新的地图
                    else
                    {
                        //产生新的地图
                        CreateMapInAtlas();
                    }
                }
                //初始化就没通过，那就直接复位吧 
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
            }
            return;
        }    //时间上相差1s以上
    }   //已经接收到了图像

    //! 3 VIO : 如果存在上一帧图像，那就直接初始化当前帧的IMU新偏差为上一帧的imu偏差
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    //! 4   从  无图像状态  过渡到 有图像但是没有完成初始化
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    //每一次状态的改变都要更新一下“上一次的状态”
    mLastProcessedState=mState;

    //! 5 VIO：进行IMU预积分
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
    {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        mTime_PreIntIMU = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
#endif
    }
    //!  6 这个时候状态稳定下来了，可以标记不用创建新地图
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    //! 7 获取地图更新锁，禁止地图更新改变
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    //用来标记是否已经产生过新的地图集合了
    mbMapUpdated = false;

    //! 8 判断当前地图是否已经属于新创建的地图，是的话更新“上一次使用的地图索引值”
    //获取本次地图索引
    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    //获取上一次的地图索引
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // cout << "Map update detected" << endl;
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;  //标记刚刚产生了新的地图集
    }

    //std::cout << "T2" << std::endl;

    //! 9 视觉初始化操作
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
            //双目初始化(RGBD相机也当做为双目相机来看待)
            StereoInitialization();
        else
        {
            //! 9-1 单目初始化
            MonocularInitialization();
        }
        //! 9-2 调用Pangolin帧绘制器来绘制一份当前帧的状态
        mpFrameDrawer->Update(this);

        //这个状态量在上面的初始化函数中被更新
        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        //! 9-3 当前为第一份地图的话，设定mnFirstFrameId索引值
        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }     // 视觉初始化操作
    //! 10 已经完成初始化了，会执行跟踪操作，跟踪估计相机位姿
    else 
    {
        // System is initialized. Track Frame.
        // bOK为临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
/**同时定位与建图模式（SLAM模式） 与之相对的选项是： 定位模式（Localization Mode = mbOnlyTracking 只跟踪不建图）
 * 同时定位与建图模式（SLAM模式）：SLAM完整模式，什么都有
 * 定位模式（Localization Mode）：只跟踪，不会插入关键帧，也自然不存在局部见图（局部建图需要关键帧）
 * 
 * 在viewer中有个开关Localization Mode，有它控制是否ActivateLocalizationMode，并最终管控mbOnlyTracking
 * 
 * mbOnlyTracking等于false表示正常VO模式（SLAM模式）（有地图更新），mbOnlyTracking等于true表示用户手动选择定位模式（只定位模式）
*/
        //! 10-1 跟踪模式（有建图）
        //下面的 else 是只跟踪不建图 的“定位模式”
        if(!mbOnlyTracking)  
        {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeStartPosePredict = std::chrono::steady_clock::now();
#endif

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            //!  正常跟踪模式
            //!  10-1-1  如果是正常跟踪的状态
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                //! 10-1-2  更新Fuse函数和SearchAndFuse函数替换的MapPoints（ // Fuse在局部建图里面，为了剔除冗余点）
                CheckReplacedInLastFrame();

                //! 10-1-3  跟踪上一帧或者参考帧或者重定位恢复
                // 运动模型是空的并且没有完成imu初始化    或      刚完成重定位
                //? 为什么这里要求没有完成初始化？？？
                // mVelocity 其实就是 Tcl  上一帧率到当前帧的变换
                //   mnLastRelocFrameId 上一次重定位的那一帧，只要 mVelocity 不为空，就优先选择 TrackWithMotionModel
                if((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //对于上述两个条件我的理解:
                    //第一个条件,如果运动模型为空,那么就意味着之前已经..跟丢了  
                    //第二个条件,就是如果当前帧紧紧地跟着在重定位的帧的后面,那么我们可以认为,通过在当前帧和发生重定位的帧(作为参考关键帧?)的基础上,
                    //我们可以恢复得到相机的位姿

                    //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                }   //从参考帧恢复
                else
                {
                    //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);

                    //!  根据恒速模型设定当前帧的初始位姿
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        // TrackReferenceKeyFrame是跟踪参考帧，不能根据固定运动速度模型预测当前帧的位姿态，通过bow加速匹配（SearchByBow）
                        // 最后通过优化得到优化后的位姿
                        //说白了就是根据恒速模型失败了.只能这样根据参考关键帧来了
                        //! 恒速模型跟踪失败了，就使用参考帧进行跟踪
                        //? 什么是参考帧
                        bOK = TrackReferenceKeyFrame();
                        /**wyp  拓展，新的关键帧的获取原则～～  根据ORB-SLAM_1 论文
                         * 1. 距离上一次全局重定位后超过20帧图像。
                         * 2. 局部地图构建处于空闲状态，或 距上一个关键帧插入后，已经有超过20帧图像。
                         * 3. 当前帧跟踪少于50个地图云点。
                         * 4. 当前帧跟踪少于参考关键帧K_ref云点的90%。
                         * 
                        */
                }   //从上一帧率和参考帧恢复

                //! 10-1-4 依旧跟踪失败的话，判断进入暂时丢失状态还是长期丢失状态
                //需要注意的是，这里面不需要的状态mState一旦改变，下一次就不会在进入来了（除非新地图的产生）
                if (!bOK)
                {
                    //! 刚刚初始化成功就跟踪失败，可以认为当前map并不重要，直接重新初始化
                    //?  如果初始化刚刚成功就丢失，可以认为当前map并不重要，直接重新初始化
                    //mnFramesToResetIMU  被赋值为 mMaxFrames（而mMaxFrames被赋值为图像的帧率fps）
                    //那么，当前帧率的id，还没有超过上一次重定位后的帧率值（一般重定位），是不是就可以认为刚刚初始化就失败了？？
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
                    {
                        mState = LOST;
                    }
                    //! 当前关键帧个数10个，进入短期丢失状态
                    //论文中提到，当前关键帧个数10个，并且丢失时间小于5秒，进入短期丢失状态
                    //?  这里怎么判断丢失时间小于5s的？？ 
                    //答 ：这里不需要判断，只需要在KF<10的时候标记状态就行了，记录下此时的时间，在下面的else 判断是否超过此时5s
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;  
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                        //mCurrentFrame.SetPose(mLastFrame.mTcw);
                    }
                    //! 剩余的就是跟踪丢失情况了
                    //或许还有未考虑的状态，但是只要上面已经考虑了KF较少的情况，以及短暂丢失的情况，
                    //只剩下的就是大地图的丢失了，直接标记LOST，开启新地图就行了
                    else
                    {
                        mState = LOST;
                    }
                }
            }  //正常跟踪模式（已经初始化成功） mState = OK
            //!  mState = OK 的下一个状态会进入这里（短期丢失状态RECENTLY_LOST   和长期丢失状态LOST）
            //! 非正常跟踪模式（短期丢失状态RECENTLY_LOST   和长期丢失状态LOST）
            else 
            {
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

                //! 10-1-5  短期丢失状态处理
                if (mState == RECENTLY_LOST)  
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    //! VIO：通过IMU恢复短期丢失
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                    {
                        //! 如果本地图IMU完成了初始化，那就使用IMU进行预测
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;
                        //! 短期丢失超过5s了，说明无法恢复了，直接标记为LOST进入下一状态
                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            //! 参考论文 ORBv3 V-D 
                            /**译文
                             * 短期丢失:当前的body状态是根据IMU读数估计的，地图点被投影到估计的相机姿态中，
                             * 并在一个大的图像窗口中搜索匹配。结果匹配包含在视觉惯性优化中。在大多数情况下，这允许恢复视觉跟踪。
                             * 否则，5秒后，我们进入下一阶段。
                             * 
                            */
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }   //imu恢复
                    //! VO：使用视觉重定位恢复
                    else
                    {
                        // TODO fix relocalization
                        bOK = Relocalization();
                        if(!bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }  //短期丢失状态
                //! 10-1-6  长期丢失状态，创建新的视觉惯性导航地图，并激活为 active map
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    /**
                     * 如果当前地图中的关键帧数量<10个，可以认为当前地图中没有重要信息，
                     * 直接ResetActiveMap(),这个思路相当于之前的Reset()， 即如果初始化刚刚成功就丢失，
                     * 可以认为当前map并不重要，直接重新初始化，这里也是一样的思路，
                     * 这个部分最终发挥作用是在函数Tracking::ResetActiveMap()中。
                     * 但如果当前的地图已经有很多关键帧(大于10帧)，则调用CreateNewMap() 暂存该地图，再新建一个空的地图。
                     * 
                    */
                   //!  如果本地图KF小于10个，说明地图不好直接复位抛弃。
                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        cout << "Reseting current map..." << endl;
                    }
                    //! 如果地图中KF大于10个，那就产生新地图
                    else
                        //  产生子地图
                        CreateMapInAtlas();

                    //!  清除mpLastKeyFrame指针指向
                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }  //长期丢失状态
            }   // mState = OK 的下一个状态会进入这里（短期丢失状态RECENTLY_LOST   和长期丢失状态LOST）


#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeEndPosePredict = std::chrono::steady_clock::now();

        mTime_PosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndPosePredict - timeStartPosePredict).count();
#endif

        }  //跟踪模式
        //! 10-2“定位模式”（只跟踪，不建图不创建关键帧）
        else
        {
            //  Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            //  定位模式:局部见图被停用(TODO在惯性模式下不可用)
            //! 如果跟丢了, 那么就只能进行重定位了
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            }
            //! 如果没有跟丢
            else
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变量
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常，
                // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                //! 10-2-1 如果匹配点充足，优先考虑恒速运动模型
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    // mbVO为0则表明此帧匹配了很多的3D map点，非常好
                    //若上一帧有速度，使用TrackWithMotionModel
                    //若上一帧没有速度，使用TrackReferenceKeyFrame

                    // 优先考虑恒速运动模型
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        //如果恒速模型不被满足,那么就只能够通过参考关键帧来定位了
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                //! 10-2-2 如果跟踪匹配点较少（匹配的点少于10个，快完蛋的节奏）
                else //mbVo =true  ，匹配的点少于10个，那么快完蛋了----
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    //译：
                    //在最后一帧，我们跟踪了主要的视觉里程计点
                    //我们计算两个相机位姿，一个来自运动模型（TrackWithMotionModel）一个做重定位。
                    //如果重定位成功我们就选择这个思路，否则我们继续使用 the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    //运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    //在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;
                    //运动模型得到的位姿
                    cv::Mat TcwMM;
                    //! 1 使用运动模型跟踪得到当前帧位姿
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    //! 2 使用重定位的方法来得到当前帧的位姿   
                    //匹配点太少了，既要跟踪，又要重定位
                    bOKReloc = Relocalization();
                    
                    //! 3 如果运动模型跟踪成功，而重定位失败的话
                    if(bOKMM && !bOKReloc)
                    {
                        // 这三行没啥用？
                        //? 这里为什么说没啥用呢?  答：具体的说应该是后面两行注释掉也没有什么影响
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        //如果内点中被匹配的点数目小于10个(mbVo=true)要跪了
                        //这里之所以还会被执行到是因为：
                        //运动模型跟踪成功的条件是 “内点足够”，mbVO=false的条件是，内点中被匹配的点数量足够
                        //! 3-1 如果内点中被匹配上的特征点较少（少于10个）,累加内点的被观测次数
                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                // 如果这个特征点存在，并且也不是外点的时候，累加被观测的次数
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            } //增加当前可视地图点的被观测次数
                        }  //如果内点中被匹配上的特征点较少（少于10个）
                    }  //如果重定位失败,但是跟踪成功了
                    //! 4 如果重定位成功了，那就直接代表整个跟踪过程正常进行着
                    else if(bOKReloc) // 只要重定位成功整个跟踪过程正常进行（定位与跟踪，更相信重定位）
                    {
                        mbVO = false; //false表示该帧匹配了很多的地图点,跟踪是正常的;如果少于10个则为true,表示快要完蛋了
                    }

                    //! 5 运动模型和重定位有一个成功，我们就认为图像跟踪成功了
                    bOK = bOKReloc || bOKMM;
                }  //上一次的结果告诉我们本帧要跪了
            }  //如果没有跟丢
        }  // 10-2“定位模式”（只跟踪，不建图不创建关键帧，局部建图不工作）

        //! 10-3  将最新的关键帧作为reference frame
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        /**
         * 在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
         * local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
         * 在步骤中主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
         * 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        */
       //! 10-4 正常跟踪模式（可建图），并且图像跟踪成功状态下，进行局部地图中跟踪
        if(!mbOnlyTracking)
        {
            //图像跟踪成功
            if(bOK)
            {
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_StartTrackLocalMap = std::chrono::steady_clock::now();
#endif
                //TODO  未看～～
                bOK = TrackLocalMap();
                //如果前面的两两帧匹配过程进行得不好,这里也就直接放弃了
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_EndTrackLocalMap = std::chrono::steady_clock::now();

                mTime_LocalMapTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndTrackLocalMap - time_StartTrackLocalMap).count();
#endif


            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        //! 10-5 定位模式（只图像跟踪，不建图）,不仅要求图像跟踪成功（内点多），还要求内点中被匹配的点足够多（mbVO=false），才能进行局部地图跟踪
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.

            // 跟踪成功 ，并且匹配的点比较多
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        //?  下面这几句话没整明白
        // mbVO值为true表示有很少的匹配点和地图中的MapPoint相匹配，我们无法检索本地地图因此此时不调用TrackLocalMap函数。
        // 一旦系统重新定位相机位置，我们将再次使用本地地图


        //! 10-6 更新跟踪状态
        //根据上面的操作来判断是否跟踪成功，bOK=true表示跟踪成功
        //! 10-6-1 上面图像跟踪成功（产生足够的内点），标记状态位正常跟踪状态
        if(bOK)
            mState = OK;
        //! 10-6-2  跟踪失败，并且失败之前的状态是“正常跟踪”,需要进一步判断属于短暂丢失还是长期丢失
        else if (mState == OK)
        {
            //! VIO:  跟踪失败，imu还没有完成初始化，判定为短期丢失状态
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState=RECENTLY_LOST;
            }
            //! VO : 直接标记长期丢失（也不再进行重定位恢复了～～）
            else
                mState=LOST; // visual to lost

            //当前帧ID相比较上一帧，已经过去了mMaxFrames帧（初始化的时候设置为帧率大小），那就记录当前帧时间为丢失帧时间
            if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            }
        }   //跟踪失败

        //! 10-7 如果最近发生了重定位，在进行IMU复位操作之前，保存最近产生的帧信息。
        //? 这一个if怎么进行理解？？？  答： 
        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        //? 本地图已经完成了IMU初始化，在即将达到IMU复位要求之前（设定帧ID作为IMU复位条件），保存关键帧（用来进行IMU复位的？？？，需要好好看看IMU复位的代码）
        //? 根据代码理解，只要进行了重定位，那么就在接下来的第 mnFramesToResetIMU 帧开始进行IMU复位
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            // 载入预积分值
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        //! 10-8 如果当前地图完成了IMU初始化，并且图像跟踪状态正常，并且位重定位后的第mnFramesToResetIMU帧，执行IMU复位操作
        if(pCurrentMap->isImuInitialized())
        {
            //如果图像跟踪正常（成功）
            if(bOK)
            {
                //如果达到了IMU复位的要求（重定位后的第mnFramesToResetIMU帧），进行IMU复位操作。
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                //? 距离重定位帧过去了30帧，更新mLastBias为当前帧的 mImuBias
                //图像帧率比较大，粗略计算了一下Euroc帧率为20帧左右，TUM30帧左右
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }


        //! 10-9 Pangolin可视化信息更新
        // Update drawer   可视化部分
        mpFrameDrawer->Update(this);
        if(!mCurrentFrame.mTcw.empty())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        //! 10-10 如果图像跟踪正常，并且之前状态位短期丢失状态
        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            //! 1 更新运动模型中的mVelocity
            if(!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty()) //只有上一帧计算的有变换矩阵T,才可以计算出来运动矩阵
            {
                // 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc; // 其实就是 Tcl  上一帧率到当前帧的变换
            }
            else
                //否则生成空矩阵
                mVelocity = cv::Mat();

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);  //相当于更新了地图绘制器

            // Clean VO matches
            // step ：清除UpdateLastFrame中为当前帧临时添加的MapPoints   
            /**为什么删除？这些地图点的出现是为了解决什么而出现的？？ 
             * 答： 不知道，不过
             * UpdateLastFrame();是在运动模型函数 TrackWithMotionModel 里面调用的～～（里面对单目没有新生成点，所以我暂时没看）
             * 在里面新生成的点，被观测书为0，这也是下面对这些点删除的判断策略
             * 
            */
           //! 2 清除UpdateLastFrame中为当前帧临时添加的MapPoints （是为了跟踪增加的，单目没有增加）
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    // 排除 UpdateLastFrame 函数中为了跟踪增加的MapPoints
                    //如果该MapPoint没有被其他帧观察到，则将该MapPoint设置为NULL,也就是去掉
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // step ：清除临时的MapPoints，这些MapPoints在 TrackWithMotionModel 的 UpdateLastFrame 函数里生成（仅双目和rgbd）
            // 步骤2.4中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            //? 为什么生成这些可以提高帧间跟踪效果?
            //? 为什么单目就不能够生成地图点呢
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint


            //不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露 
            mlpTemporalPoints.clear();

#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeStartNewKF = std::chrono::steady_clock::now();
#endif
            //! 3  判断是否需要关键帧
            bool bNeedKF = NeedNewKeyFrame();
#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeEndNewKF = std::chrono::steady_clock::now();

            mTime_NewKF_Dec = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndNewKF - timeStartNewKF).count();
#endif



            // Check if we need to insert a new keyframe
            //! 4 如果在跟踪状态正常或者vio系统的短期丢失状态下，并且上一步判断需要新的关键帧，那么就产生新的关键帧
            if(bNeedKF && (bOK|| (mState==RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            //! 5 删除那些在bundle adjustment中检测为outlier的3D map点
            //? 哪里来的BA??
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                //这里第一个条件还要执行判断是因为, 前面的操作中可能删除了其中的地图点
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }   //如果图像跟踪正常，并且之前状态位短期丢失状态

        // Reset if the camera get lost soon after initialization
        //! 10-11 当前状态为跟踪失败状态，没救了，只能重新Reset
        if(mState==LOST)
        {
            //! 1 如果地图中的关键帧数量过少(少于5个)的话甚至直接重新进行初始化了
            if(pCurrentMap->KeyFramesInMap()<=5)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            //! 2 如果跟踪丢失的情况下，imu还没有完成初始化，那imu就reset吧，放弃之前的工作。
            if ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO))
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            //! 3 创建新的地图
            CreateMapInAtlas();
        }

        //确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        // 保存上一帧的数据,当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
    }  //end   10  跟踪操作，跟踪估计相机位姿：初始化完成后会进入跟踪分支

    //! 10 跟踪状态为正常跟踪或者短期丢失的时候，存储帧位姿信息，以便以后检索完整的相机轨迹。
    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // 存储帧位姿信息，以便以后检索完整的相机轨迹。
        //跟踪成功，存储帧位姿信息
        if(!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);    //存储相对位姿
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            // 如果跟踪失败，则相对位姿使用上一次值
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }
}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        if (mSensor == System::IMU_STEREO)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (cv::norm(mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA)<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO)
        {
            cv::Mat Rwb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).colRange(0,3).clone();
            cv::Mat twb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).col(3).clone();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, cv::Mat::zeros(3,1,CV_32F));
        }
        else
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    cv::Mat x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}


void Tracking::MonocularInitialization()
{

    // 如果单目初始器还没有被创建（初始化器用来对紧挨着的下一帧进行特征点匹配的～～），则创建单目初始器 NOTICE 也只有单目会使用到这个
    if(!mpInitializer)
    {
        // Set Reference Frame
        // 单目初始帧的特征点数必须大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            // step 1：得到用于初始化的第一帧，初始化需要两帧
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            // mvbPrevMatched最大的情况就是所有特征点都被跟踪上，所以mCurrentFrame.mvKeysUn.size()是其上限
            //因为当前帧会变成"上一帧"",所以存储到了这个变量中
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)  // mvKeysUn 是去畸变之后的特征点
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }
            return;
        }
    }
    else   //（如果单目初始化器已经被创建）既然有了单目初始化器了，那么这就是第二帧了，那就开始匹配一个吧～～
    {
         // step 2：如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
        if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            //特征点数少于100，此时删除初始化器
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }

        // Find correspondences
        ORBmatcher matcher(
            0.9,   //最佳的和次佳评分的比值阈值
            true  //检查特征点的方向
            );
        /**
         * step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
         * mvbPrevMatched为前一帧的特征点的坐标，存储了mInitialFrame中哪些点将进行接下来的匹配
         * mvIniMatches用于存储mInitialFrame,mCurrentFrame之间匹配的特征点
        */
        int nmatches = matcher.SearchForInitialization(
            mInitialFrame,       //初始化时的参考帧和当前帧
            mCurrentFrame,  //在初始化参考帧中提取得到的特征点
            mvbPrevMatched, //保存匹配关系
            mvIniMatches,   //搜索窗口大小
            100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        //?  三角化对应关系
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Rcw,tcw,mvIniP3D,vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();

            // Just for video
            // bStepByStep = true;
        }
    }
}



void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    pKFcur->PrintPointDistribution();

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_NORMAL);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    cv::Mat deltaT = vKFs.back()->GetPose()*vKFs.front()->GetPoseInverse();
    mVelocity = cv::Mat();
    Eigen::Vector3d phi = LogSO3(Converter::toMatrix3d(deltaT.rowRange(0,3).colRange(0,3)));


    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    initID = pKFcur->mnId;
}


void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mbSetInit=false;

    mnInitialFrameId = mCurrentFrame.mnId+1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    //运动模型矩阵初始化
    mVelocity = cv::Mat();
    mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        if(mpInitializer)
            delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;

}

// 更新Fuse函数和SearchAndFuse函数替换的MapPoints
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/**
 * 利用参考关键帧进行恢复，代码里面将跟踪线程里面的参考关键帧赋值为最新产生的关键帧（注意了，这是跟踪线程的关键帧，不是图像帧的参考帧）
 * 
*/
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    //! 1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    //! 2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
    //! mCurrentFrame中匹配的特征点存储在 vpMapPointMatches 中，vpMapPointMatches中存储的是mpReferenceKF里面的特征点指针
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    //! 3:将上一帧的位姿态作为当前帧位姿的初始值
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);  // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

    //mCurrentFrame.PrintPointDistribution();

    //! 4:通过优化3D-2D的重投影误差来获得位姿（纯视觉优化）
    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    //! 5：剔除优化后的outlier匹配点（MapPoints）
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        //if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    //! 6: 有imu传感器的话，返回True表示跟踪成功，否则的话匹配点对要大于15个才能返回true
    //?  这里为什么要说，有IMU的话就可以返回true，大概是在强调这里可以使用IMU恢复，不用担心跟踪丢失的～
    // TODO check these conditions
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    // step 1：更新最近一帧的位姿
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    //lastframe 到 ref_keyframe的位姿
    cv::Mat Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    // 如果上一帧为关键帧，或者单目的情况，则退出  //TODO 那么，就这样没有单目的事情了 ？？？
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
        {
            break;
        }
    }
}

/** 
 * 步骤：
 * 1. 首先通过上一帧的位姿和速度来设置当前帧相机的位姿
 * 2. 通过PnP方法估计相机位姿，再将上一帧的地图点投影到当前固定大小范围的帧平面上，如果匹配点少，那么扩大两倍的采点范围
 * 3. 然后进行一次BA算法，优化相机的位姿
 * 4. 优化位姿之后，对当前帧的关键点和地图点，抛弃无用的杂点，剩下的点供下一次操作使用
*/
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    //! 1 对于单目，这里只是更新一下最新一帧关键帧的位姿
    UpdateLastFrame();

    //! 2 如果IMU已经完成了初始化，那就通过IMU预测dP dV dQ获取当前帧的位姿 
    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict ste with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    }
    //! 3 如果IMU没有完成初始化,那就通过运动模型推断当前帧的位姿
    else
    {
        //设定位姿优化的初始种子点
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    }

    //地图点初始化为 NULL，实际上感觉初始化为 nullptr更合适一点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    //这个阈值和下面的的跟踪有关,表示了匹配过程中的搜索半径
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    //! 4  对上一帧的MapPoints进行跟踪, 根据上一帧特征点对应的3D点投影到当前帧图像上，做到缩小特征点匹配范围的效果
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    //! 5  如果跟踪的点少，则扩大搜索半径再来一次
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    //! 6 如果这样还是不能够获得足够的跟踪点,那么就认为运动跟踪失败了.
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    //! 7 优化当前帧的位姿（纯视觉优化），并通过优化找出外点
    Optimizer::PoseOptimization(&mCurrentFrame);    //TODO 由于是存视觉优化的，所以没看太细致，等后期使用ceres改写

    // Discard outliers
    //! 8  优化位姿后剔除outlier的mvpMapPoints
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                 //累加成功匹配到的地图点数目
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        //如果在纯定位过程中本帧图像内点中被匹配的点数量少于10个,那么这里的 mbVO 标志就会置位
        mbVO = nmatchesMap<10;
        //如果内点数目多于20个说明运动模型跟踪成功
        return nmatches>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap>=10;
}


/**
 * @brief 对Local Map的MapPoints进行跟踪
 * 
 * 1. 更新局部地图，包括局部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return true if success
 * @see V-D track Local Map
 */
bool Tracking::TrackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    //! 1 更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
    UpdateLocalMap();
    //! 2 在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
    SearchLocalPoints();

    // TOO check outliers before PO
    //! 3在优化位姿之前先统计存在的地图点，以及其中的外点
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    //! 4 如果IMU还没有完成初始化或者纯视觉，对当前帧进行位姿优化
    if (!mpAtlas->isImuInitialized())
        // Optimize Pose
        // 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化，
        // step 3：更新局部所有MapPoints后对位姿再次优化
        Optimizer::PoseOptimization(&mCurrentFrame);
    //! 5 如果IMU完成了初始化，那就进行VO（刚完成重定位）或VIO优化
    else
    {
        //! 5.1 如果刚刚完成了重定位（重定位之后一定数量KFs后，IMU会进行初始化），在IMU初始化之前对当前帧进行位姿优化（纯视觉）
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        //! 5.2 如果最近没有进行重定位，如果刚刚产生了新地图就利用普通帧进行VIO优化，否则利用关键帧进行VIO优化
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            //如果当前帧是刚刚产生了新地图后的一帧，那就利用普通帧进行VIO优化
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                //TODO 2020-10-16 该看这一个了，这一个结束就进行局部建图线程了
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            //如果没有产生新地图，那就利用关键帧进行VIO优化
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }    //5 如果IMU完成了初始化

    //! 6 重新统计存在的地图点，以及内点数量
    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    //初始化内点匹配数量mnMatchesInliers为0
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    //! 7 更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
             // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1 -- NOTICE 严格来说这里应该不是被观测
            if(!mCurrentFrame.mvbOutlier[i])
            {
                //? 累计被找到的次数
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                //查看当前是否是在纯定位过程
                if(!mbOnlyTracking)
                {
                    // 该MapPoint被其它关键帧观测到过
                    //NOTICE 注意这里的"Obervation"和上面的"Found"并不是一个东西
                    //? 那么区别又在哪里呢?  Found表示被跟踪匹配上的次数（也就是被找到）
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    // 记录当前帧跟踪到的MapPoints，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)    //如果这个地图点是外点,并且当前相机输入还是双目的时候,就删除这个点;如果是其他类型的输入反而不管了...
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //! 8 决定是否跟踪成功
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    //! 9跟踪失败条件一：如果最近刚刚发生了重定位,并且跟踪的地图点数量小于50，我们认为跟踪失败
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    //! 10 跟踪成功条件一：如果当前跟踪状态是短暂丢失，并且匹配上地图点数量大于10就认为跟踪成功。
    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;

    //! 11 跟踪成功条件二：VIO：需要至少匹配上15个地图点
    if (mSensor == System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    //! 12 跟踪成功条件二：VO :至少匹配30个才认为匹配成功
    else
    {
        //如果是正常的状态话只要跟踪的地图点大于30个我们就认为成功了
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
}

/**
 * 判断是否需要关键帧
 * 
*/
bool Tracking::NeedNewKeyFrame()
{
    //! 1 单目VIO: IMU未完成初始化，但是当前帧的时间戳相比较上一个“关键帧”时间上至少过去了0.25s ，返回true
    if(((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    //! 2 纯定位模式，不需要产生关键帧，返回false
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    //! 3 局部建图线程由于闭环线程停止或者将要停止时，返回false
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    // Return false if IMU is initialazing
    if (mpLocalMapper->IsInitializing())
        return false;
    //获取当前帧的关键帧数量
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    //! 4 如果刚进行了重定位，那么需要经过一些图像帧数量（mMaxFrames个）的处理，才允许产生关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
        //获取被观测次数大于nMinObs的地图点数量
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    //! 5 查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    // 双目或者RGB-D
    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    //?  MapPoints中和地图关联的比例阈值
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //! 6 纯视觉条件： 很长时间没有插入关键帧
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //! 7 纯视觉条件： localMapper处于空闲状态,才有生成关键帧的基本条件（mMinFrames = 0）
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    //纯视觉非单目情况： 跟踪要跪的节奏，0.25是一个比较低的阈值
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // 阈值比c1c要高，与之前参考帧（最近的一个关键帧）重复度不是太高  
     //nRefMatches : 获取被观测次数大于nMinObs的地图点数量
     //! 8 纯视觉条件： 匹配的内点数量满足范围要求（这一个条件有点别扭）
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio ||    //较强观测次数的地图点中，被观测到的地图点的数目太少,少于给定的阈值
                                                bNeedToInsertClose)) && 
                                                mnMatchesInliers>15);           //匹配到的内点太少

    // Temporal condition for Inertial cases
    bool c3 = false;
    //! 9 VIO：条件1成立：如果存在上一次关键帧，并且当前帧距离上一个关键帧已经过去了0.5秒。
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    //! 10 VIO: 条件2成立：在VIO单目系统下，匹配的特征点在15~75之间或者跟踪状态位短期跟踪丢失
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    //! 11 综合判定是否返回true
    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        //局部建图线程空闲，返回true可以接收KF
        if(bLocalMappingIdle)
        {
            return true;
        }
        //局部建图线程繁忙状态，发送终断局部建图BA信号，如果为单目系统返回false
        else
        {
            mpLocalMapper->InterruptBA();
            //非单目相机
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            //单目相机
            else
                return false;
        }
    }
    else
        return false;
}

//产生新的关键帧
void Tracking::CreateNewKeyFrame()
{
    //! 1 局部建图线程正在初始化或者停止运行，退出
    if(mpLocalMapper->IsInitializing())
        return;
    if(!mpLocalMapper->SetNotStop(true))   //局部建图线程停止
        return;

    //! 2 将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    //! 3 如果当前地图完成了IMU初始化，在关键帧中进行标记，并使用当前帧imu bias初始化新产生的关键帧imu bias
    if(mpAtlas->isImuInitialized())
        pKF->bImu = true;
    pKF->SetNewBias(mCurrentFrame.mImuBias);
    //! 4 将新产生的关键帧设置为当前帧的参考帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    //! 5 如果上一关键帧存在，设置当前帧的上一KF和上一帧的下一个KF（主要位IMU预积分服务）
    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    //! VIO 6 计算上一个KF到当前新产生KF的预积分
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    //! IMU双目情况，跳过
    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D;

                    if(mCurrentFrame.Nleft == -1){
                        x3D = mCurrentFrame.UnprojectStereo(i);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++; // TODO check ???
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }

            Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);

        }
    }


    //! 7 将KF插入到局部建图线程
    mpLocalMapper->InsertKeyFrame(pKF);

    //! 8 启动局部建图线程
    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    //cout  << "end creating new KF" << endl;
}



/**
 * 在局部地图中地图点投影到当前帧图像上，然后进行匹配获取匹配点对。
 * 
 * 
 * 在投影之前需要进行视野判断（通过函数isInFrustum进行）：
 * V-D 1) 将MapPoint投影到当前帧, 并判断是否在图像内
 * V-D 3) 计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内（这里所说的尺度变化是指地图点到相机中心距离的一段范围，
 *             如果计算出的地图点到相机中心距离不在这个范围的话就认为这个点在当前帧相机位姿下不能够得到正确、有效、可靠的观测，就要跳过.  ）
 * V-D 2) 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回     //?  没看懂
 * V-D 4) 根据深度预测尺度（对应特征点在一层）
 * 
*/
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    //! 1：遍历当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1(被当前帧看到了)
                pMP->IncreaseVisible();
                 // 标记该点被上一次被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                 // 标记该点将来不被投影，因为已经匹配过(指的是使用恒速运动模型进行投影)
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }
    //局部地图中地图点准备向当前帧进行投影匹配的点的数目
    int nToMatch=0;

    // Project points in frame and check its visibility
    //! 2：将所有 局部MapPoints 投影到当前帧，判断是否在视野范围内（只有在视野范围内的，才能进行投影匹配）
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        // 已经被当前帧观测到MapPoint不再判断是否能被当前帧观测到
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        //局部地图中的坏点也是
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        //! 2.1：判断LocalMapPoints中的点是否在在视野内（从深度>0，在边界范围内等考虑）
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数加1，该MapPoint在某些帧的视野范围内
            pMP->IncreaseVisible();
            // 只有在视野范围内的MapPoints才参与之后的投影匹配
            nToMatch++;
        }
        //! 2.2 将满足投影匹配要求的点，记录下来投影后像素坐标系上的坐标，交给可视化显示（mmProjectPoints在FrameDrawer.cc文件里面被调用）
        // mTrackProjX  是在上面isInFrustum里面计算出来的投影到当前帧像素坐标上的点
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    //! 3：如果的确存在需要进行投影匹配的点，那就进行投影匹配
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)   //RGBD相机输入的时候,搜索的阈值会变得稍微大一些
            th=3;
        //! 3.1 如果地图完成IMU初始化，若完成了VIO局部优化匹配阈值设为2，否则设为3
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=3;
        }
        //! 3.2 如果地图没有完成IMU初始化，匹配阈值设为10
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
         //! 3.3 如果不久前进行过重定位，匹配阈值设为5（对于VO系统相当于进行一个更加宽泛的搜索，对于VIO系统来说相当于刚开始阈值设一个居中值）
         //? 这里对VIO系统是不是不太恰当，毕竟阈值变小了
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        //! 3.4 如果跟踪状态为短期或者长期跟踪丢失状态，匹配阈值要大一些，设为15
        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        //! 3.5：对视野范围内的MapPoints通过投影进行特征点匹配
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

//更新局部地图
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    //! VO : 1  将局部地图里面的 mvpLocalMapPoints 设置当前地图参考地图点
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    //! 2 更新局部地图关键帧 mvpLocalKeyFrames
    UpdateLocalKeyFrames();  
    //! 3 根据局部地图关键帧容器 mvpLocalKeyFrames 更新局部地图中地图点  mvpLocalMapPoints
    UpdateLocalPoints();
}

// 更新局部地图中地图点  mvpLocalMapPoints
/**
 * 对局部地图关键帧容器mvpLocalKeyFrames中的关键帧进行遍历，将其地图点全局添加到局部地图容器mvpLocalMapPoints中
*/
void Tracking::UpdateLocalPoints()
{
    //清空局部地图地图点 mvpLocalMapPoints
    mvpLocalMapPoints.clear();

    int count_pts = 0;
    //遍历局部地图关键帧 mvpLocalKeyFrames，将里面的地图点添加到mvpLocalMapPoints
    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

//更新局部中的关键帧（大于80个）
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
     //keyframeCounter：统计能够观察到当前帧上地图点的所有关键帧中，每个关键帧观察到当前帧上地图点的个数，也就是投票
    map<KeyFrame*,int> keyframeCounter;     
    //! 1 IMU还没有完成初始化，或者刚刚完成了重定位，使用当前帧的共视关键帧进行局部地图更新
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        //遍历当前帧的MapPoints
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            //如果该地图点存在（为了保证一些映射关系以及地图点的更新删除，会存在不存在的地图点占位）
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    //关于tuple :  http://www.cplusplus.com/reference/tuple/tuple/?kw=tuple
                    //获取观察到该特征点的所有关键帧
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        //多次循环下来，统计该关键帧观察到的 当前帧上地图点的个数
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    //! 2 IMU已经完成了初始化，或者已经距离重定位过去较长时间了，就用上一帧的共视关键帧进行局部地图更新（因为当前帧还没有完成匹配）
    //? 为啥ORB_SLAM2就可以对当前帧进行呀？？
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }


    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    //! 3 开始清空局部关键帧容器mvpLocalKeyFrames，并重设大小为 观察到当前（上一帧）地图点的所有关键帧数量 的3倍
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    //! 4 将所有与当前帧有共视关系的图像帧放入局部地图容器mvpLocalKeyFrames，并找到与当前帧共视程度最强的图像帧pKFmax。
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        // 找到与当前帧共视程度最强的图像帧pKFmax。
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        //设置该帧参与了当前帧的局部地图跟踪
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    //! 5 如果局部地图里面存储的图像帧小于80个，那就继续扩充mvpLocalKeyFrames，扩充的对象就是已经在局部地图中图像帧的相邻帧
    //! 5-0 遍历已经放入到局部地图mvpLocalKeyFrames中的关键帧pKF
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        //! 5-1 如果局部地图mvpLocalKeyFrames里面存储的图像帧大于80个，退出循环
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        //! 5-2 获取pKF共视程度最好的10个关键帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        //! 5-3 添加 pKF 共视程度最好的10个关键帧
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }
        //! 5-4 添加 pKF 的子关键帧
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        //! 5-5 添加 pKF 的父关键帧
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }  // 遍历已经放入到局部地图mvpLocalKeyFrames中的关键帧pKF

    // Add 10 last temporal KFs (mainly for IMU)
    //! VIO 5-6： 为 pKF 循环向前添加 20个关键帧（根据关键帧找上一个KF，根据上一个找上上一个）
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&mvpLocalKeyFrames.size()<80)
    {
        //cout << "CurrentKF: " << mCurrentFrame.mnId << endl;
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            //cout << "tempKF: " << tempKeyFrame << endl;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    //! 6 设置当前帧的参考帧为共视程度最强的关键帧（如果前面找到的话）
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}


//定位模式调用，用于激活定位模式（只定位，提取特征点，但是不创建关键帧）
void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).rowRange(0,3).col(3)=(*lit).rowRange(0,3).col(3)*s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    cv::Mat Gz = (cv::Mat_<float>(3,1) << 0, 0, -IMU::GRAVITY_VALUE);

    cv::Mat twb1;
    cv::Mat Rwb1;
    cv::Mat Vwb1;
    float t12;

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}


cv::Mat Tracking::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = Converter::tocvSkewMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}


void Tracking::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpLastKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpLastKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpLastKeyFrame->GetCameraCenter();

    const float &fx1 = mpLastKeyFrame->fx;
    const float &fy1 = mpLastKeyFrame->fy;
    const float &cx1 = mpLastKeyFrame->cx;
    const float &cy1 = mpLastKeyFrame->cy;
    const float &invfx1 = mpLastKeyFrame->invfx;
    const float &invfy1 = mpLastKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpLastKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpKFs[i];
        if(pKF2==mpLastKeyFrame)
            continue;

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if((mSensor!=System::MONOCULAR)||(mSensor!=System::IMU_MONOCULAR))
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpLastKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpLastKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpLastKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpLastKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpLastKeyFrame->mb/2,mpLastKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpLastKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpLastKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpLastKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpLastKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpLastKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpLastKeyFrame,mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpLastKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpLastKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            nnew++;
        }
    }
    TrackReferenceKeyFrame();
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

} //namespace ORB_SLAM
