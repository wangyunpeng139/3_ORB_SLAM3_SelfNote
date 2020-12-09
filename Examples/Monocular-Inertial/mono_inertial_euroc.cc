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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;      //累加图像跟踪时间

/**
 * ./Examples/Monocular-Inertial/mono_inertial_euroc 
 * ./Vocabulary/ORBvoc.txt 
 * ./Examples/Monocular-Inertial/EuRoC.yaml 
 * "$pathDatasetEuroc"/MH03 
 * ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt 
 * dataset-MH03_monoi
 * 
*/

/**  多段数据集输入
 * ./Examples/Monocular/mono_euroc 
 * ./Vocabulary/ORBvoc.txt 
 * ./Examples/Monocular/EuRoC.yaml 
 * "$pathDatasetEuroc"/MH01             ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt 
 * "$pathDatasetEuroc"/MH02             ./Examples/Monocular/EuRoC_TimeStamps/MH02.txt 
 * "$pathDatasetEuroc"/MH03             ./Examples/Monocular/EuRoC_TimeStamps/MH03.txt 
 * "$pathDatasetEuroc"/MH04             ./Examples/Monocular/EuRoC_TimeStamps/MH04.txt 
 * "$pathDatasetEuroc"/MH05             ./Examples/Monocular/EuRoC_TimeStamps/MH05.txt 
 *  dataset-MH01_to_MH05_mono
*/

// argc = 6:   argc[0]---> argc[5]
int main(int argc, char *argv[])
{

    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    //TODO 这里指参数输入了几个数据集，应该是为了表示他们的多地图集合部分的功能
    const int num_seq = (argc-3)/2;  //=1,
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);  
    string file_name;   //日志文件名
    //这里判断有没有第6个参数(dataset-MH03_monoi),决定是否输出日志文件 f(fk)_dataset-MH03_monoi.txt
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    //存储图像文件名
    vector< vector<string> > vstrImageFilenames;
    //图像时间戳
    vector< vector<double> > vTimestampsCam;
    //使用数据集在初始化的时候存储加速度和陀螺仪～  这里借用的opencv的数组类型
    vector< vector<cv::Point3f> > vAcc, vGyro;
    //存储imu时间戳
    vector< vector<double> > vTimestampsImu;        //imu时间戳，属于秒级的
    vector<int> nImages;    //存储每个数据集图像的数量大小
    vector<int> nImu;           //存储每个数据集imu的数量大小
    //存储每个数据集中第一个有效的imu数据集的下标（也就是时间戳对齐），相对的无效意思是在第一幅图像之前的imu数据
    //另外需要注意，当vImuMeas收集两幅图像之间的imu数据的时候，每次进入vImuMeas中一个数据，该索引值就会累加更新一个
    vector<int> first_imu(num_seq,0);   

    //num_seq代表数据集个数，下面的这些说明每个数据集只需要一个
    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;         //统计一个或者多个数据集的总图像个数大小


/**
 * ./Examples/Monocular-Inertial/mono_inertial_euroc 
 * ./Vocabulary/ORBvoc.txt 
 * ./Examples/Monocular-Inertial/EuRoC.yaml 
 * "$pathDatasetEuroc"/MH03 
 * ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt 
 * dataset-MH03_monoi
*/

/**  多段数据集输入
 * ./Examples/Monocular/mono_euroc 
 * ./Vocabulary/ORBvoc.txt 
 * ./Examples/Monocular/EuRoC.yaml 
 * "$pathDatasetEuroc"/MH01             ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt 
 * "$pathDatasetEuroc"/MH02             ./Examples/Monocular/EuRoC_TimeStamps/MH02.txt 
 * "$pathDatasetEuroc"/MH03             ./Examples/Monocular/EuRoC_TimeStamps/MH03.txt 
 * "$pathDatasetEuroc"/MH04             ./Examples/Monocular/EuRoC_TimeStamps/MH04.txt 
 * "$pathDatasetEuroc"/MH05             ./Examples/Monocular/EuRoC_TimeStamps/MH05.txt 
 *  dataset-MH01_to_MH05_mono
*/

    //! 1   获取每一段数据集的数据
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        //!     载入图像（这里只是载入图像地址）
        /**
         *  [in]pathCam0 = "/mav0/cam0/data"
         *  [in]pathTimeStamps = ./Examples/Monocular/EuRoC_TimeStamps/MH02.txt 
         *  [out]=
         *  [out]=
        */
        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        //! 载入imu数据（存储到相应的vector容器，时间戳转化为秒级别）
        cout << "Loading IMU for sequence " << seq << "...";
        //  [in]pathImu = "/mav0/imu0/data.csv"      [out]vTimestampsImu       [out]       [out] 
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];   //统计总数据集图像大小
        nImu[seq] = vTimestampsImu[seq].size();  //统计总数据集imu数据大小

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        //?  每个数据集的前面imu数据，在时间上如果先于   对应的第一幅图像   情况下，全部丢弃(只保留最后一个)
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;  //记录每一帧图像的跟踪时间
    vTimesTrack.resize(tot_images);

    cout.precision(17);     //输出的时候小数点后保留 17 位

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/


/**
 * ./Examples/Monocular-Inertial/mono_inertial_euroc 
 * ./Vocabulary/ORBvoc.txt 
 * ./Examples/Monocular-Inertial/EuRoC.yaml 
 * "$pathDatasetEuroc"/MH03 
 * ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt 
 * dataset-MH03_monoi
*/


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //   ./Examples/Monocular-Inertial/EuRoC.yaml      "$pathDatasetEuroc"/MH03 
    //!  创建SLAM对象
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);

    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;     //存储 两幅图像之间的imu数据
        proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            //!   Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],CV_LOAD_IMAGE_UNCHANGED);
            //读取当前帧图像的时间戳
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)    //有图像的情况下
            {
                // cout << "t_cam " << tframe << endl;
                //! 将两幅图像之间的imu数据放入到容器vImuMeas中
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(
                                         //三轴加速度
                                        ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                        //三轴陀螺仪
                                                                    vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                        //时间戳
                                                                    vTimestampsImu[seq][first_imu[seq]])
                                                             );
                    first_imu[seq]++;  //更新索引值
                }
            }

            /*cout << "first imu: " << first_imu << endl;
            cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl;
            cout << "size vImu: " << vImuMeas.size() << endl;*/
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            //! 图像和imu数据准备好了，开始进行跟踪
            //图像   时间戳  IMU数据（加速度+陀螺仪+时间戳）
            SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;               //级别是秒级的
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)       //如果还是在本图像数据集中
                T = vTimestampsCam[seq][ni+1]-tframe;       //记录下一副图像和本副图像的时间差
            else if(ni>0)                           //从本数据集转到下一个图像数据集的话，那就参考本帧率和上一帧的时间差
                T = tframe-vTimestampsCam[seq][ni-1];

            //还没到下一幅图像该来的时间，那就差值延时等待
            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6（为了转化为秒单位,因为usleep的单位是us）
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //!  保存轨迹
    //如果传入最后一个参数的话（argc=6）那就按照设定命名并保存轨迹
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

/**
 *  [in]strImagePath = "/mav0/cam0/data"
 *  [in]strPathTimes = ./Examples/Monocular/EuRoC_TimeStamps/MH02.txt 
 *  [out]vstrImages
 *  [out]vTimeStamps
*/
void LoadImages(
    const string &strImagePath,             //[in]strImagePath = "/mav0/cam0/data"(这里面存储图像先后位置是按照时间戳命名的)
    const string &strPathTimes,             //in]strPathTimes = ./Examples/Monocular/EuRoC_TimeStamps/MH02.txt   
    vector<string> &vstrImages,             //[out]vstrImages
    vector<double> &vTimeStamps     //[out]vTimeStamps，秒数量级
    )
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            //获取图像路径
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            //获取时间戳，转化为秒级
            vTimeStamps.push_back(t/1e9);

        }
    }
}

/**
 *  [in]strImuPath = "/mav0/imu0/data.csv"
 *  [out]vTimeStamps = 会转化为秒级的
 *  [out]vAcc
 *  [out]vGyro
*/
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);  //转化单位   ns-->> s
            //加速度   x y z
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            //陀螺仪  x y z
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
