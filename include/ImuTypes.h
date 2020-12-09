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


#ifndef IMUTYPES_H
#define IMUTYPES_H

#include<vector>
#include<utility>
#include<opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3
{

namespace IMU
{

//! 预设值，重力加速度
const float GRAVITY_VALUE=9.81;

//IMU measurement (gyro, accelerometer and timestamp)
class Point
{
public:
    Point(const float &acc_x, const float &acc_y, const float &acc_z,
             const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
             const double &timestamp): a(acc_x,acc_y,acc_z), w(ang_vel_x,ang_vel_y,ang_vel_z), t(timestamp){}
    Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp):
        a(Acc.x,Acc.y,Acc.z), w(Gyro.x,Gyro.y,Gyro.z), t(timestamp){}
public:
    cv::Point3f a;      //加速度
    cv::Point3f w;     //陀螺仪
    double t;              //时间戳（秒级的）
};

//IMU biases (gyro and accelerometer)
class Bias
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & bax;
        ar & bay;
        ar & baz;

        ar & bwx;
        ar & bwy;
        ar & bwz;
    }

public:
    //由于我们必须解决一个非线性优化问题，我们需要一个惯性参数的初始猜测。
    //尽管重力方向则沿着加速度计测量值的平均值进行初始化，但是加速度通常比重力小得多，因此，我们将偏差初始化为零
    //(人工翻译，或许不准)
    Bias():bax(0),bay(0),baz(0),bwx(0),bwy(0),bwz(0){}
    Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
            const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
            bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){}
    void CopyFrom(Bias &b);
    friend std::ostream& operator<< (std::ostream &out, const Bias &b);

public:
    float bax, bay, baz;
    float bwx, bwy, bwz;
};

//IMU calibration (Tbc, Tcb, noise)
//! 里面存储的主要是一些协方差矩阵，以及相机与imu之间的相对变换
class Calib
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
        serializeMatrix(ar,Tcb,version);
        serializeMatrix(ar,Tbc,version);
        serializeMatrix(ar,Cov,version);
        serializeMatrix(ar,CovWalk,version);
    }

public:
    /**
     * Tbc_ : 相机到imu的相对变换 
     * ng : 陀螺仪的噪声
     * na : 加速度测量的噪声
     * ngw: 陀螺仪的随机游走
     * naw：加速度的随机游走  
    */
    Calib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
    {
        Set(Tbc_,ng,na,ngw,naw);
    }
    Calib(const Calib &calib);
    Calib(){}

    void Set(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw);

public:
    cv::Mat Tcb;                  
    cv::Mat Tbc;        //相机到imu的相对变换 
    //参考函数： Calib::Set  （下面两个应该是协方差矩阵，对角线赋值噪声的平方，先陀螺仪，再加速度）
    cv::Mat Cov;                 //噪声协方差矩阵
    cv::Mat CovWalk;      //随机游走协方差矩阵
};

//Integration of 1 gyro measurement
/**
 * 计算一次预积分（确切的说是，计算一次连续两帧imu之间的旋转量）
 * 旋转的累加更新公式如下：  
 * dR (k+1)= dR(k) *  exp[  ((陀螺仪-bias)*deta t)^ ]
 *  上面这个公式参考qxc第7页下面倒数第二个公式（最后一个计算的是噪声项）
 * 
 * 本对象主要计算的是 exp[  ((陀螺仪-bias)*deta t)^ ]
*/
class IntegratedRotation
{
public:
    IntegratedRotation(){}
    //传入参数： 陀螺仪度数、imu的bias(应该没有考虑白噪声)、两个imu之间的时间间隔
    IntegratedRotation(const cv::Point3f &angVel, const Bias &imuBias, const float &time);

public:
    //相邻两帧之间的时间间隔（也有可能是imu与最近的图像帧之间的时间间隔，这个时候一般是图像最近的imu才会有）
    float deltaT; //integration time
    // dR (k+1)= dR(k) *  exp[  ((陀螺仪-bias)*deta t)^ ]  ，而 deltaR =   exp[  ((陀螺仪-bias)*deta t)^ ] 
    cv::Mat deltaR; //integrated rotation
    //?  rightJ 是做什么的？？   答：参考qxc第3页的李群李代数
    cv::Mat rightJ; // right jacobian    
};


//该对象内部并没有静态变量
//Preintegration of Imu Measurements
class Preintegrated
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

    //? 友元对象？？？
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & dT;
        serializeMatrix(ar,C,version);
        serializeMatrix(ar,Info,version);
        serializeMatrix(ar,Nga,version);
        serializeMatrix(ar,NgaWalk,version);
        ar & b;
        serializeMatrix(ar,dR,version);
        serializeMatrix(ar,dV,version);
        serializeMatrix(ar,dP,version);
        serializeMatrix(ar,JRg,version);
        serializeMatrix(ar,JVg,version);
        serializeMatrix(ar,JVa,version);
        serializeMatrix(ar,JPg,version);
        serializeMatrix(ar,JPa,version);
        serializeMatrix(ar,avgA,version);
        serializeMatrix(ar,avgW,version);

        ar & bu;
        serializeMatrix(ar,db,version);
        ar & mvMeasurements;
    }

public:
    Preintegrated(const Bias &b_, const Calib &calib);
    Preintegrated(Preintegrated* pImuPre);
    Preintegrated() {}
    ~Preintegrated() {}
    void CopyFrom(Preintegrated* pImuPre);
    void Initialize(const Bias &b_);
    void IntegrateNewMeasurement(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &dt);
    void Reintegrate();
    void MergePrevious(Preintegrated* pPrev);
    void SetNewBias(const Bias &bu_);
    IMU::Bias GetDeltaBias(const Bias &b_);
    cv::Mat GetDeltaRotation(const Bias &b_);
    cv::Mat GetDeltaVelocity(const Bias &b_);
    cv::Mat GetDeltaPosition(const Bias &b_);
    cv::Mat GetUpdatedDeltaRotation();
    cv::Mat GetUpdatedDeltaVelocity();
    cv::Mat GetUpdatedDeltaPosition();
    cv::Mat GetOriginalDeltaRotation();
    cv::Mat GetOriginalDeltaVelocity();
    cv::Mat GetOriginalDeltaPosition();
    Eigen::Matrix<double,15,15> GetInformationMatrix();
    cv::Mat GetDeltaBias();
    Bias GetOriginalBias();
    Bias GetUpdatedBias();

public:
//? 这里的dT是做什么的？？
    float dT;        // 累加IMU序列间的时间
    //!  预积分测量噪声的协方差矩阵（15*15） 。在 qxc P12下面可以找到，这里的C（15*15）比P12下面的矩阵行列（9*9）更大一些，
    //!是因为这里在测量误差矩阵的下面添加了随机游走误差部分（恒定值，不是推算计算值）
    cv::Mat C;      // 9~11 对应陀螺仪随机游走   ， 12~14对应加速度随机游走
    cv::Mat Info;    //! 信息矩阵,9*9上面协方差矩阵左上角的9*9矩阵的逆就是该值
    cv::Mat Nga, NgaWalk;  //深度copy 噪声协方差矩阵，和随机游走协方差矩阵（从上到下先是陀螺仪，然后是加速度）

    // Values for the original bias (when integration was computed)
    //加速度和陀螺仪的偏差，在初始化的时候假定为常数
    Bias b;
    //下面三个量是在累计计算两幅图像帧率之间的 R  V  P
    // 在一开始的时候会把两幅图像之间的imu数据存在一块，
    //然后调用class里面的预积分计算慢慢累加dR, dV, dP。等计算接下来新的图像之间的数据的时候，再重新累加
    cv::Mat dR, dV, dP;

    cv::Mat JRg, JVg, JVa, JPg, JPa;   //! 雅克比矩阵，例如 JRg代表，相对旋转量对陀螺仪的求导
    cv::Mat avgA;
    cv::Mat avgW;


private:
    // Updated bias
    Bias bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    cv::Mat db;

    struct integrable
    {
        integrable(const cv::Point3f &a_, const cv::Point3f &w_ , const float &t_):a(a_),w(w_),t(t_){}
        //下面的值都是指的是两个imu之间的数据，
        //orbV3使用的是两个imu数据的中间值，使用的是直接的原始数据没有坐标系的转换
        cv::Point3f a;   //imu加速度数据
        cv::Point3f w;   //imu的陀螺仪数据
        //两个imu之间的时间间隔（如果是两幅图像之间imu中，第一个和最后一个imu的对应处理会有所不同，
        //指的更多的是imu与图像之间的时间间隔）
        float t;        
    };

    //类对象 Preintegrated 里面的一个结构体，主要记录加速度、陀螺仪、时间间隔
    std::vector<integrable> mvMeasurements;

    std::mutex mMutex;
};

// Lie Algebra Functions
cv::Mat ExpSO3(const float &x, const float &y, const float &z);
Eigen::Matrix<double,3,3> ExpSO3(const double &x, const double &y, const double &z);
cv::Mat ExpSO3(const cv::Mat &v);
cv::Mat LogSO3(const cv::Mat &R);
cv::Mat RightJacobianSO3(const float &x, const float &y, const float &z);
cv::Mat RightJacobianSO3(const cv::Mat &v);
cv::Mat InverseRightJacobianSO3(const float &x, const float &y, const float &z);
cv::Mat InverseRightJacobianSO3(const cv::Mat &v);
cv::Mat Skew(const cv::Mat &v);
cv::Mat NormalizeRotation(const cv::Mat &R);

}

} //namespace ORB_SLAM2

#endif // IMUTYPES_H
