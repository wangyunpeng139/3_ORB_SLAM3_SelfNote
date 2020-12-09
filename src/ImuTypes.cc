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

#include "ImuTypes.h"
#include<iostream>

namespace ORB_SLAM3
{

namespace IMU
{

const float eps = 1e-4;

cv::Mat NormalizeRotation(const cv::Mat &R)
{
    cv::Mat U,w,Vt;
    cv::SVDecomp(R,w,U,Vt,cv::SVD::FULL_UV);
    // assert(cv::determinant(U*Vt)>0);
    return U*Vt;
}

cv::Mat Skew(const cv::Mat &v)
{
    const float x = v.at<float>(0);
    const float y = v.at<float>(1);
    const float z = v.at<float>(2);
    return (cv::Mat_<float>(3,3) << 0, -z, y,
            z, 0, -x,
            -y,  x, 0);
}

cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

Eigen::Matrix<double,3,3> ExpSO3(const double &x, const double &y, const double &z)
{
    Eigen::Matrix<double,3,3> I = Eigen::MatrixXd::Identity(3,3);
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix<double,3,3> W;
    W(0,0) = 0;
    W(0,1) = -z;
    W(0,2) = y;
    W(1,0) = z;
    W(1,1) = 0;
    W(1,2) = -x;
    W(2,0) = -y;
    W(2,1) = x;
    W(2,2) = 0;

    if(d<eps)
        return (I + W + 0.5*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

cv::Mat LogSO3(const cv::Mat &R)
{
    const float tr = R.at<float>(0,0)+R.at<float>(1,1)+R.at<float>(2,2);
    cv::Mat w = (cv::Mat_<float>(3,1) <<(R.at<float>(2,1)-R.at<float>(1,2))/2,
                                        (R.at<float>(0,2)-R.at<float>(2,0))/2,
                                        (R.at<float>(1,0)-R.at<float>(0,1))/2);
    const float costheta = (tr-1.0f)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const float theta = acos(costheta);
    const float s = sin(theta);
    if(fabs(s)<eps)
        return w;
    else
        return theta*w/s;
}

cv::Mat RightJacobianSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        return cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

cv::Mat RightJacobianSO3(const cv::Mat &v)
{
    return RightJacobianSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

cv::Mat InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        return cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

cv::Mat InverseRightJacobianSO3(const cv::Mat &v)
{
    return InverseRightJacobianSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

/**
 * 计算一次旋转量的预积分（确切的说是，计算一次连续两帧imu之间的旋转量）以及对应的 rightJ（右雅克比）
 * 
 * 旋转的累加更新公式如下：
 * dR (k+1)= dR(k) *  exp[  ((陀螺仪-bias)*deta t)^ ]
 * 本对象主要计算的是 exp[  ((陀螺仪-bias)*deta t)^ ]
 * 
 *
*/
//传入参数： 陀螺仪度数、imu的bias(应该没有考虑白噪声)、两个imu之间的时间间隔
IntegratedRotation::IntegratedRotation(const cv::Point3f &angVel, const Bias &imuBias, const float &time):
    deltaT(time)
{
    //计算   w  =  (陀螺仪-bias)*deta t 
    const float x = (angVel.x-imuBias.bwx)*time;
    const float y = (angVel.y-imuBias.bwy)*time;
    const float z = (angVel.z-imuBias.bwz)*time;

    cv::Mat I = cv::Mat::eye(3,3,CV_32F);

    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    //叉乘计算公式  W = w^ ,该公式可以在SLAM14讲叉乘中找到
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    // w是小量的时候，采用一阶泰勒展开，求解 exp(w^)
    //?  rightJ 是做什么的？？   答：参考qxc第3页的李群李代数
    if(d<eps)
    {
        deltaR = I + W;
        rightJ = cv::Mat::eye(3,3,CV_32F);
    }
    //w 非小量的时候，正常计算，可以参考自己手写笔记第一页左边那一页，或者邱笑晨的关于李群的总结公式
    else
    {
        deltaR = I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
        rightJ = I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Preintegrated::Preintegrated(const Bias &b_, const Calib &calib)
{
    //深度copy 噪声协方差矩阵，和随机游走协方差矩阵
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
}

// Copy constructor
Preintegrated::Preintegrated(Preintegrated* pImuPre): dT(pImuPre->dT), C(pImuPre->C.clone()), Info(pImuPre->Info.clone()),
    Nga(pImuPre->Nga.clone()), NgaWalk(pImuPre->NgaWalk.clone()), b(pImuPre->b), dR(pImuPre->dR.clone()), dV(pImuPre->dV.clone()),
    dP(pImuPre->dP.clone()), JRg(pImuPre->JRg.clone()), JVg(pImuPre->JVg.clone()), JVa(pImuPre->JVa.clone()), JPg(pImuPre->JPg.clone()),
    JPa(pImuPre->JPa.clone()), avgA(pImuPre->avgA.clone()), avgW(pImuPre->avgW.clone()), bu(pImuPre->bu), db(pImuPre->db.clone()), mvMeasurements(pImuPre->mvMeasurements)
{

}

void Preintegrated::CopyFrom(Preintegrated* pImuPre)
{
    std::cout << "Preintegrated: start clone" << std::endl;
    dT = pImuPre->dT;
    C = pImuPre->C.clone();
    Info = pImuPre->Info.clone();
    Nga = pImuPre->Nga.clone();
    NgaWalk = pImuPre->NgaWalk.clone();
    std::cout << "Preintegrated: first clone" << std::endl;
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR.clone();
    dV = pImuPre->dV.clone();
    dP = pImuPre->dP.clone();
    JRg = pImuPre->JRg.clone();
    JVg = pImuPre->JVg.clone();
    JVa = pImuPre->JVa.clone();
    JPg = pImuPre->JPg.clone();
    JPa = pImuPre->JPa.clone();
    avgA = pImuPre->avgA.clone();
    avgW = pImuPre->avgW.clone();
    std::cout << "Preintegrated: second clone" << std::endl;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db.clone();
    std::cout << "Preintegrated: third clone" << std::endl;
    mvMeasurements = pImuPre->mvMeasurements;
    std::cout << "Preintegrated: end clone" << std::endl;
}


void Preintegrated::Initialize(const Bias &b_)
{
    dR = cv::Mat::eye(3,3,CV_32F);
    dV = cv::Mat::zeros(3,1,CV_32F);
    dP = cv::Mat::zeros(3,1,CV_32F);

    JRg = cv::Mat::zeros(3,3,CV_32F);
    JVg = cv::Mat::zeros(3,3,CV_32F);
    JVa = cv::Mat::zeros(3,3,CV_32F);
    JPg = cv::Mat::zeros(3,3,CV_32F);
    JPa = cv::Mat::zeros(3,3,CV_32F);

    C = cv::Mat::zeros(15,15,CV_32F);
    Info=cv::Mat();
    db = cv::Mat::zeros(6,1,CV_32F);

    b=b_;
    bu=b_;

    avgA = cv::Mat::zeros(3,1,CV_32F);
    avgW = cv::Mat::zeros(3,1,CV_32F);
    
    dT=0.0f;
    mvMeasurements.clear();
}

void Preintegrated::Reintegrate()
{
    std::unique_lock<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    Initialize(bu);
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}

//传入变量： 加速度   陀螺仪  IMU之间的时间间隔进行预积分
//注意第三个参数：两个imu之间的时间间隔（如果是两幅图像之间imu中，第一个和最后一个imu的对应处理会有所不同，
//              指的更多的是imu与图像之间的时间间隔）
// 建议这个函数结合 vins-mono里文件  integration_base.h  的函数   midPointIntegration进行理解，主要是邱笑晨的对于Forster的文章的公式推导
//? 根据qxc第四章节的推到公式，分离出来白噪声后，更新公式为     理想值 = 预积分测量值 - 白噪声分量部分，但是该函数只是在这里计算出了“预积分测量值”（dR dV dP）
//? 还没有减去本次对应的 “白噪声分量部分”，所以函数结束后计算的应该还不是最终值，还缺少一步～～
//?本次的“白噪声分量部分”可以利用A、B和上一次的计算值就可以得出来（参考qxc12页），不过该函数只是计算出来A和B
//? 只是利用A和B计算了一下协方差，并没有用来计算白噪声部分。。。。。。。
void Preintegrated::IntegrateNewMeasurement(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &dt)
{
    // integrable 是 类对象 Preintegrated 里面的一个结构体，主要记录加速度、陀螺仪、时间间隔
    mvMeasurements.push_back(integrable(acceleration,angVel,dt));

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    //Matrices to compute covariance
    //下面两个矩阵是与噪声更新有关的，具体推到结果见CXQ第 P12页
    /**
     *  nm : 表示测量噪声
     *  nw: 白噪声 = [陀螺仪白噪声^T，加速度计白噪声^T]^T 
     * nm(k) = A * nm(k-1) + B * nw
     * 
    */
    cv::Mat A = cv::Mat::eye(9,9,CV_32F);
    cv::Mat B = cv::Mat::zeros(9,6,CV_32F);

    //! 注意，下面的 acc 和 accw 在公式中代表真值，表示完全理想的真值（参考自己手写的笔记）
    //加速度测量值 - 偏置
    //? 这里面的b是怎么进行赋值的？  
    //首答： 既然计算后的acc是实际的完全理想真值，按照公式，容易知道b应该包括 随机游走bias和白噪声bias，
    //      但是推导的时候将包含白噪声的部分给分离出去了，所以这里没有减去白噪声部分，可以参考qxc P12页那个大矩阵
    cv::Mat acc = (cv::Mat_<float>(3,1) << acceleration.x-b.bax,acceleration.y-b.bay, acceleration.z-b.baz);
    //陀螺仪测量值 - 除偏置
    cv::Mat accW = (cv::Mat_<float>(3,1) << angVel.x-b.bwx, angVel.y-b.bwy, angVel.z-b.bwz);

    //?这里难道计算的是总的加速度吗？？  好像就是计算的总的，如果是刚开始那么就会有avgA = 0的情况
    //? 总的加速度和陀螺仪
    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    avgW = (dT*avgW + accW*dt)/(dT+dt);

    //!  需要注意一下，下面的多出来的dR是做什么的？？？
    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    //?  这里多出来的dR是做什么的？？
    //! 参考qxc 第8、9两页（注意这个时候已经分离出来了包含白噪声部分的那一部分分量了，也就是说下面计算的是 “增量预积分测量值”）
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt;    //计算位移的
    dV = dV + dR*acc*dt;                //计算速度

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    //! 加速度的叉乘
    //叉乘的通式，_x表示叉乘，叉乘就是类似于下面的堆成矩阵
    /**
     *                 | 0      -W_z     W_y |
     * [W]_x = | W_z     0      -W_x |
     *                 | -W_y   W_x       0  |
    */
    // Wacc是叉乘得到的反对称矩阵
    cv::Mat Wacc = (cv::Mat_<float>(3,3) << 0, -acc.at<float>(2), acc.at<float>(1),
                                                   acc.at<float>(2), 0, -acc.at<float>(0),
                                                   -acc.at<float>(1), acc.at<float>(0), 0);
    //! 关于下面的A 和 B的共视原理，请参考 QXC 第12页
    //注意：这里的dR使用的是上一次的，12页里面的公式也是这样体现的
    A.rowRange(3,6).colRange(0,3) = -dR*dt*Wacc;                                 //矩阵（2,1）
    A.rowRange(6,9).colRange(0,3) = -0.5f*dR*dt*dt*Wacc;                 //矩阵（3,1）
    A.rowRange(6,9).colRange(3,6) = cv::Mat::eye(3,3,CV_32F)*dt;   //矩阵（3,2）

    B.rowRange(3,6).colRange(3,6) = dR*dt;                           //矩阵（2,2）
    B.rowRange(6,9).colRange(3,6) = 0.5f*dR*dt*dt;           //矩阵（3,2）

    // Update position and velocity jacobians wrt bias correction
    //! 参考qxc 第 16页，这里例如：  JPa     是指位置增量对加速度偏值的求导雅克比
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // Update delta rotation
    IntegratedRotation dRi(angVel,b,dt);
    //! dR (k+1)= dR(k) *  exp[  ((陀螺仪-bias)*deta t)^ ]   这个公式参考qxc第7页下面倒数第二个公式（最后一个计算的是噪声项），上面一行就是计算  exp[  ((陀螺仪-bias)*deta t)^ ] 
    //? 但是这里的 dR包括上面的 dP  dV都不算是最终的吧？这里只是累加上了 预积分测量值，并没有累加上白噪声相关的部分呀
    dR = NormalizeRotation(dR*dRi.deltaR);

    // Compute rotation parts of matrices A and B
    //上面已经计算出来了本次的 dRi 和对应的rightJ，现在可以更新矩阵的第一行第一列了
    A.rowRange(0,3).colRange(0,3) = dRi.deltaR.t();    //矩阵（1,1）
    B.rowRange(0,3).colRange(0,3) = dRi.rightJ*dt;    //矩阵（1,1）

    // Update covariance
    //! 在 qxc第12页最下面
    C.rowRange(0,9).colRange(0,9) = A*C.rowRange(0,9).colRange(0,9)*A.t() + B*Nga*B.t();
    //! 这一行的理论支持是什么？？没找到呀  上面的是测量噪声，下面的是 随机游走部分，在qxc P11、P12下面测量误差的下面添加上随机游走误差就行了
    C.rowRange(9,15).colRange(9,15) = C.rowRange(9,15).colRange(9,15) + NgaWalk;

    // Update rotation jacobian wrt bias correction
    //! qxc 第  14页   ，应该只有   - dRi.rightJ*dt 才对呀，最前面的  dRi.deltaR.t()*JRg 不该有吧？
    JRg = dRi.deltaR.t()*JRg - dRi.rightJ*dt;
    //TODO 是不是上面的应该修改为下面一行
 #if  0   
    //skylor : 王云鹏认为应该这样修改，理由如下：参考qxc14页的推到， JRg = JRg - △R(j,j)^T * rightJ*dt，
    //但是从j-->>j的变换为I，所以 JRg = JRg - dRi.rightJ*dt
    JRg = JRg - dRi.rightJ*dt;
#endif   
    // Total integrated time
    dT += dt;
}

void Preintegrated::MergePrevious(Preintegrated* pPrev)
{
    if (pPrev==this)
        return;

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

void Preintegrated::SetNewBias(const Bias &bu_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    bu = bu_;

    db.at<float>(0) = bu_.bwx-b.bwx;
    db.at<float>(1) = bu_.bwy-b.bwy;
    db.at<float>(2) = bu_.bwz-b.bwz;
    db.at<float>(3) = bu_.bax-b.bax;
    db.at<float>(4) = bu_.bay-b.bay;
    db.at<float>(5) = bu_.baz-b.baz;
}

IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    return IMU::Bias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}

//计算本次预积分的旋转增量（IMU坐标系，参考cxq13页的公式，从i->j就知道了）
//（不过只是预积分，好像并没有所谓的优化呀？或许是自己刚看到跟踪线程吧）
cv::Mat Preintegrated::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    //! 计算重力bias的变化差分量
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    //! 参考邱笑晨第13页最上面一行
    return NormalizeRotation(dR*ExpSO3(JRg*dbg));
}

//计算速度增量（IMU坐标系，参考cxq13页的公式，从i->j就知道了）
cv::Mat Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    cv::Mat dba = (cv::Mat_<float>(3,1) << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz);
    return dV + JVg*dbg + JVa*dba;
}
//计算位移增量（IMU坐标系，参考cxq13页的公式，从i->j就知道了）
cv::Mat Preintegrated::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    cv::Mat dba = (cv::Mat_<float>(3,1) << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz);
    return dP + JPg*dbg + JPa*dba;
}

//此函数，包括下面两个函数，参考 qxc P13页
cv::Mat Preintegrated::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return NormalizeRotation(dR*ExpSO3(JRg*db.rowRange(0,3)));
}

cv::Mat Preintegrated::GetUpdatedDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV + JVg*db.rowRange(0,3) + JVa*db.rowRange(3,6);
}

cv::Mat Preintegrated::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP + JPg*db.rowRange(0,3) + JPa*db.rowRange(3,6);
}

cv::Mat Preintegrated::GetOriginalDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dR.clone();
}

cv::Mat Preintegrated::GetOriginalDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV.clone();
}

cv::Mat Preintegrated::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP.clone();
}

Bias Preintegrated::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return b;
}

Bias Preintegrated::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

cv::Mat Preintegrated::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db.clone();
}

Eigen::Matrix<double,15,15> Preintegrated::GetInformationMatrix()
{
    std::unique_lock<std::mutex> lock(mMutex);
    if(Info.empty())
    {
        Info = cv::Mat::zeros(15,15,CV_32F);
        Info.rowRange(0,9).colRange(0,9)=C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
        for(int i=9;i<15;i++)
            Info.at<float>(i,i)=1.0f/C.at<float>(i,i);
    }

    Eigen::Matrix<double,15,15> EI;
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            EI(i,j)=Info.at<float>(i,j);
    return EI;
}

void Bias::CopyFrom(Bias &b)
{
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

std::ostream& operator<< (std::ostream &out, const Bias &b)
{
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

/**
 * Tbc_ : 相机到imu的相对变换 
 * ng : 陀螺仪的噪声
 * na : 加速度测量的噪声
 * ngw: 陀螺仪的随机游走
 * naw：加速度的随机游走  
*/
void Calib::Set(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
{
    Tbc = Tbc_.clone();
    Tcb = cv::Mat::eye(4,4,CV_32F);
    Tcb.rowRange(0,3).colRange(0,3) = Tbc.rowRange(0,3).colRange(0,3).t();
    Tcb.rowRange(0,3).col(3) = -Tbc.rowRange(0,3).colRange(0,3).t()*Tbc.rowRange(0,3).col(3);
    Cov = cv::Mat::eye(6,6,CV_32F);
    const float ng2 = ng*ng;
    const float na2 = na*na;
    Cov.at<float>(0,0) = ng2;
    Cov.at<float>(1,1) = ng2;
    Cov.at<float>(2,2) = ng2;
    Cov.at<float>(3,3) = na2;
    Cov.at<float>(4,4) = na2;
    Cov.at<float>(5,5) = na2;
    CovWalk = cv::Mat::eye(6,6,CV_32F);
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;
    CovWalk.at<float>(0,0) = ngw2;
    CovWalk.at<float>(1,1) = ngw2;
    CovWalk.at<float>(2,2) = ngw2;
    CovWalk.at<float>(3,3) = naw2;
    CovWalk.at<float>(4,4) = naw2;
    CovWalk.at<float>(5,5) = naw2;
}

Calib::Calib(const Calib &calib)
{
    Tbc = calib.Tbc.clone();
    Tcb = calib.Tcb.clone();
    Cov = calib.Cov.clone();
    CovWalk = calib.CovWalk.clone();
}

} //namespace IMU

} //namespace ORB_SLAM2
