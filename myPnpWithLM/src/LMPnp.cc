#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include"opencv2/opencv.hpp"
#include <sophus/se3.hpp>
#include<iostream>
#include<vector>
using namespace Eigen;
using namespace std;
using namespace cv;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
#define M_PI 3.14159265358979323846
//合作目标上的点
const VecVector3d dataPw{
			{-9, 1, 1},
			{-9, 7, 2},
			{-9, 14, 3},
			{-9, 21, 2},
			{-9, 28, 5},
			{-9, 35, 1},
			{-3, 1, 5},
			{-3, 14, 5},
			{-3, 11, 1},
			{-3, 2, 4},
			{-3, 8, 12},
			{-3, 35, 13},
			{3, 12, 1},
			{3, 7, 2},
			{3, 14, 3},
			{3, 21, 0},
			{3, 28, 0},
			{3, 35, 0},
			{9, 0, 0},
			{9, 7, 0},
			{9, 14, 0},
			{9, 21, 0},
			{9, 28, 0},
			{9, 35, 0} };
/**
*@brief 从相机坐标系转换到像素坐标系
*@param Pc 当前帧的世界坐标系
*@param K 相机内参数
*@return 返回转为的像素坐标
*/

Vector2d cam2pixel(const Vector3d &Pc) {
	auto x = Pc[0] / Pc[2];
	auto y = Pc[1] / Pc[2];
	return Vector2d(x, y);
}
void fromWorld2Pixel(const Vector3d &Pw,Vector2d &p) {
	Matrix3d K;
	K << 700, 0,500, 0, 700, 500, 0, 0, 1;
	double fx = K(0, 0);
	double fy = K(1, 1);
	double cx = K(0, 2);
	double cy = K(1, 2);
    Isometry3d T = Isometry3d::Identity(); 
	AngleAxisd Rc_vector(M_PI/3, Vector3d(0, 0, 1));
	T.rotate(Rc_vector);                                     
	T.pretranslate(Vector3d(11, 9, 5));                     
	Vector3d  Pc =K* T * Pw;
	p = cam2pixel(Pc);
}
void fromse32Se3(Matrix<double,1,6> &se3,Isometry3d &Se3){
    Vector3d N = Vector3d(se3(0,3),se3(0,4),se3(0,5));
    auto theta = N.norm();
    Vector3d n = N.normalized();
    AngleAxisd R_vector(theta,n);
    Se3.rotate(R_vector);
    Se3.pretranslate(Vector3d(se3(0,0),se3(0,1),se3(0,2)));
    
}           
void makePnpTestData(VecVector3d &Pw ,VecVector2d &p){
    RNG rng;   
    double noise1 = rng.gaussian(3);
    double noise2 = rng.gaussian(3);
    for(int i = 0;i<6;i++)
    {
        Vector2d pTemp ;
        fromWorld2Pixel(dataPw[i],pTemp);
        pTemp[0] +=noise1;
        pTemp[1] +=noise2;
        p.push_back(pTemp);
        Pw.push_back(dataPw[i]);
        }
}

void LMPnp(const VecVector3d &Pw ,const VecVector2d &p ,Sophus::SE3d &pose){
    int maxtimes = 50;
    double v = 2;
    double rho = 0.0;
    double tao = 1e-14;
    //相机内参
    Matrix3d K;
	K << 700, 0,500, 0, 700, 500, 0, 0, 1;
	double fx = K(0, 0);
	double fy = K(1, 1);
	double cx = K(0, 2);
	double cy = K(1, 2);

    //获得初值
    //注意初始化两个H和g,不然会有数值异常的风险
    Matrix<double,6,6> H = Matrix<double,6,6>::Zero();
    Matrix<double,6,1> g = Matrix<double,6,1>::Zero();
    double cost =0.0;
    //装填数据,获得一个初始值
    for (int j = 0; j < Pw.size(); j++) {
        //从世界坐标系Pw到相机坐标系Pc
        Vector3d pc = pose * Pw[j];
        //方便计算雅克比矩阵
        double inv_z = 1.0 / pc[2];
        double inv_z2 = inv_z * inv_z;
        //计算预测点的位置
        Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
        //计算残差,此处为二维
        auto e = p[j] - proj;
        cost += e.squaredNorm()*e.squaredNorm();
        //装填雅克比,用于求H和g
        Matrix<double, 2, 6> J = Matrix<double,2,6>::Zero();
        J << -fx * inv_z,
            0,
            fx * pc[0] * inv_z2,
            fx * pc[0] * pc[1] * inv_z2,
            -fx - fx * pc[0] * pc[0] * inv_z2,
            fx * pc[1] * inv_z,
            0,
            -fy * inv_z,
            fy * pc[1] * inv_z2,
            fy + fy * pc[1] * pc[1] * inv_z2,
            -fy * pc[0] * pc[1] * inv_z2,
            -fy * pc[0] * inv_z;
        //可以写成一个长列或者相加,两者等价
        H += J.transpose() * J;
        g += -J.transpose() * e;
        }
    //使用eigen
    double u = tao*H.maxCoeff();
    //cout<<"init u :"<<u<<endl;
    //cout<<"H init"<<H.matrix()<<endl;
    //cout<<"g init"<<g.matrix()<<endl;

    Matrix<double,6,6> I = Matrix<double,6,6>::Identity(6, 6);
    //cout<<I.matrix()<<endl;

    for(int i =1; i< maxtimes;i++){
        
        Matrix<double,6,6> A = H+u*I;
       //使用eigen库,求解线性方程H*delta = -J * g
        Matrix<double,6,1> delta = A.ldlt().solve(g);
        //一系列判断终止的条件
        if (isnan(delta.norm())) {
        //cout << "位姿结果已发散!" << endl;
        break;
        }
        if (delta.norm() < 1e-7) {
        //cout << "1位姿结果已收敛." << endl;
        break;
        }
        //使用左乘扰动的方法更新SE3格式的T,不可以使用+delta的方式,无法收敛
        pose = Sophus::SE3d::exp(delta) * pose;
        double cost_new = 0;

        for(int j = 0;j< Pw.size();j++){  
            //从世界坐标系Pw到相机坐标系Pc
            auto pc = pose * Pw[j];
            //方便计算雅克比矩阵
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            //计算预测点的位置
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
            //计算残差,此处为二维
            Vector2d e = p[j] - proj;
            cost_new += e.squaredNorm()*e.squaredNorm();
        }
        //cout<< "cost : "<<cost<<endl;
        //cout<< "costNew : " <<cost_new<<endl;
        rho = (cost - cost_new)/(delta.transpose()*(u*delta+g));

        //LM的工作
        if(rho >0 ){
            //装填数据
            //数据清零,因为全局的数据一直在叠加
            cost = 0;
            H = Matrix<double,6,6>::Zero();
            g = Matrix<double,6,1> ::Zero();
            for (int j = 0; j < Pw.size(); j++) {
                //从世界坐标系Pw到相机坐标系Pc
                auto pc = pose * Pw[j];
                //方便计算雅克比矩阵
                double inv_z = 1.0 / pc[2];
                double inv_z2 = inv_z * inv_z;
                //计算预测点的位置
                Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
                //计算残差,此处为二维
                auto e = p[j] - proj;
               
                //装填雅克比,用于求H和g
                Eigen::Matrix<double, 2, 6> J;
                J << -fx * inv_z,
                    0,
                    fx * pc[0] * inv_z2,
                    fx * pc[0] * pc[1] * inv_z2,
                    -fx - fx * pc[0] * pc[0] * inv_z2,
                    fx * pc[1] * inv_z,
                    0,
                    -fy * inv_z,
                    fy * pc[1] * inv_z2,
                    fy + fy * pc[1] * pc[1] * inv_z2,
                    -fy * pc[0] * pc[1] * inv_z2,
                    -fy * pc[0] * inv_z;
                //可以写成一个长列或者相加,两者等价
                H += J.transpose() * J;
                g += -J.transpose() * e;
                cost += e.squaredNorm()*e.squaredNorm();

                }
                u = u*max(0.3333,(1-(2*rho-1)*(2*rho-1)*(2*rho-1)));
               // cout<<"u乘的东西"<<max(0.3333,(1-(2*rho-1)*(2*rho-1)*(2*rho-1)))<<endl;
                v=2;
            }
            else{
                //cout<<"不满足二次逼近,残差增加"<<endl;
                u = u * v;
                v = 2 * v;
            }
            cout<<setiosflags(ios::left)
            <<setw(2)<<"第"
            <<setw(3)<<i<<"次"
            <<setw(14)<<"   cost : "
            <<setw(14)<<cost
            <<setw(14)<<"   deltaChange : "
            <<setw(14)<<delta.norm()
            <<setw(14)<<"   u :"
            <<setw(14)<<u
            <<setw(14)<<"   rho : "
            <<setw(14)<<rho
            <<endl;  
            
        }
        cout << "位姿优化结果: \n" << pose.matrix() << endl;

}
void gaussNewtonPnp(const VecVector3d &Pw ,const VecVector2d &p ,Sophus::SE3d &pose){
   
    double lastCost = 0.0;
    int maxtimes = 100;
    //相机内参
    Matrix3d K;
	K << 700, 0,500, 0, 700, 500, 0, 0, 1;
	double fx = K(0, 0);
	double fy = K(1, 1);
	double cx = K(0, 2);
	double cy = K(1, 2);
    //迭代循环
    for(int i =1; i< maxtimes;i++){
        //注意初始化两个H和g,不然会有数值异常的风险
        Matrix<double,6,6> H = Matrix<double,6,6>::Zero();
        Matrix<double,6,1> g = Matrix<double,6,1>::Zero();
         double cost =0.0;
        //装填数据
        for (int j = 0; j < Pw.size(); j++) {
            //从世界坐标系Pw到相机坐标系Pc
            auto pc = pose * Pw[j];
            //方便计算雅克比矩阵
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            //计算预测点的位置
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
            //计算残差,此处为二维
            auto e = p[j] - proj;
            cost += e.squaredNorm();
            //装填雅克比,用于求H和g
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z,
                0,
                fx * pc[0] * inv_z2,
                fx * pc[0] * pc[1] * inv_z2,
                -fx - fx * pc[0] * pc[0] * inv_z2,
                fx * pc[1] * inv_z,
                0,
                -fy * inv_z,
                fy * pc[1] * inv_z2,
                fy + fy * pc[1] * pc[1] * inv_z2,
                -fy * pc[0] * pc[1] * inv_z2,
                -fy * pc[0] * inv_z;
            //可以写成一个长列或者相加,两者等价
            H += J.transpose() * J;
            g += -J.transpose() * e;
            }

    //使用eigen库,求解线性方程H*delta = -J * g
    Matrix<double,6,1> delta = H.ldlt().solve(g);

    //一系列判断终止的条件
    if (isnan(delta.norm())) {
      cout << "位姿结果已发散!" << endl;
      break;
    }
    if (delta.norm() < 1e-7) {
      cout << "位姿结果已收敛." << endl;
      break;
    }
    if (abs(cost-lastCost) < 1e-10) {
      // cost increase, update is not good
      cout << "位姿结果已收敛." << endl;
      break;
    }

    //使用左乘扰动的方法更新SE3格式的T,不可以使用+delta的方式,无法收敛
    pose = Sophus::SE3d::exp(delta) * pose;
    lastCost = cost;//更新cost
    cout<<setiosflags(ios::left)
    <<setw(2)<<"第"
    <<setw(3)<<i<<"次"
    <<setw(14)<<"   cost : "
    <<setw(14)<<cost
    <<setw(14)<<"   deltaChange : "
    <<setw(14)<<delta.norm()
    <<endl;  
    }
    cout << "位姿优化结果: \n" << pose.matrix() << endl;

}

int main(){
    cout.precision(8);
    
    VecVector3d Pw{};
    VecVector2d p{};
    makePnpTestData(Pw ,p);

    //设置初始值
    Sophus::SE3d poseGNinit;
    Sophus::SE3d poseLMinit;
    cout<<"位姿初值 : "<<endl<<poseGNinit.matrix()<<endl;
    cout<<">>>>>>>开始GuassNewrton法求解Pnp问题 >>>>>>>>> " <<endl<<endl; 
    gaussNewtonPnp(Pw , p ,poseGNinit);
    
    cout<<"位姿初值 : "<<endl<<poseLMinit.matrix()<<endl;
    cout<<">>>>>>>开始LM法求解Pnp问题 >>>>>>>>> " <<endl<<endl; 
    LMPnp(Pw , p ,poseLMinit);

    //设置的真实位姿
    Isometry3d T = Isometry3d::Identity(); 
	AngleAxisd Rc_vector(M_PI/3, Vector3d(0, 0, 1));
	T.rotate(Rc_vector);                                     
	T.pretranslate(Vector3d(11, 9, 5));  
    cout << "真实位姿结果: \n" << T.matrix() <<endl;
 
}