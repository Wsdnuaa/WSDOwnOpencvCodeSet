#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include"opencv2/opencv.hpp"
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
			{-9, 0, 0},
			{-9, 7, 0},
			{-9, 14, 0},
			{-9, 21, 0},
			{-9, 28, 0},
			{-9, 35, 0},
			{-3, 0, 0},
			{-3, 7, 0},
			{-3, 14, 0},
			{-3, 21, 0},
			{-3, 28, 0},
			{-3, 35, 0},
			{3, 0, 0},
			{3, 7, 0},
			{3, 14, 0},
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

	AngleAxisd Rc_vector(M_PI/4, Vector3d(-1, 0, 0));
	// 
	Isometry3d T = Isometry3d::Identity();               
	T.rotate(Rc_vector);                                     
	T.pretranslate(Vector3d(10, 20, 30));                     
	//cout << " Pcur :" << Pcur << endl;
	Vector3d  Pc =K* T * Pw;
	//cout << " Pc :" << Pc << endl;
	p = cam2pixel(Pc);
}           
void makePnpTestData(VecVector3d &Pw ,VecVector2d &p){
    RNG rng;   
    double noise1 = rng.gaussian(1);
    double noise2 = rng.gaussian(1);
    for(int i = 0;i<10;i++)
    {
        Vector2d pTemp ;
        fromWorld2Pixel(dataPw[i],pTemp);
        pTemp[0] +=noise1;
        pTemp[1] +=noise2;
        p.push_back(pTemp);
        Pw.push_back(dataPw[i]);
        cout<<pTemp<<endl;
        }
}
void LM(const VecVector3d &Pw ,const VecVector2d &p ,Matrix<double,1,6> &pose){
    bool flag = 0; //记录解算是否发散
    double cost =0.0;
    double lastcost = 0.0;
    int maxtimes = 1000;
    double v = 2;
    double rho = 0;
    double tao = 1e-10;
    //获得初值
    Matrix<double,6,6> H = Matrix<double,6,6>::Zero();
    Matrix<double,1,6> g = Matrix<double,1,6>::Zero();
    Matrix<double,2,6> J;
    //装填数据
    for(int j = 0;j< Pw.size();j++){  
    
        
        Matrix<double,6,6> tempH = J.transpose()*J; 
        H +=tempH;
        g += -J.transpose*e;
        cost +=e*e;
    }
    // //使用eigen
    // double u = tao*H.maxCoeff();
    // cout<<"init u :"<<u<<endl;
    // cout<<"H init"<<H.matrix()<<endl;
    // cout<<"J init"<<J.matrix()<<endl;
    // cout<<"g init"<<g.matrix()<<endl;
    // Matrix3d I = MatrixXd::Identity(3, 3);
    // for(int i =0; i< maxtimes;i++){
        
    //     //使用eigen结算线性方程组
    //     Matrix3d A = H+u*I;
    //     Vector3d delta_abc = A.ldlt().solve(g);
    //     //cout<<"delta_abc"<<delta_abc<<endl;
    //     //Vector3d delta_abc = H.ldlt().solve(g);
    //     if(delta_abc.norm()<1e-12){   
    //         flag =1;
    //         break;
    //             }
    //     cout<<"delta_abc"<<delta_abc.transpose()<<endl;
    //     if(isnan(delta_abc[0])||isnan(delta_abc[1])||isnan(delta_abc[2])){   
    //         flag =0;
    //         break;
    //             }
    //     a += delta_abc[0];
    //     b += delta_abc[1];
    //     c += delta_abc[2];
    //     double cost_new = 0;
    //     for(int j = 0;j< xSet.size();j++){  
    //         double x = xSet[j];
    //         double y = ySet[j];
    //         double e = y - exp(a*x*x + b*x + c);
    //         cost_new +=e*e;
    //     }
    //     rho = (cost - cost_new)/(delta_abc.transpose()*(u*delta_abc+g));

    //     //LM的工作
    //     if(rho >0 ){
    //         //注意初始化两个H和g,如果不是0会有很多奇怪的错误
    //         H = Matrix3d::Zero();
    //         g = Vector3d::Zero();
    //         J = Vector3d::Zero();
    //         //装填数据
    //         for(int j = 0;j< xSet.size();j++){  
    //             double x = xSet[j];
    //             double y = ySet[j];
    //             // cout<<"x" <<x<<endl;
    //             // cout<<"y" <<y<<endl;
    //             double e = y - exp(a*x*x + b*x + c);
    //             J[0] =  -exp(a*x*x + b*x + c)*x*x;
    //             J[1] = -exp(a*x*x + b*x + c)*x;
    //             J[2] = -exp(a*x*x + b*x + c)*1;

    //             Matrix3d tempH = J*J.transpose(); 
    //             H +=tempH;
    //             g += -J*e;
    //             cost +=e*e;
    //         }
    //         if(delta_abc.norm()<1e-12||cost<1e-12){   
    //             flag =1;
    //             break;
    //             }
    //         u = u*max(0.3333,(1-(2*rho-1)*(2*rho-1)*(2*rho-1)));
    //         v=2;
    //         }
    //         else{
    //             u = u * v;
    //             v = 2 * v;
    //         }
    //         cout<<"第"<<i<<"次:"<<endl;
    //         cout<<"a : "<<a<<endl;
    //         cout<<"b : "<<b<<endl;
    //         cout<<"c : "<<c<<endl;
    //         lastcost = cost;
    //         cout<<"delta_abc"<<delta_abc.norm()<<endl;
    //     }

    //     if(flag)
    //     {
    //             cout<<"已收敛,结果为:"<<endl;
    //             cout<<"final a : "<<a<<endl;
    //             cout<<"final b : "<<b<<endl;
    //             cout<<"final c : "<<c<<endl;
    //     }
    //     else
    //     {
    //         cout<<"发散了QAQ,最后一次结果"<<endl;
    //             cout<<"final a : "<<a<<endl;
    //             cout<<"final b : "<<b<<endl;
    //             cout<<"final c : "<<c<<endl;       
    //     }



}
void LM(const vector<double> &xSet ,const vector<double> &ySet ,double &a,double &b,double &c){
    bool flag = 0;
    double cost =0.0;
    double lastcost = 0.0;
    int maxtimes = 1000;
    double v = 2;
    double rho = 0;
    double tao = 1e-10;
    //获得初值
    Matrix3d H = Matrix3d::Zero();
    Vector3d g = Vector3d::Zero();
    Vector3d J;
    //装填数据
    for(int j = 0;j< xSet.size();j++){  
        double x = xSet[j];
        double y = ySet[j];
        // cout<<"x" <<x<<endl;
        // cout<<"y" <<y<<endl;
        double e = y - exp(a*x*x + b*x + c);
        J[0] =  -exp(a*x*x + b*x + c)*x*x;
        J[1] = -exp(a*x*x + b*x + c)*x;
        J[2] = -exp(a*x*x + b*x + c)*1;

        Matrix3d tempH = J*J.transpose(); 
        H +=tempH;
        g += -J*e;
        cost +=e*e;
    }
    //使用eigen
    double u = tao*H.maxCoeff();
    cout<<"init u :"<<u<<endl;
    cout<<"H init"<<H.matrix()<<endl;
    cout<<"J init"<<J.matrix()<<endl;
    cout<<"g init"<<g.matrix()<<endl;
    Matrix3d I = MatrixXd::Identity(3, 3);
    for(int i =0; i< maxtimes;i++){
        
        //使用eigen结算线性方程组
        Matrix3d A = H+u*I;
        Vector3d delta_abc = A.ldlt().solve(g);
        //cout<<"delta_abc"<<delta_abc<<endl;
        //Vector3d delta_abc = H.ldlt().solve(g);
        if(delta_abc.norm()<1e-12){   
            flag =1;
            break;
                }
        cout<<"delta_abc"<<delta_abc.transpose()<<endl;
        if(isnan(delta_abc[0])||isnan(delta_abc[1])||isnan(delta_abc[2])){   
            flag =0;
            break;
                }
        a += delta_abc[0];
        b += delta_abc[1];
        c += delta_abc[2];
        double cost_new = 0;
        for(int j = 0;j< xSet.size();j++){  
            double x = xSet[j];
            double y = ySet[j];
            double e = y - exp(a*x*x + b*x + c);
            cost_new +=e*e;
        }
        rho = (cost - cost_new)/(delta_abc.transpose()*(u*delta_abc+g));

        //LM的工作
        if(rho >0 ){
            //注意初始化两个H和g,如果不是0会有很多奇怪的错误
            H = Matrix3d::Zero();
            g = Vector3d::Zero();
            J = Vector3d::Zero();
            //装填数据
            for(int j = 0;j< xSet.size();j++){  
                double x = xSet[j];
                double y = ySet[j];
                // cout<<"x" <<x<<endl;
                // cout<<"y" <<y<<endl;
                double e = y - exp(a*x*x + b*x + c);
                J[0] =  -exp(a*x*x + b*x + c)*x*x;
                J[1] = -exp(a*x*x + b*x + c)*x;
                J[2] = -exp(a*x*x + b*x + c)*1;

                Matrix3d tempH = J*J.transpose(); 
                H +=tempH;
                g += -J*e;
                cost +=e*e;
            }
            if(delta_abc.norm()<1e-12||cost<1e-12){   
                flag =1;
                break;
                }
            u = u*max(0.3333,(1-(2*rho-1)*(2*rho-1)*(2*rho-1)));
            v=2;
            }
            else{
                u = u * v;
                v = 2 * v;
            }
            cout<<"第"<<i<<"次:"<<endl;
            cout<<"a : "<<a<<endl;
            cout<<"b : "<<b<<endl;
            cout<<"c : "<<c<<endl;
            lastcost = cost;
            cout<<"delta_abc"<<delta_abc.norm()<<endl;
        }

        if(flag)
        {
                cout<<"已收敛,结果为:"<<endl;
                cout<<"final a : "<<a<<endl;
                cout<<"final b : "<<b<<endl;
                cout<<"final c : "<<c<<endl;
        }
        else
        {
            cout<<"发散了QAQ,最后一次结果"<<endl;
                cout<<"final a : "<<a<<endl;
                cout<<"final b : "<<b<<endl;
                cout<<"final c : "<<c<<endl;       
        }



}

int main(){
    cout.precision(9);
    
    VecVector3d Pw{};
    VecVector2d p{};
    makePnpTestData(Pw ,p);
    
    //设置初始值
    Matrix<double,1,6> pose = Matrix<double,1,6>::Zero();
    //观测值

}