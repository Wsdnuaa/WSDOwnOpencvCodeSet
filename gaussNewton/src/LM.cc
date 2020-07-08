#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include"opencv2/opencv.hpp"
#include"opencv4/opencv2/opencv.hpp"
#include<iostream>
#include<vector>
using namespace Eigen;
using namespace std;
using namespace cv;
void makeTheTestNum(vector<double> &xSet ,vector<double> &ySet);
void gaussNewton(const vector<double> &xSet ,const vector<double> &ySet ,double &a,double &b,double &c);


void makeTheTestNum(vector<double> &xSet ,vector<double> &ySet){

    RNG rng;   
    double noise = rng.gaussian(1);
    //设定值
    double a = 2;
    double b = 1;
    double c = 1;
    for(int i = 0;i <100;i++)
    {   
        double x = i/100.0;//注意这个.0,不然出来全是0
        double fx = exp(a*x*x + b*x + c)+noise;
        xSet.push_back(x);
        ySet.push_back(fx);
    }
    cout<<xSet.size()<<endl;
    if(xSet.size() != ySet.size())
        cout<<"data is bad!"<<endl;

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
    //设置初始值
    double a = 3;
    double b = -5;
    double c = 2;
    //观测值
    vector<double> xSet ;
    vector<double> ySet;
    makeTheTestNum(xSet ,ySet);
    vector<double> xSet1(xSet);
    vector<double> ySet1(ySet);
    gaussNewton(xSet, ySet, a, b, c);
    a = 3;
    b = 0;
    c = 2;
    LM(xSet1, ySet1, a, b, c);
    return 0;
}