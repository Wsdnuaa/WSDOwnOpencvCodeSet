#include "ceres/ceres.h"
#include <opencv2/opencv.hpp>
using namespace std;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
class AnalyticCostFunctor : public ceres::SizedCostFunction<1,2> {
   public:
     AnalyticCostFunctor(const double x, const double y) : x_(x), y_(y) {}
     virtual ~AnalyticCostFunctor() {}
     virtual bool Evaluate(double const* const* parameters,
                           double* residuals,
                           double** jacobians) const {
       const double a = parameters[0][0];
       const double b = parameters[0][1];

       residuals[0] = (a*x_+b)-y_;

       if (!jacobians) return true;
       double* jacobian = jacobians[0];
       if (!jacobian) return true;

       jacobian[0] = x_;
       jacobian[1] = 1.0;
       return true;
     }

   private:
     const double x_;
     const double y_;
 };


int main(int argc, char** argv)
{

  double a=1.0, b=2.0;         // 真实参数值
  int N=100;                          // 数据点
  double w_sigma=0.1;                 // 噪声Sigma值
  cv::RNG rng;                        // OpenCV随机数产生器
  double ab[2] = {0,0};            // abc参数的估计值

  vector<double> x_data, y_data;      // 数据
  cout<<"generating data: "<<endl;
  for ( int i=0; i<N; i++ )
  {
      double x = i/100.0;
      x_data.push_back ( x );
      y_data.push_back (a*x + b + rng.gaussian ( w_sigma ));
      //cout<<x_data[i]<<" "<<y_data[i]<<endl;
  }
  // Build the problem.
  Problem problem;
  for ( int i=0; i<N; i++ )
 {
      CostFunction* cost_function =new AnalyticCostFunctor( x_data[i], y_data[i] );
      problem.AddResidualBlock(cost_function, NULL, ab);
  }

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "output a: " << ab[0]
            << "output b: " << ab[1] << "\n";
  return 0;
}
