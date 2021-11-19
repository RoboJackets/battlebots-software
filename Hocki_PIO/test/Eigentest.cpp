#include <iostream>
#include <../eigen/Eigen.h>


using namespace std;
 
int main()
{
  Eigen::MatrixXf m(2, 2); //initializes a 2x2 dynamic matrix m that takes floats
  m(0,0) = 2.4; //sets values for matrix m
  m(0,1) = 3.1;
  m(1,0) = 1.0;
  m(1,1) = 0.2;

  Eigen::MatrixXf n(2, 2); //initializes 2x2 dynamic matrix n that takes floats
  n(0,0) = 1.3; //sets values for matrix n
  n(0,1) = 4.5;
  n(1,0) = 2.7;
  n(1,1) = 6.2;

  std::cout << "Here is matrix m * n\n" <<m*n<<std::endl;
  std::cout << "Here is the inverse of m\n" <<m.inverse()<<std::endl;

}
