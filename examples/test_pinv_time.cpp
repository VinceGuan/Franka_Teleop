//
// Created by yuan on 27/07/2021.
//
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <array>
#include <cmath>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <vector>

Eigen::MatrixXd pinv_Eigen_SVD(Eigen::MatrixXd &origin) {

  const float er = 0;
  // perform svd decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Build SVD decomposition results
  Eigen::MatrixXd U = svd_holder.matrixU();
  Eigen::MatrixXd V = svd_holder.matrixV();
  Eigen::MatrixXd D = svd_holder.singularValues();

  // Build the S matrix
  Eigen::MatrixXd S(V.cols(), U.cols());
  S.setZero();

  for (unsigned int i = 0; i < D.size(); ++i) {

    if (D(i, 0) > er) {
      S(i, i) = 1 / D(i, 0);
    } else {
      S(i, i) = 0;
    }
  }

  // pinv_matrix = V * S * U^T
  return V * S * U.transpose();
}

int main()
{
  std::vector<int> values(10000);

  // Generate Random values
  auto f = []() -> int { return rand() % 10000; };

  // Fill up the vector
  generate(values.begin(), values.end(), f);

  // Get starting timepoint
  auto start = std::chrono::high_resolution_clock::now();

  // Call the function, here sort()
  Eigen::MatrixXd jacobian;

//  for ( int i(0); i<1; i++ ) {
    jacobian.setRandom(6,7);
//    Eigen::MatrixXd pinv_jacobian1 = pinv_Eigen_SVD(jacobian);
//    Eigen::MatrixXd pinv_jacobian2 = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd pinv_jacobian3 = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
//  }

  // Get ending timepoint
  auto stop = std::chrono::high_resolution_clock::now();

//  std::cout << pinv_jacobian1 << std::endl << std::endl;
//  std::cout << pinv_jacobian2 << std::endl << std::endl;
//  std::cout << pinv_jacobian3 << std::endl << std::endl;

  // Get duration. Substart timepoints to
  // get durarion. To cast it to proper unit
  // use duration cast method
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Time taken by function: "
       << duration.count() << " microseconds" << std::endl;

  return 0;
}