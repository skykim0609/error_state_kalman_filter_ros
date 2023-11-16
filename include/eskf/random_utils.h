//
// Created by skykim0609 on 23. 11. 15.
//

#ifndef ERROR_STATE_KALMAN_FILTER_ROS_RANDOM_UTILS_H
#define ERROR_STATE_KALMAN_FILTER_ROS_RANDOM_UTILS_H

#include <Eigen/Dense>
#include <random>

//Added by KGC
class RandomUtils{
public:
    template <int M>
    static Eigen::Matrix<double, M, 1> generateGaussianRV(const Eigen::Matrix<double, M, 1>& mean, const Eigen::Matrix<double, M, M>& covar){
        // Compute the eigenvalues and eigenvectors for the covariance matrix
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, M, M>> eigensolver(covar);
        Eigen::Matrix<double, M, M> sqrtCovar = eigensolver.eigenvectors() * eigensolver.eigenvalues().cwiseSqrt().asDiagonal();

        // Generate a random vector
        Eigen::Matrix<double, M, 1> randomVector(mean.size());
        for(int i = 0; i < mean.size(); ++i) {
            randomVector[i] = norm_pdf(gen);
        }

        // Transform the random vector to have the desired mean and covariance
        return mean + sqrtCovar * randomVector;
    }

private:
    inline static std::mt19937 gen = std::mt19937(std::random_device()());
    inline static std::normal_distribution<double> norm_pdf = std::normal_distribution<double>(0.0, 1.0);
};

#endif //ERROR_STATE_KALMAN_FILTER_ROS_RANDOM_UTILS_H
