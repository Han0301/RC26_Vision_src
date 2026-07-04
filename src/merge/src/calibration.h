#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_
#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>


namespace Ten
{


    class Ten_calibration
    {
    public:
        Ten_calibration(const Ten_calibration& calib) = delete;
        Ten_calibration& operator=(const Ten_calibration& calib) = delete;
        Ten_calibration(){}
        
        /**
         * @brief 设置雷达和相机自己的变换矩阵
         * @param lidar ：4×4的变换矩阵
         * @param camera ：4×4的变换矩阵
         */
        void set_lidar_and_camera_T(Eigen::Matrix4d lidar, Eigen::Matrix4d camera)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            lidars_T_.push_back(lidar);
            cameras_T_.push_back(camera);
        }

        /**
         * @brief 获取雷达到相机的变换
         * @return Eigen::Matrix4d: 变换矩阵
         */
        Eigen::Matrix4d get_calibration()
        {
            double error = solveSimilarityMatrixC(lidars_T_, cameras_T_, lidar_to_camera_);
            if(error > 0)
            {
                return lidar_to_camera_;
            }
            else 
            {
                return Eigen::Matrix4d::Identity();
            }
        }


    private:
    std::vector<Eigen::Matrix4d> lidars_T_;
    std::vector<Eigen::Matrix4d> cameras_T_;
    Eigen::Matrix4d lidar_to_camera_ = Eigen::Matrix4d::Identity();
    mutable std::mutex mtx_;

    /**
     * @brief 求解相似变换 C*A*C^{-1}=B 的最优矩阵C (4×4)，多组A/B约束同一个C
     * @param A_vec 输入参数：多组A矩阵的数组，每个元素是4×4的A矩阵
     * @param B_vec 输入参数：多组B矩阵的数组，与A_vec一一对应
     * @param C_opt 输出参数：求解得到的最优4×4矩阵C
     * @return 拟合误差：所有组的平均残差||C*A - B*C||_F，越小越优；失败返回-1.0
     */
    double solveSimilarityMatrixC(const std::vector<Eigen::Matrix4d>& A_vec,
                                const std::vector<Eigen::Matrix4d>& B_vec,
                                Eigen::Matrix4d& C_opt)
    {
        // 1. 入参合法性校验
        if (A_vec.empty() || B_vec.empty())
        {
            ROS_ERROR("solveSimilarityMatrixC: 输入的A/B矩阵数组为空！");
            return -1.0;
        }
        if (A_vec.size() != B_vec.size())
        {
            ROS_ERROR("solveSimilarityMatrixC: A矩阵数量与B矩阵数量不相等！");
            return -1.0;
        }
        const int mat_size = 4;
        const int vec_size = mat_size * mat_size;

        // 2. 构造超定线性方程组 M * x = 0 （这部分逻辑完全不变）
        Eigen::MatrixXd M(0, vec_size);
        const size_t pair_num = A_vec.size();
        for (size_t idx = 0; idx < pair_num; ++idx)
        {
            const Eigen::Matrix4d& A = A_vec[idx];
            const Eigen::Matrix4d& B = B_vec[idx];

            Eigen::MatrixXd mat_constraint(vec_size, vec_size);
            mat_constraint.setZero();
            for (int i = 0; i < mat_size; ++i)
            {
                for (int j = 0; j < mat_size; ++j)
                {
                    int row_idx = i * mat_size + j;
                    for (int k = 0; k < mat_size; ++k)
                    {
                        mat_constraint(row_idx, i * mat_size + k) += A(k, j);
                        mat_constraint(row_idx, k * mat_size + j) -= B(i, k);
                    }
                }
            }
            M.conservativeResize(M.rows() + vec_size, vec_size);
            M.bottomRows(vec_size) = mat_constraint;
        }

        // ======================================
        // 核心修复部分【Eigen3.3.7兼容】 开始
        // ======================================
        Eigen::MatrixXd MtM = M.transpose() * M;
        // 特征值分解：求解对称矩阵的特征值和特征向量，Eigen3.3.7完美支持，无兼容问题
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig_solver(MtM);
        if (eig_solver.info() != Eigen::Success)
        {
            ROS_ERROR("solveSimilarityMatrixC: 特征值分解失败，无解！");
            return -1.0;
        }
        // 特征值从小到大排序，取最小特征值对应的特征向量 = 最优解
        Eigen::VectorXd eigen_values = eig_solver.eigenvalues();
        Eigen::MatrixXd eigen_vectors = eig_solver.eigenvectors();
        int min_eig_idx = 0;
        for (int i = 1; i < eigen_values.size(); ++i)
        {
            if (eigen_values(i) < eigen_values(min_eig_idx))
            {
                min_eig_idx = i;
            }
        }
        Eigen::VectorXd c_vec = eigen_vectors.col(min_eig_idx);
        // ======================================
        // 核心修复部分 结束
        // ======================================

        // 4. 归一化+向量重组为4×4矩阵（逻辑不变）
        double norm = c_vec.norm();
        if (std::fabs(norm) < 1e-8)
        {
            ROS_ERROR("solveSimilarityMatrixC: 求解得到零矩阵，无效解！");
            return -1.0;
        }
        c_vec /= norm;

        Eigen::Map<Eigen::Matrix4d> C_temp(c_vec.data(), mat_size, mat_size);
        C_opt = C_temp;

        // 5. 计算平均拟合误差（逻辑不变）
        double total_error = 0.0;
        for (size_t idx = 0; idx < pair_num; ++idx)
        {
            const Eigen::Matrix4d& A = A_vec[idx];
            const Eigen::Matrix4d& B = B_vec[idx];
            Eigen::Matrix4d error_mat = C_opt * A - B * C_opt;
            total_error += error_mat.norm();
        }
        double avg_error = total_error / pair_num;

        return avg_error;
    }


    };

    extern Ten::Ten_calibration _CALCULATE_LIDAR_TO_CAMERA_TRANSFORM_;

}









#endif


