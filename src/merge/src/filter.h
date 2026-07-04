#ifndef __FILTER_H_
#define __FILTER_H_
#include <cmath>  // 用于std::abs
#include "method_math.h"

namespace Ten
{

    /**
     * @brief XYZRPY数据滤波类（按阈值滤波：差值超阈值更新，否则保留上一次值）
     * 规则：
     * 1. 第一次输入直接作为滤波结果
     * 2. XYZ三个轴共用一个阈值，RPY三个角度共用一个阈值
     * 3. 每个分量单独判断：差值>阈值则更新，否则保留上一次滤波值
     */
    class XYZRPYFilter
    {
    public:
        /**
         * @brief 自定义阈值构造函数
         * @param xyz_threshold: XYZ坐标滤波阈值（单位：m）
         * @param rpy_threshold: RPY姿态滤波阈值（单位：rad）
         */
        XYZRPYFilter(double xyz_threshold = 0.005, double rpy_threshold = 0.002) 
            : _xyz_threshold(xyz_threshold),  // 默认XYZ阈值5mm（匹配你的需求）
              _rpy_threshold(rpy_threshold),  // 默认RPY阈值0.001弧度
              _is_initialized(false)
        {
        }

        /**
         * @brief 设置XYZ坐标的滤波阈值
         * @param threshold: 阈值（单位：m）
         */
        void setXYZThreshold(double threshold)
        {
            _xyz_threshold = threshold;
        }

        /**
         * @brief 设置RPY姿态的滤波阈值
         * @param threshold: 阈值（单位：rad）
         */
        void setRPYThreshold(double threshold)
        {
            _rpy_threshold = threshold;
        }

        /**
         * @brief 重置滤波器状态（清空上一次的滤波值，恢复初始状态）
         */
        void reset()
        {
            _is_initialized = false;
            _last_filtered = Ten::XYZRPY();  // 重置为默认值
        }

        /**
         * @brief 重载()运算符，执行滤波操作（核心接口）
         * @param input: 输入的原始XYZRPY数据
         * @return Ten::XYZRPY: 滤波后的XYZRPY数据
         */
        Ten::XYZRPY operator()(const Ten::XYZRPY& input)
        {
            Ten::XYZRPY output;
            if(input.XYZRPYisnan())
            {
                return output;
            }
            // 第一次输入：直接使用原始值作为滤波结果
            if (!_is_initialized)
            {
                output._xyz._x = input._xyz._x;
                output._xyz._y = input._xyz._y;
                output._xyz._z = input._xyz._z;
                output._rpy._roll = input._rpy._roll;
                output._rpy._pitch = input._rpy._pitch;
                output._rpy._yaw = input._rpy._yaw;

                _last_filtered = output;
                _is_initialized = true;
                return output;
            }

            // XYZ分量滤波：每个轴单独判断阈值
            output._xyz._x = (std::abs(input._xyz._x - _last_filtered._xyz._x) > _xyz_threshold) 
                             ? input._xyz._x : _last_filtered._xyz._x;
            output._xyz._y = (std::abs(input._xyz._y - _last_filtered._xyz._y) > _xyz_threshold) 
                             ? input._xyz._y : _last_filtered._xyz._y;
            output._xyz._z = (std::abs(input._xyz._z - _last_filtered._xyz._z) > _xyz_threshold) 
                             ? input._xyz._z : _last_filtered._xyz._z;

            // RPY分量滤波：每个角度单独判断阈值
            output._rpy._roll = (std::abs(input._rpy._roll - _last_filtered._rpy._roll) > _rpy_threshold) 
                                ? input._rpy._roll : _last_filtered._rpy._roll;
            output._rpy._pitch = (std::abs(input._rpy._pitch - _last_filtered._rpy._pitch) > _rpy_threshold) 
                                 ? input._rpy._pitch : _last_filtered._rpy._pitch;
            output._rpy._yaw = (std::abs(input._rpy._yaw - _last_filtered._rpy._yaw) > _rpy_threshold) 
                               ? input._rpy._yaw : _last_filtered._rpy._yaw;

            // 更新上一次的滤波结果（供下次判断使用）
            _last_filtered = output;

            return output;
        }

    private:
        double _xyz_threshold;          // XYZ坐标滤波阈值（单位：m）
        double _rpy_threshold;          // RPY姿态滤波阈值（单位：rad）
        Ten::XYZRPY _last_filtered;     // 上一次的滤波结果（类成员变量）
        bool _is_initialized;           // 初始化标志（是否有有效的上一次值）
    };
}

#endif
