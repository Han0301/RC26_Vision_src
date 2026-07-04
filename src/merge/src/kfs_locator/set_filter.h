#ifndef __SET_FILTER_H_
#define __SET_FILTER_H_
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>

namespace Ten::kfs_locator {

// ============================================================================
// 1. MAD 离群拒绝 + EMA 指数平滑 级联滤波器
// ============================================================================
class MadEmaFilter {
    static constexpr int    N     = 10;      // MAD 窗口大小（7帧即可稳定中位数）
    static constexpr double K     = 2.5;    // 离群阈值 (3×MAD ≈ 3σ)
    static constexpr double ALPHA = 0.25;   // EMA 平滑系数 (0~1, 越小越平滑)

    std::deque<double> buf_;
    double prev_   = 0.0;
    bool   warmed_ = false;

    static double median_of(std::vector<double>& v) {
        size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + m, v.end());
        return v[m];
    }

public:
    MadEmaFilter() = default;

    double filter(double x) {
        buf_.push_back(x);
        if (buf_.size() > N) buf_.pop_front();

        // ── 冷启动 ──
        if (!warmed_ || buf_.size() < N) {
            prev_   = x;
            warmed_ = true;
            return x;
        }

        // ── 1. MAD 离群检测 ──
        std::vector<double> sorted(buf_.begin(), buf_.end());
        double med = median_of(sorted);

        std::vector<double> devs;
        devs.reserve(buf_.size());
        for (double v : buf_) devs.push_back(std::abs(v - med));
        double mad = median_of(devs) * 1.4826; // 尺度因子 ≈ σ

        double cleaned = (std::abs(x - med) > K * mad) ? med : x;

        // ── 2. EMA 平滑 ──
        double smoothed = ALPHA * cleaned + (1.0 - ALPHA) * prev_;
        prev_ = smoothed;
        return smoothed;
    }

    void reset() { buf_.clear(); prev_ = 0.0; warmed_ = false; }
};

// ============================================================================
// 2. 一维匀速卡尔曼滤波器（目标静止 → 极低过程噪声）
// ============================================================================
class KalmanFilter1D {
    // 状态: [position, velocity]
    double x_ = 0.0;  // position
    double v_ = 0.0;  // velocity
    double p11_ = 1.0, p12_ = 0.0, p22_ = 1.0; // 协方差矩阵

    double Q_pos_ = 1e-6;   // 位置过程噪声（极小 → 认为静止）
    double Q_vel_ = 1e-8;   // 速度过程噪声
    double R_      = 0.01;   // 测量噪声（根据 σ 调整）
    double gate_   = 4.0;    // 新息门控阈值 (4σ ≈ 99.99% 正常值通过)
    bool   init_   = false;

public:
    KalmanFilter1D() = default;

    /// @param meas_noise 测量噪声方差 (约 = σ²)
    void set_measurement_noise(double meas_noise) { R_ = meas_noise; }
    /// @param gate 新息门控阈值 (默认4.0, 越小越严格, 2.5~5.0)
    void set_gate(double gate) { gate_ = gate; }

    double filter(double z) {
        if (!init_) {
            x_    = z;
            v_    = 0.0;
            init_ = true;
            return x_;
        }

        // ── 预测 ──
        double x_pred = x_ + v_;
        double v_pred = v_;
        double p11_pred = p11_ + p12_ + p12_ + p22_ + Q_pos_;
        double p12_pred = p12_ + p22_;
        double p22_pred = p22_ + Q_vel_;

        // ── 新息门控：尖峰检测 ──
        double innovation = z - x_pred;
        double S = p11_pred + R_;  // 新息协方差
        if (std::abs(innovation) > gate_ * std::sqrt(S)) {
            // 判定为离群尖峰 → 跳过更新，仅预测
            x_   = x_pred;
            v_   = v_pred;
            p11_ = p11_pred;
            p12_ = p12_pred;
            p22_ = p22_pred;
            return x_pred;
        }

        // ── 正常更新 ──
        double K1 = p11_pred / S;   // 卡尔曼增益 (position)
        double K2 = p12_pred / S;   // 卡尔曼增益 (velocity)

        x_   = x_pred + K1 * innovation;
        v_   = v_pred + K2 * innovation;
        p11_ = (1.0 - K1) * p11_pred;
        p12_ = (1.0 - K1) * p12_pred;
        p22_ = p22_pred - K2 * p12_pred;

        return x_;
    }

    void reset() { x_ = v_ = 0.0; p11_ = p22_ = 1.0; p12_ = 0.0; init_ = false; }
};

} // namespace Ten::kfs_locator
#endif
