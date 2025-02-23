#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <blackbox/blackbox.hpp>

namespace tf2{
  void fromMsg(const geometry_msgs::msg::Quaternion & in, tf2::Quaternion & out){
    // w at the end in the constructor 
    out = tf2::Quaternion(in.x, in.y, in.z, in.w); 
  };
}

class OdomEvaluator : public blackbox::BlackBoxNode
{
public:
    OdomEvaluator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : OdomEvaluator("", options) {}
    OdomEvaluator(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "odom_evaluator", name_space, options, true)
    {
        result_log_.init(this, blackbox::log_type_t::INFO, "eval_odom");

        // message_filters の購読者を初期化（それぞれのトピック名に合わせる）
        true_odom_sub_.subscribe(this, "true_odom");
        est_odom_sub_.subscribe(this, "est_odom");

        // ApproximateTime ポリシーで3トピックの時刻同期を行う（キューサイズは例として10）
        sync_.reset(new Sync(MySyncPolicy(10), true_odom_sub_, est_odom_sub_));
        sync_->registerCallback(&OdomEvaluator::callback, this);

        result_pub_.init(this, "eval_mean_odom");
    }

    // ノード終了時に呼ばれるデストラクタで最終評価を行う
    ~OdomEvaluator() override
    {
        computeFinalMetrics();
    }

private:
    // 各コールバック呼び出し時に、最新の誤差を計算してバッファに蓄積
    void callback(const nav_msgs::msg::Odometry::ConstSharedPtr true_odom,
                    const nav_msgs::msg::Odometry::ConstSharedPtr est_odom)
    {
        // EKF の誤差計算（平面位置とyaw）
        double dx_ekf = true_odom->pose.pose.position.x - est_odom->pose.pose.position.x;
        double dy_ekf = true_odom->pose.pose.position.y - est_odom->pose.pose.position.y;
        double error_trans_ekf = std::sqrt(dx_ekf * dx_ekf + dy_ekf * dy_ekf);
        double yaw_true = tf2::getYaw(true_odom->pose.pose.orientation);
        double yaw_ekf  = tf2::getYaw(est_odom->pose.pose.orientation);
        double error_yaw_ekf = std::abs(angles::shortest_angular_distance(yaw_true, yaw_ekf));

        // バッファに追加
        est_trans_errors_.push_back(error_trans_ekf);
        est_yaw_errors_.push_back(error_yaw_ekf);

        std_msgs::msg::Float64MultiArray result_msg;
        result_msg.data.push_back(error_trans_ekf);
        result_msg.data.push_back(error_yaw_ekf);
        result_pub_.publish(result_msg);
    }

    // バッファから平均と不偏分散を計算する関数
    std::pair<double, double> computeGaussianParameters(const std::deque<double>& data)
    {
        if (data.empty()) {
        return {0.0, 0.0};
        }
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        double mean = sum / data.size();
        
        double sq_sum = 0.0;
        for (const auto &val : data) {
        sq_sum += (val - mean) * (val - mean);
        }
        double variance = (data.size() > 1) ? (sq_sum / (data.size() - 1)) : 0.0;
        return {mean, variance};
    }

    // ノード終了時に蓄積した全誤差データから、ガウス分布のパラメータを算出・表示する
    void computeFinalMetrics()
    {
        auto [mean_est_trans, var_est_trans] = computeGaussianParameters(est_trans_errors_);
        auto [mean_est_yaw,   var_est_yaw]   = computeGaussianParameters(est_yaw_errors_);

        TAGGER(result_log_, "===== Final Evaluation Results =====");
        TAGGER(result_log_, "Trans Error: mean = %e, variance = %e", mean_est_trans, var_est_trans);
        TAGGER(result_log_, "Yaw Error:   mean = %e, variance = %e", mean_est_yaw, var_est_yaw);
    }

private:
    blackbox::Logger    result_log_;
    blackbox::PubRecord<std_msgs::msg::Float64MultiArray> result_pub_;

      // 誤差データを全て蓄積するためのバッファ
    std::deque<double> est_trans_errors_;
    std::deque<double> est_yaw_errors_;

    // 各トピックの同期購読
    message_filters::Subscriber<nav_msgs::msg::Odometry> true_odom_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> est_odom_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        nav_msgs::msg::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
};