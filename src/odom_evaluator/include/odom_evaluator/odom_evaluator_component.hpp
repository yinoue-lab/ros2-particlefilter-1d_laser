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

#define DROP_COUNT 100

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
        true_odom_mf_.subscribe(this, "true_odom");
        est_odom_mf_.subscribe(this, "est_odom");

        // ApproximateTime ポリシーで3トピックの時刻同期を行う（キューサイズは例として10）
        sync_.reset(new Sync(MySyncPolicy(10), true_odom_mf_, est_odom_mf_));
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
        if(drop_counter_ < DROP_COUNT){
            drop_counter_++;
            return;
        }

        // EKF の誤差計算（平面位置とyaw）
        double x_diff = true_odom->pose.pose.position.x - est_odom->pose.pose.position.x;
        double y_diff = true_odom->pose.pose.position.y - est_odom->pose.pose.position.y;
        double yaw_true = tf2::getYaw(true_odom->pose.pose.orientation);
        double yaw_ekf  = tf2::getYaw(est_odom->pose.pose.orientation);
        double yaw_diff = (angles::shortest_angular_distance(yaw_true, yaw_ekf)) * 180.0 / M_PI;

        // バッファに追加
        est_pos_diffs_.push_back(std::make_pair(x_diff, y_diff));
        est_yaw_diffs_.push_back(yaw_diff);

        std_msgs::msg::Float64MultiArray result_msg;
        result_msg.data.push_back(x_diff);
        result_msg.data.push_back(y_diff);
        result_msg.data.push_back(yaw_diff);
        result_pub_.publish(result_msg);
    }

    // バッファから平均と不偏分散を計算する関数
    std::pair<double, double> computeGaussianParameters(const std::vector<double>& data)
    {
        if (data.size() <= 1) {
            return {0.0, 0.0};
        }
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        double mean = sum / data.size();
        
        double sq_sum = 0.0;
        for (const auto &val : data) {
            sq_sum += (val - mean) * (val - mean);
        }
        double variance = sq_sum / (data.size() - 1);
        return {mean, variance};
    }

    // ノード終了時に蓄積した全誤差データから、ガウス分布のパラメータを算出・表示する
    void computeFinalMetrics()
    {
        double mean_x_diff = 0.0;
        double mean_y_diff = 0.0;
        std::vector<double> est_trans_errors;
        for (const auto &diff : est_pos_diffs_) {
            mean_x_diff += diff.first;            
            mean_y_diff += diff.second;
            est_trans_errors.push_back(std::sqrt(diff.first * diff.first + diff.second * diff.second));
        }

        double mean_yaw_diff = 0.0;
        std::vector<double> est_yaw_errors;
        for (const auto &diff : est_yaw_diffs_) {
            mean_yaw_diff += diff;
            est_yaw_errors.push_back(std::abs(diff));
        }

        mean_x_diff /= est_pos_diffs_.size();
        mean_y_diff /= est_pos_diffs_.size();
        mean_yaw_diff /= est_yaw_diffs_.size();
        auto [mean_est_trans, var_est_trans] = computeGaussianParameters(est_trans_errors);
        auto [mean_est_yaw,   var_est_yaw]   = computeGaussianParameters(est_yaw_errors);

        TAGGER(result_log_, "===== Final Evaluation Results =====");
        TAGGER(result_log_, "Mean X diff:  %e m", mean_x_diff);
        TAGGER(result_log_, "Mean Y diff:  %e m", mean_y_diff);
        TAGGER(result_log_, "Mean Yaw diff: %e deg", mean_yaw_diff);
        TAGGER(result_log_, "Trans Error: mean = %e m, variance = %e m^2", mean_est_trans, var_est_trans);
        TAGGER(result_log_, "Yaw Error:   mean = %e deg, variance = %e deg^2", mean_est_yaw, var_est_yaw);
    }

private:
    uint32_t drop_counter_ = 0;

    blackbox::Logger    result_log_;
    blackbox::PubRecord<std_msgs::msg::Float64MultiArray> result_pub_;

    // 誤差データを全て蓄積するためのバッファ
    std::vector<std::pair<double, double>> est_pos_diffs_;
    std::vector<double> est_yaw_diffs_;

    // 各トピックの同期購読
    message_filters::Subscriber<nav_msgs::msg::Odometry> true_odom_mf_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> est_odom_mf_;

    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        nav_msgs::msg::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
};