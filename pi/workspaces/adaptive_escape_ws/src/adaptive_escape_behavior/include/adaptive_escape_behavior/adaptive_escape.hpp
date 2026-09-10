#ifndef ADAPTIVE_ESCAPE_BEHAVIOR__ADAPTIVE_ESCAPE_HPP_
#define ADAPTIVE_ESCAPE_BEHAVIOR__ADAPTIVE_ESCAPE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "adaptive_escape_msgs/action/adaptive_escape.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_behaviors/timed_behavior.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace adaptive_escape_behavior{

using AdaptiveEscapeAction = adaptive_escape_msgs::action::AdaptiveEscape;

class AdaptiveEscape : public nav2_behaviors::TimedBehavior<AdaptiveEscapeAction>{
public:
  AdaptiveEscape() = default;
  ~AdaptiveEscape() override = default;

protected:
  using Goal = AdaptiveEscapeAction::Goal;
  using Result = AdaptiveEscapeAction::Result;
  using Feedback = AdaptiveEscapeAction::Feedback;

  // ==============================
  // 탈출 후보 종류
  // ==============================

  enum class ManeuverType{
    TRANSLATION,
    ROTATION
  };

  // ==============================
  // 하나의 탈출 후보
  // ==============================

  struct Candidate{
    std::string name;
    
    ManeuverType type{ManeuverType::TRANSLATION};

    // 실제로 보낼 속도 명령
    double linear_x{0.0};
    double angular_z{0.0};

    // 전진 / 후진 / arc일 때 목표 이동거리
    double target_distance{0.0};

    // 제자리 회전일 때 목표 회전각
    double target_angle{0.0};

    // 후보 평가 결과
    double score{std::numeric_limits<double>::infinity()};
    double max_cost{0.0};
    double mean_cost{0.0};

    bool valid{false};
  };

  // ==============================
  // TimedBehavior 기본 함수
  // ==============================
  void onConfigure() override;

  nav2_behaviors::ResultStatus onRun(const std::shared_ptr<const Goal> command) override;

  nav2_behaviors::ResultStatus onCycleUpdate() override;

  // Action이 끝날 때
  // 우리가 추가한 Result 필드 채우기
  void onActionCompletion(std::shared_ptr<Result> result) override{
    result->error_msg = result_error_msg_;
    result->selected_maneuver = selected_maneuver_;
  }

  // Local Costmap + Local Footprint 사용
  nav2_core::CostmapInfoType getResourceInfo() override{
    return nav2_core::CostmapInfoType::LOCAL;
  }

private:
  // ==============================
  // 후보 생성 / 평가
  // ==============================
  std::vector<Candidate> createCandidates() const;

  bool evaluateCandidate(Candidate & candidate, const geometry_msgs::msg::Pose2D & start_pose);

  double computeCandidateScore(const Candidate & candidate, const geometry_msgs::msg::Pose2D & end_pose, double max_cost) const;

  // ==============================
  // Trajectory simulation
  // ==============================
  geometry_msgs::msg::Pose2D simulateStep(const geometry_msgs::msg::Pose2D & pose, double linear_x, double angular_z, double dt) const;

  bool isTrajectorySafe(const Candidate & candidate, const geometry_msgs::msg::Pose2D & start_pose);

  // ==============================
  // Robot pose
  // ==============================
  bool getCurrentPose2D(geometry_msgs::msg::Pose2D & pose);

  bool transformGoalToLocal(const geometry_msgs::msg::PoseStamped & input_goal, geometry_msgs::msg::PoseStamped & local_goal);

  // ==============================
  // 실제 실행
  // ==============================
  void updateTravel(const geometry_msgs::msg::Pose2D & current_pose);

  bool isManeuverComplete() const;

  void publishSelectedCommand();

  // ==============================
  // Utility
  // ==============================
  static double normalizeAngle(double angle);

  static double distance2D(const geometry_msgs::msg::Pose2D & a, const geometry_msgs::msg::Pose2D & b);

  // ==============================
  // Action feedback
  // ==============================
  Feedback::SharedPtr feedback_{
    std::make_shared<Feedback>()};

  // ==============================
  // 현재 선택된 maneuver
  // ==============================
  std::vector<Candidate> candidates_;

  Candidate selected_candidate_;

  bool has_selected_candidate_{false};

  // ==============================
  // 실제 Robot 이동 추적
  // ==============================
  geometry_msgs::msg::Pose2D start_pose_;
  geometry_msgs::msg::Pose2D previous_pose_;

  double distance_traveled_{0.0};
  double angle_traveled_{0.0};


  // ==============================
  // Goal
  // ==============================
  geometry_msgs::msg::PoseStamped goal_local_;

  bool goal_available_{false};

  // ==============================
  // Result 정보
  // ==============================
  std::string selected_maneuver_;
  std::string result_error_msg_;

  double selected_score_{0.0};
  double selected_max_cost_{0.0};

  // ==============================
  // Timeout
  // ==============================
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;

  // ==============================
  // AdaptiveEscape Parameters
  // ==============================

  // 최대 탈출 이동거리
  double max_escape_distance_{0.30};

  // 전진 속도
  double forward_speed_{0.10};

  // 후진 속도
  double reverse_speed_{0.08};

  // 완만한 arc
  double gentle_angular_speed_{0.25};

  // 강한 arc
  double tight_angular_speed_{0.45};

  // 제자리 회전 속도
  double rotation_speed_{0.35};

  // 최대 제자리 회전각
  double max_rotation_{0.785398};  // 45 deg

  // 미래 trajectory 계산 간격
  double simulation_time_step_{0.05};

  // ==============================
  // Score weights
  // ==============================

  double clearance_weight_{3.0};

  double goal_progress_weight_{1.0};

  double reverse_penalty_{0.35};

  double rotation_penalty_{0.6};
};

}

#endif  // ADAPTIVE_ESCAPE_BEHAVIOR__ADAPTIVE_ESCAPE_HPP_