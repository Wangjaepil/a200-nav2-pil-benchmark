#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "adaptive_escape_behavior/adaptive_escape.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"


namespace adaptive_escape_behavior{

// ============================================================
// Behavior Server가 AdaptiveEscape plugin을 configure할 때 1회 실행
//
// AdaptiveEscape에서 사용할 파라미터를 선언하고 실제 값을 읽는다.
// YAML에 값이 없으면 header에 정의한 기본값을 사용한다.
// ============================================================

void AdaptiveEscape::onConfigure(){
    auto node = node_.lock();

    if(!node){
        throw std::runtime_error("AdaptiveEscape: failed to lock behavior server node");
    }

    // behavior_name_은 YAML에서 등록한 이름인
    // "adaptive_escape"가 된다.
    //
    // 따라서 실제 parameter 이름은 예를 들어:
    //
    // adaptive_escape.max_escape_distance
    // adaptive_escape.forward_speed
    //
    // 와 같은 형태가 된다.
    const std::string prefix = behavior_name_ + ".";

    // 같은 형태의 double parameter 선언이 많기 때문에
    // 반복 코드를 줄이기 위한 작은 lambda 함수
    auto declare_double = [&](const std::string & name, double default_value){
        nav2_util::declare_parameter_if_not_declared(node, prefix + name, rclcpp::ParameterValue(default_value));
    };

    // ========================================================
    // Motion parameter 선언
    // ========================================================
    declare_double("max_escape_distance", max_escape_distance_);
    declare_double("forward_speed", forward_speed_);
    declare_double("reverse_speed", reverse_speed_);
    declare_double("gentle_angular_speed", gentle_angular_speed_);
    declare_double("tight_angular_speed", tight_angular_speed_);
    declare_double("rotation_speed", rotation_speed_);
    declare_double("max_rotation", max_rotation_);
    declare_double("simulation_time_step", simulation_time_step_);

    // ========================================================
    // Candidate score parameter 선언
    // ========================================================
    declare_double("clearance_weight", clearance_weight_);
    declare_double("goal_progress_weight", goal_progress_weight_);
    declare_double("reverse_penalty", reverse_penalty_);
    declare_double("rotation_penalty", rotation_penalty_);

    // ========================================================
    // 실제 parameter 값 읽기
    // ========================================================
    node->get_parameter(prefix + "max_escape_distance", max_escape_distance_);
    node->get_parameter(prefix + "forward_speed", forward_speed_);
    node->get_parameter(prefix + "reverse_speed", reverse_speed_);
    node->get_parameter(prefix + "gentle_angular_speed", gentle_angular_speed_);
    node->get_parameter(prefix + "tight_angular_speed", tight_angular_speed_);
    node->get_parameter(prefix + "rotation_speed", rotation_speed_);
    node->get_parameter(prefix + "max_rotation", max_rotation_);
    node->get_parameter(prefix + "simulation_time_step", simulation_time_step_);

    node->get_parameter(prefix + "clearance_weight", clearance_weight_);
    node->get_parameter(prefix + "goal_progress_weight", goal_progress_weight_);
    node->get_parameter(prefix + "reverse_penalty", reverse_penalty_);
    node->get_parameter(prefix + "rotation_penalty", rotation_penalty_);

    // ========================================================
    // 잘못된 Motion parameter 차단
    //
    // 속도나 시간이 0 또는 음수가 되면
    // trajectory simulation 자체가 성립하지 않으므로
    // configure 단계에서 바로 막는다.
    // ========================================================

    if(max_escape_distance_ <= 0.0 || forward_speed_ <= 0.0 || reverse_speed_ <= 0.0 || gentle_angular_speed_ <= 0.0 || 
        tight_angular_speed_ <= 0.0 || rotation_speed_ <= 0.0 || max_rotation_ <= 0.0 || simulation_time_step_ <= 0.0){

        throw std::runtime_error("AdaptiveEscape: motion parameters must be positive");
    }

    // Score 관련 값은 0은 허용하지만 음수는 허용하지 않는다.
    if(clearance_weight_ < 0.0 || goal_progress_weight_ < 0.0 || reverse_penalty_ < 0.0 || rotation_penalty_ < 0.0){
        throw std::runtime_error("AdaptiveEscape: scoring parameters must not be negative");
    }

    RCLCPP_INFO(logger_, "AdaptiveEscape configured: distance=%.2f, forward=%.2f, reverse=%.2f, max_rotation=%.2f", max_escape_distance_, forward_speed_, reverse_speed_, max_rotation_);
}

// ============================================================
// AdaptiveEscape Action이 처음 시작될 때 딱 1회 실행
//
// 여기서 실제 이동은 하지 않는다.
//
// 1. 현재 Robot pose 확인
// 2. Goal을 local frame(odom)으로 변환
// 3. Local Costmap + Footprint 확보
// 4. 12개의 escape candidate 생성
// 5. 모든 candidate를 simulation
// 6. 가장 좋은 candidate 선택
//
// 여기까지 성공하면 onCycleUpdate()에서 실제 이동을 시작한다.
// ============================================================

nav2_behaviors::ResultStatus AdaptiveEscape::onRun(const std::shared_ptr<const Goal> command){

    // ========================================================
    // 이전 Recovery 실행 상태 초기화
    // ========================================================

    candidates_.clear();

    has_selected_candidate_ = false;

    selected_maneuver_.clear();
    result_error_msg_.clear();

    selected_score_ = 0.0;
    selected_max_cost_ = 0.0;

    distance_traveled_ = 0.0;
    angle_traveled_ = 0.0;

    goal_available_ = false;

    feedback_ = std::make_shared<Feedback>();

    // ========================================================
    // 최대 실행시간 설정
    //
    // 공식 Spin behavior도 command의 time_allowance를
    // rclcpp::Duration으로 받아 같은 방식으로 사용한다.
    // ========================================================

    command_time_allowance_ = command->time_allowance;

    end_time_ = clock_->now() + command_time_allowance_;

    // ========================================================
    // 현재 Robot pose 획득
    //
    // local_frame_ = odom
    // robot_base_frame_ = base_link
    //
    // 즉 odom 기준 base_link의 현재 위치를 얻는다.
    // ========================================================

    if(!getCurrentPose2D(start_pose_)){
        result_error_msg_ = "Current robot pose is unavailable";

        RCLCPP_ERROR(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::TF_ERROR};
    }

    previous_pose_ = start_pose_;

    // ========================================================
    // Navigation Goal을 local_frame으로 변환
    //
    // Navigation goal은 일반적으로 map frame이고,
    // AdaptiveEscape trajectory는 odom frame에서 계산한다.
    //
    // frame이 다른 상태에서 x/y를 직접 비교하면 안 된다.
    // ========================================================

    if(!transformGoalToLocal(command->goal, goal_local_)){
        result_error_msg_ = "Failed to transform navigation goal into local frame";

        RCLCPP_ERROR(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::TF_ERROR};
    }

    goal_available_ = true;

    // ========================================================
    // Local collision checker 존재 확인
    // ========================================================

    if(!local_collision_checker_){
        result_error_msg_ = "Local collision checker is unavailable";

        RCLCPP_ERROR(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::NO_ESCAPE_FOUND};
    }

    // ========================================================
    // Local Costmap + Footprint snapshot 확보
    //
    // scorePose(start_pose_, true)
    //
    // true:
    // 최신 Local Costmap과 Footprint를 가져온다.
    //
    // 이후 12개 후보 평가에서는 false를 사용해서
    // 모두 같은 snapshot을 이용한다.
    //
    // 이렇게 해야 candidate A와 candidate B가 서로 다른
    // Costmap 시점에서 평가되는 문제를 막을 수 있다.
    // ========================================================

    try{
        const double start_cost = local_collision_checker_->scorePose(start_pose_, true);

        // 254 = LETHAL_OBSTACLE
        // 255 = NO_INFORMATION
        //
        // 둘 다 254 이상이므로 이동을 시작하지 않는다.
        if(start_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)){
            result_error_msg_ = "Current robot footprint is already in collision";

            RCLCPP_WARN(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

            return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::COLLISION};
        }
    }
    catch(const std::exception & e){
        result_error_msg_ = std::string("Failed to obtain local costmap/footprint: ") + e.what();

        RCLCPP_ERROR(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::NO_ESCAPE_FOUND};
    }

    // ========================================================
    // 12개의 Escape Candidate 생성
    // ========================================================

    candidates_ = createCandidates();

    double best_score = std::numeric_limits<double>::infinity();

    // ========================================================
    // 모든 Candidate 평가
    // ========================================================

    for(auto & candidate : candidates_){

        // 충돌하거나 정상적인 trajectory가 아니면 제외
        if(!evaluateCandidate(candidate, start_pose_)){
            RCLCPP_DEBUG(logger_, "AdaptiveEscape candidate rejected: %s", candidate.name.c_str());

            continue;
        }

        RCLCPP_DEBUG(logger_, "AdaptiveEscape candidate [%s]: score=%.3f, max_cost=%.1f, mean_cost=%.1f", candidate.name.c_str(), candidate.score, candidate.max_cost, candidate.mean_cost);

        // Score가 낮을수록 좋은 후보
        //
        // 같은 score라면 먼저 생성된 후보를 유지한다.
        // 후보 생성 순서가 Forward → Rotation → Reverse이므로
        // 완전히 동일한 조건에서는 자연스럽게 Forward가 우선된다.
        if(candidate.score < best_score){
            best_score = candidate.score;

            selected_candidate_ = candidate;

            has_selected_candidate_ = true;
        }
    }

    // ========================================================
    // 살아남은 후보가 하나도 없으면 Recovery 불가능
    // ========================================================

    if(!has_selected_candidate_){
        result_error_msg_ = "No collision-free escape maneuver was found";

        RCLCPP_WARN(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::NO_ESCAPE_FOUND};
    }

    // ========================================================
    // 최종 선택 Candidate 저장
    // ========================================================

    selected_maneuver_ = selected_candidate_.name;
    selected_score_ = selected_candidate_.score;
    selected_max_cost_ = selected_candidate_.max_cost;

    feedback_->selected_maneuver = selected_maneuver_;
    feedback_->distance_traveled = 0.0F;
    feedback_->angle_traveled = 0.0F;
    feedback_->score = static_cast<float>(selected_score_);

    RCLCPP_INFO(logger_, "AdaptiveEscape selected [%s]: score=%.3f, max_cost=%.1f", selected_maneuver_.c_str(), selected_score_, selected_max_cost_);

    // onRun()에서 SUCCEEDED는
    // AdaptiveEscape 전체가 완료됐다는 의미가 아니다.
    //
    // TimedBehavior에게:
    //
    // "초기 검사 성공. 이제 onCycleUpdate()를 반복 실행해도 된다."
    //
    // 라는 의미다.
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED, Result::NONE};
}

// ============================================================
// AdaptiveEscape 실행 중 Behavior Server 주기마다 반복 호출
//
// 현재 Behavior Server cycle_frequency = 10 Hz이므로
// 대략 0.1초마다 실행된다.
//
// 1. Timeout 확인
// 2. 현재 실제 Pose 확인
// 3. 실제 이동거리 계산
// 4. 남은 trajectory 안전성 재검사
// 5. 완료 확인
// 6. cmd_vel 전송
// ============================================================

nav2_behaviors::ResultStatus AdaptiveEscape::onCycleUpdate(){

    // 선택된 후보가 없다는 것은 정상적인 상태가 아니다.
    if(!has_selected_candidate_){
        stopRobot();

        result_error_msg_ = "No selected escape maneuver";

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::NO_ESCAPE_FOUND};
    }

    // ========================================================
    // Timeout 확인
    //
    // time_allowance <= 0이면 제한시간 없는 것으로 처리
    // ========================================================

    if(command_time_allowance_.seconds() > 0.0 && clock_->now() > end_time_){
        stopRobot();

        result_error_msg_ = "AdaptiveEscape exceeded time allowance";

        RCLCPP_WARN(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::TIMEOUT};
    }

    // ========================================================
    // 현재 실제 Robot pose 획득
    // ========================================================

    geometry_msgs::msg::Pose2D current_pose;

    if(!getCurrentPose2D(current_pose)){
        stopRobot();

        result_error_msg_ = "Current robot pose became unavailable";

        RCLCPP_ERROR(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::TF_ERROR};
    }

    // ========================================================
    // 실제 이동거리 / 회전량 갱신
    //
    // 단순히 시간 × command velocity로 계산하지 않는다.
    //
    // Collision Monitor가 속도를 줄이거나,
    // 실제 robot이 예상보다 덜 움직일 수 있기 때문에
    // TF에서 실제 이동량을 계산한다.
    // ========================================================

    updateTravel(current_pose);

    // ========================================================
    // 아직 남은 trajectory만 계산
    // ========================================================

    Candidate remaining = selected_candidate_;

    if(remaining.type == ManeuverType::TRANSLATION){
        remaining.target_distance = std::max(0.0, remaining.target_distance - distance_traveled_);
    }
    else{
        remaining.target_angle = std::max(0.0, remaining.target_angle - angle_traveled_);
    }

    // ========================================================
    // 최신 Local Costmap으로 현재 pose + 남은 trajectory 재검사
    //
    // IMPORTANT:
    //
    // 완료 판정보다 이것을 먼저 한다.
    //
    // 목표 거리를 막 도달했더라도 현재 footprint가
    // 충돌 상태라면 SUCCESS로 끝내면 안 되기 때문이다.
    // ========================================================

    if(!isTrajectorySafe(remaining, current_pose)){
        stopRobot();

        result_error_msg_ = "Escape trajectory became unsafe during execution";

        RCLCPP_WARN(logger_, "AdaptiveEscape: %s", result_error_msg_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, Result::COLLISION};
    }

    // ========================================================
    // Feedback 갱신
    // ========================================================

    feedback_->selected_maneuver = selected_maneuver_;
    feedback_->distance_traveled = static_cast<float>(distance_traveled_);
    feedback_->angle_traveled = static_cast<float>(angle_traveled_);
    feedback_->score = static_cast<float>(selected_score_);

    action_server_->publish_feedback(feedback_);

    // ========================================================
    // 실제 목표 거리 / 각도 도달 확인
    // ========================================================

    if(isManeuverComplete()){
        stopRobot();

        result_error_msg_.clear();

        RCLCPP_INFO(logger_, "AdaptiveEscape maneuver [%s] completed", selected_maneuver_.c_str());

        return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED, Result::NONE};
    }

    // ========================================================
    // 아직 완료되지 않았고 앞으로도 안전하다면
    // 선택된 command를 실제로 publish
    // ========================================================

    publishSelectedCommand();

    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::RUNNING, Result::NONE};
}


// ============================================================
// Escape Candidate 생성
//
// 총 12개:
//
// 5 Forward
// 2 Rotation
// 5 Reverse
//
// 처음부터 수백 개를 sampling하지 않는 이유:
// Recovery는 최적제어 문제가 아니라
// 빠르게 bad pose를 벗어나는 것이 목적이기 때문이다.
// ============================================================

std::vector<AdaptiveEscape::Candidate> AdaptiveEscape::createCandidates() const{
    std::vector<Candidate> candidates;

    // ========================================================
    // Translation / Arc Candidate 생성 helper
    // ========================================================

    auto add_translation = [&](const std::string & name, double linear_x, double angular_z){

        if(std::abs(linear_x) < 1e-6){
            return;
        }

        Candidate candidate;

        candidate.name = name;
        candidate.type = ManeuverType::TRANSLATION;

        candidate.linear_x = linear_x;
        candidate.angular_z = angular_z;

        // 기본 탈출 거리는 30 cm
        candidate.target_distance = max_escape_distance_;

        // Arc의 경우 너무 크게 돌아버리지 않도록
        // 최대 회전각 max_rotation_으로 이동거리를 제한한다.
        //
        // arc length:
        //
        // s = R * theta
        //
        // R = |v / w|
        //
        // 따라서:
        //
        // s = theta * |v / w|
        if(std::abs(angular_z) > 1e-6){
            const double distance_for_max_rotation = max_rotation_ * std::abs(linear_x / angular_z);

            candidate.target_distance = std::min(max_escape_distance_, distance_for_max_rotation);

            candidate.target_angle = std::abs(angular_z / linear_x) * candidate.target_distance;
        }

        candidates.push_back(candidate);
    };

    // ========================================================
    // 제자리 Rotation Candidate 생성 helper
    // ========================================================

    auto add_rotation = [&](const std::string & name, double angular_z){
        Candidate candidate;

        candidate.name = name;
        candidate.type = ManeuverType::ROTATION;

        candidate.linear_x = 0.0;
        candidate.angular_z = angular_z;

        candidate.target_distance = 0.0;
        candidate.target_angle = max_rotation_;

        candidates.push_back(candidate);
    };

    // ========================================================
    // Forward 후보
    // ========================================================

    add_translation("forward", forward_speed_, 0.0);
    add_translation("forward_left_gentle", forward_speed_, gentle_angular_speed_);
    add_translation("forward_right_gentle", forward_speed_, -gentle_angular_speed_);
    add_translation("forward_left_tight", forward_speed_, tight_angular_speed_);
    add_translation("forward_right_tight", forward_speed_, -tight_angular_speed_);

    // ========================================================
    // Rotation 후보
    // ========================================================

    add_rotation("rotate_left", rotation_speed_);
    add_rotation("rotate_right", -rotation_speed_);

    // ========================================================
    // Reverse 후보
    // ========================================================

    add_translation("reverse", -reverse_speed_, 0.0);
    add_translation("reverse_left_gentle", -reverse_speed_, gentle_angular_speed_);
    add_translation("reverse_right_gentle", -reverse_speed_, -gentle_angular_speed_);
    add_translation("reverse_left_tight", -reverse_speed_, tight_angular_speed_);
    add_translation( "reverse_right_tight",-reverse_speed_,-tight_angular_speed_);

    return candidates;
}

// ============================================================
// Candidate 전체 trajectory 평가
//
// 한 Candidate에 대해:
//
// 1. 실행시간 계산
// 2. 0.05초 간격으로 미래 pose 생성
// 3. 각 pose의 실제 footprint cost 계산
// 4. LETHAL / UNKNOWN 만나면 후보 폐기
// 5. max cost / mean cost 계산
// 6. 최종 score 계산
//
// 초기 onRun()에서는 모든 Candidate가 동일한 Costmap snapshot을
// 사용하도록 scorePose(..., false)를 사용한다.
// ============================================================

bool AdaptiveEscape::evaluateCandidate(Candidate & candidate, const geometry_msgs::msg::Pose2D & start_pose){

    candidate.valid = false;
    candidate.score = std::numeric_limits<double>::infinity();

    candidate.max_cost = 0.0;
    candidate.mean_cost = 0.0;

    double duration = 0.0;

    // ========================================================
    // 예상 실행시간 계산
    // ========================================================

    if(candidate.type == ManeuverType::TRANSLATION){

        if(std::abs(candidate.linear_x) < 1e-6){
            return false;
        }

        duration = candidate.target_distance / std::abs(candidate.linear_x);
    }
    else{

        if(std::abs(candidate.angular_z) < 1e-6){
            return false;
        }

        duration =candidate.target_angle / std::abs(candidate.angular_z);
    }

    if(duration <= 0.0){
        return false;
    }

    geometry_msgs::msg::Pose2D pose = start_pose;

    double elapsed = 0.0;
    double accumulated_cost = 0.0;

    std::size_t sample_count = 0;

    // ========================================================
    // 미래 trajectory rollout
    // ========================================================

    while(elapsed < duration){

        const double dt = std::min(simulation_time_step_, duration - elapsed);

        pose = simulateStep(pose, candidate.linear_x, candidate.angular_z, dt);

        double cost = 0.0;

        try{
            // onRun() 시작 시 scorePose(start_pose_, true)를 이미
            // 호출했기 때문에 동일한 snapshot을 재사용한다.
            cost = local_collision_checker_->scorePose(pose, false);
        }
        catch(const std::exception & e){
            RCLCPP_DEBUG(logger_, "Candidate [%s] rejected: %s", candidate.name.c_str(), e.what());

            return false;
        }

        // 254 = LETHAL_OBSTACLE
        // 255 = NO_INFORMATION
        //
        // 두 경우 모두 Candidate 폐기
        if(cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)){
            return false;
        }


        candidate.max_cost = std::max(candidate.max_cost, cost);

        accumulated_cost += cost;

        ++sample_count;

        elapsed += dt;
    }

    if(sample_count == 0){
        return false;
    }

    candidate.mean_cost = accumulated_cost / static_cast<double>(sample_count);

    // pose는 while문이 끝났으므로
    // 해당 Candidate의 최종 예상 pose다.
    candidate.score = computeCandidateScore(candidate, pose, candidate.max_cost);

    candidate.valid = std::isfinite(candidate.score);

    return candidate.valid;
}


// ============================================================
// Candidate Score 계산
//
// Score는 작을수록 좋다.
//
// 크게 세 가지를 본다:
//
// 1. Clearance
// 2. Goal progress
// 3. Maneuver penalty
// ============================================================

double AdaptiveEscape::computeCandidateScore(const Candidate & candidate, const geometry_msgs::msg::Pose2D & end_pose, double max_cost) const{

    // LETHAL 바로 아래의 최대 soft cost
    constexpr double max_soft_cost = 253.0;

    // ========================================================
    // 1. Clearance
    //
    // max_cost:
    // trajectory 중 가장 위험했던 순간
    //
    // mean_cost:
    // trajectory 전체가 전반적으로 얼마나 장애물과 가까웠는가
    // ========================================================

    const double normalized_max_cost = std::clamp(max_cost / max_soft_cost, 0.0, 1.0);

    const double normalized_mean_cost = std::clamp(candidate.mean_cost / max_soft_cost, 0.0, 1.0);

    // 순간적으로 장애물에 매우 가까워지는 것을 더 위험하게 본다.
    //
    // max  : 70%
    // mean : 30%
    const double clearance_term = clearance_weight_ * (0.7 * normalized_max_cost + 0.3 * normalized_mean_cost);

    // ========================================================
    // 2. Goal Progress
    //
    // Recovery 목적은 Goal까지 직접 주행하는 것이 아니지만,
    // 똑같이 안전한 후보라면 Goal에 가까워지는 방향을 선호한다.
    // ========================================================

    double goal_term = 0.0;

    if(goal_available_){

        const double start_goal_distance = std::hypot(goal_local_.pose.position.x - start_pose_.x, goal_local_.pose.position.y - start_pose_.y);

        const double end_goal_distance = std::hypot(goal_local_.pose.position.x - end_pose.x, goal_local_.pose.position.y - end_pose.y);

        // 양수:
        // Goal에 가까워짐
        //
        // 음수:
        // Goal에서 멀어짐
        const double progress = start_goal_distance - end_goal_distance;

        // 각 후보 이동거리가 약간 다르기 때문에
        // 최대 탈출거리 기준으로 정규화
        const double normalized_progress = std::clamp(progress / max_escape_distance_, -1.0, 1.0);

        // Score는 낮을수록 좋으므로
        // Goal로 가까워지면 음수 보상
        goal_term = -goal_progress_weight_ * normalized_progress;
    }

    // ========================================================
    // 3. Maneuver Penalty
    //
    // 안전성과 Goal progress가 비슷하다면
    // 가능하면 단순한 전진을 선호한다.
    // ========================================================

    double maneuver_penalty = 0.0;

    // 후진은 필요할 때만 선택하도록 penalty
    if(candidate.linear_x < -1e-6){
        maneuver_penalty += reverse_penalty_;
    }

    // 회전량이 클수록 조금 더 penalty
    //
    // Arc와 제자리 회전 모두 적용된다.
    if(candidate.target_angle > 1e-6 && max_rotation_ > 1e-6){

        const double rotation_fraction = std::clamp(candidate.target_angle / max_rotation_, 0.0, 1.0);

        maneuver_penalty += rotation_penalty_ * rotation_fraction;
    }

    return clearance_term + goal_term + maneuver_penalty;
}

// ============================================================
// 현재 시점에서 남은 trajectory가 여전히 안전한지 검사
//
// onRun() 후보 선택:
// 같은 Costmap snapshot 사용
//
// onCycleUpdate() 실제 실행:
// 매 cycle 최신 Costmap을 다시 가져옴
//
// 그래서 사람이 새로 들어오거나 장애물 상황이 변하면
// Recovery 실행 도중에도 즉시 중단할 수 있다.
// ============================================================

bool AdaptiveEscape::isTrajectorySafe(
    const Candidate & candidate,
    const geometry_msgs::msg::Pose2D & start_pose){

    if(!local_collision_checker_){
        return false;
    }

    geometry_msgs::msg::Pose2D pose = start_pose;

    // ========================================================
    // 반드시 현재 pose부터 확인
    //
    // remaining distance가 이미 0이어도
    // 현재 footprint가 collision이면 SUCCESS를 주면 안 된다.
    // ========================================================

    if(!local_collision_checker_->isCollisionFree(pose, true)){
        return false;
    }

    double duration = 0.0;

    if(candidate.type == ManeuverType::TRANSLATION){

        // 이미 목표거리를 달성한 경우
        // 현재 pose가 안전하다는 것까지 확인했으므로 true
        if(candidate.target_distance <= 1e-6){
            return true;
        }

        if(std::abs(candidate.linear_x) < 1e-6){
            return false;
        }

        duration = candidate.target_distance / std::abs(candidate.linear_x);
    }
    else{

        if(candidate.target_angle <= 1e-6){
            return true;
        }

        if(std::abs(candidate.angular_z) < 1e-6){
            return false;
        }

        duration = candidate.target_angle / std::abs(candidate.angular_z);
    }

    double elapsed = 0.0;

    while(elapsed < duration){

        const double dt = std::min(simulation_time_step_, duration - elapsed);


        pose = simulateStep(pose, candidate.linear_x, candidate.angular_z, dt);

        // 첫 pose에서 최신 Costmap을 fetch했으므로
        // 이후에는 같은 snapshot 사용
        if(!local_collision_checker_->isCollisionFree(pose, false)){
            return false;
        }

        elapsed += dt;
    }

    return true;
}

// ============================================================
// Constant Velocity Unicycle Model
//
// 현재 pose와 v, w, dt를 이용해서
// dt 이후의 예상 pose를 계산한다.
//
// A200은 skid-steer이지만 recovery의 짧은 저속 trajectory를
// 예측하는 1차 모델로 unicycle model을 사용한다.
// ============================================================

geometry_msgs::msg::Pose2D AdaptiveEscape::simulateStep(const geometry_msgs::msg::Pose2D & pose, double linear_x, double angular_z, double dt) const{

    geometry_msgs::msg::Pose2D next_pose = pose;

    // ========================================================
    // 거의 직진
    // ========================================================

    if(std::abs(angular_z) < 1e-6){

        next_pose.x += linear_x * std::cos(pose.theta) * dt;

        next_pose.y += linear_x * std::sin(pose.theta) * dt;

        return next_pose;
    }

    // ========================================================
    // Arc / Rotation
    //
    // 일정한 v, w일 때의 원호 운동을 정확하게 계산
    // ========================================================

    const double next_theta = pose.theta + angular_z * dt;

    const double radius = linear_x / angular_z;

    next_pose.x += radius * (std::sin(next_theta) - std::sin(pose.theta));

    next_pose.y += -radius * (std::cos(next_theta) - std::cos(pose.theta));

    next_pose.theta = normalizeAngle(next_theta);

    return next_pose;
}

// ============================================================
// 현재 Robot Pose를 Pose2D 형태로 가져오기
// ============================================================

bool AdaptiveEscape::getCurrentPose2D(geometry_msgs::msg::Pose2D & pose){

    geometry_msgs::msg::PoseStamped current_pose;

    if(!nav2_util::getCurrentPose(current_pose, *tf_, local_frame_, robot_base_frame_, transform_tolerance_)){

        return false;
    }

    pose.x = current_pose.pose.position.x;
    pose.y = current_pose.pose.position.y;
    pose.theta = tf2::getYaw(current_pose.pose.orientation);

    return true;
}

// ============================================================
// Navigation Goal을 AdaptiveEscape local frame으로 변환
//
// 일반적으로:
//
// map → odom
//
// 변환이 된다.
// ============================================================

bool AdaptiveEscape::transformGoalToLocal(const geometry_msgs::msg::PoseStamped & input_goal, geometry_msgs::msg::PoseStamped & local_goal){

    return nav2_util::transformPoseInTargetFrame(input_goal, local_goal, *tf_, local_frame_, transform_tolerance_);
}

// ============================================================
// 실제 이동거리 / 회전량 누적
//
// 중요한 점:
// 명령 속도로 계산하지 않고 TF에서 실제 움직인 값을 계산한다.
// ============================================================

void AdaptiveEscape::updateTravel(const geometry_msgs::msg::Pose2D & current_pose){

    // 실제 XY 이동거리
    distance_traveled_ += distance2D(previous_pose_, current_pose);

    const double delta_angle = normalizeAngle(current_pose.theta - previous_pose_.theta);

    // ========================================================
    // 제자리 회전 후보
    //
    // 명령한 방향으로 실제 회전한 양만 누적한다.
    //
    // 예를 들어 rotate_left인데 localization noise 때문에
    // 아주 조금 오른쪽으로 흔들린 것을 완료 각도에 더하지 않는다.
    // ========================================================

    if(has_selected_candidate_ && selected_candidate_.type == ManeuverType::ROTATION){

        if(selected_candidate_.angular_z > 0.0){
            angle_traveled_ += std::max(0.0, delta_angle);
        }
        else{
            angle_traveled_ += std::max(0.0, -delta_angle);
        }
    }
    else{
        // Arc / Translation에서는 feedback 용도로
        // 실제 회전량의 절대값을 기록
        angle_traveled_ += std::abs(delta_angle);
    }

    previous_pose_ = current_pose;
}

// ============================================================
// 선택한 Maneuver가 실제로 완료됐는지 확인
// ============================================================

bool AdaptiveEscape::isManeuverComplete() const{

    if(!has_selected_candidate_){
        return false;
    }

    // 전진 / 후진 / Arc
    if(selected_candidate_.type == ManeuverType::TRANSLATION){

        return distance_traveled_ >= selected_candidate_.target_distance;
    }

    // 제자리 회전
    return angle_traveled_ >= selected_candidate_.target_angle;
}


// ============================================================
// 최종 선택된 Candidate의 cmd_vel publish
// ============================================================

void AdaptiveEscape::publishSelectedCommand(){

    if(!has_selected_candidate_){
        return;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();

    cmd_vel->header.frame_id = robot_base_frame_;
    cmd_vel->header.stamp = clock_->now();

    cmd_vel->twist.linear.x = selected_candidate_.linear_x;

    cmd_vel->twist.angular.z = selected_candidate_.angular_z;

    vel_pub_->publish(std::move(cmd_vel));
}

// ============================================================
// Angle을 -pi ~ +pi 범위로 정규화
// ============================================================

double AdaptiveEscape::normalizeAngle(double angle){

    return std::atan2(std::sin(angle), std::cos(angle));
}

// ============================================================
// 두 Pose 사이 XY 거리
// ============================================================

double AdaptiveEscape::distance2D(
    const geometry_msgs::msg::Pose2D & a,
    const geometry_msgs::msg::Pose2D & b){

    return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace adaptive_escape_behavior

// ============================================================
// pluginlib에 AdaptiveEscape 등록
// ============================================================

PLUGINLIB_EXPORT_CLASS(adaptive_escape_behavior::AdaptiveEscape, nav2_core::Behavior)