"""
Far Goal Manager - costmap 경계 클램핑 방식

기존 방식(직선 등분 waypoint)의 문제:
    출발점~목표 직선을 기하학적으로 등분해서 중간 waypoint를 만들고,
    각 waypoint에 switch_distance(0.5m) 이내로 "도달"해야 다음으로 넘어갔다.
    분할 기준이 실제 제약(costmap 경계)과 무관해서 등분점이 장애물 안에
    떨어질 수 있었고, 그러면 도달이 불가능해 영원히 다음으로 넘어가지 못했다.

이 방식:
    분할 기준을 costmap 반경으로 바꾸고, 중간 목표를 연속 갱신한다.
    - 최종 목표가 costmap 안이면  -> 실제 목표를 그대로 전송
    - 밖이면                      -> 로봇에서 safe_goal_distance 지점으로 클램핑

    중간 목표는 "도달해야 하는 지점"이 아니라 로봇이 전진하면 함께 전진하는
    지점이므로, 도달 실패로 인한 데드락이 구조적으로 발생하지 않는다.

    재전송은 로봇이 resend_threshold 만큼 실제로 이동했을 때만 일어난다.
    따라서 로봇이 막혀 있으면 목표가 갱신되지 않고, Nav2의 progress checker와
    복구 동작이 방해받지 않는다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener


class FarGoalManager(Node):

    def __init__(self):
        super().__init__('far_goal_manager')

        # ==============================================================
        # Parameters
        # ==============================================================

        # 로봇에서 이 거리까지는 Nav2에 직접 목표를 줄 수 있다.
        # global costmap 반경보다 충분히 작아야 플래너가 목표 주변을
        # 우회할 여유를 가진다.
        #   costmap 50x50 -> 반경 25m -> 20.0 권장 (여유 5m)
        #   costmap 30x30 -> 반경 15m -> 11.0
        self.declare_parameter('safe_goal_distance', 10.0)
        self.safe_goal_distance = float(
            self.get_parameter('safe_goal_distance').value)

        # 직전에 보낸 목표에서 이만큼 벗어나야 재전송한다.
        # 너무 작으면 preemption이 잦아 Nav2가 계속 재시작되고,
        # 너무 크면 목표가 로봇 뒤로 처진다.
        self.declare_parameter('resend_threshold', 2.0)
        self.resend_threshold = float(
            self.get_parameter('resend_threshold').value)

        # 목표 갱신 주기 (초)
        self.declare_parameter('update_period', 0.5)
        self.update_period = float(
            self.get_parameter('update_period').value)

        self.declare_parameter('global_frame', 'map')
        self.global_frame = str(
            self.get_parameter('global_frame').value)

        self.declare_parameter('base_frame', 'base_link')
        self.base_frame = str(
            self.get_parameter('base_frame').value)

        # Nav2가 ABORTED를 반환했는데도 최종 목표와 매우 가까운 경우
        # 로그로 알려주기 위한 거리 (성공 판정은 하지 않음)
        self.declare_parameter('final_near_distance', 0.5)
        self.final_near_distance = float(
            self.get_parameter('final_near_distance').value)

        # --------------------------------------------------------------
        # 클램핑 목표의 costmap 검사
        #
        # 클램핑 목표는 보통 센서 범위(12m) 밖이라 unknown/free로 나온다.
        # 로봇이 접근하면서 그 지점이 장애물로 드러나는 경우를 위한 보험.
        # --------------------------------------------------------------

        self.declare_parameter('check_target_cost', True)
        self.check_target_cost = bool(
            self.get_parameter('check_target_cost').value)

        # OccupancyGrid 스케일(0~100). 99=inscribed, 100=lethal.
        # inflation_radius 0.75 / cost_scaling_factor 3.0 / inscribed 0.455
        # 기준으로 50은 장애물 표면에서 약 0.68m 여유에 해당한다.
        self.declare_parameter('max_target_cost', 50)
        self.max_target_cost = int(
            self.get_parameter('max_target_cost').value)

        self.declare_parameter('max_lateral_search_m', 4.0)
        self.max_lateral_search_m = float(
            self.get_parameter('max_lateral_search_m').value)

        # 수직 탐색으로 못 찾으면 로봇 쪽으로 당기며 재탐색
        self.declare_parameter('max_pullback_m', 8.0)
        self.max_pullback_m = float(
            self.get_parameter('max_pullback_m').value)

        self.declare_parameter('search_step_m', 0.25)
        self.search_step_m = float(
            self.get_parameter('search_step_m').value)

        self.declare_parameter('allow_unknown_cells', True)
        self.allow_unknown_cells = bool(
            self.get_parameter('allow_unknown_cells').value)

        self.declare_parameter(
            'global_costmap_topic', '/global_costmap/costmap')
        self.global_costmap_topic = str(
            self.get_parameter('global_costmap_topic').value)

        # 중간 목표에서 Nav2가 ABORTED를 낸 횟수 한계.
        # 클램핑 목표는 도달 대상이 아니므로 ABORT는 실제 주행 실패를 뜻한다.
        self.declare_parameter('max_intermediate_aborts', 3)
        self.max_intermediate_aborts = int(
            self.get_parameter('max_intermediate_aborts').value)

        # 현재 Nav2에 걸려 있는 목표가 장애물로 확인되면 이동 거리와
        # 무관하게 즉시 교체할지 여부.
        #
        # 기본 False.
        # 이 기능을 켜면 목표 교체가 잦아질 수 있고, 교체가 계획 생성보다
        # 빠르면 Nav2가 계획을 낼 시간을 받지 못해 로봇이 출발조차 못 한다.
        # 끈 상태(=run_006과 동일한 거동)로 기준선을 먼저 잡고,
        # 필요할 때만 켜서 효과를 확인할 것.
        self.declare_parameter('force_resend_on_blocked', False)
        self.force_resend_on_blocked = bool(
            self.get_parameter('force_resend_on_blocked').value)

        # ==============================================================
        # Subscriptions / Action client
        # ==============================================================

        # /goal_pose를 직접 쓰면 bt_navigator도 같이 받으므로 별도 토픽 사용
        self.goal_sub = self.create_subscription(
            PoseStamped, '/far_goal_pose', self.goal_callback, 10)

        # nav2 costmap publisher는 transient_local QoS를 쓴다.
        # global_costmap에 always_send_full_costmap: true 필요.
        costmap_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.costmap = None

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.global_costmap_topic,
            self.costmap_callback, costmap_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # ==============================================================
        # Internal state
        # ==============================================================

        self.final_goal = None
        self.running = False
        self.goal_pending = False

        # 직전에 Nav2로 보낸 목표 좌표
        self.last_sent_xy = None

        # 최종 목표를 실제로 보냈는가.
        # 보낸 뒤에는 preempt하지 않고 Nav2의 GoalChecker 판정을 기다린다.
        self.final_goal_sent = False

        # 재전송 시 이전 goal의 늦은 result를 무시하기 위한 세대 번호
        self.goal_generation = 0

        self.intermediate_abort_count = 0

        self.update_timer = self.create_timer(
            self.update_period, self.update_target)

        self.get_logger().info(
            'Far Goal Manager (costmap clamping) started\n'
            f'  global_frame        = {self.global_frame}\n'
            f'  safe_goal_distance  = {self.safe_goal_distance:.2f} m\n'
            f'  resend_threshold    = {self.resend_threshold:.2f} m\n'
            f'  update_period       = {self.update_period:.2f} s\n'
            f'  check_target_cost   = {self.check_target_cost}\n'
            f'  max_target_cost     = {self.max_target_cost}\n'
            f'  costmap_topic       = {self.global_costmap_topic}')

    # ==================================================================
    # 유틸
    # ==================================================================

    def get_robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
            return (tf.transform.translation.x,
                    tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(
                f'Cannot get {self.global_frame} -> {self.base_frame}: {e}',
                throttle_duration_sec=2.0)
            return None

    def status_name(self, status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'UNKNOWN({status})')

    # ==================================================================
    # Costmap 조회
    # ==================================================================

    def costmap_callback(self, msg):
        self.costmap = msg

    def cost_at(self, x, y):
        """
        map 좌표의 costmap 값.
            0~100 : cost (99=inscribed, 100=lethal)
            -1    : unknown
            None  : costmap 범위 밖 또는 미수신
        """
        cm = self.costmap

        if cm is None:
            return None

        info = cm.info

        if info.resolution <= 0.0:
            return None

        # global costmap의 origin은 회전이 없다고 가정
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None

        return cm.data[my * info.width + mx]

    def is_cell_usable(self, x, y):
        """
        이 자리를 '새 목표로 삼아도 되는가'.

        모르는 곳(범위 밖)은 보수적으로 거부한다.
        후보를 고르는 용도이므로 이게 맞다.
        """
        cost = self.cost_at(x, y)

        if cost is None:
            return False

        if cost < 0:
            return self.allow_unknown_cells

        return cost <= self.max_target_cost

    def is_cell_known_blocked(self, x, y):
        """
        이 자리가 '실제로 장애물로 확인되었는가'.

        is_cell_usable의 반대가 아니다. 반드시 구분해야 한다.
            범위 밖(None)   -> 모름   -> False
            unknown(-1)     -> 모름   -> False
            cost > 임계값   -> 막힘   -> True

        이미 Nav2에 걸려 있는 목표를 강제 교체할지 판단하는 데 쓴다.
        여기서 '모름'을 '막힘'으로 취급하면, 출발 직후 costmap이 아직
        목표 지점을 덮지 못한 동안 매 주기 재전송이 일어나고,
        Nav2가 계획을 낼 시간을 받지 못해 로봇이 아예 출발하지 못한다.
        (S1_05 run_002: goal 3회 전송 / plan 0회 / 이동 0 m)
        """
        cost = self.cost_at(x, y)

        if cost is None:
            return False

        if cost < 0:
            return False

        return cost > self.max_target_cost

    def repair_target(self, x, y, robot_x, robot_y, direction_yaw):
        """
        클램핑 목표가 장애물 위면 대체 지점을 찾는다.

        1) 경로 수직방향 좌/우 (진행 거리 유지)
        2) 로봇 쪽으로 당기며 재탐색

        반환: ((x, y), reason) 또는 (None, 'failed')
        """
        if not self.check_target_cost:
            return (x, y), 'disabled'

        if self.costmap is None:
            return (x, y), 'no_costmap'

        if self.is_cell_usable(x, y):
            return (x, y), 'ok'

        fx = math.cos(direction_yaw)
        fy = math.sin(direction_yaw)
        px = -fy
        py = fx

        step = max(self.search_step_m, 0.01)
        n_lat = int(self.max_lateral_search_m / step)
        n_pull = int(self.max_pullback_m / step)

        for p in range(n_pull + 1):
            pull = p * step
            bx = x - fx * pull
            by = y - fy * pull

            # 로봇 바로 앞까지 당겨졌으면 더 볼 필요 없다
            if (bx - robot_x) * fx + (by - robot_y) * fy < 1.0:
                break

            for l in range(n_lat + 1):
                lat = l * step
                candidates = []

                for sign in ((0,) if l == 0 else (1, -1)):
                    cx = bx + px * lat * sign
                    cy = by + py * lat * sign

                    if self.is_cell_usable(cx, cy):
                        c = self.cost_at(cx, cy)
                        key = 50 if (c is None or c < 0) else c
                        candidates.append((key, cx, cy))

                if candidates:
                    candidates.sort(key=lambda t: t[0])
                    _, cx, cy = candidates[0]
                    return (cx, cy), 'moved'

        return None, 'failed'

    # ==================================================================
    # Far Goal 수신
    # ==================================================================

    def goal_callback(self, msg):

        if self.running:
            self.get_logger().warn(
                'Navigation already running. New far goal ignored.')
            return

        if msg.header.frame_id != self.global_frame:
            self.get_logger().error(
                f'Goal frame must be {self.global_frame}, '
                f'but received: {msg.header.frame_id}')
            return

        robot_pos = self.get_robot_position()

        if robot_pos is None:
            self.get_logger().error(
                'Cannot get robot pose. Goal rejected.')
            return

        distance = math.hypot(msg.pose.position.x - robot_pos[0],
                              msg.pose.position.y - robot_pos[1])

        if distance < 0.1:
            self.get_logger().warn('Goal is too close.')
            return

        self.final_goal = msg
        self.running = True
        self.goal_pending = False
        self.last_sent_xy = None
        self.final_goal_sent = False
        self.intermediate_abort_count = 0

        self.get_logger().info('========================================')
        self.get_logger().info(
            f'Robot      : ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')
        self.get_logger().info(
            f'Final goal : ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')
        self.get_logger().info(f'Distance   : {distance:.2f} m')

        if distance <= self.safe_goal_distance:
            self.get_logger().info(
                'Within safe distance - sending final goal directly')
        else:
            self.get_logger().info(
                f'Beyond safe distance - clamping to '
                f'{self.safe_goal_distance:.1f} m ahead')

        if self.costmap is None and self.check_target_cost:
            self.get_logger().warn(
                f'Global costmap ({self.global_costmap_topic}) not received. '
                f'Check always_send_full_costmap in global_costmap params.')

        self.get_logger().info('========================================')

        self.update_target()

    # ==================================================================
    # 목표 갱신 (핵심)
    # ==================================================================

    def update_target(self):

        if not self.running or self.final_goal is None:
            return

        if self.goal_pending:
            return

        # 최종 목표를 이미 보냈으면 preempt하지 않는다.
        # Nav2의 StoppedGoalChecker가 판정할 시간을 준다.
        if self.final_goal_sent:
            return

        robot_pos = self.get_robot_position()

        if robot_pos is None:
            return

        robot_x, robot_y = robot_pos

        goal_x = self.final_goal.pose.position.x
        goal_y = self.final_goal.pose.position.y

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.hypot(dx, dy)

        # --------------------------------------------------------------
        # 최종 목표가 사정거리 안 -> 실제 목표 전송
        # --------------------------------------------------------------

        if distance <= self.safe_goal_distance:

            self.get_logger().info(
                f'Final goal within {distance:.2f} m - sending it directly')

            self.send_goal(goal_x, goal_y,
                           orientation=self.final_goal.pose.orientation,
                           is_final=True)
            return

        # --------------------------------------------------------------
        # 밖이면 safe_goal_distance 지점으로 클램핑
        # --------------------------------------------------------------

        direction_yaw = math.atan2(dy, dx)

        tx = robot_x + math.cos(direction_yaw) * self.safe_goal_distance
        ty = robot_y + math.sin(direction_yaw) * self.safe_goal_distance

        raw_cost = self.cost_at(tx, ty)

        fixed, how = self.repair_target(
            tx, ty, robot_x, robot_y, direction_yaw)

        if fixed is None:
            self.get_logger().warn(
                f'Clamped target ({tx:.2f}, {ty:.2f}) is blocked and no '
                f'free cell found. Keeping current goal.',
                throttle_duration_sec=5.0)
            return

        if how == 'moved':
            self.get_logger().warn(
                f'Clamped target ({tx:.2f}, {ty:.2f}) cost={raw_cost} '
                f'not usable -> ({fixed[0]:.2f}, {fixed[1]:.2f})')

        tx, ty = fixed

        # --------------------------------------------------------------
        # 재전송 판정
        #
        # 기본: 로봇이 resend_threshold 만큼 이동했을 때만 재전송.
        #       로봇이 막혀 있으면 목표가 갱신되지 않으므로 Nav2의
        #       progress checker와 복구 동작이 방해받지 않는다.
        #
        # 예외: 지금 Nav2에 걸려 있는 목표가 장애물 위에 있으면
        #       이동 거리와 무관하게 즉시 교체한다.
        #
        #       출발 직후에는 costmap이 아직 비어 있어 첫 목표가 장애물
        #       안에 찍힐 수 있다. 0.5초 뒤 costmap이 채워져 repair가
        #       대체 지점을 찾아도, 그 거리가 resend_threshold 미만이면
        #       교체되지 않은 채 플래너만 5Hz로 헛돌게 된다.
        #       (S1_03 run_006에서 목표 1번에 계획 52회)
        # --------------------------------------------------------------

        force_resend = False

        if (self.force_resend_on_blocked
                and self.last_sent_xy is not None
                and self.check_target_cost
                and self.costmap is not None
                and self.is_cell_known_blocked(*self.last_sent_xy)):

            # 무의미한 재전송 방지를 위한 최소 이동량
            if math.hypot(tx - self.last_sent_xy[0],
                          ty - self.last_sent_xy[1]) >= 0.5:

                force_resend = True

                self.get_logger().warn(
                    f'Active goal ({self.last_sent_xy[0]:.2f}, '
                    f'{self.last_sent_xy[1]:.2f}) is now blocked '
                    f'(cost={self.cost_at(*self.last_sent_xy)}) - '
                    f'replacing immediately')

        if not force_resend and self.last_sent_xy is not None:
            moved = math.hypot(tx - self.last_sent_xy[0],
                               ty - self.last_sent_xy[1])

            if moved < self.resend_threshold:
                return

        self.send_goal(tx, ty, yaw=direction_yaw, is_final=False)

    # ==================================================================
    # 목표 전송
    # ==================================================================

    def send_goal(self, x, y, yaw=None, orientation=None, is_final=False):

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'NavigateToPose action server not available')
            self.running = False
            return

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        if orientation is not None:
            pose.pose.orientation = orientation
        else:
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.goal_generation += 1
        generation = self.goal_generation

        self.goal_pending = True
        self.last_sent_xy = (x, y)

        if is_final:
            self.final_goal_sent = True

        self.get_logger().info(
            f'Sending {"FINAL" if is_final else "clamped"} goal: '
            f'({x:.2f}, {y:.2f})')

        future = self.nav_client.send_goal_async(goal_msg)

        future.add_done_callback(
            lambda f, gen=generation, fin=is_final:
            self.goal_response_callback(f, gen, fin))

    # ==================================================================
    # Goal accepted / rejected
    # ==================================================================

    def goal_response_callback(self, future, generation, is_final):

        if generation != self.goal_generation:
            return

        self.goal_pending = False

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')
            self.running = False
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.running = False
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f, gen=generation, fin=is_final:
            self.result_callback(f, gen, fin))

    # ==================================================================
    # Result
    # ==================================================================

    def result_callback(self, future, generation, is_final):

        # 갱신된 목표로 preempt된 이전 goal
        if generation != self.goal_generation:
            return

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to get result: {e}')
            self.running = False
            return

        status = result.status
        status_text = self.status_name(status)

        robot_pos = self.get_robot_position()

        remaining = None

        if robot_pos is not None and self.final_goal is not None:
            remaining = math.hypot(
                self.final_goal.pose.position.x - robot_pos[0],
                self.final_goal.pose.position.y - robot_pos[1])

        self.get_logger().info('========================================')
        self.get_logger().info(
            f'Navigation result ({"FINAL" if is_final else "clamped"})')
        self.get_logger().info(f'  status    : {status_text}')

        if robot_pos is not None:
            self.get_logger().info(
                f'  robot     : ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')

        if remaining is not None:
            self.get_logger().info(
                f'  to goal   : {remaining:.2f} m')

        self.get_logger().info('========================================')

        # --------------------------------------------------------------
        # SUCCEEDED
        # --------------------------------------------------------------

        if status == GoalStatus.STATUS_SUCCEEDED:

            if is_final:
                self.get_logger().info('')
                self.get_logger().info(
                    '########################################')
                self.get_logger().info(
                    '#####   FINAL GOAL REACHED !!!     #####')

                if remaining is not None:
                    self.get_logger().info(
                        f'#####   Remaining: {remaining:.2f} m'
                        f'              #####')

                self.get_logger().info(
                    '########################################')

                self.running = False
                return

            # 클램핑 목표에 도달한 경우.
            # 정상적으로는 목표가 계속 앞서가므로 발생하지 않지만,
            # 발생해도 다음 갱신에서 새 목표가 나가면 된다.
            self.intermediate_abort_count = 0
            self.last_sent_xy = None
            self.update_target()
            return

        # --------------------------------------------------------------
        # 최종 목표 근처에서의 ABORTED/CANCELED
        # --------------------------------------------------------------

        if (is_final and remaining is not None
                and remaining <= self.final_near_distance):

            self.get_logger().warn(
                '########################################')
            self.get_logger().warn(
                f'Nav2 returned {status_text}, but robot is very close '
                f'to final goal ({remaining:.2f} m).')
            self.get_logger().warn(
                'Position reached, but Nav2 did NOT declare SUCCESS.')
            self.get_logger().warn(
                'Check final orientation / GoalChecker.')
            self.get_logger().warn(
                '########################################')

            self.running = False
            return

        # --------------------------------------------------------------
        # 중간 목표 실패
        #
        # 클램핑 목표는 도달 대상이 아니므로 ABORT는 실제 주행 실패다.
        # (Nav2의 복구 동작은 이미 전부 소진된 상태)
        # 상황이 바뀌었을 수 있으므로 제한된 횟수만 재시도한다.
        # --------------------------------------------------------------

        if not is_final:

            self.intermediate_abort_count += 1

            if self.intermediate_abort_count < self.max_intermediate_aborts:

                self.get_logger().warn(
                    f'Clamped goal {status_text} '
                    f'({self.intermediate_abort_count}/'
                    f'{self.max_intermediate_aborts}). Retrying with a '
                    f'freshly computed target.')

                self.last_sent_xy = None
                self.update_target()
                return

        self.get_logger().error(
            f'Navigation failed: status={status_text}')

        self.running = False


def main(args=None):

    rclpy.init(args=args)

    node = FarGoalManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()