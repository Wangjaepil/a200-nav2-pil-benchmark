#!/usr/bin/env python3

import sys
import math
import time
import subprocess
from pathlib import Path

import yaml

# ============================================================
# 기본 경로
# ============================================================

HOME = Path.home()

BENCHMARK_ROOT = HOME / "nav_benchmark"

BASE_WORLD = (
    BENCHMARK_ROOT
    / "worlds"
    / "benchmark.sdf"
)

CASE_ROOT = (
    BENCHMARK_ROOT
    / "cases"
)

ACTIVE_DIR = (
    BENCHMARK_ROOT
    / "active"
)

CLEARPATH_SETUP = HOME / "clearpath"

WORLD_NAME = "benchmark"

# ============================================================
# 숫자 출력용
# ============================================================

def f(value):
    return f"{float(value):.6f}"

# ============================================================
# Case YAML 찾기
#
# S1_01 입력
#      ↓
# ~/nav_benchmark/cases/S1/S1_01.yaml
# ============================================================

def find_case(case_id):

    scenario = case_id.split("_")[0]

    case_file = (
        CASE_ROOT
        / scenario
        / f"{case_id}.yaml"
    )

    if not case_file.exists():
        raise FileNotFoundError(f"Case 파일을 찾을 수 없습니다:\n{case_file}")

    return case_file

# ============================================================
# 기울어진 장애물이 바닥을 뚫지 않도록 Z 계산
# ============================================================

def calculate_z(obs):

    shape = obs["shape"]

    pose = obs.get("pose", {})

    roll = float(pose.get("roll", 0.0))
    pitch = float(pose.get("pitch", 0.0))

    # 회전행렬의 Z 성분
    r31 = -math.sin(pitch)
    r32 = math.cos(pitch) * math.sin(roll)
    r33 = math.cos(pitch) * math.cos(roll)

    # --------------------------------------------------------
    # BOX
    # --------------------------------------------------------

    if shape == "box":

        sx, sy, sz = map(float, obs["size"])

        hx = sx / 2.0
        hy = sy / 2.0
        hz = sz / 2.0

        z = (abs(r31) * hx + abs(r32) * hy + abs(r33) * hz)

        return z

    # --------------------------------------------------------
    # CYLINDER
    # --------------------------------------------------------

    elif shape == "cylinder":

        radius = float(obs["radius"])
        height = float(obs["height"])

        radial_z = radius * math.sqrt(r31 * r31 + r32 * r32)

        axial_z = (height / 2.0) * abs(r33)

        return radial_z + axial_z

    # --------------------------------------------------------
    # POLYGON
    # --------------------------------------------------------

    elif shape == "polygon":

        height = float(obs["height"])

        points = [
            (float(p[0]), float(p[1]))
            for p in obs["points"]
        ]

        z_values = []

        for x, y in points:

            # polygon 바닥 / 천장 모두 계산
            for local_z in (0.0, height):

                world_z = (r31 * x + r32 * y + r33 * local_z)

                z_values.append(world_z)

        # 가장 낮은 부분이 정확히 z=0에 오도록
        return -min(z_values)

    else:
        raise ValueError(f"지원하지 않는 shape: {shape}")


# ============================================================
# YAML geometry → SDF geometry 변환
# ============================================================

def geometry_xml(obs):

    shape = obs["shape"]

    # --------------------------------------------------------
    # BOX
    # --------------------------------------------------------

    if shape == "box":

        sx, sy, sz = obs["size"]

        return f"""
            <box>
              <size>
                {f(sx)} {f(sy)} {f(sz)}
              </size>
            </box>
        """

    # --------------------------------------------------------
    # CYLINDER
    # --------------------------------------------------------

    elif shape == "cylinder":

        return f"""
            <cylinder>
              <radius>{f(obs["radius"])}</radius>
              <length>{f(obs["height"])}</length>
            </cylinder>
        """

    # --------------------------------------------------------
    # POLYGON
    # --------------------------------------------------------

    elif shape == "polygon":

        points_xml = ""

        for point in obs["points"]:

            points_xml += f"""
              <point>
                {f(point[0])} {f(point[1])}
              </point>
            """

        return f"""
            <polyline>

              <height>
                {f(obs["height"])}
              </height>

              {points_xml}

            </polyline>
        """

    else:
        raise ValueError(
            f"지원하지 않는 shape: {shape}"
        )


# ============================================================
# 장애물 하나를 SDF Model로 변환
# ============================================================

def obstacle_to_sdf(obs, index):

    pose = obs["pose"]

    x = float(pose["x"])
    y = float(pose["y"])

    roll = float(pose.get("roll", 0.0))

    pitch = float(pose.get("pitch", 0.0))

    yaw = float(pose.get("yaw", 0.0))

    z = calculate_z(obs)

    geometry = geometry_xml(obs)

    name = obs.get(
        "name",
        f"obstacle_{index:02d}"
    )

    return f"""

    <model name="{name}">

      <static>true</static>

      <pose>
        {f(x)}
        {f(y)}
        {f(z)}
        {f(roll)}
        {f(pitch)}
        {f(yaw)}
      </pose>

      <link name="link">

        <collision name="collision">

          <geometry>
            {geometry}
          </geometry>

        </collision>


        <visual name="visual">

          <geometry>
            {geometry}
          </geometry>

          <material>

            <ambient>
              0.55 0.55 0.55 1
            </ambient>

            <diffuse>
              0.55 0.55 0.55 1
            </diffuse>

          </material>

        </visual>

      </link>

    </model>

    """

# ============================================================
# 실제 Case World 생성
#
# benchmark.sdf
#       +
# 해당 YAML 장애물
#       ↓
# active/S1_01.sdf
# ============================================================

def build_case_world(data):

    if not BASE_WORLD.exists():

        raise FileNotFoundError(
            f"기본 World가 없습니다:\n{BASE_WORLD}"
        )

    world_text = BASE_WORLD.read_text(
        encoding="utf-8"
    )

    obstacles = data.get(
        "obstacles",
        []
    )

    obstacle_xml = ""

    for index, obs in enumerate(
        obstacles,
        start=1
    ):

        obstacle_xml += obstacle_to_sdf(obs, index)

    # </world> 직전에 장애물 삽입
    world_text = world_text.replace("</world>", obstacle_xml + "\n</world>", 1)

    ACTIVE_DIR.mkdir(parents=True, exist_ok=True)

    case_id = data["case_id"]

    output_file = (
        ACTIVE_DIR
        / f"{case_id}.sdf"
    )

    output_file.write_text(
        world_text,
        encoding="utf-8"
    )

    return output_file


# ============================================================
# Gazebo 실행
# ============================================================

def start_gazebo(world_file):

    # clearpath gz_sim.launch.py에는
    # 확장자를 제외한 world 경로 전달
    world_without_suffix = (
        world_file.with_suffix("")
    )

    command = [

        "ros2",
        "launch",
        "clearpath_gz",
        "gz_sim.launch.py",

        f"world:={world_without_suffix}",

        f"setup_path:={CLEARPATH_SETUP}"
    ]

    print("\n[Gazebo 실행]")

    print(
        " ".join(
            map(str, command)
        )
    )

    return subprocess.Popen(command)


# ============================================================
# A200 Spawn
#
# 여기서 Start 좌표를 YAML에서 자동으로 가져온다.
# ============================================================

def spawn_robot(robot):

    x = float(robot["x"])
    y = float(robot["y"])

    yaw = float(robot.get("yaw", 0.0))

    command = [

        "ros2",
        "launch",
        "clearpath_gz",
        "robot_spawn.launch.py",

        f"world:={WORLD_NAME}",

        f"setup_path:={CLEARPATH_SETUP}",

        f"x:={x}",
        f"y:={y}",

        "z:=0.15",

        f"yaw:={yaw}"
    ]

    print("\n[A200 Spawn]")

    print(f"Start = ({x}, {y})")

    print(f"Yaw   = {yaw}")

    return subprocess.Popen(command)


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # Case 이름 확인
    # --------------------------------------------------------

    if len(sys.argv) != 2:

        print("\n사용법:")

        print("python3 run_case.py S1_01\n")

        sys.exit(1)

    case_id = (
        sys.argv[1]
        .upper()
    )

    # --------------------------------------------------------
    # YAML 찾기
    # --------------------------------------------------------

    case_file = find_case(case_id)

    print(
        f"\nCase: {case_file}"
    )

    # --------------------------------------------------------
    # YAML 읽기
    # --------------------------------------------------------

    with open(
        case_file,
        "r",
        encoding="utf-8"
    ) as file:

        data = yaml.safe_load(file)

    # --------------------------------------------------------
    # Case 정보
    # --------------------------------------------------------

    robot = data["robot"]
    goal = data["goal"]

    start_x = float(robot["x"])
    start_y = float(robot["y"])

    goal_x = float(goal["x"])
    goal_y = float(goal["y"])

    distance = math.hypot(
        goal_x - start_x,
        goal_y - start_y
    )

    print("\n==============================")

    print(f"CASE : {case_id}")

    print(f"START : ({start_x}, {start_y})")

    print(f"GOAL  : ({goal_x}, {goal_y})")

    print(f"DIST  : {distance:.2f} m")

    print(
        f"OBSTACLES : "
        f"{len(data.get('obstacles', []))}"
    )

    print("==============================")

    # --------------------------------------------------------
    # 실제 SDF 생성
    # --------------------------------------------------------

    active_world = build_case_world(
        data
    )

    print(f"\n생성된 World:\n{active_world}")

    # --------------------------------------------------------
    # Gazebo 실행
    # --------------------------------------------------------

    gazebo_process = start_gazebo(
        active_world
    )

    # Gazebo가 뜰 시간
    print("\nGazebo 시작 대기...")

    time.sleep(5)

    # --------------------------------------------------------
    # YAML Start 좌표로 A200 자동 Spawn
    # --------------------------------------------------------

    robot_process = spawn_robot(
        robot
    )

    # --------------------------------------------------------
    # Goal은 아직 전송하지 않음
    #
    # 다음 단계에서 Pi Nav2에 자동 전송하도록 붙일 예정.
    # --------------------------------------------------------

    print("\n==============================")

    print("PC Benchmark 준비 완료")

    print(f"Case : {case_id}")

    print(
        f"Goal : "
        f"({goal_x}, {goal_y})"
    )

    print("==============================")

    print("\n종료하려면 Ctrl+C")

    try:
        gazebo_process.wait()

    except KeyboardInterrupt:

        print("\nBenchmark 종료")

        gazebo_process.terminate()
        robot_process.terminate()


if __name__ == "__main__":
    main()