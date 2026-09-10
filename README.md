# A200 Nav2 PIL Benchmark

ROS 2 Nav2의 장애물 회피 성능을 자동으로 실행하고 평가하기 위한  
**Clearpath A200 Navigation Benchmark GUI**입니다.

PC의 WSL 환경에서는 Gazebo 시뮬레이션과 Benchmark GUI를 실행하고,  
Raspberry Pi 5에서는 실제 Nav2 경로 계획과 주행 제어 연산을 수행합니다.  
PC와 Pi는 Zenoh를 통해 ROS 2 데이터를 주고받습니다.

실제 Pi에서 Navigation 연산을 수행하면서 로봇과 센서는 Gazebo로
시뮬레이션하므로 **Processor-in-the-Loop(PIL)** 방식으로 검증합니다.

## Nav Benchmark QA

<img width="1331" height="831" alt="image" src="https://github.com/user-attachments/assets/30ac47e0-d0cd-41b0-96f7-379a6aba6919" />

GUI에서 Scenario와 Case를 선택하면 테스트가 자동으로 실행됩니다.

주요 기능은 다음과 같습니다.

- 단일 Case 또는 여러 Case 자동 실행
- 실행 상태와 로그 실시간 확인
- 성공, 실패 및 충돌 결과 판정
- 주행 시간, 경로 효율 및 최종 위치 오차 확인
- Raspberry Pi 실행 로그 수집
- 실행 결과 및 경로 시각화

현재 정적 장애물, 다중 장애물, 좁은 통로 및 동적 장애물 등
다양한 환경에서 Nav2의 회피 성능을 평가하고 있습니다.

## Path View

<img width="1069" height="788" alt="image" src="https://github.com/user-attachments/assets/9ae81264-5349-444e-947e-0bfd11468403" />

Path View에서는 다음 정보를 한 화면에서 비교할 수 있습니다.

- 장애물 위치와 형상
- 로봇의 실제 주행 궤적
- Nav2가 생성한 Global Path
- 주행 중 발생한 재계획 경로
- 시작점과 최종 목표점

이를 통해 로봇이 장애물을 어떻게 우회했는지와
계획 경로를 얼마나 정확하게 따라갔는지 확인할 수 있습니다.

## System

- **PC / WSL:** Gazebo 시뮬레이션, Benchmark 실행 및 결과 분석
- **Raspberry Pi 5:** ROS 2 Nav2 경로 계획, 장애물 회피 및 주행 제어
- **Communication:** Zenoh 기반 PC↔Pi ROS 2 통신

## Repository

- `wsl/`: Gazebo 환경, Benchmark GUI, Case 및 분석 스크립트
- `pi/`: Raspberry Pi에서 실행되는 Nav2 설정과 ROS 2 패키지
