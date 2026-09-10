# S4 — Narrow Passage / Constrained-Space Navigation (12 Cases)

## 목적
S4는 단순 장애물 회피가 아니라 **제한된 여유 공간을 실제 차체 크기로 안정적으로 통과할 수 있는가**를 평가한다.

현재 프로젝트에서 확인된 주요 기준:
- Local footprint: 약 0.988 m × 0.670 m
- Local inflation radius: 약 0.65 m
- Global robot_radius: 약 0.60 m
- Global inflation radius: 약 0.75 m

따라서 단순히 '벽 간 물리 폭'만 보는 것이 아니라,
Global Planner가 좁은 통로를 유효 경로로 선택하는지,
DWB가 footprint를 고려해 중앙을 유지하는지,
Collision Monitor가 과도하게 개입하지 않는지도 함께 본다.

## 케이스
- S4_01: 3.6 m 직선 협로 — 쉬움
- S4_02: 3.0 m 긴 직선 협로
- S4_03: 4.0 -> 3.2 -> 2.6 m Funnel
- S4_04: 좌/우로 중심이 바뀌는 Offset Double Gate
- S4_05: 중심선이 이동하는 S-bend
- S4_06: 90도 L-turn 협로
- S4_07: 협로 벽에서 원통형 돌출물이 번갈아 나오는 구조
- S4_08: 좌우 벽에서 Polygon wedge가 번갈아 안쪽으로 돌출되는 비정형 pinch-point 협로
- S4_09: 3.5 -> 2.2 m까지 변하는 장거리 variable-width bottleneck
- S4_10: 가장 어려운 multi-stage S-channel + 좁은 exit

## S3와의 차이
S3는 넓은 2D 공간에서 여러 장애물을 연속 회피하는 테스트다.
S4는 **회피할 공간 자체가 제한되어 있고 특정 통과 공간을 정밀하게 지나가야 하는 것**이 핵심이다.

## 주의
nominal_min_gap_m은 YAML의 물리 장애물 형상 기준 설계값이다. S4_08의 wedge tip 구간은 약 1.95 m로 가장 공격적인 국부 pinch point다.
Costmap inflation이 적용된 실제 planning cost 관점의 '유효 폭'은 더 좁게 느껴진다.
따라서 S4_09~10은 preview 후 Global Path가 실제로 통로를 통과 가능하다고 판단하는지 먼저 확인하는 것이 좋다.

## 설치
mkdir -p ~/nav_benchmark/cases/S4
cp S4_*.yaml ~/nav_benchmark/cases/S4/

## 추가 케이스
- S4_11: Asymmetric pocket corridor — 한쪽 벽이 들어왔다가 다시 넓어지고, 반대쪽에서 다음 bottleneck이 나타나는 구조
- S4_12: Diagonal mixed-geometry alternating gates — box/polygon/cylinder가 섞인 최종 극난도 협로. 좁은 통과구간이 연속적으로 좌우 이동함.
