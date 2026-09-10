# S2 — Alternative Route Selection (12 Cases)

## 목적
S1의 '단일 장애물 기본 회피'와 달리,
S2는 좌/우 두 우회로가 모두 가능한 상황에서 더 합리적인 경로를 선택하는지 평가한다.

## 설계 원칙
- 각 Case는 2개의 정적 box 장애물 사용
- 중앙 장벽이 좌/우 우회 선택을 강제
- 두 번째 장애물이 한쪽 끝을 연장해 해당 방향의 우회 비용을 증가
- 일부 Case는 약한 비용 차이만 줌
- 마지막 2개 Case는 좌/우 대칭으로 알고리즘 고유 편향 확인
- 좁은 corridor는 만들지 않음: 좁은 공간 통과는 S4에서 별도 평가
- Start→Goal 직선거리: 14~18 m

## Case 구성
- S2_01~03 : LEFT가 명확히 유리
- S2_04~06 : RIGHT가 명확히 유리
- S2_07~08 : LEFT가 약간 유리
- S2_09~10 : RIGHT가 약간 유리
- S2_11~12 : 좌우 대칭(EITHER)

## 설치
mkdir -p ~/nav_benchmark/cases/S2
cp S2_*.yaml ~/nav_benchmark/cases/S2/

## 빠른 확인
python3 ~/nav_benchmark/scripts/run_case.py S2_01 --build-only
