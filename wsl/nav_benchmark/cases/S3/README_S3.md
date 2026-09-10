# S3 — Multi-Obstacle Sequential Avoidance (20 Cases)

## 핵심 목적
S3는 여러 정적 장애물을 연속적으로 만나면서
`계획 -> 회피 -> 경로 복귀 -> 다음 장애물 대응`을 반복적으로 안정 수행하는지 평가한다.

## 다른 시나리오와의 구분
- S1: 단일 장애물 기본 회피
- S2: 두 개의 유효 우회로 중 경로 선택
- S3: **다중 정적 장애물의 연속 회피 안정성**
- S4: 의도적으로 좁은 통로 / 제한 공간 통과
- S5: 동적 장애물 대응
- S6: 100 m 이상 장거리 통합 안정성

## 거리/난이도 분포
- S3_01~04 : 22~28 m / 장애물 3개
- S3_05~08 : 30~36 m / 장애물 4개
- S3_09~12 : 36~42 m / 장애물 5개
- S3_13~16 : 42~48 m / 장애물 6개
- S3_17~18 : 50~52 m / 장애물 7개
- S3_19~20 : 56~60 m / 장애물 8개

## 설계 원칙
- S2보다 확실히 긴 주행거리
- 후반으로 갈수록 장애물 수와 주행거리 모두 증가
- 좌/우 오프셋을 번갈아 배치해 반복적인 경로 이탈/복귀 유도
- 장애물 크기와 yaw도 일부 변화
- S4와 겹치지 않도록 의도적인 초협소 corridor는 만들지 않음
- S6와 겹치지 않도록 100 m 이상은 사용하지 않음

## 설치
mkdir -p ~/nav_benchmark/cases/S3
cp S3_*.yaml ~/nav_benchmark/cases/S3/

## 빠른 확인
python3 ~/nav_benchmark/scripts/run_case.py S3_01 --build-only
python3 ~/nav_benchmark/scripts/run_case.py S3_10 --build-only
python3 ~/nav_benchmark/scripts/run_case.py S3_20 --build-only
