# S3 Multi-Obstacle Benchmark — Mixed Shapes v2

## 변경 범위
- S3_01~S3_05: 기존 파일 그대로 유지
- S3_06~S3_20: 장애물 형상을 box/cylinder/polygon 혼합으로 변경

## 유지한 것
- 각 Case의 Start/Goal
- Start→Goal 거리
- 장애물 개수
- 장애물 중심 위치
- S3의 핵심 목적: 다중 정적 장애물의 연속 회피 안정성

## 변경한 것
- 일부 box를 cylinder로 변경
- 일부 box를 triangle/trapezoid/pentagon polygon으로 변경
- 후반 Case일수록 형상 다양성을 조금 더 높임

## 목적
형상 다양성은 보조 변수일 뿐이며,
S3의 주목적은 여전히
`계획 -> 회피 -> 경로 복귀 -> 다음 장애물 대응`
을 반복적으로 안정 수행하는지 평가하는 것이다.

## 설치
mkdir -p ~/nav_benchmark/cases/S3
cp S3_*.yaml ~/nav_benchmark/cases/S3/
