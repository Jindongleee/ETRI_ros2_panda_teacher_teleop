# 🚀 Quick Start: 데이터 수집

모방학습 데이터를 빠르게 수집하는 방법입니다.

## ⚡ 빠른 시작 (3단계)

### 1️⃣ 시스템 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py enable_data_collection:=true
```

### 2️⃣ 데이터 수집 (x3)

1. **대기**: 빨간 구체(목표점)가 RViz에 나타날 때까지 ~7초 대기
2. **시작**: 발 페달을 밟아 clutch 활성화 → Episode 시작
3. **이동**: 로봇을 빨간 구체로 이동
4. **정지**: 목표점에 도달하면 3초간 정지 상태 유지 → Episode 자동 저장
5. **반복**: 총 3번 반복

### 3️⃣ 데이터 확인

```bash
# 저장된 파일 확인
ls -lh data/omy_l100/session_*/

# 데이터 검증
python3 scripts/validate_dataset.py data/omy_l100/session_20260123_143022/

# 시각화 (matplotlib 필요)
python3 scripts/visualize_episode.py data/omy_l100/session_20260123_143022/episode_001.jsonl
```

## 📊 예상 결과

```
data/omy_l100/session_20260123_143022/
├── episode_001.jsonl         # ~45 samples (3초 @ 15Hz)
├── episode_001_meta.json
├── episode_002.jsonl         # ~52 samples
├── episode_002_meta.json
├── episode_003.jsonl         # ~48 samples
├── episode_003_meta.json
└── session_summary.json
```

**총 데이터 크기**: ~145 samples (약 10초)

**데이터 구조**: 각 샘플은 `{state, action}` 쌍. 다음 샘플의 state가 이전 action의 결과.

## 🎯 중요 포인트

### Episode 종료 조건
- ✅ 목표점 도달 (2cm 이내)
- ✅ 3초간 정지 (속도 < 8mm/s)
- 자동으로 다음 episode로 전환

### RViz에서 확인할 것
- 🔴 빨간 구체: 목표 위치
- 📏 텍스트: 거리 및 진행 상황
- 🤖 Panda 로봇: 현재 위치

### 터미널 로그
```
[INFO] 🎯 Target Position: [0.452, 0.123, 0.341]
[INFO] ⏳ Waiting for clutch activation...
[INFO] 📝 Episode 1/3 STARTED
[INFO] Distance: 18.3 cm | Samples: 45
[INFO] 🎯 Target reached!
[INFO] ⏸️  Stationary: 2.1s / 3.0s
[INFO] ✅ Episode 1/3 COMPLETED!
```

## 🔧 설정 변경 (선택사항)

다른 컨트롤러로 테스트:

```bash
# config/data_collection_config.yaml 수정
controller_type: "spacemouse"  # 또는 "keyboard"

# 재시작
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py enable_data_collection:=true
```

## 🆘 문제 해결

| 문제 | 해결 |
|------|------|
| Episode 시작 안 됨 | 발 페달 확인, `/clutch/active` 토픽 체크 |
| 목표가 안 보임 | RViz에서 `/data_collection/markers` 추가 |
| 거리가 안 줄어듦 | 로봇 움직임 확인, TF 체크 |
| 3초 타이머 리셋됨 | 완전히 정지할 때까지 기다리기 |

## 📚 자세한 문서

- [전체 문서](DATA_COLLECTION_README.md)
- [패키지 README](README_KR.md)

## 🎉 성공하셨나요?

다음 단계:
1. ✅ 데이터 검증: `validate_dataset.py`
2. ✅ 시각화: `visualize_episode.py`
3. ✅ 다른 컨트롤러로 반복
4. 🤖 모델 학습 시작!

---

**문제가 있으신가요?** 로그를 확인하고 RViz 마커를 체크하세요!
