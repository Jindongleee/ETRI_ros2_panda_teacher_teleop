# Data Collection for Imitation Learning

로봇 모방학습(Imitation Learning)을 위한 State-Action 쌍 데이터셋 수집 시스템입니다.

## 개요

이 시스템은 Panda 로봇의 teleoperation 중에 15Hz로 데이터를 수집하며, 각 에피소드는 로봇이 목표 지점에 도달하여 3초간 정지 상태를 유지할 때 자동으로 종료됩니다.

### 데이터 구조

각 샘플은 다음과 같은 구조를 가집니다:

```json
{
  "timestamp": 1706012345678901234,
  "seq": 123,
  "episode_id": "episode_001",
  
  "state": {
    "ee_pose": [x, y, z, qx, qy, qz, qw],
    "joints": [j1, j2, j3, j4, j5, j6, j7]
  },
  
  "action": {
    "delta_twist": [vx, vy, vz, wx, wy, wz]
  }
}
```

**중요**: 다음 샘플의 `state`는 암묵적으로 이전 샘플의 `action` 결과입니다.
즉, `state[i+1]`은 `state[i]`에서 `action[i]`를 실행한 결과입니다.

```
샘플 123: {state_123, action_123}
샘플 124: {state_124, action_124}  ← state_124는 action_123의 결과
샘플 125: {state_125, action_125}  ← state_125는 action_124의 결과
```

## 사용법

### 1. 데이터 수집 시작

```bash
# Terminal 1: 전체 시스템 실행 (데이터 수집 활성화)
cd ~/ros2_ws
source install/setup.bash
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py enable_data_collection:=true
```

### 2. 데이터 수집 워크플로우

1. **시스템 초기화** (~7초)
   - 모든 노드가 시작됩니다
   - 목표 지점이 생성되고 RViz에 빨간 구체로 표시됩니다

2. **Clutch 활성화 대기**
   - 발 페달을 밟아 clutch를 활성화합니다
   - Episode 1이 시작됩니다

3. **데이터 수집**
   - 목표 지점으로 로봇을 이동시킵니다
   - RViz에서 목표까지의 거리를 확인할 수 있습니다
   - 15Hz로 State-Action 쌍이 자동 수집됩니다

4. **Episode 종료 조건**
   - 목표 지점 도달 (2cm 이내)
   - 3초간 정지 상태 유지 (속도 < 8mm/s)
   - Episode가 자동 저장되고 새 목표 생성

5. **반복**
   - 총 3개 episode가 수집될 때까지 반복
   - 각 episode마다 clutch를 다시 활성화해야 합니다

6. **완료**
   - 3개 episode 수집 후 자동 완료
   - 데이터는 `./data/{controller_type}/session_{timestamp}/` 에 저장

### 3. 수집된 데이터 확인

```bash
# 데이터 파일 확인
ls -lh data/omy_l100/session_*/

# 출력 예시:
# episode_001.jsonl          - Episode 1 데이터 (JSONL 형식)
# episode_001_meta.json      - Episode 1 메타데이터
# episode_002.jsonl          - Episode 2 데이터
# episode_002_meta.json      - Episode 2 메타데이터
# episode_003.jsonl          - Episode 3 데이터
# episode_003_meta.json      - Episode 3 메타데이터
# session_summary.json       - 전체 세션 요약
```

## 설정

### 파라미터 설정

`config/data_collection_config.yaml` 파일에서 설정을 변경할 수 있습니다:

```yaml
data_collection_node:
  ros__parameters:
    # Episode 수집 개수
    max_episodes: 3
    
    # 컨트롤러 이름 (비교 실험용)
    controller_type: "omy_l100"
    
    # 수집 주파수
    collection_rate: 15.0  # Hz
    
    # Workspace 제한 (panda_link0 기준)
    workspace_x_min: 0.3
    workspace_x_max: 0.65
    workspace_y_min: -0.3
    workspace_y_max: 0.3
    workspace_z_min: 0.15
    workspace_z_max: 0.55
    
    # Episode 종료 조건
    reach_threshold: 0.02          # 2cm
    velocity_threshold: 0.008      # 8mm/s
    stationary_duration: 3.0       # seconds
```

### 컨트롤러 비교 실험

다른 컨트롤러로 데이터를 수집할 때:

1. `config/data_collection_config.yaml`에서 `controller_type` 변경
   ```yaml
   controller_type: "spacemouse"  # 또는 "keyboard", "haptic" 등
   ```

2. 시스템 재시작 및 데이터 수집

3. 데이터는 자동으로 별도 폴더에 저장됩니다:
   ```
   data/
   ├── omy_l100/
   │   └── session_20260123_143022/
   ├── spacemouse/
   │   └── session_20260123_151245/
   └── keyboard/
       └── session_20260123_160000/
   ```

## RViz 시각화

데이터 수집 중 RViz에서 다음을 확인할 수 있습니다:

- **빨간 구체**: 목표 지점
- **텍스트**: 현재 episode 번호 및 목표까지의 거리

## 로그 메시지

```
[INFO] 🤖 Data Collection Node Started!
[INFO] Controller Type: omy_l100
[INFO] Max Episodes: 3
[INFO] 🎯 Target Position: [0.452, 0.123, 0.341]
[INFO] ⏳ Waiting for clutch activation...

[INFO] 📝 Episode 1/3 STARTED
[INFO] [Episode 1/3] Distance: 18.3 cm | Samples: 45
[INFO] 🎯 Target reached! Distance: 1.8 cm
[INFO] ⏸️  Stationary: 1.2s / 3.0s
[INFO] ✅ Stationary condition met! (3.0s)
[INFO] ✅ Episode 1/3 COMPLETED!
[INFO] 📊 Samples: 45 | Duration: 3.2s
[INFO] 💾 Data saved: ./data/omy_l100/session_20260123_143022/episode_001.jsonl

[INFO] 🎉 DATA COLLECTION COMPLETED!
[INFO] Total Episodes: 3/3
[INFO] Data saved in: ./data/omy_l100/session_20260123_143022/
```

## 데이터 형식

### JSONL 파일 (.jsonl)

각 라인은 하나의 샘플을 나타내는 JSON 객체입니다:

```python
# Python에서 읽기
import json

samples = []
with open('episode_001.jsonl', 'r') as f:
    for line in f:
        sample = json.loads(line)
        samples.append(sample)

print(f"Total samples: {len(samples)}")
print(f"First sample: {samples[0]}")
```

### 메타데이터 파일 (.json)

```json
{
  "episode_id": "episode_001",
  "controller_type": "omy_l100",
  "target_position": [0.452, 0.123, 0.341],
  "start_time": "2026-01-23T14:30:22",
  "end_time": "2026-01-23T14:30:35",
  "duration": 13.2,
  "num_samples": 198,
  "success": true
}
```

## 트러블슈팅

### 1. TF lookup 실패

```
[WARN] TF lookup failed: ...
```

**해결**: 모든 노드가 정상 실행 중인지 확인. 특히 `robot_state_publisher`와 TF 발행 노드들.

### 2. Workspace 경고

```
[WARN] EE out of workspace! Skipping sample...
```

**해결**: 로봇이 안전한 작업 영역 내에 있는지 확인. 필요시 `config/data_collection_config.yaml`에서 workspace 범위 조정.

### 3. Episode 시작 안 됨

**해결**: 
- Clutch가 활성화되었는지 확인
- `/clutch/active` 토픽 확인: `ros2 topic echo /clutch/active`

### 4. 데이터 저장 안 됨

**해결**: 
- 출력 디렉토리 권한 확인
- 디스크 공간 확인

## 다음 단계

수집된 데이터로 다음을 수행할 수 있습니다:

1. **데이터 전처리**: 정규화, 필터링
2. **모델 학습**: Behavior Cloning, Diffusion Policy 등
3. **평가**: 컨트롤러 간 성능 비교

## 참고

- 데이터 형식: JSONL (JSON Lines)
- 좌표계: Panda base frame (`panda_link0`)
- End-effector frame: `panda_link8`
- 관절 순서: [panda_joint1, ..., panda_joint7]
- Quaternion 순서: [x, y, z, w]
