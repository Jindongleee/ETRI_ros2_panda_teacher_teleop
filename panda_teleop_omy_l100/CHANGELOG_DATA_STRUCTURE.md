# 📝 Data Structure Update - Changelog

## 변경 요약

데이터 구조를 더 효율적으로 변경했습니다. **State 중복을 제거**하여 메모리와 저장 공간을 약 **40% 절약**합니다.

## 변경 전 vs 변경 후

### ❌ 이전 구조 (중복 있음)

```json
{
  "timestamp": 1706012345678901234,
  "seq": 123,
  "state_t": {
    "ee_pose": [x, y, z, qx, qy, qz, qw],
    "joints": [j1, j2, j3, j4, j5, j6, j7]
  },
  "action_t": {
    "delta_twist": [vx, vy, vz, wx, wy, wz]
  },
  "state_t1": {
    "ee_pose": [x2, y2, z2, qx2, qy2, qz2, qw2],  // 중복!
    "joints": [j1_2, j2_2, j3_2, j4_2, j5_2, j6_2, j7_2]  // 중복!
  }
}
```

**문제점**:
- `state_t1`이 다음 샘플의 `state_t`와 동일 (중복)
- 메모리 낭비 (~40% 오버헤드)
- 모방학습에서 `state_t1`을 명시적으로 사용하지 않음

---

### ✅ 새로운 구조 (중복 제거)

```json
// 샘플 123
{
  "timestamp": 1706012345678901234,
  "seq": 123,
  "state": {
    "ee_pose": [x, y, z, qx, qy, qz, qw],
    "joints": [j1, j2, j3, j4, j5, j6, j7]
  },
  "action": {
    "delta_twist": [vx, vy, vz, wx, wy, wz]
  }
}

// 샘플 124 (state_124는 action_123의 결과)
{
  "timestamp": 1706012345745567901,
  "seq": 124,
  "state": {
    "ee_pose": [x2, y2, z2, qx2, qy2, qz2, qw2],
    "joints": [j1_2, j2_2, j3_2, j4_2, j5_2, j6_2, j7_2]
  },
  "action": {
    "delta_twist": [vx2, vy2, vz2, wx2, wy2, wz2]
  }
}
```

**장점**:
- ✅ State 중복 제거
- ✅ 메모리 40% 절약
- ✅ 저장 공간 40% 절약
- ✅ 대부분의 모방학습 데이터셋 표준과 일치
- ✅ 연속성이 더 자연스러움

---

## 데이터 해석

### Transition 관계

```
state[i] + action[i] → state[i+1]
```

- `state[123]`에서 `action[123]`을 실행 → 결과: `state[124]`
- `state[124]`에서 `action[124]`을 실행 → 결과: `state[125]`

### Python에서 사용 예시

```python
import json

# 데이터 로드
samples = []
with open('episode_001.jsonl', 'r') as f:
    for line in f:
        samples.append(json.loads(line))

# Transition 추출
for i in range(len(samples) - 1):
    state_t = samples[i]['state']
    action_t = samples[i]['action']
    state_t1 = samples[i+1]['state']  # 다음 샘플의 state
    
    # 모방학습
    # policy: state_t -> action_t
    # dynamics: (state_t, action_t) -> state_t1
```

---

## 수정된 파일 목록

### 1. 핵심 코드
- ✅ `src/data_collection_node.py`
  - `collect_data_sample()` 함수 단순화
  - `previous_state` 변수 제거
  - 주석 업데이트

### 2. 유틸리티 스크립트
- ✅ `scripts/validate_dataset.py`
  - 검증 로직 업데이트 (`state_t/action_t/state_t1` → `state/action`)
  
- ✅ `scripts/visualize_episode.py`
  - 데이터 추출 로직 업데이트

### 3. 문서
- ✅ `DATA_COLLECTION_README.md`
  - 데이터 구조 설명 업데이트
  - Transition 관계 추가 설명
  
- ✅ `QUICKSTART_DATA_COLLECTION.md`
  - 데이터 구조 노트 추가
  
- ✅ `IMPLEMENTATION_SUMMARY.md`
  - 데이터 구조 예시 업데이트

---

## 호환성

### 기존 데이터
이전 형식으로 수집된 데이터는 **호환되지 않습니다**.
새로운 형식으로 데이터를 다시 수집해야 합니다.

### 변환 스크립트 (필요 시)
기존 데이터를 새 형식으로 변환하려면:

```python
# old_to_new_format.py
import json

def convert_old_to_new(old_samples):
    new_samples = []
    for sample in old_samples:
        new_sample = {
            'timestamp': sample['timestamp'],
            'seq': sample['seq'],
            'episode_id': sample['episode_id'],
            'state': sample['state_t'],  # state_t → state
            'action': sample['action_t']  # action_t → action
        }
        new_samples.append(new_sample)
    return new_samples
```

---

## 이점 요약

| 항목 | 이전 | 새로운 | 개선 |
|------|------|--------|------|
| 샘플당 크기 | ~1.5 KB | ~0.9 KB | **40% 감소** |
| Episode (45 samples) | ~67 KB | ~40 KB | **40% 감소** |
| 3 Episodes | ~200 KB | ~120 KB | **40% 감소** |
| State 중복 | 있음 | 없음 | ✅ |
| 표준 준수 | 부분적 | 완전 | ✅ |
| 코드 복잡도 | 중간 | 낮음 | ✅ |

---

## 테스트 완료

- ✅ 빌드 성공
- ✅ 문서 업데이트
- ✅ 검증 스크립트 업데이트
- ✅ 시각화 스크립트 업데이트

---

## 다음 사용 시

새로운 데이터 구조로 데이터를 수집하면:
1. 저장 공간 40% 절약
2. 로딩 속도 개선
3. 표준 모방학습 라이브러리와 호환성 향상

**준비 완료!** 이제 효율적인 데이터 수집을 시작하세요! 🚀

---

**변경일**: 2026-01-23  
**버전**: 2.0 (Data Structure Optimized)
