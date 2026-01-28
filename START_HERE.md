# 🎯 START HERE!

**환영합니다!** Panda Robot Teleoperation System에 오신 것을 환영합니다! 🤖

---

## 🚀 5분 안에 시작하기

### 1️⃣ 문서 읽기 (2분)
```bash
cat ~/ros2_ws/SUMMARY.md
```
→ 시스템 전체 개요를 한눈에!

### 2️⃣ 시스템 실행 (1분)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch panda_teleop_joystick joystick_teleop.launch.py
```

### 3️⃣ 로봇 조작 (2분)
- **클러치 페달** 밟기
- **조이스틱** 움직이기
- **RViz**에서 로봇 관찰

---

## 📚 문서 읽는 순서

### 🥇 첫째 날
1. ✅ **`SUMMARY.md`** (7KB) - 전체 시스템 한눈에 보기
2. ✅ **`README.md`** (8KB) - 프로젝트 개요
3. ✅ **`QUICK_REFERENCE.md`** (2.4KB) - 자주 쓰는 명령어

### 🥈 둘째 날
1. ✅ **`ARCHITECTURE.md`** Part 1-3 (시스템 개요, 패키지 구조)
2. ✅ 실제로 시스템 실행해보기
3. ✅ 데이터 수집 테스트

### 🥉 셋째 날 이후
1. ✅ **`ARCHITECTURE.md`** 전체 읽기 (36KB, 969줄)
2. ✅ 노드 코드 분석
3. ✅ 새로운 기능 추가해보기

---

## 📖 문서 맵

```
START_HERE.md (여기!) ──┬─→ SUMMARY.md (빠른 개요)
                        │
                        ├─→ README.md (Quick Start)
                        │
                        ├─→ QUICK_REFERENCE.md (명령어 모음)
                        │
                        ├─→ ARCHITECTURE.md (완전한 설명 ⭐⭐⭐)
                        │
                        ├─→ TOPIC_INFORMATION.md (토픽 참조)
                        │
                        └─→ DOCS_INDEX.md (문서 네비게이션)
```

---

## 🎮 빠른 실행 가이드

### 조이스틱 모드 (추천 - 간단함)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_teleop_joystick joystick_teleop.launch.py
```

**조작법**:
- Left Stick: 전후/좌우
- Right Stick: 상하
- L1/L2: Roll 회전
- △/✕: 그리퍼

### OMY L100 모드 (리더암 필요)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py
```

**주의**: OMY L100 리더암이 `/dev/ttyUSB0`에 연결되어 있어야 합니다.

---

## 📊 데이터 수집

### 수집 시작
```bash
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    enable_data_collection:=true
```

### 데이터 확인
```bash
ls ~/ros2_ws/data/joystick/
```

### 데이터 분석
```bash
# 검증
python3 ~/ros2_ws/scripts/validate_dataset.py \
    ~/ros2_ws/data/joystick/session_YYYYMMDD_HHMMSS/

# 시각화
python3 ~/ros2_ws/scripts/visualize_episode.py \
    ~/ros2_ws/data/joystick/session_YYYYMMDD_HHMMSS/episode_001.jsonl
```

---

## 🆘 문제 해결

### 로봇이 안 움직여요!
```bash
# 1. 클러치 페달 확인
ros2 topic echo /clutch/active
# → True가 나와야 합니다

# 2. Twist 명령 확인
ros2 topic echo /servo_node/delta_twist_cmds
# → 조이스틱 움직이면 값이 변해야 합니다

# 3. 노드 실행 확인
ros2 node list
# → 모든 노드가 있어야 합니다
```

### RViz에 로봇이 안 보여요!
```bash
# TF 확인
ros2 run tf2_ros tf2_echo panda_link0 gripper_tip_link

# joint_states 확인
ros2 topic echo /joint_states
```

---

## 💡 유용한 명령어

### 시스템 상태 확인
```bash
# 모든 노드 확인
ros2 node list

# 모든 토픽 확인
ros2 topic list

# TF 트리 확인
ros2 run tf2_tools view_frames
```

### 디버깅
```bash
# 특정 노드 정보
ros2 node info /servo_node

# 토픽 정보
ros2 topic info /servo_node/delta_twist_cmds

# 토픽 데이터 보기
ros2 topic echo /clutch/active
```

---

## 🎓 학습 리소스

### 내부 문서
- 📖 **SUMMARY.md** - 시스템 개요
- 📘 **ARCHITECTURE.md** - 완전한 아키텍처 (필독!)
- 🚀 **QUICK_REFERENCE.md** - 빠른 참조

### 추천 읽기 순서
```
Day 1: START_HERE → SUMMARY → README
Day 2: QUICK_REFERENCE → 실습
Day 3: ARCHITECTURE (Part 1-3)
Week 2: ARCHITECTURE (전체)
```

---

## 🎯 다음 단계

### ✅ 완료했다면
- [x] 문서 읽기
- [x] 시스템 실행
- [x] 로봇 조작

### 🚀 이제 뭘 할까요?
1. **데이터 수집**: `enable_data_collection:=true`로 실행
2. **코드 분석**: `ARCHITECTURE.md` 읽고 노드 코드 보기
3. **새 기능**: 새로운 컨트롤러 추가해보기

---

## 📞 도움이 필요하신가요?

### 문서 찾기
```bash
# 모든 문서 보기
ls ~/ros2_ws/*.md

# 특정 키워드 검색
grep -r "키워드" ~/ros2_ws/*.md
```

### 문서 인덱스
```bash
cat ~/ros2_ws/DOCS_INDEX.md
```

---

## 🎊 Ready?

**좋습니다! 이제 시작해봅시다!**

```bash
# 1. 터미널 열기
# 2. 다음 명령어 입력:

cd ~/ros2_ws
source install/setup.bash
ros2 launch panda_teleop_joystick joystick_teleop.launch.py

# 3. 클러치 밟고 조이스틱 움직이기!
# 4. RViz에서 로봇 관찰하기!
```

**즐거운 로봇 프로그래밍 되세요!** 🤖✨

---

*궁금한 점이 있으면 `DOCS_INDEX.md`를 참조하세요!*
