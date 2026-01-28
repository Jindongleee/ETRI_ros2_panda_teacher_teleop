# 📚 Documentation Index

## 🎯 어떤 문서를 읽어야 할까요?

### 🚀 **처음 시작하시나요?**
1. **`README.md`** 읽기
   - 시스템 개요
   - Quick Start
   - 기본 사용법

2. **`QUICK_REFERENCE.md`** 북마크
   - 자주 쓰는 명령어
   - 조이스틱 조작법
   - 빠른 문제 해결

---

### 📘 **시스템을 깊이 이해하고 싶으신가요?**
**`ARCHITECTURE.md`** (필독!) - **36KB, 969줄**
- 🏗️ 완전한 시스템 아키텍처
- 📊 노드 다이어그램
- 🔄 데이터 플로우
- 🎯 컨트롤러별 상세 설명
- 🛠️ 새 컨트롤러 추가 가이드

---

### 🔍 **특정 정보를 찾으시나요?**

#### 토픽 정보가 필요하신가요?
**`TOPIC_INFORMATION.md`**
- 모든 토픽 리스트
- 메시지 타입
- 샘플 데이터

#### 빠른 명령어가 필요하신가요?
**`QUICK_REFERENCE.md`**
- 실행 명령어
- 디버깅 명령어
- 문제 해결 체크리스트

---

## 📄 전체 문서 목록

| 문서 | 크기 | 설명 | 대상 독자 |
|------|------|------|----------|
| **README.md** | 7.9KB | 프로젝트 개요 및 Quick Start | ⭐ 모든 사용자 |
| **ARCHITECTURE.md** | 36KB | 완전한 시스템 아키텍처 | ⭐⭐⭐ 개발자 필독 |
| **QUICK_REFERENCE.md** | 2.4KB | 빠른 참조 카드 | ⭐ 일상적 사용 |
| **TOPIC_INFORMATION.md** | 5.8KB | ROS2 토픽 레퍼런스 | ⭐⭐ 디버깅/개발 |
| **DOCS_INDEX.md** | (this) | 문서 네비게이션 | 📚 가이드 |

---

## 🗂️ 문서 구조

```
ros2_ws/
├── README.md                  ← 시작점
├── QUICK_REFERENCE.md         ← 일상 사용
├── ARCHITECTURE.md            ← 심화 학습 ⭐⭐⭐
├── TOPIC_INFORMATION.md       ← 토픽 참조
└── DOCS_INDEX.md              ← 여기!
```

---

## 🎓 학습 경로

### 초급 (첫 주)
1. ✅ README.md 읽기
2. ✅ 실제로 실행해보기
3. ✅ QUICK_REFERENCE.md 북마크

### 중급 (둘째 주)
1. ✅ ARCHITECTURE.md 1-5장 읽기
   - 시스템 개요
   - 패키지 구조
   - 컨트롤러 아키텍처
2. ✅ 노드 다이어그램 이해하기
3. ✅ 토픽 플로우 추적해보기

### 고급 (셋째 주 이후)
1. ✅ ARCHITECTURE.md 전체 읽기
2. ✅ 데이터 수집 시스템 분석
3. ✅ 새 컨트롤러 추가해보기
4. ✅ 코드 수정 및 기여

---

## 🔖 북마크 추천

### 매일 사용
- `QUICK_REFERENCE.md` - 명령어 모음

### 개발 시
- `ARCHITECTURE.md` - 시스템 설계
- `TOPIC_INFORMATION.md` - 토픽 참조

### 문제 해결 시
- `QUICK_REFERENCE.md` → 문제 해결 섹션
- `ARCHITECTURE.md` → 🐛 문제 해결 섹션

---

## 💡 팁

### 문서 검색하기
```bash
# 특정 키워드로 모든 문서 검색
grep -r "키워드" ~/ros2_ws/*.md

# 토픽 이름으로 검색
grep -r "/servo_node" ~/ros2_ws/*.md
```

### 문서 읽는 순서
```
처음: README → QUICK_REFERENCE
↓
깊이: ARCHITECTURE (Part 1-5)
↓
참조: TOPIC_INFORMATION
↓
전문: ARCHITECTURE (전체)
```

---

## 📝 문서 업데이트

**최종 업데이트**: 2026-01-28  
**문서 버전**: 1.0  
**작성자**: etri

### 변경 이력
- 2026-01-28: 초기 문서 세트 생성
  - ARCHITECTURE.md (969줄)
  - QUICK_REFERENCE.md
  - DOCS_INDEX.md

---

**Tip**: 이 문서를 시작 페이지로 북마크하세요! 📖
