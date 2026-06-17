# robot_description / robot_state_publisher 마이그레이션 — 구현 체크리스트

> **진행 상태 (2026-06-16)**: Phase 0~3 코드 구현 + 빌드 완료(경고 0). 모든 신규 플래그는 기본 off로
> legacy 동작을 100% 보존하며, RSP 활성화/TF 억제는 opt-in. **런타임 검증(아래 수용 기준)은 미수행** —
> `enable_robot_state_publisher=true` → `disable_urdf_tf=true` 순서로 single_mode에서 먼저 검증할 것 (INV-4).
>
> **주요 파일**: 게이팅·URDF 캐시 [base.cpp](../../cloisim_ros_base/src/base.cpp) ·
> owner 선출/RSP 기동 [bringup.cpp](../../cloisim_ros_bringup/src/bringup.cpp) ·
> owner 게이팅 [joint_control.cpp](../../cloisim_ros_joint_control/src/joint_control.cpp),
> [micom.cpp](../../cloisim_ros_micom/src/micom.cpp)

## 핵심 불변식 (전 단계 공통)

- **INV-1**: robot당 `robot_description` 발행자는 런타임에 정확히 1개
- **INV-2**: 소유권 우선순위 — `joint_control` 활성 시 owner, 비활성일 때만 `micom` fallback
- **INV-3**: URDF kinematic TF와 runtime/world/navigation TF 경로는 분리 유지
- **INV-4**: 검증 완료 전까지 기존 `SetTf`/`SetStaticTf2` 경로 제거 금지 (gating 플래그로만 비활성)

---

## Phase 0 — Gating 인프라

**작업**
- [x] `bringup` 파라미터 + 런치 인자 추가: `enable_robot_state_publisher`(기본 `false`), `disable_urdf_tf`(기본 `false`) — [bringup_launch.py](../../cloisim_ros_bringup/launch/bringup_launch.py), `make_bringup_list` 전파
  - 소유권은 별도 `urdf_tf_owner` 파라미터 대신 model 구조(JOINTCONTROL 존재 여부)로 자동 결정 — INV-2 정책 고정
- [x] `Base`에 `disable_urdf_tf_` gating 플래그 추가 (기본 `false` = 기존 동작 유지) + URDF 캐시(`robot_description_`)와 `RequestRobotDescription()` 헬퍼를 Base로 통합

**수용 기준**
- 모든 플래그 기본값에서 **현재 동작과 100% 동일** (회귀 없음) ✓ 기본 off
- 플래그 미설정 시 빌드/런타임 경고 없음 ✓ 빌드 경고 0

---

## Phase 1 — bringup 소유자 선출

**작업**
- [x] `bringup`에서 robot별 소유권 결정 — `parse_target_model`에서 `item_list.isMember("JOINTCONTROL")`로 model 단위 판별
- [x] 선출된 owner에게만 발행 신호 전달 — micom 노드에 `publish_robot_description = !model_has_joint_control` 파라미터 주입
- [x] `enable_robot_state_publisher=true`일 때 robot당 `robot_state_publisher` 1개 기동 — bringup이 런타임 동적 발견 구조라 정적 런치 대신 **in-process RSP 노드**(`make_robot_state_publisher`)를 owner namespace로 생성, owner URDF를 `robot_description` 파라미터로 주입, 기존 executor 생명주기(`g_node_map_list`)에 편입. RSP 활성 시 owner 노드는 토픽 발행 억제(RSP가 단일 소유)

**수용 기준**
- `joint_control` 존재 시 → owner=`joint_control`, `micom`은 미발행
- `joint_control` 부재 시 → owner=`micom`
- robot당 `/robot_description` 토픽 latch 1개만 존재 (`ros2 topic info`로 publisher count == 1)
- 동일 frame을 두 노드가 동시에 TF 발행하지 않음 (`/tf_static` 중복 frame 없음)

---

## Phase 2 — micom fallback publisher

**작업**
- [x] `micom`에 `robot_description` 발행 경로 추가 (owner이며 RSP 비활성일 때만) — Base `RequestRobotDescription()` 사용, `transient_local` QoS publisher
- [x] owner 노드 토픽 발행을 `enable_robot_state_publisher`로 게이팅 (RSP 활성 시 RSP가 토픽 소유, owner는 URDF만 캐시) — joint_control/micom 공통
- [ ] **(런타임 검증 필요)** `joint_states` 정합성 — micom은 joint_states 미발행. owner=micom인 robot은 joint가 없는 케이스(고정 링크)여야 RSP가 정적 TF만 생성하면 충분. 움직이는 joint가 있는 micom 로봇은 검증에서 확인 필요

**수용 기준**
- owner=`micom`일 때만 `/robot_description` 발행, owner=`joint_control`일 때 침묵
- `robot_state_publisher`가 micom URDF로 정상 kinematic TF 트리 생성
- owner 전환 시 (joint_control 등장/소멸) **중복 발행 구간 없음**

---

## Phase 3 — 단계적 TF 비활성화

**작업**
- [x] `disable_urdf_tf=true`일 때 `Base::SetStaticTransforms`(시뮬레이터 정적 변환 = 센서 마운트)를 우회 — RSP가 대체 발행. 코드 구현 완료, 기본 off
- [x] runtime/world/navigation frame은 **비활성화 대상에서 제외** (INV-3) — micom odom TF(`odom→base_footprint`), `GenerateTF`(동적), world/ground_truth TF는 게이팅하지 않음. `base_footprint→base_link`도 nav↔URDF root 브리지이므로 유지
- [ ] **(런타임 검증 필요)** `single_mode`에서 `enable_robot_state_publisher=true` 먼저 → TF 트리 비교 → `disable_urdf_tf=true` 적용 → 재비교 → multi-namespace 확대

**수용 기준**
- `disable_urdf_tf` 전후 `/tf`+`/tf_static` 트리가 **frame 누락 없이 동일** (robot_state_publisher가 대체 발행)
- `ros2 run tf2_tools view_frames` 결과에 끊긴 frame chain 없음
- `single_mode` 통과 후에만 multi-namespace 적용

> **주의**: joint_control의 동적 joint TF(`GenerateTF`)는 현재 `disable_urdf_tf`로 게이팅하지 않음.
> RSP가 joint_states로 동일 TF를 발행하므로 검증 시 중복 가능 — 확인 후 별도 게이팅 추가 검토.

---

## 전체 회귀 게이트 (각 Phase 종료 시)

- [ ] 빌드: 현재 표준/경고 설정에서 신규 경고 0
- [ ] `/tf_static` 중복 frame 0
- [ ] robot당 `/robot_description` publisher == 1
- [ ] TF 트리 disconnected frame 0
- [ ] 롤백 경로 존재: 모든 gating 플래그 기본값 복귀 시 legacy 동작 완전 복원
