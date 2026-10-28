# PiPER B안 리팩터 (piper-b-refactor)

LeRobot 0.4.3 체계(Robot / Teleoperator 인터페이스, config 등록, LeRobotDataset, 학습·추론 스크립트)는 그대로 두고,
**MotorsBus 계층만 제거**한 버전. Robot / Teleoperator 클래스는 piper_sdk를 얇은 헬퍼(`PiperArm`)로 직접 호출함.

```
기존: lerobot-record → PiperFollower → PiperMotorsBus(+tables.py) → piper_sdk
변경: lerobot-record → PiperFollower → PiperArm(단위 변환·클리핑만) → piper_sdk
```

## 바뀐 파일

| 파일 | 변경 |
|---|---|
| `motors/piper/piper.py`, `tables.py` | **삭제** (MotorsBus, Dynamixel 테이블) |
| `motors/piper/piper_arm.py` | **신규**. MotorsBus 아님. SDK 단위 변환, 범위 클리핑, 연결 확인, enable/disable/move_to |
| `robots/piper_follower/*` | piper_sdk 직접 사용, passive 모드, 안전 종료 |
| `teleoperators/piper_leader/*` | 명령 프레임 읽기 + 수신 확인 |
| `scripts/lerobot_record_singleport.py` | action 출처 선택(`--action_source`), 업로드 기본 off |
| `scripts/lerobot_record_HIL_joint_delta.py` | 리셋 구간 delta 텔레옵, 개입 플래그 저장, 무한 루프 수정, 업로드 기본 off |
| `examples/piper/piper_check.py`, `0_check.sh` | **신규**. 피드백/명령 프레임 확인, 추종 지연 측정, 역할 설정, 홈 이동 |

`wego_piper` 패키지는 더 이상 필요 없음 (`piper_sdk`만 필요).

## ⚠️ 단위 변경 (기존 데이터셋과 호환 안 됨)

| 키 | 기존 (WeGo) | 변경 |
|---|---|---|
| `joint1.pos` … `joint6.pos` | -100~100 정규화 | **degree** |
| `gripper.pos` | 0~100 정규화 (0~100mm 기준) | **mm** (0~70, SDK 기본 범위) |

정책은 데이터셋 통계로 정규화하므로 물리 단위를 그대로 써도 학습에 문제없음.
단, **기존 데이터셋/체크포인트를 새 코드로 실행하면 단위가 안 맞음** → 새로 수집하거나 변환 필요.
(변환: `deg = min + (v+100)/200*(max-min)` / 1000, 범위는 기존 하드코딩 캘리브레이션 값)

## 고친 문제

- 연결 직후 SDK가 주는 **기본값 0 프레임**을 실제 값으로 오인 → 첫 피드백 수신까지 대기, 없으면 에러
- 리더가 마스터 모드가 아니면 명령 프레임이 안 옴 → 기존엔 0 자세를 action으로 보내 팔로워가 영점으로 감. 이제 연결 시 에러
- 오래된(stale) 피드백/명령은 에러 처리 (`max_feedback_age_s`, `max_message_age_s`)
- `max_relative_target`: `sync_read` 미구현으로 깨지던 것 → SDK 피드백으로 구현, 단위는 deg/mm
- `use_degrees` / 4096 해상도 문제 → 정규화 자체 제거
- 종료 시: 홈 이동 성공했을 때만 토크 해제 (실패하면 팔 떨어짐 방지로 토크 유지)
- 그리퍼 명령 매 프레임 `0x03`(에러 클리어) → `0x01`(enable)
- `disable_torque_on_disconnect` 등 config 값이 실제로 반영됨
- single-port: `action = state` 복사 → **버스 위 리더 명령 프레임**을 action으로 (`--action_source=leader_command`, 기본)
- single-port: PC가 팔로워에 명령 안 보내도록 `passive` 강제 (기존 `clear_gripper` 우회 코드 삭제)
- HIL: 리셋 구간에서 리더 절댓값 → 팔로워 점프 → **delta 텔레옵**으로 변경
- HIL: 프레임별 `intervention` (1=사람, 0=정책) 저장. 정책은 이 키를 무시함
- HIL: 텔레옵 없이 정책만 쓸 때 리셋 구간 무한 루프 수정
- `push_to_hub` 기본 False, `private` 기본 True

참고: 이전 분석에서 `disable_torque`의 while 조건이 반대라고 했는데, SDK의 `DisablePiper()`는
"아직 enable된 모터가 있으면 True"를 반환해서 **기존 로직이 맞았음**. 정정.

## 실기에서 먼저 할 것 (순서대로)

1. `bash 1_init_can.sh` (USB 포트 매핑 수정 후)
2. 역할 설정 (한 번만): `python examples/piper/piper_check.py set-role --port can_leader --role master` /
   `--port can_follower --role slave` → 마스터→슬레이브 변경 시 전원 재시작
3. **명령 프레임 확인**: `bash 0_check.sh can0` 하면서 리더를 손으로 움직이기
   - `command frames: yes` → single-port에서 `leader_command` 사용 가능
   - `NO` → single-port는 `--action_source=follower_state`(구 방식)만 가능, 또는 dual-port 녹화 사용
   - 출력되는 `est. follow lag`가 리더→팔로워 추종 지연
4. 리더 설정 확인: `PiperLeaderConfig.enable_on_connect`(기본 True, 기존 동작 유지).
   드래그가 뻑뻑하면 `--teleop.enable_on_connect=false`.
   리더가 멈춰 있을 때 명령 프레임을 안 보내서 stale 에러가 나면 `--teleop.max_message_age_s=null`
5. `2_teleop.sh`로 텔레옵 확인 → 녹화

## 주요 config

`--robot.*` (piper_follower): `port`, `speed_rate`(30), `gripper_effort`(1000), `max_relative_target`(deg/mm),
`passive`, `enable_on_connect`, `home_on_connect`, `home_on_disconnect`, `disable_torque_on_disconnect`,
`home_position`, `max_feedback_age_s`, `cameras`

`--teleop.*` (piper_leader): `port`, `source`(`command`|`feedback`), `enable_on_connect`, `max_message_age_s`

## 검증 범위

실기 없이 piper_sdk 시뮬레이터로 확인: 단위 변환·클리핑, 연결 실패 처리, passive 모드, 리더 stale 처리,
single-port 두 모드(leader_command: action≠state / follower_state: action=state), HIL 녹화 + intervention 저장,
리셋 구간 delta(기존 60° 점프 → 현재 위치에서 시작). **실기 CAN 동작(특히 3번)은 직접 확인 필요.**
