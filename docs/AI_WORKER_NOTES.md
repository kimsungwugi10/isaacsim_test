# ROBOTIS AI Worker — 문서 분석 노트

출처: https://docs.robotis.com/docs/systems/aiworker/introduction 및 하위 페이지 60개 (2026-10-29 기준).
목적: AI Worker **FFW-BG2** 실물과 노트북의 **Isaac Sim**을 함께 쓰는 작업의 참고 자료.

## 1. 보유 모델: FFW-BG2

- 한국 전용 모델. 이동 베이스가 없는 고정형 (SG2는 스워브 드라이브 이동형).
- 19 DOF: 팔 7 DOF x 2, 그리퍼 1 DOF x 2 (RH-P12-RN), 머리 2 DOF, 리프트 1 DOF.
- LiDAR 없음, 배터리 없음. AC 전원 (SMPS 24VDC 80A 1,920W). 내부 라우터 없음 (호스트 PC와 이더넷 직결, VR 사용 시 라우터 연결 권장).
- 치수 604x564x1,607 mm, 85 kg. 팔 도달 거리 641 mm(손목까지), 가반하중 단일 팔 3 kg (피크 5 kg).
- 센서: 머리 Stereolabs ZED Mini, 손목 Intel RealSense D405 x2.
- 컴퓨터: NVIDIA Jetson AGX Orin 32GB, JetPack 6.2, Docker 안의 ROS 2 Jazzy (Ubuntu 24.04).
- 액추에이터: DYNAMIXEL Y(팔 1~6, 리프트), P(팔 7), X(머리). RS-485 4 Mbps, Protocol 2.0, U2D2.
- 도면(PDF/DWG/STEP): https://www.robotis.com/service/download.php?no=2214 / 2231 / 2227

### 관절 이름과 범위

| ID | 관절 | 이름 | 범위 |
|---|---|---|---|
| 1~7 | 오른팔 | arm_r_joint1 ~ arm_r_joint7 | J1 ±180°, J2 -190~10°, J3 ±180°, J4 -170~65°, J5 ±180°, J6 ±105°, J7 -120~90° |
| 8 | 오른 그리퍼 | gripper_r_joint1 | 0 ~ 107.6 mm |
| 31~37 | 왼팔 | arm_l_joint1 ~ arm_l_joint7 | J2 -10~190°, J7 -90~120° (나머지는 오른팔과 동일) |
| 38 | 왼 그리퍼 | gripper_l_joint1 | 0 ~ 107.6 mm |
| 61 | 머리 피치 | head_joint1 | -50 ~ 30° |
| 62 | 머리 요 | head_joint2 | -20 ~ 20° |
| 81 | 리프트 | lift_joint | 0 ~ 500 mm |

## 2. 소프트웨어 구조

- ros2_control, 100 Hz 제어 루프. 기본은 위치 모드.
- 컨트롤러와 입력 토픽:
  - `arm_l_controller` / `arm_r_controller` (8 DOF, 그리퍼 포함, JointTrajectoryController)
    ← `/leader/joint_trajectory_command_broadcaster_{left,right}/joint_trajectory`
  - `head_controller` ← `/leader/joystick_controller_left`
  - `lift_controller` ← `/leader/joystick_controller_right`
  - `joint_state_broadcaster` → `/joint_states`
- 컨트롤러 설정: `ai_worker/ffw_bringup/config/ffw_bg2_rev4_follower/ffw_bg2_follower_ai_hardware_controller.yaml`
- 미들웨어: v2.0.0부터 **Zenoh** (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`). 로봇 실행 전에 Zenoh 데몬 (`zenohd`, 포트 7447)이 필요.
  - 외부 PC 연결: `export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;mode="client";connect/endpoints=["tcp/<robot_ip>:7447"]'`
  - 컨테이너 기본 `ROS_DOMAIN_ID=30`. 모든 컨테이너에서 같은 값을 써야 함.
- 로봇 PC 접속: `ssh robotis@ffw-<시리얼번호>.local`. **Orin에서 `apt upgrade` 금지.**
- 사용자 PC 요구 사항: Ubuntu, Docker Engine, NVIDIA 드라이버 (권장 nvidia-driver-570-server-open, CUDA 12.8), NVIDIA Container Toolkit.

### 주요 실행 명령 (ai_worker 컨테이너 안)

```bash
ffw_bg2_ai            # 리더(LG2) + 팔로워(BG2) 동시 실행 = ros2 launch ffw_bringup ffw_bg2_ai.launch.py
ffw_bg2_follower_ai   # 팔로워만 (추론 시 사용). 옵션: launch_cameras:=false init_position:=false
ffw_lg2_leader_ai     # 리더만
ros2 launch ffw_bringup ffw_bg2_follower_ai_gazebo.launch.py   # Gazebo
ros2 launch ffw_description ffw_bg2_rev4.launch.py             # RViz 모델만
ros2 launch ffw_moveit_config moveit.launch.py use_sim:=true   # MoveIt (Gazebo와 함께)
```

- 원격조종: 양손 트리거를 2초 이상 눌러야 팔로워가 움직이기 시작함. 같은 동작으로 일시정지.
- 처음 전원을 켜면 토크 OFF 상태. Remote E-STOP의 A 버튼을 눌러 안전 잠금 해제.

## 3. 오픈소스 저장소 (ROBOTIS-GIT)

| 저장소 | 용도 |
|---|---|
| `ai_worker` (jazzy 브랜치) | 로봇 description(URDF/MJCF), bringup, 컨트롤러, 내비게이션, 원격조종 |
| `cyclo_lab` | **Isaac Lab 기반 시뮬레이션**, USD 로봇 에셋, RL/IL 태스크 |
| `cyclo_intelligence` | 모방학습 웹 UI (데이터 녹화, LeRobot 변환, HF 업로드, 추론). 포트 7080 |
| `cyclo_control` | QP 기반 모션 컨트롤러 (MoveL / MoveJ / bimanual / VR) |
| `robotis_applications` | ROBOTIS Vuer (Meta Quest 3 VR 원격조종) |
| `cyclo_solution` | Isaac ROS cuMotion 연동 |
| `physical_ai_tools` | 이전 모방학습 도구 (레거시, 더 이상 업데이트 안 됨) |

- 로봇 쪽 운영 도구: **Cyclo Manager** (`pip install cyclo-manager`, `cyclo_manager up`). 웹 UI 포트 3000, API 포트 8081.

## 4. Isaac Sim / Isaac Lab (cyclo_lab)

- USD 에셋: `cyclo_lab/source/cyclo_lab/data/robots/FFW`
- IsaacLab v2.2.0 이상 필요. Docker 이미지 구성: Isaac Sim 5.1.0, Isaac Lab v2.3.0, CycloneDDS 0.10.2, robotis_dds_python, LeRobot 0.3.3 (`~/lerobot_env`).

```bash
git clone --recurse-submodules https://github.com/ROBOTIS-GIT/cyclo_lab.git
cd cyclo_lab
./docker/container.sh start   # enter / stop / logs / clean
./docker/container.sh enter
```

- **BG2 강화학습 예제 (Reach)**:
  ```bash
  python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Reach-FFW-BG2-v0 --num_envs=512 --headless
  python scripts/reinforcement_learning/rsl_rl/play.py --task Cyclo-Reach-FFW-BG2-Play-v0 --num_envs=16
  ```
- 모방학습 예제는 **SG2 전용**으로만 문서화됨 (`Cyclo-Real-Pick-Place-FFW-SG2-v0`):
  `record_demos.py` → `mimic/action_data_converter.py`(ik) → `mimic/annotate_demos.py` → `mimic/generate_dataset.py` → `action_data_converter.py`(joint) → `data_converter/isaaclab2lerobot.py` → 학습 → `inference/inference_demos.py`
- Isaac Sim ↔ ROS 2 브리지 예제는 **SH5 전용** (`scripts/sim2real/bringup/sh5_dds_bringup.py --domain_id 30`).
  CycloneDDS로 리더 JointTrajectory 토픽과 `/cmd_vel`을 구독하고 `/joint_states`, `/tf`를 발행. BG2용 브리지는 문서에 없음.
- cuMotion용 XRDF 충돌 구(sphere)는 Isaac Sim의 Lula Robot Description Editor로 제작.

## 5. 모방학습 파이프라인 (Cyclo Intelligence)

1. 원격조종으로 시연 녹화 (rosbag2/MCAP, `/workspace/rosbag2`)
2. Data Tools: 리뷰, 삭제, 병합, LeRobot v2.1/v3.0 변환 (`/workspace/lerobot`), Hugging Face 업로드
3. 외부 서버에서 `lerobot-train`으로 학습
4. HF에서 모델을 `/workspace/model`로 다운로드
5. Model Inference: 먼저 3D Sim Deploy로 확인한 뒤 Real Robot Deploy

- 지원 정책: ACT, Diffusion, SmolVLA, XVLA, Pi0, Pi0.5, GR00T N1.7, MolmoAct2, FastWAM, VLA-JEPA
- 처음이면 ACT 권장. SmolVLA는 AI Worker 상태 차원이 16이라 `--policy.type` + `--policy.pretrained_path`로 학습해야 함.
- Diffusion은 모든 카메라 해상도가 같아야 하므로 AI Worker에서는 카메라 하나만 선택해서 녹화.
- 미리 빌드된 ACT/GR00T 추론 이미지는 Blackwell GPU 환경만 지원.

## 6. BG2 사용 시 주의

- Autonomy Studio는 `ffw_sg2_rev1`만 지원 (BG2 불가).
- VR 원격조종에서 BG2는 sg2 명령을 사용.
- 내비게이션(Nav2), Jog는 이동 베이스가 있는 모델용이라 BG2에는 해당 없음.

## 7. 원문 페이지 목록

- 소개: https://docs.robotis.com/docs/systems/aiworker/introduction
- 하드웨어 사양: https://docs.robotis.com/docs/systems/aiworker/specifications/hardware
- 소프트웨어 사양: https://docs.robotis.com/docs/systems/aiworker/specifications/software
- 셋업 (HW/SW/Zenoh): https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/setup_overview/
- 원격조종: https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/teleoperation
- VR 원격조종: https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/vr_teleoperation
- Cyclo Manager: https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/cyclo_manager/
- 시뮬레이션 개요: https://docs.robotis.com/docs/systems/aiworker/simulation/
- Isaac Sim/Lab: https://docs.robotis.com/docs/systems/aiworker/simulation/isaac_sim_lab
- Gazebo: https://docs.robotis.com/docs/systems/aiworker/simulation/gazebo
- 모방학습: https://docs.robotis.com/docs/systems/aiworker/imitation_learning/
- Cyclo Control: https://docs.robotis.com/docs/systems/aiworker/advanced_features/cyclo_control
- 오픈소스: https://docs.robotis.com/docs/systems/aiworker/resources/open_source
- Isaac VR 원격조종 (SH5 DDS 브리지): https://docs.robotis.com/docs/systems/aiworker/resources/technical_story/isaac_vr_teleoperation
- Isaac ROS cuMotion: https://docs.robotis.com/docs/systems/aiworker/resources/technical_story/isaac_cumotion
- 전체 목록: https://docs.robotis.com/sitemap.xml (`aiworker` 포함 URL)
