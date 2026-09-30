#include "mode_tdcn_CLAW.h"
#include "mode_tdcn_rtwtypes.h"
#include "mode_tdcn_CLAW_types.h"
#include "mode_tdcn_CLAW_private.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// [Sejong/TDCN] M_PI: SITL(glibc)에선 math.h가 제공하나 ARM newlib(__STRICT_ANSI__)은 미정의
//                     → 하드웨어(Pixhawk4) 빌드 위생용 가드 정의 (로직 무변경)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern volatile uint8_t Arming;

// =========================================================
// [BSC 제어 변수 정의] - 내부 연산용이므로 double 사용
// =========================================================

// 1. 상수 정의 (double 정밀도)
#define DT           0.0025   // 제어 주기 (400Hz) [Sejong]
#define D2R          0.017453292519943295

// 2. 상태 변수 및 제어 입력
double STV[12];         // State Vector [u,v,w, p,q,r, phi,theta,psi, x,y,z]
double XTV[6];          // Inertial Vector [vx,vy,vz, wx,wy,wz]
double Del_Control[4];  // Control Input (Thrust, Roll, Pitch, Yaw)

// 3. 내부 연산 변수 (Trim 값 및 궤적)
double STV_trim[12] = { 0.0, };
double pos_des_dot_trim[3] = { 0.0, };
double Xtraj[4] = { 0 }, dXtraj[4] = { 0 }, ddXtraj[4] = { 0 };
double TV_BSC[4] = { 0 };

// 4. IBSC 전용 전역 변수
double Est_Acc_Body[6] = { 0.0, };           // [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot]
double Acc_0[4] = { 0.0, };                  // Inertial/Euler Frame Acceleration
double Prev_CTRL[4] = { 0.0, 0.0, 0.0, 0.0}; // Previous Control Input (u_t-1)
double STV_OLD[3] = { 0.0, };                // Previous Body Velocity (for differentiation)

// 5. 임시 변수들
double Ph, Th, Ps, Sph, Cph, Sth, Cth, Sps, Cps;
double CTML[3][3], CTMA[3][3], CTML_inv[3][3], CTMA_inv[3][3];
double uv_des[3], pos_dot_des[3];
double z1[4], z2[4], alpha[4], alpha_dot[4], f[4];
double G_mat[4][4], G_mat_inv[4][4];

// 6. 가속도 추정기용 버퍼
#define BUFFER_SIZE 3 
static double state_buffer[4][BUFFER_SIZE]; // 1:p, 2:q, 3:r (0번 인덱스는 미사용)
static bool buffer_filled = false;
    
// 7. IBSC 안정화를 위한 게인 및 필터 계수 추가
#define IBSC_DELTA_GAIN  0.1    // 증분 반영 비율 (0.1 ~ 0.3 권장)
static double Est_Acc_Body_Filtered[6] = { 0.0, };

// =========================================================
// [헬퍼 함수 정의]
// =========================================================

// 1. 가속도 추정 함수 (3-Point Polynomial Fitting)
double Estimate_Accel_Hermite(int axis, double current_val, double dt) 
{
  // 버퍼 시프트
  state_buffer[axis][2] = state_buffer[axis][1];
  state_buffer[axis][1] = state_buffer[axis][0];
  state_buffer[axis][0] = current_val;
    
  if (!buffer_filled) {
    static int init_count = 0;
    if (axis == 3) init_count++;
    if (init_count > BUFFER_SIZE + 5) buffer_filled = true;
    return 0.0;
  }
    
  // 3-Point Difference (Backward)
  double deriv = (3.0 * state_buffer[axis][0] - 4.0 * state_buffer[axis][1] + 1.0 * state_buffer[axis][2]) / (2.0 * dt);

  return deriv;
}

// 2. 거리 변환 (Lat/Lon -> Meter)
double Lat2m(double lat)
{
  double lat_rad = lat * D2R;
  return 111132.92 - 559.82 * cos(2.0 * lat_rad) + 1.175 * cos(4.0 * lat_rad) - 0.0023 * cos(6.0 * lat_rad);
}
double Lon2m(double lat)
{
  double lat_rad = lat * D2R;
  return 111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad) + 0.118 * cos(5.0 * lat_rad);
}

// =========================================================

/* Block signals (default storage) */
B_CLAW_T CLAW_B;

/* Block states (default storage) */
DW_CLAW_T CLAW_DW;

/* External inputs (root inport signals with default storage) */
ExtU_CLAW_T CLAW_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_CLAW_T CLAW_Y;

/* Real-time model */
static RT_MODEL_CLAW_T CLAW_M_;
RT_MODEL_CLAW_T *const CLAW_M = &CLAW_M_;


// [수정] 전역 변수나 정적 변수로 기준점(Home) 저장
double Target_X = 0.0, Target_Y = 0.0, Target_Z = 0.0;
double Home_Lat = 0.0, Home_Lon = 0.0, Home_Alt = 0.0, Home_Yaw = 0.0;
static bool home_init = false;

// [Sejong]
double Err_N, Err_E, Err_D;

// [Sejong] 
// CLAW_reset: disarm/재이륙 대비 추가 함수
// home_init을 false로 → 다음 FOLLOW 진입 시 현재 위치를 새 home으로 재캡처
// mode_tdcn.cpp의 init()·disarm 엣지에서 호출. 선언은 CLAW.h
void CLAW_reset(void)
{
  home_init = false;
}


/* Model step function */
void CLAW_step(void)
{
  // ======================================================================
  // [PART 1] GPS -> NED 좌표 변환 (Gauss-Newton 삭제됨)
  // ======================================================================

  // 1. 현 GPS 입력
  // [Sejong] 
  double Cur_Lat = (double)CLAW_U.Cur_Pos.x;   // Latitude  [deg]
  double Cur_Lon = (double)CLAW_U.Cur_Pos.y;   // Longitude [deg]
  double Cur_Alt = -(double)CLAW_U.Cur_Pos.z;  // Down [m]

  // [보완] Arming이 0(Disarmed)일 때는 제어 로직을 완전히 초기화하고 중단
  if (Arming == 0) {
    // 1. IBSC 누적 제어량 및 적분기 강제 리셋
    // for (int i = 0; i < 4; i++) {
      // Prev_CTRL[i] = 0.0;
      // TV_BSC[i] = 0.0;    // 위치 제어 적분기 리셋
      // Xtraj[i] = 0.0;     // 커맨드 필터 초기화
      // dXtraj[i] = 0.0;
      // ddXtraj[i] = 0.0;
    // }
    
    // 3. 출력값 안전값(호버링 바이어스)으로 강제 고정
    CLAW_Y.v_cmd.cmd_height = 0.0f; // Mission Planner 호버링 기준
    CLAW_Y.v_cmd.cmd_roll   = 0.0f;
    CLAW_Y.v_cmd.cmd_pitch  = 0.0f;
    CLAW_Y.v_cmd.cmd_yaw    = 0.0f;

    // [핵심 추가] Arming 대기 중에는 현재 위치/각도를 목표값으로 계속 추적
    // Xtraj[0] = STV[11]; // 현재 고도
    // Xtraj[1] = STV[6];  // 현재 Roll
    // Xtraj[2] = STV[7];  // 현재 Pitch
    // Xtraj[3] = STV[8];  // 현재 Yaw
    
    // 2. 가속도 필터 및 버퍼 초기화
    for (int i = 0; i < 6; i++) Est_Acc_Body_Filtered[i] = 0.0;
    buffer_filled = false;
    
    return; // 아래의 제어 연산(IBSC)을 수행하지 않고 나감
  }

  // 2. 최초 실행 시 현재 위치를 기준점(Home, 0,0,0)으로 설정
  if (!home_init) {
    // GPS 데이터가 유효할 때만 초기화 수행
    if (fabs(Cur_Lat) > 1.0 && fabs(Cur_Lon) > 1.0) {
      Home_Lat = Cur_Lat;
      Home_Lon = Cur_Lon;
      Home_Alt = Cur_Alt;

      // 초기 목표 지점을 현재 위치로 설정 (거리 0m에서 시작)
      // [Currnet Input is LLD]
      CLAW_U.Dest_poti_i.y = (real32_T)Cur_Lat;  // [deg]
      CLAW_U.Dest_poti_i.x = (real32_T)Cur_Lon;  // [deg]
      CLAW_U.Dest_poti_i.z = -(real32_T)Cur_Alt; // [m] (Down)

      // 미분기 및 속도 추정 초기화
      CLAW_DW.UD_DSTATE = 0.0;
      CLAW_DW.UD_DSTATE_i = 0.0;
      CLAW_DW.UD_DSTATE_p = 0.0;

      buffer_filled = false;
      for (int k = 0; k < BUFFER_SIZE; k++) {
        state_buffer[1][k] = STV[3]; // 현재 p값으로 채움
        state_buffer[2][k] = STV[4]; // 현재 q값으로 채움
        state_buffer[3][k] = STV[5]; // 현재 r값으로 채움
      }
              
      for (int k = 0; k < 6; k++) 
        Est_Acc_Body_Filtered[k] = 0.0;
      // [핵심] Pitch 포화 방지를 위한 적분기 및 필터 리셋
      for(int i=0; i<4; i++) {
        TV_BSC[i] = 0.0;
        Xtraj[i] = 0.0;
        dXtraj[i] = 0.0;
        ddXtraj[i] = 0.0;
        Prev_CTRL[i] = 0.0;
      }
      
      // prev_CTRL[0] = 0.5;

      // Xtraj[1] = STV[6];  // 현재 Roll
      // Xtraj[2] = STV[7];  // 현재 Pitch
      // Xtraj[3] = STV[8];  // 현재 Yaw

      home_init = true; // 모든 초기화가 끝난 후 플래그 ON
    }
  }
      
  // 2. 목표 입력
  // [Sejong]
  double Dest_N = (double)CLAW_U.Dest_poti_i.x;  // North (m)
  double Dest_E = (double)CLAW_U.Dest_poti_i.y;  // East  (m)
  double Dest_U = -(double)CLAW_U.Dest_poti_i.z; // Down  (m, home 기준)

  // 3. 변환 수행 (Lat/Lon -> NED Meter)
  if (home_init) {
    // 위도 차이 -> North 거리 변환
    STV[9] = (Cur_Lat - Home_Lat) * Lat2m(Home_Lat);
    // 경도 차이 -> East 거리 변환
    STV[10] = (Cur_Lon - Home_Lon) * Lon2m(Home_Lat);
    // 고도 차이 -> Down 거리 변환 (Z는 아래가 양수이므로 부호 반대)
    STV[11] = (Cur_Alt - Home_Alt);
  }
  else {
    // 초기화 전에는 0으로 유지
    STV[9] = 0.0;
    STV[10] = 0.0;
    STV[11] = 0.0;
  }

  // 2. 속도 데이터 (Velocity Estimation)
  // [Sejong]
  // [EKF 버전] mode_tdcn.cpp가 주입한 EKF 속도(NED, m/s)를 관성속도 XTV에 직접 사용
  // XTV는 내부 Up 규약(STV[11]=Cur_Alt-Home_Alt, +위)이므로 Z만 부호 반전
  XTV[0] =  (double)CLAW_U.Vel_N;   // North (m/s)
  XTV[1] =  (double)CLAW_U.Vel_E;   // East  (m/s)
  XTV[2] =  -(double)CLAW_U.Vel_D;   // Down    (m)


  // 3. IMU 데이터 매핑
  // [Sejong]
  STV[3] = (double)CLAW_U.p;							          // IMU(rad/s)
  STV[4] = (double)CLAW_U.q;							          // IMU(rad/s)
  STV[5] = (double)CLAW_U.r;							          // IMU(rad/s)
  STV[6] = (double)CLAW_U.Roll;						          // IMU(rad)
  STV[7] = (double)CLAW_U.Pitch;						        // IMU(rad)
  STV[8] = (double)CLAW_U.DR_heading_f.DR_heading;	// IMU(rad)

  // 3. 좌표 변환 (Body <-> Inertial)
  Ph = STV[6]; Th = STV[7]; Ps = STV[8];
  Sph = sin(Ph); Cph = cos(Ph); 
  Sth = sin(Th); Cth = cos(Th); 
  Sps = sin(Ps); Cps = cos(Ps);

  CTML[0][0] = Cth * Cps;                       CTML[0][1] = Cth * Sps;                       CTML[0][2] = -Sth;
  CTML[1][0] = Sph * Sth * Cps - Cph * Sps;     CTML[1][1] = Sph * Sth * Sps + Cph * Cps;     CTML[1][2] = Sph * Cth;
  CTML[2][0] = Cph * Sth * Cps + Sph * Sps;     CTML[2][1] = Cph * Sth * Sps - Sph * Cps;     CTML[2][2] = Cph * Cth;

  // 역행렬
  CTML_inv[0][0] = CTML[0][0]; CTML_inv[0][1] = CTML[1][0]; CTML_inv[0][2] = CTML[2][0];
  CTML_inv[1][0] = CTML[0][1]; CTML_inv[1][1] = CTML[1][1]; CTML_inv[1][2] = CTML[2][1];
  CTML_inv[2][0] = CTML[0][2]; CTML_inv[2][1] = CTML[1][2]; CTML_inv[2][2] = CTML[2][2];

  if (fabs(Cth) < 1e-6) Cth = 1e-6;
  CTMA_inv[0][0] = 1.0; CTMA_inv[0][1] = Sph * Sth / Cth; CTMA_inv[0][2] = Cph * Sth / Cth;
  CTMA_inv[1][0] = 0.0; CTMA_inv[1][1] = Cph;         CTMA_inv[1][2] = -Sph;
  CTMA_inv[2][0] = 0.0; CTMA_inv[2][1] = Sph / Cth;     CTMA_inv[2][2] = Cph / Cth;

  // Body Frame 속도 계산
  STV[0] = CTML[0][0] * XTV[0] + CTML[0][1] * XTV[1] + CTML[0][2] * XTV[2];
  STV[1] = CTML[1][0] * XTV[0] + CTML[1][1] * XTV[1] + CTML[1][2] * XTV[2];
  STV[2] = CTML[2][0] * XTV[0] + CTML[2][1] * XTV[1] + CTML[2][2] * XTV[2];

  // Euler Rates 2026.02.04 수정(추가)
  XTV[3] = CTMA_inv[0][0] * STV[3] + CTMA_inv[0][1] * STV[4] + CTMA_inv[0][2] * STV[5];
  XTV[4] = CTMA_inv[1][0] * STV[3] + CTMA_inv[1][1] * STV[4] + CTMA_inv[1][2] * STV[5];
  XTV[5] = CTMA_inv[2][0] * STV[3] + CTMA_inv[2][1] * STV[4] + CTMA_inv[2][2] * STV[5];

  // ======================================================================
  // [PART 3] IBSC 제어 로직 (이하 동일)
  // ======================================================================

  // 1) 선형 가속도 (Linear Acc) 추정
  // raw_acc_angular[0] = (STV[0] - STV_OLD[0]) / DT; // u_dot
  // raw_acc_angular[1] = (STV[1] - STV_OLD[1]) / DT; // v_dot
  // raw_acc_angular[2] = (STV[2] - STV_OLD[2]) / DT; // w_dot

  // 2) 각가속도 (Angular Acc) 추정
  // Est_Acc_Body[3] = Estimate_Accel_Hermite(1, STV[3], DT); // p_dot
  // Est_Acc_Body[4] = Estimate_Accel_Hermite(2, STV[4], DT); // q_dot
  // Est_Acc_Body[5] = Estimate_Accel_Hermite(3, STV[5], DT); // r_dot
  // [수정] 가속도 제한 상수 추가
  #define ACCEL_LIMIT_LINEAR  20.0  // m/s^2 (약 2G)
  #define ACCEL_LIMIT_ANGULAR 10.0  // rad/s^2

  double ACCEL_LPF_ALPHA = 0.5;    // 시뮬레이션과 동일한 필터 계수
  // 1) 선형 가속도 추정 (u_dot, v_dot, w_dot)
  double raw_acc_linear[3];
  raw_acc_linear[0] = (STV[0] - STV_OLD[0]) / DT;
  raw_acc_linear[1] = (STV[1] - STV_OLD[1]) / DT;
  raw_acc_linear[2] = (STV[2] - STV_OLD[2]) / DT;

  // 2) 각가속도 추정 (p_dot, q_dot, r_dot)
  double raw_acc_angular[3];
  raw_acc_angular[0] = Estimate_Accel_Hermite(1, STV[3], DT);
  raw_acc_angular[1] = Estimate_Accel_Hermite(2, STV[4], DT);
  raw_acc_angular[2] = Estimate_Accel_Hermite(3, STV[5], DT);

  for (int i = 0; i < 3; i++) {
    // 선형 가속도 제한
    if (raw_acc_linear[i] >  ACCEL_LIMIT_LINEAR) raw_acc_linear[i] =  ACCEL_LIMIT_LINEAR;
    if (raw_acc_linear[i] < -ACCEL_LIMIT_LINEAR) raw_acc_linear[i] = -ACCEL_LIMIT_LINEAR;

    // 각가속도 제한
    if (raw_acc_angular[i] >  ACCEL_LIMIT_ANGULAR) raw_acc_angular[i] =  ACCEL_LIMIT_ANGULAR;
    if (raw_acc_angular[i] < -ACCEL_LIMIT_ANGULAR) raw_acc_angular[i] = -ACCEL_LIMIT_ANGULAR;

    Est_Acc_Body_Filtered[i] = (1.0 - ACCEL_LPF_ALPHA) * Est_Acc_Body_Filtered[i] + ACCEL_LPF_ALPHA * raw_acc_linear[i];
    Est_Acc_Body_Filtered[i+3] = (1.0 - ACCEL_LPF_ALPHA) * Est_Acc_Body_Filtered[i+3] + ACCEL_LPF_ALPHA * raw_acc_angular[i];
  }

  // 3) 관성/오일러 프레임 가속도 변환
  Acc_0[0] = CTML_inv[2][0] * Est_Acc_Body_Filtered[0] + CTML_inv[2][1] * Est_Acc_Body_Filtered[1] + CTML_inv[2][2] * Est_Acc_Body_Filtered[2];

  Acc_0[1] = CTMA_inv[0][0] * Est_Acc_Body_Filtered[3] + CTMA_inv[0][1] * Est_Acc_Body_Filtered[4] + CTMA_inv[0][2] * Est_Acc_Body_Filtered[5];
  Acc_0[2] = CTMA_inv[1][0] * Est_Acc_Body_Filtered[3] + CTMA_inv[1][1] * Est_Acc_Body_Filtered[4] + CTMA_inv[1][2] * Est_Acc_Body_Filtered[5];
  Acc_0[3] = CTMA_inv[2][0] * Est_Acc_Body_Filtered[3] + CTMA_inv[2][1] * Est_Acc_Body_Filtered[4] + CTMA_inv[2][2] * Est_Acc_Body_Filtered[5];

  // (1) Gain Calculation
  double q_gain[6], k1_gain[6], k2_gain[6];

  // ZZ (Height)
  q_gain[2] = 1.0 / (CLAW_P.BSC_Ome_ZZ * CLAW_P.BSC_Ome_ZZ * (1.0 - CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Zeta_ZZ));
  k1_gain[2] = CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Ome_ZZ / q_gain[2];
  k2_gain[2] = CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Ome_ZZ;

  // PH (Roll)
  q_gain[3] = 1.0 / (CLAW_P.BSC_Ome_PH * CLAW_P.BSC_Ome_PH * (1.0 - CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Zeta_PH));
  k1_gain[3] = CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Ome_PH / q_gain[3];
  k2_gain[3] = CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Ome_PH;

  // TH (Pitch)
  q_gain[4] = 1.0 / (CLAW_P.BSC_Ome_TH * CLAW_P.BSC_Ome_TH * (1.0 - CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Zeta_TH));
  k1_gain[4] = CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Ome_TH / q_gain[4];
  k2_gain[4] = CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Ome_TH;

  // PS (Yaw)
  q_gain[5] = 1.0 / (CLAW_P.BSC_Ome_PS * CLAW_P.BSC_Ome_PS * (1.0 - CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Zeta_PS));
  k1_gain[5] = CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Ome_PS / q_gain[5];
  k2_gain[5] = CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Ome_PS;

  // 3. 목표치 변환
  // [Sejong]
  // [버전 : NED] GCS가 home 기준 미터로 직접 주므로 NE는 변환 없이 사용 (현재 사용)
      
  if (home_init){
    Target_X = Dest_N;              // North 목표 거리(m)
    Target_Y = Dest_E;              // East  목표 거리(m)
    Target_Z = Dest_U - Home_Alt;   // Up 목표 고도(m, 진입 datum 정합)
  }
  else {
    Target_X = 0.0;
    Target_Y = 0.0;
    Target_Z = 0.0;
  }
  // [원본 : 위/경도] GPS -> NED Meter
  // if (home_init) {
  // Target_X = (Dest_Lat - Home_Lat) * Lat2m(Home_Lat);
  // Target_Y = (Dest_Lon - Home_Lon) * Lon2m(Home_Lon);
  // Target_Z = (Dest_Alt - Home_Alt);
  // }
  // else {
  // Target_X = 0.0; Target_Y = 0.0; Target_Z = 0.0;
  // }

  // 4. NED 오차 계산 (이미 미터 단위로 변환된 값끼리 계산됨)
  // [Sejong]
  Err_N = Target_X - STV[9];  // (목표N - 현재N) (m)
  Err_E = Target_Y - STV[10]; // (목표E - 현재E) (m)
  Err_D = Target_Z - STV[11]; // (목표D - 현재D) (m)

  // 제어기가 한 번에 인식하는 최대 오차를 15m로 제한 (고삐 역할)
  double leash_dist_1 = 15.0;
  double leash_dist_2 = 15.0;

  if (Err_N >  leash_dist_1) Err_N =  leash_dist_1;
  else if (Err_N < -leash_dist_1) Err_N = -leash_dist_1;

  if (Err_E >  leash_dist_2) Err_E =  leash_dist_2;
  else if (Err_E < -leash_dist_2) Err_E = -leash_dist_2;

  // 적분기 업데이트
  // TV_BSC[0] += Err_N * DT;
  // TV_BSC[1] += Err_E * DT;

  double pos_lim = CLAW_P.BSC_Int_Limit;
  TV_BSC[0] += Err_N * DT;
  if (TV_BSC[0] > pos_lim) TV_BSC[0] = pos_lim; 
  else if (TV_BSC[0] < -pos_lim) TV_BSC[0] = -pos_lim;

  TV_BSC[1] += Err_E * DT;
  if (TV_BSC[1] > pos_lim) TV_BSC[1] = pos_lim; 
  else if (TV_BSC[1] < -pos_lim) TV_BSC[1] = -pos_lim;

  pos_des_dot_trim[0] = CTML_inv[0][0] * STV_trim[0] + CTML_inv[0][1] * STV_trim[1] + CTML_inv[0][2] * STV_trim[2];
  pos_des_dot_trim[1] = CTML_inv[1][0] * STV_trim[0] + CTML_inv[1][1] * STV_trim[1] + CTML_inv[1][2] * STV_trim[2];
  pos_des_dot_trim[2] = CTML_inv[2][0] * STV_trim[0] + CTML_inv[2][1] * STV_trim[1] + CTML_inv[2][2] * STV_trim[2];

  pos_dot_des[0] = pos_des_dot_trim[0] + CLAW_P.BSC_K_POS_P * Err_N + CLAW_P.BSC_K_POS_I * TV_BSC[0];
  pos_dot_des[1] = pos_des_dot_trim[1] + CLAW_P.BSC_K_POS_P * Err_E + CLAW_P.BSC_K_POS_I * TV_BSC[1];
  pos_dot_des[2] = 0.0;

  uv_des[0] = CTML[0][0] * pos_dot_des[0] + CTML[0][1] * pos_dot_des[1] + CTML[0][2] * pos_dot_des[2];
  uv_des[1] = CTML[1][0] * pos_dot_des[0] + CTML[1][1] * pos_dot_des[1] + CTML[1][2] * pos_dot_des[2];
  uv_des[2] = CTML[2][0] * pos_dot_des[0] + CTML[2][1] * pos_dot_des[1] + CTML[2][2] * pos_dot_des[2];

  // [수정 제안] 목표 속도 제한 로직 (CLAW_step 내)
  double max_vel = 5.0; // 최고 속도 5m/s 제한

  // uv_des 성분별 제한
  if (uv_des[0] >  max_vel) uv_des[0] =  max_vel;
  if (uv_des[0] < -max_vel) uv_des[0] = -max_vel;
  if (uv_des[1] >  max_vel) uv_des[1] =  max_vel;
  if (uv_des[1] < -max_vel) uv_des[1] = -max_vel;

  TV_BSC[2] += (STV[1] - uv_des[1]) * DT;
  TV_BSC[3] += (STV[0] - uv_des[0]) * DT;

  double lim = CLAW_P.BSC_Int_Limit;
  if (TV_BSC[2] > lim) TV_BSC[2] = lim; else if (TV_BSC[2] < -lim) TV_BSC[2] = -lim;
  if (TV_BSC[3] > lim) TV_BSC[3] = lim; else if (TV_BSC[3] < -lim) TV_BSC[3] = -lim;

  // [Sejong]
  double ph_cmd = STV_trim[6] - CLAW_P.BSC_K_VEL_P * (STV[1] - uv_des[1]) - CLAW_P.BSC_K_VEL_I * TV_BSC[2];
  double th_cmd = STV_trim[7] + CLAW_P.BSC_K_VEL_P * (STV[0] - uv_des[0]) + CLAW_P.BSC_K_VEL_I * TV_BSC[3];
  double ps_cmd = (double)CLAW_U.Tar_heading;	// GCS 목표 heading (rad, NED)
  double zz_cmd = Target_Z;

	double psi_diff = ps_cmd - STV[8];
  while(psi_diff > M_PI) psi_diff -= 2.0 * M_PI;
  while(psi_diff < -M_PI) psi_diff += 2.0 * M_PI;

  ps_cmd = STV[8] + psi_diff;

  // [Sejong]
  double max_tilt = 25.0 * D2R;  // 25도 제한
  if (ph_cmd >  max_tilt) ph_cmd =  max_tilt;
  if (ph_cmd < -max_tilt) ph_cmd = -max_tilt;
  if (th_cmd >  max_tilt) th_cmd =  max_tilt;
  if (th_cmd < -max_tilt) th_cmd = -max_tilt;

  // [Sejong]
  // 커맨드 필터 및 Error Dynamics
  ddXtraj[0] = (zz_cmd - Xtraj[0]) * CLAW_P.BSC_Ome_ZZ * CLAW_P.BSC_Ome_ZZ - 2.0 * CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Ome_ZZ * dXtraj[0];
  if (ddXtraj[0] >  15.0) ddXtraj[0] =  15.0;
  if (ddXtraj[0] < -15.0) ddXtraj[0] = -15.0;
  ddXtraj[1] = (ph_cmd - Xtraj[1]) * CLAW_P.BSC_Ome_PH * CLAW_P.BSC_Ome_PH - 2.0 * CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Ome_PH * dXtraj[1];
  ddXtraj[2] = (th_cmd - Xtraj[2]) * CLAW_P.BSC_Ome_TH * CLAW_P.BSC_Ome_TH - 2.0 * CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Ome_TH * dXtraj[2];
  ddXtraj[3] = (ps_cmd - Xtraj[3]) * CLAW_P.BSC_Ome_PS * CLAW_P.BSC_Ome_PS - 2.0 * CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Ome_PS * dXtraj[3];

  dXtraj[0] += DT * ddXtraj[0]; if (dXtraj[0] < -10.0) dXtraj[0] = -10.0; Xtraj[0] += DT * dXtraj[0]; // 하강 속도 제한 10 m/s (throttle=0 방지) 
  dXtraj[1] += DT * ddXtraj[1]; Xtraj[1] += DT * dXtraj[1];
  dXtraj[2] += DT * ddXtraj[2]; Xtraj[2] += DT * dXtraj[2];
  dXtraj[3] += DT * ddXtraj[3]; Xtraj[3] += DT * dXtraj[3];

  int i, j;
  for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) G_mat[i][j] = 0.0;

  for (j = 0; j < 4; j++) {
    G_mat[0][j] = CTML_inv[2][0] * CLAW_P.BSC_B_mat[0 * 4 + j] + CTML_inv[2][1] * CLAW_P.BSC_B_mat[1 * 4 + j] + CTML_inv[2][2] * CLAW_P.BSC_B_mat[2 * 4 + j];
  }

  for (i = 1; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      G_mat[i][j] = CTMA_inv[i - 1][0] * CLAW_P.BSC_B_mat[3 * 4 + j] + CTMA_inv[i - 1][1] * CLAW_P.BSC_B_mat[4 * 4 + j] + CTMA_inv[i - 1][2] * CLAW_P.BSC_B_mat[5 * 4 + j];
    }
  }

  for (i = 0; i < 4; i++) {
    // 절대값이 1e-6보다 작을 때, 원래 부호를 유지하며 하한선 고정
    if (fabs(G_mat[i][i]) < 1e-6) {
      G_mat[i][i] = (G_mat[i][i] >= 0) ? 1e-6 : -1e-6;
    }
    G_mat_inv[i][i] = 1.0 / G_mat[i][i];
  }

  z1[0] = STV[11] - Xtraj[0]; z1[1] = STV[6] - Xtraj[1]; z1[2] = STV[7] - Xtraj[2]; z1[3] = STV[8] - Xtraj[3];
	while (z1[3] >  M_PI) z1[3] -= 2.0*M_PI;
  while (z1[3] < -M_PI) z1[3] += 2.0*M_PI;
  if (fabs(z1[3]) < 0.035) z1[3] = 0.0;    // 약 2도(0.035 rad) 이내의 오차는 무시하여 미세 진동이 증폭되는 것을 방지

  double z1_dot[4];
  z1_dot[0] = XTV[2] - dXtraj[0];
  z1_dot[1] = XTV[3] - dXtraj[1];
  z1_dot[2] = XTV[4] - dXtraj[2];
  z1_dot[3] = XTV[5] - dXtraj[3];

  alpha[0] = -1.0 * q_gain[2] * k1_gain[2] * z1[0] + dXtraj[0];
  alpha[1] = -1.0 * q_gain[3] * k1_gain[3] * z1[1] + dXtraj[1];
  alpha[2] = -1.0 * q_gain[4] * k1_gain[4] * z1[2] + dXtraj[2];
  alpha[3] = -1.0 * q_gain[5] * k1_gain[5] * z1[3] + dXtraj[3];

  z2[0] = XTV[2] - alpha[0]; z2[1] = XTV[3] - alpha[1]; z2[2] = XTV[4] - alpha[2]; z2[3] = XTV[5] - alpha[3];

  alpha_dot[0] = -1.0 * q_gain[2] * k1_gain[2] * z1_dot[0] + ddXtraj[0];
  alpha_dot[1] = -1.0 * q_gain[3] * k1_gain[3] * z1_dot[1] + ddXtraj[1];
  alpha_dot[2] = -1.0 * q_gain[4] * k1_gain[4] * z1_dot[2] + ddXtraj[2];
  alpha_dot[3] = -1.0 * q_gain[5] * k1_gain[5] * z1_dot[3] + ddXtraj[3];

  f[0] = 0.0; f[1] = 0.0; f[2] = 0.0; f[3] = 0.0;

  double u_temp[4];
  u_temp[0] = k2_gain[2] * z2[0] + z1[0] / q_gain[2] - alpha_dot[0] - Acc_0[0];
  u_temp[1] = k2_gain[3] * z2[1] + z1[1] / q_gain[3] - alpha_dot[1] - Acc_0[1];
  u_temp[2] = k2_gain[4] * z2[2] + z1[2] / q_gain[4] - alpha_dot[2] - Acc_0[2];
  u_temp[3] = k2_gain[5] * z2[3] + z1[3] / q_gain[5] - alpha_dot[3] - Acc_0[3];

  Del_Control[0] = -1.0 * (G_mat_inv[0][0] * u_temp[0] + G_mat_inv[0][1] * u_temp[1] + G_mat_inv[0][2] * u_temp[2] + G_mat_inv[0][3] * u_temp[3]);
  Del_Control[1] = -1.0 * (G_mat_inv[1][0] * u_temp[0] + G_mat_inv[1][1] * u_temp[1] + G_mat_inv[1][2] * u_temp[2] + G_mat_inv[1][3] * u_temp[3]);
  Del_Control[2] = -1.0 * (G_mat_inv[2][0] * u_temp[0] + G_mat_inv[2][1] * u_temp[1] + G_mat_inv[2][2] * u_temp[2] + G_mat_inv[2][3] * u_temp[3]);
  Del_Control[3] = -1.0 * (G_mat_inv[3][0] * u_temp[0] + G_mat_inv[3][1] * u_temp[1] + G_mat_inv[3][2] * u_temp[2] + G_mat_inv[3][3] * u_temp[3]);

  for (i = 0; i < 3; i++) STV_OLD[i] = STV[i];

  // 출력 스케일링
  // [Sejong]
	CLAW_Y.v_cmd.cmd_height = (real32_T)(Del_Control[0] * CLAW_P.BSC_Scale_Thrust * -1.0);      // Height 제어량 
  CLAW_Y.v_cmd.cmd_roll   = (real32_T)(Del_Control[1] * CLAW_P.BSC_Scale_Roll   * -1.0);      // Roll 제어량
  CLAW_Y.v_cmd.cmd_pitch  = (real32_T)(Del_Control[2] * CLAW_P.BSC_Scale_Pitch  * -1.0);      // Pitch 제어량
  CLAW_Y.v_cmd.cmd_yaw    = (real32_T)(Del_Control[3] * CLAW_P.BSC_Scale_Yaw);                // Yaw 제어량

  // 포화 (Saturation)
  if (CLAW_Y.v_cmd.cmd_roll > 1.0F) CLAW_Y.v_cmd.cmd_roll = 1.0F; 
  else if (CLAW_Y.v_cmd.cmd_roll < -1.0F) CLAW_Y.v_cmd.cmd_roll = -1.0F;

  if (CLAW_Y.v_cmd.cmd_pitch > 1.0F) CLAW_Y.v_cmd.cmd_pitch = 1.0F; 
  else if (CLAW_Y.v_cmd.cmd_pitch < -1.0F) CLAW_Y.v_cmd.cmd_pitch = -1.0F;

  if (CLAW_Y.v_cmd.cmd_yaw > 1.0F) CLAW_Y.v_cmd.cmd_yaw = 1.0F; 
  else if (CLAW_Y.v_cmd.cmd_yaw < -1.0F) CLAW_Y.v_cmd.cmd_yaw = -1.0F;

  if (CLAW_Y.v_cmd.cmd_height > 1.0F) CLAW_Y.v_cmd.cmd_height = 1.0F; 
  else if (CLAW_Y.v_cmd.cmd_height < -1.0F) CLAW_Y.v_cmd.cmd_height = -1.0F;

  Prev_CTRL[0] = (double)(CLAW_Y.v_cmd.cmd_height - 0.5f) / CLAW_P.BSC_Scale_Thrust;
  Prev_CTRL[1] = (double)(CLAW_Y.v_cmd.cmd_roll / (CLAW_P.BSC_Scale_Roll * -1.0));
  Prev_CTRL[2] = (double)(CLAW_Y.v_cmd.cmd_pitch / (CLAW_P.BSC_Scale_Pitch * -1.0));
  Prev_CTRL[3] = (double)(CLAW_Y.v_cmd.cmd_yaw / CLAW_P.BSC_Scale_Yaw);

  // 디버깅 출력
  CLAW_Y.cur_poti.x = (real32_T)STV[9];			// 화면상 Drone x 위치 (Home(0,0,0)기준, m)
  CLAW_Y.cur_poti.y = (real32_T)STV[10];		// 화면상 Drone y 위치 (Home(0,0,0)기준, m)
  CLAW_Y.cur_poti.z = -(real32_T)STV[11];		// 화면상 Drone h 위치 (Home(0,0,0)기준, m)
}