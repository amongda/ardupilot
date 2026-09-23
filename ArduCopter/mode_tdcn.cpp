// 26-08-18 비행 실험에 사용한 TDCN v2 firmware
//
// TDCN mode : CLAW 제어기 통합
//
// 작성자 : 세종대학교 정우주
//
// [Version 2] CLAW는 항상 돌린다 (모니터링 목적).
//             CLAW의 최종 출력(제어값)을 아두파일럿 제어기 대신 쓸지 말지는 state 6(추종비행)에서 결정하며, 이를 파라미터화 하여 외부에서 선택할 수 있게 한다.
//
// 관련 파일
//   mode.h                                 ModeTDCN 클래스 선언
//   GCS_Mavlink.cpp                        MAV_CMD_USER_1 수신 -> GCS_command()
//   mode_tdcn_CLAW_IBSC_ship_Fianl_NED.c   CLAW 제어기 본체
//

#include "Copter.h"

#if MODE_TDCN_ENABLED

// ---------------------------------------------------------------------------
// CLAW.c 코드를 연동하기 위해 extern "C" 문법 사용
// ---------------------------------------------------------------------------
extern "C" {
#include "mode_tdcn_CLAW.h"

// CLAW_step() 이 참조하는 arming 플래그를 전달해주기 위함
volatile uint8_t Arming = 0;

// TDCN 모드 재진입마다 CLAW의 Home을 초기화 하기 위함
extern bool home_init;

// XTV[0..2]에 EKF 속도를 넣기 위함
extern double XTV[6];

// CLAW 내부 상태를 로깅하기 위함
extern double TV_BSC[4];     
extern double pos_dot_des[3];
extern double uv_des[3];      
extern double Xtraj[4], dXtraj[4]; 
extern double alpha[4];    
extern double Del_Control[4];
extern double ps_cmd;       
extern double Cur_Lat, Cur_Lon, Cur_Alt;      
extern double Dest_Lat, Dest_Lon, Dest_Alt;
}

// ---------------------------------------------------------------------------
// 파라미터
//
//   state 5/6/7/8 의 수평 이동 속도   WPNAV_SPEED, WPNAV_ACCEL
//   state 5/6/7 의 수직 속도          WPNAV_SPEED_UP, WPNAV_SPEED_DN, WPNAV_ACCEL_Z
//   state 9 의 착륙 속도              LAND_SPEED, LAND_SPEED_HIGH, LAND_ALT_LOW
// ---------------------------------------------------------------------------
const AP_Param::GroupInfo ModeTDCN::var_info[] = {

    // @Param: TKO_ALT
    // @DisplayName: TDCN takeoff altitude
    // @Description: Target altitude for state 4 launch, above home
    // @Units: cm
    // @Range: 100 5000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("TKO_ALT", 1, ModeTDCN, _takeoff_alt, 1000),

    // @Param: TKO_SPD
    // @DisplayName: TDCN takeoff climb speed
    // @Description: Climb speed used during state 4 launch
    // @Units: cm/s
    // @Range: 20 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("TKO_SPD", 2, ModeTDCN, _takeoff_spd, 100),

    // @Param: LND_ALT
    // @DisplayName: TDCN landing sync altitude
    // @Description: Altitude held at the end of state 8 landing sync, above home. State 9 starts its descent from here.
    // @Units: cm
    // @Range: 100 5000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LND_ALT", 3, ModeTDCN, _land_alt, 1000),

    // @Param: LND_SPD
    // @DisplayName: TDCN landing sync descent speed
    // @Description: Descent speed used during state 8 landing sync. The final touchdown in state 9 uses LAND_SPEED instead.
    // @Units: cm/s
    // @Range: 20 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LND_SPD", 4, ModeTDCN, _land_spd, 100),

    // @Param: CLAW_ON_OFF
    // @DisplayName: TDCN use CLAW control output
    // @Description: 0 leaves ArduPilot flying the vehicle with CLAW running in parallel for monitoring only. 1 replaces the ArduPilot roll pitch yaw and throttle mixer inputs with the CLAW output during state 6 tracking. Only take off with 1 after the CLAW gains have been verified for this airframe.
    // @Values: 0:ArduPilot flies CLAW monitors,1:CLAW flies
    // @User: Advanced
    AP_GROUPINFO("CLAW_ON_OFF", 5, ModeTDCN, _claw_on_off, 0),

    AP_GROUPEND
};

ModeTDCN::ModeTDCN(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// ---------------------------------------------------------------------------
// CLAW 게인 파라미터
//
// mode_tdcn_CLAW_data_0729.c 의 CLAW_P 초기값을 그대로 기본값으로 옮김
//
// BSC_B_mat (제어효과 행렬, 48개) 은 제외함
// ---------------------------------------------------------------------------
const AP_Param::GroupInfo CLAW_Gains::var_info[] = {

    // @Param: SCALE_TH
    // @DisplayName: CLAW thrust output scale
    // @Description: Scales the backstepping thrust output into the normalised command range
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("SCALE_TH", 1, CLAW_Gains, _scale_th, 3.2),

    // @Param: SCALE_R
    // @DisplayName: CLAW roll output scale
    // @Description: Scales the backstepping roll output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("SCALE_R", 2, CLAW_Gains, _scale_r, 0.25),

    // @Param: SCALE_P
    // @DisplayName: CLAW pitch output scale
    // @Description: Scales the backstepping pitch output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("SCALE_P", 3, CLAW_Gains, _scale_p, 0.20),

    // @Param: SCALE_Y
    // @DisplayName: CLAW yaw output scale
    // @Description: Scales the backstepping yaw output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("SCALE_Y", 4, CLAW_Gains, _scale_y, 0.13),

    // @Param: K_POS_P
    // @DisplayName: CLAW outer loop position P
    // @Description: Position error to velocity command gain
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("K_POS_P", 5, CLAW_Gains, _k_pos_p, 0.3),

    // @Param: K_POS_I
    // @DisplayName: CLAW outer loop position I
    // @Description: Position error integral gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K_POS_I", 6, CLAW_Gains, _k_pos_i, 0.01),

    // @Param: K_VEL_P
    // @DisplayName: CLAW outer loop velocity P
    // @Description: Velocity error to attitude command gain
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("K_VEL_P", 7, CLAW_Gains, _k_vel_p, 0.4),

    // @Param: K_VEL_I
    // @DisplayName: CLAW outer loop velocity I
    // @Description: Velocity error integral gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K_VEL_I", 8, CLAW_Gains, _k_vel_i, 0.05),

    // @Param: AWU_LIMIT
    // @DisplayName: CLAW integrator limit
    // @Description: Anti windup clamp applied to all four CLAW integrators
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("AWU_LIMIT", 9, CLAW_Gains, _awu_limit, 1.0),

    // @Param: OMEGA_XX
    // @DisplayName: CLAW trajectory natural frequency North
    // @Description: Natural frequency of the North axis trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO("OMEGA_XX", 10, CLAW_Gains, _ome_xx, 0.3),

    // @Param: OMEGA_YY
    // @DisplayName: CLAW trajectory natural frequency East
    // @Description: Natural frequency of the East axis trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO("OMEGA_YY", 11, CLAW_Gains, _ome_yy, 0.3),

    // @Param: OMEGA_ZZ
    // @DisplayName: CLAW trajectory natural frequency height
    // @Description: Natural frequency of the height trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO("OMEGA_ZZ", 12, CLAW_Gains, _ome_zz, 2.0),

    // @Param: OMEGA_PH
    // @DisplayName: CLAW trajectory natural frequency roll
    // @Description: Natural frequency of the roll trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO("OMEGA_PH", 13, CLAW_Gains, _ome_ph, 9.3),

    // @Param: OMEGA_TH
    // @DisplayName: CLAW trajectory natural frequency pitch
    // @Description: Natural frequency of the pitch trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO("OMEGA_TH", 14, CLAW_Gains, _ome_th, 12.0),

    // @Param: OMEGA_PS
    // @DisplayName: CLAW trajectory natural frequency yaw
    // @Description: Natural frequency of the yaw trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO("OMEGA_PS", 15, CLAW_Gains, _ome_ps, 3.0),

    // @Param: ZETA_XX
    // @DisplayName: CLAW trajectory damping North
    // @Description: Damping ratio of the North axis trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_XX", 16, CLAW_Gains, _zeta_xx, 1.015),

    // @Param: ZETA_YY
    // @DisplayName: CLAW trajectory damping East
    // @Description: Damping ratio of the East axis trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_YY", 17, CLAW_Gains, _zeta_yy, 1.015),

    // @Param: ZETA_ZZ
    // @DisplayName: CLAW trajectory damping height
    // @Description: Damping ratio of the height trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_ZZ", 18, CLAW_Gains, _zeta_zz, 0.75),

    // @Param: ZETA_PH
    // @DisplayName: CLAW trajectory damping roll
    // @Description: Damping ratio of the roll trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_PH", 19, CLAW_Gains, _zeta_ph, 0.98),

    // @Param: ZETA_TH
    // @DisplayName: CLAW trajectory damping pitch
    // @Description: Damping ratio of the pitch trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_TH", 20, CLAW_Gains, _zeta_th, 0.98),

    // @Param: ZETA_PS
    // @DisplayName: CLAW trajectory damping yaw
    // @Description: Damping ratio of the yaw trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_PS", 21, CLAW_Gains, _zeta_ps, 0.9),

    // @Param: TAU_HDOT
    // @DisplayName: CLAW climb rate time constant
    // @Description: Time constant of the climb rate channel
    // @Units: s
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("TAU_HDOT", 22, CLAW_Gains, _tau_hdot, 0.164297),

    // @Param: TAU_R
    // @DisplayName: CLAW yaw rate time constant
    // @Description: Time constant of the yaw rate channel
    // @Units: s
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("TAU_R", 23, CLAW_Gains, _tau_r, 0.150985),

    AP_GROUPEND
};

CLAW_Gains::CLAW_Gains(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void CLAW_Gains::apply(void) const
{
    CLAW_P.BSC_Scale_Thrust = _scale_th;
    CLAW_P.BSC_Scale_Roll   = _scale_r;
    CLAW_P.BSC_Scale_Pitch  = _scale_p;
    CLAW_P.BSC_Scale_Yaw    = _scale_y;

    CLAW_P.BSC_K_POS_P = _k_pos_p;
    CLAW_P.BSC_K_POS_I = _k_pos_i;
    CLAW_P.BSC_K_VEL_P = _k_vel_p;
    CLAW_P.BSC_K_VEL_I = _k_vel_i;
    CLAW_P.BSC_Int_Limit = _awu_limit;

    CLAW_P.BSC_Ome_XX = _ome_xx;
    CLAW_P.BSC_Ome_YY = _ome_yy;
    CLAW_P.BSC_Ome_ZZ = _ome_zz;
    CLAW_P.BSC_Ome_PH = _ome_ph;
    CLAW_P.BSC_Ome_TH = _ome_th;
    CLAW_P.BSC_Ome_PS = _ome_ps;

    CLAW_P.BSC_Zeta_XX = _zeta_xx;
    CLAW_P.BSC_Zeta_YY = _zeta_yy;
    CLAW_P.BSC_Zeta_ZZ = _zeta_zz;
    CLAW_P.BSC_Zeta_PH = _zeta_ph;
    CLAW_P.BSC_Zeta_TH = _zeta_th;
    CLAW_P.BSC_Zeta_PS = _zeta_ps;

    CLAW_P.BSC_Tau_hdot = _tau_hdot;
    CLAW_P.BSC_Tau_r    = _tau_r;

    // BSC_B_mat 은 파라미터화 하지 않음
}

// ---------------------------------------------------------------------------
// 모드 진입
// ---------------------------------------------------------------------------
bool ModeTDCN::init(bool ignore_checks)
{
    // GCS 명령을 받기 전까지는 아무 state 도 아님
    _state = State::NONE;

    // 타겟 초기값 = 현재 위치 / 현재 헤딩
    _target_loc = copter.current_loc;                      // 위경도 + 고도(AltFrame)
    _target_heading_deg = degrees(ahrs.get_yaw());         // rad -> deg, 진북

    // 모드에 들어올 때마다 CLAW의 home 을 다시 잡음
    // 이로 인해 (TV_BSC, Xtraj, 가속도 추정기 버퍼, UD_DSTATE) 를 모두 리셋
    home_init = false;
    _log_counter = 0;

    // state 2 진입 시 무조건 한 번 보고하도록 초기값을 false 로 둔다
    _prearm_ready = false;

    // state 진입 훅
    _state_entered = false;
    _state_start_ms = AP_HAL::millis();

    // NONE 은 할 일이 없으므로 완료 상태로 둔다 (1번은 언제든 받는다)
    _state_done = true;
    _action_retry_ms = 0;
    _was_armed = motors->armed();
    _air_hold_valid = false;

    // 무장 시각
    _armed_ms = AP_HAL::millis();
    _armed_prev = motors->armed();
    _takeoff_started = false;

    _in_cur_lat = _in_cur_lng = _in_cur_alt = 0.0;
    _in_dst_lat = _in_dst_lng = _in_dst_alt = 0.0;
    _in_vel_n = _in_vel_e = _in_vel_d = 0.0;
    _in_p = _in_q = _in_r = 0.0f;
    _in_roll = _in_pitch = _in_yaw = 0.0f;
    _in_ship_hdg = 0.0f;

    // CLAW 상시 실행 - 모니터링을 위해 모드에 있는 동안은 계속 돌린다
    Arming = 1;

    return true;
}

// ---------------------------------------------------------------------------
// 모드 이탈
// ---------------------------------------------------------------------------
void ModeTDCN::exit()
{
    // CLAW 를 정지시킨다.  CLAW 는 Arming == 0 에서 출력을 0 으로 만들고
    // 가속도 추정기 버퍼를 리셋한다.
    Arming = 0;
}

// ---------------------------------------------------------------------------
// 메인 루프 (400Hz)
// ---------------------------------------------------------------------------
void ModeTDCN::run()
{
    // 무장 엣지 추적   
    {
        const bool armed_now = motors->armed();
        if (armed_now && !_armed_prev) {
            _armed_ms = AP_HAL::millis();
        }
        _armed_prev = armed_now;
    }

    // State 함수 처리 ?(main)
    switch (_state) {

    case State::NONE:           preflight_vehicle_handling(); break;  // 0  GCS 명령 대기.  기체는 지상이면 안전 처리, 공중이면 제자리 유지.

    case State::HANGAR_OPEN:    state_hangar_open();    break;  // 1  격납함 열기

    case State::TAKEOFF_WAIT:   state_takeoff_wait();   break;  // 2  이륙 대기

    case State::ARMED:          state_armed();          break;  // 3  ARMED

    case State::LAUNCH:         state_launch();         break;  // 4  이륙 사출

    case State::FLIGHT_WAIT:    state_flight_wait();    break;  // 5  비행 대기

    case State::TRACKING:       state_tracking();       break;  // 6  추종 비행

    case State::LANDING_WAIT:   state_landing_wait();   break;  // 7  착륙 대기

    case State::LANDING_SYNC:   state_landing_sync();   break;  // 8  착륙 동기

    case State::LANDING_STOW:   state_landing_stow();   break;  // 9  착륙 수납

    case State::DISARMED:       state_disarmed();       break;  // 10 DISARMED

    case State::HANGAR_CLOSE:   state_hangar_close();   break;  // 11 격납함 닫기
     
    }

    // 진입 훅 소비
    _state_entered = false;
}

// ---------------------------------------------------------------------------
// MAV_CMD_USER_1 (31010) 수신, 파싱, 순서 가드
// ---------------------------------------------------------------------------
MAV_RESULT ModeTDCN::GCS_command(const mavlink_command_int_t &packet)
{
    if (!isfinite(packet.param1)) {
        return MAV_RESULT_DENIED;
    }
    const int32_t state_num = (int32_t)roundf(packet.param1);
    if (state_num < (int32_t)State::HANGAR_OPEN ||
        state_num > (int32_t)State::HANGAR_CLOSE) {
        return MAV_RESULT_DENIED;
    }

    const State state = (State)state_num;

    // --- state 순서 가드 ---
    //
    // GCS 는 1 -> 11 을 차례로 보내는 것을 가정하며 강제함
    //
    // 반환값을 둘 로나눠 GCS 가 대응을 구분할 수 있게 함:
    //   DENIED               순서가 틀렸다
    //   TEMPORARILY_REJECTED 순서는 맞지만 아직 완료 전
    if (!state_order_ok(_state, state)) {
        return MAV_RESULT_DENIED;
    }
    if (state != _state && !_state_done) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // 목표값 파싱 (state 6 에서만 유효)
    Location loc;
    if (state == State::TRACKING) {
        if (!isfinite(packet.z) || !isfinite(packet.param2)) {
            return MAV_RESULT_DENIED;
        }

        switch (packet.frame) {

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            // 목표값을 위경도로 받는 경우 (최종 결과물 루트)
            loc.lat = packet.x;
            loc.lng = packet.y;
            break;

        case MAV_FRAME_LOCAL_NED:
            // 목표값을 NEU로 받는 경우 (실험용)
            // CLAW 입력을 맞추기 위해 NE를 위경도로 변환하여 CLAW에 전달
            if (!ahrs.home_is_set()) {
                return MAV_RESULT_DENIED;       // 기준점이 없으면 변환 불가
            }
            loc = ahrs.get_home();
            loc.offset(packet.x * 0.01,         // North (cm -> m)
                       packet.y * 0.01);        // East  (cm -> m)
            break;

        default:
            return MAV_RESULT_DENIED;
        }

        // 고도는 두 frame 모두 home 기준 up (m)
        loc.set_alt_cm((int32_t)(packet.z * 100.0f),
                       Location::AltFrame::ABOVE_HOME);
    }

    // --- 여기서부터 반영 ---
    const State prev_state = _state;
    _state = state;

    if (state == State::TRACKING) {
        _target_loc = loc;
        _target_heading_deg = packet.param2;    // 진북 기준 (deg)
    }

    // 상태 전이 - state 진입 훅
    if (_state != prev_state) {
        _state_entered = true;
        _state_start_ms = AP_HAL::millis();

        // state_*() 가 자기 조건을 보고 다시 true 로 올릴 때까지 다음 번호로 넘어갈 수 없음
        _state_done = false;
    }

    return MAV_RESULT_ACCEPTED;
}

// ---------------------------------------------------------------------------
// state 순서 검사
// state 유동성은 협의가 필요한 부분이며 현재는 1->11 순차적으로 입력되게 강제함
// ---------------------------------------------------------------------------

bool ModeTDCN::state_order_ok(State from, State to)
{
    if (to == from) {
        return true;                                    // 같은 번호 재전송
    }
    return (uint8_t)to == (uint8_t)from + 1;            // 바로 다음 번호만
}

// ---------------------------------------------------------------------------
// 이륙/착륙 상태
// ---------------------------------------------------------------------------
bool ModeTDCN::is_taking_off() const
{
    return (_state == State::LAUNCH) && !auto_takeoff.complete;
}

bool ModeTDCN::is_landing() const
{
    return _state == State::LANDING_STOW;
}

const char *ModeTDCN::state_name(State state)
{
    switch (state) {
    case State::NONE:           return "NONE";
    case State::HANGAR_OPEN:    return "HANGAR_OPEN";
    case State::TAKEOFF_WAIT:   return "TAKEOFF_WAIT";
    case State::ARMED:          return "ARMED";
    case State::LAUNCH:         return "LAUNCH";
    case State::FLIGHT_WAIT:    return "FLIGHT_WAIT";
    case State::TRACKING:       return "TRACKING";
    case State::LANDING_WAIT:   return "LANDING_WAIT";
    case State::LANDING_SYNC:   return "LANDING_SYNC";
    case State::LANDING_STOW:   return "LANDING_STOW";
    case State::DISARMED:       return "DISARMED";
    case State::HANGAR_CLOSE:   return "HANGAR_CLOSE";
    }
    return "?";
}

// [중요] CLAW의 입력을 맞춰주는 부분
void ModeTDCN::Update_Info_for_CLAW()
{
    // IMU - Gyro => CLAW: STV[3..5]
    const Vector3f &gyro = ahrs.get_gyro();
    CLAW_U.p = gyro.x;                           // roll  rate (rad/s, body F)
    CLAW_U.q = gyro.y;                           // pitch rate (rad/s, body R)
    CLAW_U.r = gyro.z;                           // yaw   rate (rad/s, body D)

    // IMU - Euler angle => CLAW: STV[6..8]
    CLAW_U.Roll                    = (real32_T)ahrs.get_roll();     // rad
    CLAW_U.Pitch                   = (real32_T)ahrs.get_pitch();    // rad
    CLAW_U.DR_heading_f.DR_heading =           ahrs.get_yaw();      // rad

    // EKF - Velocity => CLAW: XTV[0..2]
    const Vector3f &vel_neu_cms = inertial_nav.get_velocity_neu_cms();
    XTV[0] =  (double)vel_neu_cms.x * 0.01;       // North (m/s, N)
    XTV[1] =  (double)vel_neu_cms.y * 0.01;       // East  (m/s, E)
    XTV[2] = -(double)vel_neu_cms.z * 0.01;       // Down  (m/s, D)

    // Current Position
    const Location &loc = copter.current_loc;
    CLAW_U.Cur_Pos.x = (double)loc.lat * 1.0e-7;    // 위도 (deg)
    CLAW_U.Cur_Pos.y = (double)loc.lng * 1.0e-7;    // 경도 (deg)
    CLAW_U.Cur_Pos.z = (double)loc.alt * 0.01;      // 고도 (m, up)

    // Target Position (MAV_CMD_USER_1)
    CLAW_U.Dest_poti_i.x = (double)_target_loc.lat * 1.0e-7;   // 위도 (deg)
    CLAW_U.Dest_poti_i.y = (double)_target_loc.lng * 1.0e-7;   // 경도 (deg)
    CLAW_U.Dest_poti_i.z = (double)_target_loc.alt * 0.01;     // 고도 (m, up)

    // Target Heading  (MAV_CMD_USER_1)
    CLAW_U.Ship_heading = radians(wrap_180(_target_heading_deg));   // (rad, 진북)
}

// ---------------------------------------------------------------------------
// TDCN <-> CLAW 인터페이스 검증 로그 (파라미터화를 통해 내부 변수들 로깅)
// ---------------------------------------------------------------------------
void ModeTDCN::Log_Write_TDCN()
{
#if HAL_LOGGING_ENABLED
    if (_log_counter++ % 8 != 0) {
        return;
    }

    const uint64_t now_us = AP_HAL::micros64();

// @LoggerMessage: TDCP
// @Description: TDCN current position handed to CLAW vs read inside CLAW
// @Field: TimeUS: Time since system startup
// @Field: St: TDCN scenario state, 1 to 11
// @Field: TLat: Current latitude handed to CLAW
// @Field: TLng: Current longitude handed to CLAW
// @Field: TDwn: Current altitude handed to CLAW, down positive
// @Field: CLat: Current latitude inside CLAW
// @Field: CLng: Current longitude inside CLAW
// @Field: CDwn: Current altitude inside CLAW, down positive
    AP::logger().WriteStreaming("TDCP",
                                "TimeUS,St,TLat,TLng,TDwn,CLat,CLng,CDwn",
                                "QBddfddf",
                                now_us,
                                (uint8_t)_state,
                                _in_cur_lat,
                                _in_cur_lng,
                                (double)(-_in_cur_alt),     // up -> down
                                Cur_Lat,
                                Cur_Lon,
                                (double)Cur_Alt);

// @LoggerMessage: TDCT
// @Description: TDCN target handed to CLAW vs read inside CLAW
// @Field: TimeUS: Time since system startup
// @Field: TLat: Target latitude handed to CLAW
// @Field: TLng: Target longitude handed to CLAW
// @Field: TDwn: Target altitude handed to CLAW, down positive
// @Field: THdg: Target heading handed to CLAW
// @Field: CLat: Target latitude inside CLAW
// @Field: CLng: Target longitude inside CLAW
// @Field: CDwn: Target altitude inside CLAW, down positive
// @Field: CHdg: Target heading inside CLAW, after wrapToPi
    AP::logger().WriteStreaming("TDCT",
                                "TimeUS,TLat,TLng,TDwn,THdg,CLat,CLng,CDwn,CHdg",
                                "Qddffddff",
                                now_us,
                                _in_dst_lat,
                                _in_dst_lng,
                                (double)(-_in_dst_alt),     // up -> down
                                (double)degrees(_in_ship_hdg),
                                Dest_Lat,
                                Dest_Lon,
                                (double)Dest_Alt,
                                (double)degrees(ps_cmd));

// @LoggerMessage: TDCI
// @Description: TDCN IMU values handed to CLAW vs read inside CLAW
// @Field: TimeUS: Time since system startup
// @Field: Tp: Roll rate handed to CLAW
// @Field: Tq: Pitch rate handed to CLAW
// @Field: Tr: Yaw rate handed to CLAW
// @Field: TRol: Roll angle handed to CLAW
// @Field: TPit: Pitch angle handed to CLAW
// @Field: TYaw: Yaw angle handed to CLAW
// @Field: Cp: Roll rate inside CLAW, STV3
// @Field: Cq: Pitch rate inside CLAW, STV4
// @Field: Cr: Yaw rate inside CLAW, STV5
// @Field: CRol: Roll angle inside CLAW, STV6
// @Field: CPit: Pitch angle inside CLAW, STV7
// @Field: CYaw: Yaw angle inside CLAW, STV8 after wrapToPi
    AP::logger().WriteStreaming("TDCI",
                                "TimeUS,Tp,Tq,Tr,TRol,TPit,TYaw,Cp,Cq,Cr,CRol,CPit,CYaw",
                                "Qffffffffffff",
                                now_us,
                                (double)_in_p,
                                (double)_in_q,
                                (double)_in_r,
                                (double)_in_roll,
                                (double)_in_pitch,
                                (double)_in_yaw,
                                (double)STV[3],
                                (double)STV[4],
                                (double)STV[5],
                                (double)STV[6],
                                (double)STV[7],
                                (double)STV[8]);

// @LoggerMessage: TDCV
// @Description: TDCN velocity handed to CLAW vs read inside CLAW, NED
// @Field: TimeUS: Time since system startup
// @Field: TVN: North velocity handed to CLAW
// @Field: TVE: East velocity handed to CLAW
// @Field: TVD: Down velocity handed to CLAW
// @Field: CVN: North velocity inside CLAW, XTV0
// @Field: CVE: East velocity inside CLAW, XTV1
// @Field: CVD: Down velocity inside CLAW, XTV2
    AP::logger().WriteStreaming("TDCV",
                                "TimeUS,TVN,TVE,TVD,CVN,CVE,CVD",
                                "Qffffff",
                                now_us,
                                (double)_in_vel_n,
                                (double)_in_vel_e,
                                (double)_in_vel_d,
                                (double)XTV[0],
                                (double)XTV[1],
                                (double)XTV[2]);

    // --- NED 위치 비교 (TDCL) ---
    //
    // 아두파일럿은 EKF 가 낸 NED 를 그대로 쓰고, CLAW 는 위경도를 받아 자기
    // 식(Lat2m/Lon2m)으로 NED 를 다시 만든다.  그 변환식 차이를 보는 것이 목적이다.
    //
    // [원점] 서로 다르다.  아두파일럿은 EKF origin, CLAW 는 state 6 진입 위치다.
    // 제자리 이륙 후 넘어오므로 수평은 거의 같고 고도만 진입 고도만큼 차이난다.
    // 원점을 억지로 맞추지 않고 각자 값을 그대로 남긴다 - 맞추려면 환산식이
    // 하나 더 끼어들어 정작 보려는 변환식 차이가 가려진다.
    //
    // [부호] 양쪽 다 down 양수다.  CLAW 의 STV[11] 은 Cur_Alt - Home_Alt 로
    // 이미 down 양수이고, 아두파일럿은 NEU 의 z 를 뒤집어 맞춘다.
    const Vector3f &pos_neu_cm = inertial_nav.get_position_neu_cm();

// @LoggerMessage: TDCL
// @Description: TDCN NED position, ArduPilot EKF vs CLAW lat lon conversion
// @Field: TimeUS: Time since system startup
// @Field: AN: ArduPilot North, from EKF origin
// @Field: AE: ArduPilot East, from EKF origin
// @Field: AD: ArduPilot Down, from EKF origin
// @Field: CN: CLAW North, STV9, from CLAW home
// @Field: CE: CLAW East, STV10, from CLAW home
// @Field: CD: CLAW Down, STV11, from CLAW home
    AP::logger().WriteStreaming("TDCL",
                                "TimeUS,AN,AE,AD,CN,CE,CD",
                                "Qffffff",
                                now_us,
                                (double)(pos_neu_cm.x * 0.01f),
                                (double)(pos_neu_cm.y * 0.01f),
                                (double)(-pos_neu_cm.z * 0.01f),   // up -> down
                                (double)STV[9],
                                (double)STV[10],
                                (double)STV[11]);

// @LoggerMessage: TDCA
// @Description: TDCN attitude target, CLAW vs ArduPilot
// @Field: TimeUS: Time since system startup
// @Field: XR: CLAW trajectory generator roll target
// @Field: XP: CLAW trajectory generator pitch target
// @Field: XY: CLAW trajectory generator yaw target
// @Field: DR: ArduPilot attitude controller roll target
// @Field: DP: ArduPilot attitude controller pitch target
// @Field: DY: ArduPilot attitude controller yaw target
    AP::logger().WriteStreaming("TDCA",
                                "TimeUS,XR,XP,XY,DR,DP,DY",
                                "Qffffff",
                                now_us,
                                (double)Xtraj[1],
                                (double)Xtraj[2],
                                (double)Xtraj[3],
                                (double)attitude_control->get_att_target_euler_rad().x,
                                (double)attitude_control->get_att_target_euler_rad().y,
                                (double)attitude_control->get_att_target_euler_rad().z);

// @LoggerMessage: TDCR
// @Description: TDCN angular rate command, CLAW vs ArduPilot
// @Field: TimeUS: Time since system startup
// @Field: CRR: CLAW roll rate command
// @Field: CRP: CLAW pitch rate command
// @Field: CRY: CLAW yaw rate command
// @Field: ARR: ArduPilot roll rate target
// @Field: ARP: ArduPilot pitch rate target
// @Field: ARY: ArduPilot yaw rate target
    AP::logger().WriteStreaming("TDCR",
                                "TimeUS,CRR,CRP,CRY,ARR,ARP,ARY",
                                "Qffffff",
                                now_us,
                                (double)alpha[1],
                                (double)alpha[2],
                                (double)alpha[3],
                                (double)attitude_control->get_rate_ef_targets().x,
                                (double)attitude_control->get_rate_ef_targets().y,
                                (double)attitude_control->get_rate_ef_targets().z);

// @LoggerMessage: TDCC
// @Description: TDCN control output, CLAW vs ArduPilot, normalised
// @Field: TimeUS: Time since system startup
// @Field: CR: CLAW roll command
// @Field: CP: CLAW pitch command
// @Field: CY: CLAW yaw command
// @Field: CH: CLAW height command
// @Field: MR: ArduPilot roll input to mixer, rate PID plus feedforward
// @Field: MP: ArduPilot pitch input to mixer, rate PID plus feedforward
// @Field: MY: ArduPilot yaw input to mixer, rate PID plus feedforward
// @Field: MT: ArduPilot throttle input to mixer, 0 to 1
// @Field: UR: CLAW roll command before clamp
// @Field: UP: CLAW pitch command before clamp
// @Field: UY: CLAW yaw command before clamp
// @Field: UH: CLAW height command before clamp
// @Field: ACT: 1 when the CLAW output is actually driving the mixer
    AP::logger().WriteStreaming("TDCC",
                                "TimeUS,CR,CP,CY,CH,MR,MP,MY,MT,UR,UP,UY,UH,ACT",
                                "QffffffffffffB",
                                now_us,
                                (double)CLAW_Y.v_cmd.cmd_roll,
                                (double)CLAW_Y.v_cmd.cmd_pitch,
                                (double)CLAW_Y.v_cmd.cmd_yaw,
                                (double)CLAW_Y.v_cmd.cmd_height,
                                // 믹서가 실제로 소비하는 값과 같게 맞춘다.
                                // AP_MotorsMatrix::output_armed_stabilizing() 은
                                //   roll_thrust = (_roll_in + _roll_in_ff) * gain
                                // 처럼 rate PID 출력에 피드포워드를 더해 쓴다.
                                // get_roll() 은 _roll_in 만이라 그것만 비교하면
                                // 아두파일럿 몫을 과소평가한다.  스로틀은
                                // 피드포워드가 없어 get_throttle() 그대로다.
                                (double)(motors->get_roll()  + motors->get_roll_ff()),
                                (double)(motors->get_pitch() + motors->get_pitch_ff()),
                                (double)(motors->get_yaw()   + motors->get_yaw_ff()),
                                (double)motors->get_throttle(),
                                // CLAW.c 의 스케일 순서를 그대로 재현한다.
                                // clamp 만 빼면 CR..CH 와 같아야 하므로,
                                // 벌어지는 구간이 곧 포화 구간이다.
                                (double)(Del_Control[1] * CLAW_P.BSC_Scale_Roll),
                                (double)(Del_Control[2] * CLAW_P.BSC_Scale_Pitch * -1.0),
                                (double)(Del_Control[3] * CLAW_P.BSC_Scale_Yaw),
                                (double)(Del_Control[0] * CLAW_P.BSC_Scale_Thrust),
                                // CLAW 출력이 실제로 믹서를 몰고 있는가.
                                // TDCN_CLAW_ON_OFF 를 켰어도 state / home_init /
                                // 비행 여부 조건이 안 맞으면 0 이다.
                                (uint8_t)(claw_output_active() ? 1 : 0));

// @LoggerMessage: TDCE
// @Description: TDCN CLAW controller internal state
// @Field: TimeUS: Time since system startup
// @Field: EN: CLAW position error North, leash clamped
// @Field: EE: CLAW position error East, leash clamped
// @Field: ED: CLAW position error Down
// @Field: I0: CLAW position error integrator North
// @Field: I1: CLAW position error integrator East
// @Field: I2: CLAW velocity error integrator lateral
// @Field: I3: CLAW velocity error integrator forward
// @Field: DN: CLAW velocity command North, before body rotation
// @Field: DE: CLAW velocity command East, before body rotation
// @Field: UU: CLAW desired body forward velocity
// @Field: UV: CLAW desired body lateral velocity
// @Field: XZ: CLAW trajectory generator height target
    AP::logger().WriteStreaming("TDCE",
                                "TimeUS,EN,EE,ED,I0,I1,I2,I3,DN,DE,UU,UV,XZ",
                                "Qffffffffffff",
                                now_us,
                                (double)Err_N,
                                (double)Err_E,
                                (double)Err_D,
                                (double)TV_BSC[0],
                                (double)TV_BSC[1],
                                (double)TV_BSC[2],
                                (double)TV_BSC[3],
                                (double)pos_dot_des[0],
                                (double)pos_dot_des[1],
                                (double)uv_des[0],
                                (double)uv_des[1],
                                (double)Xtraj[0]);
#endif  // HAL_LOGGING_ENABLED
}

// ---------------------------------------------------------------------------
// CLAW 출력을 믹서에 넣을지 판단 (파라미터로 1을 설정하면 해당 함수 실행)
// ---------------------------------------------------------------------------
bool ModeTDCN::claw_output_active() const
{
    return _claw_on_off == 1
           && _state == State::TRACKING
           && home_init
           && motors->armed()
           && !is_disarmed_or_landed();
}

// ---------------------------------------------------------------------------
// 믹서 직전에 CLAW 출력을 대입
// PID는 계속 돌지만 이 함수를 통해 해당 변수의 값이 CLAW 값으로 변경됨
// ---------------------------------------------------------------------------
void ModeTDCN::output_to_motors()
{
    if (claw_output_active()) {
        motors->set_roll(constrain_float(CLAW_Y.v_cmd.cmd_roll,  -1.0f, 1.0f));
        motors->set_pitch(constrain_float(CLAW_Y.v_cmd.cmd_pitch, -1.0f, 1.0f));
        motors->set_yaw(constrain_float(CLAW_Y.v_cmd.cmd_yaw,   -1.0f, 1.0f));

        const float hover = motors->get_throttle_hover();
        const float ch    = constrain_float(CLAW_Y.v_cmd.cmd_height, -1.0f, 1.0f);
        const float thr   = is_negative(ch) ? hover * (1.0f + ch)
                                            : hover + ch * (1.0f - hover);
        motors->set_throttle(constrain_float(thr, 0.0f, 1.0f));

        // 아두파일럿 각속도 PID 의 피드포워드가 더해지지 않게 지움
        motors->set_roll_ff(0.0f);
        motors->set_pitch_ff(0.0f);
        motors->set_yaw_ff(0.0f);
    }

    Mode::output_to_motors();
}

// ---------------------------------------------------------------------------
// CLAW
// ---------------------------------------------------------------------------
void ModeTDCN::Run_CLAW()
{
    claw_gains.apply();         // 파라미터 -> CLAW_P (바꾸면 즉시 반영)
    Update_Info_for_CLAW();     // CLAW_U 에 기체 정보 + GCS 타겟 전달

    // --- 넘긴 값 스냅샷 ---
    _in_cur_lat   = CLAW_U.Cur_Pos.x;
    _in_cur_lng   = CLAW_U.Cur_Pos.y;
    _in_cur_alt   = CLAW_U.Cur_Pos.z;           // up 양수 (로그에서 뒤집음)
    _in_dst_lat   = CLAW_U.Dest_poti_i.x;
    _in_dst_lng   = CLAW_U.Dest_poti_i.y;
    _in_dst_alt   = CLAW_U.Dest_poti_i.z;       // up 양수
    _in_vel_n     = XTV[0];
    _in_vel_e     = XTV[1];
    _in_vel_d     = XTV[2];
    _in_p         = CLAW_U.p;
    _in_q         = CLAW_U.q;
    _in_r         = CLAW_U.r;
    _in_roll      = CLAW_U.Roll;
    _in_pitch     = CLAW_U.Pitch;
    _in_yaw       = CLAW_U.DR_heading_f.DR_heading;
    _in_ship_hdg  = CLAW_U.Ship_heading;

    const bool home_was_init = home_init;

    CLAW_step();                // CLAW 실행 (CLAW.c 의 함수 직접 호출)

    // Home을 잡을 때 목표 요를 0(진북)으로 잡기 때문에 어떤 상황에서도 확 도는 것을 막기 위함
    if (!home_was_init && home_init) {
        Xtraj[3]  = STV[8];     // 요 목표 = 현재 헤딩
        dXtraj[3] = 0.0;        // 각속도도 0 에서 출발
    }

    Log_Write_TDCN();           // 스냅샷 + CLAW 내부값을 짝지어 기록
}

// ---------------------------------------------------------------------------
// state 1~11 별 처리
// ---------------------------------------------------------------------------

// state 0 : 어떤 상태에서 TDCN 모드를 돌입할 때를 대비한 안전 장치
void ModeTDCN::preflight_vehicle_handling()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        _air_hold_valid = false;   
        return;
    }

    if (!_air_hold_valid) {
        _air_hold_valid = true;

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                            wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                    wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                            wp_nav->get_default_speed_up(),
                                            wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                    wp_nav->get_default_speed_up(),
                                                    wp_nav->get_accel_z());

        // 다른 모드에서 막 넘어왔을 수 있으므로 정지점 기준으로 초기화한다.
        pos_control->init_xy_controller_stopping_point();
        pos_control->init_z_controller_stopping_point();
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);
    pos_control->update_xy_controller();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

// state 1 : 격납함 열기
void ModeTDCN::state_hangar_open() 
{
    _state_done = true;

    preflight_vehicle_handling();
}

// state 2 : 이륙 대기 (pream check)
void ModeTDCN::state_takeoff_wait()
{
    const bool ready = copter.ap.pre_arm_check;

    _state_done = ready;

    if (_state_entered || ready != _prearm_ready) {
        _prearm_ready = ready;
        gcs().send_text(ready ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                        "%s: prearm %s", name(), ready ? "OK" : "FAIL");
    }

    preflight_vehicle_handling();
}

// state 3 : ARMED
void ModeTDCN::state_armed()         
{
    _state_done = motors->armed();

    // 무장 상태를 유지함
    if (!_state_done) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || _was_armed || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
    }
    _was_armed = motors->armed();

    preflight_vehicle_handling();
}


// 무장 후 이륙하기까지 대기 시간 (ms).
static const uint32_t _takeoff_settle_ms = 2000;    // 2 초

// state 4 : 이륙 사출 
void ModeTDCN::state_launch()        
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_state_entered) {
        _takeoff_started = false;
    }

    // 만약 해당 state에 들어왔는데도 불구하고 무장 해제 되어 있으면 무장 및 이륙 재시도
    if (!_state_done && !motors->armed()) {
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
        if (!motors->armed()) {
            make_safe_ground_handling();
            return;              
        }
        _takeoff_started = false;  
    }

    if (!_takeoff_started && is_disarmed_or_landed() &&
        (now_ms - _armed_ms) < _takeoff_settle_ms) {
        make_safe_ground_handling();
        return;
    }

    if (!_takeoff_started) {
        // 이륙 시작 (ModeGuided::do_user_takeoff_start() 와 같은 순서)

        // 이륙 중에는 heading 을 유지
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        pos_control->set_max_speed_accel_z(-_takeoff_spd, _takeoff_spd,
                                          g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-_takeoff_spd, _takeoff_spd,
                                                  g.pilot_accel_z);

        pos_control->init_z_controller();

        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm((int32_t)_takeoff_alt, Location::AltFrame::ABOVE_HOME);
        int32_t alt_above_origin_cm;
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,
                                   alt_above_origin_cm)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "%s: takeoff alt failed", name());
            return;
        }
        auto_takeoff.start((float)alt_above_origin_cm, false);

        copter.set_auto_armed(true);

        _takeoff_started = true;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: takeoff to %.1fm", name(),
                        (double)(_takeoff_alt * 0.01f));
    }

    // 이륙 후 제자리 호버링
    auto_takeoff.run();

    _state_done = auto_takeoff.complete;
}


// state 5 : 비행 대기 (호버링)
// [기반 모드] GUIDED (Position 서브모드)
//   초기화: ModeGuided::pva_control_start()   (mode_guided.cpp)
//   제어  : ModeGuided::pos_control_run()     (mode_guided.cpp)
void ModeTDCN::state_flight_wait()     
{
    _state_done = true;

    if (_state_entered) {
        if (!auto_takeoff.get_completion_pos(_hold_pos_neu_cm)) {
            _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
        }
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

// [중요] state 6 : 추종 비행 
void ModeTDCN::state_tracking()    
{
    _state_done = true;

    // Guided 모드 기반 추종 비행
    if (_state_entered) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }
        _track_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    // CLAW 기반 추종 비행
    Run_CLAW();

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector3f target_neu_cm;
    if (_target_loc.get_vector_from_origin_NEU(target_neu_cm)) {
        _track_pos_neu_cm = target_neu_cm.topostype();
    }

    pos_control->input_pos_xyz(_track_pos_neu_cm, 0.0f, 0.0f);
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    auto_yaw.set_yaw_angle_rate(_target_heading_deg, 0.0f);

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());

    if (claw_output_active()) {
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
    }
}

// state 7 : 착륙 대기 (호버링)
void ModeTDCN::state_landing_wait() 
{
    _state_done = true;

    if (_state_entered) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        // CLAW -> 아두파일럿 인수인계 ---
        pos_control->init_xy_controller_stopping_point();
        pos_control->init_z_controller_stopping_point();

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

// state 8 : 착륙 동기
void ModeTDCN::state_landing_sync()
{
    if (_state_entered) {
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(-_land_spd,
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(-_land_spd,
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    Location sync_loc = copter.current_loc;
    sync_loc.set_alt_cm((int32_t)_land_alt, Location::AltFrame::ABOVE_HOME);
    Vector3f sync_neu_cm;
    if (sync_loc.get_vector_from_origin_NEU(sync_neu_cm)) {
        _hold_pos_neu_cm.z = sync_neu_cm.z; 
    }

    _state_done = fabsf((float)copter.current_loc.alt - _land_alt) < 50.0f;

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

// state 9 : 착륙 수납
void ModeTDCN::state_landing_stow()
{
    _state_done = copter.ap.land_complete;

    if (_state_entered) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        copter.ap.land_repo_active = false;
        copter.ap.prec_land_active = false;

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    land_run_horiz_and_vert_control();
}

// state 10 : DISARMED
void ModeTDCN::state_disarmed()        
{
    _state_done = !motors->armed();

    if (!_state_done) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.disarm(AP_Arming::Method::MAVLINK);
        }
    }

    if (_state_entered) {
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

// state 11 : 격납함 닫기 
void ModeTDCN::state_hangar_close() 
{
    _state_done = true;
    make_safe_ground_handling();
}

#endif  // MODE_TDCN_ENABLED