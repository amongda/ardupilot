#include "Copter.h"

extern "C" {
#include "mode_tdcn_CLAW.h"
volatile uint8_t Arming = 0;
}

bool ModeTDCN::init(bool ignore_checks)
{
    _gcs_state       = 0;
    _current_state   = 0;
    _prearm_ok       = false;
    _takeoff_started = false;
    _was_armed       = copter.motors->armed();

    CLAW_reset();

    return true;
}

void ModeTDCN::run()
{
    update_command();   // 1. GCS로부터 받은 시나리오 번호 처리 → _current_state 갱신

    /* 시나리오별 매 루프 동작*/
    switch (_current_state) {
    
    // Not Auto
    case  1:                                break; // Open Case
    case  2:  run_prearm_check();           break; // Pre-ARMED Check
    case  3:  run_arm();                    break; // ARMED
    case  4:  run_takeoff();                break; // Takeoff
    case  5:  run_hover_for_IBSC();         break; // Hover for IBSC
    case  6:  run_follow_ibsc();            break; // IBSC
    case  7:  run_hover_for_landing();      break; // Hover for Landing
    case  8:  run_descend_for_landing();    break; // Descend for Landing (10m)
    case  9:  run_land();                   break; // Landing
    case 10:  run_disarm();                 break; // DISARMED
    case 11:                                break; // Close Case — 무동작(격납함은 외부 HW)

    // Auto — TODO: 12~16 보류 (현재 미구현, 임시 break)
    case 12:     // Auto Up
    case 13:     // Alt Up   (+10m)
    case 14:     // Alt Down (-10m)
    case 15:     // Alt Hold (100m)
    case 16:     // Auto Down
        break;
    }
}

// MAV_CMD_USER_1 수신
void ModeTDCN::handle_gcs_state(int8_t state)
{
    _gcs_state = state;
}

// MAV_CMD_USER_1 for Target Position 
void ModeTDCN::handle_gcs_follow(float north_m, float east_m, float yaw_deg, float alt_m)
{
    _tgt_north_m = north_m;
    _tgt_east_m  = east_m;
    // [2026-06-16] yaw_deg = pre-arm 시점 heading datum(_yaw_datum_rad) 기준 offset(deg) → 절대 목표 heading.
    //   0이면 datum 유지, 30이면 datum+30°. datum이 고정이라 여러 번 보내도 누적 안 됨(위치 home기준 offset과 동일 철학).
    // _tgt_yaw_rad = wrap_PI(radians(yaw_deg));                                              // 옛1: 절대 heading(0=북)
    // _tgt_yaw_rad = is_zero(yaw_deg) ? copter.ahrs.get_yaw() : wrap_PI(radians(yaw_deg));   // 옛2: 0이면 현재유지/아니면 절대
    // _tgt_yaw_rad = wrap_PI(copter.ahrs.get_yaw() + radians(yaw_deg));                      // 옛3: 메시지 수신 순간 현재 heading 기준(누적됨)
    _tgt_yaw_rad = wrap_PI(_yaw_datum_rad + radians(yaw_deg));
    _tgt_alt_m   = alt_m;
    _gcs_state   = 6;
}

// GCS Command
void ModeTDCN::update_command()
{
    const bool armed_now = copter.motors->armed();
    if (_was_armed && !armed_now) {
        CLAW_reset();
        _takeoff_started = false;
    }
    _was_armed = armed_now;

    if (_gcs_state == 0) { return; }

    // Condition of case 3
    if (_gcs_state == 3 && !AP_Notify::flags.pre_arm_check) {
        gcs().send_text(MAV_SEVERITY_WARNING, "TDCN: pre-arm not complete");
        _gcs_state = 0;
        return;
    }

    _current_state = _gcs_state;
    _gcs_state     = 0;
}

/* Not Auto */
// case  1 : Open Case

// case  2 : Pre-ARMED Check
void ModeTDCN::run_prearm_check()
{
    // [2026-06-16] pre-arm 단계의 heading을 yaw datum(=follow yaw offset의 0 기준)으로 캡처.
    //   pre-arm 중 매 루프 갱신되다 case 3(arm)으로 넘어가면 그 값으로 고정됨.
    _yaw_datum_rad = copter.ahrs.get_yaw();

    const bool ok = AP_Notify::flags.pre_arm_check;
    if (ok == _prearm_ok) { return; }

    gcs().send_text(ok ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                    ok ? "TDCN: pre-arm SUCCESS" : "TDCN: pre-arm FAIL");
    _prearm_ok = ok;
}

// case  3 : ARMED
void ModeTDCN::run_arm()
{
    if (copter.motors->armed()) { return; }

    if (!AP_Notify::flags.pre_arm_check) {
        _current_state = 2;
        gcs().send_text(MAV_SEVERITY_WARNING, "TDCN: pre-arm lost, back to PRE-ARM");
        return;
    }

    copter.arming.arm(AP_Arming::Method::MAVLINK);
}

// case  4 : Takeoff
// Target Altitude for Takeoff = 300 cm
// Up Accel                    =  100 cm/s²
// static constexpr float TAKEOFF_ALT_M = 5.0f;
static constexpr float TAKEOFF_ALT_M = 3.0f;   // [2026-06-16] 이륙 고도 3m
void ModeTDCN::run_takeoff()
{
    // 진입 첫 루프: 자동 이륙 1회 시작
    if (!_takeoff_started) {
        copter.set_auto_armed(true);

        // [2026-06-16 추가] 수평 위치 제어 초기화 — auto_takeoff.run()이 매 루프 update_xy_controller()로
        //   수평 station-keeping을 하는데, xy 컨트롤러 init·속도한계 설정이 없으면 수평/자세가 불안정해진다.
        //   (Guided는 모드 init→velaccel_control_start→pva_control_start에서 이미 init_xy_controller 수행)
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());  // 위치 오차 보정(_p_pos_xy) 한계 — 없으면 제자리 복귀 못하고 드리프트
        pos_control->init_xy_controller();

        pos_control->init_z_controller();

        // Up Velocity : set_max_speed_accel_z(하강, 상승, 가속도) [cm/s, cm/s²]
        // pos_control->set_max_speed_accel_z(-150.0f, 500.0f, 100.0f);  // 이전: 상승 최대 5m/s (가속도만 낮추면 5m 거리상 피크가 거리에 의존)
        pos_control->set_max_speed_accel_z(-150.0f, 100.0f, 100.0f);     // 상승 최대 1m/s (5m 이륙 → 약 5초), 가속 1m/s²로 부드럽게 도달

        Location target = copter.ahrs.get_home();
        // target.alt += (int32_t)(TAKEOFF_ALT_M * 50.0f);   // 이전: *50은 TAKEOFF_ALT_M=10에서 5m 만들던 우회 계수 → 상수 5와 곱하면 2.5m가 됨(버그)
        target.alt += (int32_t)(TAKEOFF_ALT_M * 100.0f);     // home + 목표고도(cm): 5m × 100 = 500cm (정상 m→cm 변환)
        int32_t alt_cm;
        if (!target.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_cm)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "TDCN: takeoff alt failed");
            return;
        }
        auto_takeoff.start((float)alt_cm, false);
        _takeoff_started = true;
        gcs().send_text(MAV_SEVERITY_INFO, "TDCN: takeoff to %.0fm", (double)TAKEOFF_ALT_M);
    }

    // 매 루프: 모터 스풀 최대 + 자동 이륙 실행
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    auto_takeoff.run();
}

// case  5 : Hover for IBSC
void ModeTDCN::run_hover_for_IBSC()
{
    if (!pos_control->is_active_xy()) {
        // [2026-06-16] 위치 보정 한계 설정 필수 — set_correction_speed_accel_xy 누락 시 _p_pos_xy 한계가 0이라 제자리 복귀 못하고 드리프트
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->init_xy_controller();
    }
    if (!pos_control->is_active_z())  { pos_control->init_z_controller();  }

    // 수평: 속도 0 명령 → 제자리 유지
    Vector2f vel, accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // 수직: 상승률 0 → 현재 고도 유지
    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    // 자세: 위치 컨트롤러 추력벡터 + 현재 heading 유지(yaw rate 0)
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);
}

// case  6 : IBSC
void ModeTDCN::run_follow_ibsc()
{
    /* Input Value - IMU, GPS, Target, EKF_Velocity */

    // Current - IMU [FRD, Euler Angle]
    CLAW_U.Roll                    = copter.ahrs.get_roll();       // rad, [-π, π]
    CLAW_U.Pitch                   = copter.ahrs.get_pitch();      // rad, [-π/2, π/2]
    CLAW_U.DR_heading_f.DR_heading = copter.ahrs.get_yaw();        // rad, [-π, π]
    const Vector3f gyro            = copter.ahrs.get_gyro();
    CLAW_U.p = gyro.x;                                             // rad/s
    CLAW_U.q = gyro.y;                                             // rad/s
    CLAW_U.r = gyro.z;                                             // rad/s

    // Current - GPS [Latitude(deg), Longitude(deg), Up(cm)]
    Location loc;
    copter.ahrs.get_location(loc);
    CLAW_U.Cur_Pos.x = (double)loc.lat * 1.0e-7;                // deg
    CLAW_U.Cur_Pos.y = (double)loc.lng * 1.0e-7;                // deg
    CLAW_U.Cur_Pos.z = -(double)copter.current_loc.alt * 0.01;  // Down (m)

    // Current - EKF Velocity [NED]
    Vector3f vel_ned; 
    if (copter.ahrs.get_velocity_NED(vel_ned)) {
        CLAW_U.Vel_N = vel_ned.x;   // m/s
        CLAW_U.Vel_E = vel_ned.y;   // m/s
        CLAW_U.Vel_D = vel_ned.z;   // m/s (Down)
    }

    // Target Position [Latitude(deg), Longitude(deg), Up (m)]   // But, now is NEU
    CLAW_U.Dest_poti_i.x = (double)_tgt_north_m;                 // Target North (m) or Latitude  (deg)
    CLAW_U.Dest_poti_i.y = (double)_tgt_east_m;                  // Target East  (m) or Longitude (deg)
    CLAW_U.Dest_poti_i.z = -(double)_tgt_alt_m;                  // Target Alt   (m) (Down)

    // Target Heading [rad]
    CLAW_U.Tar_heading   = _tgt_yaw_rad;                         // Target Yaw   (rad)


    // Arming State 
    Arming = copter.motors->armed() ? 1 : 0;

    // IBSC
    CLAW_step();

    /* Output Value - Control Value, Current Position */

    // Control Value
    _out_thr   = constrain_float(CLAW_Y.v_cmd.cmd_height + copter.motors->get_throttle_hover(), 0.0f, 1.0f);  // Thrust [-1,1] -> [0,1]
    _out_roll  = CLAW_Y.v_cmd.cmd_roll;                                                                       // Roll   [-1,1]
    _out_pitch = CLAW_Y.v_cmd.cmd_pitch;                                                                      // Pitch  [-1,1]
    _out_yaw   = CLAW_Y.v_cmd.cmd_yaw;                                                                        // Yaw    [-1,1]

    // Current - Position [NED]
    _pos_n_m = CLAW_Y.cur_poti.x;
    _pos_e_m = CLAW_Y.cur_poti.y;
    _pos_d_m = CLAW_Y.cur_poti.z;
}

// case  7 : Hover for Landing
void ModeTDCN::run_hover_for_landing()
{
    if (!pos_control->is_active_xy()) {
        // [2026-06-16] 위치 보정 한계 설정 필수 — set_correction_speed_accel_xy 누락 시 _p_pos_xy 한계가 0이라 제자리 복귀 못하고 드리프트
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->init_xy_controller();

        // [2026-06-16] case 6(IBSC)은 Attitude.cpp에서 rate controller를 우회 → 자세 PID I항이 동결됨.
        //   6→7 전환 시 동결된 I항이 그대로 적용돼 자세 킥이 발생하므로, 핸드오프 첫 루프에 부드럽게 리셋.
        attitude_control->reset_rate_controller_I_terms_smoothly();
    }
    if (!pos_control->is_active_z())  { pos_control->init_z_controller();  }

    // 수평: 속도 0 명령 → 제자리 유지
    Vector2f vel, accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // 수직: 상승률 0 → 현재 고도 유지
    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    // 자세: 위치 컨트롤러 추력벡터 + 현재 heading 유지(yaw rate 0)
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);
}

// case  8 : Descend for Landing
// Target Altitude for Landing = 1000 cm  
static constexpr float LANDING_ALT_M = 3.0f;
void ModeTDCN::run_descend_for_landing()
{
    if (!pos_control->is_active_xy()) {
        // [2026-06-16] 위치 보정 한계 설정 필수 — set_correction_speed_accel_xy 누락 시 _p_pos_xy 한계가 0이라 제자리 복귀 못하고 드리프트
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->init_xy_controller();
    }
    if (!pos_control->is_active_z())  { pos_control->init_z_controller();  }

    // 수평: 속도 0 명령 → 제자리 유지
    Vector2f vel, accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // 수직: home 기준 10m를 목표로 하강(도달 후 그 고도 유지). EKF origin 기준 cm로 변환해 입력.
    Location target = copter.ahrs.get_home();
    target.alt += (int32_t)(LANDING_ALT_M * 100.0f);
    int32_t alt_cm;
    if (target.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_cm)) {
        float pos_z = (float)alt_cm;
        float vel_z = 0.0f;
        pos_control->input_pos_vel_accel_z(pos_z, vel_z, 0.0f);
    }
    pos_control->update_z_controller();

    // 자세: 위치 컨트롤러 추력벡터 + 현재 heading 유지(yaw rate 0)
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);
}

// case  9 : Landing
// Down Speed = 50cm/s
// static constexpr float LAND_DESCENT_CMS = 50.0f;
// static constexpr float LAND_DESCENT_CMS = 10.0f;   // [2026-06-16] 착륙 하강 0.1m/s
static constexpr float LAND_DESCENT_CMS = 50.0f;   // [2026-06-22] 착륙 하강 0.5m/s
void ModeTDCN::run_land()
{
    // 지면 도달/착륙 완료 시: 모터 안전 처리 후 종료
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    if (!pos_control->is_active_xy()) {
        // [2026-06-16] 위치 보정 한계 설정 필수 — set_correction_speed_accel_xy 누락 시 _p_pos_xy 한계가 0이라 제자리 복귀 못하고 드리프트
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->init_xy_controller();
    }
    if (!pos_control->is_active_z())  { pos_control->init_z_controller();  }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // [하강 속도] set_max_speed_accel_z(하강, 상승, 가속도) — 하강률을 LAND_DESCENT_CMS로 명시(조절 가능)
    pos_control->set_max_speed_accel_z(-LAND_DESCENT_CMS, 250.0f, 250.0f);

    // 수평: 속도 0 명령 → 제자리 유지
    Vector2f vel, accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // 수직: 일정 하강률로 착륙 (지면 감지될 때까지)
    pos_control->set_pos_target_z_from_climb_rate_cm(-LAND_DESCENT_CMS);
    pos_control->update_z_controller();

    // 자세: 위치 컨트롤러 추력벡터 + 현재 heading 유지(yaw rate 0)
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);
}

// case 10 : DISARMED
void ModeTDCN::run_disarm()
{
    if (!copter.motors->armed()) { return; }
    copter.arming.disarm(AP_Arming::Method::MAVLINK);
}

// case 11 : Close Case


/* Auto */
// case 12 : Auto Up

// case 13 : Alt Up   (+10m)

// case 14 : Alt Down (-10m)

// case 15 : Alt Hold (100m)

// case 16 : Auto Down