#include "GCS_Mavlink.h"

#include "Plane.h"


void GCS_MAVLINK_Plane::setupuart(AP_HAL::UARTDriver *uart, const char *name)
{//bcode
    if (uart == NULL)
    {
        // that UART doesn't exist on this platform
        return;
    }
    ///begin(baudrate,Rx,Tx)
    uart->begin(57600, 256, 256);
}

MAV_TYPE GCS_MAVLINK_Plane::frame_type() const
{
    return plane.quadplane.get_mav_type();
}

void GCS_MAVLINK_Plane::send_albatros_1() 
{
    const AP_AHRS &ahrs = AP::ahrs();
    ahrs.get_position(global_position_current_loc); // return value ignored; we send stale data
    const AP_BattMonitor &battery = AP::battery();
    Vector3f vel;
    ahrs.get_velocity_NED(vel); 
    // catch the battery backend not supporting the required number of cells
    static_assert(sizeof(AP_BattMonitor::cells) >= (sizeof(uint16_t) * MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN),
                  "Not enough battery cells for the MAVLink message");
    Location &next_WP_loc = plane.next_WP_loc;

    Vector3f wind = ahrs.wind_estimate();


    mavlink_msg_albatros_1_msg_send(
        chan,
        ahrs.yaw,
        next_WP_loc.alt * 0.01f, // HUD\Target_alt (перенос значения alt из POSITION_TARGET_GLOBAL_INT) 1
        plane.mission.get_current_nav_index(),
        battery.capacity_remaining_pct(0),
        ahrs.yaw_sensor, // hdg
        AP::gps().status(0), // fix type
        AP::gps().num_sats(0), // sattelite visible
        degrees(atan2f(-wind.y, -wind.x)),
        wind.length()
    );



}

void GCS_MAVLINK_Plane::send_albatros_2_4()
{
    AP_AHRS &ahrs = AP::ahrs();

    // return values ignored; we send stale data
    ahrs.get_position(global_position_current_loc);
    ahrs.get_velocity_NED(vfr_hud_velned);


    //BATTERY_STATUS
    const AP_BattMonitor &battery = AP::battery();
    // catch the battery backend not supporting the required number of cells
    static_assert(sizeof(AP_BattMonitor::cells) >= (sizeof(uint16_t) * MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN),
                  "Not enough battery cells for the MAVLink message");

    //SYS_STATUS
    //gcs().send_text(MAV_SEVERITY_DEBUG, "Sen24");

    //Vibration
    const AP_InertialSensor &ins = AP::ins();
    Vector3f vibration = ins.get_vibration_levels();

    //GPS_RAW_INT
    const Location &loc = AP::gps().location(0);
    mavlink_msg_albatros_2_4_msg_send(
        chan,
        vfr_hud_airspeed(),
        ahrs.groundspeed(),
        vfr_hud_throttle(),
        global_position_current_loc.alt * 0.01f, // cm -> m
        battery.has_current(0) ? battery.current_amps(0) * 100 : -1, // current in centiampere
        battery.has_current(0) ? battery.consumed_mah(0) : -1,       // total consumed current in milliampere.hour
        battery.voltage() * 1000,// sysstatus voltage battary
        vibration.z,
        loc.lat,        // in 1E7 degrees
        loc.lng,        // in 1E7 degrees
        vfr_hud_climbrate(),
        AP_HAL::millis() //time_boot_ms
        );
}

void GCS_MAVLINK_Plane::send_albatros_4()
{
    //gcs().send_text(MAV_SEVERITY_DEBUG, "Sen4");
    mavlink_msg_albatros_4_msg_send(
        chan,
        global_position_int_alt(),       // millimeters above ground/sea level
        global_position_int_relative_alt() // millimeters above home
        );
}

void GCS_MAVLINK_Plane::send_albatros_10()
{
    const AP_AHRS &ahrs = AP::ahrs();
    //gcs().send_text(MAV_SEVERITY_DEBUG, "Sen10");
    float r = ahrs.roll;
    float p = ahrs.pitch - radians(plane.g.pitch_trim_cd*0.01f);
    mavlink_msg_albatros_10_msg_send(
        chan,
        r,
        p);
}

MAV_MODE GCS_MAVLINK_Plane::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (plane.control_mode) {
    case MANUAL:
    case TRAINING:
    case ACRO:
        _base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case CRUISE:
        _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
    case CIRCLE:
    case QRTL:
        _base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        break;
    }

    if (!plane.training_manual_pitch || !plane.training_manual_roll) {
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (plane.control_mode != MANUAL && plane.control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (plane.g.stick_mixing != STICK_MIXING_DISABLED && plane.control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

#if HIL_SUPPORT
    if (plane.g.hil_mode == 1) {
        _base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
    }
#endif

    // we are armed if we are not initialising
    if (plane.control_mode != INITIALISING && plane.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_MAVLINK_Plane::custom_mode() const
{
    return plane.control_mode;
}

MAV_STATE GCS_MAVLINK_Plane::system_status() const
{
    if (plane.control_mode == INITIALISING) {
        return MAV_STATE_CALIBRATING;
    }
    if (plane.any_failsafe_triggered()) {
        return MAV_STATE_CRITICAL;
    }
    if (plane.crash_state.is_crashed) {
        return MAV_STATE_EMERGENCY;
    }
    if (plane.is_flying()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
}


void GCS_MAVLINK_Plane::send_attitude() const
{
    const AP_AHRS &ahrs = AP::ahrs();

    float r = ahrs.roll;
    float p = ahrs.pitch - radians(plane.g.pitch_trim_cd*0.01f);
    float y = ahrs.yaw;
    
    if (plane.quadplane.tailsitter_active()) {
        r = plane.quadplane.ahrs_view->roll;
        p = plane.quadplane.ahrs_view->pitch;
        y = plane.quadplane.ahrs_view->yaw;
    }
    
    const Vector3f &omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        r,
        p,
        y,
        omega.x,
        omega.y,
        omega.z);
}

void Plane::send_aoa_ssa(mavlink_channel_t chan)
{
    mavlink_msg_aoa_ssa_send(
        chan,
        micros(),
        ahrs.getAOA(),
        ahrs.getSSA());
}

#if GEOFENCE_ENABLED == ENABLED
void Plane::send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


void Plane::send_extended_status1(mavlink_channel_t chan)
{
    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    update_sensor_status_flags();
    
    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average() * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);
}

void Plane::send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll_cd * 0.01f,
        nav_pitch_cd * 0.01f,
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        MIN(auto_state.wp_distance, UINT16_MAX),
        altitude_error_cm * 0.01f,
        airspeed_error * 100,
        nav_controller->crosstrack_error());
}

void GCS_MAVLINK_Plane::send_position_target_global_int()
{
    if (plane.control_mode == MANUAL) {
        return;
    }
    Location &next_WP_loc = plane.next_WP_loc;
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL_INT, // targets are always global altitude
        0xFFF8, // ignore everything except the x/y/z components
        next_WP_loc.lat, // latitude as 1e7
        next_WP_loc.lng, // longitude as 1e7
        next_WP_loc.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}


void Plane::send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) / 4500.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / 4500.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / 4500.0f),
        0,
        0,
        0,
        0,
        rssi.read_receiver_rssi_uint8());
}

float GCS_MAVLINK_Plane::vfr_hud_airspeed() const
{
    // airspeed sensors are best.  While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed.  When reporting we should send the true airspeed
    // value if possible:
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }

    // airspeed estimates are OK:
    float aspeed;
    if (AP::ahrs().airspeed_estimate(&aspeed)) {
        return aspeed;
    }

    // lying is worst:
    return 0;
}

int16_t GCS_MAVLINK_Plane::vfr_hud_throttle() const
{
    return abs(plane.throttle_percentage());
}

float GCS_MAVLINK_Plane::vfr_hud_climbrate() const
{
    if (plane.g2.soaring_controller.is_active()) {
        return plane.g2.soaring_controller.get_vario_reading();
    }
    return AP::baro().get_climb_rate();
}

/*
  keep last HIL_STATE message to allow sending SIM_STATE
 */
#if HIL_SUPPORT
static mavlink_hil_state_t last_hil_state;
#endif

// report simulator state
void GCS_MAVLINK_Plane::send_simstate() const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    GCS_MAVLINK::send_simstate();
#elif HIL_SUPPORT
    if (plane.g.hil_mode == 1) {
        mavlink_msg_simstate_send(chan,
                                  last_hil_state.roll,
                                  last_hil_state.pitch,
                                  last_hil_state.yaw,
                                  last_hil_state.xacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.yacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.zacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.rollspeed,
                                  last_hil_state.pitchspeed,
                                  last_hil_state.yawspeed,
                                  last_hil_state.lat,
                                  last_hil_state.lon);
    }
#endif
}

void Plane::send_wind(mavlink_channel_t chan)
{
    Vector3f wind = ahrs.wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

/*
  send RPM packet
 */
void NOINLINE Plane::send_rpm(mavlink_channel_t chan)
{
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        mavlink_msg_rpm_send(
            chan,
            rpm_sensor.get_rpm(0),
            rpm_sensor.get_rpm(1));
    }
}

// sends a single pid info over the provided channel
void Plane::send_pid_info(const mavlink_channel_t chan, const DataFlash_Class::PID_Info *pid_info,
                          const uint8_t axis, const float achieved)
{
    if (pid_info == nullptr) {
        return;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
        return;
    }
     mavlink_msg_pid_tuning_send(chan, axis,
                                 pid_info->desired,
                                 achieved,
                                 pid_info->FF,
                                 pid_info->P,
                                 pid_info->I,
                                 pid_info->D);
}

/*
  send PID tuning message
 */
void Plane::send_pid_tuning(mavlink_channel_t chan)
{
    const Vector3f &gyro = ahrs.get_gyro();
    const DataFlash_Class::PID_Info *pid_info;
    if (g.gcs_pid_mask & TUNING_BITS_ROLL) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        } else {
            pid_info = &rollController.get_pid_info();
        }
        send_pid_info(chan, pid_info, PID_TUNING_ROLL, degrees(gyro.x));
    }
    if (g.gcs_pid_mask & TUNING_BITS_PITCH) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        } else {
            pid_info = &pitchController.get_pid_info();
        }
        send_pid_info(chan, pid_info, PID_TUNING_PITCH, degrees(gyro.y));
    }
    if (g.gcs_pid_mask & TUNING_BITS_YAW) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        } else {
            pid_info = &yawController.get_pid_info();
        }
        send_pid_info(chan, pid_info, PID_TUNING_YAW, degrees(gyro.z));
    }
    if (g.gcs_pid_mask & TUNING_BITS_STEER) {
        send_pid_info(chan, &steerController.get_pid_info(), PID_TUNING_STEER, degrees(gyro.z));
    }
    if ((g.gcs_pid_mask & TUNING_BITS_LAND) && (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND)) {
        send_pid_info(chan, landing.get_pid_info(), PID_TUNING_LANDING, degrees(gyro.z));
    }
}

uint8_t GCS_MAVLINK_Plane::sysid_my_gcs() const
{
    return plane.g.sysid_my_gcs;
}

uint32_t GCS_MAVLINK_Plane::telem_delay() const
{
    return (uint32_t)(plane.g.telem_delay);
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Plane::try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

    // if we don't have at least 0.2ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!hal.scheduler->in_delay_callback() &&
        plane.scheduler.time_available_usec() < 200) {
        gcs().set_out_of_time(true);
        return false;
    }

    switch (id) {

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        plane.send_extended_status1(chan);
        CHECK_PAYLOAD_SIZE2(POWER_STATUS);
        send_power_status();
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (plane.control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            plane.send_nav_controller_output(chan);
        }
        break;

    case MSG_SERVO_OUT:
#if HIL_SUPPORT
        if (plane.g.hil_mode == 1) {
            CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
            plane.send_servo_out(chan);
        }
#endif
        break;

    case MSG_FENCE_STATUS:
#if GEOFENCE_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        plane.send_fence_status(chan);
#endif
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
#endif
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        plane.send_wind(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        plane.camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_OPTICAL_FLOW:
#if OPTFLOW == ENABLED
        if (plane.optflow.enabled()) {        
            CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
            send_opticalflow(plane.optflow);
        }
#endif
        break;

    case MSG_GIMBAL_REPORT:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        plane.camera_mount.send_gimbal_report(chan);
#endif
        break;

    case MSG_PID_TUNING:
        if (plane.control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(PID_TUNING);
            plane.send_pid_tuning(chan);
        }
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        plane.send_rpm(chan);
        break;

    case MSG_ADSB_VEHICLE:
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        plane.adsb.send_adsb_vehicle(chan);
        break;

    case MSG_AOA_SSA:
        CHECK_PAYLOAD_SIZE(AOA_SSA);
        plane.send_aoa_ssa(chan);
        break;
    case MSG_LANDING:
        plane.landing.send_landing_message(chan);
        break;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  10),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK, streamRates[9],  5),
    // @Param: ALBATROS1
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ALBATROS_1",    10, GCS_MAVLINK, streamRates[10],  1),
    AP_GROUPINFO("ALBATROS_2_4",  11, GCS_MAVLINK, streamRates[11],  3),
    AP_GROUPINFO("ALBATROS_4",    12, GCS_MAVLINK, streamRates[12],  4),
    AP_GROUPINFO("ALBATROS_10",   13, GCS_MAVLINK, streamRates[13],  10),
    AP_GROUPEND
};

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU1,  // RAW_IMU, SCALED_IMU2, SCALED_IMU3
    MSG_RAW_IMU2,  // SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3
    MSG_RAW_IMU3  // SENSOR_OFFSETS
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_EXTENDED_STATUS1, // SYS_STATUS, POWER_STATUS
    MSG_EXTENDED_STATUS2, // MEMINFO
    MSG_CURRENT_WAYPOINT,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_FENCE_STATUS,
    MSG_POSITION_TARGET_GLOBAL_INT,
};
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};
static const ap_message STREAM_RAW_CONTROLLER_msgs[] = {
    MSG_SERVO_OUT,
};
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RADIO_IN
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
    MSG_SIMSTATE, // SIMSTATE, AHRS2
    MSG_RPM,
    MSG_AOA_SSA,
    MSG_PID_TUNING,
    MSG_LANDING,
    MSG_ESC_TELEMETRY,
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD,
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_SYSTEM_TIME,
#if AP_TERRAIN_AVAILABLE
    MSG_TERRAIN,
#endif
    MSG_BATTERY2,
    MSG_BATTERY_STATUS,
    MSG_MOUNT_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_GIMBAL_REPORT,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
};
static const ap_message STREAM_ALBATROS_1_msgs[] = {
    MSG_ALBATROS_1,
};
static const ap_message STREAM_ALBATROS_2_4_msgs[] = {
    MSG_ALBATROS_2_4,
};
static const ap_message STREAM_ALBATROS_4_msgs[] = {
    MSG_ALBATROS_4,
};
static const ap_message STREAM_ALBATROS_10_msgs[] = {
    MSG_ALBATROS_10,
};
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_GPS_RTK,
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RAW_CONTROLLER),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),    
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_ENTRY(STREAM_ALBATROS_1),
    MAV_STREAM_ENTRY(STREAM_ALBATROS_2_4),
    MAV_STREAM_ENTRY(STREAM_ALBATROS_4),
    MAV_STREAM_ENTRY(STREAM_ALBATROS_10),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

bool GCS_MAVLINK_Plane::in_hil_mode() const
{
#if HIL_SUPPORT
    return plane.g.hil_mode == 1;
#endif
    return false;
}

/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
bool GCS_MAVLINK_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (plane.control_mode != GUIDED) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    plane.guided_WP_loc = cmd.content.location;
    
    // add home alt if needed
    if (plane.guided_WP_loc.flags.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.flags.relative_alt = 0;
    }

    plane.set_guided_WP();
    return true;
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK_Plane::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    plane.next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.flags.relative_alt) {
        plane.next_WP_loc.alt += plane.home.alt;
    }
    plane.next_WP_loc.flags.relative_alt = false;
    plane.next_WP_loc.flags.terrain_alt = cmd.content.location.flags.terrain_alt;
    plane.reset_offset_altitude();
}


MAV_RESULT GCS_MAVLINK_Plane::handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    plane.in_calibration = true;
    MAV_RESULT ret = GCS_MAVLINK::handle_command_preflight_calibration(packet);
    plane.in_calibration = false;

    return ret;
}

MAV_RESULT GCS_MAVLINK_Plane::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param4,1.0f)) {
        if (plane.trim_radio()) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}

void GCS_MAVLINK_Plane::packetReceived(const mavlink_status_t &status,
                                        mavlink_message_t &msg)
{
    plane.avoidance_adsb.handle_msg(msg);
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Plane::should_disable_overrides_on_reboot() const
{
    return (plane.quadplane.enable != 0);
}


MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch(packet.command) {

    case MAV_CMD_DO_SET_HOME:
        if (is_equal(packet.param1, 1.0f)) {
            plane.set_home_persistently(AP::gps().location());
            AP::ahrs().lock_home();
            return MAV_RESULT_ACCEPTED;
        } else {
            // ensure param1 is zero
            if (!is_zero(packet.param1)) {
                return MAV_RESULT_FAILED;
            }
            if ((packet.x == 0) && (packet.y == 0) && is_zero(packet.z)) {
                // don't allow the 0,0 position
                return MAV_RESULT_FAILED;
            }
            // check frame type is supported
            if (packet.frame != MAV_FRAME_GLOBAL &&
                packet.frame != MAV_FRAME_GLOBAL_INT &&
                packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT &&
                packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                return MAV_RESULT_FAILED;
            }
            // sanity check location
            if (!check_latlng(packet.x, packet.y)) {
                return MAV_RESULT_FAILED;
            }
            Location new_home_loc {};
            new_home_loc.lat = packet.x;
            new_home_loc.lng = packet.y;
            new_home_loc.alt = packet.z * 100;
            // handle relative altitude
            if (packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT || packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                if (!AP::ahrs().home_is_set()) {
                    // cannot use relative altitude if home is not set
                    return MAV_RESULT_FAILED;
                }
                new_home_loc.alt += plane.ahrs.get_home().alt;
            }
            plane.set_home(new_home_loc);
            AP::ahrs().lock_home();
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_REPOSITION: {
        // sanity check location
        if (!check_latlng(packet.x, packet.y)) {
            return MAV_RESULT_FAILED;
        }

        Location requested_position {};
        requested_position.lat = packet.x;
        requested_position.lng = packet.y;

        // check the floating representation for overflow of altitude
        if (fabsf(packet.z * 100.0f) >= 0x7fffff) {
            return MAV_RESULT_FAILED;
        }
        requested_position.alt = (int32_t)(packet.z * 100.0f);

        // load option flags
        if (packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
            requested_position.flags.relative_alt = 1;
        }
        else if (packet.frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
            requested_position.flags.terrain_alt = 1;
        }
        else if (packet.frame != MAV_FRAME_GLOBAL_INT) {
            // not a supported frame
            return MAV_RESULT_FAILED;
        }

        if (is_zero(packet.param4)) {
            requested_position.flags.loiter_ccw = 0;
        } else {
            requested_position.flags.loiter_ccw = 1;
        }

        if (location_sanitize(plane.current_loc, requested_position)) {
            // if the location wasn't already sane don't load it
            return MAV_RESULT_FAILED; // failed as the location is not valid
        }

        // location is valid load and set
        if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
            (plane.control_mode == GUIDED)) {
            plane.set_mode(GUIDED, MODE_REASON_GCS_COMMAND);
            plane.guided_WP_loc = requested_position;

            // add home alt if needed
            if (plane.guided_WP_loc.flags.relative_alt) {
                plane.guided_WP_loc.alt += plane.home.alt;
                plane.guided_WP_loc.flags.relative_alt = 0;
            }

            plane.set_guided_WP();

            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch(packet.command) {

case MAV_CMD_DO_DIGICAM_CONTROL: {
	setupuart(hal.uartE, "uartE");
	//bcode
	const uint8_t *command_t;
	//0x8x - first byte with x = addr of device: 0 is mainframe 1 is videocamSony/Tamron 2 is ATOM550
	//Sony/Tamron Commands	
	uint8_t tele[6]= {0x81,0x01,0x04,0x07,0x21,0xff}; // tele
	uint8_t wide[6]= {0x81,0x01,0x04,0x07,0x31,0xff}; // wide
	uint8_t stop[6]= {0x81,0x01,0x04,0x07,0x00,0xff}; // stop

    //uint8_t fix_zoom_1[9] = {0x81,0x01,0x04,0x47,0x00,0x00,0x00,0x00,0xff};

    //uint8_t fix_zoom_30[9] = {0x81,0x01,0x04,0x47,0x04,0x00,0x00,0x00,0xff};

    uint8_t fixed_zoom[9] = {0x81,0x01,0x04,0x47,0x00,0x00,0x00,0x00,0xff};

    int8_t zoom[30][4] = {
        {0x00, 0x00, 0x00, 0x00},
        {0x01, 0x06, 0x0A, 0x01},
        {0x02, 0x00, 0x06, 0x03},
        {0x02, 0x06, 0x02, 0x08},
        {0x02, 0x0A, 0x01, 0x0D},
        {0x02, 0x0D, 0x01, 0x03},
        {0x02, 0x0F, 0x06, 0x0D},
        {0x03, 0x01, 0x06, 0x01},
        {0x03, 0x03, 0x00, 0x0D},
        {0x03, 0x04, 0x08, 0x06},
        {0x03, 0x05, 0x0D, 0x07},
        {0x03, 0x07, 0x00, 0x09},
        {0x03, 0x08, 0x02, 0x00},
        {0x03, 0x09, 0x02, 0x00},
        {0x03, 0x0A, 0x00, 0x0A},
        {0x03, 0x0A, 0x0D, 0x0D},
        {0x03, 0x0B, 0x09, 0x0C},
        {0x03, 0x0C, 0x04, 0x06},
        {0x03, 0x0C, 0x0D, 0x0C},
        {0x03, 0x0D, 0x06, 0x00},
        {0x03, 0x0D, 0x0D, 0x04},
        {0x03, 0x0E, 0x03, 0x09},
        {0x03, 0x0E, 0x09, 0x00},
        {0x03, 0x0E, 0x0D, 0x0C},
        {0x03, 0x0F, 0x01, 0x0E},
        {0x03, 0x0F, 0x05, 0x07},
        {0x03, 0x0F, 0x08, 0x0A},
        {0x03, 0x0F, 0x0B, 0x06},
        {0x03, 0x0F, 0x0D, 0x0C},
        {0x04, 0x00, 0x00, 0x00}
    };

    //8x 01 04 47 0p 0q 0r 0s FF
	
	
	uint8_t wb1[6]= {0x81,0x01,0x04,0x35,0x00,0xff}; // wb1	 8x 01 04 35 01 FF
	uint8_t wb2[6]= {0x81,0x01,0x04,0x35,0x04,0xff}; // wb2 8x 01 04 35 02 FF
	uint8_t blc[6]= {0x81,0x01,0x04,0x33,0x02,0xff}; //blc 8x 01 04 33 02 FF on   // 33 03 for off
	uint8_t wdr[6]= {0x81,0x01,0x04,0x3d,0x02,0xff}; // wdr 8x 01 04 3d 02 FF on // 03 ff for off
	uint8_t dfg_s[6]= {0x81,0x01,0x04,0x37,0x02,0xff}; // dfg 8x 01 04 37 02 FF // 03 ff for off
	uint8_t dfg[7]= {0x81,0x01,0x04,0x37,0x02,0x00,0xff}; // dfg 8x 01 04 37 02 00 FF // 03 00 for off
	uint8_t nrd[6]= {0x81,0x01,0x04,0x53,0x01,0xff}; // nrd 8x 01 04 53 0p FF // p=1..5
	uint8_t zom[6]= {0x81,0x01,0x04,0x07,0x20,0xff}; // zom 8x 01 04 07 2p FF // 8x 01 04 07 3p FF ZoomOut //11-18 (p = 0..7)
	uint8_t afs[6]= {0x81,0x01,0x04,0x58,0x02,0xff}; // afs 8x 01 04 58 02 FF // 8x 01 04 58 03 FF  LOW
	uint8_t stb[6]= {0x81,0x01,0x04,0x34,0x02,0xff}; // stb 8x 01 04 34 02 FF // 8x 01 04 34 03 FF  off
	//Commands for ATOM550
	uint8_t auto_con[7]= {0xf0,0x03,0x26,0x03,0x02,0x2b,0xff}; //F0 03 26 03 02 2B FF – autoContrast
	uint8_t pol_w[7]= {0xf0,0x03,0x26,0x01,0x00,0x27,0xff}; //F0 03 26 01 00 27 FF – White 
	uint8_t pol_b[7]= {0xf0,0x03,0x26,0x01,0x0f,0x36,0xff}; //F0 03 26 01 0F 36 FF - Black
	uint8_t con_p[7]= {0xf0,0x03,0x26,0x05,0x04,0x2f,0xff}; //F0 03 26 05 04 2F FF – Plus 
	uint8_t con_m[7]= {0xf0,0x03,0x26,0x06,0x04,0x30,0xff}; //F0 03 26 06 04 30 FF - Minus
	uint8_t zom_n[7]= {0xf0,0x03,0x26,0x02,0x00,0x28,0xff}; //F0 03 26 02 00 28 FF – Normal
	uint8_t zom_2[7]= {0xf0,0x03,0x26,0x02,0x02,0x2a,0xff}; //F0 03 26 02 02 2A FF – 2x
	uint8_t zom_4[7]= {0xf0,0x03,0x26,0x02,0x04,0x2c,0xff}; //F0 03 26 02 04 2C FF - 4x
	uint8_t pal_1[7]= {0xf0,0x03,0x26,0x11,0x00,0x37,0xff}; //F0 03 26 11 00 37 FF - pal1
	uint8_t pal_2[7]= {0xf0,0x03,0x26,0x11,0x01,0x38,0xff}; //F0 03 26 11 01 38 FF - pal2
	uint8_t pal_3[7]= {0xf0,0x03,0x26,0x11,0x02,0x39,0xff}; //F0 03 26 11 02 39 FF - pal3
	uint8_t pal_4[7]= {0xf0,0x03,0x26,0x11,0x03,0x3a,0xff}; //F0 03 26 11 03 3a FF - pal4
	uint8_t pal_5[7]= {0xf0,0x03,0x26,0x11,0x04,0x3b,0xff}; //F0 03 26 11 04 3b FF - pal5
	uint8_t pal_6[7]= {0xf0,0x03,0x26,0x11,0x05,0x3c,0xff}; //F0 03 26 11 05 3c FF - pal6
	uint8_t pal_7[7]= {0xf0,0x03,0x26,0x11,0x06,0x3d,0xff}; //F0 03 26 11 06 3d FF - pal7
	uint8_t pal_8[7]= {0xf0,0x03,0x26,0x11,0x07,0x3e,0xff}; //F0 03 26 11 07 3e FF - pal8
	uint8_t pal_9[7]= {0xf0,0x03,0x26,0x11,0x08,0x3f,0xff}; //F0 03 26 11 08 3f FF - pal9
	uint8_t pal_s[6]= {0xf0,0x02,0x26,0x90,0xb6,0xff}; //F0 02 26 90 B6 FF - save pal
	
	//
	/*	
	if (is_equal(packet.param3,1.0f)) {
	   command_t =&tele[0];
           //hal.uartE->printf("\nZoomIn\n");
	   hal.uartE->write(command_t,6);

        } else   
	{if (is_equal(packet.param3,-1.0f))
           command_t =&wide[0];
           ////hal.uartE->printf("\nZoomOut\n"); 
	   hal.uartE->write(command_t,6);
        }
	*/
	if (is_equal(packet.param7,5.0f)||is_equal(packet.param7,-5.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		command_t =&wb1[0];
          	////hal.uartE->printf("\nWB1\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t = is_equal(packet.param7,5.0f)?&pol_w[0]:&pol_b[0];
				////hal.uartE->printf("\nPOL\n");
	   			hal.uartE->write(command_t,7);
				}
			}
	}
	if (is_equal(packet.param7,6.0f)||is_equal(packet.param7,-6.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		command_t =&wb2[0];
          	//hal.uartE->printf("\nWB2\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t = is_equal(packet.param7,6.0f)?&con_p[0]:&con_m[0];
				//hal.uartE->printf("\nCON\n");
	   			hal.uartE->write(command_t,7);
				}
			}
	}
	if (is_equal(packet.param7,7.0f)||is_equal(packet.param7,-7.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		blc[4] = is_equal(packet.param7,7.0f)?0x02:0x03;
		command_t =&blc[0];
          	//hal.uartE->printf("\nBLC\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t = is_equal(packet.param7,7.0f)?&zom_2[0]:&zom_n[0];
				//hal.uartE->printf("\nZOM\n");
	   			hal.uartE->write(command_t,7);
				}
			}
	}
	if (is_equal(packet.param7,77.0f)&&is_equal(packet.param6,3.0f)) { 
		
		command_t =&zom_4[0];
          	//hal.uartE->printf("\nZOM4\n");
	   	hal.uartE->write(command_t,7);
		}
		

	if (is_equal(packet.param7,8.0f)||is_equal(packet.param7,-8.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		wdr[4] = is_equal(packet.param7,8.0f)?0x03:0x00;
		command_t =&wdr[0];
          	//hal.uartE->printf("\nWDR\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {}
			}
	}
	if (is_equal(packet.param7,9.0f)||is_equal(packet.param7,-9.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		dfg[4] = is_equal(packet.param7,9.0f)?0x02:0x03;
		dfg_s[4] = is_equal(packet.param7,9.0f)?0x02:0x03;
		command_t =is_equal(packet.param6,1.0f)?&dfg_s[0]:&dfg[0];
          	//hal.uartE->printf("\nDFG\n");
		if (is_equal(packet.param6,1.0f)){hal.uartE->write(command_t,6);}
		else{
	   	hal.uartE->write(command_t,7);
		}}
		else	{
				if (is_equal(packet.param6,3.0f)) {}
			}
	}
	if (is_equal(packet.param7,10.0f)||is_equal(packet.param7,-10.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		nrd[4] = is_equal(packet.param7,10.0f)?0x03:0x00;
		command_t =&nrd[0];
          	//hal.uartE->printf("\nNRD\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_1[0];
          			//hal.uartE->printf("\nPAL1\n");
	   			hal.uartE->write(command_t,7);

				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);	
				}
			}
	}
	if (is_equal(packet.param7,11.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x21;
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_2[0];
          			//hal.uartE->printf("\nPAL2\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);	
				}
			}
	}
	if (is_equal(packet.param7,12.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x22;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_3[0];
          			//hal.uartE->printf("\nPAL3\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);	
				}
			}
	}
	if (is_equal(packet.param7,13.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x23;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_4[0];
          			//hal.uartE->printf("\nPAL4\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);	
}
			}
	}
	if (is_equal(packet.param7,14.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x24;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_5[0];
          			//hal.uartE->printf("\nPAL5\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);	
				}
			}
	}
	if (is_equal(packet.param7,15.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x25;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_6[0];
          			//hal.uartE->printf("\nPAL6\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);				
				}
			}
	}
	if (is_equal(packet.param7,16.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x26;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_7[0];
          			//hal.uartE->printf("\nPAL7\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);					
				}
			}
	}
	if (is_equal(packet.param7,17.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x27;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_8[0];
          			//hal.uartE->printf("\nPAL8\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);			
				}}
	}
	if (is_equal(packet.param7,18.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		zom[4] = 0x28;		
		command_t =&zom[0];
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
				command_t =&pal_9[0];
          			//hal.uartE->printf("\nPAL9\n");
	   			hal.uartE->write(command_t,7);
				
				//command_t =&pal_s[0];
          			//hal.uartE->printf("\nPALS\n");
	   			//hal.uartE->write(command_t,6);
				}
			}
	}
	if (is_equal(packet.param7,19.0f)||is_equal(packet.param7,-19.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		afs[4] = is_equal(packet.param7,19.0f)?0x02:0x03;
		command_t =&afs[0];
          	//hal.uartE->printf("\nAFS\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {}
			}
	}
	if (is_equal(packet.param7,20.0f)||is_equal(packet.param7,-20.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		if (is_equal(packet.param6,1.0f)){tele[4]=0x21;wide[4]=0x31;}
		else{tele[4] =0x25 ; wide[4] =0x35 ;}
		command_t = is_equal(packet.param7,20.0f)?&tele[0]:&wide[0];
		
          	//hal.uartE->printf("\nZOM\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {command_t =&pal_s[0];
          				//hal.uartE->printf("\nSTB\n");
	   				//hal.uartE->write(command_t,6);}
			}
            if (is_equal(packet.param7,20.0f)){
                //hal.uartE->write(fix_zoom_30,9);
            }
            else{
                //hal.uartE->write(fix_zoom_1,9);
            }
        }

	}

    if (((packet.param7) > 200.5f) && ((packet.param7) <= 230.2f)) { // from 201.0f to 230.0f
        if (is_equal(packet.param6,2.0f)){
        float fl = packet.param7;
        int zm = (int)fl - 201;

        fixed_zoom[4] = zoom[zm][0];
        fixed_zoom[5] = zoom[zm][1];
        fixed_zoom[6] = zoom[zm][2];
        fixed_zoom[7] = zoom[zm][3];

        hal.uartE->write(fixed_zoom,9);
        }
    }
	if (is_equal(packet.param7,21.0f)||is_equal(packet.param7,-21.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		stb[4] = is_equal(packet.param7,21.0f)?0x02:0x03;
		command_t =&stb[0];
          	//hal.uartE->printf("\nSTB\n");
	   	hal.uartE->write(command_t,6);
		}
		else	{
				if (is_equal(packet.param6,3.0f)) {
					command_t =&auto_con[0];
          				//hal.uartE->printf("\nSTB\n");
	   				hal.uartE->write(command_t,6);}
			}
	}
	if (is_equal(packet.param7,101.0f)||is_equal(packet.param7,102.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		nrd[4] = is_equal(packet.param7,101.0f)?0x01:0x02;
		command_t =&nrd[0];
          	//hal.uartE->printf("\nNRD\n");
	   	hal.uartE->write(command_t,6);
		}
	}    
	if (is_equal(packet.param7,103.0f)||is_equal(packet.param7,104.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		nrd[4] = is_equal(packet.param7,103.0f)?0x03:0x04;
		command_t =&nrd[0];
          	//hal.uartE->printf("\nNRD\n");
	   	hal.uartE->write(command_t,6);
		}
	}     
	if (is_equal(packet.param7,105.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		nrd[4] = 0x05;
		command_t =&nrd[0];
          	//hal.uartE->printf("\nNRD\n");
	   	hal.uartE->write(command_t,6);
		}
	}  
	if (is_equal(packet.param7,200.0f)) { 
		if (is_equal(packet.param6,1.0f)|| is_equal(packet.param6,2.0f)) {
		
		command_t =&stop[0];
          	//hal.uartE->printf("\nstop\n");
	   	hal.uartE->write(command_t,6);
		}
	}  

	
	
	return MAV_RESULT_ACCEPTED;
			} 

    case MAV_CMD_DO_CHANGE_SPEED:
        // if we're in failsafe modes (e.g., RTL, LOITER) or in pilot
        // controlled modes (e.g., MANUAL, TRAINING)
        // this command should be ignored since it comes in from GCS
        // or a companion computer:
        if (plane.control_mode != GUIDED && plane.control_mode != AUTO && plane.control_mode != AVOID_ADSB) {
            // failed
            return MAV_RESULT_FAILED;
        }

        AP_Mission::Mission_Command cmd;
        if (AP_Mission::mavlink_cmd_long_to_mission_cmd(packet, cmd) == MAV_MISSION_ACCEPTED) {
            plane.do_change_speed(cmd);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_NAV_LOITER_UNLIM:
        plane.set_mode(LOITER, MODE_REASON_GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        plane.set_mode(RTL, MODE_REASON_GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_TAKEOFF: {
        // user takeoff only works with quadplane code for now
        // param7 : altitude [metres]
        float takeoff_alt = packet.param7;
        if (plane.quadplane.available() && plane.quadplane.do_user_takeoff(takeoff_alt)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

#if MOUNT == ENABLED
        // Sets the region of interest (ROI) for the camera
    case MAV_CMD_DO_SET_ROI:
        // sanity check location
        if (!check_latlng(packet.param5, packet.param6)) {
            return MAV_RESULT_FAILED;
        }
        Location roi_loc;
        roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
        roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
        roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
        if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
            // switch off the camera tracking if enabled
            if (plane.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                plane.camera_mount.set_mode_to_default();
            }
        } else {
            // send the command to the camera mount
            plane.camera_mount.set_roi_target(roi_loc);
        }
        return MAV_RESULT_ACCEPTED;
#endif

#if MOUNT == ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:
        plane.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
        return MAV_RESULT_ACCEPTED;
#endif

    case MAV_CMD_MISSION_START:
        plane.set_mode(AUTO, MODE_REASON_GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (is_equal(packet.param1,1.0f)) {
            // run pre_arm_checks and arm_checks and display failures
            const bool do_arming_checks = !is_equal(packet.param2,magic_force_arm_value);
            if (plane.arm_motors(AP_Arming::MAVLINK, do_arming_checks)) {
                return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_FAILED;
        } else if (is_zero(packet.param1))  {
            if (plane.disarm_motors()) {
                return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_UNSUPPORTED;

    case MAV_CMD_DO_LAND_START:
        // attempt to switch to next DO_LAND_START command in the mission
        if (plane.mission.jump_to_landing_sequence()) {
            plane.set_mode(AUTO, MODE_REASON_UNKNOWN);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_GO_AROUND:
        if (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            // Initiate an aborted landing. This will trigger a pitch-up and
            // climb-out to a safe altitude holding heading then one of the
            // following actions will occur, check for in this order:
            // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
            //      increment mission index to execute it
            // - else if DO_LAND_START is available, jump to it
            // - else decrement the mission index to repeat the landing approach

            if (!is_zero(packet.param1)) {
                plane.auto_state.takeoff_altitude_rel_cm = packet.param1 * 100;
            }
            if (plane.landing.request_go_around()) {
                plane.auto_state.next_wp_crosstrack = false;
                return MAV_RESULT_ACCEPTED;
            }
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_FENCE_ENABLE:
        if (!plane.geofence_present()) {
            gcs().send_text(MAV_SEVERITY_NOTICE,"Fence not configured");
            return MAV_RESULT_FAILED;
        }
        switch((uint16_t)packet.param1) {
        case 0:
            if (! plane.geofence_set_enabled(false, GCS_TOGGLED)) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        case 1:
            if (! plane.geofence_set_enabled(true, GCS_TOGGLED)) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        case 2: //disable fence floor only
            if (! plane.geofence_set_floor_enabled(false)) {
                return MAV_RESULT_FAILED;
            }
            gcs().send_text(MAV_SEVERITY_NOTICE,"Fence floor disabled");
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_SET_HOME: {
        // param1 : use current (1=use current location, 0=use specified location)
        // param5 : latitude
        // param6 : longitude
        // param7 : altitude (absolute)
        if (is_equal(packet.param1,1.0f)) {
            plane.set_home_persistently(AP::gps().location());
            AP::ahrs().lock_home();
            return MAV_RESULT_ACCEPTED;
        } else {
            // ensure param1 is zero
            if (!is_zero(packet.param1)) {
                return MAV_RESULT_FAILED;
            }
            if (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7)) {
                // don't allow the 0,0 position
                return MAV_RESULT_FAILED;
            }
            // sanity check location
            if (!check_latlng(packet.param5,packet.param6)) {
                return MAV_RESULT_FAILED;
            }
            Location new_home_loc {};
            new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
            plane.set_home(new_home_loc);
            AP::ahrs().lock_home();
            return MAV_RESULT_ACCEPTED;
        }
        break;
    }

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        // param1 : enable/disable
        plane.autotune_enable(!is_zero(packet.param1));
        return MAV_RESULT_ACCEPTED;

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case 100:
            plane.parachute.enabled(false);
            return MAV_RESULT_ACCEPTED;
        case 101:
            plane.parachute.enabled(true);
            return MAV_RESULT_ACCEPTED;
        case 102:
            // treat as a manual release which performs some additional check of altitude
            if (plane.parachute.released()) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute already released");
                return MAV_RESULT_FAILED;
            }
            if (!plane.parachute.enabled()) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute not enabled");
                return MAV_RESULT_FAILED;
            }
            if (!plane.parachute_manual_release()) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_DO_MOTOR_TEST:
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        // param5 : motor count (number of motors to test in sequence)
        return plane.quadplane.mavlink_motor_test_start(chan,
                                                        (uint8_t)packet.param1,
                                                        (uint8_t)packet.param2,
                                                        (uint16_t)packet.param3,
                                                        packet.param4,
                                                        (uint8_t)packet.param5);

    case MAV_CMD_DO_VTOL_TRANSITION:
        if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_ENGINE_CONTROL:
        if (!plane.g2.ice_control.engine_control(packet.param1, packet.param2, packet.param3)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}

void GCS_MAVLINK_Plane::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (plane.g.fence_action != FENCE_ACTION_NONE) {
            send_text(MAV_SEVERITY_WARNING,"Fencing must be disabled");
        } else if (packet.count != plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else if (!check_latlng(packet.lat,packet.lng)) {
            send_text(MAV_SEVERITY_WARNING,"Invalid fence point, lat or lng too large");
        } else {
            plane.set_fence_point_with_index(Vector2l(packet.lat*1.0e7f, packet.lng*1.0e7f), packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (packet.idx >= plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else {
            Vector2l point = plane.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send_buf(msg, chan, msg->sysid, msg->compid, packet.idx, plane.g.fence_total,
                                             point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    {
#if MOUNT == ENABLED
        handle_gimbal_report(plane.camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: // get mount yaw for reversed fpv
    {
        mavlink_attitude_t att;
#if MOUNT == ENABLED
        if ((msg->sysid == 3) && (msg->compid == 156))
        {
            mavlink_msg_attitude_decode(msg, &att);
            plane.alexmos_heading = att.yaw;
        }
        
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != plane.g.sysid_my_gcs) {
            break; // Only accept control from our gcs
        }

        uint32_t tnow = AP_HAL::millis();

        mavlink_rc_channels_override_t packet;
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        RC_Channels::set_override(0, packet.chan1_raw, tnow);
        RC_Channels::set_override(1, packet.chan2_raw, tnow);
        RC_Channels::set_override(2, packet.chan3_raw, tnow);
        RC_Channels::set_override(3, packet.chan4_raw, tnow);
        RC_Channels::set_override(4, packet.chan5_raw, tnow);
        RC_Channels::set_override(5, packet.chan6_raw, tnow);
        RC_Channels::set_override(6, packet.chan7_raw, tnow);
        RC_Channels::set_override(7, packet.chan8_raw, tnow);

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = tnow;
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg->sysid != plane.g.sysid_my_gcs) {
            break; // only accept control from our gcs
        }

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        if (packet.target != plane.g.sysid_this_mav) {
            break; // only accept messages aimed at us
        }

        uint32_t tnow = AP_HAL::millis();

        int16_t roll = (packet.y == INT16_MAX) ? 0 : plane.channel_roll->get_radio_min() + (plane.channel_roll->get_radio_max() - plane.channel_roll->get_radio_min()) * (packet.y + 1000) / 2000.0f;
        int16_t pitch = (packet.x == INT16_MAX) ? 0 : plane.channel_pitch->get_radio_min() + (plane.channel_pitch->get_radio_max() - plane.channel_pitch->get_radio_min()) * (-packet.x + 1000) / 2000.0f;
        int16_t throttle = (packet.z == INT16_MAX) ? 0 : plane.channel_throttle->get_radio_min() + (plane.channel_throttle->get_radio_max() - plane.channel_throttle->get_radio_min()) * (packet.z) / 1000.0f;
        int16_t yaw = (packet.r == INT16_MAX) ? 0 : plane.channel_rudder->get_radio_min() + (plane.channel_rudder->get_radio_max() - plane.channel_rudder->get_radio_min()) * (packet.r + 1000) / 2000.0f;

        RC_Channels::set_override(uint8_t(plane.rcmap.roll() - 1), roll, tnow);
        RC_Channels::set_override(uint8_t(plane.rcmap.pitch() - 1), pitch, tnow);
        RC_Channels::set_override(uint8_t(plane.rcmap.throttle() - 1), throttle, tnow);
        RC_Channels::set_override(uint8_t(plane.rcmap.yaw() - 1), yaw, tnow);

        // a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = tnow;
        break;
    }
    
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != plane.g.sysid_my_gcs) break;
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HIL_STATE:
    {
#if HIL_SUPPORT
        if (plane.g.hil_mode != 1) {
            break;
        }

        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        // sanity check location
        if (!check_latlng(packet.lat, packet.lon)) {
            break;
        }

        last_hil_state = packet;

        // set gps hil sensor
        Location loc;
        memset(&loc, 0, sizeof(loc));
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        // setup airspeed pressure based on 3D speed, no wind
        plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);

        plane.gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                         packet.time_usec/1000,
                         loc, vel, 10, 0);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * GRAVITY_MSS*0.001f;
        accels.y = packet.yacc * GRAVITY_MSS*0.001f;
        accels.z = packet.zacc * GRAVITY_MSS*0.001f;

        plane.ins.set_gyro(0, gyros);
        plane.ins.set_accel(0, accels);

        plane.barometer.setHIL(packet.alt*0.001f);
        plane.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        plane.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        // cope with DCM getting badly off due to HIL lag
        if (plane.g.hil_err_limit > 0 &&
            (fabsf(packet.roll - plane.ahrs.roll) > ToRad(plane.g.hil_err_limit) ||
             fabsf(packet.pitch - plane.ahrs.pitch) > ToRad(plane.g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - plane.ahrs.yaw)) > ToRad(plane.g.hil_err_limit))) {
            plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
#endif
        break;
    }

#if MOUNT == ENABLED
    //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        plane.camera_mount.configure_msg(msg);
        break;
    }

    //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        plane.camera_mount.control_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        handle_radio_status(msg, plane.DataFlash, plane.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        plane.rangefinder.handle_msg(msg);
        break;

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
    {
        // Only allow companion computer (or other external controller) to
        // control attitude in GUIDED mode.  We DON'T want external control
        // in e.g., RTL, CICLE. Specifying a single mode for companion
        // computer control is more safe (even more so when using
        // FENCE_ACTION = 4 for geofence failures).
        if (plane.control_mode != GUIDED && plane.control_mode != AVOID_ADSB) { // don't screw up failsafes
            break; 
        }

        mavlink_set_attitude_target_t att_target;
        mavlink_msg_set_attitude_target_decode(msg, &att_target);

        // Mappings: If any of these bits are set, the corresponding input should be ignored.
        // NOTE, when parsing the bits we invert them for easier interpretation but transport has them inverted
        // bit 1: body roll rate
        // bit 2: body pitch rate
        // bit 3: body yaw rate
        // bit 4: unknown
        // bit 5: unknown
        // bit 6: reserved
        // bit 7: throttle
        // bit 8: attitude

        // if not setting all Quaternion values, use _rate flags to indicate which fields.

        // Extract the Euler roll angle from the Quaternion.
        Quaternion q(att_target.q[0], att_target.q[1],
                att_target.q[2], att_target.q[3]);

        // NOTE: att_target.type_mask is inverted for easier interpretation
        att_target.type_mask = att_target.type_mask ^ 0xFF;

        uint8_t attitude_mask = att_target.type_mask & 0b10000111; // q plus rpy

        uint32_t now = AP_HAL::millis();
        if ((attitude_mask & 0b10000001) ||    // partial, including roll
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.x = degrees(q.get_euler_roll()) * 100.0f;

            // Update timer for external roll to the nav control
            plane.guided_state.last_forced_rpy_ms.x = now;
        }

        if ((attitude_mask & 0b10000010) ||    // partial, including pitch
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.y = degrees(q.get_euler_pitch()) * 100.0f;

            // Update timer for external pitch to the nav control
            plane.guided_state.last_forced_rpy_ms.y = now;
        }

        if ((attitude_mask & 0b10000100) ||    // partial, including yaw
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.z = degrees(q.get_euler_yaw()) * 100.0f;

            // Update timer for external yaw to the nav control
            plane.guided_state.last_forced_rpy_ms.z = now;
        }
        if (att_target.type_mask & 0b01000000) { // throttle
            plane.guided_state.forced_throttle = att_target.thrust * 100.0f;

            // Update timer for external throttle
            plane.guided_state.last_forced_throttle_ms = now;
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            // don't allow the 0,0 position
            break;
        }
        // sanity check location
        if (!check_latlng(packet.latitude,packet.longitude)) {
            break;
        }
        Location new_home_loc {};
        new_home_loc.lat = packet.latitude;
        new_home_loc.lng = packet.longitude;
        new_home_loc.alt = packet.altitude / 10;
        plane.set_home(new_home_loc);
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

        // exit if vehicle is not in Guided mode
        if (plane.control_mode != GUIDED) {
            break;
        }

        // only local moves for now
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
            break;
        }

        // just do altitude for now
        plane.next_WP_loc.alt += -packet.z*100.0;
        gcs().send_text(MAV_SEVERITY_INFO, "Change alt to %.1f",
                        (double)((plane.next_WP_loc.alt - plane.home.alt)*0.01));
        
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
    {
        // Only want to allow companion computer position control when
        // in a certain mode to avoid inadvertently sending these
        // kinds of commands when the autopilot is responding to problems
        // in modes such as RTL, CIRCLE, etc.  Specifying ONLY one mode
        // for companion computer control is more safe (provided
        // one uses the FENCE_ACTION = 4 (RTL) for geofence failures).
        if (plane.control_mode != GUIDED && plane.control_mode != AVOID_ADSB) {
            //don't screw up failsafes
            break;
        }

        mavlink_set_position_target_global_int_t pos_target;
        mavlink_msg_set_position_target_global_int_decode(msg, &pos_target);
        // Unexpectedly, the mask is expecting "ones" for dimensions that should
        // be IGNORNED rather than INCLUDED.  See mavlink documentation of the
        // SET_POSITION_TARGET_GLOBAL_INT message, type_mask field.
        const uint16_t alt_mask = 0b1111111111111011; // (z mask at bit 3)
            
        bool msg_valid = true;
        AP_Mission::Mission_Command cmd = {0};
        
        if (pos_target.type_mask & alt_mask)
        {
            cmd.content.location.alt = pos_target.alt * 100;
            cmd.content.location.flags.relative_alt = false;
            cmd.content.location.flags.terrain_alt = false;
            switch (pos_target.coordinate_frame) 
            {
                case MAV_FRAME_GLOBAL_INT:
                    break; //default to MSL altitude
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    cmd.content.location.flags.relative_alt = true;          
                    break;
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    cmd.content.location.flags.relative_alt = true;          
                    cmd.content.location.flags.terrain_alt = true;
                    break;
                default:
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid coord frame in SET_POSTION_TARGET_GLOBAL_INT");
                    msg_valid = false;
                    break;
            }    

            if (msg_valid) {
                handle_change_alt_request(cmd);
            }
        } // end if alt_mask       

        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
        plane.adsb.handle_message(chan, msg);
        break;

    default:
        handle_common_message(msg);
        break;
    } // end switch
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Plane::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised) return;

    DataFlash.EnableWrites(false);

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs().send_text(MAV_SEVERITY_INFO, "Initialising APM");
    }

    DataFlash.EnableWrites(true);
}

/*
 *  send data streams in the given rate range on both links
 */
void Plane::gcs_data_stream_send(void)
{
    gcs().data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Plane::gcs_update(void)
{
    gcs().update();
}

/*
  send airspeed calibration data
 */
void Plane::gcs_send_airspeed_calibration(const Vector3f &vg)
{
    gcs().send_airspeed_calibration(vg);
}

/**
   retry any deferred messages
 */
void Plane::gcs_retry_deferred(void)
{
    gcs().retry_deferred();
}

/*
  return true if we will accept this packet. Used to implement SYSID_ENFORCE
 */
bool GCS_MAVLINK_Plane::accept_packet(const mavlink_status_t &status, mavlink_message_t &msg)
{
    if (!plane.g2.sysid_enforce) {
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_RADIO || msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        return true;
    }
    return (msg.sysid == plane.g.sysid_my_gcs);
}

Compass *GCS_MAVLINK_Plane::get_compass() const
{
    return &plane.compass;
}

AP_Mission *GCS_MAVLINK_Plane::get_mission()
{
    return &plane.mission;
}

void GCS_MAVLINK_Plane::handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg)
{
    plane.auto_state.next_wp_crosstrack = false;
    GCS_MAVLINK::handle_mission_set_current(mission, msg);
    if (plane.control_mode == AUTO && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }
}

AP_Camera *GCS_MAVLINK_Plane::get_camera() const
{
#if CAMERA == ENABLED
    return &plane.camera;
#else
    return nullptr;
#endif
}

AP_AdvancedFailsafe *GCS_MAVLINK_Plane::get_advanced_failsafe() const
{
    return &plane.afs;
}

AP_Rally *GCS_MAVLINK_Plane::get_rally() const
{
    return &plane.rally;
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool GCS_MAVLINK_Plane::set_mode(const uint8_t mode)
{
    switch (mode) {
    case MANUAL:
    case CIRCLE:
    case STABILIZE:
    case TRAINING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case AVOID_ADSB:
    case GUIDED:
    case AUTO:
    case RTL:
    case LOITER:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
        plane.set_mode((enum FlightMode)mode, MODE_REASON_GCS_COMMAND);
        return true;
    }
    return false;
}
