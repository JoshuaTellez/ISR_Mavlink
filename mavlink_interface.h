/*
 * NGCP UAV Mavlink Interface
 * Purpose: Be able to use mavlink commands to control and get data from the pixhawk
 *
 * Based off of https://github.com/mavlink/c_uart_interface_example
 * Main difference is restructuring of writing messages (send commands)
 *
 * @author Joshua Tellez joshtel@live.com
 *
 */
#ifndef MAVLINK_INTERFACE_H
#define MAVLINK_INTERFACE_H

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"
#include "UAV_Database.h"


#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <vector>

// ------------------------------------------------------------------------------
//   Defines -- from https://github.com/mavlink/c_uart_interface_example
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

// bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_YAW_RATE     0b0000010111111111


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();
void* start_mavlink_interface_read_thread(void *args);
void* start_mavlink_interface_write_thread(void *args);

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

/*
 * Timestamps of the different info recieved
 * i.e heartbeat info received at certain time
 */
struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint16_t autopilot_version;
    uint64_t power_status;
    uint64_t mission_ack;
    uint64_t command_ack;

    void
    reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        global_position_int = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
        autopilot_version = 0;
        power_status = 0;
        mission_ack = 0;
        command_ack = 0;
    }

};
/*
 * Boolean value for the desired messages to be read
 * Ex: To read battery status. Set read_battery_status to true
 */
struct Read_Messages {


    bool read_heartbeat;
    bool read_sys_status;
    bool read_battery_status;
    bool read_radio_status;
    bool read_local_position_ned;
    bool read_global_position_int;
    bool read_position_target_local_ned;
    bool read_position_target_global_int;
    bool read_highres_imu;
    bool read_attitude;
    bool read_autopilot_version;
    bool read_power_status;
    bool read_mission_ack;
    bool read_command_ack;

    /*Default to not read any messages*/
    Read_Messages()
    {
        read_heartbeat = false;
        read_sys_status = false;
        read_battery_status = false;
        read_radio_status = false;
        read_local_position_ned = false;
        read_global_position_int = false;
        read_position_target_local_ned = false;
        read_position_target_global_int = false;
        read_highres_imu = false;
        read_attitude = false;
        read_autopilot_version = false;
        read_power_status = false;
        read_mission_ack = false;
        read_command_ack = false;

    }

    /*
     * Read all messages
     */
    void read_all()
    {
        read_heartbeat = true;
        read_sys_status = true;
        read_battery_status = true;
        read_radio_status = true;
        read_local_position_ned = true;
        read_global_position_int = true;
        read_position_target_local_ned = true;
        read_position_target_global_int = true;
        read_highres_imu = true;
        read_attitude = true;
        read_autopilot_version = true;
        read_power_status = true;
        read_mission_ack = true;
        read_command_ack = true;
    }

};

/*
 * Used to hold the current messages recieved by the pixhawk
 */
struct Mavlink_Messages {

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    //version
    mavlink_autopilot_version_t autopilot_version;


    //power status
    mavlink_power_status_t power_status_t;

    // ack of commands
    mavlink_command_ack_t mavlink_command_ack;

    // ack of missions cmd
    mavlink_mission_ack_t mavlink_mission_ack;

    //mission item return from mission request by sending seq number
    mavlink_mission_item_t mavlink_mission_item;

    //mission current seq number
    mavlink_mission_current_t mavlink_mission_current;

    //mission count returned from mission request list only seq numbers
    mavlink_mission_count_t mavlink_mission_count;

    //mission item reached
    mavlink_mission_item_reached_t malink_mission_item_reached;


    // System Parameters?


    // Time Stamps
    Time_Stamps time_stamps;


    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }

};

class Mavlink_Interface{
public:
    //Constructor
    Mavlink_Interface(Serial_Port* sp);

    char reading_status;
    char writing_status;
    char control_status;
    uint64_t write_count;

    int system_id;
    int autopilot_id;
    int companion_id;

    Mavlink_Messages current_messages;
    Read_Messages messages_to_read; //which messages to read

    void read_messages();
    int  write_message(mavlink_message_t message);

    void enable_offboard_control();
    void disable_offboard_control();

    void start();
    void stop();
    void handle_quit( int sig );

    void start_read_thread();

    void write_set_servo(const int &servo, const int &pwm);
    void write_waypoints(std::vector<mavlink_mission_item_t> waypoints);

    mavlink_mission_item_t create_waypoint(const float &lat, const float &lon, const int &alt,const int &wp_number,
                                           const int &radius = 30, const int &by_pass_distance = 0, const int &yaw = NAN);
    int send_waypoint_count(mavlink_mission_count_t mavlink_mission_count);


    bool recieved_all_messages(const Time_Stamps &time_stamps);


private:
    Serial_Port *serial_port;

    bool time_to_exit;

    pthread_t read_tid;
    pthread_t write_tid;

    mavlink_set_position_target_global_int_t current_setpoint;

    void read_thread();
    void write_thread(void);

    int toggle_offboard_control( bool flag );
    void write_setpoint();

};

#endif //MAVLINK_INTERFACE_H
