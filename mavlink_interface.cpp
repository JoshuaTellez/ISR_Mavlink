// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_interface.h"


// ----------------------------------------------------------------------------------
//   Mavlink Interface Class
// ----------------------------------------------------------------------------------


/*
 * Constructor
 */
Mavlink_Interface::
Mavlink_Interface(Serial_Port *serial_port_)
{
    // initialize attributes
    write_count = 0;

    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running
    control_status = 0;      // whether the autopilot is in offboard control mode
    time_to_exit   = false;  // flag to signal thread exit

    read_tid  = 0; // read thread id
    write_tid = 0; // write thread id

    system_id    = 0; // system id
    autopilot_id = 0; // pixhawk component id
    companion_id = 1; // companion computer component id

    current_messages.sysid  = system_id;
    current_messages.compid = autopilot_id;

    serial_port = serial_port_; // serial port management object

}

/*
 * Open threads and start listening for messages and writing commands
 */
void
Mavlink_Interface::
start() {
    int result;

    //Check Serial port
    if (serial_port->status != 1) // SERIAL_PORT_OPEN
    {
        fprintf(stderr, "ERROR: serial port not open\n");
        throw 1;
    }

    printf("START READ THREAD \n");

    //uses pthread helper function &start_mavlink_interface_read_thread
    //to call function start_read_thread in a new thread
    result = pthread_create(&read_tid, NULL, &start_mavlink_interface_read_thread, this);
    if (result) throw result;

    // now we're reading messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------

    printf("CHECK FOR MESSAGES\n");

    while ( not current_messages.sysid )
    {
        if ( time_to_exit )
            return;
        usleep(500000); // check at 2Hz
    }

    printf("Found\n");

    // now we know autopilot is sending messages
    printf("\n");


    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------

    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.

    // System ID
    if ( not system_id )
    {
        system_id = current_messages.sysid;
        printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
    }

    // Component ID
    if ( not autopilot_id )
    {
        autopilot_id = current_messages.compid;
        printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
        printf("\n");
    }

}


// -------------------------------------------------------------------------------------
// READ MESSAGES
// -------------------------------------------------------------------------------------

/*
 * If read thread is already running do nothing, else run read thread
 */
void
Mavlink_Interface::
start_read_thread()
{

    if ( reading_status != 0 )
    {
        fprintf(stderr,"read thread already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }

}

/*
 * Read messages at 10Hz
 */
void
Mavlink_Interface::
read_thread()
{
    reading_status = true;

    while ( ! time_to_exit )
    {
        read_messages();
        usleep(100000); // Read batches at 10Hz
    }

    reading_status = false;

    return;
}

/*
 * Decode messages recieved
 */
void
Mavlink_Interface::
read_messages()
{
    bool success;               // receive success flag
    bool received_all = false;  // recieved all desired messages?
    Time_Stamps this_timestamps;

    int count = 0;

    // Blocking wait for new data
    while ( !received_all && !time_to_exit )
    {
        //Read message from serial
        mavlink_message_t message;
        success = serial_port->read_message(message);

        //Handle recieved message
        if( success )
        {
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid  = message.sysid;
            current_messages.compid = message.compid;

            // Handle Message ID
            switch (message.msgid)
            {

                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                    current_messages.time_stamps.heartbeat = get_time_usec();
                    this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                    current_messages.time_stamps.sys_status = get_time_usec();
                    this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                    mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                    current_messages.time_stamps.battery_status = get_time_usec();
                    this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                    printf("Battery Status: %d", current_messages.battery_status.current_battery);
                    break;
                }

                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                    mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                    current_messages.time_stamps.radio_status = get_time_usec();
                    this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                    current_messages.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                    current_messages.time_stamps.global_position_int = get_time_usec();
                    this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                    printf("\nGPS Lat: %d\n", current_messages.global_position_int.lat);
                    printf("GPS Long: %d\n", current_messages.global_position_int.lon);
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                    current_messages.time_stamps.position_target_local_ned = get_time_usec();
                    this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                    current_messages.time_stamps.position_target_global_int = get_time_usec();
                    this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;

                    break;
                }

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                    current_messages.time_stamps.highres_imu = get_time_usec();
                    this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                    current_messages.time_stamps.attitude = get_time_usec();
                    this_timestamps.attitude = current_messages.time_stamps.attitude;
                    break;
                }

                case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                {
                    printf("recv version\n");
                    mavlink_msg_autopilot_version_decode(&message, &(current_messages.autopilot_version));
                    current_messages.time_stamps.autopilot_version = get_time_usec();
                    this_timestamps.autopilot_version = current_messages.time_stamps.autopilot_version;
                    break;
                }

                case MAVLINK_MSG_ID_POWER_STATUS:
                {
                    mavlink_msg_power_status_decode(&message, &(current_messages.power_status_t));
                    current_messages.time_stamps.power_status = get_time_usec();
                    this_timestamps.power_status = current_messages.time_stamps.power_status;
                   // printf("Power status: %d\n", current_messages.power_status_t.Vcc);
                    break;
                }

                case MAVLINK_MSG_ID_COMMAND_ACK:
                {
                    mavlink_msg_command_ack_decode(&message, &(current_messages.mavlink_command_ack));
                    current_messages.time_stamps.command_ack = get_time_usec();
                    this_timestamps.command_ack = current_messages.time_stamps.command_ack;
                    //printf("Command Acknowledged\n");
                }
                case MAVLINK_MSG_ID_MISSION_ACK:
                {
                    mavlink_msg_mission_ack_decode(&message, &(current_messages.mavlink_mission_ack));
                    current_messages.time_stamps.mission_ack = get_time_usec();
                    this_timestamps.mission_ack = current_messages.time_stamps.mission_ack;
                    //printf("Mission Acknowledged\n");`
                }

                default:
                {
                   // printf("Warning, did not handle message id %i\n",message.msgid);
                    //printf("message not processed %d\n",message.msgid);
                   // printf("Message id: %d\n", message.msgid);

                    break;
                }


            } // end: switch msgid

        } // end: if read message

        // Check for receipt of all items
        received_all = recieved_all_messages(this_timestamps);
        // give the write thread time to use the port
        if ( writing_status > false ) {
            usleep(100); // look for components of batches at 10kHz
        }


    } // end: while not received all and time to exit

    return;
}

/*
 * Check if all desired messages have been recieved
 */
bool
Mavlink_Interface::recieved_all_messages(const Time_Stamps &time_stamps) {

    //Returns false only if you want to read something and haven't
    // If you want to read attitude (1) and haven't (0). 1 == 0 ==> false
    // If you don't want do read attitude(0) and haven't(0). 0 == 0 ==> true
    return
        (messages_to_read.read_attitude == time_stamps.attitude)
        &&  (messages_to_read.read_autopilot_version == time_stamps.autopilot_version)
        &&  (messages_to_read.read_battery_status == time_stamps.battery_status)
        &&  (messages_to_read.read_global_position_int == time_stamps.global_position_int)
        &&  (messages_to_read.read_heartbeat == time_stamps.heartbeat)
        &&  (messages_to_read.read_highres_imu == time_stamps.highres_imu)
        &&  (messages_to_read.read_local_position_ned == time_stamps.local_position_ned)
        &&  (messages_to_read.read_position_target_global_int == time_stamps.position_target_global_int)
        &&  (messages_to_read.read_position_target_local_ned == time_stamps.position_target_local_ned)
        &&  (messages_to_read.read_radio_status == time_stamps.radio_status)
        &&  (messages_to_read.read_sys_status == time_stamps.sys_status)
        &&  (messages_to_read.read_power_status == time_stamps.power_status);
}
// -------------------------------------------------------------------------------------
// End read messages
// -------------------------------------------------------------------------------------




// -------------------------------------------------------------------------------------
// WRITE MESSAGES -- TODO
// -------------------------------------------------------------------------------------

/*
 * Send waypoints to pixhawk
 */
void
Mavlink_Interface::
write_waypoints(std::vector<mavlink_mission_item_t> waypoints) {

    printf("Sending Waypoints\n");
    writing_status = true;

    //Pixhawk needs to know how many waypoints it will receive
    mavlink_mission_count_t mission_count;
    mission_count.count = (int) waypoints.size();
    if(send_waypoint_count(mission_count) <= 0){
        fprintf(stderr,"WARNING: could not send waypoint count \n");
    }

    //Send all waypoints
    for(int i = 0; i < waypoints.size(); i++){
        waypoints[i].target_system = system_id;
        waypoints[i].target_component = autopilot_id;
        waypoints[i].frame = MAV_FRAME_GLOBAL;
        waypoints[i].autocontinue = true;
        waypoints[i].current = 1;
        mavlink_message_t message;
        mavlink_msg_mission_item_encode(system_id, companion_id, &message, &waypoints[i]);

        if ( write_message(message) <= 0 )
            fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");
    }

    writing_status = false;
}

/*
 * Tells pixhawk how many waypoints it will recieve
 */
int
Mavlink_Interface::
send_waypoint_count(mavlink_mission_count_t mavlink_mission_count) {
    mavlink_mission_count.target_system = system_id;
    mavlink_mission_count.target_component = autopilot_id;
    mavlink_message_t message;
    mavlink_msg_mission_count_encode(system_id, autopilot_id, &message, &mavlink_mission_count);
    return write_message(message);
}

/*
 * Change PWM value of servo. (Moves Servo)
 */
void
Mavlink_Interface::
write_set_servo(const int &servo, const int &pwm)
{
    printf("Changing PWM value of servo %d to %d\n", servo, pwm);
    writing_status = true;
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------


    mavlink_command_long_t cmd;
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_DO_SET_SERVO;
    cmd.param1 = servo;
    cmd.param2 = pwm;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id,companion_id, &message, &cmd);


    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send MAV_CMD_DO_SET_SERVO \n");

    writing_status = false;
}




/*
 * Sending message to pixhawk
 */
int
Mavlink_Interface::
write_message(mavlink_message_t message)
{
    // do the write
    int len = serial_port->write_message(message);

    // book keep
    write_count++;

    // Done!
    return len;
}

// -------------------------------------------------------------------------------------
// End write messages
// -------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------
// Helper functions for writing messages
// -------------------------------------------------------------------------------------


/*
 * Returns a mavlink acceptable waypoint
 */
mavlink_mission_item_t
Mavlink_Interface::create_waypoint(const float &lat, const float &lon, const int &alt, const int &wp_number,
                                   const int &radius, const int &by_pass_distance, const int &yaw) {
    mavlink_mission_item_t mission_item;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
    mission_item.param1 = 0;//hold time in decimal second IGNORED by ArduPlane
    mission_item.param2 = radius;//Acceptance radius in meters
    mission_item.param3 = by_pass_distance;//0 to pass through WP if >0 radius in meters to pass by WP
    mission_item.param4 = yaw;//Desired yaw angle NaN for unchanged
    mission_item.x = lat;//latitude
    mission_item.y = lon;//longitude
    mission_item.z = alt;//altitude
    mission_item.seq = wp_number;//waypoint number
    return mission_item;
}



// -------------------------------------------------------------------------------------
// Offboard control functions
// -------------------------------------------------------------------------------------

void
Mavlink_Interface::
enable_offboard_control()
{
    // Should only send this command once
    if ( control_status == false )
    {
        printf("ENABLE OFFBOARD MODE\n");

        /*
         * Toggle offboard control
         */

        // Sends the command to go off-board
        int success = toggle_offboard_control( true );

        // Check the command was written
        if ( success )
            control_status = true;
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if not offboard_status

}


/*
 * Stop Off-Board Mode
 */
void
Mavlink_Interface::
disable_offboard_control()
{

    // Should only send this command once
    if ( control_status == true )
    {
        printf("DISABLE OFFBOARD MODE\n");

        /*
         * Toggle offboard control
         */

        // Sends the command to stop off-board
        int success = toggle_offboard_control( false );

        // Check the command was written
        if ( success )
            control_status = false;
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if offboard_status

}


/*
 * Toggle offboard control
 */
int
Mavlink_Interface::
toggle_offboard_control( bool flag )
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0 };
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation     = true;
    com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = serial_port->write_message(message);

    // Done!
    return len;
}


// -------------------------------------------------------------------------------------
// End offboard control functions
// -------------------------------------------------------------------------------------



// -------------------------------------------------------------------------------------
// Close threads and quit handler
// -------------------------------------------------------------------------------------
void
Mavlink_Interface::
stop()
{

    printf("CLOSE THREADS\n");

    // signal exit
    time_to_exit = true;

    // wait for exit
    pthread_join(read_tid ,NULL);
    pthread_join(write_tid,NULL);

    // now the read and write threads are closed
    printf("\n");

    // still need to close the serial_port separately
}

/*
 * Quit Handler (Ctrl+C)
 *
 * Disables offboard control and closes threads
 */
void
Mavlink_Interface::
handle_quit( int sig )
{

    disable_offboard_control();

    try {
        stop();

    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop autopilot interface\n");
    }

}

// -------------------------------------------------------------------------------------
// End close threads and quit handler
// -------------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------
//   End Mavlink Interface Class
// ---------------------------------------------------------------------------------






// ----------------------------------------------------------------------------------
//   Time
// ------------------- --------------------------------------------------------------
uint64_t
get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}
//end Time


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_mavlink_interface_read_thread(void *args)
{
    // takes an autopilot object argument
    Mavlink_Interface *mavlink_interface = (Mavlink_Interface *)args;

    // run the object's read thread
    mavlink_interface->start_read_thread();

    // done!
    return NULL;
}

void*
start_mavlink_interface_write_thread(void *args)
{
    // takes an autopilot object argument
    Mavlink_Interface *mavlink_interface = (Mavlink_Interface *)args;

    // run the object's write thread
    //mavlink_interface->start_write_thread();  Not being used right now. Writing on main thread instead

    // done!
    return NULL;
}
// end pthread helper functions
