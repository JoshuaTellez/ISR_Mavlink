/*
 * NGCP - UAV - Avionics
 * Purpose - Autonomously send GPS to ISR plane via mavlink
 *
 * @author - Joshua Tellez <joshtel@live.com>
 *
 */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
using namespace std;

#include "Mavlink/common/mavlink.h"
#include <fstream> //config file

#include "mavlink_interface.h"
#include "serial_port.h"
#include "UAV_Database.h"

/*
 * Quit Handler (Ctrl+C)
 * These objects will reference the actual mavlink and serial objects
 * mavlink_interface_quit = &mavlink_interface
 */
Mavlink_Interface *mavlink_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

// ------------------------------------------------------------------------------
//   TOP -- essentially main. main() calls this function
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

    /*
     * Config variables
     */
    int poll_rate;
    string port;

    /*
     * Preprocessing variables in config file
     */
    string line;
    ifstream config ("config.cfg");
    if(config.is_open()){
        int i = 1;
        while(getline(config,line)){

            switch (i++) {
                case 1:
                {
                    poll_rate = stoi(line);
                    printf("Polling rate: %d\n",poll_rate);
                }
                case 2:
                {
                    port = line;
                    break;
                }
            }

        }
    }

    else{
        printf("ERROR: Could not open config file\n");
    }

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS -- TODO
    // --------------------------------------------------------------------------

    // Default input argument
    char *uart_name = (char*) port.c_str();
    int baudrate = 57600;

    // do the parse, will throw an int if it fails
  //  parse_commandline(argc, argv, uart_name, baudrate);


    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it guards port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Mavlink_Interface mavlink_interface(&serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit         = &serial_port;
    mavlink_interface_quit = &mavlink_interface;
    signal(SIGINT,quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();


    /*
     * Set messages to read
     */
    mavlink_interface.messages_to_read.read_global_position_int = true;

    /*
     * Start reading messages
     */
    mavlink_interface.start();




    /*
     * Send GPS data to database
     */
    UAV_DatabaseConnect("plane1", "root", "ngcp");
    while(true){
        //Updating our GPS location
        UAV_InsertGPS_LOCAL(mavlink_interface.current_messages.global_position_int.relative_alt / 1E3,
                            mavlink_interface.current_messages.global_position_int.lat / 1E7,
                            mavlink_interface.current_messages.global_position_int.lon / 1E7);
        
        usleep(poll_rate);
    }



    /*
     * End of
     */

    

    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    mavlink_interface.stop();
    serial_port.stop();


    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    try {
        mavlink_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);

}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
    // This program uses throw, wrap one big try/catch here
    try
    {
        int result = top(argc,argv);
        return result;
    }

    catch ( int error )
    {
        fprintf(stderr,"ISR_Mavlink threw exception %i \n" , error);
        return error;
    }

}


