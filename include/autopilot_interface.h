/**
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "generic_port.h"
#include "mavlink_types.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>

#include <common/mavlink.h>

// ------------------------------------------------------------------------------
//   Defines
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
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();

void* start_autopilot_interface_telem_read_thread(void *args);
void* start_autopilot_interface_telem_write_thread(void *args);

void* start_autopilot_interface_uart_read_thread(void *args);
void* start_autopilot_interface_uart_write_thread(void *args);

struct MAVLink_Message {
	mavlink_message_t mavlink_message;
	std::mutex mutex;
	int sysid;
	int compid;
};

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
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
 */
class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Generic_Port *telem, Generic_Port *uart);
	~Autopilot_Interface();

	bool time_to_exit;

	char telem_reading_status;
	bool telem_read_ready;

	char telem_writing_status;
	uint64_t telem_write_count;
	bool telem_write_ready;

	char uart_reading_status;
	bool uart_read_ready;

	char uart_writing_status;
	uint64_t uart_write_count;
	bool uart_write_ready;

    int system_id;
	int autopilot_id;
	int companion_id;

	struct MAVLink_Message telem_send_message;
	struct MAVLink_Message telem_recv_message;
	struct MAVLink_Message uart_send_message;
	struct MAVLink_Message uart_recv_message;

	void telem_read_messages();
	int  telem_write_message(mavlink_message_t message);

	void uart_read_messages();
	int  uart_write_message(mavlink_message_t message);	

	void start();
	void stop();

	void start_telem_read_thread();
	void start_telem_write_thread(void);

	void start_uart_read_thread();
	void start_uart_write_thread(void);

	void handle_quit( int sig );


private:

	Generic_Port *telem_port;
	Generic_Port *uart_port;

	pthread_t telem_read_tid;
	pthread_t telem_write_tid;

	pthread_t uart_read_tid;
	pthread_t uart_write_tid;

	void telem_read_thread();
	void telem_write_thread(void);

	void uart_read_thread();
	void uart_write_thread(void);
};



#endif // AUTOPILOT_INTERFACE_H_


