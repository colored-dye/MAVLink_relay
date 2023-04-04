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
#include "queue.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>
#include <semaphore.h>

#include <common/mavlink.h>


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();

void* start_autopilot_interface_telem_read_thread(void *args);
void* start_autopilot_interface_telem_write_thread(void *args);

void* start_autopilot_interface_uart_read_thread(void *args);
void* start_autopilot_interface_uart_write_thread(void *args);

void *start_autopilot_interface_decrypt_thread(void *args);
void *start_autopilot_interface_encrypt_thread(void *args);

struct MAVLink_Message {
	queue_t message_queue;
	// std::mutex mutex;
	sem_t sem;
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

	uint64_t telem_write_count;
	uint64_t uart_write_count;

    int system_id;
	int autopilot_id;
	int companion_id;

	sem_t telem_read_ready, telem_write_ready;
	sem_t uart_read_ready, uart_write_ready;
	sem_t telem_read_finish, uart_read_finish;

	struct MAVLink_Message telem_send_queue;
	struct MAVLink_Message telem_recv_queue;
	struct MAVLink_Message uart_send_queue;
	struct MAVLink_Message uart_recv_queue;

	void telem_read_messages();
	int  telem_write_message(mavlink_message_t message);

	void uart_read_messages();
	int  uart_write_message(mavlink_message_t message);	

	void start();
	void stop();

	void start_decrypt_thread();
	void start_encrypt_thread();

	void start_telem_read_thread();
	void start_telem_write_thread();
	void start_uart_read_thread();
	void start_uart_write_thread();

	void handle_quit( int sig );


private:

	Generic_Port *telem_port;
	Generic_Port *uart_port;

	pthread_t telem_read_tid;
	pthread_t telem_write_tid;

	pthread_t uart_read_tid;
	pthread_t uart_write_tid;

	pthread_t encrypt_tid;
	pthread_t decrypt_tid;

	void telem_read_thread();
	void telem_write_thread(void);

	void uart_read_thread();
	void uart_write_thread(void);
};



#endif // AUTOPILOT_INTERFACE_H_


