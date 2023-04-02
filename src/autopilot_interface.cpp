/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"
#include "generic_port.h"
#include <mutex>


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Generic_Port *telem_port_, Generic_Port *uart_port_)
{
	// initialize attributes
	telem_write_count = 0;
	uart_write_count = 0;

	telem_reading_status = 0;      // whether the read thread is running
	telem_writing_status = 0;      // whether the write thread is running
	uart_reading_status = 0;      // whether the read thread is running
	uart_writing_status = 0;      // whether the write thread is running
	// control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	telem_read_tid  = 0; // read thread id
	telem_write_tid = 0; // write thread id

	uart_read_tid  = 0; // read thread id
	uart_write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	telem_messages.sysid  = system_id;
	telem_messages.compid = autopilot_id;

	uart_messages.sysid  = system_id;
	uart_messages.compid = autopilot_id;

	telem_port = telem_port_; // port management object
	uart_port = uart_port_;

}

Autopilot_Interface::
~Autopilot_Interface()
{}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
telem_read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = telem_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			telem_messages.sysid  = message.sysid;
			telem_messages.compid = message.compid;
			{
				std::lock_guard<std::mutex> lock(telem_messages.telem_mutex);
				telem_messages.mavlink_message = message;
			}
			received_all = true;
		} // end: if read message

		// give the write thread time to use the port
		if ( telem_writing_status > 0 ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

void
Autopilot_Interface::
uart_read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	// Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = uart_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			uart_messages.sysid  = message.sysid;
			uart_messages.compid = message.compid;
			{
				std::lock_guard<std::mutex> lock(uart_messages.uart_mutex);
				uart_messages.mavlink_message = message;
			}
			received_all = true;
		} // end: if read message

		// give the write thread time to use the port
		if ( uart_writing_status > 0 ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
telem_write_message(mavlink_message_t message)
{
	// do the write
	int len = telem_port->write_message(message);

	// book keep
	telem_write_count++;

	// Done!
	return len;
}

int
Autopilot_Interface::
uart_write_message(mavlink_message_t message)
{
	// do the write
	int len = uart_port->write_message(message);

	// book keep
	uart_write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------

	if ( !telem_port->is_running() || !uart_port->is_running() ) // PORT_OPEN
	{
		fprintf(stderr,"ERROR: port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &telem_read_tid, NULL, &start_autopilot_interface_telem_read_thread, this );
	if ( result ) throw result;
	result = pthread_create( &uart_read_tid, NULL, &start_autopilot_interface_uart_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( ! telem_messages.sysid )
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
		system_id = uart_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = uart_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}

	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &telem_write_tid, NULL, &start_autopilot_interface_telem_write_thread, this );
	if ( result ) throw result;
	result = pthread_create( &uart_write_tid, NULL, &start_autopilot_interface_uart_write_thread, this );
	if ( result ) throw result;

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(telem_read_tid ,NULL);
	pthread_join(telem_write_tid,NULL);
	pthread_join(uart_read_tid, NULL);
	pthread_join(uart_write_tid, NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_telem_read_thread()
{

	if ( telem_reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		telem_read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_telem_write_thread(void)
{
	if ( telem_writing_status )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		telem_write_thread();
		return;
	}

}

void
Autopilot_Interface::
start_uart_read_thread()
{

	if ( uart_reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		uart_read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_uart_write_thread(void)
{
	if ( uart_writing_status )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		uart_write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	// disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------

void
Autopilot_Interface::
telem_read_thread()
{
	telem_reading_status = true;

	while ( ! time_to_exit )
	{
		telem_read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	telem_reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
telem_write_thread(void)
{
	// signal startup
	telem_writing_status = 2;

	// write a message and signal writing
	telem_writing_status = 1;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		// write_setpoint();
	}

	// signal end
	telem_writing_status = 0;

	return;

}

void
Autopilot_Interface::
uart_read_thread()
{
	uart_reading_status = true;

	while ( ! time_to_exit )
	{
		uart_read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	uart_reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
uart_write_thread(void)
{
	// signal startup
	uart_writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	// {
	// 	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	// 	current_setpoint.data = sp;
	// }

	// write a message and signal writing
	// write_setpoint();
	uart_writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		// write_setpoint();
	}

	// signal end
	uart_writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_telem_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_telem_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_telem_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_telem_write_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_uart_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_uart_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_uart_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_uart_write_thread();

	// done!
	return NULL;
}

