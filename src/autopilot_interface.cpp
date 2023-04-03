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
#include "mavlink_types.h"
#include <mutex>
#include <unistd.h>


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
	telem_read_ready = false;

	telem_writing_status = 0;      // whether the write thread is running
	telem_write_ready = false;

	uart_reading_status = 0;      // whether the read thread is running
	uart_read_ready = false;

	uart_writing_status = 0;      // whether the write thread is running
	uart_write_ready = false;
	time_to_exit   = false;  // flag to signal thread exit

	telem_read_tid  = 0; // read thread id
	telem_write_tid = 0; // write thread id

	uart_read_tid  = 0; // read thread id
	uart_write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	telem_send_message.sysid  = system_id;
	telem_send_message.compid = autopilot_id;

	telem_recv_message.sysid  = system_id;
	telem_recv_message.compid = autopilot_id;

	uart_send_message.sysid  = system_id;
	uart_send_message.compid = autopilot_id;

	uart_recv_message.sysid  = system_id;
	uart_recv_message.compid = autopilot_id;

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
			{
				std::lock_guard<std::mutex> lock(telem_recv_message.mutex);
				telem_recv_message.mavlink_message = message;
				telem_recv_message.sysid  = message.sysid;
				telem_recv_message.compid = message.compid;
			}
			received_all = true;
			telem_read_ready = true;

			printf("Telem received message: [MAGIC]: %02X, [SYSID]: %d, [COMPID]: %d, [SEQ]: %d, [MSGID]: %d\n", message.magic, message.sysid, message.compid, message.seq, message.msgid);
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
			{
				std::lock_guard<std::mutex> lock(uart_recv_message.mutex);
				uart_recv_message.mavlink_message = message;
				uart_recv_message.sysid  = message.sysid;
				uart_recv_message.compid = message.compid;
			}
			received_all = true;
			uart_read_ready = true;

			printf("UART received message: [MAGIC]: %02X, [SYSID]: %d, [COMPID]: %d, [SEQ]: %d, [MSGID]: %d\n", message.magic, message.sysid, message.compid, message.seq, message.msgid);
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

	while ( ! telem_recv_message.sysid )
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
		system_id = telem_recv_message.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = telem_recv_message.compid;
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
	while (!time_to_exit) {
		while (!time_to_exit && !telem_write_ready) {

		}
		if (time_to_exit) {
			return;
		}
		telem_write_ready = false;

		telem_writing_status = 1;

		mavlink_message_t msg;
		{
			std::lock_guard<std::mutex> lock(telem_send_message.mutex);
			msg = telem_send_message.mavlink_message;
		}
		int len = telem_write_message(msg);
		if (len <= 0) {
			fprintf(stderr, "WARNING: Could not send message to telem");
		}
		printf("Telem sent message: [SEQ]: %d, [MSGID]: %d\n", msg.seq, msg.msgid);

		// signal end
		telem_writing_status = 0;

		usleep(250000);	// 4Hz
	}

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
	while (!time_to_exit) {
		while (!time_to_exit && !uart_write_ready) {

		}
		if (time_to_exit) {
			return;
		}
		uart_write_ready = false;

		uart_writing_status = true;

		mavlink_message_t msg;
		{
			std::lock_guard<std::mutex> lock(uart_send_message.mutex);
			msg = uart_send_message.mavlink_message;
		}
		int len = uart_write_message(msg);
		if (len <= 0) {
			fprintf(stderr, "WARNING: Could not send message to uart");
		}
		printf("UART sent message: [SEQ]: %d, [MSGID]: %d\n", msg.seq, msg.msgid);

		uart_writing_status = false;

		usleep(250000);	// 4Hz
	}

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

