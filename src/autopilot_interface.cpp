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
#include "queue.h"
#include <bits/stdint-uintn.h>
#include <cstdio>
#include <mutex>
#include <pthread.h>
#include <semaphore.h>
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
	time_to_exit = false;

	// initialize attributes
	telem_write_count = 0;
	uart_write_count = 0;

	sem_init(&telem_read_ready, 0, 0);
	sem_init(&telem_write_ready, 0, 0);
	sem_init(&uart_read_ready, 0, 0);
	sem_init(&uart_write_ready, 0, 0);

	sem_init(&telem_read_finish, 0, 0);
	sem_init(&uart_read_finish, 0, 0);

	sem_post(&telem_read_finish);
	sem_post(&uart_read_finish);

	// sem_post(&telem_read_ready);
	// sem_post(&telem_write_ready);
	// sem_post(&uart_read_ready);
	// sem_post(&uart_write_ready);

	telem_read_tid  = 0; // read thread id
	telem_write_tid = 0; // write thread id

	uart_read_tid  = 0; // read thread id
	uart_write_tid = 0; // write thread id

	decrypt_tid = encrypt_tid = 0;

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	queue_init(&telem_recv_queue.message_queue);
	queue_init(&telem_send_queue.message_queue);
	queue_init(&uart_recv_queue.message_queue);
	queue_init(&uart_send_queue.message_queue);

	sem_init(&telem_recv_queue.sem, 0, 0);
	sem_init(&telem_send_queue.sem, 0, 0);
	sem_init(&uart_recv_queue.sem, 0, 0);
	sem_init(&uart_send_queue.sem, 0, 0);

	sem_post(&telem_recv_queue.sem);
	sem_post(&telem_send_queue.sem);
	sem_post(&uart_recv_queue.sem);
	sem_post(&uart_send_queue.sem);

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
	bool received_msg = false;  // receive only one message

	// Blocking wait for new data
	while ( !received_msg and !time_to_exit )
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
				// std::lock_guard<std::mutex> lock(telem_recv_queue.mutex);
				sem_wait(&telem_recv_queue.sem);
				if (enqueue(&telem_recv_queue.message_queue, message)) {
					fprintf(stderr, "WARNING: telem_recv_queue full!\n");
				}
				telem_recv_queue.sysid  = message.sysid;
				telem_recv_queue.compid = message.compid;
				sem_post(&telem_recv_queue.sem);
			}
			received_msg = true;

			printf("Telem received message: [MAGIC]: 0x%02X, [SYSID]: %d, [COMPID]: %d, [SEQ]: %d, [MSGID]: %d\n", message.magic, message.sysid, message.compid, message.seq, message.msgid);
		} // end: if read message
	} // end: while not received all

	return;
}

void
Autopilot_Interface::
uart_read_messages()
{
	bool success;               // receive success flag
	bool received_msg = false;  // receive only one message
	// Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_msg and !time_to_exit )
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
				// std::lock_guard<std::mutex> lock(uart_recv_queue.mutex);
				sem_wait(&uart_recv_queue.sem);
				if (enqueue(&uart_recv_queue.message_queue, message)) {
					fprintf(stderr, "WARNING: uart_recv_queue full!\n");
				}
				uart_recv_queue.sysid  = message.sysid;
				uart_recv_queue.compid = message.compid;
				sem_post(&uart_recv_queue.sem);
			}
			received_msg = true;

			printf("UART received message: [MAGIC]: 0x%02X, [SYSID]: %d, [COMPID]: %d, [SEQ]: %d, [MSGID]: %d\n", message.magic, message.sysid, message.compid, message.seq, message.msgid);
		} // end: if read message
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

	printf("START READ THREAD\n");
	result = pthread_create( &telem_read_tid, NULL, &start_autopilot_interface_telem_read_thread, this );
	if ( result ) throw result;
	result = pthread_create( &uart_read_tid, NULL, &start_autopilot_interface_uart_read_thread, this );
	if ( result ) throw result;

	printf("START WRITE THREAD\n");
	result = pthread_create( &telem_write_tid, NULL, &start_autopilot_interface_telem_write_thread, this );
	if ( result ) throw result;
	result = pthread_create( &uart_write_tid, NULL, &start_autopilot_interface_uart_write_thread, this );
	if ( result ) throw result;

	printf("START DECRYPT THREAD\n");
	result = pthread_create( &decrypt_tid, NULL, &start_autopilot_interface_decrypt_thread, this );
	if ( result ) {
		puts("Failed");
		throw result;
	}
	printf("START ENCRYPT THREAD\n");
	result = pthread_create( &encrypt_tid, NULL, &start_autopilot_interface_encrypt_thread, this );
	if ( result ) {
		puts("Failed");
		throw result;
	}

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
	pthread_join(decrypt_tid, NULL);
	pthread_join(encrypt_tid, NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the port separately
}

void
Autopilot_Interface::start_decrypt_thread()
{
	while ( ! time_to_exit )
	{
		sem_wait(&telem_read_ready);

		sem_wait(&telem_recv_queue.sem);
		sem_wait(&uart_send_queue.sem);

		printf("Telem copy to UART\n");

		uint32_t copy_size = telem_recv_queue.message_queue.size;
		if (copy_size + uart_send_queue.message_queue.size > MAX_QUEUE_SIZE) {
			printf("WARNING: uart_send_queue not enough\n");
			copy_size = MAX_QUEUE_SIZE - uart_send_queue.message_queue.size;
		}

		mavlink_message_t tmp;
		for (uint32_t i = 0; i < copy_size; i++) {
			if (dequeue(&telem_recv_queue.message_queue, &tmp)) {
				printf("Should not get here\n");
				break;
			}
			if (enqueue(&uart_send_queue.message_queue, tmp)) {
				printf("Should not get here\n");
				break;
			}
		}

		sem_post(&uart_send_queue.sem);
		sem_post(&telem_recv_queue.sem);

		// uart_write_thread
		sem_post(&uart_write_ready);

		// telem_read_thread resume
		// sem_post(&telem_read_finish);
	}
}

void
Autopilot_Interface::start_encrypt_thread()
{
	while (!time_to_exit) {
		sem_wait(&uart_read_ready);

		sem_wait(&uart_recv_queue.sem);
		sem_wait(&telem_send_queue.sem);

		printf("UART copy to Telem\n");

		uint32_t copy_size = uart_recv_queue.message_queue.size;
		if (copy_size + telem_send_queue.message_queue.size > MAX_QUEUE_SIZE) {
			printf("WARNING: telem_send_queue not enough\n");
			copy_size = MAX_QUEUE_SIZE - telem_send_queue.message_queue.size;
		}

		mavlink_message_t tmp;
		for (uint32_t i = 0; i < copy_size; i++) {
			if (dequeue(&uart_recv_queue.message_queue, &tmp)) {
				printf("Should not get here\n");
				break;
			}
			if (enqueue(&telem_send_queue.message_queue, tmp)) {
				printf("Should not get here\n");
				break;
			}
		}

		sem_post(&telem_send_queue.sem);
		sem_post(&uart_recv_queue.sem);

		// telem_write thread
		sem_post(&telem_write_ready);

		// uart_read thread resume
		// sem_post(&uart_read_finish);
	}
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_telem_read_thread()
{
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
	{
		telem_write_thread();
		return;
	}

}

void
Autopilot_Interface::
start_uart_read_thread()
{
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
	while ( ! time_to_exit )
	{
		sem_wait(&telem_read_finish);
		telem_read_messages();
		sem_post(&telem_read_ready);
	}

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
		sem_wait(&telem_write_ready);

		if (queue_empty(&telem_send_queue.message_queue)) {
			continue;
		}

		mavlink_message_t msg;
		{
			// std::lock_guard<std::mutex> lock(telem_send_queue.mutex);
			sem_wait(&telem_send_queue.sem);
			// printf("telem_write_thread obtains lock\n");
			if (dequeue(&telem_send_queue.message_queue, &msg)) {
				fprintf(stderr, "WARNING: telem_send_queue empty!\n");
			}
			sem_post(&telem_send_queue.sem);
		}
		int len = telem_write_message(msg);
		if (len <= 0) {
			fprintf(stderr, "WARNING: Could not send message to telem\n");
		} else {
			printf("Telem sent message: [SEQ]: %d, [SYSID]: %d, [COMPID]: %d, [MSGID]: %d\n", msg.seq, msg.sysid, msg.compid, msg.msgid);
		}

		// uart_read thread resume
		sem_post(&uart_read_finish);
	}

	return;

}

void
Autopilot_Interface::
uart_read_thread()
{
	while ( ! time_to_exit )
	{
		sem_wait(&uart_read_finish);
		uart_read_messages();
		sem_post(&uart_read_ready);
	}

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
		sem_wait(&uart_write_ready);

		if (queue_empty(&uart_send_queue.message_queue)) {
			continue;
		}

		mavlink_message_t msg;
		{
			// std::lock_guard<std::mutex> lock(uart_send_queue.mutex);
			sem_wait(&uart_send_queue.sem);
			if (dequeue(&uart_send_queue.message_queue, &msg)) {
				fprintf(stderr, "WARNING: uart_send_queue empty!\n");
			}
			sem_post(&uart_send_queue.sem);
		}
		int len = uart_write_message(msg);
		if (len <= 0) {
			fprintf(stderr, "WARNING: Could not send message to uart\n");
		} else {
			printf("UART sent message: [SEQ]: %d, [SYSID]: %d, [COMPID]: %d, [MSGID]: %d\n", msg.seq, msg.sysid, msg.compid, msg.msgid);
		}

		// telem_read_thread resume
		sem_post(&telem_read_finish);
	}

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_decrypt_thread(void *arg) {
	Autopilot_Interface *ai = (Autopilot_Interface *) arg;
	ai->start_decrypt_thread();

	return NULL;
}

void*
start_autopilot_interface_encrypt_thread(void *arg) {
	Autopilot_Interface *ai = (Autopilot_Interface *) arg;
	ai->start_encrypt_thread();

	return NULL;
}


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
