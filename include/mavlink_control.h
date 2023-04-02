// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#include <common/mavlink.h>

#include "autopilot_interface.h"
#include "serial_port.h"

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);

void commands(Autopilot_Interface &autopilot_interface, bool autotakeoff);
void parse_commandline(int argc, char **argv, char *&in_uart_name, int &in_baudrate, char *&out_uart_name, int &out_baudrate, bool &autotakeoff);

// quit handler
Autopilot_Interface *autopilot_interface_quit;
Generic_Port *port_quit_telem;
Generic_Port *port_quit_uart;
void quit_handler( int sig );

