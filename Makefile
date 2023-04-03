all: mavlink_control

mavlink_control: src/mavlink_control.cpp src/serial_port.cpp src/autopilot_interface.cpp
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 -I include -o $@ $^ -lpthread

modules:
	git submodule update --init --recursive

run:
	./mavlink_control -i /dev/ttyAMA1 -ib 57600 -o /dev/ttyAMA3 -ob 57600

clean:
	rm -rf *.o mavlink_control
