C_SRCS = $(wildcard src/*.c)
CPP_SRCS =  $(wildcard src/*.cpp)
SRCS = $(C_SRCS) $(CPP_SRCS)
OBJS = $(patsubst %.c,%.o,$(C_SRCS)) $(patsubst %.cpp,%.o,$(CPP_SRCS))
CFLAGS := -g -Wall -I mavlink/include/mavlink/v2.0 -I include
LDFLAGS := -lpthread

all: mavlink_control

mavlink_control: $(OBJS)
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	g++ $(CFLAGS) -c -o $@ $<

%.o: %.cpp
	g++ $(CFLAGS) -c -o $@ $<

modules:
	git submodule update --init --recursive

run:
	./mavlink_control -i /dev/ttyAMA1 -ib 57600 -o /dev/ttyAMA3 -ob 57600

clean:
	rm -rf *.o mavlink_control
