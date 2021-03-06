
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = PuckPlayer

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./arm_fft.c \
		./mic_remote_control.c \
		./move_command.c \
		./ir_sensors.c \
		./distance.c \
		./maze_navigator.c \
		./corridor_navigation.c \
		./action_queue.c

#Header folders to include
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
