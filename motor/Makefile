# The name of the output program
TARGET      = motor_control

# Directories used for the SDK and object files
DIR_DXL    = ../DynamixelSDK
DIR_OBJS   = .objects

# Compiler options
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) -pthread  # Add -pthread flag here
FORMAT      = -m64

# SDK components
INCLUDES += -I$(DIR_DXL)/c++/include/dynamixel_sdk  # Use the correct path to your SDK's include directory
LIBRARIES  += -ldxl_x64_cpp
LIBRARIES  += -lrt

# Your new project's source files
SOURCES = src/motor_control.cpp

# Object files are the intermediate files we get when we compile our sources
OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))

# Rules to compile our project
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: src/%.cpp
	$(CX) $(CXFLAGS) -c $< -o $@
