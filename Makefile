#Compiler and Linker
CXX	= g++
LD	= g++

#The Directories, Source, Includes, Objects, Binary and Resources
BUILDDIR	= build
TARGETDIR 	= bin
SRCEXT		= cpp
OBJEXT		= o

#Flags, Libraries and Includes
INCLIST = $(shell find -name include -type d)
INC		= $(addprefix -I, $(INCLIST))
#The Target Binary Program
LIBS    = -levent
LIBDIR  = ./libs/libevent-2.1.8/.libs
#The Target Binary Program
TARGET   = main
CXXFLAGS = -std=c++11 -g
LDFLAGS  = -L $(LIBDIR)
LDFLAGS += -Wl,-rpath,$(LIBDIR)



#---------------------------------------------------------------------------------
#DO NOT EDIT BELOW THIS LINE
#---------------------------------------------------------------------------------

#List of all sources and objects
SOURCES = $(shell find -name *.cpp)
OBJECTS = $(addprefix $(BUILDDIR)/,$(patsubst %.cpp,%.o, $(notdir $(SOURCES))))
VPATH 	= $(dir $(SOURCES))

#Default Make
all: directories $(TARGET)

#Remake
remake: clean all

#Create directories
directories:
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(TARGETDIR)

#Linking
$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $(TARGETDIR)/$(TARGET) $(LIBS)

#Compiling
$(BUILDDIR)/%.o : %.cpp
	$(CXX) $(INC) $(CXXFLAGS) -c $< -o $@

#Clean all
clean:
	rm -rf $(BUILDDIR)
	rm -rf $(TARGETDIR)

#TO-DO: Create Test rule:

#Non-File Targets
.PHONY: all remake clean directories
