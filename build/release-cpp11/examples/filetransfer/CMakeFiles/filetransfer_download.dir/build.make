# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ccpang/Desktop/Git/muduo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ccpang/Desktop/Git/build/release-cpp11

# Include any dependencies generated for this target.
include examples/filetransfer/CMakeFiles/filetransfer_download.dir/depend.make

# Include the progress variables for this target.
include examples/filetransfer/CMakeFiles/filetransfer_download.dir/progress.make

# Include the compile flags for this target's objects.
include examples/filetransfer/CMakeFiles/filetransfer_download.dir/flags.make

examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o: examples/filetransfer/CMakeFiles/filetransfer_download.dir/flags.make
examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o: /home/ccpang/Desktop/Git/muduo/examples/filetransfer/download.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ccpang/Desktop/Git/build/release-cpp11/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/filetransfer_download.dir/download.cc.o -c /home/ccpang/Desktop/Git/muduo/examples/filetransfer/download.cc

examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/filetransfer_download.dir/download.cc.i"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ccpang/Desktop/Git/muduo/examples/filetransfer/download.cc > CMakeFiles/filetransfer_download.dir/download.cc.i

examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/filetransfer_download.dir/download.cc.s"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ccpang/Desktop/Git/muduo/examples/filetransfer/download.cc -o CMakeFiles/filetransfer_download.dir/download.cc.s

examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.requires:

.PHONY : examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.requires

examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.provides: examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.requires
	$(MAKE) -f examples/filetransfer/CMakeFiles/filetransfer_download.dir/build.make examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.provides.build
.PHONY : examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.provides

examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.provides.build: examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o


# Object files for target filetransfer_download
filetransfer_download_OBJECTS = \
"CMakeFiles/filetransfer_download.dir/download.cc.o"

# External object files for target filetransfer_download
filetransfer_download_EXTERNAL_OBJECTS =

bin/filetransfer_download: examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o
bin/filetransfer_download: examples/filetransfer/CMakeFiles/filetransfer_download.dir/build.make
bin/filetransfer_download: lib/libmuduo_net.a
bin/filetransfer_download: lib/libmuduo_base.a
bin/filetransfer_download: examples/filetransfer/CMakeFiles/filetransfer_download.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ccpang/Desktop/Git/build/release-cpp11/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/filetransfer_download"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/filetransfer_download.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/filetransfer/CMakeFiles/filetransfer_download.dir/build: bin/filetransfer_download

.PHONY : examples/filetransfer/CMakeFiles/filetransfer_download.dir/build

examples/filetransfer/CMakeFiles/filetransfer_download.dir/requires: examples/filetransfer/CMakeFiles/filetransfer_download.dir/download.cc.o.requires

.PHONY : examples/filetransfer/CMakeFiles/filetransfer_download.dir/requires

examples/filetransfer/CMakeFiles/filetransfer_download.dir/clean:
	cd /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer && $(CMAKE_COMMAND) -P CMakeFiles/filetransfer_download.dir/cmake_clean.cmake
.PHONY : examples/filetransfer/CMakeFiles/filetransfer_download.dir/clean

examples/filetransfer/CMakeFiles/filetransfer_download.dir/depend:
	cd /home/ccpang/Desktop/Git/build/release-cpp11 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ccpang/Desktop/Git/muduo /home/ccpang/Desktop/Git/muduo/examples/filetransfer /home/ccpang/Desktop/Git/build/release-cpp11 /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer /home/ccpang/Desktop/Git/build/release-cpp11/examples/filetransfer/CMakeFiles/filetransfer_download.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/filetransfer/CMakeFiles/filetransfer_download.dir/depend

