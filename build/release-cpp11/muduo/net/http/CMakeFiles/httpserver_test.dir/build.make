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
include muduo/net/http/CMakeFiles/httpserver_test.dir/depend.make

# Include the progress variables for this target.
include muduo/net/http/CMakeFiles/httpserver_test.dir/progress.make

# Include the compile flags for this target's objects.
include muduo/net/http/CMakeFiles/httpserver_test.dir/flags.make

muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o: muduo/net/http/CMakeFiles/httpserver_test.dir/flags.make
muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o: /home/ccpang/Desktop/Git/muduo/muduo/net/http/tests/HttpServer_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ccpang/Desktop/Git/build/release-cpp11/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o -c /home/ccpang/Desktop/Git/muduo/muduo/net/http/tests/HttpServer_test.cc

muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.i"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ccpang/Desktop/Git/muduo/muduo/net/http/tests/HttpServer_test.cc > CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.i

muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.s"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ccpang/Desktop/Git/muduo/muduo/net/http/tests/HttpServer_test.cc -o CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.s

muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.requires:

.PHONY : muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.requires

muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.provides: muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.requires
	$(MAKE) -f muduo/net/http/CMakeFiles/httpserver_test.dir/build.make muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.provides.build
.PHONY : muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.provides

muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.provides.build: muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o


# Object files for target httpserver_test
httpserver_test_OBJECTS = \
"CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o"

# External object files for target httpserver_test
httpserver_test_EXTERNAL_OBJECTS =

bin/httpserver_test: muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o
bin/httpserver_test: muduo/net/http/CMakeFiles/httpserver_test.dir/build.make
bin/httpserver_test: lib/libmuduo_http.a
bin/httpserver_test: lib/libmuduo_net.a
bin/httpserver_test: lib/libmuduo_base.a
bin/httpserver_test: muduo/net/http/CMakeFiles/httpserver_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ccpang/Desktop/Git/build/release-cpp11/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/httpserver_test"
	cd /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/httpserver_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
muduo/net/http/CMakeFiles/httpserver_test.dir/build: bin/httpserver_test

.PHONY : muduo/net/http/CMakeFiles/httpserver_test.dir/build

muduo/net/http/CMakeFiles/httpserver_test.dir/requires: muduo/net/http/CMakeFiles/httpserver_test.dir/tests/HttpServer_test.cc.o.requires

.PHONY : muduo/net/http/CMakeFiles/httpserver_test.dir/requires

muduo/net/http/CMakeFiles/httpserver_test.dir/clean:
	cd /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http && $(CMAKE_COMMAND) -P CMakeFiles/httpserver_test.dir/cmake_clean.cmake
.PHONY : muduo/net/http/CMakeFiles/httpserver_test.dir/clean

muduo/net/http/CMakeFiles/httpserver_test.dir/depend:
	cd /home/ccpang/Desktop/Git/build/release-cpp11 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ccpang/Desktop/Git/muduo /home/ccpang/Desktop/Git/muduo/muduo/net/http /home/ccpang/Desktop/Git/build/release-cpp11 /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http /home/ccpang/Desktop/Git/build/release-cpp11/muduo/net/http/CMakeFiles/httpserver_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : muduo/net/http/CMakeFiles/httpserver_test.dir/depend

