# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/ellipse/cmake-3.24.2/bin/cmake

# The command to remove a file.
RM = /home/ellipse/cmake-3.24.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Barnes_Hut_Demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Barnes_Hut_Demo.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Barnes_Hut_Demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Barnes_Hut_Demo.dir/flags.make

CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o: CMakeFiles/Barnes_Hut_Demo.dir/flags.make
CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o: /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/main.cc
CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o: CMakeFiles/Barnes_Hut_Demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o -MF CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o.d -o CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o -c /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/main.cc

CMakeFiles/Barnes_Hut_Demo.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Barnes_Hut_Demo.dir/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/main.cc > CMakeFiles/Barnes_Hut_Demo.dir/main.cc.i

CMakeFiles/Barnes_Hut_Demo.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Barnes_Hut_Demo.dir/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/main.cc -o CMakeFiles/Barnes_Hut_Demo.dir/main.cc.s

CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o: CMakeFiles/Barnes_Hut_Demo.dir/flags.make
CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o: /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/quadtree.cc
CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o: CMakeFiles/Barnes_Hut_Demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o -MF CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o.d -o CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o -c /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/quadtree.cc

CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/quadtree.cc > CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.i

CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/quadtree.cc -o CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.s

CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o: CMakeFiles/Barnes_Hut_Demo.dir/flags.make
CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o: /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/barnes_hut.cc
CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o: CMakeFiles/Barnes_Hut_Demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o -MF CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o.d -o CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o -c /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/barnes_hut.cc

CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/barnes_hut.cc > CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.i

CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/barnes_hut.cc -o CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.s

# Object files for target Barnes_Hut_Demo
Barnes_Hut_Demo_OBJECTS = \
"CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o" \
"CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o" \
"CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o"

# External object files for target Barnes_Hut_Demo
Barnes_Hut_Demo_EXTERNAL_OBJECTS =

Barnes_Hut_Demo: CMakeFiles/Barnes_Hut_Demo.dir/main.cc.o
Barnes_Hut_Demo: CMakeFiles/Barnes_Hut_Demo.dir/quadtree.cc.o
Barnes_Hut_Demo: CMakeFiles/Barnes_Hut_Demo.dir/barnes_hut.cc.o
Barnes_Hut_Demo: CMakeFiles/Barnes_Hut_Demo.dir/build.make
Barnes_Hut_Demo: CMakeFiles/Barnes_Hut_Demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Barnes_Hut_Demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Barnes_Hut_Demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Barnes_Hut_Demo.dir/build: Barnes_Hut_Demo
.PHONY : CMakeFiles/Barnes_Hut_Demo.dir/build

CMakeFiles/Barnes_Hut_Demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Barnes_Hut_Demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Barnes_Hut_Demo.dir/clean

CMakeFiles/Barnes_Hut_Demo.dir/depend:
	cd /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug /mnt/c/Users/송국선/CLionProjects/Barnes_Hut_Demo/cmake-build-debug/CMakeFiles/Barnes_Hut_Demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Barnes_Hut_Demo.dir/depend

