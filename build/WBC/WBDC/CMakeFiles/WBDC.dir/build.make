# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build

# Include any dependencies generated for this target.
include WBC/WBDC/CMakeFiles/WBDC.dir/depend.make

# Include the progress variables for this target.
include WBC/WBDC/CMakeFiles/WBDC.dir/progress.make

# Include the compile flags for this target's objects.
include WBC/WBDC/CMakeFiles/WBDC.dir/flags.make

WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o: WBC/WBDC/CMakeFiles/WBDC.dir/flags.make
WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o: ../WBC/WBDC/WBDC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o"
	cd /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/WBDC.dir/WBDC.cpp.o -c /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/WBC/WBDC/WBDC.cpp

WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/WBDC.dir/WBDC.cpp.i"
	cd /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/WBC/WBDC/WBDC.cpp > CMakeFiles/WBDC.dir/WBDC.cpp.i

WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/WBDC.dir/WBDC.cpp.s"
	cd /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/WBC/WBDC/WBDC.cpp -o CMakeFiles/WBDC.dir/WBDC.cpp.s

WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.requires:

.PHONY : WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.requires

WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.provides: WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.requires
	$(MAKE) -f WBC/WBDC/CMakeFiles/WBDC.dir/build.make WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.provides.build
.PHONY : WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.provides

WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.provides.build: WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o


# Object files for target WBDC
WBDC_OBJECTS = \
"CMakeFiles/WBDC.dir/WBDC.cpp.o"

# External object files for target WBDC
WBDC_EXTERNAL_OBJECTS =

WBC/WBDC/libWBDC.dylib: WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o
WBC/WBDC/libWBDC.dylib: WBC/WBDC/CMakeFiles/WBDC.dir/build.make
WBC/WBDC/libWBDC.dylib: Utils/libSJutils.dylib
WBC/WBDC/libWBDC.dylib: Optimizer/gurobi/libSJgurobi.dylib
WBC/WBDC/libWBDC.dylib: Optimizer/Goldfarb/libSJGoldfarb.dylib
WBC/WBDC/libWBDC.dylib: ../Optimizer/gurobi/mac_lib/libgurobi_c++.a
WBC/WBDC/libWBDC.dylib: ../Optimizer/gurobi/mac_lib/libgurobi65.so
WBC/WBDC/libWBDC.dylib: WBC/WBDC/CMakeFiles/WBDC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libWBDC.dylib"
	cd /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/WBDC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
WBC/WBDC/CMakeFiles/WBDC.dir/build: WBC/WBDC/libWBDC.dylib

.PHONY : WBC/WBDC/CMakeFiles/WBDC.dir/build

WBC/WBDC/CMakeFiles/WBDC.dir/requires: WBC/WBDC/CMakeFiles/WBDC.dir/WBDC.cpp.o.requires

.PHONY : WBC/WBDC/CMakeFiles/WBDC.dir/requires

WBC/WBDC/CMakeFiles/WBDC.dir/clean:
	cd /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC && $(CMAKE_COMMAND) -P CMakeFiles/WBDC.dir/cmake_clean.cmake
.PHONY : WBC/WBDC/CMakeFiles/WBDC.dir/clean

WBC/WBDC/CMakeFiles/WBDC.dir/depend:
	cd /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/WBC/WBDC /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC /Users/donghyunkim/Sejong_Dynamic_Control_Toolkit/build/WBC/WBDC/CMakeFiles/WBDC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : WBC/WBDC/CMakeFiles/WBDC.dir/depend
