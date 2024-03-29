# See docs/CMake.html for instructions about how to build LLVM with CMake.

project(LLVM)
cmake_minimum_required(VERSION 2.8)

# Add path for custom modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules"
  )

set(LLVM_VERSION_MAJOR 3)
set(LLVM_VERSION_MINOR 1)

set(PACKAGE_VERSION "${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR}svn")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(VersionFromVCS)

option(LLVM_APPEND_VC_REV
  "Append the version control system revision id to LLVM version" OFF)

if( LLVM_APPEND_VC_REV )
  add_version_info_from_vcs(PACKAGE_VERSION)
endif()

set(PACKAGE_NAME LLVM)
set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
set(PACKAGE_BUGREPORT "http://llvm.org/bugs/")

# Sanity check our source directory to make sure that we are not trying to
# generate an in-tree build (unless on MSVC_IDE, where it is ok), and to make
# sure that we don't have any stray generated files lying around in the tree
# (which would end up getting picked up by header search, instead of the correct
# versions).
if( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR AND NOT MSVC_IDE )
  message(FATAL_ERROR "In-source builds are not allowed.
CMake would overwrite the makefiles distributed with LLVM.
Please create a directory and run cmake from there, passing the path
to this source directory as the last argument.
This process created the file `CMakeCache.txt' and the directory `CMakeFiles'.
Please delete them.")
endif()
if( NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR )
  file(GLOB_RECURSE
    tablegenned_files_on_include_dir
    "${CMAKE_CURRENT_SOURCE_DIR}/include/llvm/*.gen")
  file(GLOB_RECURSE
    tablegenned_files_on_lib_dir
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/Target/*.inc")
  if( tablegenned_files_on_include_dir OR tablegenned_files_on_lib_dir)
    message(FATAL_ERROR "Apparently there is a previous in-source build,
probably as the result of running `configure' and `make' on
${CMAKE_CURRENT_SOURCE_DIR}.
This may cause problems. The suspicious files are:
${tablegenned_files_on_lib_dir}
${tablegenned_files_on_include_dir}
Please clean the source directory.")
  endif()
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)

set(LLVM_MAIN_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(LLVM_MAIN_INCLUDE_DIR ${LLVM_MAIN_SRC_DIR}/include)
set(LLVM_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LLVM_TOOLS_BINARY_DIR ${LLVM_BINARY_DIR}/bin)
set(LLVM_EXAMPLES_BINARY_DIR ${LLVM_BINARY_DIR}/examples)
set(LLVM_LIBDIR_SUFFIX "" CACHE STRING "Define suffix of library directory name (32/64)" )

set(LLVM_ALL_TARGETS
  ARM
  CBackend
  CellSPU
  CppBackend
  Hexagon
  Mips
  MBlaze
  MSP430
  PowerPC
  PTX
  Sparc
  X86
  XCore
  XNCM
  )

# List of targets with JIT support:
set(LLVM_TARGETS_WITH_JIT X86 PowerPC ARM Mips)

if( MSVC )
  set(LLVM_TARGETS_TO_BUILD X86
    CACHE STRING "Semicolon-separated list of targets to build, or \"all\".")
else( MSVC )
  set(LLVM_TARGETS_TO_BUILD "all"
    CACHE STRING "Semicolon-separated list of targets to build, or \"all\".")
endif( MSVC )

option(BUILD_SHARED_LIBS
  "Build all libraries as shared libraries instead of static" OFF)

option(LLVM_ENABLE_CBE_PRINTF_A "Set to ON if CBE is enabled for printf %a output" ON)
if(LLVM_ENABLE_CBE_PRINTF_A)
  set(ENABLE_CBE_PRINTF_A 1)
endif()

option(LLVM_ENABLE_TIMESTAMPS "Enable embedding timestamp information in build" ON)
if(LLVM_ENABLE_TIMESTAMPS)
  set(ENABLE_TIMESTAMPS 1)
endif()

option(LLVM_ENABLE_FFI "Use libffi to call external functions from the interpreter" OFF)
set(FFI_LIBRARY_DIR "" CACHE PATH "Additional directory, where CMake should search for libffi.so")
set(FFI_INCLUDE_DIR "" CACHE PATH "Additional directory, where CMake should search for ffi.h or ffi/ffi.h")

set(LLVM_TARGET_ARCH "host"
  CACHE STRING "Set target to use for LLVM JIT or use \"host\" for automatic detection.")

option(LLVM_ENABLE_THREADS "Use threads if available." ON)

if( LLVM_TARGETS_TO_BUILD STREQUAL "all" )
  set( LLVM_TARGETS_TO_BUILD ${LLVM_ALL_TARGETS} )
endif()

set(LLVM_ENUM_TARGETS "")
foreach(c ${LLVM_TARGETS_TO_BUILD})
  list(FIND LLVM_ALL_TARGETS ${c} idx)
  if( idx LESS 0 )
    message(FATAL_ERROR "The target `${c}' does not exist.
    It should be one of\n${LLVM_ALL_TARGETS}")
  else()
    set(LLVM_ENUM_TARGETS "${LLVM_ENUM_TARGETS}LLVM_TARGET(${c})\n")
  endif()
endforeach(c)

set(llvm_builded_incs_dir ${LLVM_BINARY_DIR}/include/llvm)

include(AddLLVMDefinitions)

option(LLVM_ENABLE_PIC "Build Position-Independent Code" ON)

# MSVC has a gazillion warnings with this.
if( MSVC )
  option(LLVM_ENABLE_WARNINGS "Enable compiler warnings." OFF)
else( MSVC )
  option(LLVM_ENABLE_WARNINGS "Enable compiler warnings." ON)
endif()

option(LLVM_ENABLE_PEDANTIC "Compile with pedantic enabled." ON)
option(LLVM_ENABLE_WERROR "Fail and stop if a warning is triggered." OFF)

if( uppercase_CMAKE_BUILD_TYPE STREQUAL "RELEASE" )
  option(LLVM_ENABLE_ASSERTIONS "Enable assertions" OFF)
else()
  option(LLVM_ENABLE_ASSERTIONS "Enable assertions" ON)
endif()

option(LLVM_USE_INTEL_JITEVENTS
  "Use Intel JIT API to inform Intel(R) VTune(TM) Amplifier XE 2011 about JIT code"
  OFF)

if( LLVM_USE_INTEL_JITEVENTS )
  # Verify we are on a supported platform
  if( CMAKE_SYSTEM_NAME MATCHES "Windows" OR CMAKE_SYSTEM_NAME MATCHES "Linux" )
    # Directory where Intel Parallel Amplifier XE 2011 is installed.
    if ( WIN32 )
      set(LLVM_INTEL_JITEVENTS_DIR $ENV{VTUNE_AMPLIFIER_XE_2011_DIR})
    else ( WIN32 )
      set(LLVM_INTEL_JITEVENTS_DIR "/opt/intel/vtune_amplifier_xe_2011")
    endif ( WIN32 )

    # Set include and library search paths for Intel JIT Events API
    set(LLVM_INTEL_JITEVENTS_INCDIR "${LLVM_INTEL_JITEVENTS_DIR}/include")

    if ( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set(LLVM_INTEL_JITEVENTS_LIBDIR "${LLVM_INTEL_JITEVENTS_DIR}/lib64")
    else ( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set(LLVM_INTEL_JITEVENTS_LIBDIR "${LLVM_INTEL_JITEVENTS_DIR}/lib32")
    endif ( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  else()
    message(FATAL_ERROR
      "Intel JIT API support is available on Linux and Windows only.")
  endif()
endif( LLVM_USE_INTEL_JITEVENTS )

option(LLVM_USE_OPROFILE
  "Use opagent JIT interface to inform OProfile about JIT code" OFF)

# If enabled, ierify we are on a platform that supports oprofile.
if( LLVM_USE_OPROFILE )
  if( NOT CMAKE_SYSTEM_NAME MATCHES "Linux" )
    message(FATAL_ERROR "OProfile support is available on Linux only.") 
  endif( NOT CMAKE_SYSTEM_NAME MATCHES "Linux" )
endif( LLVM_USE_OPROFILE )

# Define an option controlling whether we should build for 32-bit on 64-bit
# platforms, where supported.
if( CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT WIN32 )
  # TODO: support other platforms and toolchains.
  option(LLVM_BUILD_32_BITS "Build 32 bits executables and libraries." OFF)
endif()

# Define the default arguments to use with 'lit', and an option for the user to
# override.
set(LIT_ARGS_DEFAULT "-sv")
if (MSVC OR XCODE)
  set(LIT_ARGS_DEFAULT "${LIT_ARGS_DEFAULT} --no-progress-bar")
endif()
set(LLVM_LIT_ARGS "${LIT_ARGS_DEFAULT}" CACHE STRING "Default options for lit")

# On Win32 hosts, provide an option to specify the path to the GnuWin32 tools.
if( WIN32 AND NOT CYGWIN )
  set(LLVM_LIT_TOOLS_DIR "" CACHE PATH "Path to GnuWin32 tools")
endif()

# On Win32 using MS tools, provide an option to set the number of parallel jobs
# to use.
if( MSVC_IDE AND ( MSVC90 OR MSVC10 ) )
  # Only Visual Studio 2008 and 2010 officially supports /MP.  Visual Studio
  # 2005 supports it but it is experimental.
  set(LLVM_COMPILER_JOBS "0" CACHE STRING
    "Number of parallel compiler jobs. 0 means use all processors. Default is 0.")
endif()

# Define options to control the inclusion and default build behavior for
# components which may not strictly be necessary (tools, runtime, examples, and
# tests).
#
# This is primarily to support building smaller or faster project files.
option(LLVM_INCLUDE_TOOLS "Generate build targets for the LLVM tools." ON)
option(LLVM_BUILD_TOOLS
  "Build the LLVM tools. If OFF, just generate build targets." ON)

option(LLVM_INCLUDE_RUNTIME "Generate build targets for the LLVM runtimes" ON)
option(LLVM_BUILD_RUNTIME
  "Build the LLVM runtime libraries. If OFF, just generate build targets." ON)

option(LLVM_BUILD_EXAMPLES
  "Build the LLVM example programs. If OFF, just generate build targets." OFF)
option(LLVM_INCLUDE_EXAMPLES "Generate build targets for the LLVM examples" ON)

option(LLVM_BUILD_TESTS
  "Build LLVM unit tests. If OFF, just generate build targets." OFF)
option(LLVM_INCLUDE_TESTS "Generate build targets for the LLVM unit tests." ON)

# All options referred to from HandleLLVMOptions have to be specified
# BEFORE this include, otherwise options will not be correctly set on
# first cmake run
include(config-ix)
include(HandleLLVMOptions)

# Verify that we can find a Python interpreter,
include(FindPythonInterp)
if( NOT PYTHONINTERP_FOUND )
  message(FATAL_ERROR
"Unable to find Python interpreter, required for builds and testing.

Please install Python or specify the PYTHON_EXECUTABLE CMake variable.")
endif()

######
# LLVMBuild Integration
#
# We use llvm-build to generate all the data required by the CMake based
# build system in one swoop:
#
#  - We generate a file (a CMake fragment) in the object root which contains
#    all the definitions that are required by CMake.
#
#  - We generate the library table used by llvm-config.
#
#  - We generate the dependencies for the CMake fragment, so that we will
#    automatically reconfigure outselves.

set(LLVMBUILDTOOL "${LLVM_MAIN_SRC_DIR}/utils/llvm-build/llvm-build")
set(LLVMCONFIGLIBRARYDEPENDENCIESINC
  "${LLVM_BINARY_DIR}/tools/llvm-config/LibraryDependencies.inc")
set(LLVMBUILDCMAKEFRAG
  "${LLVM_BINARY_DIR}/LLVMBuild.cmake")
message(STATUS "Constructing LLVMBuild project information")
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${LLVMBUILDTOOL}
            --native-target "${LLVM_NATIVE_ARCH}"
            --enable-targets "${LLVM_TARGETS_TO_BUILD}"
            --write-library-table ${LLVMCONFIGLIBRARYDEPENDENCIESINC}
            --write-cmake-fragment ${LLVMBUILDCMAKEFRAG}
            ERROR_VARIABLE LLVMBUILDOUTPUT
            ERROR_VARIABLE LLVMBUILDERRORS
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE LLVMBUILDRESULT)

# On Win32, CMake doesn't properly handle piping the default output/error
# streams into the GUI console. So, we explicitly catch and report them.
if( NOT "${LLVMBUILDOUTPUT}" STREQUAL "")
  message(STATUS "llvm-build output: ${LLVMBUILDOUTPUT}")
endif()
if( NOT "${LLVMBUILDRESULT}" STREQUAL "0" )
  message(FATAL_ERROR
    "Unexpected failure executing llvm-build: ${LLVMBUILDERRORS}")
endif()

# Include the generated CMake fragment. This will define properties from the
# LLVMBuild files in a format which is easy to consume from CMake, and will add
# the dependencies so that CMake will reconfigure properly when the LLVMBuild
# files change.
include(${LLVMBUILDCMAKEFRAG})

######

# Configure all of the various header file fragments LLVM uses which depend on
# configuration variables.
set(LLVM_ENUM_ASM_PRINTERS "")
set(LLVM_ENUM_ASM_PARSERS "")
set(LLVM_ENUM_DISASSEMBLERS "")
foreach(t ${LLVM_TARGETS_TO_BUILD})
  set( td ${LLVM_MAIN_SRC_DIR}/lib/Target/${t} )
  file(GLOB asmp_file "${td}/*AsmPrinter.cpp")
  if( asmp_file )
    set(LLVM_ENUM_ASM_PRINTERS
      "${LLVM_ENUM_ASM_PRINTERS}LLVM_ASM_PRINTER(${t})\n")
  endif()
  if( EXISTS ${td}/AsmParser/CMakeLists.txt )
    set(LLVM_ENUM_ASM_PARSERS
      "${LLVM_ENUM_ASM_PARSERS}LLVM_ASM_PARSER(${t})\n")
  endif()
  if( EXISTS ${td}/Disassembler/CMakeLists.txt )
    set(LLVM_ENUM_DISASSEMBLERS
      "${LLVM_ENUM_DISASSEMBLERS}LLVM_DISASSEMBLER(${t})\n")
  endif()
endforeach(t)

# Produce the target definition files, which provide a way for clients to easily
# include various classes of targets.
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Config/AsmPrinters.def.in
  ${LLVM_BINARY_DIR}/include/llvm/Config/AsmPrinters.def
  )
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Config/AsmParsers.def.in
  ${LLVM_BINARY_DIR}/include/llvm/Config/AsmParsers.def
  )
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Config/Disassemblers.def.in
  ${LLVM_BINARY_DIR}/include/llvm/Config/Disassemblers.def
  )
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Config/Targets.def.in
  ${LLVM_BINARY_DIR}/include/llvm/Config/Targets.def
  )

# Configure the three LLVM configuration header files.
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Config/config.h.cmake
  ${LLVM_BINARY_DIR}/include/llvm/Config/config.h)
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Config/llvm-config.h.cmake
  ${LLVM_BINARY_DIR}/include/llvm/Config/llvm-config.h)
configure_file(
  ${LLVM_MAIN_INCLUDE_DIR}/llvm/Support/DataTypes.h.cmake
  ${LLVM_BINARY_DIR}/include/llvm/Support/DataTypes.h)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LLVM_TOOLS_BINARY_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LLVM_BINARY_DIR}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LLVM_BINARY_DIR}/lib )

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories( ${LLVM_BINARY_DIR}/include ${LLVM_MAIN_INCLUDE_DIR})

if( ${CMAKE_SYSTEM_NAME} MATCHES SunOS )
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include llvm/Support/Solaris.h")
endif( ${CMAKE_SYSTEM_NAME} MATCHES SunOS )

include(AddLLVM)
include(TableGen)

if( MINGW )
  # People report that -O3 is unreliable on MinGW. The traditional
  # build also uses -O2 for that reason:
  llvm_replace_compiler_option(CMAKE_CXX_FLAGS_RELEASE "-O3" "-O2")
endif()

# Put this before tblgen. Else we have a circular dependence.
add_subdirectory(lib/Support)
add_subdirectory(lib/TableGen)

add_subdirectory(utils/TableGen)

add_subdirectory(include/llvm)

add_subdirectory(lib)

add_subdirectory(utils/FileCheck)
add_subdirectory(utils/FileUpdate)
add_subdirectory(utils/count)
add_subdirectory(utils/not)
add_subdirectory(utils/llvm-lit)
add_subdirectory(utils/json-bench)

add_subdirectory(projects)

if( LLVM_INCLUDE_TOOLS )
  add_subdirectory(tools)
endif()

if( LLVM_INCLUDE_RUNTIME )
  add_subdirectory(runtime)
endif()

if( LLVM_INCLUDE_EXAMPLES )
  add_subdirectory(examples)
endif()

if( LLVM_INCLUDE_TESTS )
  add_subdirectory(test)
  add_subdirectory(utils/unittest)
  add_subdirectory(unittests)
  if (MSVC)
    # This utility is used to prevent chrashing tests from calling Dr. Watson on
    # Windows.
    add_subdirectory(utils/KillTheDoctor)
  endif()
endif()

add_subdirectory(cmake/modules)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.def"
  PATTERN "*.h"
  PATTERN "*.td"
  PATTERN "*.inc"
  PATTERN "LICENSE.TXT"
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.def"
  PATTERN "*.h"
  PATTERN "*.gen"
  PATTERN "*.inc"
  # Exclude include/llvm/CMakeFiles/intrinsics_gen.dir, matched by "*.def"
  PATTERN "CMakeFiles" EXCLUDE
  PATTERN ".svn" EXCLUDE
  )

# TODO: make and install documentation.

set(CPACK_PACKAGE_VENDOR "LLVM")
set(CPACK_PACKAGE_VERSION_MAJOR ${LLVM_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${LLVM_VERSION_MINOR})
add_version_info_from_vcs(CPACK_PACKAGE_VERSION_PATCH)
include(CPack)

# Workaround for MSVS10 to avoid the Dialog Hell
# FIXME: This could be removed with future version of CMake.
if(MSVC_VERSION EQUAL 1600)
  set(LLVM_SLN_FILENAME "${CMAKE_CURRENT_BINARY_DIR}/LLVM.sln")
  if( EXISTS "${LLVM_SLN_FILENAME}" )
    file(APPEND "${LLVM_SLN_FILENAME}" "\n# This should be regenerated!\n")
  endif()
endif()
