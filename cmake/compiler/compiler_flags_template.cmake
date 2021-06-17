# Overview of used compiler properties for gcc / g++ compilers.
#
# Define the flags your toolchain support, and keep the unsupported flags empty.

#####################################################
# This section covers flags related to optimization #
#####################################################
set_compiler_property(PROPERTY no_optimization)

set_compiler_property(PROPERTY optimization_debug)

set_compiler_property(PROPERTY optimization_speed)

set_compiler_property(PROPERTY optimization_size)

#######################################################
# This section covers flags related to warning levels #
#######################################################

# Property for standard warning base in Zephyr, this will always bet set when compiling.
set_compiler_property(PROPERTY warning_base)

# GCC options for warning levels 1, 2, 3, when using `-DW=[1|2|3]`
# Property for warning levels 1, 2, 3 in Zephyr when using `-DW=[1|2|3]`
set_compiler_property(PROPERTY warning_dw_1)

set_compiler_property(PROPERTY warning_dw_2)

set_compiler_property(PROPERTY warning_dw_3)

# Extended warning set supported by the compiler
set_compiler_property(PROPERTY warning_extended)

# Compiler property that will issue error if a declaration does not specify a type
set_compiler_property(PROPERTY warning_error_implicit_int)

# Compiler flags to use when compiling according to MISRA
set_compiler_property(PROPERTY warning_error_misra_sane)

###########################################################################
# This section covers flags related to C or C++ standards / standard libs #
###########################################################################

# Compiler flags for C standard. The specific standard must be appended by user.
# For example, gcc specifies this as: set_compiler_property(PROPERTY cstd -std=)
set_compiler_property(PROPERTY cstd)

# Compiler flags for disabling C standard include and instead specify include
# dirs in nostdinc_include to use.
set_compiler_property(PROPERTY nostdinc)
set_compiler_property(PROPERTY nostdinc_include)

# Required C++ flags when compiling C++ code
set_property(TARGET compiler-cpp PROPERTY required)

# Compiler flags to use for specific C++ dialects
set_property(TARGET compiler-cpp PROPERTY dialect_cpp98)
set_property(TARGET compiler-cpp PROPERTY dialect_cpp11)
set_property(TARGET compiler-cpp PROPERTY dialect_cpp14)
set_property(TARGET compiler-cpp PROPERTY dialect_cpp17)
set_property(TARGET compiler-cpp PROPERTY dialect_cpp2a)

# Flag for disabling exeptions in C++
set_property(TARGET compiler-cpp PROPERTY no_exceptions)

# Flag for disabling rtti in C++
set_property(TARGET compiler-cpp PROPERTY no_rtti)


###################################################
# This section covers all remaining C / C++ flags #
###################################################

# Flags for coverage generation
set_compiler_property(PROPERTY coverage)

# Security canaries flags.
set_compiler_property(PROPERTY security_canaries)

set_compiler_property(PROPERTY security_fortify)

# Flag for a hosted (no-freestanding) application
set_compiler_property(PROPERTY hosted)

# gcc flag for a freestanding application
set_compiler_property(PROPERTY freestanding)

# Flag to include debugging symbol in compilation
set_compiler_property(PROPERTY debug)

set_compiler_property(PROPERTY no_common)

# Flags for imacros. The specific header must be appended by user.
set_compiler_property(PROPERTY imacros)

# Compiler flags for sanitizing.
set_compiler_property(PROPERTY sanitize_address)

set_compiler_property(PROPERTY sanitize_undefined)

# Compiler flag for turning off thread-safe initialization of local statics
set_property(TARGET compiler-cpp PROPERTY no_threadsafe_statics)

# Required ASM flags when compiling
set_property(TARGET asm PROPERTY required)
