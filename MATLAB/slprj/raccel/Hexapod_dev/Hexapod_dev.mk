###########################################################################
## Makefile generated for component 'Hexapod_dev'. 
## 
## Makefile     : Hexapod_dev.mk
## Generated on : Mon Aug 24 18:16:12 2026
## Final product: .\Hexapod_dev.dll
## Product type : shared library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# DEF_FILE                Definition file

PRODUCT_NAME              = Hexapod_dev
MAKEFILE                  = Hexapod_dev.mk
MATLAB_ROOT               = C:\PROGRA~1\MATLAB\R2026a
MATLAB_BIN                = C:\PROGRA~1\MATLAB\R2026a\bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)\win64
START_DIR                 = C:\Users\james\OneDrive\Desktop\Hexapod\CAD\temp\xml
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 1
TGT_FCN_LIB               = ISO_C
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..\..\..
COMPILER_COMMAND_FILE     = Hexapod_dev_comp.rsp
CMD_FILE                  = Hexapod_dev.rsp
DEF_FILE                  = $(PRODUCT_NAME).def
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
NODEBUG                   = 1
LIBSSC_SLI_VCX64_OBJS     = 
LIBSM_SSCI_VCX64_OBJS     = 
LIBSSC_CORE_VCX64_OBJS    = 
LIBSM_VCX64_OBJS          = 
LIBPM_ST_VCX64_OBJS       = 
LIBMC_VCX64_OBJS          = 
LIBPM_MATH_VCX64_OBJS     = 
LIBPM_VCX64_OBJS          = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Microsoft Visual C++ 2022 v17.0 | nmake (64-bit Windows)
# Supported Version(s):    17.0
# ToolchainInfo Version:   2026a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS
# NODEBUG
# cvarsdll
# cvarsmt
# conlibsmt
# ldebug
# conflags
# cflags

#-----------
# MACROS
#-----------

MW_EXTERNLIB_DIR    = $(MATLAB_ROOT)\extern\lib\win64\microsoft
MW_LIB_DIR          = $(MATLAB_ROOT)\lib\win64
CPU                 = AMD64
APPVER              = 5.02
CVARSFLAG           = $(cvarsmt)
CFLAGS_ADDITIONAL   = -D_CRT_SECURE_NO_WARNINGS
CPPFLAGS_ADDITIONAL = -EHs -D_CRT_SECURE_NO_WARNINGS /wd4251 /Zc:__cplusplus
LIBS_TOOLCHAIN      = $(conlibs)

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Microsoft Visual C Compiler
CC = cl

# Linker: Microsoft Visual C Linker
LD = link

# C++ Compiler: Microsoft Visual C++ Compiler
CPP = cl

# C++ Linker: Microsoft Visual C++ Linker
CPP_LD = link

# Archiver: Microsoft Visual C/C++ Archiver
AR = lib

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)\mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: NMAKE Utility
MAKE = nmake


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -Z7
C_OUTPUT_FLAG       = -Fo
LDDEBUG             = /DEBUG
OUTPUT_FLAG         = -out:
CPPDEBUG            = -Z7
CPP_OUTPUT_FLAG     = -Fo
CPPLDDEBUG          = /DEBUG
OUTPUT_FLAG         = -out:
ARDEBUG             =
STATICLIB_OUTPUT_FLAG = -out:
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @ren
RUN                 = @cmd /C

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =



#---------------------------
# Model-Specific Options
#---------------------------

CFLAGS = $(cflags) $(cvarsdll) $(CFLAGS_ADDITIONAL) $(C_STANDARD_OPTS) /Od /Oy-

LDFLAGS = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)

SHAREDLIB_LDFLAGS = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) -dll -def:$(DEF_FILE)

CPPFLAGS = /TP $(cflags) $(cvarsdll) $(CPPFLAGS_ADDITIONAL) $(CPP_STANDARD_OPTS) /Od /Oy-

CPP_LDFLAGS = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)

CPP_SHAREDLIB_LDFLAGS = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) -dll -def:$(DEF_FILE)

ARFLAGS = /nologo

DOWNLOAD_FLAGS = 

EXECUTE_FLAGS = 

MAKE_FLAGS = -f $(MAKEFILE)

###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = .\Hexapod_dev.dll
PRODUCT_TYPE = "shared library"
BUILD_TYPE = "Shared Library Target"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=1 -DALLOCATIONFCN=0 -DONESTEPFCN=0 -DTERMFCN=1 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0
DEFINES_CUSTOM = -DEXT_MODE -DIS_RAPID_ACCEL
DEFINES_OPTS = -DTGTCONN -DIS_SIM_TARGET -DENABLE_SLEXEC_SSBRIDGE=1 -DNRT -DRSIM_PARAMETER_LOADING -DRSIM_WITH_SL_SOLVER -DUSE_LOCALHOST -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DON_TARGET_WAIT_FOR_START=0 -DTID01EQ=1
DEFINES_STANDARD = -DMODEL=Hexapod_dev -DNUMST=2 -DNCSTATES=61 -DHAVESTDIO

DEFINES = $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_create.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_setParameters.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_asserts.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_deriv.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_checkDynamics.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_compOutputs.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_asm_delegate.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_sim_delegate.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_mode_zero_crossings.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_logging.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_geometries.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_gateway.c $(START_DIR)\slprj\raccel\Hexapod_dev\rt_backsubrr_dbl.c $(START_DIR)\slprj\raccel\Hexapod_dev\rt_forwardsubrr_dbl.c $(START_DIR)\slprj\raccel\Hexapod_dev\rt_lu_real.c $(START_DIR)\slprj\raccel\Hexapod_dev\rt_matrixlib_dbl.c $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev.cpp $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_capi.cpp $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_data.cpp $(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_tgtconn.cpp $(START_DIR)\slprj\raccel\Hexapod_dev\rtGetInf.cpp $(START_DIR)\slprj\raccel\Hexapod_dev\rt_nonfinite.cpp $(MATLAB_ROOT)\rtw\c\raccel\raccel_main.c $(MATLAB_ROOT)\rtw\c\raccel\raccel_sup.c $(MATLAB_ROOT)\rtw\c\raccel\raccel_mat.c $(MATLAB_ROOT)\simulink\include\simulink_solver_api.c $(MATLAB_ROOT)\rtw\c\src\rapid\raccel_utils.c $(MATLAB_ROOT)\rtw\c\src\rapid\slsa_sim_common_utils.c $(MATLAB_ROOT)\rtw\c\src\ext_mode\common\ext_svr.c $(MATLAB_ROOT)\rtw\c\src\ext_mode\common\updown.c $(MATLAB_ROOT)\rtw\c\src\ext_mode\common\ext_work.c $(MATLAB_ROOT)\rtw\c\src\ext_mode\common\rtiostream_interface.c $(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip\rtiostream_tcpip.c $(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils\rtiostream_utils.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = Hexapod_dev_ceeedbd7_1.obj Hexapod_dev_ceeedbd7_1_create.obj Hexapod_dev_ceeedbd7_1_setParameters.obj Hexapod_dev_ceeedbd7_1_asserts.obj Hexapod_dev_ceeedbd7_1_deriv.obj Hexapod_dev_ceeedbd7_1_checkDynamics.obj Hexapod_dev_ceeedbd7_1_compOutputs.obj Hexapod_dev_ceeedbd7_1_asm_delegate.obj Hexapod_dev_ceeedbd7_1_sim_delegate.obj Hexapod_dev_ceeedbd7_1_mode_zero_crossings.obj Hexapod_dev_ceeedbd7_1_logging.obj Hexapod_dev_ceeedbd7_1_geometries.obj Hexapod_dev_ceeedbd7_1_gateway.obj rt_backsubrr_dbl.obj rt_forwardsubrr_dbl.obj rt_lu_real.obj rt_matrixlib_dbl.obj Hexapod_dev.obj Hexapod_dev_capi.obj Hexapod_dev_data.obj Hexapod_dev_tgtconn.obj rtGetInf.obj rt_nonfinite.obj raccel_main.obj raccel_sup.obj raccel_mat.obj simulink_solver_api.obj raccel_utils.obj slsa_sim_common_utils.obj ext_svr.obj updown.obj ext_work.obj rtiostream_interface.obj rtiostream_tcpip.obj rtiostream_utils.obj

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(MATLAB_ROOT)\extern\physmod\win64\ssc_sli\lib\ssc_sli_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\sm_ssci\lib\sm_ssci_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\ssc_core\lib\ssc_core_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\sm\lib\sm_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\pm_st\lib\pm_st_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\mc\lib\mc_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\pm_math\lib\pm_math_vcx64.lib $(MATLAB_ROOT)\extern\physmod\win64\pm\lib\pm_vcx64.lib

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = /LIBPATH:"$(MATLAB_ROOT)\extern\lib\win64\microsoft" "$(MW_EXTERNLIB_DIR)\libmwphysmod_common_logging2_core_rtw.lib" "$(MW_EXTERNLIB_DIR)\libmwphysmod_common_logging2_sdi_stream_rtw.lib" "$(MW_LIB_DIR)\libmwipp.lib" "$(MW_EXTERNLIB_DIR)\libmwipp.lib" "$(MW_EXTERNLIB_DIR)\libut.lib" "$(MW_EXTERNLIB_DIR)\libmx.lib" "$(MW_EXTERNLIB_DIR)\libmex.lib" "$(MW_EXTERNLIB_DIR)\libmat.lib" "$(MW_EXTERNLIB_DIR)\libmwmathutil.lib" "$(MW_EXTERNLIB_DIR)\libmwslsa_engine.lib" "$(MW_EXTERNLIB_DIR)\libmwslexec_simbridge.lib" "$(MW_EXTERNLIB_DIR)\libmwsl_fileio.lib" "$(MW_EXTERNLIB_DIR)\libmwsigstream.lib" "$(MW_EXTERNLIB_DIR)\libmwsl_AsyncioQueue.lib" "$(MW_EXTERNLIB_DIR)\libmwsl_services.lib" "$(MW_EXTERNLIB_DIR)\libmwsdi_raccel.lib" "$(MW_EXTERNLIB_DIR)\libmwcoder_target_services.lib" "$(MW_EXTERNLIB_DIR)\libmwcoder_ParamTuningTgtAppSvc.lib" "$(MW_EXTERNLIB_DIR)\libmwslpointerutil.lib" "$(MW_EXTERNLIB_DIR)\libmwfoundation_i18n_init_c_api.lib" "$(MW_EXTERNLIB_DIR)\libmwsimulinkcoder_capi.lib" "$(MW_EXTERNLIB_DIR)\libmwsl_simtarget_instrumentation.lib" "$(MW_EXTERNLIB_DIR)\libfixedpoint.lib" "$(MW_EXTERNLIB_DIR)\libmwslexec_simlog.lib" "$(MW_EXTERNLIB_DIR)\libmwstringutil.lib"

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_ = -utf-8
CFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CFLAGS = $(CFLAGS) $(CFLAGS_) $(CFLAGS_BASIC)

#-----------
# Linker
#-----------

LDFLAGS_ = -LARGEADDRESSAWARE

LDFLAGS = $(LDFLAGS) $(LDFLAGS_)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = -LARGEADDRESSAWARE

SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS) $(SHAREDLIB_LDFLAGS_)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_ = -utf-8
CPPFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS = $(CPPFLAGS) $(CPPFLAGS_) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = -LARGEADDRESSAWARE

CPP_LDFLAGS = $(CPP_LDFLAGS) $(CPP_LDFLAGS_)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = -LARGEADDRESSAWARE

CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS) $(CPP_SHAREDLIB_LDFLAGS_)

###########################################################################
## INLINED COMMANDS
###########################################################################


!include $(MATLAB_ROOT)\rtw\c\tools\vcdefs.mak


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute set_environment_variables


all : build
	@cmd /C @echo ### Successfully generated all binary outputs.


build : set_environment_variables prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


set_environment_variables : 
	@set INCLUDE=$(INCLUDES);$(INCLUDE)
	@set LIB=$(LIB)


###########################################################################
## FINAL TARGET
###########################################################################

#----------------------------------------
# Create a shared library
#----------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	@cmd /C @echo ### Creating shared library "$(PRODUCT)" ...
	$(CPP_LD) $(CPP_SHAREDLIB_LDFLAGS) -out:$(PRODUCT) @$(CMD_FILE) $(LIBS) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	@cmd /C @echo ### Created: "$(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\mc\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\mc\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\mc\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\mc\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_math\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_math\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_math\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_math\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_st\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_st\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_st\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\pm_st\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm_ssci\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm_ssci\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm_ssci\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\sm_ssci\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_core\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_core\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_core\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_core\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_sli\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_sli\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_sli\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\extern\physmod\win64\ssc_sli\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)\slprj\raccel\Hexapod_dev}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(START_DIR)\slprj\raccel\Hexapod_dev}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)\slprj\raccel\Hexapod_dev}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)\slprj\raccel\Hexapod_dev}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\simulink\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\simulink\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\simulink\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\simulink\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\simulink\blocks\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\simulink\blocks\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\simulink\blocks\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\simulink\blocks\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src\ext_mode\common}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src\ext_mode\common}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src\ext_mode\common}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\rtw\c\src\ext_mode\common}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


Hexapod_dev_ceeedbd7_1.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1.c"


Hexapod_dev_ceeedbd7_1_create.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_create.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_create.c"


Hexapod_dev_ceeedbd7_1_setParameters.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_setParameters.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_setParameters.c"


Hexapod_dev_ceeedbd7_1_asserts.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_asserts.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_asserts.c"


Hexapod_dev_ceeedbd7_1_deriv.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_deriv.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_deriv.c"


Hexapod_dev_ceeedbd7_1_checkDynamics.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_checkDynamics.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_checkDynamics.c"


Hexapod_dev_ceeedbd7_1_compOutputs.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_compOutputs.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_compOutputs.c"


Hexapod_dev_ceeedbd7_1_asm_delegate.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_asm_delegate.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_asm_delegate.c"


Hexapod_dev_ceeedbd7_1_sim_delegate.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_sim_delegate.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_sim_delegate.c"


Hexapod_dev_ceeedbd7_1_mode_zero_crossings.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_mode_zero_crossings.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_mode_zero_crossings.c"


Hexapod_dev_ceeedbd7_1_logging.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_logging.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_logging.c"


Hexapod_dev_ceeedbd7_1_geometries.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_geometries.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_geometries.c"


Hexapod_dev_ceeedbd7_1_gateway.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_gateway.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_ceeedbd7_1_gateway.c"


rt_backsubrr_dbl.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_backsubrr_dbl.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_backsubrr_dbl.c"


rt_forwardsubrr_dbl.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_forwardsubrr_dbl.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_forwardsubrr_dbl.c"


rt_lu_real.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_lu_real.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_lu_real.c"


rt_matrixlib_dbl.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_matrixlib_dbl.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_matrixlib_dbl.c"


Hexapod_dev.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev.cpp"


Hexapod_dev_capi.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_capi.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_capi.cpp"


Hexapod_dev_data.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_data.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_data.cpp"


Hexapod_dev_tgtconn.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_tgtconn.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\Hexapod_dev_tgtconn.cpp"


rtGetInf.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\rtGetInf.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\rtGetInf.cpp"


rt_nonfinite.obj : "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_nonfinite.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\slprj\raccel\Hexapod_dev\rt_nonfinite.cpp"


raccel_main.obj : "$(MATLAB_ROOT)\rtw\c\raccel\raccel_main.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\raccel\raccel_main.c"


raccel_sup.obj : "$(MATLAB_ROOT)\rtw\c\raccel\raccel_sup.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\raccel\raccel_sup.c"


raccel_mat.obj : "$(MATLAB_ROOT)\rtw\c\raccel\raccel_mat.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\raccel\raccel_mat.c"


simulink_solver_api.obj : "$(MATLAB_ROOT)\simulink\include\simulink_solver_api.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\simulink\include\simulink_solver_api.c"


raccel_utils.obj : "$(MATLAB_ROOT)\rtw\c\src\rapid\raccel_utils.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\src\rapid\raccel_utils.c"


slsa_sim_common_utils.obj : "$(MATLAB_ROOT)\rtw\c\src\rapid\slsa_sim_common_utils.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\src\rapid\slsa_sim_common_utils.c"


ext_svr.obj : "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\ext_svr.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\ext_svr.c"


updown.obj : "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\updown.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\updown.c"


ext_work.obj : "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\ext_work.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\ext_work.c"


rtiostream_interface.obj : "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\rtiostream_interface.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\rtw\c\src\ext_mode\common\rtiostream_interface.c"


rtiostream_tcpip.obj : "$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip\rtiostream_tcpip.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\rtiostreamtcpip\rtiostream_tcpip.c"


rtiostream_utils.obj : "$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils\rtiostream_utils.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\coder\rtiostream\src\utils\rtiostream_utils.c"


#------------------------
# BUILDABLE LIBRARIES
#------------------------

$(MATLAB_ROOT)\extern\physmod\win64\ssc_sli\lib\ssc_sli_vcx64.lib : $(LIBSSC_SLI_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBSSC_SLI_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\sm_ssci\lib\sm_ssci_vcx64.lib : $(LIBSM_SSCI_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBSM_SSCI_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\ssc_core\lib\ssc_core_vcx64.lib : $(LIBSSC_CORE_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBSSC_CORE_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\sm\lib\sm_vcx64.lib : $(LIBSM_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBSM_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\pm_st\lib\pm_st_vcx64.lib : $(LIBPM_ST_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBPM_ST_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\mc\lib\mc_vcx64.lib : $(LIBMC_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBMC_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\pm_math\lib\pm_math_vcx64.lib : $(LIBPM_MATH_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBPM_MATH_VCX64_OBJS)


$(MATLAB_ROOT)\extern\physmod\win64\pm\lib\pm_vcx64.lib : $(LIBPM_VCX64_OBJS)
	@cmd /C @echo ### Creating static library $@ ...
	$(AR) $(ARFLAGS) -out:$@ $(LIBPM_VCX64_OBJS)


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@cmd /C @echo ### PRODUCT = $(PRODUCT)
	@cmd /C @echo ### PRODUCT_TYPE = $(PRODUCT_TYPE)
	@cmd /C @echo ### BUILD_TYPE = $(BUILD_TYPE)
	@cmd /C @echo ### INCLUDES = $(INCLUDES)
	@cmd /C @echo ### DEFINES = $(DEFINES)
	@cmd /C @echo ### ALL_SRCS = $(ALL_SRCS)
	@cmd /C @echo ### ALL_OBJS = $(ALL_OBJS)
	@cmd /C @echo ### LIBS = $(LIBS)
	@cmd /C @echo ### MODELREF_LIBS = $(MODELREF_LIBS)
	@cmd /C @echo ### SYSTEM_LIBS = $(SYSTEM_LIBS)
	@cmd /C @echo ### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)
	@cmd /C @echo ### CFLAGS = $(CFLAGS)
	@cmd /C @echo ### LDFLAGS = $(LDFLAGS)
	@cmd /C @echo ### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)
	@cmd /C @echo ### CPPFLAGS = $(CPPFLAGS)
	@cmd /C @echo ### CPP_LDFLAGS = $(CPP_LDFLAGS)
	@cmd /C @echo ### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)
	@cmd /C @echo ### ARFLAGS = $(ARFLAGS)
	@cmd /C @echo ### MEX_CFLAGS = $(MEX_CFLAGS)
	@cmd /C @echo ### MEX_CPPFLAGS = $(MEX_CPPFLAGS)
	@cmd /C @echo ### MEX_LDFLAGS = $(MEX_LDFLAGS)
	@cmd /C @echo ### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)
	@cmd /C @echo ### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)
	@cmd /C @echo ### EXECUTE_FLAGS = $(EXECUTE_FLAGS)
	@cmd /C @echo ### MAKE_FLAGS = $(MAKE_FLAGS)


clean : 
	$(ECHO) "### Deleting all derived files ..."
	@if exist $(PRODUCT) $(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


