#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/ukf.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/io.o \
	${OBJECTDIR}/simCamera.o \
	${OBJECTDIR}/grid_features.o \
	${OBJECTDIR}/normalRandom.o \
	${OBJECTDIR}/simScene.o \
	${OBJECTDIR}/landmark.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_ts -lopencv_video -lboost_filesystem -lboost_system -lGL -lGLU -lglut

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ukf_slam

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ukf_slam: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ukf_slam ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/ukf.o: ukf.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/ukf.o ukf.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/io.o: io.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/io.o io.cpp

${OBJECTDIR}/simCamera.o: simCamera.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/simCamera.o simCamera.cpp

${OBJECTDIR}/grid_features.o: grid_features.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/grid_features.o grid_features.cpp

${OBJECTDIR}/normalRandom.o: normalRandom.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/normalRandom.o normalRandom.cpp

${OBJECTDIR}/simScene.o: simScene.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/simScene.o simScene.cpp

${OBJECTDIR}/landmark.o: landmark.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/eigen3 -MMD -MP -MF $@.d -o ${OBJECTDIR}/landmark.o landmark.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ukf_slam

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
