cmake_minimum_required(VERSION 3.0)

PROJECT(equi2omni)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
	INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

# libPeR COmni related functionalities...
find_package(PER REQUIRED per_core per_io)
if(PER_FOUND)
	include(${PER_USE_FILE})
endif(PER_FOUND)

ADD_EXECUTABLE(equi2omni

		equi2omni.cpp
)

