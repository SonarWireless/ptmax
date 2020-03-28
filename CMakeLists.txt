CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

PROJECT(ptmax C)
include(GNUInstallDirs)

ADD_DEFINITIONS(-std=gnu17 -O2 -fno-strict-aliasing -Wall -Wextra -Wno-unused-parameter -Wno-unused-function)

# make assert() happy
STRING(REPLACE "-DNDEBUG" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")

SET(PTMAX_SRC
	ptmax.c
)

add_executable(ptmax ${PTMAX_SRC})

install(TARGETS ptmax RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
