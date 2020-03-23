CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

PROJECT(ptmax C)

ADD_DEFINITIONS(-std=gnu17 -O2 -fno-strict-aliasing -Wall -Wextra -Wno-unused-parameter -Wno-unused-function)

# make assert() happy
STRING(REPLACE "-DNDEBUG" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")

SET(PTMAX_SRC
	ptmax.c
)

add_executable(ptmax ${PTMAX_SRC})

INSTALL(TARGETS ptmax LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR})
