# Generated from orogen/lib/orogen/templates/typekit/corba/CMakeLists.txt



find_package(OrocosCORBA REQUIRED COMPONENTS Typekit)
INCLUDE_DIRECTORIES(${OrocosCORBA_INCLUDE_DIRS})
ADD_DEFINITIONS(${OrocosCORBA_DEFINES})

include_directories(${OrocosCORBA_INCLUDE_DIRS})
add_definitions(${OrocosCORBA_CFLAGS_OTHER})
link_directories(${OrocosCORBA_LIBRARY_DIRS})



# Set up generation of IDL files for CORBA support
IF (NOT OrocosCORBA_Typekit_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/CORBA")
ENDIF(NOT OrocosCORBA_Typekit_FOUND)

set(CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/HuboTypesC.cpp)
if(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")
    list(APPEND CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/HuboTypesDynSK.cpp)
endif(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")

add_custom_command(OUTPUT ${CORBA_FILES}
    COMMAND ${OrocosCORBA_IDL} -Wbkeep_inc_path
        
        ${CMAKE_CURRENT_SOURCE_DIR}/HuboTypes.idl
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/HuboTypes.idl)

add_definitions(${OrocosCORBA_Typekit_DEFINES})
include_directories(${OrocosCORBA_Typekit_INCLUDE_DIRS})

# Generate the CORBA transport
set(libname_corba Hubo-transport-corba-${OROCOS_TARGET})
add_library(${libname_corba} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/Convertions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TransportPlugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HuboData___std__string.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/__std__vector__double__.cpp
    ${CORBA_FILES})
target_link_libraries(${libname_corba}
    Hubo-typekit-${OROCOS_TARGET}
    ${OrocosCORBA_LIBRARIES})


target_link_libraries(${libname_corba} LINK_INTERFACE_LIBRARIES ${OrocosCORBA_LIBRARIES})

SET(PKG_CONFIG_FILE_CORBA ${CMAKE_CURRENT_BINARY_DIR}/Hubo-transport-corba-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(Hubo-transport-corba.pc.in ${PKG_CONFIG_FILE_CORBA} @ONLY)

install(TARGETS ${libname_corba} LIBRARY DESTINATION lib/orocos/types)
install(FILES ${PKG_CONFIG_FILE_CORBA} DESTINATION lib/pkgconfig)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Convertions.hpp
   ${CMAKE_CURRENT_SOURCE_DIR}/TransportPlugin.hpp
    Convertions.hpp
    HuboTypes.idl
    ${CMAKE_CURRENT_BINARY_DIR}/HuboTypesC.h
    DESTINATION include/orocos/Hubo/transports/corba)

