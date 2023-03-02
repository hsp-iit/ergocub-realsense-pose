set(EXE_TARGET_NAME ergocub-rs-pose)

find_package(Eigen3 REQUIRED)

find_package(iDynTree REQUIRED)

find_package(YARP REQUIRED COMPONENTS
             dev
             os
             sig
)

find_package(icub-models REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    include/ergoCubHeadEncoders.h
    include/ForwardKinematics.h
    include/ForwardKinematicsiDynTree.h
    include/Probe.h
)

set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
    src/ergoCubHeadEncoders.cpp
    src/ForwardKinematicsiDynTree.cpp
    src/Probe.cpp
)

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
)

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      Eigen3::Eigen
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ${iDynTree_LIBRARIES}
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
