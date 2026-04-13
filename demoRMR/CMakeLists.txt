set(SOURCES
    main.cpp
    mainwindow.cpp
    robot.cpp
)

set(HEADERS
    mainwindow.h
    robot.h
)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

if(NOT DISABLE_JOYSTICK)
    add_compile_definitions(USING_QJOYSTICKS)
endif()

qt_add_executable(${PROJECT_NAME} WIN32
    ${SOURCES}
    ${HEADERS}
    mainwindow.ui
    odometry.cpp
    odometry.h
    path_tracker.h
    path_tracker.cpp
    lidarodometry.h
    lidarodometry.cpp
    navigation.h
    navigation.cpp
    mapper.cpp
    mapper.h
    utility.h
    utility.cpp
    custom_types.h
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_LIBRARY)
if(DISABLE_OPENCV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_OPENCV)
endif()
if(DISABLE_SKELETON)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_SKELETON)
endif()
if(DISABLE_JOYSTICK)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_JOYSTICK)
endif()

target_link_libraries(${PROJECT_NAME} 
    PRIVATE librobot::librobot
    PRIVATE Qt6::Core Qt6::Widgets Qt6::Gui Qt6::Network
    PRIVATE ${OpenCV_LIBS}
)

if(NOT DISABLE_JOYSTICK)
    target_link_libraries(${PROJECT_NAME} PRIVATE QJoysticks::QJoysticksLib)
endif()

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32 Winmm)
endif()
