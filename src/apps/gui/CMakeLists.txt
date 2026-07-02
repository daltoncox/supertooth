cmake_minimum_required(VERSION 3.16)

project(Supertooth VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickLayouts)

qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(appSupertooth
    main.cpp
    radio_device_model.h
    radio_device_model.cpp
    backend_api.h
    backend_api.c
    receiver_controller.h
    receiver_controller.cpp
    frame_list_model.h
    frame_list_model.cpp
)

file(GLOB IMAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} assets/images/*.svg assets/images/*.png)
file(GLOB FONT_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} assets/fonts/*.ttf assets/fonts/*.otf)

qt_add_resources(appSupertooth "app_resources"
    PREFIX "/"
    FILES qtquickcontrols2.conf
    ${IMAGE_FILES}
    ${FONT_FILES}
)

qt_add_qml_module(appSupertooth
    URI Supertooth
    VERSION 1.0
    QML_FILES
        Main.qml
        Sidebar.qml
        Header.qml
        FrameListView.qml
)

set_target_properties(appSupertooth PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appSupertooth
    PRIVATE Qt6::Quick Qt6::QuickLayouts supertooth_core
)

include(GNUInstallDirs)
install(TARGETS appSupertooth
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
