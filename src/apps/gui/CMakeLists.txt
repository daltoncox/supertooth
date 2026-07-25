cmake_minimum_required(VERSION 3.16)

project(Supertooth LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickLayouts Graphs)

qt_standard_project_setup(REQUIRES 6.8)

set(MAC_ICNS "${CMAKE_CURRENT_SOURCE_DIR}/assets/icons/Supertooth.icns")
set(LINUX_PNG "${CMAKE_CURRENT_SOURCE_DIR}/assets/icons/Supertooth_Spaced_512_2x.png")

if(APPLE)
    set_source_files_properties(${MAC_ICNS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
endif()

qt_add_executable(appSupertooth
    main.cpp
    backend/backend_api.h
    backend/backend_api.c
    controllers/receiver_controller.h
    controllers/receiver_controller.cpp
    models/frame_list_model.h
    models/frame_list_model.cpp
    models/device_list_model.h
    models/device_list_model.cpp
    models/device_list_sort_proxy_model.h
    models/device_list_sort_proxy_model.cpp
    models/radio_device_model.h
    models/radio_device_model.cpp
    ${MAC_ICNS}
)

file(GLOB IMAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} assets/images/*.svg assets/images/*.png)
file(GLOB FONT_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} assets/fonts/*.ttf assets/fonts/*.otf)
set(LINUX_PNG_REL "assets/icons/Supertooth_Spaced_512_2x.png")

qt_add_resources(appSupertooth "app_resources"
    PREFIX "/"
    FILES qtquickcontrols2.conf
    ${IMAGE_FILES}
    ${FONT_FILES}
    ${LINUX_PNG_REL}
)

qt_add_qml_module(appSupertooth
    URI Supertooth
    VERSION 1.0
    QML_FILES
        qml/Main.qml
        qml/Sidebar.qml
        qml/Header.qml
        qml/views/FrameListView.qml
        qml/views/DeviceListView.qml
        qml/views/CaptureView.qml
        qml/views/ChannelSpectrum.qml
)

set_target_properties(appSupertooth PROPERTIES
    OUTPUT_NAME supertooth
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_VERSION "${SUPERTOOTH_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${SUPERTOOTH_VERSION}"
    MACOSX_BUNDLE_ICON_FILE "Supertooth.icns"
    WIN32_EXECUTABLE TRUE
)

target_include_directories(appSupertooth PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers
    ${CMAKE_CURRENT_SOURCE_DIR}/backend
)

target_link_libraries(appSupertooth
    PRIVATE Qt6::Quick Qt6::QuickLayouts Qt6::Graphs supertooth_core
)

if(LINUX)
    install(FILES ${LINUX_PNG}
        DESTINATION share/icons/hicolor/512x512/apps
        RENAME "supertooth-app.png"
    )
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/assets/icons/supertooth.desktop"
        DESTINATION share/applications
    )
endif()

include(GNUInstallDirs)
install(TARGETS appSupertooth
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
