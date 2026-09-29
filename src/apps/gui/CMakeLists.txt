cmake_minimum_required(VERSION 3.16)

project(Supertooth LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickLayouts Graphs)

qt_standard_project_setup(REQUIRES 6.8)

set(MAC_ICNS "${CMAKE_CURRENT_SOURCE_DIR}/assets/icons/Supertooth.icns")
set(LINUX_PNG "${CMAKE_CURRENT_SOURCE_DIR}/assets/icons/Supertooth_Squircle_1024x1024.png")

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
    models/radio_device_model.h
    models/radio_device_model.cpp
    ${MAC_ICNS}
)

file(GLOB IMAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} assets/images/*.svg assets/images/*.png)
file(GLOB FONT_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} assets/fonts/*.ttf assets/fonts/*.otf)
set(LINUX_PNG_REL "assets/icons/Supertooth_Squircle_1024x1024.png")

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
        qml/views/FrameListView.qml
        qml/views/DeviceListView.qml
        qml/views/CaptureView.qml
        qml/views/ChannelSpectrum.qml
)

if(APPLE)
    # CMake derives both the .app folder name and the executable name from
    # OUTPUT_NAME, so it cannot natively produce Supertooth.app containing
    # an executable named supertooth-desktop. Build Supertooth.app and rename
    # only the executable inside via the POST_BUILD step below (Info.plist is
    # patched to match). The original executable is kept in place so
    # incremental builds stay up to date.
    # The GUI executable keeps the supertooth-desktop name to avoid colliding
    # with the `supertooth` CLI that packaging/bundle-macos.sh stages into
    # Contents/MacOS alongside it.
    set(GUI_OUTPUT_NAME "Supertooth")
else()
    set(GUI_OUTPUT_NAME "supertooth-desktop")
endif()

set_target_properties(appSupertooth PROPERTIES
    OUTPUT_NAME ${GUI_OUTPUT_NAME}
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_NAME "Supertooth"
    MACOSX_BUNDLE_BUNDLE_VERSION "${SUPERTOOTH_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${SUPERTOOTH_VERSION}"
    MACOSX_BUNDLE_ICON_FILE "Supertooth.icns"
    WIN32_EXECUTABLE TRUE
)

if(APPLE)
    add_custom_command(TARGET appSupertooth POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:appSupertooth>"
            "$<TARGET_BUNDLE_CONTENT_DIR:appSupertooth>/MacOS/supertooth-desktop"
        COMMAND python3 -c
            "import os,plistlib,sys; p=sys.argv[1]; os.chmod(p,0o644); d=plistlib.load(open(p,'rb')); d['CFBundleExecutable']='supertooth-desktop'; plistlib.dump(d,open(p,'wb'))"
            "$<TARGET_BUNDLE_CONTENT_DIR:appSupertooth>/Info.plist"
        COMMENT "Naming bundle executable supertooth-desktop inside Supertooth.app"
        VERBATIM)
endif()

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
    install(FILES "${CMAKE_SOURCE_DIR}/packaging/supertooth.desktop"
        DESTINATION share/applications
    )
endif()

include(GNUInstallDirs)
install(TARGETS appSupertooth
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
