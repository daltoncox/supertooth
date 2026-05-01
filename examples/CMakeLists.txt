# Find required libraries
find_package(PkgConfig)

# On macOS, add common Homebrew paths
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
    set(CMAKE_FIND_FRAMEWORK NEVER)
endif()

# Try to find libraries using pkg-config first
if(PkgConfig_FOUND)
    pkg_check_modules(HACKRF libhackrf)
    pkg_check_modules(LIQUID liquid-dsp)
    pkg_check_modules(BTBB libbtbb)
endif()

# Set up proper library paths for pkg-config found libraries
if(HACKRF_FOUND)
    # Convert library names to full paths
    find_library(HACKRF_LIBRARY_PATH NAMES hackrf libhackrf
        PATHS ${HACKRF_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(HACKRF_LIBRARY_PATH)
        set(HACKRF_LIBRARIES ${HACKRF_LIBRARY_PATH})
    endif()
endif()

if(BTBB_FOUND)
    # Convert library names to full paths
    find_library(BTBB_LIBRARY_PATH NAMES btbb libbtbb
        PATHS ${BTBB_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(BTBB_LIBRARY_PATH)
        set(BTBB_LIBRARIES ${BTBB_LIBRARY_PATH})
    endif()
endif()

# Fallback to find_package if pkg-config didn't find them
if(NOT LIQUID_FOUND)
    find_library(LIQUID_LIBRARIES NAMES liquid 
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(LIQUID_INCLUDE_DIRS liquid/liquid.h 
        PATHS /opt/homebrew/include /usr/local/include)
    
    if(LIQUID_LIBRARIES AND LIQUID_INCLUDE_DIRS)
        set(LIQUID_FOUND TRUE)
        message(STATUS "Found Liquid DSP manually: ${LIQUID_LIBRARIES}")
    endif()
endif()

# Fallback for HackRF if pkg-config didn't find it
if(NOT HACKRF_FOUND)
    find_library(HACKRF_LIBRARIES NAMES hackrf libhackrf
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(HACKRF_INCLUDE_DIRS libhackrf/hackrf.h 
        PATHS /opt/homebrew/include /usr/local/include)
    
    if(HACKRF_LIBRARIES AND HACKRF_INCLUDE_DIRS)
        set(HACKRF_FOUND TRUE)
        message(STATUS "Found HackRF manually: ${HACKRF_LIBRARIES}")
    endif()
endif()

# Fallback for BTBB if pkg-config didn't find it - search more locations
if(NOT BTBB_FOUND)
    find_library(BTBB_LIBRARIES NAMES btbb libbtbb
        PATHS 
            /opt/homebrew/lib 
            /usr/local/lib
            /usr/lib
            /opt/local/lib)
    find_path(BTBB_INCLUDE_DIRS btbb.h 
        PATHS 
            /opt/homebrew/include
            /usr/local/include
            /usr/include
            /opt/local/include)
    
    if(BTBB_LIBRARIES AND BTBB_INCLUDE_DIRS)
        set(BTBB_FOUND TRUE)
        message(STATUS "Found BTBB manually: ${BTBB_LIBRARIES}")
    else()
        # Try to find it in a libbtbb subdirectory
        find_path(BTBB_INCLUDE_DIRS btbb.h 
            PATHS 
                /opt/homebrew/include/libbtbb
                /usr/local/include/libbtbb
                /usr/include/libbtbb
                /opt/local/include/libbtbb)
        
        if(BTBB_INCLUDE_DIRS)
            set(BTBB_FOUND TRUE)
            message(STATUS "Found BTBB headers in subdirectory: ${BTBB_INCLUDE_DIRS}")
        endif()
    endif()
endif()

# Check if all libraries were found (warn instead of fatal error for examples)
if(NOT HACKRF_FOUND)
    message(WARNING "HackRF library not found. iq-recorder example will be skipped.")
endif()

if(NOT LIQUID_FOUND)
    message(WARNING "Liquid DSP library not found. liquid-channelizer and liquid-gfsk examples will be skipped.")
endif()

if(NOT BTBB_FOUND)
    message(WARNING "BTBB library not found. ac-finder example will be skipped.")
endif()

# Create a custom target for all examples
add_custom_target(examples)

# Create executables only if required libraries are found
if(BTBB_FOUND)
    add_executable(ac-finder ac-finder.c wav.c)
    target_link_libraries(ac-finder ${BTBB_LIBRARIES})
    target_include_directories(ac-finder PRIVATE ${BTBB_INCLUDE_DIRS})
    if(BTBB_CFLAGS_OTHER)
        target_compile_options(ac-finder PRIVATE ${BTBB_CFLAGS_OTHER})
    endif()
    # Add to examples target
    add_dependencies(examples ac-finder)
endif()

if(HACKRF_FOUND)
    add_executable(iq-recorder iq-recorder.c wav.c)
    target_link_libraries(iq-recorder ${HACKRF_LIBRARIES})
    target_include_directories(iq-recorder PRIVATE ${HACKRF_INCLUDE_DIRS})
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(iq-recorder PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
    # Add to examples target
    add_dependencies(examples iq-recorder)
endif()

if(LIQUID_FOUND)
    add_executable(liquid-channelizer liquid-channelizer.c wav.c)
    target_link_libraries(liquid-channelizer ${LIQUID_LIBRARIES})
    target_include_directories(liquid-channelizer PRIVATE ${LIQUID_INCLUDE_DIRS})
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(liquid-channelizer PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    # Add to examples target
    add_dependencies(examples liquid-channelizer)
    
    add_executable(liquid-gfsk liquid-gfsk.c wav.c)
    target_link_libraries(liquid-gfsk ${LIQUID_LIBRARIES})
    target_include_directories(liquid-gfsk PRIVATE ${LIQUID_INCLUDE_DIRS})
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(liquid-gfsk PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    # Add to examples target
    add_dependencies(examples liquid-gfsk)
    
    add_executable(liquid-mix liquid-mix.c wav.c)
    target_link_libraries(liquid-mix ${LIQUID_LIBRARIES})
    target_include_directories(liquid-mix PRIVATE ${LIQUID_INCLUDE_DIRS})
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(liquid-mix PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    # Add to examples target
    add_dependencies(examples liquid-mix)
endif()

# live-channel executable (requires all libraries)
if(HACKRF_FOUND AND LIQUID_FOUND AND BTBB_FOUND)
    add_executable(live-channel live-channel.c ${CMAKE_SOURCE_DIR}/src/hackrf.c ${CMAKE_SOURCE_DIR}/src/hackrf.h)
    target_link_libraries(live-channel 
        ${HACKRF_LIBRARIES}
        ${LIQUID_LIBRARIES} 
        ${BTBB_LIBRARIES}
    )
    target_include_directories(live-channel PRIVATE
        ${HACKRF_INCLUDE_DIRS}
        ${LIQUID_INCLUDE_DIRS}
        ${BTBB_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src
    )
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(live-channel PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(live-channel PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    if(BTBB_CFLAGS_OTHER)
        target_compile_options(live-channel PRIVATE ${BTBB_CFLAGS_OTHER})
    endif()
    add_dependencies(examples live-channel)
endif()

# bredr-detector: live BR/EDR PHY detector (requires HackRF + Liquid)
if(HACKRF_FOUND AND LIQUID_FOUND)
    add_executable(bredr-detector
        bredr-detector.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.h
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.c
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.h
    )
    target_link_libraries(bredr-detector
        ${HACKRF_LIBRARIES}
        ${LIQUID_LIBRARIES}
    )
    target_include_directories(bredr-detector PRIVATE
        ${HACKRF_INCLUDE_DIRS}
        ${LIQUID_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src
    )
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(bredr-detector PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(bredr-detector PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    add_dependencies(examples bredr-detector)
endif()

# bredr-scanner: real-time BR/EDR scanner with piconet tracking + libbtbb UAP solving
# Requires HackRF + Liquid DSP + BTBB (hard dependency)
if(HACKRF_FOUND AND LIQUID_FOUND AND BTBB_FOUND)
    add_executable(bredr-scanner
        bredr-scanner.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.h
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.c
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.h
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet.c
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet.h
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet_store.c
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet_store.h
    )
    target_link_libraries(bredr-scanner
        ${HACKRF_LIBRARIES}
        ${LIQUID_LIBRARIES}
        ${BTBB_LIBRARIES}
    )
    target_include_directories(bredr-scanner PRIVATE
        ${HACKRF_INCLUDE_DIRS}
        ${LIQUID_INCLUDE_DIRS}
        ${BTBB_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src
    )
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(bredr-scanner PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(bredr-scanner PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    if(BTBB_CFLAGS_OTHER)
        target_compile_options(bredr-scanner PRIVATE ${BTBB_CFLAGS_OTHER})
    endif()
    add_dependencies(examples bredr-scanner)
endif()

# bredr2csv: 20-channel BR/EDR receiver — writes LAP, RX_TIME, HEADER, CHANNEL to CSV
# Only requires HackRF + Liquid DSP (no libbtbb — no UAP/clock solving).
if(HACKRF_FOUND AND LIQUID_FOUND)
    add_executable(bredr2csv
        bredr2csv.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.h
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.c
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.h
    )
    target_link_libraries(bredr2csv
        ${HACKRF_LIBRARIES}
        ${LIQUID_LIBRARIES}
    )
    target_include_directories(bredr2csv PRIVATE
        ${HACKRF_INCLUDE_DIRS}
        ${LIQUID_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src
    )
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(bredr2csv PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(bredr2csv PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    add_dependencies(examples bredr2csv)
endif()

# edr_intro: supertooth-rx style 20-channel BR/EDR receiver with example EDR hooks.
# Requires HackRF + Liquid DSP + BTBB (uses piconet store integration).
if(HACKRF_FOUND AND LIQUID_FOUND AND BTBB_FOUND)
    add_executable(edr_intro
        edr_intro.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.c
        ${CMAKE_SOURCE_DIR}/src/hackrf.h
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.c
        ${CMAKE_SOURCE_DIR}/src/bredr_phy.h
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet.c
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet.h
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet_store.c
        ${CMAKE_SOURCE_DIR}/src/bredr_piconet_store.h
    )
    target_link_libraries(edr_intro
        ${HACKRF_LIBRARIES}
        ${LIQUID_LIBRARIES}
        ${BTBB_LIBRARIES}
    )
    target_include_directories(edr_intro PRIVATE
        ${HACKRF_INCLUDE_DIRS}
        ${LIQUID_INCLUDE_DIRS}
        ${BTBB_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src
    )
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(edr_intro PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
    if(LIQUID_CFLAGS_OTHER)
        target_compile_options(edr_intro PRIVATE ${LIQUID_CFLAGS_OTHER})
    endif()
    if(BTBB_CFLAGS_OTHER)
        target_compile_options(edr_intro PRIVATE ${BTBB_CFLAGS_OTHER})
    endif()
    add_dependencies(examples edr_intro)
endif()

# Add math library which might be needed on some systems
if(UNIX AND NOT APPLE)
    if(TARGET ac-finder)
        target_link_libraries(ac-finder m)
    endif()
    if(TARGET iq-recorder)
        target_link_libraries(iq-recorder m)
    endif()
    if(TARGET liquid-channelizer)
        target_link_libraries(liquid-channelizer m)
    endif()
    if(TARGET liquid-gfsk)
        target_link_libraries(liquid-gfsk m)
    endif()
    if(TARGET liquid-mix)
        target_link_libraries(liquid-mix m)
    endif()
    if(TARGET live-channel)
        target_link_libraries(live-channel m)
    endif()
    if(TARGET bredr-detector)
        target_link_libraries(bredr-detector m)
    endif()
endif()
