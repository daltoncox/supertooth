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

# Check if all libraries were found
if(NOT HACKRF_FOUND)
    message(FATAL_ERROR "HackRF library not found. Please install with: brew install hackrf")
endif()

if(NOT LIQUID_FOUND)
    message(FATAL_ERROR "Liquid DSP library not found. Please install with: brew install liquid-dsp")
endif()

if(NOT BTBB_FOUND)
    message(FATAL_ERROR "BTBB library is required for this project")
endif()

find_package(Threads REQUIRED)

add_executable(supertooth-rx
    supertooth-rx.c
    hackrf.c hackrf.h
    bredr_phy.c bredr_phy.h
    bredr_piconet.c bredr_piconet.h
    bredr_piconet_store.c bredr_piconet_store.h)

# Link libraries - use full paths for more reliable linking
target_link_libraries(supertooth-rx 
    ${HACKRF_LIBRARIES}
    ${LIQUID_LIBRARIES} 
    ${BTBB_LIBRARIES}
)

# Add include directories
target_include_directories(supertooth-rx PRIVATE
    ${HACKRF_INCLUDE_DIRS}
    ${LIQUID_INCLUDE_DIRS}
    ${BTBB_INCLUDE_DIRS}
)

# Add compiler flags if needed
if(HACKRF_CFLAGS_OTHER)
    target_compile_options(supertooth-rx PRIVATE ${HACKRF_CFLAGS_OTHER})
endif()
if(LIQUID_CFLAGS_OTHER)
    target_compile_options(supertooth-rx PRIVATE ${LIQUID_CFLAGS_OTHER})
endif()
if(BTBB_CFLAGS_OTHER)
    target_compile_options(supertooth-rx PRIVATE ${BTBB_CFLAGS_OTHER})
endif()

# Add math library which might be needed
if(UNIX AND NOT APPLE)
    target_link_libraries(supertooth-rx m)
endif()

# Add supertooth-btle executable (BTLE advertising packet detector)
add_executable(supertooth-btle supertooth-btle.c hackrf.c hackrf.h ble_phy.c ble_phy.h)

# Link libraries for supertooth-btle (doesn't need BTBB)
target_link_libraries(supertooth-btle 
    ${HACKRF_LIBRARIES}
    ${LIQUID_LIBRARIES}
)

# Add include directories
target_include_directories(supertooth-btle PRIVATE
    ${HACKRF_INCLUDE_DIRS}
    ${LIQUID_INCLUDE_DIRS}
)

# Add compiler flags if needed
if(HACKRF_CFLAGS_OTHER)
    target_compile_options(supertooth-btle PRIVATE ${HACKRF_CFLAGS_OTHER})
endif()
if(LIQUID_CFLAGS_OTHER)
    target_compile_options(supertooth-btle PRIVATE ${LIQUID_CFLAGS_OTHER})
endif()

# Add math library which might be needed for supertooth-btle
if(UNIX AND NOT APPLE)
    target_link_libraries(supertooth-btle m)
endif()

# supertooth-hybrid: BR/EDR ch0-19 + BLE ch37 simultaneous capture
add_executable(supertooth-hybrid
    supertooth-hybrid.c hackrf.c hackrf.h
    ble_phy.c ble_phy.h
    bredr_phy.c bredr_phy.h
    bredr_piconet.c bredr_piconet.h)

target_link_libraries(supertooth-hybrid
    ${HACKRF_LIBRARIES}
    ${LIQUID_LIBRARIES}
    ${BTBB_LIBRARIES}
)

target_include_directories(supertooth-hybrid PRIVATE
    ${HACKRF_INCLUDE_DIRS}
    ${LIQUID_INCLUDE_DIRS}
    ${BTBB_INCLUDE_DIRS}
)

if(HACKRF_CFLAGS_OTHER)
    target_compile_options(supertooth-hybrid PRIVATE ${HACKRF_CFLAGS_OTHER})
endif()
if(LIQUID_CFLAGS_OTHER)
    target_compile_options(supertooth-hybrid PRIVATE ${LIQUID_CFLAGS_OTHER})
endif()
if(BTBB_CFLAGS_OTHER)
    target_compile_options(supertooth-hybrid PRIVATE ${BTBB_CFLAGS_OTHER})
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(supertooth-hybrid m)
endif()
