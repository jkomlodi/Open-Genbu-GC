cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(genbu-fw)

pico_sdk_init()

add_executable(genbu-fw
    dev_lowlevel.c
    proc_queue.c
    board_io.c
    debounce.c
)

pico_enable_stdio_usb(genbu-fw 0)
pico_enable_stdio_uart(genbu-fw 1)

pico_add_extra_outputs(genbu-fw)

target_link_libraries(genbu-fw pico_stdlib hardware_pio)

target_include_directories(genbu-fw PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# add url via pico_set_program_url
#example_auto_set_url(genbu-fw)
