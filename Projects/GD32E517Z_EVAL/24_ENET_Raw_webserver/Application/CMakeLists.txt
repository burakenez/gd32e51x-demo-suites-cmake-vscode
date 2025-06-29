project(Application LANGUAGES C CXX ASM)

add_executable(Application)

set(TARGET_SRC
	# Core
    Core/Src/fs.c
    # Core/Src/fsdata.c
    Core/Src/gd32e51x_enet_eval.c
    Core/Src/gd32e51x_it.c
    Core/Src/http_adc_led.c
    Core/Src/httpd.c
    Core/Src/main.c
    Core/Src/netconf.c
    Core/Src/system_gd32e51x.c
	
    # lwip/port/GD32E51x/Basic
    lwip/port/GD32E51x/Basic/ethernetif.c

    # Startup
    Startup/startup_gd32e517.s

    # User
    User/syscalls.c
    )

target_sources(Application PRIVATE ${TARGET_SRC})

set(TARGET_INC_DIR
	${CMAKE_SOURCE_DIR}/Application/Core/Inc
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32E51x
    ${CMAKE_SOURCE_DIR}/Application/lwip/port/GD32E51x/Basic
    )

target_include_directories(Application PRIVATE ${TARGET_INC_DIR})

target_link_options(Application PRIVATE
	-T${CMAKE_SOURCE_DIR}/gd32e51x_flash.ld -Xlinker
    -L${CMAKE_SOURCE_DIR}
	)

target_link_options(Application PRIVATE
	-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.map
	)

target_link_libraries(Application PRIVATE CMSIS)
target_link_libraries(Application PRIVATE GD32E517Z_EVAL)
target_link_libraries(Application PRIVATE GD32E51x_standard_peripheral)
target_link_libraries(Application PRIVATE lwip)

add_custom_command(TARGET Application
    POST_BUILD
    COMMAND echo -- Running Post Build Commands
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:Application> ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:Application> ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:Application>
    COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:Application> > ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.list
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:Application> > ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.bsz
    COMMAND ${CMAKE_SIZE} --format=sysv -x $<TARGET_FILE:Application> > ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_NAME:Application>.ssz
    )
