# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(n64
	main.c
)

target_compile_definitions(n64 PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}

	DVI_16BPP_RED_MSB=4
	DVI_16BPP_RED_LSB=0
	DVI_16BPP_GREEN_MSB=9
	DVI_16BPP_GREEN_LSB=5
	DVI_16BPP_BLUE_MSB=14
	DVI_16BPP_BLUE_LSB=10

	# DVI_VERTICAL_REPEAT=1
	# DVI_SYMBOLS_PER_WORD=1
	# DVI_N_TMDS_BUFFERS=5

	)

target_link_libraries(n64
	pico_stdlib
	pico_multicore
	pico_util
	libdvi
	libsprite
	hardware_pio
)

# Build pio
pico_generate_pio_header(n64 ${CMAKE_CURRENT_LIST_DIR}/n64.pio)


# create map/bin/hex file etc.
pico_add_extra_outputs(n64)
