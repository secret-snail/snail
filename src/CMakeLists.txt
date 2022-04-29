file(GLOB snail_src CONFIGURE_DEPENDS "*.h" "*.cpp")
# cannot compile emp on pi so just pull out required files by hand
set(snail_src
${snail_src}
${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/utils/prg.h
${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/utils/block.h
${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/io/io_channel.h
${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/io/net_io_channel.h
)
# to exclude a file from the above list
#list(FILTER emp_float_server_src EXCLUDE REGEX ".*server-gn\\.cpp$")

add_executable(snail ${snail_src})
target_include_directories(snail PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/")
target_link_libraries(snail ${OpenCV_LIBS} ${VISP_LIBRARIES})

add_executable(secure_snail ${snail_src})
target_include_directories(secure_snail PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/")
target_compile_options(secure_snail PRIVATE "-DSECURE")
target_link_libraries(secure_snail ${OpenCV_LIBS} ${VISP_LIBRARIES})
