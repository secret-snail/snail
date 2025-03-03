set(emp_client
  ${CMAKE_CURRENT_SOURCE_DIR}/emp_client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/emp_client.h
)

# cannot compile emp on pi so just pull out required files by hand
set(emp_src
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/utils/prg.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/utils/block.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/io/io_channel.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/emp-tool/io/net_io_channel.h
)

# to exclude a file from the above list
#list(FILTER emp_float_server_src EXCLUDE REGEX ".*server-gn\\.cpp$")

include_directories(PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../extern/emp-tool/")

set(THREADS_PREFER_PTHREAD_FLAG ON)

add_executable(charuco_snail ${CMAKE_CURRENT_SOURCE_DIR}/charuco_snail.cpp ${emp_client} ${emp_src})
target_link_libraries(charuco_snail ${OpenCV_LIBS} ${VISP_LIBRARIES})

add_executable(visp_snail ${CMAKE_CURRENT_SOURCE_DIR}/visp_snail.cpp ${emp_client} ${emp_src})
target_link_libraries(visp_snail ${OpenCV_LIBS} ${VISP_LIBRARIES} pigpio)

add_executable(power_test ${CMAKE_CURRENT_SOURCE_DIR}/power_test.cpp ${emp_client} ${emp_src})
target_link_libraries(power_test ${OpenCV_LIBS})
