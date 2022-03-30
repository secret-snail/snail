file(GLOB snail_src CONFIGURE_DEPENDS "*.h" "*.cpp")
# to exclude a file from the above list
#list(FILTER emp_float_server_src EXCLUDE REGEX ".*server-gn\\.cpp$")

add_executable(snail ${snail_src})
target_include_directories(snail PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(snail ${OpenCV_LIBS})
