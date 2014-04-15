
message("\n*****\n*****\nIn ${CMAKE_CURRENT_LIST_FILE}\n*****\n*****")

find_package(Boost REQUIRED COMPONENTS filesystem)
add_library(cpp_introspection STATIC introspection.cpp types.cpp dlfcn.cpp)
target_link_libraries(cpp_introspection ${Boost_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS cpp_introspection DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

option(BUILD_TEST "If true, some introspection libraries for std_msgs and a small test executable is built." OFF)
if(BUILD_TEST)
  introspection_add(std_msgs)
  add_executable(introspection_test test.cpp)
  target_link_libraries(introspection_test cpp_introspection)
endif(BUILD_TEST)
