cmake_minimum_required(VERSION 2.8.3)

project(s3a_main)

add_compile_options(-std=c++17)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################

catkin_package(
	LIBRARIES ${PROJECT_NAME}
)
