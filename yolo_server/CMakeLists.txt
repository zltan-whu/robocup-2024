cmake_minimum_required(VERSION 3.0.2)
project(yolo_server)

# Find necessary packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs 
  cv_bridge 
  image_transport
)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Add service files
add_service_files(
  FILES
  yolo.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

# Include directories
include_directories(
  include
  include/yolo_server
  /usr/local/cuda/include
  /usr/include/aarch64-linux-gnu
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Link directories
link_directories(
  /usr/local/cuda/lib64
  /usr/lib/aarch64-linux-gnu
)

# Declare the CUDA libraries as a shared library
cuda_add_library(myplugins SHARED
  ./src/yololayer.cu
  ./src/basic_transform.cu
)
target_link_libraries(myplugins nvinfer cudart)

# Declare C++ executables
add_executable(yoloclient src/client.cpp)
cuda_add_executable(yoloserver src/start_server_node.cpp src/YoloServer.cpp src/preprocess.cu src/common.cpp)
# cuda_add_executable(yoloserver src/yoloserver.cpp src/preprocess.cu src/common.cpp)

# Add dependencies
add_dependencies(yoloclient ${PROJECT_NAME}_gencpp)
add_dependencies(yoloserver ${PROJECT_NAME}_gencpp)

# Specify libraries to link a library or executable target against
target_link_libraries(yoloclient
  ${catkin_LIBRARIES}
)
target_link_libraries(yoloserver
  nvinfer
  cudart
  myplugins
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

