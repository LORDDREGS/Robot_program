cmake_minimum_required(VERSION 3.30)
project(Robot_program)

set(CMAKE_CXX_STANDARD 17)

# Создаем исполняемый файл
add_executable(Robot_program
    src/main.cpp
    src/KalmanFilter.cpp
    src/Dijkstra.cpp
    src/StateModel.cpp
    src/MeasurementModel.cpp
    src/Graph.cpp
)

# Подключение библиотеки Eigen
find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)



# Настройка include directories и линковка библиотек для таргета Robot_program
target_include_directories(Robot_program PRIVATE
    ${CMAKE_SOURCE_DIR}/include  
)

target_link_libraries(Robot_program
    Eigen3::Eigen
    matplotlib_cpp::matplotlib_cpp
)
