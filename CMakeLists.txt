cmake_minimum_required(VERSION 3.10)
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

# Подключение библиотеки Matplot++
set(MATPLOT_DIR "C:/Users/User/Desktop/Robot_program/matplotplusplus")  # Путь к Matplot++
add_library(Matplot++ INTERFACE)  # Создаем интерфейсный таргет для Matplot++
target_include_directories(Matplot++ INTERFACE ${MATPLOT_DIR}/include)

# Настройка include directories и линковка библиотек для таргета Robot_program
target_include_directories(Robot_program PRIVATE
    ${CMAKE_SOURCE_DIR}/include  # Директория с вашими заголовками
)

target_link_libraries(Robot_program
    Eigen3::Eigen  # Линковка с Eigen
    Matplot++  # Линковка с Matplot++
)
