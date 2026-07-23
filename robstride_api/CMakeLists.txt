# Khai báo các file mã nguồn
set(ROBSTRIDE_SRC
    src/can_bus.cpp
    src/can_device.cpp
    src/robstride.cpp
    src/xyber_controller.cpp
    src/common_utils.cpp       # Bổ sung dòng này
    ../protocol/socket_can.cpp
)

# Tạo thư viện tĩnh
add_library(robstride_api ${ROBSTRIDE_SRC})

# Cung cấp đường dẫn include cho thư viện này và các mục tiêu liên kết với nó
target_include_directories(robstride_api PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol
)

# Link các thư viện bên ngoài
target_link_libraries(robstride_api PUBLIC 
    Threads::Threads 
    Boost::boost 
)

pybind11_add_module(xyber_py
  src/pybind_module.cpp
)

target_include_directories(xyber_py PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol
)

target_link_libraries(xyber_py PUBLIC robstride_api)

set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(TARGETS xyber_py
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
  RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
)