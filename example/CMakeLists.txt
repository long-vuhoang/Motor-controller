# Khởi tạo file thực thi từ main.cpp
add_executable(main main.cpp)

# Link file thực thi với thư viện động cơ
target_link_libraries(main PRIVATE robstride_api)

# Đổi tên output đầu ra thành main.exe (trên Linux mặc định file không có đuôi)
set_target_properties(main PROPERTIES OUTPUT_NAME "main.exe")