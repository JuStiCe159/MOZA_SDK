cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

 set(CMAKE_AUTOMOC ON)
 find_package(QT
         NAMES
         Qt6 Qt5
         REQUIRED COMPONENTS
         Test)
 find_package(Qt${QT_VERSION_MAJOR}
         REQUIRED COMPONENTS
         Test
         Widgets)

#加入openSSL库
find_package(OpenSSL REQUIRED)
#加入RS21_sdk库，根据sdk的路径添加
set(RS21_sdk "C:/Program Files (x86)/MOZA_SDK/1.0.0.6/MSVC2019-32")

set(CMAKE_CXX_STANDARD 20)
add_executable(APITest sdk_api_test.cc)

#依赖openSSL和RS21_sdk库
target_link_libraries(APITest
        PUBLIC
        Qt${QT_VERSION_MAJOR}::Test
        Qt::Widgets
        OpenSSL::SSL
        ${RS21_sdk}/lib/MOZA_SDK.lib
    )

#添加RS21_sdk头文件路径
target_include_directories(APITest
    PUBLIC
    Qt${QT_VERSION_MAJOR}::Test
    ${RS21_sdk}/include
)

add_test(
        NAME sdk_api_test
        COMMAND $<TARGET_FILE:sdk_api_test>
)
