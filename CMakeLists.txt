cmake_minimum_required(VERSION 3.20)
project(OpenGL_GLUT_Triangle)

set(CMAKE_CXX_STANDARD 17)

include_directories("./freeglut/include")
link_directories("./freeglut/lib/x64")

add_executable(OpenGL_GLUT_Triangle main.cpp)

target_link_libraries(OpenGL_GLUT_Triangle
        glu32
        opengl32
        freeglut
)
