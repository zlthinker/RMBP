include_directories(include)
include_directories(src)

file(GLOB ANN_SOURCES "src/*.cpp")

ADD_LIBRARY(ann ${ANN_SOURCES})
