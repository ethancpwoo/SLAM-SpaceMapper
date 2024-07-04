add_library(slamspacemapper SHARED
    backend.cpp
    frontend.cpp
    loopclose.cpp
    map.cpp)
target_link_libraries(slamspacemapper ${THIRD_PARTY_LIBS})