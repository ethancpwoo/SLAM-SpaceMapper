add_library(slamspacemapper SHARED
    frontend.cpp
    map.cpp)
target_link_libraries(slamspacemapper ${THIRD_PARTY_LIBS})