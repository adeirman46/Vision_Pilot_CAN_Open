# The route corridor, driven over a synthesised junction where the right answer
# is known. See the file header for why this cannot be judged on the road.
add_executable(test_route_corridor test_route_corridor.cpp)
target_link_libraries(test_route_corridor PRIVATE fusion)
target_compile_features(test_route_corridor PRIVATE cxx_std_17)
