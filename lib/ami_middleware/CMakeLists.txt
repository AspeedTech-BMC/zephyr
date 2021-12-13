cmake_minimum_required(VERSION 3.13.1)
# project(ami_middleware)
zephyr_interface_library_named(ami_middleware)
zephyr_library()

# zephyr_system_include_directories(
#     .
#     cerberus/crypto/
# )

zephyr_library_sources(
    crypto/hash_aspeed.c
)
zephyr_system_include_directories(	
    .
    cerberus/crypto/
    $ENV{ZEPHYR_BASE}/build/zephyr/include/generated/
)

zephyr_library_link_libraries(ami_middleware)
