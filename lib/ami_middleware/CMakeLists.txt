set(AMI_PORT_ROOT ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "AMI_PORT_ROOT")

zephyr_interface_library_named(ami_middleware)
zephyr_library()

set(AMI_MIDDLEWARE_ROOT ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "AMI_MIDDLEWARE_ROOT")
zephyr_library_sources(
    ${AMI_MIDDLEWARE_ROOT}/flash/flash_aspeed.c
    ${AMI_MIDDLEWARE_ROOT}/gpio/gpio_aspeed.c
    ${AMI_MIDDLEWARE_ROOT}/crypto/hash_aspeed.c
    ${AMI_MIDDLEWARE_ROOT}/crypto/rsa_aspeed.c
)
zephyr_library_include_directories(	
    .
    $ENV{ZEPHYR_BASE}/build/zephyr/include/generated/
)

zephyr_library_link_libraries(ami_middleware)
