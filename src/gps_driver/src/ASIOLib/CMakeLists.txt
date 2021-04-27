
aux_source_directory(. ASIO_SRC)

add_library(ASIO ${ASIO_SRC})

install(TARGETS ASIO
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
