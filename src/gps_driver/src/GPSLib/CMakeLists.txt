
aux_source_directory(. GPS_SRC)

add_library(GPS_Decoder ${GPS_SRC})

install(TARGETS GPS_Decoder
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
