set(gps_srcs "src/gps.c")

idf_component_register(SRCS "${gps_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES main nvs_flash app_update
                      )

