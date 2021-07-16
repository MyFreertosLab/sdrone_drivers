set(ina3221_srcs "src/ina3221.c")

idf_component_register(SRCS "${ina3221_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES main nvs_flash app_update
                      )

