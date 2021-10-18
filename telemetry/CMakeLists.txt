set(telemetry_srcs "src/telemetry.c")

idf_component_register(SRCS "${telemetry_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES main nvs_flash app_update
                      )
