set(wsserver_srcs "src/wsserver.c" "src/keep_alive.c")

idf_component_register(SRCS "${wsserver_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES main nvs_flash app_update
                      )

