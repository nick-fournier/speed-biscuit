idf_component_register(SRCS "diskio.c" "fatfs_storage.c" "ff.c"
                       INCLUDE_DIRS "."
                       REQUIRES config sdcard lcd)
