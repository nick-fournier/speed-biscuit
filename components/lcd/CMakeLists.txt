idf_component_register(SRCS "LCD_Bmp.c" "LCD_Driver.c" "LCD_GUI.c" "LCD_Touch.c"
                       INCLUDE_DIRS "."
                       REQUIRES config fatfs font)
