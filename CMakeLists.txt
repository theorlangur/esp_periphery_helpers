idf_component_register(SRCS 
                    include/ph_board_led.hpp
                    include/ph_uart.hpp 
                    include/ph_uart_primitives.hpp 
                    include/ph_i2c.hpp 
                    include/ph_adc.hpp 
                    src/board_led.cpp
                    src/uart.cpp 
                    src/i2c.cpp 
                    src/adc.cpp 
                    INCLUDE_DIRS "include"
                    REQUIRES esp_generic_lib esp_driver_uart esp_driver_i2c esp_adc
)

#for being able to compile with clang
target_compile_options(${COMPONENT_LIB} PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)

