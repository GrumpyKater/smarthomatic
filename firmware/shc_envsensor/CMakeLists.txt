CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
SET(CMAKE_SYSTEM_NAME Generic)

#============================================================================================
PROJECT(shc_envsensor)
enable_language(C ASM)

SET(MCU "atmega328p")
SET(F_CPU "4000000")
SET(UART_BAUD_RATE "4800")

#============================================================================================

SET(CMAKE_C_COMPILER "avr-gcc")
SET(OPT "s")
SET(DEBUG "dwarf-2")
SET(FORMAT "ihex")

SET(CMAKE_C_FLAGS "-g${DEBUG} -DF_CPU=${F_CPU}UL -DUART_BAUD_RATE=${UART_BAUD_RATE}UL -O${OPT} -mmcu=${MCU} -I. -fdata-sections -ffunction-sections -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wundef -std=c99 -Wl,--gc-sections,--cref")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "firmware/shc_hr20/bin")

#============================================================================================

SET(SOURCE_FILES

        ${PROJECT_NAME}.c
        rfm12.c
        ../src_common/util.c
        ../src_common/uart.c
        ../src_common/aes256.c
        fuses.c
        ../src_common/avr-asm-macros.S
        ../src_common/aes_keyschedule-asm.S
        ../src_common/aes_enc-asm.S
        ../src_common/aes_dec-asm.S
        ../src_common/aes_sbox-asm.S
        ../src_common/aes_invsbox-asm.S

        )

set_source_files_properties(avr-asm-macros.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
set_source_files_properties(aes_keyschedule-asm.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
set_source_files_properties(aes_enc-asm.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
set_source_files_properties(aes_dec-asm.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
set_source_files_properties(aes_sbox-asm.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
set_source_files_properties(aes_invsbox-asm.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")


#============================================================================================

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCE_FILES})

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-objcopy -O ${FORMAT} -R.eeprom ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-objcopy -O ${FORMAT} -j .eeprom --set-section-flags=.eeprom="alloc,load"  --change-section-lma .eeprom=0 ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.eep)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-size ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} --mcu=${DEVICE} --format=avr)