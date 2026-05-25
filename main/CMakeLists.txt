set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

if(NOT DEFINED VESC_ENABLE_WIFI AND DEFINED ENV{VESC_ENABLE_WIFI})
    set(VESC_ENABLE_WIFI $ENV{VESC_ENABLE_WIFI})
endif()

if(NOT DEFINED VESC_ENABLE_BLE AND DEFINED ENV{VESC_ENABLE_BLE})
    set(VESC_ENABLE_BLE $ENV{VESC_ENABLE_BLE})
endif()

if(NOT DEFINED VESC_ENABLE_GNSS AND DEFINED ENV{VESC_ENABLE_GNSS})
    set(VESC_ENABLE_GNSS $ENV{VESC_ENABLE_GNSS})
endif()

if(NOT DEFINED VESC_ENABLE_DISPLAY AND DEFINED ENV{VESC_ENABLE_DISPLAY})
    set(VESC_ENABLE_DISPLAY $ENV{VESC_ENABLE_DISPLAY})
endif()

if(NOT DEFINED VESC_ENABLE_TOUCH AND DEFINED ENV{VESC_ENABLE_TOUCH})
    set(VESC_ENABLE_TOUCH $ENV{VESC_ENABLE_TOUCH})
endif()

if(NOT DEFINED VESC_ENABLE_STORAGE AND DEFINED ENV{VESC_ENABLE_STORAGE})
    set(VESC_ENABLE_STORAGE $ENV{VESC_ENABLE_STORAGE})
endif()

if(NOT DEFINED VESC_ENABLE_WIFI)
    set(VESC_ENABLE_WIFI 1)
endif()

if(NOT DEFINED VESC_ENABLE_BLE)
    set(VESC_ENABLE_BLE 1)
endif()

if(NOT DEFINED VESC_ENABLE_GNSS)
    set(VESC_ENABLE_GNSS 1)
endif()

if(NOT DEFINED VESC_ENABLE_DISPLAY)
    set(VESC_ENABLE_DISPLAY 1)
endif()

if(NOT DEFINED VESC_ENABLE_TOUCH)
    set(VESC_ENABLE_TOUCH 1)
endif()

if(NOT DEFINED VESC_ENABLE_STORAGE)
    set(VESC_ENABLE_STORAGE 1)
endif()

set(COMPONENT_SRCS
"main.c"
"buffer.c"
"comm_uart.c"
"comm_usb.c"
"comm_can.c"
"packet.c"
"crc.c"
"commands.c"
"terminal.c"
"mempools.c"
"adc.c"
"nmea.c"
"utils.c"
"flash_helper.c"
"rb.c"
"bms.c"
"log_comm.c"
"digital_filter.c"
"i2c_compat.c"

"config/confsrc.c"
"hwconf/hw.c"

"lispif.c"
"lispif_events.c"
"lbm_vesc_utils.c"
"lispif_vesc_extensions.c"
"lbm_color_extensions.c"
"lispBM/src/env.c"
"lispBM/src/fundamental.c"
"lispBM/src/heap.c"
"lispBM/src/lbm_memory.c"
"lispBM/src/print.c"
"lispBM/src/stack.c"
"lispBM/src/symrepr.c"
"lispBM/src/tokpar.c"
"lispBM/src/extensions.c"
"lispBM/src/lispbm.c"
"lispBM/src/eval_cps.c"
"lispBM/platform/freertos/src/platform_mutex.c"
"lispBM/platform/freertos/src/platform_timestamp.c"
"lispBM/src/lbm_channel.c"
"lispBM/src/lbm_c_interop.c"
"lispBM/src/lbm_custom_type.c"
"lispBM/src/lbm_flat_value.c"
"lispBM/src/lbm_prof.c"
"lispBM/src/lbm_defrag_mem.c"
"lispBM/src/lbm_image.c"
"lispBM/src/extensions/array_extensions.c"
"lispBM/src/extensions/math_extensions.c"
"lispBM/src/extensions/string_extensions.c"
"lispBM/src/extensions/display_extensions.c"
"lispBM/src/extensions/tjpgd.c"
"lispBM/src/extensions/mutex_extensions.c"
"lispBM/src/extensions/lbm_dyn_lib.c"
"lispBM/src/extensions/ttf_extensions.c"
"lispBM/src/extensions/schrift.c"
"lispBM/src/extensions/ttf_backend.c"

"rgbled/lispif_rgbled_extensions.c"

"drivers/bme280/bme280.c"
"drivers/bme280/bme280_if.c"
"drivers/hwspi.c"
"drivers/spi_bb.c"
"drivers/enc_as504x.c"
"drivers/imu/imu.c"
"drivers/imu/lsm6ds3.c"
"drivers/imu/ahrs.c"
)

if(VESC_ENABLE_GNSS)
    list(APPEND COMPONENT_SRCS
        "ublox.c"
    )
else()
    list(APPEND COMPONENT_SRCS
        "ublox_stub.c"
    )
endif()

if(VESC_ENABLE_TOUCH)
    list(APPEND COMPONENT_SRCS
        "touch/lispif_touch_extensions.c"
        "touch/touch_cst836u.c"
    )
endif()

if(VESC_ENABLE_DISPLAY)
    list(APPEND COMPONENT_SRCS
        "display/lispif_disp_extensions.c"
        "display/disp_sh8501b.c"
        "display/disp_ili9341.c"
        "display/disp_ssd1306.c"
        "display/disp_st7789.c"
        "display/disp_st7789a.c"
        "display/disp_ili9488.c"
        "display/disp_st7735.c"
        "display/disp_ssd1351.c"
        "display/disp_icna3306.c"
        "display/disp_sh8601.c"
        "display/disp_axs15231.c"
        "display/disp_gc9a01.c"
        "display/disp_jd9853.c"
        "display/disp_st7701.c"
    )
endif()

if(VESC_ENABLE_WIFI AND NOT CONFIG_IDF_TARGET_ESP32P4)
    list(APPEND COMPONENT_SRCS
        "comm_wifi.c"
        "wifi/lispif_wifi_extensions.c"
    )
else()
    list(APPEND COMPONENT_SRCS
        "comm_wifi_stub.c"
    )
endif()

if(VESC_ENABLE_BLE AND NOT CONFIG_IDF_TARGET_ESP32P4)
    list(APPEND COMPONENT_SRCS
        "ble/lispif_ble_extensions.c"
    )

    if(CONFIG_BT_NIMBLE_ENABLED)
        list(APPEND COMPONENT_SRCS
            "comm_ble_nimble.c"
            "ble/custom_ble_nimble.c"
        )
    else()
        list(APPEND COMPONENT_SRCS
            "comm_ble.c"
            "ble/custom_ble.c"
        )
    endif()
else()
    list(APPEND COMPONENT_SRCS
        "comm_ble_stub.c"
        "ble/custom_ble_stub.c"
    )
endif()

if(VESC_ENABLE_STORAGE)
    list(APPEND COMPONENT_SRCS
        "log.c"
        "lowzip/lowzip.c"
    )
else()
    list(APPEND COMPONENT_SRCS
        "log_stub.c"
    )
endif()

set(COMPONENT_ADD_INCLUDEDIRS
"."
"config"
"hwconf"
"hwconf/trampa"
"hwconf/trampa/bms_rb"
"hwconf/vesc"
"hwconf/vesc/vbms32"
"hwconf/vesc/vdisp_dual"
"hwconf/vesc/scope"
"hwconf/vesc/duet"
"hwconf/vesc/link"
"hwconf/vesc/vdisp_900"
"hwconf/vesc/nanolog"
"hwconf/vesc/vbms16"
"hwconf/vesc/rmcore"
"hwconf/vesc/dash16"
"hwconf/vesc/wcore"
"hwconf/rescue"
"hwconf/avaspark"
"hwconf/jetfleet"
"hwconf/jetfleet/jfbms32"
"hwconf/jetfleet/jf_link"
"hwconf/jetfleet/jfbms_slave"

"lispBM"
"lispBM/include"
"lispBM/platform/freertos/include"
"display"
"wifi"
"ble"
"rgbled"
"touch"
"drivers"
"drivers/bme280"
"drivers/imu"
"lowzip"
)

set(COMPONENT_REQUIRES
    app_update
    console
    driver
    efuse
    esp_adc
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_ledc
    esp_driver_rmt
    esp_driver_spi
    esp_driver_twai
    esp_driver_uart
    esp_driver_usb_serial_jtag
    esp_event
    esp_hal_ana_conv
    esp_hal_usb
    esp_hw_support
    esp_partition
    esp_rom
    esp_system
    esp_timer
    freertos
    hal
    heap
    log
    mbedtls
    nvs_flash
    soc
    spi_flash
    vfs
)

if(VESC_ENABLE_WIFI)
    list(APPEND COMPONENT_REQUIRES
        esp_netif
        esp_wifi
        lwip
    )
endif()

if(VESC_ENABLE_BLE)
    list(APPEND COMPONENT_REQUIRES
        bt
    )
endif()

if(VESC_ENABLE_STORAGE)
    list(APPEND COMPONENT_REQUIRES
        fatfs
        sdmmc
        wear_levelling
    )
endif()

idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
    REQUIRES ${COMPONENT_REQUIRES}
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    VESC_ENABLE_WIFI=${VESC_ENABLE_WIFI}
    VESC_ENABLE_BLE=${VESC_ENABLE_BLE}
    VESC_ENABLE_GNSS=${VESC_ENABLE_GNSS}
    VESC_ENABLE_DISPLAY=${VESC_ENABLE_DISPLAY}
    VESC_ENABLE_TOUCH=${VESC_ENABLE_TOUCH}
    VESC_ENABLE_STORAGE=${VESC_ENABLE_STORAGE}
)

if(VESC_ENABLE_DISPLAY)
    # esp_lcd_axs15231b 1.0.1 still uses the IDF 5 panel config field name.
    # Scope the compatibility alias to that component only.
    idf_component_get_property(AXS15231B_LIB espressif__esp_lcd_axs15231b COMPONENT_LIB)
    target_compile_definitions(${AXS15231B_LIB} PRIVATE color_space=rgb_ele_order)
endif()

add_compile_definitions(
    LBM_USE_DYN_FUNS
    LBM_USE_DYN_MACROS
    LBM_USE_DYN_LOOPS
    LBM_USE_DYN_ARRAYS
    LBM_USE_TIME_QUOTA
    LBM_USE_ERROR_LINENO
    LBM_USE_MACRO_REST_ARGS
)

if((DEFINED ENV{HW_SRC}) OR (DEFINED ENV{HW_HEADER}))
    if(NOT DEFINED ENV{HW_SRC})
        message(FATAL_ERROR "HW_SRC not defined while HW_HEADER is set. You must either set both or none.")
    endif()
    if(NOT DEFINED ENV{HW_HEADER})
        message(FATAL_ERROR "HW_HEADER not defined while HW_SRC is set. You must either set both or none.")
    endif()
    
    message(STATUS "Setting HW_SOURCE=\"$ENV{HW_SRC}\" and HW_HEADER=\"$ENV{HW_HEADER}\" definitions")
    
    add_compile_definitions(
        HW_SOURCE=\"$ENV{HW_SRC}\"
        HW_HEADER=\"$ENV{HW_HEADER}\"
    )
endif()
