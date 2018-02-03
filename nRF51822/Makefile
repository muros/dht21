# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS  = -DNRF_LOG_USES_UART=1
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DBOARD_CUSTOM
ASFLAGS += -g 
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBOARD_CUSTOM
#LDLIBS += -lc -lgcc -lrdimon

LDSCRIPT = ./s130_nrf51822.ld
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers
INC_PATHS += -I$(SDK_PATH)/components/ble/device_manager
INC_PATHS += -I$(SDK_PATH)/components/ble/device_manager/config
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_advertising/config
INC_PATHS += -I$(SDK_PATH)/components/libraries/fstorage
INC_PATHS += -I$(SDK_PATH)/components/libraries/fstorage/config

vpath %.c $(SDK_PATH)/components/softdevice/s130/headers

# object files

OBJS =  app_button.o app_error.o app_error_weak.o app_timer.o app_trace.o app_util_platform.o 
OBJS += fstorage.o nrf_assert.o nrf_log.o retarget.o sensorsim.o
OBJS += nrf_delay.o nrf_drv_common.o nrf_drv_gpiote.o nrf_drv_clock.o
OBJS += ble_advdata.o ble_advertising.o ble_conn_params.o ble_srv_common.o device_manager_peripheral.o 
OBJS += pstorage.o dht.o dht_service.o timers.o main.o
OBJS += system_nrf51.o softdevice_handler.o
# include common make file

include ./Makefile.common
