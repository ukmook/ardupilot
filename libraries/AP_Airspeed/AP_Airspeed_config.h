#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/AP_MSP_config.h>

#ifndef AP_AIRSPEED_ENABLED
#define AP_AIRSPEED_ENABLED 1
#endif

#ifndef AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED AP_AIRSPEED_ENABLED
#endif

// backends
#ifndef AP_AIRSPEED_ANALOG_ENABLED
#define AP_AIRSPEED_ANALOG_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_ASP5033_ENABLED
#define AP_AIRSPEED_ASP5033_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_DLVR_ENABLED
#define AP_AIRSPEED_DLVR_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_DRONECAN_ENABLED
#define AP_AIRSPEED_DRONECAN_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

#ifndef AP_AIRSPEED_MS4525_ENABLED
#define AP_AIRSPEED_MS4525_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_MS5525_ENABLED
#define AP_AIRSPEED_MS5525_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_MSP_ENABLED
#define AP_AIRSPEED_MSP_ENABLED (AP_AIRSPEED_BACKEND_DEFAULT_ENABLED && HAL_MSP_SENSORS_ENABLED)
#endif

// note additional vehicle restrictions are made in the .cpp file!
#ifndef AP_AIRSPEED_NMEA_ENABLED
#define AP_AIRSPEED_NMEA_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_SDP3X_ENABLED
#define AP_AIRSPEED_SDP3X_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_AIRSPEED_SITL_ENABLED
#define AP_AIRSPEED_SITL_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED && AP_SIM_ENABLED
#endif

// other AP_Airspeed options:
#ifndef AIRSPEED_MAX_SENSORS
#define AIRSPEED_MAX_SENSORS 2
#endif

#ifndef AP_AIRSPEED_AUTOCAL_ENABLE
#define AP_AIRSPEED_AUTOCAL_ENABLE AP_AIRSPEED_ENABLED
#endif

#ifndef AP_AIRSPEED_HYGROMETER_ENABLE
#define AP_AIRSPEED_HYGROMETER_ENABLE (AP_AIRSPEED_ENABLED && BOARD_FLASH_SIZE > 1024)
#endif
