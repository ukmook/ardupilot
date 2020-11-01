#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "../AP_Bootloader/app_comms.h"
#include "hwing_esc.h"
#ifdef HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR
#if HAL_NUM_CAN_IFACES > 1
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_KDECAN/AP_KDECAN.h>
#endif
#include <uavcan/equipment/esc/Status.h>
#endif
#if defined(HAL_PERIPH_NEOPIXEL_COUNT) || defined(HAL_PERIPH_ENABLE_NCP5623_LED)
#define AP_PERIPH_HAVE_LED
#endif

#include "Parameters.h"
#include "ch.h"

/*
  app descriptor compatible with MissionPlanner
 */
extern const struct app_descriptor app_descriptor;

class AP_Periph_FW {
public:
    void init();
    void update();

    Parameters g;

    void can_start();
    void can_update();
    void can_mag_update();
    void can_gps_update();
    void can_baro_update();
    void can_airspeed_update();
    void can_rangefinder_update();

    void load_parameters();

    AP_SerialManager serial_manager;

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    struct {
        AP_MSP msp;
        MSP::msp_port_t port;
        uint32_t last_gps_ms;
        uint32_t last_baro_ms;
        uint32_t last_mag_ms;
    } msp;
    void msp_init(AP_HAL::UARTDriver *_uart);
    void msp_sensor_update(void);
    void send_msp_packet(uint16_t cmd, void *p, uint16_t size);
    void send_msp_GPS(void);
    void send_msp_compass(void);
    void send_msp_baro(void);
#endif
    
#ifdef HAL_PERIPH_ENABLE_ADSB
    void adsb_init();
    void adsb_update();
    void can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg);
    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    void pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    void pwm_hardpoint_init();
    void pwm_hardpoint_update();
    struct {
        uint8_t last_state;
        uint32_t last_ts_us;
        uint32_t last_send_ms;
        uint16_t pwm_value;
        uint16_t highest_pwm;
    } pwm_hardpoint;
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    HWESC_Telem hwesc_telem;
    void hwesc_telem_update();
#endif

#ifdef HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR
    //Parameter Interface for CANDrivers
    class RCOUTTranslator_Params
    {
        friend class AP_Periph_FW;

    public:
        RCOUTTranslator_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        static const struct AP_Param::GroupInfo var_info[];

        enum can_translate_mode {
            RCOUT_UAVCAN,
            RCOUT_KDECAN,
        };
    private:
        AP_Int8 _chan_start;
        AP_Int8 _chan_end;
        AP_Int8 _protocol;
        AP_Int8 _can_out;
        AP_Int8 _act_type;
        AP_Int8 _enum_mode;
        AP_Int16 _pwm_min;
        AP_Int16 _pwm_max;
        AP_Int16 _frequency;
#if HAL_NUM_CAN_IFACES > 1
        AP_KDECAN* _kdecan;
#endif
    } rcout_translator;
    uint8_t _num_rcout_channels;
    uint8_t _rcout_protocol;
    bool _enum_state;
    void init_rcout_translator();
    void translate_rcout_esc(int16_t *rc, uint8_t num_channels);
    void translate_rcout_srv(uint8_t chan, float rc);
    void translate_rcout_update();
    void translate_rcout_handle_safety_state(uint8_t safety_state);
    void can_send_esc_telem(uavcan_equipment_esc_Status esc_telem);
#endif

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;
};

extern AP_Periph_FW periph;

extern "C" {
void can_printf(const char *fmt, ...);
}

