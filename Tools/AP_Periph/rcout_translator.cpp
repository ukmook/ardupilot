/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>
#ifdef HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR
#include <AP_Math/AP_Math.h>
#include "AP_Periph.h"

// magic value from UAVCAN driver packet
// dsdl/uavcan/equipment/esc/1030.RawCommand.uavcan
// Raw ESC command normalized into [-8192, 8191]
#define UAVCAN_ESC_MAX_VALUE    8191


#if HAL_PWM_COUNT == 0
    #error "You must define a PWM output in your hwdef.dat"
#endif

extern const AP_HAL::HAL &hal;

void AP_Periph_FW::translate_rcout_init()
{
    for (uint8_t i=0; i<HAL_PWM_COUNT; i++) {
        servo_channels.set_default_function(i, SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i));
    }

    for (uint8_t i=0; i<16; i++) {
        // SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16
        SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i), 100);
    }
    for (uint8_t i=0; i<12; i++) {
        // SRV_Channel::k_motor1 ... SRV_Channel::k_motor8, SRV_Channel::k_motor9 ... SRV_Channel::k_motor12
        SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), UAVCAN_ESC_MAX_VALUE);
    }

    has_new_data_to_update = true;
}

void AP_Periph_FW::translate_rcout_esc(int16_t *rc, uint8_t num_channels)
{
    if (rc == nullptr) {
        return;
    }

    num_channels = MIN(num_channels, 12);

    for (uint16_t i=0; i<num_channels; i++) {
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), rc[i]);
    }

    has_new_data_to_update = true;
}

void AP_Periph_FW::translate_rcout_srv(uint8_t actuator_id, const float command_value)
{
    if ((actuator_id == 0) || (actuator_id > HAL_PWM_COUNT)) {
        // not supported or out of range
        return;
    }

    const SRV_Channel::Aux_servo_function_t function = SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_norm(function, command_value);
    has_new_data_to_update = true;
}

void AP_Periph_FW::translate_rcout_handle_safety_state(uint8_t safety_state)
{
    if (safety_state == 255) {
        hal.rcout->force_safety_off();
    } else {
        hal.rcout->force_safety_on();
    }
    has_new_data_to_update = true;
}

void AP_Periph_FW::translate_rcout_update()
{
    if (!has_new_data_to_update) {
        return;
    }
    has_new_data_to_update = false;

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}

#endif // HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR

