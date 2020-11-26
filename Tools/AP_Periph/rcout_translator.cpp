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
#define UAVCAN_ESC_MIN_VALUE    (-8192)
#define UAVCAN_ESC_MAX_VALUE    (8191)



extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Periph_FW::RCOUTTranslator_Params::var_info[] = {

    // @Param: RCOUT_CHAN_LOW
    // @DisplayName: RCOUT first servo output channel number.
    // @Description: RCOUT first servo output channel number. Consecutive output channels are enabled from RCOUT_CHAN_LOW through RCOUT_CHAN_HIGH. Use 1 for PWM1
    // @Range: 900 2100
    AP_GROUPINFO("CHAN_LOW", 1, AP_Periph_FW::RCOUTTranslator_Params, chan_start, 0),

    // @Param: RCOUT_CHAN_HIGH
    // @DisplayName: RCOUT last servo output channel number.
    // @Description: RCOUT last servo output channel number. Consecutive output channels are enabled from RCOUT_CHAN_LOW through RCOUT_CHAN_HIGH.
    // @Range: 900 2100
    AP_GROUPINFO("CHAN_HIGH", 2, AP_Periph_FW::RCOUTTranslator_Params, chan_end, 0),

    // @Param: RCOUT_PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This enables and selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output
    // @Values: 0:Disabled,1:Normal,2:OneShot,3:OneShot125,4:Brushed,5:DShot150,6:DShot300,7:DShot600,8:DShot1200
    // @User: Advanced
    AP_GROUPINFO("PWM_TYPE", 3, AP_Periph_FW::RCOUTTranslator_Params, pwm_type, AP_HAL::RCOutput::MODE_PWM_NONE),

    // @Param: RCOUT_RATE
    // @DisplayName: Servo output rate
    // @Description: This sets the output rate in Hz for all outputs.
    // @Range: 25 400
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("RATE", 4, AP_Periph_FW::RCOUTTranslator_Params, frequency, 50),

    // @Param: RCOUT_OUT_TYPE
    // @DisplayName: Select ESC or SRV output
    // @Description: Select to route UAVCAN ESC msgs or Servo/Actuator msgs to PWM output. For ESC Fwd Only, the positive values are scaled to the whole PWM range. For Fwd and Rev then a value of 0 is the middle PWM and
    // @Values : 0:ESC Fwd only,1:Servo,2:ESC Fwd and Rev
    AP_GROUPINFO("OUT_TYPE", 5, AP_Periph_FW::RCOUTTranslator_Params, output_type, RCOUTTranslator_Params::OutputType::ESC_Fwd_Only),

    // @Param: RCOUT_PWM_MIN
    // @DisplayName: RC Out Minimum PWM
    // @Description: Minimum PWM pulse width in microseconds. Typically 1000 is lower limit and 2000 is upper limit.
    // @Units: PWM
    // @Range: 500 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PWM_MIN", 6, AP_Periph_FW::RCOUTTranslator_Params, pwm_min, 1100),

    // @Param: RCOUT_PWM_MAX
    // @DisplayName: RC Out Maximum PWM
    // @Description: Maximum PWM pulse width in microseconds. Typically 1000 is lower limit and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
   AP_GROUPINFO("PWM_MAX", 7, AP_Periph_FW::RCOUTTranslator_Params, pwm_max, 1900),

    AP_GROUPEND
};

void AP_Periph_FW::init_rcout_translator()
{
    rcout.chan_start = rcout_params.chan_start;
    rcout.chan_end = rcout_params.chan_end;
    rcout.pwm_type = rcout_params.pwm_type;
    rcout.output_type = rcout_params.output_type;
    rcout.frequency = rcout_params.frequency;


    for (uint8_t i = 0; i < hal.rcout->get_max_channels(); i++) {
        hal.rcout->disable_ch(i);
    }

    // Configure PWM translation via hal rcoutput driver
    const AP_HAL::RCOutput::output_mode mode = (AP_HAL::RCOutput::output_mode)rcout.pwm_type;
    if (mode == AP_HAL::RCOutput::MODE_PWM_NONE || rcout.chan_end < rcout.chan_start || rcout.chan_start <= 0) {
        rcout.num_channels = 0;
        return;
    }
    rcout.num_channels = rcout.chan_end - rcout.chan_start + 1;

    for (uint8_t i = rcout.chan_start; i <= rcout.chan_end; i++) {
        hal.rcout->enable_ch(i-1);
    }
    const uint16_t mask = (1UL << (rcout.num_channels)) - 1UL;
    hal.rcout->set_freq(mask, rcout.frequency);
    hal.rcout->set_output_mode(mask, mode);
}

void AP_Periph_FW::translate_rcout_esc(int16_t *rc, uint8_t num_channels)
{
    if (rc == nullptr ||
        rcout.num_channels == 0 ||
        (num_channels < rcout.chan_end))
    {
        return;
    }

    // check that rcout.output_type is configured for ESC. If not, we don't handle the packet
    int16_t min_range;
    if (rcout.output_type == RCOUTTranslator_Params::OutputType::ESC_Fwd_Only) {
        min_range = 0;
    } else if (rcout.output_type == (uint8_t)RCOUTTranslator_Params::OutputType::ESC_Fwd_and_Rev) {
        min_range = UAVCAN_ESC_MIN_VALUE;
    } else {
        return;
    }

    hal.rcout->set_esc_scaling(rcout_params.pwm_min, rcout_params.pwm_max);

    for (uint8_t i = rcout.chan_start; i <= rcout.chan_end; i++) {
        // scale rc from "0 -> 8191" to "pwm_min -> pwm_max"
        const uint16_t output_pwm = linear_interpolate(rcout_params.pwm_min, rcout_params.pwm_max, rc[i], min_range, UAVCAN_ESC_MAX_VALUE);
        hal.rcout->write(i - rcout.chan_start - 1, output_pwm);
    }
}

void AP_Periph_FW::translate_rcout_srv(uint8_t chan, float rc)
{
    if (rcout.num_channels == 0 ||
        chan < rcout.chan_start ||
        chan > rcout.chan_end ||
        rcout.output_type != RCOUTTranslator_Params::OutputType::Servo)
    {
        return;
    }

    // scale rc from "-1 -> +1" to "_pwm_min -> _pwm_max"
    const uint16_t output_pwm = linear_interpolate(rcout_params.pwm_min, rcout_params.pwm_max, rc, -1.0f, 1.0f);
    hal.rcout->write(chan-1, output_pwm);
}

void AP_Periph_FW::translate_rcout_handle_safety_state(uint8_t safety_state)
{
    if (safety_state == 255) {
        hal.rcout->force_safety_off();
    } else {
        hal.rcout->force_safety_on();
    }
}


void AP_Periph_FW::translate_rcout_update()
{
    if (rcout.chan_start != rcout_params.chan_start ||
            rcout.chan_end != rcout_params.chan_end ||
            rcout.pwm_type != rcout_params.pwm_type ||
            rcout.output_type != rcout_params.output_type ||
            rcout.frequency != rcout_params.frequency)
    {
        // allow for run-time configuration
        init_rcout_translator();
    }

}

#endif // HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR
