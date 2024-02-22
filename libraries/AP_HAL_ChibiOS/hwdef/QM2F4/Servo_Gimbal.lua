-- Configuration
local PITCH_SERVO_CHANNEL = 1 -- Adjust for pitch servo's channel
local ROLL_SERVO_CHANNEL = 2 -- Adjust for roll servo's channel
local PITCH_SWITCH_CHANNEL = 3 -- RC channel for the pitch control switch
local ROLL_SWITCH_CHANNEL = 4 -- RC channel for the roll control switch
local P_GAIN = 10 -- Proportional gain
local I_GAIN = 0.1 -- Integral gain
local MAX_INTEGRAL = 500 -- Maximum integral term
local INTEGRAL_DECAY_FACTOR = 0.99 -- Decay factor
local PITCH_NEUTRAL_PWM = 1500 -- Neutral point for pitch servo
local ROLL_NEUTRAL_PWM = 1500 -- Neutral point for roll servo

-- State
local pitch_integral = 0
local roll_integral = 0

function getSwitchPosition(switch_channel)
    local pwm = rc:get_pwm(switch_channel)
    if pwm < 1300 then
        return 1 -- Servo Centered
    elseif pwm < 1700 then
        return 2 -- Decay Mode
    else
        return 3 -- Full Gimbal Control
    end
end

function update()
    local pitch_mode = getSwitchPosition(PITCH_SWITCH_CHANNEL)
    local roll_mode = getSwitchPosition(ROLL_SWITCH_CHANNEL)
    local pitch = math.deg(ahrs:get_pitch())
    local roll = math.deg(ahrs:get_roll())

    -- Handle pitch servo based on pitch_mode
    local pitch_target_pwm = handleServo(pitch_mode, pitch, PITCH_SERVO_CHANNEL, pitch_integral, PITCH_NEUTRAL_PWM)
    pitch_integral = pitch_target_pwm.integral

    -- Handle roll servo based on roll_mode
    local roll_target_pwm = handleServo(roll_mode, roll, ROLL_SERVO_CHANNEL, roll_integral, ROLL_NEUTRAL_PWM)
    roll_integral = roll_target_pwm.integral

    return update, 100 -- Run at 10 Hz
end

function handleServo(mode, angle, servo_channel, integral, neutral_pwm)
    local target_pwm
    local proportional = angle * P_GAIN

    if mode == 1 then
        -- Servo Centered Mode
        integral = 0
        target_pwm = neutral_pwm
    else
        if mode == 2 then
            -- Decay Mode
            integral = (integral + angle * I_GAIN) * INTEGRAL_DECAY_FACTOR
        elseif mode == 3 then
            -- Full Gimbal Control
            integral = integral + angle * I_GAIN
        end

        -- Prevent integral wind-up
        integral = math.max(-MAX_INTEGRAL, math.min(MAX_INTEGRAL, integral))
        
        -- Calculate servo adjustment
        local servo_adjustment = proportional + integral
        target_pwm = neutral_pwm - servo_adjustment
    end

    -- Limit PWM to valid servo range
    target_pwm = math.max(1000, math.min(2000, target_pwm))
    
    -- Set servo position
    SRV_Channels:set_output_pwm(servo_channel, target_pwm)

    return {integral = integral}
end

return update()
