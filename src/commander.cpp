#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <SDL2/SDL_gamecontroller.h>
#include <sys/time.h>
#include <linux/can.h>

#include "oscc.h"
#include "vehicles.h"
#include "compile.h"

#include "can_protocols/brake_can_protocol.h"
#include "can_protocols/steering_can_protocol.h"
#include "can_protocols/throttle_can_protocol.h"
#include "can_protocols/fault_can_protocol.h"

#include "node.h"

#define STEERING_RANGE_PERCENTAGE (0.36)

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define JOYSTICK_AXIS_THROTTLE (SDL_CONTROLLER_AXIS_TRIGGERRIGHT)
#define JOYSTICK_AXIS_BRAKE (SDL_CONTROLLER_AXIS_TRIGGERLEFT)
#define JOYSTICK_AXIS_STEER (SDL_CONTROLLER_AXIS_LEFTX)
#define JOYSTICK_BUTTON_ENABLE_CONTROLS (SDL_CONTROLLER_BUTTON_START)
#define JOYSTICK_BUTTON_DISABLE_CONTROLS (SDL_CONTROLLER_BUTTON_BACK)

#define BRAKES_ENABLED_MIN (0.05)
#define JOYSTICK_DELAY_INTERVAL (50000)
#define COMMANDER_ENABLED (1)
#define COMMANDER_DISABLED (0)
#define BRAKE_FILTER_FACTOR (0.2)
#define THROTTLE_FILTER_FACTOR (0.2)
#define STEERING_FILTER_FACTOR (0.1)
#define STEERING_RANGE_PERCENTAGE (0.36)
// STEERING_RANGE_PERCENTAGE 0.36

static int commander_enabled = COMMANDER_DISABLED;

static bool control_enabled = false;

static double curr_angle;

static int commander_disable_controls();
static int commander_enable_controls();
static void brake_callback(oscc_brake_report_s *report);
static void throttle_callback(oscc_throttle_report_s *report);
static void steering_callback(oscc_steering_report_s *report);
static void fault_callback(oscc_fault_report_s *report);
static void obd_callback(struct can_frame *frame);

state previous;

int commander_init(int channel)
{
    int return_code = OSCC_ERROR;

    if (commander_enabled == COMMANDER_DISABLED)
    {
        commander_enabled = COMMANDER_ENABLED;

        return_code = oscc_open(channel);

        if (return_code != OSCC_ERROR)
        {
            // register callback handlers
            oscc_subscribe_to_obd_messages(obd_callback);
            oscc_subscribe_to_brake_reports(brake_callback);
            oscc_subscribe_to_steering_reports(steering_callback);
            oscc_subscribe_to_throttle_reports(throttle_callback);
            oscc_subscribe_to_fault_reports(fault_callback);
        }
    }
    return (return_code);
}

void commander_close(int channel)
{
    if (commander_enabled == COMMANDER_ENABLED)
    {
        commander_disable_controls();

        oscc_disable();

        oscc_close(channel);

        commander_enabled = COMMANDER_DISABLED;
    }
}

static int commander_disable_controls()
{
    int return_code = OSCC_ERROR;

    if ((commander_enabled == COMMANDER_ENABLED) && (control_enabled == true))
    {
        printf("Disable controls\n");

        return_code = oscc_disable();

        if (return_code == OSCC_OK)
        {
            control_enabled = false;
        }
    }
    else
    {
        return_code = OSCC_OK;
    }

    return return_code;
}

static int commander_enable_controls()
{
    int return_code = OSCC_ERROR;

    if ((commander_enabled == COMMANDER_ENABLED) && (control_enabled == false))
    {
        printf("Enable controls\n");

        return_code = oscc_enable();

        if (return_code == OSCC_OK)
        {
            control_enabled = true;
        }
    }
    else
    {
        return_code = OSCC_OK;
    }

    return (return_code);
}

/*
 * These callback functions just check the reports for operator overrides. The
 * firmware modules should have disabled themselves, but we will send the
 * command again just to be safe.
 *
 */
static void throttle_callback(oscc_throttle_report_s *report)
{
    if (report->operator_override)
    {
        commander_disable_controls();

        printf("Override: Throttle\n");
    }
}

static void steering_callback(oscc_steering_report_s *report)
{
    if (report->operator_override)
    {
        commander_disable_controls();

        printf("Override: Steering\n");
    }
}

static void brake_callback(oscc_brake_report_s *report)
{
    if (report->operator_override)
    {
        commander_disable_controls();

        printf("Override: Brake\n");
    }
}

static void fault_callback(oscc_fault_report_s *report)
{
    commander_disable_controls();

    printf("Fault: ");

    if (report->fault_origin_id == FAULT_ORIGIN_BRAKE)
    {
        printf("Brake\n");
    }
    else if (report->fault_origin_id == FAULT_ORIGIN_STEERING)
    {
        printf("Steering\n");
    }
    else if (report->fault_origin_id == FAULT_ORIGIN_THROTTLE)
    {
        printf("Throttle\n");
    }
}
// To cast specific OBD messages, you need to know the structure of the
// data fields and the CAN_ID.
static void obd_callback(struct can_frame *frame)
{
    if (frame->can_id == KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID)
    {
        kia_soul_obd_steering_wheel_angle_data_s *steering_data = (kia_soul_obd_steering_wheel_angle_data_s *)frame->data;

        curr_angle = steering_data->steering_wheel_angle * KIA_SOUL_OBD_STEERING_ANGLE_SCALAR;

        DrivekitNode::publishAngle(curr_angle);
    }
    if (frame->can_id == KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID)
    {
        long offset = 0;
        uint16_t raw = ((frame->data[offset + 1] & 0x0F) << 8) | frame->data[offset];
        // 10^-1 precision, raw / 32.0
        double wheel_speed = (double)((int)((double)raw / 3.2) / 10.0);

        DrivekitNode::publishSpeed(wheel_speed);
    }
}

void commander_update(state &car_state, rclcpp::Time time)
{
    if (car_state.enabled && !previous.enabled)
        commander_enable_controls();
    if (!car_state.enabled && previous.enabled)
        commander_disable_controls();
    if (commander_enabled == COMMANDER_ENABLED && control_enabled == true)
    {
        //if (car_state.brakes_time + car_state.max_duration > time)
            oscc_publish_brake_position(car_state.brakes);
        auto normalized_throttle_position = car_state.throttle;
        if (car_state.brakes >= BRAKES_ENABLED_MIN)
        {
            normalized_throttle_position = 0.0;
        }
        //if (car_state.throttle_time + car_state.max_duration > time)
            oscc_publish_throttle_position(normalized_throttle_position);
        //if (car_state.steering_torque_time + car_state.max_duration > time)
            oscc_publish_steering_torque(car_state.steering_torque);
    }
    previous = car_state;
}