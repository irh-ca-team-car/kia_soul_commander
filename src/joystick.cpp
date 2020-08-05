/**
 * @file joystick.c
 * @brief Joystick Interface Source
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <SDL2/SDL_gamecontroller.h>

#include "oscc.h"
#include "compile.h"
#include "joystick.h"

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

#define BUTTON_PRESSED_DELAY (5000)

#define JOYSTICK_DEVICE_CONTROLLER_INVALID (NULL)

#define JOYSTICK_ID_DATA_SIZE (16)

#define JOYSTICK_ID_STRING_SIZE (64)

oscc_result_t check_trigger_positions();

typedef struct
{
    unsigned char data[JOYSTICK_ID_DATA_SIZE];
    char ascii_string[JOYSTICK_ID_STRING_SIZE];

} joystick_guid_s;

typedef struct
{
    void *controller;

    void *haptic;

    joystick_guid_s *guid;

} joystick_device_data_s;

static joystick_guid_s joystick_guid;
static joystick_device_data_s joystick_data = {
    .controller = NULL,
    .haptic = NULL,
    .guid = &joystick_guid};

static joystick_device_data_s *joystick = NULL;

static oscc_result_t joystick_init_subsystem()
{
    oscc_result_t ret = OSCC_ERROR;

    if (joystick == NULL)
    {
        int init_result = SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);

        ret = OSCC_OK;

        if (init_result < 0)
        {
            printf("OSCC_ERROR: SDL_Init - %s\n", SDL_GetError());
            ret = OSCC_ERROR;
        }
    }

    return ret;
}
static oscc_result_t get_button(unsigned long button, unsigned int *const state)
{
    oscc_result_t return_code = OSCC_ERROR;

    if (state != NULL)
    {
        unsigned int button_state;

        return_code = joystick_get_button(button, &button_state);

        if ((return_code == OSCC_OK) &&
            (button_state == JOYSTICK_BUTTON_STATE_PRESSED))
        {
            (*state) = 1;
        }
        else
        {
            (*state) = 0;
        }
    }

    return (return_code);
}
void smooth(double *v, int e)
{
    bool negative = *v < 0;

    double tmp = *v;
    for (int i = 1; i < e; i++)
        tmp = tmp * *v;
    if (e == 0)
        tmp = 1 - sqrt(1 - tmp * tmp);
    *v = tmp;
    if (negative)
        *v = -abs(*v);
}
static oscc_result_t joystick_get_guid_at_index(unsigned long device_index)
{
    oscc_result_t ret = OSCC_ERROR;

    if (joystick != NULL)
    {
        ret = OSCC_OK;

        const SDL_JoystickGUID m_guid =
            SDL_JoystickGetDeviceGUID((int)device_index);

        memcpy(joystick_guid.data, m_guid.data, sizeof(m_guid.data));

        memset(joystick_guid.ascii_string, 0,
               sizeof(joystick_guid.ascii_string));

        SDL_JoystickGetGUIDString(m_guid,
                                  joystick_guid.ascii_string,
                                  sizeof(joystick_guid.ascii_string));
    }
    return ret;
}

static int joystick_get_num_devices()
{
    int num_joysticks = OSCC_ERROR;

    if (joystick != NULL)
    {
        num_joysticks = SDL_NumJoysticks();

        if (num_joysticks < 0)
        {
            printf("OSCC_ERROR: SDL_NumJoysticks - %s\n", SDL_GetError());
            num_joysticks = OSCC_ERROR;
        }
    }
    return (num_joysticks);
}

oscc_result_t joystick_init()
{
    oscc_result_t ret = OSCC_OK;

    ret = joystick_init_subsystem();

    if (ret == OSCC_ERROR)
    {
        printf("init subsystem error\n");
    }
    else
    {
        joystick = &joystick_data;
        joystick->controller = JOYSTICK_DEVICE_CONTROLLER_INVALID;

        const int num_joysticks = joystick_get_num_devices();

        if (num_joysticks > 0)
        {
            unsigned long device_index = 0;

            ret = joystick_get_guid_at_index(device_index);

            if (ret == OSCC_OK)
            {
                printf("Found %d devices -- connecting to device at system index %lu - GUID: %s\n",
                       num_joysticks,
                       device_index,
                       joystick_guid.ascii_string);

                ret = (oscc_result_t)joystick_open(device_index);
            }
        }
        else
        {
            printf("No joystick/devices available on the host\n");
        }
    }
    printf("Waiting for joystick controls to zero\n");
    ret = OSCC_OK;
    while (ret != OSCC_ERROR)
    {
        ret = check_trigger_positions();

        if (ret == OSCC_WARNING)
        {
            (void)usleep(JOYSTICK_DELAY_INTERVAL);
        }
        else if (ret == OSCC_ERROR)
        {
            printf("Failed to wait for joystick to zero the control values\n");
        }
        else
        {
            printf("Joystick controls successfully initialized\n");

            break;
        }
    }
}

oscc_result_t joystick_open(unsigned long device_index)
{
    oscc_result_t ret = OSCC_ERROR;

    if (joystick != NULL)
    {
        joystick->controller =
            (void *)SDL_GameControllerOpen((int)device_index);

        if (joystick->controller == JOYSTICK_DEVICE_CONTROLLER_INVALID)
        {
            printf("OSCC_ERROR: SDL_JoystickOpen - %s\n", SDL_GetError());
        }
        else
        {
            ret = OSCC_OK;

            const SDL_JoystickGUID m_guid =
                SDL_JoystickGetGUID(
                    SDL_GameControllerGetJoystick((SDL_GameController *)joystick->controller));

            memcpy(joystick_guid.data, m_guid.data, sizeof(m_guid.data));

            memset(joystick_guid.ascii_string,
                   0,
                   sizeof(joystick_guid.ascii_string));

            SDL_JoystickGetGUIDString(m_guid,
                                      joystick_guid.ascii_string,
                                      sizeof(joystick_guid.ascii_string));

            joystick->haptic =
                (void *)SDL_HapticOpenFromJoystick(
                    SDL_GameControllerGetJoystick((SDL_GameController *)joystick->controller));

            if (SDL_HapticRumbleInit((SDL_Haptic *)joystick->haptic) != 0)
            {
                SDL_HapticClose((SDL_Haptic *)joystick->haptic);
            }
        }
    }
    return ret;
}

void joystick_close()
{
    if (joystick != NULL)
    {
        if (joystick->controller != JOYSTICK_DEVICE_CONTROLLER_INVALID)
        {
            if (SDL_GameControllerGetAttached((SDL_GameController *)joystick->controller) == SDL_TRUE)
            {
                if (joystick->haptic)
                {
                    SDL_HapticClose((SDL_Haptic *)joystick->haptic);
                }
                SDL_GameControllerClose((SDL_GameController *)joystick->controller);
            }

            joystick->controller = JOYSTICK_DEVICE_CONTROLLER_INVALID;
        }
        joystick = NULL;
    }
    // Release the joystick subsystem
    SDL_Quit();
}

oscc_result_t joystick_update()
{
    oscc_result_t ret = OSCC_ERROR;

    if (joystick != NULL)
    {
        if (joystick->controller != JOYSTICK_DEVICE_CONTROLLER_INVALID)
        {
            SDL_GameControllerUpdate();

            if (SDL_GameControllerGetAttached((SDL_GameController *)joystick->controller) == SDL_FALSE)
            {
                printf("SDL_GameControllerGetAttached - device not attached\n");
            }
            else
            {
                ret = OSCC_OK;
            }
        }
    }
    return ret;
}

oscc_result_t joystick_get_axis(unsigned long axis_index, int *const position)
{
    oscc_result_t ret = OSCC_ERROR;

    if ((joystick != NULL) && (position != NULL))
    {
        ret = OSCC_OK;

        const Sint16 pos = SDL_GameControllerGetAxis((SDL_GameController *)joystick->controller,
                                                     (SDL_GameControllerAxis)axis_index);

        (*position) = (int)pos;
    }

    return ret;
}

oscc_result_t joystick_get_button(unsigned long button_index,
                                  unsigned int *const button_state)
{
    oscc_result_t ret = OSCC_ERROR;

    if ((joystick != NULL) && (button_state != NULL))
    {
        ret = OSCC_OK;

        const Uint8 m_state = SDL_GameControllerGetButton((SDL_GameController *)joystick->controller,
                                                          (SDL_GameControllerButton)button_index);

        if (m_state == 1)
        {
            (*button_state) = JOYSTICK_BUTTON_STATE_PRESSED;

            if (joystick->haptic)
            {
                SDL_HapticRumblePlay((SDL_Haptic *)joystick->haptic, 1.0f, 100);
            }

            (void)usleep(BUTTON_PRESSED_DELAY);
        }
        else
        {
            (*button_state) = JOYSTICK_BUTTON_STATE_NOT_PRESSED;
        }
    }

    return ret;
}
oscc_result_t check_for_controller_update(state &car_state)
{
    static unsigned int disable_button_previous = 0;
    unsigned int disable_button_current = 0;

    oscc_result_t return_code = joystick_update();

    if (return_code == OSCC_OK)
    {
        return_code = get_button(JOYSTICK_BUTTON_DISABLE_CONTROLS,
                                 &disable_button_current);
    }

    if (return_code == OSCC_OK)
    {
        if ((disable_button_previous != 1) && (disable_button_current != 0))
        {
            car_state.enabled = false;
        }

        disable_button_previous = disable_button_current;
    }

    static unsigned int enable_button_previous = 0;
    unsigned int enable_button_current = 0;

    if (return_code == OSCC_OK)
    {
        return_code = get_button(JOYSTICK_BUTTON_ENABLE_CONTROLS,
                                 &enable_button_current);

        if (return_code == OSCC_OK)
        {
            if ((enable_button_previous != 1) && (enable_button_current != 0))
            {
                car_state.enabled = true;
            }
            enable_button_previous = enable_button_current;
        }
    }

    if (return_code == OSCC_OK)
    {
        return_code = command_brakes(car_state);

        if (return_code == OSCC_OK)
        {
            return_code = command_throttle(car_state);
        }

        if (return_code == OSCC_OK)
        {
            return_code = command_steering(car_state);
        }
    }

    return return_code;
}
static oscc_result_t get_normalized_position(unsigned long axis_index, double *const normalized_position)
{
    oscc_result_t return_code = OSCC_ERROR;

    int axis_position = 0;

    static const float deadzone = 0.1;

    return_code = joystick_get_axis(axis_index, &axis_position);

    if (return_code == OSCC_OK)
    {
        // this is between -1.0 and 1.0
        double raw_normalized_position = ((double)axis_position / INT16_MAX);

        if (axis_index == JOYSTICK_AXIS_STEER)
        {
            // if axis is in deadzone, do nothing
            if (fabs(raw_normalized_position) < deadzone)
            {
                (*normalized_position) = 0.0;
            }
            else
            {
                // normalize over non deadzone range
                raw_normalized_position *= (fabs(raw_normalized_position) - deadzone) / (1.0 - deadzone);

                (*normalized_position) = CONSTRAIN(
                    raw_normalized_position,
                    -1.0,
                    1.0);
            }
        }
        else
        {
            (*normalized_position) = CONSTRAIN(
                raw_normalized_position,
                0.0,
                1.0);
        }
    }

    *normalized_position = *normalized_position;

    return (return_code);
}
oscc_result_t check_trigger_positions()
{
    oscc_result_t return_code = OSCC_ERROR;

    return_code = joystick_update();

    double normalized_brake_position = 0;

    if (return_code == OSCC_OK)
    {
        return_code = get_normalized_position(JOYSTICK_AXIS_BRAKE, &normalized_brake_position);
    }

    double normalized_throttle_position = 0;

    if (return_code == OSCC_OK)
    {
        return_code = get_normalized_position(JOYSTICK_AXIS_THROTTLE, &normalized_throttle_position);
    }

    if (return_code == OSCC_OK)
    {
        if ((normalized_throttle_position > 0.0) || (normalized_brake_position > 0.0))
        {
            return_code = OSCC_WARNING;
        }
    }

    return return_code;
}
oscc_result_t command_brakes(state &car_state)
{
    oscc_result_t return_code = OSCC_OK;
    double normalized_position;
    return_code = get_normalized_position(JOYSTICK_AXIS_BRAKE, &normalized_position);
    if (return_code == OSCC_OK && normalized_position >= 0.0)
    {
        car_state.brakes=normalized_position;
    }
    return (return_code);
}
oscc_result_t command_steering(state &car_state)
{
    oscc_result_t return_code = OSCC_OK;
    double normalized_position;
    return_code = get_normalized_position(JOYSTICK_AXIS_STEER, &normalized_position);
    if (return_code == OSCC_OK )
    {
        car_state.steering_torque=normalized_position* STEERING_RANGE_PERCENTAGE;
    }
    return (return_code);
}
oscc_result_t command_throttle(state &car_state)
{
    oscc_result_t return_code = OSCC_OK;
    double normalized_position;
    return_code = get_normalized_position(JOYSTICK_AXIS_THROTTLE, &normalized_position);
    if (return_code == OSCC_OK&& normalized_position >= 0.0 )
    {
        car_state.throttle=normalized_position;
    }
    return (return_code);
}