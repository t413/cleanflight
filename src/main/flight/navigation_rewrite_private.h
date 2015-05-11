/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "flight/pid.h"
#include "sensors/barometer.h"
#include "io/rc_controls.h"
#include "io/escservo.h"

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR 1.113195f  // MagicEarthNumber from APM

#define LANDING_DETECTION_TIMEOUT   10000000     // 10 second timeout
#define RADX100                     0.000174532925f
#define CROSSTRACK_GAIN             1
#define NAV_ROLL_PITCH_MAX          300 // Max control input from NAV

// Should apply position-to-velocity PID controller for POS_HOLD
#define navShouldApplyPosHold() ((navMode & (NAV_MODE_POSHOLD_2D | NAV_MODE_POSHOLD_3D)) != 0)
// Should apply position-to-velocity PID controller for waypoint navigation (WP/RTH)
#define navShouldApplyWaypoint() ((navMode & (NAV_MODE_WP | NAV_MODE_RTH)) != 0)
// Should apply altitude PID controller
#define navShouldApplyAltHold() ((navMode & (NAV_MODE_ALTHOLD | NAV_MODE_POSHOLD_3D | NAV_MODE_WP | NAV_MODE_RTH)) != 0)

// 
#define navShouldApplyHeadingControl() ((navMode & (NAV_MODE_WP | NAV_MODE_RTH | NAV_MODE_POSHOLD_2D | NAV_MODE_POSHOLD_3D)) != 0)
#define navShouldAdjustHeading() ((navMode & (NAV_MODE_WP | NAV_MODE_RTH)) != 0)

#define navCanAdjustVerticalVelocityFromRCInput() ((navMode & (NAV_MODE_ALTHOLD | NAV_MODE_POSHOLD_3D)) != 0)
#define navCanAdjustHorizontalVelocityFromRCInput() ((navMode & (NAV_MODE_POSHOLD_2D | NAV_MODE_POSHOLD_3D)) != 0)
#define navCanAdjustHeadingFromRCInput() ((navMode & (NAV_MODE_POSHOLD_2D | NAV_MODE_POSHOLD_3D)) != 0)

typedef enum {
    NAV_RTH_STATE_INIT = 0,
    NAV_RTH_STATE_CLIMB_TO_SAVE_ALTITUDE,
    NAV_RTH_STATE_HEAD_HOME,
    NAV_RTH_STATE_HOME_AUTOLAND,
    NAV_RTH_STATE_LANDED,
    NAV_RTH_STATE_FINISHED,
} navRthState_t;

typedef struct {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

typedef struct {
    PID_PARAM param;
    float integrator;       // integrator value
    float last_input;       // last input for derivative
    float last_derivative;  // last derivative for low-pass filter
    float output;
    float derivative;
} PID;
