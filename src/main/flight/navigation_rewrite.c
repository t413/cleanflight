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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_rx.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/gps_conversion.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// Position hold / waypoint PIDs, take position error (cm) and output desired velocities (cm/s)
// NODE: Velocities are output in global frame of reference (ENU coordinates)
static PID altitudePID;     // Used for all except ALTHOLD
static PID positionPID;     // Used for NAV_MODE_POSHOLD_2D, NAV_MODE_POSHOLD_3D, NAV_MODE_ASSISTED
                               // NAV_RTH & NAV_WP calculate desired speed without PID controller

// Heading PID
static PID headingRatePID;      // Takes heading error (deg) and outputs yaw rate

// NOTE: Input velocities are in *rotated* frame of reference (aircraft based, LON = roll axis, LAT = pitch axis)
static PID altitudeRatePID;         // Takes desired velocity (cm/s) and outputs raw control adjustment
static PID navigationRatePID[2];   // PID controller for WP and RTH
static PID posholdRatePID[2];  // PID controller for PH

// Current velocities in 3D space in global ENU coordinate system
float actualVerticalVelocity;        // cm/s
float actualHorizontalVelocity[2];   // cm/s

// Current position in 3D space for navigation purposes (may be different from GPS output)
navPosition3D_t actualPosition;
static float gpsScaleLonDown;

// Home & hold position
navPosition3D_t homePosition;      // Home position (ENU coordinates)
static int16_t altholdInitialThrottle;  // Throttle input when althold was activated
static navRthState_t rthState;

uint32_t distanceToHome;
int32_t directionToHome;

// Current active waypoint and desired heading. For PH, ALTHOLD this is used for heading lock,
// For WP/RTH used for target bearing
navPosition3D_t activeWpOrHoldPosition;    // Used for WP/ALTHOLD/PH/RTH

// Desired velocities, might be set by pilot or by NAV position PIDs (ENU coordinates)
static float desiredVerticalVel;
static float desiredHorizontalVel[2];
static int32_t desiredHeading;

// Desired pitch/roll/yaw/throttle adjustments
static int16_t rcAdjustment[4];
int16_t rcCommandNav[4];

// Current navigation mode & profile
navigationMode_e navMode = NAV_MODE_NONE;    // Navigation mode
static navProfile_t *navProfile;
static barometerConfig_t *barometerConfig;
static rcControlsConfig_t *rcControlsConfig;

/*-----------------------------------------------------------
 * PID implementation
 *-----------------------------------------------------------*/
static float pidGetP(float error, PID *pid)
{
    return error * pid->param.kP;
}

static float pidGetI(float error, float dt, PID *pid)
{
    pid->integrator += ((float)error * pid->param.kI) * dt;
    pid->integrator = constrainf(pid->integrator, -pid->param.Imax, pid->param.Imax);
    return pid->integrator;
}

static float pidGetD(float input, float dt, PID *pid)
{
    pid->derivative = (input - pid->last_input) / dt;

    // Low pass filter cut frequency for derivative calculation
    // Set to  "1 / ( 2 * PI * nav_lpf )
    float pidFilter = (1.0f / (2.0f * M_PIf * (float)navProfile->nav_lpf));
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->last_derivative + (dt / (pidFilter + dt)) * (pid->derivative - pid->last_derivative);
    // update state
    pid->last_input = input;
    pid->last_derivative = pid->derivative;
    // add in derivative component
    return pid->param.kD * pid->derivative;
}

static float pidGetPI(float error, float dt, PID *pid)
{
    return pidGetP(error, pid) + pidGetI(error, dt, pid);
}

static float pidGetPID(float error, float dt, PID *pid)
{
    return pidGetP(error, pid) + pidGetI(error, dt, pid) + pidGetD(error, dt, pid);
}

static void pidReset(PID *pid)
{
    pid->integrator = 0;
    pid->last_input = 0;
    pid->last_derivative = 0;
}

static void pidInit(PID *pid, float _kP, float _kI, float _kD, float _Imax)
{
    pid->param.kP = _kP;
    pid->param.kI = _kI;
    pid->param.kD = _kD;
    pid->param.Imax = _Imax;
    pidReset(pid);
}

/*-----------------------------------------------------------
 * Utilities
 *-----------------------------------------------------------*/
static int32_t wrap_18000(int32_t error)
{
    if (error > 18000)
        error -= 36000;
    if (error < -18000)
        error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle)
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

#define DEGREES_80_IN_DECIDEGREES 800
bool navIsThrustFacingDownwards(rollAndPitchInclination_t *inclination)
{
    return ABS(inclination->values.rollDeciDegrees) < DEGREES_80_IN_DECIDEGREES && ABS(inclination->values.pitchDeciDegrees) < DEGREES_80_IN_DECIDEGREES;
}

/*
* This (poorly named) function merely returns whichever is higher, roll inclination or pitch inclination.
* //TODO: Fix this up. We could either actually return the angle between 'down' and the normal of the craft
* (my best interpretation of scalar 'tiltAngle') or rename the function.
*/
int16_t calculateTiltAngle(rollAndPitchInclination_t *inclination)
{
    return MAX(ABS(inclination->values.rollDeciDegrees), ABS(inclination->values.pitchDeciDegrees));
}

void resetNavigation(void)
{
    int i;

    pidReset(&positionPID);
    pidReset(&altitudePID);
    pidReset(&altitudeRatePID);
    //pidReset(&headingRatePID);

    for (i = 0; i < 2; i++) {
        pidReset(&navigationRatePID[i]);
        pidReset(&posholdRatePID[i]);
    }

    for (i = 0; i < 4; i++) {
        rcAdjustment[i] = 0;
    }
}

/*-----------------------------------------------------------
 * NAV actual position calculations
 *  - position & altitude might be updated using different sensors,
 *    so we update them separately
 *-----------------------------------------------------------*/
static void updateActualHorizontalPosition(int32_t newLat, int32_t newLon)
{
    actualPosition.coordinates[LAT] = newLat;
    actualPosition.coordinates[LON] = newLon;
}

static void updateActualHorizontalVelocity(float newLatVel, float newLonVel)
{
    actualHorizontalVelocity[X] = newLonVel;
    actualHorizontalVelocity[Y] = newLatVel;
}

static void updateActualAltitudeAndVelocity(float newAltitude, float newVelocity)
{
    actualPosition.altitude = newAltitude;
    actualVerticalVelocity = newVelocity;
}

static void updateActualHeading(int32_t newHeading)
{
    /* Update heading */
    actualPosition.heading = newHeading;
}

/*-----------------------------------------------------------
 * NAV generic position control
 *-----------------------------------------------------------*/
#define TAN_89_99_DEGREES 5729.57795f
static void calculateDistanceAndBearingToDestination(navPosition3D_t *currentPos, navPosition3D_t *destinationPos, uint32_t *dist, int32_t *bearing)
{
    float dX = (destinationPos->coordinates[LON] - currentPos->coordinates[LON]) * gpsScaleLonDown;
    float dY = destinationPos->coordinates[LAT] - currentPos->coordinates[LAT];

    if (dist) {
        *dist = sqrtf(sq(dX) + sq(dY)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
    }

    if (bearing) {
        *bearing = 9000.0f + atan2f(-dY, dX) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
        if (*bearing < 0)
            *bearing += 36000;
    }
}

// Takes current and previous position in centidegrees and calculates error in cm
static void calculatePositionError(navPosition3D_t *currentPos, navPosition3D_t *destinationPos, navPosition3D_t *error)
{
    error->coordinates[LON] = (destinationPos->coordinates[LON] - currentPos->coordinates[LON]) * gpsScaleLonDown * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
    error->coordinates[LAT] = (destinationPos->coordinates[LAT] - currentPos->coordinates[LAT]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
    error->altitude = destinationPos->altitude - currentPos->altitude;
}

static bool navIsWaypointReached(navPosition3D_t *currentPos, navPosition3D_t *destinationPos)
{
    uint32_t wpDistance;

    calculateDistanceAndBearingToDestination(currentPos, destinationPos, &wpDistance, NULL);

    // We consider waypoint reached if within specified radius
    return (wpDistance <= navProfile->nav_wp_radius);
}

static void calculateDesiredHorizontalVelocity(navPosition3D_t *currentPos, navPosition3D_t *destinationPos, float dTnav)
{
    navPosition3D_t posError;
    uint32_t wpDistance;

    UNUSED(dTnav);

    // Calculate position error
    calculatePositionError(currentPos, destinationPos, &posError);
    calculateDistanceAndBearingToDestination(currentPos, destinationPos, &wpDistance, NULL);

    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        // Algorithm depends on navigation mode (WP/RTH or PH)
        // We use PH PID governors if explicitly in NAV_MODE_POSHOLD or within 2*waypoint radius
        if (navShouldApplyPosHold() || (wpDistance < 2 * navProfile->nav_wp_radius)) {
            desiredHorizontalVel[X] = pidGetPI(posError.coordinates[LON], dTnav, &positionPID);
            desiredHorizontalVel[Y] = pidGetPI(posError.coordinates[LAT], dTnav, &positionPID);
        }
        else if (navShouldApplyWaypoint()) {
            float navCurrentSpeed = sqrtf(sq(actualHorizontalVelocity[LON] + sq(actualHorizontalVelocity[LAT])));
            float targetSpeed = MIN(navProfile->nav_speed_max, wpDistance / 2.0f); // if close - navigate to reach a waypoint within 2 sec.

            // Avoid fast acceleration, increase speed in small steps
            if (navCurrentSpeed < targetSpeed) {
                targetSpeed = navCurrentSpeed + 50.0f;
            }

            targetSpeed = MAX(navProfile->nav_speed_min, targetSpeed);  // go at least min_speed

            // Calculate desired horizontal velocities
            desiredHorizontalVel[X] = targetSpeed * (posError.coordinates[LON] / wpDistance);
            desiredHorizontalVel[Y] = targetSpeed * (posError.coordinates[LAT] / wpDistance);
        }
        else {
            desiredHorizontalVel[X] = 0;
            desiredHorizontalVel[Y] = 0;
        }
    }
}

static void calculateDesiredHeading(navPosition3D_t *currentPos, navPosition3D_t *destinationPos, float dTnav)
{
    UNUSED(dTnav);

    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        // Depending on flight mode, we rotate out heading towards the next waypoint or keep it as it is
        if (navShouldAdjustHeading()) {
            // Navigating, rotate head towards next waypoint
            uint32_t wpDistance;
            int32_t wpBearing;

            calculateDistanceAndBearingToDestination(currentPos, destinationPos, &wpDistance, &wpBearing);

            // TODO: Apply crosstrack correction

            /*
            // Calculating cross track error, this tries to keep the copter on a direct line when flying to a waypoint.
            if (ABS(wrap_18000(wpBearing - originalWaypointBearing)) < 4500) {     // If we are too far off or too close we don't do track following
                float temp = (wpBearing - originalWaypointBearing) * RADX100;
                float crosstrackError = sinf(temp) * (wpDistance * CROSSTRACK_GAIN); // Meters we are off track line
                desiredBearing = wpBearing + constrain(crosstrackError, -3000, 3000);
                desiredBearing = wrap_36000(desiredBearing);
            } else {
                desiredBearing = wpBearing;
            }
            */

            desiredHeading = wrap_36000(wpBearing);
        }
        else {
            // Keep the heading as it was rewuired when setting the destination
            desiredHeading = wrap_36000(destinationPos->heading);
        }
    }
}

static void calculateDesiredVerticalVelocity(navPosition3D_t *currentPos, navPosition3D_t *destinationPos, float dTnav)
{
    navPosition3D_t posError;

    UNUSED(dTnav);

    // Calculate position error
    calculatePositionError(currentPos, destinationPos, &posError);

    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        // Should calculate altitude hold if ALTHOLD, 3D poshold or navigation
        if (navShouldApplyAltHold()) {
            if (navIsThrustFacingDownwards(&inclination)) {
                // Use only P term for PH velocity calculation
                int32_t altitudeError = constrain(posError.altitude, -500, 500);
                altitudeError = applyDeadband(altitudeError, 10); // remove small P parameter to reduce noise near zero position
                desiredVerticalVel = pidGetP(altitudeError, &altitudePID);
                desiredVerticalVel = constrainf(desiredVerticalVel, -300, 300); // hard limit velocity to +/- 3 m/s
            }
            else {
                // don't apply altitude hold if flying upside down
                desiredVerticalVel = 0;
            }
        }
    }
}

/*-----------------------------------------------------------
 * NAV home position
 *-----------------------------------------------------------*/
void resetHomePosition(void)
{
    if (STATE(GPS_FIX) && GPS_numSat >= 5) {
        homePosition.coordinates[LON] = actualPosition.coordinates[LON];
        homePosition.coordinates[LAT] = actualPosition.coordinates[LAT];
        homePosition.altitude = actualPosition.altitude;
        homePosition.heading = actualPosition.heading;
        ENABLE_STATE(GPS_FIX_HOME);
    }
}

void updateHomePosition(void)
{
    // Disarmed, reset home position
    if (!ARMING_FLAG(ARMED))
        DISABLE_STATE(GPS_FIX_HOME);

    // Arming and don't have home position set - do it now
    if (!STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED) && STATE(GPS_FIX) && GPS_numSat >= 5) {
        resetHomePosition();
    }

    // Update distance and direction to home
    if (STATE(GPS_FIX_HOME)) {
        calculateDistanceAndBearingToDestination(&actualPosition, &homePosition, &distanceToHome, &directionToHome);
        distanceToHome = distanceToHome / 100; // back to meters
        directionToHome = directionToHome / 100; // directionToHome should be degrees
    }
}

/*-----------------------------------------------------------
 * NAV set waypoint
 *-----------------------------------------------------------*/
void setNextWaypointAndHeadingLock(uint32_t lat, uint32_t lon, int32_t alt, int32_t head)
{
    activeWpOrHoldPosition.coordinates[LAT] = lat;
    activeWpOrHoldPosition.coordinates[LON] = lon;
    activeWpOrHoldPosition.altitude = alt;
    activeWpOrHoldPosition.heading = head;
}

void setNextWaypointAndCalculateBearing(uint32_t lat, uint32_t lon, int32_t alt)
{
    int32_t wpBearing;

    activeWpOrHoldPosition.coordinates[LAT] = lat;
    activeWpOrHoldPosition.coordinates[LON] = lon;
    activeWpOrHoldPosition.altitude = alt;

    calculateDistanceAndBearingToDestination(&actualPosition, &activeWpOrHoldPosition, NULL, &wpBearing);
    activeWpOrHoldPosition.heading = wpBearing;
}

/*-----------------------------------------------------------
 * NAV attitude PID controllers
 *-----------------------------------------------------------*/
static void calculateAttitudeAdjustment(float dTnav)
{
    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        int axis;

        // rotate velocities into aircraft's frame of reference
        //float rotationAngle = (9000l - desiredBearing) * RADX100;
        float sin_yaw_y = sinf(-actualPosition.heading * RADX100);
        float cos_yaw_x = cosf(-actualPosition.heading * RADX100);

        float desiredAircraftVel[2];
        desiredAircraftVel[X] = desiredHorizontalVel[X] * cos_yaw_x - desiredHorizontalVel[Y] * sin_yaw_y;
        desiredAircraftVel[Y] = desiredHorizontalVel[X] * sin_yaw_y + desiredHorizontalVel[Y] * cos_yaw_x;

        float actualAircraftVel[2];
        actualAircraftVel[X] = actualHorizontalVelocity[X] * cos_yaw_x - actualHorizontalVelocity[Y] * sin_yaw_y;
        actualAircraftVel[Y] = actualHorizontalVelocity[X] * sin_yaw_y + actualHorizontalVelocity[Y] * cos_yaw_x;

        // Start with zero adjustments
        for (axis = 0; axis < 4; axis++) {
            rcAdjustment[axis] = 0;
        }

        // Now calculate pitch/roll adjustments to achieve desired velocities
        if (navShouldApplyPosHold()) {
            // Calculate pitch/roll
            for (axis = 0; axis < 2; axis++) {
                float error = desiredAircraftVel[axis] - actualAircraftVel[axis];
                error = constrainf(error, -500, 500); // limit error to 5 m/s

                rcAdjustment[axis] = pidGetP(error, &posholdRatePID[axis]) +
                                     pidGetI(error, dTnav, &posholdRatePID[axis]);

                float d = pidGetD(error, dTnav, &posholdRatePID[axis]);
                d = constrainf(d, -2000, 2000);

                // get rid of noise
                if (ABS(actualHorizontalVelocity[axis]) < 10)
                    d = 0;

                rcAdjustment[axis] += d;

                rcAdjustment[axis] = constrain(rcAdjustment[axis] / 10, -NAV_ROLL_PITCH_MAX, NAV_ROLL_PITCH_MAX);
                navigationRatePID[axis].integrator = posholdRatePID[axis].integrator;
            }
        }
        else if (navShouldApplyWaypoint()) {
            // Calculate pitch/roll
            for (axis = 0; axis < 2; axis++) {
                float error = desiredAircraftVel[axis] - actualAircraftVel[axis];
                error = constrainf(error, -500, 500); // limit error to 5 m/s

                rcAdjustment[axis] = pidGetPID(error, dTnav, &navigationRatePID[axis]);
                rcAdjustment[axis] = constrain(rcAdjustment[axis] / 10, -NAV_ROLL_PITCH_MAX, NAV_ROLL_PITCH_MAX);
            }
        }

        // Calculate yaw correction
        if (navShouldApplyHeadingControl()) {
            int32_t headingError = wrap_18000((actualPosition.heading - desiredHeading) * 100);
            headingError *= masterConfig.yaw_control_direction;

            headingError = constrain(headingError, -3000, +3000); // limit error to +- 30 degrees to avoid fast rotation

            // FIXME: SMALL_ANGLE might prevent NAV from adjusting yaw when banking is too high (i.e. nav in high wind)
            if (STATE(SMALL_ANGLE)) {
                // Heading PID controller takes degrees, not centidegrees (this pid duplicates MAGHOLD)
                rcAdjustment[YAW] = pidGetP(headingError / 100.0f, &headingRatePID);
            }
        }
    }
}

static void calculateThrottleAdjustment(float dTnav)
{
    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        // Calculate throttle adjustment for althold
        if (navShouldApplyAltHold()) {
            if (navIsThrustFacingDownwards(&inclination)) {
                float error = desiredVerticalVel - actualVerticalVelocity;

                /* TODO: Tilt compensation */
                rcAdjustment[THROTTLE] = pidGetPID(error, dTnav, &altitudeRatePID);
                rcAdjustment[THROTTLE] = constrain(rcAdjustment[THROTTLE], -500, 500);
            }
        }
    }
}

/*-----------------------------------------------------------
 * NAV throttle PID controllers
 *-----------------------------------------------------------*/
static void setAltHoldInitialThrottle(int16_t throttle)
{
    // We can do althold two ways:
    //  1. CF-style, when FC remembers the throttle when hold was initiated and detects stick movement from that point
    //  2. NAZA-style, when throttle stick is centered when not used
    if (navProfile->nav_use_midrc_for_althold)
        altholdInitialThrottle = masterConfig.rxConfig.midrc;
    else
        altholdInitialThrottle = throttle;
}

static void adjustVerticalVelocityFromRCInput(void)
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustVerticalVelocityFromRCInput())
        return;

    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        if (ABS(rcData[THROTTLE] - altholdInitialThrottle) > rcControlsConfig->alt_hold_deadband) {
            // set velocity proportional to stick movement
            desiredVerticalVel = (rcData[THROTTLE] - altholdInitialThrottle) * navProfile->nav_manual_speed_vertical / 500.0f;

            // We are changing altitude, apply new altitude hold setpoint
            activeWpOrHoldPosition.altitude = actualPosition.altitude;
        }
    }
}

/*-----------------------------------------------------------
 * NAV pilot's adjustments to attitude/throttle
 *-----------------------------------------------------------*/
static void adjustHorizontalVelocityFromRCInput()
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustHorizontalVelocityFromRCInput())
        return;

    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        if (ABS(rcCommand[ROLL]) >= navProfile->nav_rc_deadband || ABS(rcCommand[PITCH]) >= navProfile->nav_rc_deadband) {
            // Calculate desired velocities according to stick movement (copter frame of reference)
            float rcVelX = (rcCommand[PITCH] - masterConfig.rxConfig.midrc) * navProfile->nav_manual_speed_horizontal / 500.0f;
            float rcVelY = (rcCommand[ROLL] - masterConfig.rxConfig.midrc) * navProfile->nav_manual_speed_horizontal / 500.0f;

            // Rotate these velocities from body frame to to earth frame
            float sin_yaw_y = sinf(actualPosition.heading * RADX100);
            float cos_yaw_x = cosf(actualPosition.heading * RADX100);

            desiredHorizontalVel[X] = rcVelX * cos_yaw_x - rcVelY * sin_yaw_y;
            desiredHorizontalVel[Y] = rcVelX * sin_yaw_y + rcVelY * cos_yaw_x;

            // We are in position hold mode, so adjust poshold setpoint
            // Regardless of 2D or 3D mode we are only adjusting horizontal position
            activeWpOrHoldPosition.coordinates[LAT] = actualPosition.coordinates[LAT];
            activeWpOrHoldPosition.coordinates[LON] = actualPosition.coordinates[LON];
        }
    }
}

static void adjustHeadingFromRCInput()
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustHeadingFromRCInput())
        return;

    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        // Passthrough yaw input if stick is moved
        if (ABS(rcCommand[YAW]) >= navProfile->nav_rc_deadband) {
            rcAdjustment[YAW] = rcCommand[YAW];

            // Pilot is adjusting heading so update heading lock
            activeWpOrHoldPosition.heading = actualPosition.heading;
        }
    }
}

/*-----------------------------------------------------------
 * NAV updates
 *-----------------------------------------------------------*/
void applyWaypointNavigationAndAltitudeHold(void)
{
    static uint32_t previousTime;
    uint32_t currentTime = micros();
    float dTnav = (currentTime - previousTime) / 1e6;

    previousTime = currentTime;

    // Shouldn't apply navigation corrections when not armed
    if (!ARMING_FLAG(ARMED)) {
        return;
    }
    
    // If throttle low don't apply navigation either 
    // TODO
    
    // Set rcCommandNav to rcCommand (passthrough control)
    rcCommandNav[PITCH] = rcCommand[PITCH];
    rcCommandNav[ROLL] = rcCommand[ROLL];
    rcCommandNav[YAW] = rcCommand[YAW];
    rcCommandNav[THROTTLE] = rcCommand[THROTTLE];

    // Apply navigation adjustments
    if (STATE(FIXED_WING)) { // FIXED_WING
        // TODO
    }
    else { // MULTIROTOR
        // We do adjustments NAZA-style and think for pilot. In NAV mode pilot does not control the THROTTLE, PITCH and ROLL angles/rates directly,
        // except for a few navigation modes. Instead of that pilot controls velocities in 3D space.
        //      In RTH and WP pilot actually controls NOTHING but flight mode!
        if (navShouldApplyAltHold()) {
            // Calculate desired vertical velocity & throttle adjustment
            calculateDesiredVerticalVelocity(&actualPosition, &activeWpOrHoldPosition, dTnav);
            adjustVerticalVelocityFromRCInput();
            calculateThrottleAdjustment(dTnav);

            // Apply rcAdjustment to throttle
            rcCommandNav[THROTTLE] = constrain(altholdInitialThrottle + rcAdjustment[THROTTLE], masterConfig.escAndServoConfig.minthrottle, masterConfig.escAndServoConfig.maxthrottle);
        }

        if (navShouldApplyPosHold() || navShouldApplyWaypoint() || navShouldApplyHeadingControl()) {
            // Calculate PH/RTH/WP and attritude adjustment
            if (navShouldApplyPosHold() || navShouldApplyWaypoint()) {
                calculateDesiredHorizontalVelocity(&actualPosition, &activeWpOrHoldPosition, dTnav);
                adjustHorizontalVelocityFromRCInput();
            }

            // Apply rcAdjustment to yaw
            if (navShouldApplyHeadingControl()) {
                calculateDesiredHeading(&actualPosition, &activeWpOrHoldPosition, dTnav);
                adjustHeadingFromRCInput();
            }

            // Now correct desired velocities and heading to attitude corrections
            calculateAttitudeAdjustment(dTnav);

            // Apply rcAdjustment to pitch/roll
            if (navShouldApplyPosHold() || navShouldApplyWaypoint()) {
                rcCommandNav[PITCH] = constrain(rcAdjustment[PITCH], -NAV_ROLL_PITCH_MAX, NAV_ROLL_PITCH_MAX);
                rcCommandNav[ROLL] = constrain(rcAdjustment[ROLL], -NAV_ROLL_PITCH_MAX, NAV_ROLL_PITCH_MAX);
            }

            if (navShouldApplyHeadingControl()) {
                rcCommandNav[YAW] = constrain(rcAdjustment[YAW], -500, 500);
            }
        }
    }

/*    
    debug[0] = actualPosition.coordinates[LAT] - activeWpOrHoldPosition.coordinates[LAT];
    debug[1] = desiredHorizontalVel[Y];
    debug[2] = actualHorizontalVelocity[Y];
    debug[3] = rcCommandNav[PITCH];
*/
    
    //debug[0] = rcCommandNav[THROTTLE];
    //debug[2] = desiredHorizontalVel[LON];
    
}

/*-----------------------------------------------------------
 * NAV land detector
 *-----------------------------------------------------------*/
static bool isLandingDetected(void)
{
    static uint32_t landingConditionsNotSatisfiedTime;
    bool landingConditionsSatisfied = true;
    uint32_t currentTime = micros();

    // land detector can not use the following sensors because they are unreliable during landing
    // calculated vertical velocity or altitude : poor barometer and large acceleration from ground impact, ground effect
    // earth frame angle or angle error :         landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                              on uneven surface the airframe may rock back an forth after landing
    // input throttle :                           in slow land the input throttle may be only slightly less than hover

    // TODO

    // Currently the only condition verified is throttle - should be less than 25%
    if (rcCommand[THROTTLE] >= (masterConfig.escAndServoConfig.minthrottle + (masterConfig.escAndServoConfig.maxthrottle - masterConfig.escAndServoConfig.minthrottle) / 4)) {
        landingConditionsSatisfied = false;
    }

    if (landingConditionsSatisfied) {
        if ((currentTime - landingConditionsNotSatisfiedTime) > LANDING_DETECTION_TIMEOUT) {
            return true;
        }
    }
    else {
        landingConditionsNotSatisfiedTime = currentTime;
    }

    return false;
}

/*-----------------------------------------------------------
 * NAV mode updates
 *-----------------------------------------------------------*/
void swithNavigationFlightModes(navigationMode_e navMode)
{
    switch(navMode) {
        case NAV_MODE_ALTHOLD:
            ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
            DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE | NAV_RTH_MODE | NAV_WP_MODE);
            break;
        case NAV_MODE_POSHOLD_2D:
            ENABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
            DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE | NAV_RTH_MODE | NAV_WP_MODE);
            break;
        case NAV_MODE_POSHOLD_3D:
            ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE);
            DISABLE_FLIGHT_MODE(NAV_RTH_MODE | NAV_WP_MODE);
            break;
        case NAV_MODE_WP:
            ENABLE_FLIGHT_MODE(NAV_WP_MODE);
            DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE | NAV_RTH_MODE);
            break;
        case NAV_MODE_RTH:
            ENABLE_FLIGHT_MODE(NAV_RTH_MODE);
            DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE | NAV_WP_MODE);
            break;
        case NAV_MODE_NONE:
        default:
            DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE | NAV_RTH_MODE | NAV_WP_MODE);
            break;
    }
}

bool naivationRequiresAngleMode(void)
{
    return (navMode != NAV_MODE_NONE) && (navMode != NAV_MODE_ALTHOLD);
}

bool naivationControlsHeadingNow(void)
{
    return (navMode != NAV_MODE_NONE) && (navMode != NAV_MODE_ALTHOLD);
}

void updateWaypointsAndNavigationMode(void)
{
    // NOTE: compatibility with current BOXMODES, no support for all NAV-specific mode
    navigationMode_e newNavMode = NAV_MODE_NONE;

    // Flags if we can activate certain nav modes (check if we have required sensors and they provide valid data)
    bool canActivatePosHold = sensors(SENSOR_ACC) && (sensors(SENSOR_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5) && sensors(SENSOR_MAG);
    bool canActivateAltHold = sensors(SENSOR_BARO) || sensors(SENSOR_SONAR);
    bool canActivateRTHOrWP = canActivatePosHold && canActivateAltHold;

    // Figure out, what mode pilot want to activate, also check if it is possible
    if (IS_RC_MODE_ACTIVE(BOXNAVRTH) && canActivateRTHOrWP && STATE(GPS_FIX_HOME)) {
        newNavMode = NAV_MODE_RTH;
    }
    else if (IS_RC_MODE_ACTIVE(BOXNAVWP) && canActivateRTHOrWP) {
        newNavMode = NAV_MODE_WP;
    }
    else if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD) && IS_RC_MODE_ACTIVE(BOXNAVALTHOLD) && canActivatePosHold && canActivateAltHold) {
        newNavMode = NAV_MODE_POSHOLD_3D;
    }
    else if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD) && canActivatePosHold) {
        newNavMode = NAV_MODE_POSHOLD_2D;
    }
    else if (IS_RC_MODE_ACTIVE(BOXNAVALTHOLD) && canActivateAltHold) {
        newNavMode = NAV_MODE_ALTHOLD;
    }
    else {
        newNavMode = NAV_MODE_NONE;
    }

    // Process mode transition
    if (newNavMode != navMode) {
        if (navMode == NAV_MODE_NONE) {
            resetNavigation();
        }

        switch (newNavMode) {
            case NAV_MODE_ALTHOLD:
                // Set altitude hold data
                setNextWaypointAndHeadingLock(actualPosition.coordinates[LAT], actualPosition.coordinates[LON], actualPosition.altitude, actualPosition.heading);
                setAltHoldInitialThrottle(rcData[THROTTLE]);
                navMode = NAV_MODE_ALTHOLD;
                break;
            case NAV_MODE_POSHOLD_2D:
                setNextWaypointAndHeadingLock(actualPosition.coordinates[LAT], actualPosition.coordinates[LON], actualPosition.altitude, actualPosition.heading);
                navMode = NAV_MODE_POSHOLD_2D;
                break;
            case NAV_MODE_POSHOLD_3D:
                setNextWaypointAndHeadingLock(actualPosition.coordinates[LAT], actualPosition.coordinates[LON], actualPosition.altitude, actualPosition.heading);
                setAltHoldInitialThrottle(rcData[THROTTLE]);
                navMode = NAV_MODE_POSHOLD_3D;
                break;
            case NAV_MODE_WP:
                // TODO
                navMode = NAV_MODE_NONE;
                break;
            case NAV_MODE_RTH:
                // We fix @ current position and climb to safe altitude
                setNextWaypointAndCalculateBearing(actualPosition.coordinates[LAT], actualPosition.coordinates[LON], actualPosition.altitude);
                navMode = NAV_MODE_RTH;
                rthState = NAV_RTH_STATE_INIT;
                break;
            default: // NAV_MODE_NONE
                navMode = newNavMode;
                break;
        }
    }

    swithNavigationFlightModes(navMode);

    // Process RTH state machine
    // FIXME: Multirotor only, fixed-wing logic must be different
    if (navMode == NAV_MODE_RTH) {
        switch (rthState) {
            case NAV_RTH_STATE_INIT:
                if (distanceToHome < navProfile->nav_min_rth_distance) {
                    // Prevent RTH jump in your face, when arming copter accidentally activating RTH (or RTH on failsafe)
                    // Inspired by CrashPilot1000's TestCode3
                    resetHomePosition();
                    rthState = NAV_RTH_STATE_HOME_AUTOLAND;
                }
                else {
                    // Climb to safe altitude if needed
                    if (actualPosition.altitude <= 1000) {
                        setNextWaypointAndHeadingLock(actualPosition.coordinates[LAT], actualPosition.coordinates[LON], 1000 + 50.0f, actualPosition.heading);
                    }
                    rthState = NAV_RTH_STATE_CLIMB_TO_SAVE_ALTITUDE;
                }
                break;
            case NAV_RTH_STATE_CLIMB_TO_SAVE_ALTITUDE:
                if (actualPosition.altitude > 1000) {
                    setNextWaypointAndCalculateBearing(homePosition.coordinates[LAT], homePosition.coordinates[LON], actualPosition.altitude);
                    rthState = NAV_RTH_STATE_HEAD_HOME;
                }
                break;
            case NAV_RTH_STATE_HEAD_HOME:
                // Stay at this state until home reached
                if (navIsWaypointReached(&actualPosition, &homePosition)) {
                    rthState = NAV_RTH_STATE_HOME_AUTOLAND;
                }
                break;
            case NAV_RTH_STATE_HOME_AUTOLAND:
                if (isLandingDetected() || !ARMING_FLAG(ARMED)) {
                    rthState = NAV_RTH_STATE_LANDED;
                }
                else {
                    // Gradually reduce descent speed depending on actual altitude. Descent from 20m should take about 50 seconds with default PIDs
                    if (actualPosition.altitude > 1000) {
                        // Fast descent (altitude setpoint 2.5m below actual altitude, about 1m/s descent at default PIDs)
                        setNextWaypointAndHeadingLock(homePosition.coordinates[LAT], homePosition.coordinates[LON], actualPosition.altitude - 250.0f, homePosition.heading);
                    }
                    else if (actualPosition.altitude > 250) {
                        // Medium descent (altitude setpoint 1.3m below actual altitude, about 50 cm/s descent at default PIDs)
                        setNextWaypointAndHeadingLock(homePosition.coordinates[LAT], homePosition.coordinates[LON], actualPosition.altitude - 130.0f, homePosition.heading);
                    }
                    else {
                        // Slow descent (altitude setpoint 26cm below actual altitude, about 10 cm/s descent at default PIDs)
                        setNextWaypointAndHeadingLock(homePosition.coordinates[LAT], homePosition.coordinates[LON], actualPosition.altitude - 26.0f, homePosition.heading);
                    }
                }
                break;
            case NAV_RTH_STATE_LANDED:
                // RTH is a non-normal flight mode. Engaging RTH likely means that pilot cannot or don't want to control aircraft.
                // Craft in RTH mode should return home, land, disarm and lock out rearming to prevent accidental takeoff
                ENABLE_ARMING_FLAG(PREVENT_ARMING);
                mwDisarm();

                rthState = NAV_RTH_STATE_FINISHED;
                break;
            case NAV_RTH_STATE_FINISHED:
                // Stay in this state forever
                break;
        }
    }
}

/*-----------------------------------------------------------
 * NAV main control functions
 *-----------------------------------------------------------*/
void navigationUseProfile(navProfile_t *navProfileToUse)
{
    navProfile = navProfileToUse;
}

void navigationUseBarometerConfig(barometerConfig_t * intialBarometerConfig)
{
    barometerConfig = intialBarometerConfig;
}

#define POSHOLD_IMAX           20       // degrees
#define POSHOLD_VEL_IMAX       20       // degrees
#define NAV_VEL_IMAX           20       // degrees
void navigationUsePIDs(pidProfile_t *pidProfile)
{
    int axis;

    // Initialize position hold PI-controller
    pidInit(&positionPID, (float)pidProfile->P8[PIDPOS] / 100.0f,
                          (float)pidProfile->I8[PIDPOS] / 100.0f,
                          0,
                          POSHOLD_IMAX * 100.0);

    // Initialize altitude hold P-controller
    pidInit(&altitudePID, (float)pidProfile->P8[PIDALT] / 128.0f, 0, 0, 0);

    // Initialize vertical velocity PID-controller
    pidInit(&altitudeRatePID, (float)pidProfile->P8[PIDVEL] / 32.0f,
                              (float)pidProfile->I8[PIDVEL] / 8192.0f,
                              (float)pidProfile->D8[PIDVEL] / 256.0f,
                              200.0);

    // Initialize horizontal velocity PID-controllers
    for (axis = 0; axis < 2; axis++) {
        pidInit(&navigationRatePID[axis], (float)pidProfile->P8[PIDNAVR] / 10.0f,
                                          (float)pidProfile->I8[PIDNAVR] / 100.0f,
                                          (float)pidProfile->D8[PIDNAVR] / 1000.0f,
                                          NAV_VEL_IMAX * 100.0);

        pidInit(&posholdRatePID[axis], (float)pidProfile->P8[PIDPOSR] / 10.0f,
                                           (float)pidProfile->I8[PIDPOSR] / 100.0f,
                                           (float)pidProfile->D8[PIDPOSR] / 1000.0f,
                                           NAV_VEL_IMAX * 100.0);
    }

    // Heading PID (duplicates maghold)
    pidInit(&headingRatePID, (float)pidProfile->P8[PIDMAG] / 30.0f, 0, 0, 0);
}

void navigationInit(navProfile_t *initialNavProfile,
                    pidProfile_t *initialPidProfile,
                    barometerConfig_t *intialBarometerConfig,
                    rcControlsConfig_t *initialRcControlsConfig)
{
    navigationUseProfile(initialNavProfile);
    navigationUsePIDs(initialPidProfile);
    navigationUseBarometerConfig(intialBarometerConfig);

    /* Save configuration for future use */
    rcControlsConfig = initialRcControlsConfig;
}

/*-----------------------------------------------------------
 * NAV data collection and pre-processing code
 * This is the largest sensor-dependent part of nav-rewrite.
 * Adding new sensors, implementing EKF, etc. should modify
 * this part of code and do not touch the above code (if possible)
 *-----------------------------------------------------------*/
void onNewGPSData(int32_t newLat, int32_t newLon)
{
    static uint32_t previousTime;
    static bool isFirstUpdate = true;
    static int32_t previousLat;
    static int32_t previousLon;

    // Don't have a valid GPS 3D fix, do nothing
    if (!(STATE(GPS_FIX) && GPS_numSat >= 5)) {
        return;
    }

    uint32_t currentTime = micros();

    // If not first update - calculate velocities
    if (!isFirstUpdate) {
        float dT = (currentTime - previousTime) * 1e-6;

        // this is used to offset the shrinking longitude as we go towards the poles
        gpsScaleLonDown = cosf((ABS(newLat) / 10000000.0f) * 0.0174532925f);

        // Calculate velocities based on GPS coordinates change
        float gpsVelocityX = gpsScaleLonDown * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (newLon - previousLon) / dT;
        float gpsVelocityY = DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (newLat - previousLat) / dT;

        // Update IMU velocities with complementary filter to keep them close to real velocities (as given by GPS)
        imuApplyFilterToActualVelocity(X, navProfile->gps_cf_vel, gpsVelocityX);
        imuApplyFilterToActualVelocity(Y, navProfile->gps_cf_vel, gpsVelocityY);
    }

    // Set previous coordinates
    previousLat = newLat;
    previousLon = newLon;
    isFirstUpdate = false;

    previousTime = currentTime;

    // Update NAV's position (from GPS) and velocity (from IMU)
    updateActualHorizontalPosition(newLat, newLon);
    updateActualHorizontalVelocity(imuAverageVelocity[Y], imuAverageVelocity[X]);
    

    // Handle update of home position
    updateHomePosition();
}

void updateEstimatedPositionFromIMU(void)
{
    // TODO: calculate estimated position based on IMU

    // IMU velocity updates at more rate than GPS, update velocities only
    updateActualHorizontalVelocity(imuAverageVelocity[Y], imuAverageVelocity[X]);
}

void updateEstimatedHeading(void)
{
    // NAV uses heading in centidegrees
    updateActualHeading((int32_t)heading * 100);
}

// max 40hz update rate (20hz LPF on acc), seconds
#define BARO_UPDATE_FREQUENCY_40HZ (1.0f / 25)

// TODO: this is mostly ported from CF's original althold code, need cleaning up
void updateEstimatedAltitude(void)
{
    static uint32_t previousTime = 0;
    int32_t sonarAlt = -1;
    float sonarTransition;
    static int32_t baroAlt_offset = 0;
    int32_t baroVel;
    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

#ifdef SONAR
    int16_t tiltAngle;
#endif

    // If we have baro and it is not ready - skip update
    if (sensors(SENSOR_BARO) && !isBaroReady())
        return;

    // We currently can use only BARO and SONAR as sources of altitude
    if (!(sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)))
        return;

    uint32_t currentTime = micros();
    float dT = (currentTime - previousTime) * 1e-6;

    // too fast, likely no new data available
    if (dT < BARO_UPDATE_FREQUENCY_40HZ)
        return;

    previousTime = currentTime;

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        accAlt = 0;
    }
    BaroAlt = baroCalculateAltitude();
#else
    BaroAlt = 0;
#endif

#ifdef SONAR
    tiltAngle = calculateTiltAngle(&inclination);
    sonarAlt = sonarRead();
    sonarAlt = sonarCalculateAltitude(sonarAlt, tiltAngle);
#endif

    // Use sonar up to 2/3 of maximum range, smoothly transit to baro if upper 1/3 sonar range
    if (sonarAlt > 0 && sonarAlt < (SONAR_MAX_RANGE * 2 / 3)) {
        baroAlt_offset = BaroAlt - sonarAlt;
        BaroAlt = sonarAlt;
    } else {
        BaroAlt -= baroAlt_offset;
        if (sonarAlt > (SONAR_MAX_RANGE * 2 / 3) && sonarAlt < SONAR_MAX_RANGE) {
            sonarTransition = (SONAR_MAX_RANGE - sonarAlt) / (SONAR_MAX_RANGE / 3);
            BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
        }
    }

    // Integrator - Altitude in cm
    accAlt += (0.5f * imuAverageAcceleration[Z] * dT + imuAverageVelocity[Z]) * dT;  //  a * t^2 / 2 + v * t
    accAlt = accAlt * barometerConfig->baro_cf_alt + (float)BaroAlt * (1.0f - barometerConfig->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        return;
    }
#endif

    if (sonarAlt > 0 && sonarAlt < (SONAR_MAX_RANGE * 2 / 3)) {
        // the sonar has the best range
        updateActualAltitudeAndVelocity(BaroAlt, imuAverageVelocity[Z]);
    } else {
        updateActualAltitudeAndVelocity(accAlt, imuAverageVelocity[Z]);
    }

    baroVel = (BaroAlt - lastBaroAlt) / dT;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    imuApplyFilterToActualVelocity(Z, barometerConfig->baro_cf_vel, baroVel);
}
