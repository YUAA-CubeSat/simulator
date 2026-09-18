/* Example C ABI controller: elementary B-dot detumbling and boom deployment. */
#include <stddef.h>

static double previous_mag[3], current_mag[3], previous_time, current_time;
static int have_current, have_previous;
static double sun_body[3];
static int sun_valid;

void adcs_magnetometer(double time_s, const double mag1_T[3], const double mag2_T[3])
{
    (void)mag2_T; /* Second sensor uses a rotated frame in the template. */
    if (have_current) {
        for (int i = 0; i < 3; ++i) previous_mag[i] = current_mag[i];
        previous_time = current_time;
        have_previous = 1;
    }
    for (int i = 0; i < 3; ++i) current_mag[i] = mag1_T[i];
    current_time = time_s;
    have_current = 1;
}

void adcs_sun(double time_s, const double vector_body[3], int valid)
{
    (void)time_s;
    sun_valid = valid;
    for (int i = 0; i < 3; ++i) sun_body[i] = vector_body[i];
}

int adcs_step(double time_s, double dt_s, double dipole_Am2[3], double *boom_length_m)
{
    (void)time_s;
    (void)dt_s;
    (void)sun_valid;
    (void)sun_body;
    if (!dipole_Am2 || !boom_length_m) return 1;
    for (int i = 0; i < 3; ++i) {
        double command = 0.0;
        if (have_previous && current_time > previous_time)
            command = -100000.0 * (current_mag[i] - previous_mag[i]) / (current_time - previous_time);
        if (command > 0.2) command = 0.2;
        if (command < -0.2) command = -0.2;
        dipole_Am2[i] = command;
    }
    *boom_length_m = 0.5;
    return 0;
}
