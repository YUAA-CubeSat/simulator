/* Example stdin/stdout controller: elementary B-dot detumbling and boom deployment. */
#include <stddef.h>
#include <stdio.h>

static double previous_mag[3], current_mag[3], previous_time, current_time;
static int have_current, have_previous;
static double sun_body[3];
static int sun_valid;
static double battery_soc, battery_power_W;

void adcs_battery(double time_s, double soc, double power_W)
{
    (void)time_s;
    battery_soc = soc;
    battery_power_W = power_W;
}

void adcs_magnetometer(double time_s, int sensor_id, const double mag_T[3])
{
    /* Use sensor 0 for B-dot; other sensors may use different frames. */
    if (sensor_id != 0) return;
    if (have_current) {
        for (int i = 0; i < 3; ++i) previous_mag[i] = current_mag[i];
        previous_time = current_time;
        have_previous = 1;
    }
    for (int i = 0; i < 3; ++i) current_mag[i] = mag_T[i];
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
    /* Available for power-aware control; this example keeps B-dot unchanged. */
    (void)battery_soc;
    (void)battery_power_W;
    if (!dipole_Am2 || !boom_length_m) return 1;
    for (int i = 0; i < 3; ++i) {
        double command = 0.0;
        if (have_previous && current_time > previous_time)
            command = -100000.0 * (current_mag[i] - previous_mag[i]) / (current_time - previous_time);
        if (command > 0.2) command = 0.2;
        if (command < -0.2) command = -0.2;
        dipole_Am2[i] = command;
    }
    *boom_length_m = 0.0;
    return 0;
}

/* Same integer line protocol as the HITL firmware. stdout is protocol only. */
int main(void)
{
    char line[256];
    while (fgets(line, sizeof line, stdin)) {
        long long time_ms, tick, period_ms, values[3];
        int valid, sensor_id;
        if (line[0] == 'M' && sscanf(line, "M %lld %d %lld %lld %lld",
                &time_ms, &sensor_id, &values[0], &values[1], &values[2]) == 5
                && sensor_id >= 0) {
            double mag[3];
            for (int i = 0; i < 3; ++i) mag[i] = values[i] * 1e-9;
            adcs_magnetometer(time_ms / 1000.0, sensor_id, mag);
        } else if (line[0] == 'S' && sscanf(line, "S %lld %d %lld %lld %lld",
                &time_ms, &valid, &values[0], &values[1], &values[2]) == 5) {
            double vector[3];
            for (int i = 0; i < 3; ++i) vector[i] = values[i] / 32767.0;
            adcs_sun(time_ms / 1000.0, vector, valid);
        } else if (line[0] == 'B' && sscanf(line, "B %lld %lld %lld",
                &time_ms, &values[0], &values[1]) == 3
                && values[0] >= 0 && values[0] <= 1000) {
            adcs_battery(time_ms / 1000.0, values[0] / 1000.0, values[1] / 1000.0);
        } else if (line[0] == 'T' && sscanf(line, "T %lld %lld %lld",
                &tick, &time_ms, &period_ms) == 3) {
            double dipole[3], boom;
            if (adcs_step(time_ms / 1000.0, period_ms / 1000.0, dipole, &boom)) return 1;
            printf("A %lld %.0f %.0f %.0f %.0f\n", tick,
                   dipole[0] * 1e6, dipole[1] * 1e6, dipole[2] * 1e6, boom * 1000);
            if (fflush(stdout) == EOF) return 1;
        } else {
            fprintf(stderr, "Invalid simulator message: %s", line);
            return 1;
        }
    }
    return ferror(stdin) ? 1 : 0;
}
