/* Example stdin/stdout controller: elementary B-dot detumbling and boom deployment. */
#include <stddef.h>
#include <stdio.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include "../protocol/adcs_wire.h"

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

void adcs_accelerometer(double time_s, int sensor_id, const double accel_m_s2[3])
{
    /* Hook for acceleration-aware control; readings are in each sensor's frame. */
    (void)time_s;
    (void)sensor_id;
    (void)accel_m_s2;
}

void adcs_gyroscope(double time_s, int sensor_id, const double rate_rad_s[3])
{
    /* Hook for rate-aware control; IDs 0/1 have independent sensor frames. */
    (void)time_s;
    (void)sensor_id;
    (void)rate_rad_s;
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

/* Binary transport; stdout is protocol only. */
int main(void)
{
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    WireDecoder decoder={0}; WireFrame f; int byte;
    while((byte=getchar())!=EOF) {
        if(Wire_Feed(&decoder,(uint8_t)byte,&f)!=1) continue;
        const uint8_t *p=f.payload; double t=Wire_Get(p,4)/1000.0;
        double v[3];
        if(f.type==WIRE_M || f.type==WIRE_C) {
            unsigned width=f.type==WIRE_M?3:4;
            for(unsigned i=0;i<3;++i) v[i]=Wire_Signed(p+5+i*width,width)*(f.type==WIRE_M?1e-9:1e-6);
            if(f.type==WIRE_M) adcs_magnetometer(t,p[4],v);
            else adcs_accelerometer(t,p[4],v);
        } else if(f.type==WIRE_G) {
            unsigned offset=f.length==14?5:4;
            unsigned sensor_id=f.length==14?p[4]:0;
            if(sensor_id>1) continue;
            for(unsigned i=0;i<3;++i) v[i]=Wire_Signed(p+offset+i*3,3)*1e-6;
            adcs_gyroscope(t,(int)sensor_id,v);
        } else if(f.type==WIRE_S) {
            for(unsigned i=0;i<3;++i) v[i]=Wire_Signed(p+5+i*2,2)/32767.0;
            adcs_sun(t,v,p[4]);
        } else if(f.type==WIRE_B) adcs_battery(t,Wire_Get(p+4,2)/1000.0,Wire_Signed(p+6,4)/1000.0);
        else if(f.type==WIRE_T) {
            double dipole[3],boom;
            if(adcs_step(Wire_Get(p+4,4)/1000.0,Wire_Get(p+8,2)/1000.0,dipole,&boom)) return 1;
            uint8_t payload[29]={0},output[WIRE_MAX];
            Wire_Put(payload,Wire_Get(p,4),4);
            for(unsigned i=0;i<3;++i) Wire_Put(payload+4+i*3,(uint32_t)(int32_t)(dipole[i]*1e6),3);
            Wire_Put(payload+13,(uint32_t)(boom*1000),2);
            size_t n=Wire_Encode(output,sizeof output,WIRE_A,payload);
            if(!n || fwrite(output,1,n,stdout)!=n || fflush(stdout)==EOF) return 1;
        }
    }
    return ferror(stdin)?1:0;
}
