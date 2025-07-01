#include "main.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


void can_filter_init(void);
void can1_four( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006);
void can1_two( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006);
void can2_four( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006);
void can2_two( int16_t D2006, int16_t L2006, int16_t R2006,int16_t S2006);



