#pragma once

// STANDARD LIBRARIES
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// BASE STM32 HEADER
#include "stm32f4xx.h"
#include "stm32f401xe.h"
#include "stm32f4xx_hal.h"
// HAL DRIVERS
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"

// INTERRUPTS
#include "stm32f4xx_it.h"

// You can declare any other shared functions or globals here

#define PSM_TO_I(psm) ((int)((psm - 1.0) * 40.0))

typedef enum {
    CO2,
    CO5,
    CO6,
    CO7,
    CO8,
    CO9,
    CO11
} Curves;

typedef struct {
    double time_delay;
    double current_pickup;
    Curves type;
    double direction_angle;
}relayType;

typedef struct {
    double constT;
    double constK;
    double constC;
    uint8_t constP;
    double constR;
}constTable;

typedef struct {
    double real;
    double img;
} complexNum;

static void SystemClock_Config(void);

void adc_init(void);

void TableSetup(constTable *mytable);

double getTime( constTable *currTable, relayType *curRelay, double current_PSM);

complexNum getFiltered(float *adc_data, float *cos_table, float *sin_table);

double map(double x, double in_min, double in_max, double out_min, double out_max);

double getRMSquared(complexNum current_fund);

void quickTrip();

void buildProgress(double *progress, constTable *calTable, relayType *calRelay);

void relay_init(void);

void pll_init(void);

void timer_init(void);

void setupTrig(float *cos_table, float *sin_table);

void indicator_init(void);

void quickWalk();
