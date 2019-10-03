/*
 * 	analog.h
 *
 *  Created on: October 3, 2019
 *  Author: Lezyne
 */

#ifndef ANALOG_H
#define ANALOG_H

#define PERCENT_OVER_90 	600	 // > 2.9v
#define PERCENT_OVER_80 	598  // > 2.89v
#define PERCENT_OVER_70 	596  // > 2.88v
#define PERCENT_OVER_60 	593  // > 2.87v
#define PERCENT_OVER_50 	590  // > 2.85v
#define PERCENT_OVER_40 	585  // > 2.83v
#define PERCENT_OVER_30 	579  // > 2.80v
#define PERCENT_OVER_20 	559  // > 2.70v
#define PERCENT_OVER_10 	517  // > 2.50v
#define PERCENT_0			434  // > 2.10v


/* FUNCTION */
void saadc_init(void);
void bat_init(void);
void bat_sleep(void);

#endif // ANALOG_H

