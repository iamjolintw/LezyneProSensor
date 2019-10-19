/*
 * 	analog.h
 *
 *  Created on: October 3, 2019
 *  Author: Lezyne
 */

#ifndef ANALOG_H
#define ANALOG_H

#define PERCENT_OVER_90  	774	// > 2.72v
#define PERCENT_OVER_80		759	// > 2.67v
#define PERCENT_OVER_70  	740	// > 2.60v
#define PERCENT_OVER_60  	725	// > 2.55v
#define PERCENT_OVER_50  	703	// > 2.47v
#define PERCENT_OVER_40  	680	// > 2.39v
#define PERCENT_OVER_30  	646	// > 2.27v
#define PERCENT_OVER_20  	606	// > 2.13v
#define PERCENT_OVER_10  	558	// > 1.96v
#define PERCENT_0   		484	// > 1.70v


/* FUNCTION */
void saadc_init(void);
void bat_init(void);
void bat_sleep(void);

#endif // ANALOG_H

