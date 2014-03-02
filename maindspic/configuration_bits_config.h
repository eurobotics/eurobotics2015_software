/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */

#ifndef CONFIGURATION_BITS_CONFIG_H
#define CONFIGURATION_BITS_CONFIG_H

#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#endif 

#if defined(__dsPIC33F__)

/* FBS: Boot Code Segment Configuration Register */
//_FBS(FBS_CONFIG);

/* FSS: Secure Code Segment Configuration Register */
//_FSS(FSS_CONFIG)

/* FGS: General Code Segment Configuration Register */
//_FGS(FGS_CONFIG);

/* FOSCSEL: Oscillator Source Selection Register */
// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC)

/* FOSC: Oscillator Configuration Register */
// Enable Clock Switching and Configure Posc in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT)

/* FWDT: Watchdog Timer (WDT) Configuration Register */
// Disable Watchdog Timer
_FWDT(FWDTEN_OFF);


/* FPOR: POR Configuration Register */
_FPOR(ALTI2C_ON);

/* FICD: In-Circuit Debugger Configuration Register */
//_FICD();

/* Unit ID Field */
//_FUID1();
//_FUID2();
//_FUID3();


#else
#error No device supported in "configuration_bits_config.h"
#endif

#endif /* CONFIGURATION_BITS_CONFIG_H */

/* EOF */
