/*
 * AS5048A.h
 *
 *  Created on: 2017/12/27
 *      Author: kusakabe
 */

#ifndef AS5048A_H_
#define AS5048A_H_

#ifdef __cplusplus
 extern "C" {
#endif

// AS4048A Register Addresses

/** Control and Error Registers **/
#define AS5048A_NOP 		0x0000	/*No operation dummy information*/
#define AS5048A_CLRERRFL 	0x0001	/*Error register. All errors are cleared by access*/
#define AS5048A_PROGCON		0x0003	/*Programming control register.*/

/*Programmable Customer Settings*/
#define AS5048A_ZPOSM		0x0016	/*Zero Position value high byte*/
#define AS5048A_ZPOSL 		0x0017	/*Zero Position remaining 6 lower LSB's*/

/*Readout Registers*/
#define AS5048A_DIAAGC 		0x3FFD
#define AS5048A_MAGNITUDE	0x3FFE
#define AS4048A_ANGLE		0x3FFF

#define AS4048A_RD 0x4000    // bit 14 = "1" is Read + parity even
#define AS4048A_WR 0x3FFF    // bit 14 = "0" is Write

#ifdef __cplusplus
}
#endif

#endif /* AS5048A_H_ */
