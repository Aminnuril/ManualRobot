/*
 * ds4.h
 *
 *  Created on: Jan 11, 2024
 *      Author: asus
 */

#ifndef INC_DS4_H_
#define INC_DS4_H_

// Mendefinisikan nilai bit untuk setiap kondisi
#define L1_KANAN_BIT 1
#define R1_BAWAH_BIT 2
#define L3_ATAS_BIT 4
#define R3_KIRI_BIT 8
#define SHARE_KOTAK_BIT 16
#define OPTIONS_SILANG_BIT 32
#define PS_BULAT_BIT 64
#define TPAD_SEGITIGA_BIT 128

#include "CAN_SPI.h"

extern uCAN_MSG txMessage;
extern uCAN_MSG rxMessage;
extern int atas, bawah, kanan, kiri, kotak, silang, bulat, segitiga;
extern int l1, r1, l3, r3, share, options, ps, tpad;
extern int data0_buttons[];
//extern int data1_buttons[];
extern int lx, ly, l2, r2,lxs,lys;

void ds4();
void handleData0Buttons(uint8_t data0);
void handleData1Buttons(uint8_t data1);

#endif /* INC_DS4_H_ */
