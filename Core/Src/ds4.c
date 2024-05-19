/*
 * ds4.c
 *
 *  Created on: Jan 11, 2024
 *      Author: asus
 */

#include "ds4.h"

int data0_buttons[] = {L1_KANAN_BIT, R1_BAWAH_BIT, L3_ATAS_BIT, R3_KIRI_BIT, SHARE_KOTAK_BIT, OPTIONS_SILANG_BIT, PS_BULAT_BIT, TPAD_SEGITIGA_BIT};

void handleData0Buttons(uint8_t data0) {
	   // Set nilai awal
	    kanan = 0;
	    bawah = 0;
	    atas = 0;
	    kiri = 0;
	    kotak = 0;
	    silang = 0;
	    bulat = 0;
	    segitiga = 0;
    for (int i = 0; i < sizeof(data0_buttons) / sizeof(data0_buttons[0]); i++) {
        if (data0 & data0_buttons[i]) {
            // Tombol aktif, lakukan sesuatu
            switch (i) {
                case 0: kanan = 1; break;
                case 1: bawah = 1; break;
                case 2: atas = 1; break;
                case 3: kiri = 1; break;
                case 4: kotak = 1; break;
                case 5: silang = 1; break;
                case 6: bulat = 1; break;
                case 7: segitiga = 1; break;
            }
        }
    }
}

void handleData1Buttons(uint8_t data1) {
	// Set nilai awal
	    l1 = 0;
	    r1 = 0;
	    l3 = 0;
	    r3 = 0;
	    share = 0;
	    options = 0;
	    ps = 0;
	    tpad = 0;
    for (int i = 0; i < sizeof(data0_buttons) / sizeof(data0_buttons[0]); i++) {
        if (data1 & data0_buttons[i]) {
            // Tombol aktif, lakukan sesuatu
            switch (i) {
                case 0: l1 = 1; break;
                case 1: r1 = 1; break;
                case 2: l3 = 1; break;
                case 3: r3 = 1; break;
                case 4: share = 1; break;
                case 5: options = 1; break;
                case 6: ps = 1; break;
                case 7: tpad = 1; break;
            }
        }
    }
}

void ds4(){
	if (CANSPI_Receive(&rxMessage)){
		if(rxMessage.frame.id == 0x36){
			handleData0Buttons(rxMessage.frame.data0);
			handleData1Buttons(rxMessage.frame.data1);
			l2 = rxMessage.frame.data2;
			r2 = rxMessage.frame.data3;
			lx = rxMessage.frame.data4;
			lxs = rxMessage.frame.data5;
			lys = rxMessage.frame.data6;
			ly = rxMessage.frame.data7;
		}
	}
}
