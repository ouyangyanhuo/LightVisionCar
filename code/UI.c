/*
 *
 *  Name: UI.c
 *  Created on: 2025/2/24
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"

void EncoderData(void) {
	// Âö³åÊý¾Ý
	tft180_show_int(0, 80, Left_Pulse_now, 1);
	tft180_show_int(10, 80, Right_Pulse_now, 1);
}
void WIFISPIData(void) {
	tft180_show_string(0, 80, wifi_spi_mac_addr);
	tft180_show_string(0, 95, wifi_spi_ip_addr_port);
}
