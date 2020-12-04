/** @file wlan_mac_sta_uart_menu.c
 *  @brief Station UART Menu
 *
 *  This contains code for the 802.11 Station's UART menu.
 *
 *  @copyright Copyright 2013-2017, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *                See LICENSE.txt included in the design archive or
 *                at http://mangocomm.com/802.11/license
 *
 *  This file is part of the Mango 802.11 Reference Design (https://mangocomm.com/802.11)
 */

/***************************** Include Files *********************************/
#include "wlan_mac_high_sw_config.h"

// Xilinx SDK includes
#include "xparameters.h"
#include "stdio.h"
#include "stdlib.h"
#include "xtmrctr.h"
#include "xio.h"
#include "string.h"
#include "xintc.h"

// WLAN includes
#include "wlan_mac_time_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_high.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_sta.h"
#include "wlan_mac_sta_join.h"
#include "ascii_characters.h"
#include "wlan_mac_schedule.h"
#include "wlan_mac_event_log.h"
#include "wlan_mac_bss_info.h"
#include "wlan_mac_scan.h"
#include "wlan_mac_station_info.h"


//
// Use the UART Menu
//     - If WLAN_USE_UART_MENU in wlan_mac_sta.h is commented out, then this function
//       will do nothing.  This might be necessary to save code space.
//


#ifndef WLAN_USE_UART_MENU

void uart_rx(u8 rxByte){ };

#else

/*************************** Constant Definitions ****************************/

//-----------------------------------------------
// UART Menu Modes
#define UART_MODE_MAIN                                     0
#define UART_MODE_INTERACTIVE                              1
#define UART_MODE_JOIN                                     2


/*********************** Global Variable Definitions *************************/
extern tx_params_t                          default_unicast_data_tx_params;

extern bss_info_t*                          active_bss_info;

/*************************** Variable Definitions ****************************/

static volatile u8                          uart_mode            = UART_MODE_MAIN;
static volatile u32                         schedule_id;
static volatile u32                         check_join_status_id;
static volatile u8                          print_scheduled      = 0;

static char                                 text_entry[SSID_LEN_MAX + 1];
static u8                                   curr_char            = 0;

/*************************** Functions Prototypes ****************************/

void print_main_menu();

void print_station_status();

void check_join_status();

void start_periodic_print();
void stop_periodic_print();


/*************************** Variable Definitions ****************************/


/******************************** Functions **********************************/


/*****************************************************************************/
/**
 * Process each character received by the UART
 *
 * The following functionality is supported:
 *    - Main Menu
 *      - Interactive Menu (prints all station infos)
 *      - Print all counts
 *      - Print event log size (hidden)
 *      - Print Network List
 *      - Print Malloc info (hidden)
 *      - Join BSS
 *      - Reset network list (hidden)
 *    - Interactive Menu
 *      - Reset counts
 *      - Turn on/off "Traffic Blaster" (hidden)
 *
 * The escape key is used to return to the Main Menu.
 *
 *****************************************************************************/
void uart_rx(u8 rxByte){
	volatile join_parameters_t  * join_parameters;
	volatile scan_parameters_t  * scan_params;
	u32                           is_scanning;

	// ----------------------------------------------------
	// Return to the Main Menu
	//    - Stops any prints / LTGs
	if (rxByte == ASCII_ESC) {
		uart_mode = UART_MODE_MAIN;
		stop_periodic_print();
		print_main_menu();

		// Stop join process
		if (wlan_mac_sta_is_joining()) {
			wlan_mac_sta_join_return_to_idle();
		}
		return;
	}

	switch(uart_mode){

		// ------------------------------------------------
		// Main Menu processing
		//
		case UART_MODE_MAIN:
			switch(rxByte){

				// ----------------------------------------
				// '1' - Switch to Interactive Menu
				//
				case ASCII_1:
					uart_mode = UART_MODE_INTERACTIVE;
					start_periodic_print();
				break;

				// ----------------------------------------
				// '2' - Print Station Info with Counts
				//
				case ASCII_2:
					station_info_print(NULL , STATION_INFO_PRINT_OPTION_FLAG_INCLUDE_COUNTS);
				break;

				// ----------------------------------------
				// 'e' - Print event log size
				//
				case ASCII_e:
#if WLAN_SW_CONFIG_ENABLE_LOGGING
					event_log_config_logging(EVENT_LOG_LOGGING_DISABLE);
					print_event_log_size();
					event_log_config_logging(EVENT_LOG_LOGGING_ENABLE);
#endif //WLAN_SW_CONFIG_ENABLE_LOGGING
				break;

				// ----------------------------------------
				// 'a' - Print BSS information
				//
				case ASCII_a:
					print_bss_info();
				break;

				// ----------------------------------------
				// 'm' - Display Heap / Malloc information
				//
				case ASCII_m:
					wlan_mac_high_display_mallinfo();
				break;

				// ----------------------------------------
				// 'x' - Reset network list
				//
				case ASCII_x:
					wlan_mac_high_reset_network_list();
				break;

				// ----------------------------------------
				// 's' - Scan
				//
				case ASCII_s:
					xil_printf("\f");

					// Check if node is currently in a scan
					is_scanning = wlan_mac_scan_is_scanning();

					// Stop the current scan to update the scan parameters
					if (is_scanning) {
						xil_printf("Stopping Scan\n");
						wlan_mac_scan_stop();
					} else {
						xil_printf("Starting Scan\n");

						// Get current scan parameters
						scan_params = wlan_mac_scan_get_parameters();

						// Free the current scan SSID, it will be replaced
						if (scan_params->ssid != NULL) {
							wlan_mac_high_free(scan_params->ssid);
						}

						// Set to passive scan
						scan_params->ssid = strndup("", SSID_LEN_MAX);

						wlan_mac_scan_start();
					}
				break;
				// ----------------------------------------
				// 'j' - Join
				//
				case ASCII_j:
					uart_mode = UART_MODE_JOIN;

					xil_printf("\f");

					// Check if node is currently in a scan
					is_scanning = wlan_mac_scan_is_scanning();

					// Stop the current scan to update the scan parameters
					if (is_scanning) {
						wlan_mac_scan_stop();
					}

					// Print results of the scan
					print_bss_info();

					// Print prompt
					xil_printf("Enter SSID to join, please type a new string and press enter\n");
					xil_printf(": ");
				break;
			}
		break;


		// ------------------------------------------------
		// Interactive Menu processing
		//
		case UART_MODE_INTERACTIVE:
			switch(rxByte){

				// ----------------------------------------
				// 'r' - Reset station counts
				//
				case ASCII_r:
#if WLAN_SW_CONFIG_ENABLE_TXRX_COUNTS
					txrx_counts_zero_all();
#endif
				break;
			}
		break;


		// ------------------------------------------------
		// Get SSID to Join
		//
		case UART_MODE_JOIN:
			switch(rxByte){

				// ----------------------------------------
				// Press <Enter> - process new SSID
				//
				case ASCII_CR:
					// Create a '\0' as the final character so SSID is a proper string
					text_entry[curr_char] = 0;

					// Get the current join parameters
					join_parameters = wlan_mac_sta_get_join_parameters();

					// Free the current join SSID, it will be replaced
					if (join_parameters->ssid != NULL) {
						wlan_mac_high_free(join_parameters->ssid);
					}

					// If SSID was not "", perform the join
					if (curr_char != 0) {
						// Set the SSID
						join_parameters->ssid = strndup(text_entry, SSID_LEN_MAX);

						// Clear the BSSID and channel
						bzero((void *)join_parameters->bssid, MAC_ADDR_LEN);
						join_parameters->channel = 0;

						// Call join function
						wlan_mac_sta_join();

						// Reset SSID character pointer
						curr_char = 0;

						// Start the check_join_status
						check_join_status_id = wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, 100000, SCHEDULE_REPEAT_FOREVER, (void*)check_join_status);

						// Print info to screen
						//     - Pause for a second since the return to the Main Menu will erase the screen
						xil_printf("\nJoining: %s\n", text_entry);

					} else {
						uart_mode = UART_MODE_MAIN;

						// Print error message
						xil_printf("\nNo SSID entered.  Returning to Main Menu.\n");

						// Wait a second before proceeding
						wlan_usleep(2000000);

						// Print the main menu
						print_main_menu();
					}
				break;

				// ----------------------------------------
				// Press <Delete> - Remove last character
				//
				case ASCII_DEL:
					if (curr_char > 0) {
						curr_char--;
						xil_printf("\b \b");
					}
				break;

				// ----------------------------------------
				// Process character
				//
				default:
					if (((rxByte <= ASCII_z) && (rxByte >= ASCII_A)) ||
						(rxByte == ASCII_SPACE) || (rxByte == ASCII_DASH)) {
						if (curr_char < SSID_LEN_MAX) {
							xil_printf("%c", rxByte);
							text_entry[curr_char] = rxByte;
							curr_char++;
						}
					}
				break;
			}
		break;


		default:
			uart_mode = UART_MODE_MAIN;
			print_main_menu();
		break;
	}
}



void print_main_menu(){
	xil_printf("\f");
	xil_printf("********************** Station Menu **********************\n");
	xil_printf("[1]   - Interactive Station Status\n");
	xil_printf("[2]   - Print all Observed Counts\n");
	xil_printf("\n");
	xil_printf("[a]   - Display Network List\n");
	xil_printf("[j]   - Join a network\n");
	xil_printf("[s]   - Toggle active scan\n");
	xil_printf("**********************************************************\n");
}



void print_station_status(){

    u64            timestamp;
    dl_entry     	* access_point_entry  = NULL;
    station_info_t  * access_point        = NULL;

    if(active_bss_info != NULL){
        access_point_entry = active_bss_info->members.first;
        access_point = ((station_info_t*)(access_point_entry->data));
    }

    if (uart_mode == UART_MODE_INTERACTIVE) {
        timestamp = get_system_time_usec();
        xil_printf("\f");
        xil_printf("---------------------------------------------------\n");

            if(active_bss_info != NULL){
                xil_printf(" MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
                            access_point->addr[0],access_point->addr[1],access_point->addr[2],access_point->addr[3],access_point->addr[4],access_point->addr[5]);
                xil_printf("     - Last heard from         %d ms ago\n",((u32)(timestamp - (access_point->latest_rx_timestamp)))/1000);
            }

        xil_printf("---------------------------------------------------\n");
        xil_printf("\n");
        xil_printf("[r] - reset counts\n\n");
    }
}

void check_join_status() {
	// If node is no longer in the join process:
	//     - Check BSS info
	//     - Return to Main Menu

	if (wlan_mac_sta_is_joining() == 0) {
		// Stop the scheduled event
		wlan_mac_remove_schedule(SCHEDULE_COARSE, check_join_status_id);

		// Return to main menu
		uart_mode = UART_MODE_MAIN;

		if (active_bss_info != NULL) {
			// Print success message
			xil_printf("\nSuccessfully Joined: %s\n", active_bss_info->ssid);
		} else {
			// Print error message
			xil_printf("\nJoin not successful.  Returning to Main Menu.\n");
		}

		// Wait a second before proceeding
		wlan_usleep(3000000);

		// Print the main menu
		print_main_menu();
	}
}




void start_periodic_print(){
	stop_periodic_print();
	print_station_status();
	print_scheduled = 1;
	schedule_id = wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, 1000000, SCHEDULE_REPEAT_FOREVER, (void*)print_station_status);
}



void stop_periodic_print(){
	if (print_scheduled) {
		print_scheduled = 0;
		wlan_mac_remove_schedule(SCHEDULE_COARSE, schedule_id);
	}
}



#endif


