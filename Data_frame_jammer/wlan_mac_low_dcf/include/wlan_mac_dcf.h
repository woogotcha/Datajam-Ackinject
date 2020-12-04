/** @file wlan_mac_dcf.h
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


/*************************** Constant Definitions ****************************/
#ifndef WLAN_MAC_DCF_H_
#define WLAN_MAC_DCF_H_

#define PKT_BUF_INVALID                                   0xFF
#define MAX_NUM_PENDING_TX_PKT_BUFS 					  5


//-----------------------------------------------
// MAC Timing Structure
typedef struct{
	u16 t_slot;
	u16 t_sifs;
	u16 t_difs;
	u16	t_eifs;
	u16 t_phy_rx_start_dly;
	u16 t_timeout;
} mac_timing;

//-----------------------------------------------
// CW Update Reasons
#define DCF_CW_UPDATE_MPDU_TX_ERR                          0
#define DCF_CW_UPDATE_MPDU_RX_ACK                          1
#define DCF_CW_UPDATE_BCAST_TX                             2


//-----------------------------------------------
// Reason codes for generating a random number of slots
//     See:  rand_num_slots()
//
#define RAND_SLOT_REASON_STANDARD_ACCESS                   0
#define RAND_SLOT_REASON_IBSS_BEACON                       1


//-----------------------------------------------
// These are hardcoded OFDM TX times for CTS frames of various rates
//     Since CTS is a fixed size, we can precompute these to save time
//
#define TX_TIME_CTS_R6                                     50
#define TX_TIME_CTS_R12                                    38
#define TX_TIME_CTS_R24                                    34


//-----------------------------------------------
// According to 10.21.1 of 802.11-2012, the timestamp value of
// transmitted beacons and probe responses shall be set to the
// time of the data symbol containing the first bit of the timestamp
// This is a rate-specific value. For this release, we will hardcode
// the value for the default MCS 0, NONHT (aka 6 Mbps) rate. This
// needs to be changed for beacons & probe responses sent at a different
// rate
#define	T_TIMESTAMP_FIELD_OFFSET						   52


//-----------------------------------------------
// WLAN Exp low parameter defines (DCF)
//     NOTE:  Need to make sure that these values do not conflict with any of the LOW PARAM
//     callback defines
//
#define LOW_PARAM_DCF_RTS_THRESH                           0x10000001
#define LOW_PARAM_DCF_DOT11SHORTRETRY                      0x10000002
#define LOW_PARAM_DCF_DOT11LONGRETRY                       0x10000003
#define LOW_PARAM_DCF_PHYSICAL_CS_THRESH                   0x10000004
#define LOW_PARAM_DCF_CW_EXP_MIN                           0x10000005
#define LOW_PARAM_DCF_CW_EXP_MAX                           0x10000006



/*********************** Global Structure Definitions ************************/

typedef enum {
    RX_FINISH_SEND_NONE,
    RX_FINISH_SEND_A,
    RX_FINISH_SEND_B
} rx_finish_state_t;


typedef enum {
    TX_PENDING_NONE,
    TX_PENDING_A,
    TX_PENDING_B
} tx_pending_state_t;


typedef enum {
    TX_WAIT_NONE,
    TX_WAIT_ACK,
    TX_WAIT_CTS
} tx_wait_state_t;


typedef enum {
    TX_MODE_SHORT,
    TX_MODE_LONG
} tx_mode_t;

/*************************** Function Prototypes *****************************/
int                main();

u32                frame_receive(u8 rx_pkt_buf, phy_rx_details_t* phy_details);
void			   handle_mcast_buffer_enable(u32 enable);
void 			   update_tx_pkt_buf_lists();
void 			   handle_sample_rate_change(phy_samp_rate_t phy_samp_rate);
void               update_dtim_count();
void 			   update_tu_target(u8 recompute);
void 			   handle_mactime_change(s64 time_delta_usec);
void 			   configure_beacon_txrx(beacon_txrx_configure_t* beacon_txrx_configure);
void 			   frame_transmit_general(u8 pkt_buf);

#define			   DTIM_MCAST_RETURN_PAUSED			0x00000001
u32 			   frame_transmit_dtim_mcast(u8 pkt_buf, u8 resume);
int 			   handle_tx_pkt_buf_ready(u8 pkt_buf);

#define			   POLL_TX_PKT_BUF_LIST_RETURN_ERROR			0x80000000
#define			   POLL_TX_PKT_BUF_LIST_RETURN_TRANSMITTED		0x00000001
#define			   POLL_TX_PKT_BUF_LIST_RETURN_PAUSED			0x00000002
#define			   POLL_TX_PKT_BUF_LIST_RETURN_MORE_DATA		0x00000004
u32 			   poll_tx_pkt_buf_list(pkt_buf_group_t pkt_buf_group);

void        	   increment_src(u16* src_ptr);
void        	   increment_lrc(u16* lrc_ptr);

void 	   		   poll_tbtt_and_send_beacon();

#define 		   SEND_BEACON_RETURN_DTIM			0x00000001
#define 		   SEND_BEACON_RETURN_CANCELLED		0x00000002
u32 		   	   send_beacon(u8 tx_pkt_buf);

void        	   reset_cw();
void        	   reset_ssrc();
void        	   reset_slrc();

u32         	   rand_num_slots(u8 reason);

void               wlan_mac_dcf_hw_start_backoff(u16 num_slots);

int                wlan_create_ack_frame(void* pkt_buf_addr, u8* address_ra);
int                wlan_create_cts_frame(void* pkt_buf_addr, u8* address_ra, u16 duration);
int                wlan_create_rts_frame(void* pkt_buf_addr, u8* address_ra, u8* address_ta, u16 duration);

#endif /* WLAN_MAC_DCF_H_ */
