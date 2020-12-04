/** @file wlan_mac_entries.h
 *  @brief Event log
 *
 *  This contains the code for accessing event log.
 *
 *  @copyright Copyright 2014-2017, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *              See LICENSE.txt included in the design archive or
 *              at http://mangocomm.com/802.11/license
 *
 *  @note
 *  This is the only code that the user should modify in order to add entries
 *  to the event log.  To add a new entry, please follow the template provided
 *  and create:
 *    1) A new entry type in wlan_mac_entries.h
 *    2) Wrapper function:  get_next_empty_*_entry()
 *    3) Update the print function so that it is easy to print the log to the
 *    terminal
 *
 *  This file is part of the Mango 802.11 Reference Design (https://mangocomm.com/802.11)
 */

/***************************** Include Files *********************************/



/*************************** Constant Definitions ****************************/
#ifndef WLAN_MAC_ENTRIES_H_
#define WLAN_MAC_ENTRIES_H_

#include "wlan_mac_high_sw_config.h"

#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_high.h"
#include "wlan_mac_common.h"
#include "wlan_mac_bss_info.h"

#define WLAN_MAC_ENTRIES_LOG_CHAN_EST

// ****************************************************************************
// Define Entry Constants
//

//-----------------------------------------------
// Entry Enable Masks

#define ENTRY_EN_MASK_TXRX_CTRL                            0x01
#define ENTRY_EN_MASK_TXRX_MPDU                            0x02

//------------------------------------------------------------------------
// Entry Types
//
//     NOTE:  These values must match the definitions in Python WLAN Exp framework.
//

//-----------------------------------------------
// Management Entries

#define ENTRY_TYPE_NODE_INFO                               1
#define ENTRY_TYPE_EXP_INFO                                2

#define ENTRY_TYPE_TEMPERATURE                             4

#define ENTRY_TYPE_TIME_INFO                               6

//-----------------------------------------------
// Receive Entries

#define ENTRY_TYPE_RX_OFDM                                 10
#define ENTRY_TYPE_RX_OFDM_LTG                             11

#define ENTRY_TYPE_RX_DSSS                                 15

//-----------------------------------------------
// Transmit Entries

#define ENTRY_TYPE_TX_HIGH                                 20
#define ENTRY_TYPE_TX_HIGH_LTG                             21

#define ENTRY_TYPE_TX_LOW                                  25
#define ENTRY_TYPE_TX_LOW_LTG                              26




//------------------------------------------------------------------------
// MAC payload length
//
//    NOTE:  For normal packets, the minimum payload length must include the MAC header (ie 24 bytes)
//        so that the log entry is complete.  For LTG packets, the minimum payload length must include
//        the MAC header, LLC header, and LTG specific fields (ie 44 bytes).  The maximum number of
//        bytes that should be logged is a standard MTU (ie 1500 bytes).
//
#define MIN_MAC_PAYLOAD_LOG_LEN                            24
#define MIN_MAC_PAYLOAD_LTG_LOG_LEN                        44
#define MAX_MAC_PAYLOAD_LOG_LEN                            1500

// To never record payloads, you can set the min / max defines as follows:
//
//#define MAX_MAC_PAYLOAD_LOG_LEN                          MIN_MAC_PAYLOAD_LOG_LEN





/*********************** Global Structure Definitions ************************/

//-----------------------------------------------
// Node Info Entry
//   NOTE:  This structure was designed to work easily with the WLAN Exp Tag
//       Parameters.  The order and size of the fields match the corresponding
//       Tag Parameter so that population of this structure is easy.  Please see
//       "Node Info Structure for Tag Parameter Information" in wlan_exp_node.h
//       for the corresponding structure that populates this entry.
//
//   NOTE:  This structure is always at the start of the event log.  There is
//       the methods to add this entry type to the log are in wlan_mac_event_log.*
//
typedef struct{
    u64                 	timestamp;                         // Timestamp of the node info
    														   //   - This will reflect the oldest time of an
                                                           	   //     entry for a given log wrap
    u32                 	node_type;                         // Node type
    u32                 	node_id;                           // Node ID
    u32                 	hw_generation;                     // Node hardware generation
    u32                 	serial_number;                     // Node serial number
    u64                 	fpga_dna;                          // Node FPGA DNA
    u32                 	wlan_exp_version;                  // WLAN Exp Version
    u32                 	wlan_scheduler_resolution;         // Minimum Scheduler resolution (microseconds)
    u32                 	wlan_mac_addr[2];                  // WLAN MAC Address
    int                 	wlan_max_tx_power_dbm;             // WLAN maximum transmit power (in dBm)
    int                 	wlan_min_tx_power_dbm;             // WLAN minimum transmit power (in dBm)
    compilation_details_t	cpu_high_compilation_details;
    compilation_details_t	cpu_low_compilation_details;
} node_info_entry;



//-----------------------------------------------
// Experiment Info Entry
//
// NOTE:  When creating this entry, you need to allocate the size of the entry
//   plus the (info_length - 4).  For example:
//
//    (exp_info_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_EXP_INFO, (sizeof(exp_info_entry) + size - 4))
//
// NOTE:  The longest Experiment Info is:  ((2^16 - 1) - (sizeof(exp_info_entry) - 4)) bytes
//
typedef struct{
    u64                 timestamp;               // Timestamp of the log entry
    u16                 info_type;               // Type of Experiment Info
    u16                 info_length;             // Length of the experiment info data (in bytes)
    u8                  info_payload[4];         // Reference to payload contents for easy access in C
                                                 //   You can treat this array as the first 4 bytes of
                                                 //   the info payload but the array is actually valid
                                                 //   for info_length bytes.
} exp_info_entry;



//-----------------------------------------------
// Temperature Entry
//   NOTE: The temperature values are copied directly from the system monitor and need
//         to be converted to Celsius:
//           celsius = ((double(temp)/65536.0)/0.00198421639) - 273.15;
//
// Example request for a new temperature entry:
//
//     (temperature_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_TEMPERATURE, sizeof(temperature_entry))
//
typedef struct{
    u64                 timestamp;               // Timestamp of the log entry
    u32                 id;                      // Node ID
    u32                 serial_number;           // Node serial number
    u32                 curr_temp;               // Current Temperature of the node
    u32                 min_temp;                // Minimum recorded temperature of the node
    u32                 max_temp;                // Maximum recorded temperature of the node
} temperature_entry;



//-----------------------------------------------
// Time Info Entry
//
// Example request for a new Time info entry:
//
//     (time_info_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_TIME_INFO, sizeof(time_info_entry))
//
typedef struct{
    u64                 timestamp;               // Timestamp of the log entry (Timestamp of MAC time of old timebase)
    u32                 time_id;                 // ID of the time info entry so that these entries
                                                 //   can be synced across multiple nodes
    u32                 reason;                  // Reason code for log entry:
                                                 //     0 - SYSTEM (system added time info entry; eg beacon update)
                                                 //     1 - WLAN_EXP_SET_TIME
                                                 //     2 - WLAN_EXP_ADD_LOG
    u64                 mac_timestamp;           // Timestamp of MAC time (new timebase)
    u64                 system_timestamp;        // Timestamp of System time
    u64                 host_timestamp;          // Timestamp of Host time          (0xFFFFFFFF_FFFFFFFF if not known)
} time_info_entry;


#define TIME_INFO_ENTRY_SYSTEM                   0
#define TIME_INFO_ENTRY_WLAN_EXP_SET_TIME        1
#define TIME_INFO_ENTRY_WLAN_EXP_ADD_LOG         2

#define TIME_INFO_ENTRY_BASE_SYSTEM_TIME_ID      0x80000000

#define TIME_INFO_ENTRY_TIME_RSVD_VAL_64         0xFFFFFFFFFFFFFFFFULL



//-----------------------------------------------
// Common Receive Entry
//
typedef struct{
    u64                 timestamp;               // Timestamp of the log entry
    u8                  timestamp_frac;          // Additional fractional timestamp (160MHz clock units)
    u8                  phy_samp_rate;           // PHY Sampling Rate Mode
    u16                 length;                  // Length of the received packet
    u32                 cfo_est;                 // CFO Estimate
    u8                  mcs;                     // MCS rate at which the packet was received
    u8                  phy_mode;                // Mode of the PHY when the packet was received
    u8                  ant_mode;                // Antenna mode of the received packet
    s8                  power;                   // Power of the received packet
    u8                  reserved0;
    u8                  pkt_type;                // Type of packet
    u8                  chan_num;                // Channel on which the packet was received
    u8                  reserved1;
    u8                  rf_gain;                 // RF gain of the received packet
    u8                  bb_gain;                 // Baseband gain of the received packet
    u16                 flags;                   // 1-bit flags
} rx_common_entry;
#define RX_FLAGS_FCS_GOOD   			0x0001
#define RX_FLAGS_DUPLICATE				0x0002
#define RX_FLAGS_UNEXPECTED_RESPONSE	0x0004
#define RX_FLAGS_LTG_PYLD				0x0040
#define RX_FLAGS_LTG					0x0080





//-----------------------------------------------
// Receive OFDM Entry
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new receive OFDM entry:
//
//     (rx_ofdm_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_RX_OFDM, sizeof(rx_ofdm_entry) + extra_payload)
//
typedef struct{
    rx_common_entry     rx_entry;

#ifdef WLAN_MAC_ENTRIES_LOG_CHAN_EST
    u32                 channel_est[64];         // Channel estimates for the packet
#endif

    u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
    u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} rx_ofdm_entry;



//-----------------------------------------------
// Receive DSSS Entry
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new receive DSSS entry:
//
//     (rx_dsss_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_RX_DSSS, sizeof(rx_dsss_entry) + extra_payload)
//
typedef struct{
    rx_common_entry     rx_entry;
    u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
    u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} rx_dsss_entry;



//-----------------------------------------------
// High-level Transmit Entry
//   NOTE:  padding field is to have a 32-bit aligned struct.  That way sizeof()
//          accurately reflects the number of bytes in the struct.
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new high-level transmit entry:
//
//     (tx_high_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_TX_HIGH, sizeof(tx_high_entry) + extra_payload)
//
typedef struct{
    u64                 timestamp_create;        // Timestamp of the log entry creation
    u32                 delay_accept;            // Delay from timestamp_create to when accepted by CPU Low
    u32                 delay_done;              // Delay from delay_accept to when CPU Low was done
    u64                 unique_seq;              // Unique packet sequence number
    u32					padding0;
    u16                 num_tx;                  // Number of Transmissions that it took to send the packet
    u16                 length;                  // Length of the packet
    u8                  padding1;
    u8                  pkt_type;                // Type of packet
    u16                 queue_id;                // Queue ID this packet was sent from
    u16                 queue_occupancy;         // Occupancy of queue (includes itself)
    u16                 flags;                   // 1-bit flags
    u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
    u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} tx_high_entry;

#define TX_HIGH_FLAGS_SUCCESSFUL 0x0001
#define TX_HIGH_FLAGS_LTG_PYLD	 0x0040
#define TX_HIGH_FLAGS_LTG		 0x0080

//-----------------------------------------------
// Low-level Transmit Entry
//   NOTE:  rsvd field is to have a 32-bit aligned struct.  That way sizeof()
//          accurately reflects the number of bytes in the struct.
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new low-level transmit entry:
//
//     (tx_low_entry *) wlan_exp_log_create_entry(ENTRY_TYPE_TX_LOW, sizeof(tx_low_entry))
//
typedef struct{
    u64                 timestamp_send;          // Timestamp of when packet was sent
    u64                 unique_seq;              // Unique packet sequence number
    phy_tx_params_t     phy_params;              // Transmission parameters
    u8					reserved0;
    u8                  chan_num;                // Channel on which this packet was sent
    u16                 length;                  // Length of the packet
    s16                 num_slots;               // Number of backoff slots
    u16                 cw;                      // Contention Window
    u8                  pkt_type;                // Type of packet
    u8                  flags;                   // Misc. flags
    u8                  timestamp_send_frac;     // Additional fractional timestamp (160MHz clock units)
    u8                  phy_samp_rate;           // PHY Sampling Rate Mode
    u16					transmission_count;      // What transmission was this packet
    u16					reserved1;
    u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
    u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} tx_low_entry;
#define TX_LOW_FLAGS_RECEIVED_RESPONSE      0x01
#define TX_LOW_FLAGS_LTG_PYLD				0x40
#define TX_LOW_FLAGS_LTG	    			0x80





/*************************** Function Prototypes *****************************/

u8                 wlan_exp_log_get_entry_en_mask();
void               wlan_exp_log_set_entry_en_mask(u8 mask);

void               wlan_exp_log_reset_system_time_id();

//-----------------------------------------------
// Method to get / set the global variable mac_payload_log_len
//
u32                wlan_exp_log_get_mac_payload_len();
void               wlan_exp_log_set_mac_payload_len(u32 payload_len);


//-----------------------------------------------
// Wrapper method to get an entry
//
void             * wlan_exp_log_create_entry(u16 entry_type_id, u16 entry_size);


//-----------------------------------------------
// Methods to create an entry
//
tx_high_entry    * wlan_exp_log_create_tx_high_entry(tx_frame_info_t* tx_frame_info);
tx_low_entry     * wlan_exp_log_create_tx_low_entry(tx_frame_info_t* tx_frame_info, wlan_mac_low_tx_details_t* tx_low_details);

rx_common_entry * wlan_exp_log_create_rx_entry(rx_frame_info_t* rx_frame_info);

//-----------------------------------------------
// Print function for all entries
//
#ifdef _DEBUG_
void               print_entry(u32 entry_number, u32 entry_type, void * entry);
#endif


//-----------------------------------------------
// Methods to add entries to the log
//
void      add_node_info_entry();

void      add_time_info_entry(u64 timestamp, u64 mac_time, u64 system_time, u64 host_time, u32 reason, u32 time_id, u8 use_time_id);

u32       add_temperature_to_log();

#endif /* WLAN_MAC_ENTRIES_H_ */
