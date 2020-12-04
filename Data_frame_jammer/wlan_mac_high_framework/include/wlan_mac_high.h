/** @file wlan_mac_high.h
 *  @brief Top-level WLAN MAC High Framework
 *
 *  This contains the top-level code for accessing the WLAN MAC High Framework.
 *
 *  @copyright Copyright 2014-2017, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *                See LICENSE.txt included in the design archive or
 *                at http://mangocomm.com/802.11/license
 *
 *  This file is part of the Mango 802.11 Reference Design (https://mangocomm.com/802.11)
 */


/*************************** Constant Definitions ****************************/
#ifndef WLAN_MAC_HIGH_H_
#define WLAN_MAC_HIGH_H_

#include "wlan_mac_high_sw_config.h"

#include "xil_types.h"
#include "wlan_mac_common.h"

/********************************************************************
 * Auxiliary (AUX) BRAM and DRAM (DDR) Memory Maps

 * The 802.11 Reference Design hardware includes a 64 KB BRAM block named
 * AUX BRAM. This block is mapped into the address space of CPU High
 * and provides a low-latency memory block for data storage beyond the
 * DLMB.
 *
 * The reference code uses the AUX BRAM to store various data structures
 * that provides references to larger blocks in DRAM. These data structures
 * benefit from the low-latency access of the BRAM block.
 *
 * For example, the doubly-linked list of Tx Queue entries is stored in the
 * AUX BRAM. Each list entry points to a dedicated 4KB buffer in DRAM. The C code
 * can manage a queue with quick list operatations in BRAM while the queued packets
 * themselves are stored in the slower but *much* larger DRAM.
 *
 * The 64 KB AUX BRAM block is divided as follows:
 *
 * ***************************** AUX BRAM Map *********************************************
 * Description                      | Size
 * -------------------------------------------------------------------------------------------------
 * Tx Queue list entries (entry.data points to DRAM)      | 40960 B (TX_QUEUE_DL_ENTRY_MEM_SIZE)
 * BSS Info list entries (entry.data points to DRAM)      |  4608 B (BSS_INFO_DL_ENTRY_MEM_SIZE)
 * Station Info list entries (entry.data points to DRAM)  |  4608 B (STATION_INFO_DL_ENTRY_MEM_SIZE)
 * ETH A Tx DMA buffer descriptors                        |    64 B (ETH_TX_BD_MEM_SIZE)
 * ETH A Rx DMA buffer descriptors                        | 15296 B (ETH_RX_BD_MEM_SIZE)
 * -------------------------------------------------------------------------------------------------
 *
 * On WARP v3 1 GB of the DDR3 SO-DIMM DRAM is mapped into the address space of CPU High.
 * This memory space is used as follows:
 *
 * ******************************* DRAM Map *********************************************************
 * Description                      | Size
 * --------------------------------------------------------------------------------------------------
 * wlan_exp Eth buffers             |    1024 KB (WLAN_EXP_ETH_BUFFERS_SECTION_SIZE)
 * Tx queue buffers	                |    1400 KB (TX_QUEUE_BUFFER_SIZE)
 * BSS Info buffers	                |      27 KB (BSS_INFO_BUFFER_SIZE)
 * Station Info buffers             |      69 KB (STATION_INFO_BUFFER_SIZE)
 * User scratch space               |   10000 KB (USER_SCRATCH_SIZE)
 * Event log                        | 1036056 KB (EVENT_LOG_SIZE)
 * --------------------------------------------------------------------------------------------------
 *
 *
 ************************************************************************************************/


#define AUX_BRAM_BASE                      (XPAR_MB_HIGH_AUX_BRAM_CTRL_S_AXI_BASEADDR)
#define AUX_BRAM_SIZE                      (XPAR_MB_HIGH_AUX_BRAM_CTRL_S_AXI_HIGHADDR - XPAR_MB_HIGH_AUX_BRAM_CTRL_S_AXI_BASEADDR + 1)
#define AUX_BRAM_HIGH                      (XPAR_MB_HIGH_AUX_BRAM_CTRL_S_AXI_HIGHADDR)

#define DRAM_BASE                          (XPAR_DDR3_SODIMM_S_AXI_BASEADDR)
#define DRAM_SIZE                          (XPAR_DDR3_SODIMM_S_AXI_HIGHADDR - XPAR_DDR3_SODIMM_S_AXI_BASEADDR + 1)
#define DRAM_HIGH                          (XPAR_DDR3_SODIMM_S_AXI_HIGHADDR)



#define CALC_HIGH_ADDR(base, size)         ((base)+((size)-1))



/********************************************************************
 * wlan_exp and IP/UDP library Ethernet buffers
 *
 * The wlan_exp Ethernet handling code uses large buffers for constructing packets
 * for transmission and for processing received packets. The IP/UDP library uses
 * multiple buffers to pipeline Ethernet operations. The library also supports jumbo
 * frames. As a result the wlan_exp and IP/UDP library Ethernet buffers are too large
 * to store in on-chip BRAM. Instead the first 1MB of the DRAM is reserved for use
 * by the wlan_exp Ethernet code.
 *
 * The linker script for CPU High *must* include a dedicated section allocated at the
 * base of the DRAM address range. The macros below assume this section exist and
 * are used to verify the wlan_exp and IP/UDP library code does not overflow the
 * DRAM allocation.
 *
 ********************************************************************/
#define WLAN_EXP_ETH_BUFFERS_SECTION_BASE   (DRAM_BASE)
#define WLAN_EXP_ETH_BUFFERS_SECTION_SIZE   (1024 * 1024)
#define WLAN_EXP_ETH_BUFFERS_SECTION_HIGH   CALC_HIGH_ADDR(WLAN_EXP_ETH_BUFFERS_SECTION_BASE, WLAN_EXP_ETH_BUFFERS_SECTION_SIZE)


/********************************************************************
 * TX Queue
 *
 * The Tx Queue consists of two pieces:
 *     (1) dl_entry structs that live in the AUX BRAM
 *     (2) Data buffers for the packets themselves than live in DRAM
 *
 * The definitions below reserve memory for these two pieces.  The default
 * value of 40 kB do the dl_entry memory space was chosen. Because each dl_entry has a
 * size of 12 bytes, this space allows for a potential of 3413 dl_entry structs describing
 * Tx queue elements.
 *
 * As far as the actual payload space in DRAM, 14000 kB was chosen because this is enough
 * to store each of the 3413 Tx queue elements. Each queue element points to a unique 4KB-sized
 * buffer.
 *
 ********************************************************************/

#define TX_QUEUE_DL_ENTRY_MEM_BASE         (AUX_BRAM_BASE)
#define TX_QUEUE_DL_ENTRY_MEM_SIZE         (40 * 1024)
#define TX_QUEUE_DL_ENTRY_MEM_HIGH          CALC_HIGH_ADDR(TX_QUEUE_DL_ENTRY_MEM_BASE, TX_QUEUE_DL_ENTRY_MEM_SIZE)

#define TX_QUEUE_BUFFER_BASE               (WLAN_EXP_ETH_BUFFERS_SECTION_HIGH + 1)
#define TX_QUEUE_BUFFER_SIZE               (14000 * 1024)
#define TX_QUEUE_BUFFER_HIGH                CALC_HIGH_ADDR(TX_QUEUE_BUFFER_BASE, TX_QUEUE_BUFFER_SIZE)



/********************************************************************
 * BSS Info
 *
 * The BSS Info storage consists of two pieces:
 *     (1) dl_entry structs that live in the aux. BRAM and
 *     (2) bss_info_t buffers with the actual content that live in DRAM
 *
 ********************************************************************/
#define BSS_INFO_DL_ENTRY_MEM_BASE         (TX_QUEUE_DL_ENTRY_MEM_BASE + TX_QUEUE_DL_ENTRY_MEM_SIZE)
#define BSS_INFO_DL_ENTRY_MEM_SIZE         (4608)
#define BSS_INFO_DL_ENTRY_MEM_HIGH          CALC_HIGH_ADDR(BSS_INFO_DL_ENTRY_MEM_BASE, BSS_INFO_DL_ENTRY_MEM_SIZE)

#define BSS_INFO_BUFFER_BASE               (TX_QUEUE_BUFFER_HIGH + 1)
#define BSS_INFO_BUFFER_SIZE			   ((BSS_INFO_DL_ENTRY_MEM_SIZE/sizeof(dl_entry))*sizeof(bss_info_t))
#define BSS_INFO_BUFFER_HIGH                CALC_HIGH_ADDR(BSS_INFO_BUFFER_BASE, BSS_INFO_BUFFER_SIZE)


/********************************************************************
 * Station Info
 *
 * The Station Info storage consists of two pieces:
 *     (1) dl_entry structs that live in the aux. BRAM and
 *     (2) station_info_t buffers with the actual content that live in DRAM
 *
 ********************************************************************/
#define STATION_INFO_DL_ENTRY_MEM_BASE     (BSS_INFO_DL_ENTRY_MEM_HIGH + 1)
#define STATION_INFO_DL_ENTRY_MEM_SIZE     (4608)
#define STATION_INFO_DL_ENTRY_MEM_NUM      (STATION_INFO_DL_ENTRY_MEM_SIZE/sizeof(dl_entry))
#define STATION_INFO_DL_ENTRY_MEM_HIGH      CALC_HIGH_ADDR(STATION_INFO_DL_ENTRY_MEM_BASE, STATION_INFO_DL_ENTRY_MEM_SIZE)

#define STATION_INFO_BUFFER_BASE          (BSS_INFO_BUFFER_HIGH + 1)
#define STATION_INFO_BUFFER_SIZE		  ((STATION_INFO_DL_ENTRY_MEM_SIZE/sizeof(dl_entry))*sizeof(station_info_t))
#define STATION_INFO_BUFFER_HIGH           CALC_HIGH_ADDR(STATION_INFO_BUFFER_BASE, STATION_INFO_BUFFER_SIZE)



/********************************************************************
 * Ethernet TX Buffer Descriptors
 *
 * The current architecture blocks on Ethernet transmissions. As such, only a single
 * Eth DMA buffer descriptor (BD) is needed. Each BD is 64 bytes in size (see
 * XAXIDMA_BD_MINIMUM_ALIGNMENT), so we set ETH_TX_BD_SIZE to 64.
 *
 ********************************************************************/
#define ETH_DMA_BD_SIZE			       64 // hard-code sizeof(XAxiDma_Bd) to avoid including axi_dma header

#define ETH_TX_BD_MEM_BASE                     (STATION_INFO_DL_ENTRY_MEM_HIGH + 1)
#define ETH_TX_BD_MEM_SIZE                     (1 * ETH_DMA_BD_SIZE)
#define ETH_TX_BD_MEM_HIGH                      CALC_HIGH_ADDR(ETH_TX_BD_MEM_BASE, ETH_TX_BD_MEM_SIZE)



/********************************************************************
 * Ethernet RX Buffer Descriptors
 *
 * The last section we are defining in the aux. BRAM is for ETH_RX.
 * Like TX, each BD is 64 bytes. Unlike TX, we need more than a single
 * BD to be able to handle bursty Ethernet receptions.
 *
 ********************************************************************/
#define ETH_RX_BD_MEM_BASE                     (ETH_TX_BD_MEM_HIGH + 1)
#define ETH_RX_BD_MEM_SIZE                     (239 * ETH_DMA_BD_SIZE)
#define ETH_RX_BD_MEM_HIGH                      CALC_HIGH_ADDR(ETH_RX_BD_MEM_BASE, ETH_RX_BD_MEM_SIZE)



/********************************************************************
 * User Scratch Space
 *
 * We have set aside 10MB of space for users to use the DRAM in their applications.
 * We do not use the below definitions in any part of the reference design.
 *
 ********************************************************************/
#define USER_SCRATCH_BASE                  (STATION_INFO_BUFFER_HIGH + 1)
#define USER_SCRATCH_SIZE                  (10000 * 1024)
#define USER_SCRATCH_HIGH                   CALC_HIGH_ADDR(USER_SCRATCH_BASE, USER_SCRATCH_SIZE)


/********************************************************************
 * Event Log
 *
 * The remaining space in DRAM is used for the WLAN Experiment Framework event log. The above
 * sections in DRAM are much smaller than the space set aside for the event log. In the current
 * implementation, the event log is ~995 MB.
 *
 ********************************************************************/
#define EVENT_LOG_BASE                     (USER_SCRATCH_HIGH + 1)
#define EVENT_LOG_SIZE                     (DRAM_HIGH - EVENT_LOG_BASE + 1) // log occupies all remianing DRAM
#define EVENT_LOG_HIGH                      CALC_HIGH_ADDR(EVENT_LOG_BASE, EVENT_LOG_SIZE)



//-----------------------------------------------
// Device IDs
//
// NOTE:  These are re-definitions of xparameters.h #defines so that the name of the
//     underlying hardware component can change and it will only require modifying
//     one line of code in the SDK project.
//
#define INTC_DEVICE_ID                                     XPAR_INTC_0_DEVICE_ID              ///< XParameters rename of interrupt controller device ID
#define UARTLITE_DEVICE_ID                                 XPAR_UARTLITE_0_DEVICE_ID          ///< XParameters rename for UART
#define GPIO_USERIO_DEVICE_ID                              XPAR_MB_HIGH_SW_GPIO_DEVICE_ID     ///< XParameters rename of device ID of GPIO

//-----------------------------------------------
// Interrupt IDs
//
//  These macros define the index of each interrupt signal in the axi_intc input
//  The defines below rename macros from xparameters.h to remove instance-name-specific
//    strings from the application code
#define INTC_GPIO_USERIO_INTERRUPT_ID                      XPAR_MB_HIGH_INTC_MB_HIGH_SW_GPIO_IP2INTC_IRPT_INTR               ///< XParameters rename of GPIO interrupt ID
#define UARTLITE_INT_IRQ_ID                                XPAR_INTC_0_UARTLITE_0_VEC_ID      ///< XParameters rename of UART interrupt ID
#define TMRCTR_INTERRUPT_ID                                XPAR_INTC_0_TMRCTR_0_VEC_ID        ///< XParameters rename of timer interrupt ID



//-----------------------------------------------
// WLAN Constants
//
#define ENCAP_MODE_AP                                      0                                  ///< Used as a flag for AP encapsulation and de-encapsulation
#define ENCAP_MODE_STA                                     1                                  ///< Used as a flag for STA encapsulation and de-encapsulation
#define ENCAP_MODE_IBSS                                    2                                  ///< Used as a flag for IBSS encapsulation and de-encapsulation

#define TX_BUFFER_NUM                                      2                                  ///< Number of PHY transmit buffers to use. This should remain 2 (ping/pong buffering).

#define GPIO_USERIO_INPUT_CHANNEL                          1                                  ///< Channel used as input user IO inputs (buttons, DIP switch)
#define GPIO_USERIO_INPUT_IR_CH_MASK                       XGPIO_IR_CH1_MASK                  ///< Mask for enabling interrupts on GPIO input

#define GPIO_MASK_DRAM_INIT_DONE                           0x00000100                         ///< Mask for GPIO -- DRAM initialization bit
#define GPIO_MASK_PB_U                                     0x00000040                         ///< Mask for GPIO -- "Up" Pushbutton
#define GPIO_MASK_PB_M                                     0x00000020                         ///< Mask for GPIO -- "Middle" Pushbutton
#define GPIO_MASK_PB_D                                     0x00000010                         ///< Mask for GPIO -- "Down" Pushbutton
#define GPIO_MASK_DS_3                                     0x00000008                         ///< Mask for GPIO -- MSB of Dip Switch

#define UART_BUFFER_SIZE                                   1                                  ///< UART is configured to read 1 byte at a time

#define NUM_VALID_RATES                                    12                                 ///< Number of supported rates


//-----------------------------------------------
// Callback Return Flags
//
#define MAC_RX_CALLBACK_RETURN_FLAG_DUP							0x00000001
#define MAC_RX_CALLBACK_RETURN_FLAG_NO_COUNTS					0x00000002
#define MAC_RX_CALLBACK_RETURN_FLAG_NO_LOG_ENTRY				0x00000004


/***************************** Include Files *********************************/

/********************************************************************
 * Include other framework headers
 *
 * NOTE:  Includes have to be after any #define that are needed in the typedefs within the includes.
 *
 ********************************************************************/
#include "wlan_mac_queue.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_mailbox_util.h"
#include "wlan_mac_dl_list.h"



/************************** Global Type Definitions **************************/

typedef enum {INTERRUPTS_DISABLED, INTERRUPTS_ENABLED} interrupt_state_t;



/******************** Global Structure/Enum Definitions **********************/

/***************************** Global Constants ******************************/

extern const  u8 bcast_addr[MAC_ADDR_LEN];
extern const  u8 zero_addr[MAC_ADDR_LEN];


/*************************** Function Prototypes *****************************/
void               wlan_mac_high_init();
void 			   wlan_mac_high_malloc_init();

int                wlan_mac_high_interrupt_init();

int                wlan_mac_high_interrupt_restore_state(interrupt_state_t new_interrupt_state);
interrupt_state_t  wlan_mac_high_interrupt_stop();

void               wlan_mac_high_uart_rx_handler(void *CallBackRef, unsigned int EventData);
void               wlan_mac_high_userio_gpio_handler(void *InstancePtr);

dl_entry*          wlan_mac_high_find_counts_ADDR(dl_list* list, u8* addr);

u32                wlan_mac_high_get_user_io_state();

void               wlan_mac_high_set_pb_u_callback(function_ptr_t callback);
void               wlan_mac_high_set_pb_m_callback(function_ptr_t callback);
void               wlan_mac_high_set_pb_d_callback(function_ptr_t callback);
void               wlan_mac_high_set_uart_rx_callback(function_ptr_t callback);
void               wlan_mac_high_set_mpdu_tx_high_done_callback(function_ptr_t callback);
void               wlan_mac_high_set_mpdu_tx_low_done_callback(function_ptr_t callback);
void               wlan_mac_high_set_beacon_tx_done_callback(function_ptr_t callback);
void 			   wlan_mac_high_set_mpdu_rx_callback(function_ptr_t callback);
void               wlan_mac_high_set_poll_tx_queues_callback(function_ptr_t callback);
void               wlan_mac_high_set_mpdu_dequeue_callback(function_ptr_t callback);
void 			   wlan_mac_high_set_cpu_low_reboot_callback(function_ptr_t callback);

void*              wlan_mac_high_malloc(u32 size);
void*              wlan_mac_high_calloc(u32 size);
void*              wlan_mac_high_realloc(void* addr, u32 size);
void               wlan_mac_high_free(void* addr);
void               wlan_mac_high_display_mallinfo();

int                wlan_mac_high_memory_test();
int                wlan_mac_high_right_shift_test();

int                wlan_mac_high_cdma_start_transfer(void* dest, void* src, u32 size);
void               wlan_mac_high_cdma_finish_transfer();

void               wlan_mac_high_mpdu_transmit(dl_entry* packet, int tx_pkt_buf);

u8                 wlan_mac_high_valid_tagged_rate(u8 rate);
void               wlan_mac_high_tagged_rate_to_readable_rate(u8 rate, char* str);

void               wlan_mac_high_setup_tx_header( mac_header_80211_common * header, u8 * addr_1, u8 * addr_3 );
void 			   wlan_mac_high_setup_tx_frame_info(mac_header_80211_common * header, dl_entry * curr_tx_queue_element, u32 tx_length, u8 flags, u8 queue_id, pkt_buf_group_t pkt_buf_group);

void               wlan_mac_high_ipc_rx();
void               wlan_mac_high_process_ipc_msg(wlan_ipc_msg_t * msg);

void               wlan_mac_high_set_srand(u32 seed);
u8                 wlan_mac_high_bss_channel_spec_to_radio_chan(chan_spec_t chan_spec);
void               wlan_mac_high_set_radio_channel(u32 mac_channel);
void 			   wlan_mac_high_enable_mcast_buffering(u8 enable);
void               wlan_mac_high_config_txrx_beacon(beacon_txrx_configure_t* beacon_txrx_configure);
void               wlan_mac_high_set_rx_ant_mode(u8 ant_mode);
void               wlan_mac_high_set_tx_ctrl_pow(s8 pow);
void               wlan_mac_high_set_rx_filter_mode(u32 filter_mode);
void               wlan_mac_high_set_dsss(u32 dsss_value);

int                wlan_mac_high_write_low_mem(u32 num_words, u32* payload);
int                wlan_mac_high_read_low_mem(u32 num_words, u32 baseaddr, u32* payload);
int                wlan_mac_high_write_low_param(u32 num_words, u32* payload);

void               wlan_mac_high_request_low_state();
int 			   wlan_mac_high_is_cpu_low_initialized();
int         	   wlan_mac_high_is_dequeue_allowed(pkt_buf_group_t pkt_buf_group);
int                wlan_mac_high_get_empty_tx_packet_buffer();
u8                 wlan_mac_high_is_pkt_ltg(void* mac_payload, u16 length);


int                wlan_mac_high_configure_beacon_tx_template(mac_header_80211_common* tx_header_common_ptr, bss_info_t* bss_info, tx_params_t* tx_params_ptr, u8 flags);
int                wlan_mac_high_update_beacon_tx_params(tx_params_t* tx_params_ptr);


void               wlan_mac_high_print_station_infos(dl_list* assoc_tbl);


// Common functions that must be implemented by users of the framework
// TODO: Make these into callback. We should not require these implementations
dl_list*          get_bss_member_list();

#endif /* WLAN_MAC_HIGH_H_ */
