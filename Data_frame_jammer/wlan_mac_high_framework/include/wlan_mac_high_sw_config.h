/** @file wlan_mac_high_sw_config.h
 *  @brief Software Configuration Options
 *
 *  This contains definitions that affect the size of the CPU_HIGH projects.
 *
 *  @copyright Copyright 2013-2017, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *              See LICENSE.txt included in the design archive or
 *              at http://mangocomm.com/802.11/license
 *
 *  This file is part of the Mango 802.11 Reference Design (https://mangocomm.com/802.11)
 */

#ifndef WLAN_MAC_HIGH_SW_CONFIG_H_
#define WLAN_MAC_HIGH_SW_CONFIG_H_


//---------- COMPILATION TOGGLES ----------
//  The following toggles directly affect the size of the .text section after compilation.
//  They also implicitly affect DRAM usage since DRAM is used for the storage of
//  station_info_t structs as well as Tx/Rx logs.


#define WLAN_SW_CONFIG_ENABLE_WLAN_EXP      1       //Top-level switch for compiling wlan_exp. Setting to 0 implicitly removes
                                                    // logging code if set to 0 since there would be no way to retrieve the log.

#define WLAN_SW_CONFIG_ENABLE_TXRX_COUNTS   1       //Top-level switch for compiling counts_txrx.  Setting to 0 removes counts
                                                    // from station_info_t struct definition and disables counts retrieval via
                                                    // wlan_exp.

#define WLAN_SW_CONFIG_ENABLE_LOGGING       1       //Top-level switch for compiling Tx/Rx logging. Setting to 0 will not cause
                                                    // the design to not log any entries to DRAM. It will also disable any log
                                                    // retrieval capabilities in wlan_exp. Note: this is logically distinct from
                                                    // WLAN_SW_CONFIG_ENABLE_WLAN_EXP. (WLAN_SW_CONFIG_ENABLE_WLAN_EXP 1, COMPILE_LOGGING 0)
													// still allows wlan_exp control over a node but no logging capabilities.

#define WLAN_SW_CONFIG_ENABLE_LTG           1       //Top-level switch for compiling LTG functionality. Setting to 0 will remove
                                                    // all LTG-related code from the design as well we disable any wlan_exp
                                                    // commands that control LTGs.

#define WLAN_SW_CONFIG_ENABLE_ETH_BRIDGE	1		//Top-level switch for compiling Ethernet bridging functionality.

#endif /* WLAN_MAC_HIGH_SW_CONFIG_H_ */
