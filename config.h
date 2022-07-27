// this file is fairly empty. The SD card is used to read the configuration.

/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

// LoRaWAN region to use, full list of regions can be found at: 
//   http://stackforce.github.io/LoRaMac-doc/LoRaMac-doc-v4.5.1/group___l_o_r_a_m_a_c.html#ga3b9d54f0355b51e85df8b33fd1757eec
//#define LORAWAN_REGION          LORAMAC_REGION_EU868

// LoRaWAN Device EUI (64-bit), NULL value will use Default Dev EUI
//#define LORAWAN_DEVICE_EUI      "1234560000000001"

// LoRaWAN Application / Join EUI (64-bit)
//#define LORAWAN_APP_EUI         "1234570000000100"

// LoRaWAN Application Key (128-bit)
//#define LORAWAN_APP_KEY         "11223344556677889900aabbccddeeff"

// LoRaWAN Channel Mask, NULL value will use the default channel mask 
// for the region
#define LORAWAN_CHANNEL_MASK    NULL
