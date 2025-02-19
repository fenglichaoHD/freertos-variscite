/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v9.0
processor: MIMX8MQ6xxxJZ
package_id: MIMX8MQ6DVAJZ
mcu_data: ksdk2_0
processor_version: 9.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm4}
- pin_list:
  - {pin_num: B6, peripheral: UART2, signal: uart_rx, pin_signal: UART2_RXD, PUE: Enabled, SRE: MEDIUM, DSE: OHM_45}
  - {pin_num: D6, peripheral: UART2, signal: uart_tx, pin_signal: UART2_TXD, PUE: Enabled, SRE: MEDIUM, DSE: OHM_45}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {                                /*!< Function assigned for the core: Cortex-M4[m4] */
    IOMUXC_SetPinMux(IOMUXC_UART2_RXD_UART2_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART2_RXD_UART2_RX, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
    IOMUXC_SetPinMux(IOMUXC_UART2_TXD_UART2_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART2_TXD_UART2_TX, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C3_InitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: G7, peripheral: I2C3, signal: i2c_scl, pin_signal: I2C3_SCL, ODE: Enabled, SION: ENABLED, HYS: Enabled, SRE: MEDIUM, DSE: OHM_45}
  - {pin_num: F7, peripheral: I2C3, signal: i2c_sda, pin_signal: I2C3_SDA, ODE: Enabled, SION: ENABLED, HYS: Enabled, SRE: MEDIUM, DSE: OHM_45}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C3_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C3_InitPins(void) {                                 /*!< Function assigned for the core: Cortex-M4[m4] */
    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_I2C3_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SCL_I2C3_SCL, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_I2C3_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SDA_I2C3_SDA, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C3_DeinitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: G7, peripheral: I2C3, signal: i2c_scl, pin_signal: I2C3_SCL}
  - {pin_num: F7, peripheral: I2C3, signal: i2c_sda, pin_signal: I2C3_SDA}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C3_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C3_DeinitPins(void) {                               /*!< Function assigned for the core: Cortex-M4[m4] */
    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_I2C3_SCL, 0U);
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_I2C3_SDA, 0U);
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
