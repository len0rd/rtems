/**
 * @file
 *
 * @ingroup RTEMSBSPsARMTMS570
 *
 * @brief Specification of individual pins mapping to the package
 */

/*
 * Copyright (c) 2015 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 * Czech Technical University in Prague
 * Zikova 1903/4
 * 166 36 Praha 6
 * Czech Republic
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_TMS570_TMS570LS3137ZWT_PINS_H
#define LIBBSP_ARM_TMS570_TMS570LS3137ZWT_PINS_H

#define TMS570_BALL_W10 TMS570_BALL_WITH_MMR(0, 0)
#define TMS570_BALL_W10_GIOB_3 TMS570_PIN_AND_FNC(TMS570_BALL_W10, 0)

#define TMS570_BALL_A5 TMS570_BALL_WITH_MMR(0, 1)
#define TMS570_BALL_A5_GIOA_0 TMS570_PIN_AND_FNC(TMS570_BALL_A5, 0)

#define TMS570_BALL_C3 TMS570_BALL_WITH_MMR(0, 2)
#define TMS570_BALL_C3_MIBSPI3NCS_3 TMS570_PIN_AND_FNC(TMS570_BALL_C3, 0)
#define TMS570_BALL_C3_I2C_SCL TMS570_PIN_AND_FNC(TMS570_BALL_C3, 1)
#define TMS570_BALL_C3_HET1_29 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_C3, 2), \
                TMS570_PIN_AND_FNC(TMS570_BALL_A3, 0) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_B2 TMS570_BALL_WITH_MMR(0, 3)
#define TMS570_BALL_B2_MIBSPI3NCS_2 TMS570_PIN_AND_FNC(TMS570_BALL_B2, 0)
#define TMS570_BALL_B2_I2C_SDA TMS570_PIN_AND_FNC(TMS570_BALL_B2, 1)
#define TMS570_BALL_B2_HET1_27 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_B2, 2), \
                TMS570_PIN_AND_FNC(TMS570_BALL_A9, 0) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_C2 TMS570_BALL_WITH_MMR(1, 0)
#define TMS570_BALL_C2_GIOA_1 TMS570_PIN_AND_FNC(TMS570_BALL_C2, 0)

#define TMS570_BALL_E3 TMS570_BALL_WITH_MMR(1, 1)
#define TMS570_BALL_E3_HET1_11 TMS570_PIN_AND_FNC(TMS570_BALL_E3, 0)
#define TMS570_BALL_E3_MIBSPI3NCS_4 TMS570_PIN_AND_FNC(TMS570_BALL_E3, 1)
#define TMS570_BALL_E3_HET2_18 TMS570_PIN_AND_FNC(TMS570_BALL_E3, 2)

#define TMS570_BALL_E5 TMS570_BALL_WITH_MMR(1, 2)
#define TMS570_BALL_E5_ETMDATA_20 TMS570_PIN_AND_FNC(TMS570_BALL_E5, 0)
#define TMS570_BALL_E5_EMIF_DATA_4 TMS570_PIN_AND_FNC(TMS570_BALL_E5, 1)

#define TMS570_BALL_F5 TMS570_BALL_WITH_MMR(1, 3)
#define TMS570_BALL_F5_ETMDATA_21 TMS570_PIN_AND_FNC(TMS570_BALL_F5, 0)
#define TMS570_BALL_F5_EMIF_DATA_5 TMS570_PIN_AND_FNC(TMS570_BALL_F5, 1)

#define TMS570_BALL_C1 TMS570_BALL_WITH_MMR(2, 0)
#define TMS570_BALL_C1_GIOA_2 TMS570_PIN_AND_FNC(TMS570_BALL_C1, 0)
#define TMS570_BALL_C1_HET2_00 TMS570_PIN_AND_FNC(TMS570_BALL_C1, 3)

#define TMS570_BALL_G5 TMS570_BALL_WITH_MMR(2, 1)
#define TMS570_BALL_G5_ETMDATA_22 TMS570_PIN_AND_FNC(TMS570_BALL_G5, 0)
#define TMS570_BALL_G5_EMIF_DATA_6 TMS570_PIN_AND_FNC(TMS570_BALL_G5, 1)

#define TMS570_BALL_E1 TMS570_BALL_WITH_MMR(2, 2)
#define TMS570_BALL_E1_GIOA_3 TMS570_PIN_AND_FNC(TMS570_BALL_E1, 0)
#define TMS570_BALL_E1_HET2_02 TMS570_PIN_AND_FNC(TMS570_BALL_E1, 1)

#define TMS570_BALL_B5 TMS570_BALL_WITH_MMR(2, 3)
#define TMS570_BALL_B5_GIOA_5 TMS570_PIN_AND_FNC(TMS570_BALL_B5, 0)
#define TMS570_BALL_B5_EXTCLKIN TMS570_PIN_AND_FNC(TMS570_BALL_B5, 1)

#define TMS570_BALL_K5 TMS570_BALL_WITH_MMR(3, 0)
#define TMS570_BALL_K5_ETMDATA_23 TMS570_PIN_AND_FNC(TMS570_BALL_K5, 0)
#define TMS570_BALL_K5_EMIF_DATA_7 TMS570_PIN_AND_FNC(TMS570_BALL_K5, 1)

#define TMS570_BALL_B3 TMS570_BALL_WITH_MMR(3, 1)
#define TMS570_BALL_B3_HET1_22 TMS570_PIN_AND_FNC(TMS570_BALL_B3, 0)

#define TMS570_BALL_H3 TMS570_BALL_WITH_MMR(3, 2)
#define TMS570_BALL_H3_GIOA_6 TMS570_PIN_AND_FNC(TMS570_BALL_H3, 0)
#define TMS570_BALL_H3_HET2_04 TMS570_PIN_AND_FNC(TMS570_BALL_H3, 1)

#define TMS570_BALL_L5 TMS570_BALL_WITH_MMR(3, 3)
#define TMS570_BALL_L5_ETMDATA_24 TMS570_PIN_AND_FNC(TMS570_BALL_L5, 0)
#define TMS570_BALL_L5_EMIF_DATA_8 TMS570_PIN_AND_FNC(TMS570_BALL_L5, 1)

#define TMS570_BALL_M1 TMS570_BALL_WITH_MMR(4, 0)
#define TMS570_BALL_M1_GIOA_7 TMS570_PIN_AND_FNC(TMS570_BALL_M1, 0)
#define TMS570_BALL_M1_HET2_06 TMS570_PIN_AND_FNC(TMS570_BALL_M1, 1)

#define TMS570_BALL_M5 TMS570_BALL_WITH_MMR(4, 1)
#define TMS570_BALL_M5_ETMDATA_25 TMS570_PIN_AND_FNC(TMS570_BALL_M5, 0)
#define TMS570_BALL_M5_EMIF_DATA_9 TMS570_PIN_AND_FNC(TMS570_BALL_M5, 1)

#define TMS570_BALL_V2 TMS570_BALL_WITH_MMR(4, 2)
#define TMS570_BALL_V2_HET1_01 TMS570_PIN_AND_FNC(TMS570_BALL_V2, 0)
#define TMS570_BALL_V2_SPI4NENA TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_V2, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(24, 0), 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_V2_HET2_08 TMS570_PIN_AND_FNC(TMS570_BALL_V2, 4)

#define TMS570_BALL_U1 TMS570_BALL_WITH_MMR(4, 3)
#define TMS570_BALL_U1_HET1_03 TMS570_PIN_AND_FNC(TMS570_BALL_U1, 0)
#define TMS570_BALL_U1_SPI4NCS_0 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_U1, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(24, 1), 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_U1_HET2_10 TMS570_PIN_AND_FNC(TMS570_BALL_U1, 4)

#define TMS570_BALL_K18 TMS570_BALL_WITH_MMR(5, 0)
#define TMS570_BALL_K18_HET1_00 TMS570_PIN_AND_FNC(TMS570_BALL_K18, 0)
#define TMS570_BALL_K18_SPI4CLK TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_K18, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(23, 1), 0) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_W5 TMS570_BALL_WITH_MMR(5, 1)
#define TMS570_BALL_W5_HET1_02 TMS570_PIN_AND_FNC(TMS570_BALL_W5, 0)
#define TMS570_BALL_W5_SPI4SIMO TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_W5, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(23, 2), 0) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_V6 TMS570_BALL_WITH_MMR(5, 2)
#define TMS570_BALL_V6_HET1_05 TMS570_PIN_AND_FNC(TMS570_BALL_V6, 0)
#define TMS570_BALL_V6_SPI4SOMI TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_V6, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(23, 3), 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_V6_HET2_12 TMS570_PIN_AND_FNC(TMS570_BALL_V6, 2)

#define TMS570_BALL_N5 TMS570_BALL_WITH_MMR(5, 3)
#define TMS570_BALL_N5_ETMDATA_26 TMS570_PIN_AND_FNC(TMS570_BALL_N5, 0)
#define TMS570_BALL_N5_EMIF_DATA_10 TMS570_PIN_AND_FNC(TMS570_BALL_N5, 1)

#define TMS570_BALL_T1 TMS570_BALL_WITH_MMR(6, 0)
#define TMS570_BALL_T1_HET1_07 TMS570_PIN_AND_FNC(TMS570_BALL_T1, 0)
#define TMS570_BALL_T1_HET2_14 TMS570_PIN_AND_FNC(TMS570_BALL_T1, 3)

#define TMS570_BALL_P5 TMS570_BALL_WITH_MMR(6, 1)
#define TMS570_BALL_P5_ETMDATA_27 TMS570_PIN_AND_FNC(TMS570_BALL_P5, 0)
#define TMS570_BALL_P5_EMIF_DATA_11 TMS570_PIN_AND_FNC(TMS570_BALL_P5, 1)

#define TMS570_BALL_V7 TMS570_BALL_WITH_MMR(6, 2)
#define TMS570_BALL_V7_HET1_09 TMS570_PIN_AND_FNC(TMS570_BALL_V7, 0)
#define TMS570_BALL_V7_HET2_16 TMS570_PIN_AND_FNC(TMS570_BALL_V7, 1)

#define TMS570_BALL_R5 TMS570_BALL_WITH_MMR(6, 3)
#define TMS570_BALL_R5_ETMDATA_28 TMS570_PIN_AND_FNC(TMS570_BALL_R5, 0)
#define TMS570_BALL_R5_EMIF_DATA_12 TMS570_PIN_AND_FNC(TMS570_BALL_R5, 1)

#define TMS570_BALL_R6 TMS570_BALL_WITH_MMR(7, 0)
#define TMS570_BALL_R6_ETMDATA_29 TMS570_PIN_AND_FNC(TMS570_BALL_R6, 0)
#define TMS570_BALL_R6_EMIF_DATA_13 TMS570_PIN_AND_FNC(TMS570_BALL_R6, 1)

#define TMS570_BALL_V5 TMS570_BALL_WITH_MMR(7, 1)
#define TMS570_BALL_V5_MIBSPI3NCS_1 TMS570_PIN_AND_FNC(TMS570_BALL_V5, 0)
#define TMS570_BALL_V5_HET1_25 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_V5, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_M3, 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_V5_MDCLK TMS570_PIN_AND_FNC(TMS570_BALL_V5, 2)

#define TMS570_BALL_W3 TMS570_BALL_WITH_MMR(7, 2)
#define TMS570_BALL_W3_HET1_06 TMS570_PIN_AND_FNC(TMS570_BALL_W3, 0)
#define TMS570_BALL_W3_SCIRX TMS570_PIN_AND_FNC(TMS570_BALL_W3, 1)

#define TMS570_BALL_R7 TMS570_BALL_WITH_MMR(7, 3)
#define TMS570_BALL_R7_ETMDATA_30 TMS570_PIN_AND_FNC(TMS570_BALL_R7, 0)
#define TMS570_BALL_R7_EMIF_DATA_14 TMS570_PIN_AND_FNC(TMS570_BALL_R7, 1)

#define TMS570_BALL_N2 TMS570_BALL_WITH_MMR(8, 0)
#define TMS570_BALL_N2_HET1_13 TMS570_PIN_AND_FNC(TMS570_BALL_N2, 0)
#define TMS570_BALL_N2_SCITX TMS570_PIN_AND_FNC(TMS570_BALL_N2, 1)

#define TMS570_BALL_G3 TMS570_BALL_WITH_MMR(8, 1)
#define TMS570_BALL_G3_MIBSPI1NCS_2 TMS570_PIN_AND_FNC(TMS570_BALL_G3, 0)
#define TMS570_BALL_G3_HET1_19 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_G3, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_B13, 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_G3_MDIO TMS570_PIN_AND_FNC(TMS570_BALL_G3, 2)

#define TMS570_BALL_N1 TMS570_BALL_WITH_MMR(8, 2)
#define TMS570_BALL_N1_HET1_15 TMS570_PIN_AND_FNC(TMS570_BALL_N1, 0)
#define TMS570_BALL_N1_MIBSPI1NCS_4 TMS570_PIN_AND_FNC(TMS570_BALL_N1, 1)

#define TMS570_BALL_R8 TMS570_BALL_WITH_MMR(8, 3)
#define TMS570_BALL_R8_ETMDATA_31 TMS570_PIN_AND_FNC(TMS570_BALL_R8, 0)
#define TMS570_BALL_R8_EMIF_DATA_15 TMS570_PIN_AND_FNC(TMS570_BALL_R8, 1)

#define TMS570_BALL_R9 TMS570_BALL_WITH_MMR(9, 0)
#define TMS570_BALL_R9_ETMTRACECLKIN TMS570_PIN_AND_FNC(TMS570_BALL_R9, 0)
#define TMS570_BALL_R9_EXTCLKIN2 TMS570_PIN_AND_FNC(TMS570_BALL_R9, 1)

#define TMS570_BALL_W9 TMS570_BALL_WITH_MMR(9, 1)
#define TMS570_BALL_W9_MIBSPI3NENA TMS570_PIN_AND_FNC(TMS570_BALL_W9, 0)
#define TMS570_BALL_W9_MIBSPI3NCS_5 TMS570_PIN_AND_FNC(TMS570_BALL_W9, 1)
#define TMS570_BALL_W9_HET1_31 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_W9, 2), \
                TMS570_PIN_AND_FNC(TMS570_BALL_J17, 0) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_V10 TMS570_BALL_WITH_MMR(9, 2)
#define TMS570_BALL_V10_MIBSPI3NCS_0 TMS570_PIN_AND_FNC(TMS570_BALL_V10, 0)
#define TMS570_BALL_V10_AD2EVT TMS570_PIN_AND_FNC(TMS570_BALL_V10, 1)

#define TMS570_BALL_J3 TMS570_BALL_WITH_MMR(9, 3)
#define TMS570_BALL_J3_MIBSPI1NCS_3 TMS570_PIN_AND_FNC(TMS570_BALL_J3, 0)
#define TMS570_BALL_J3_HET1_21 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_J3, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_H4, 0) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_N19 TMS570_BALL_WITH_MMR(10, 0)
#define TMS570_BALL_N19_AD1EVT TMS570_PIN_AND_FNC(TMS570_BALL_N19, 0)
#define TMS570_BALL_N19_MII_RX_ER TMS570_PIN_AND_FNC(TMS570_BALL_N19, 1)
#define TMS570_BALL_N19_RMII_RX_ER TMS570_PIN_AND_FNC(TMS570_BALL_N19, 2)

#define TMS570_BALL_N15 TMS570_BALL_WITH_MMR(10, 1)
#define TMS570_BALL_N15_ETMDATA_19 TMS570_PIN_AND_FNC(TMS570_BALL_N15, 0)
#define TMS570_BALL_N15_EMIF_DATA_3 TMS570_PIN_AND_FNC(TMS570_BALL_N15, 1)

#define TMS570_BALL_N17 TMS570_BALL_WITH_MMR(10, 2)
#define TMS570_BALL_N17_EMIF_nCS_0 TMS570_PIN_AND_FNC(TMS570_BALL_N17, 0)
#define TMS570_BALL_N17_RTP_DATA_15 TMS570_PIN_AND_FNC(TMS570_BALL_N17, 1)
#define TMS570_BALL_N17_HET2_07 TMS570_PIN_AND_FNC(TMS570_BALL_N17, 2)

#define TMS570_BALL_M15 TMS570_BALL_WITH_MMR(10, 3)
#define TMS570_BALL_M15_ETMDATA_18 TMS570_PIN_AND_FNC(TMS570_BALL_M15, 0)
#define TMS570_BALL_M15_EMIF_DATA_2 TMS570_PIN_AND_FNC(TMS570_BALL_M15, 1)

#define TMS570_BALL_K17 TMS570_BALL_WITH_MMR(11, 0)
#define TMS570_BALL_K17_EMIF_nCS_3 TMS570_PIN_AND_FNC(TMS570_BALL_K17, 0)
#define TMS570_BALL_K17_RTP_DATA_14 TMS570_PIN_AND_FNC(TMS570_BALL_K17, 1)
#define TMS570_BALL_K17_HET2_09 TMS570_PIN_AND_FNC(TMS570_BALL_K17, 2)

#define TMS570_BALL_M17 TMS570_BALL_WITH_MMR(11, 1)
#define TMS570_BALL_M17_EMIF_nCS_4 TMS570_PIN_AND_FNC(TMS570_BALL_M17, 0)
#define TMS570_BALL_M17_RTP_DATA_07 TMS570_PIN_AND_FNC(TMS570_BALL_M17, 1)

#define TMS570_BALL_L15 TMS570_BALL_WITH_MMR(11, 2)
#define TMS570_BALL_L15_ETMDATA_17 TMS570_PIN_AND_FNC(TMS570_BALL_L15, 0)
#define TMS570_BALL_L15_EMIF_DATA_1 TMS570_PIN_AND_FNC(TMS570_BALL_L15, 1)

#define TMS570_BALL_P1 TMS570_BALL_WITH_MMR(11, 3)
#define TMS570_BALL_P1_HET1_24 TMS570_PIN_AND_FNC(TMS570_BALL_P1, 0)
#define TMS570_BALL_P1_MIBSPI1NCS_5 TMS570_PIN_AND_FNC(TMS570_BALL_P1, 1)
#define TMS570_BALL_P1_MII_RXD_0 TMS570_PIN_AND_FNC(TMS570_BALL_P1, 2)
#define TMS570_BALL_P1_RMII_RXD_0 TMS570_PIN_AND_FNC(TMS570_BALL_P1, 3)

#define TMS570_BALL_A14 TMS570_BALL_WITH_MMR(12, 0)
#define TMS570_BALL_A14_HET1_26 TMS570_PIN_AND_FNC(TMS570_BALL_A14, 0)
#define TMS570_BALL_A14_MII_RXD_1 TMS570_PIN_AND_FNC(TMS570_BALL_A14, 1)
#define TMS570_BALL_A14_RMII_RXD_1 TMS570_PIN_AND_FNC(TMS570_BALL_A14, 2)

#define TMS570_BALL_K15 TMS570_BALL_WITH_MMR(12, 1)
#define TMS570_BALL_K15_ETMDATA_16 TMS570_PIN_AND_FNC(TMS570_BALL_K15, 0)
#define TMS570_BALL_K15_EMIF_DATA_0 TMS570_PIN_AND_FNC(TMS570_BALL_K15, 1)

#define TMS570_BALL_G19 TMS570_BALL_WITH_MMR(12, 2)
#define TMS570_BALL_G19_MIBSPI1NENA TMS570_PIN_AND_FNC(TMS570_BALL_G19, 0)
#define TMS570_BALL_G19_HET1_23 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_G19, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_J4, 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_G19_MII_RXD_2 TMS570_PIN_AND_FNC(TMS570_BALL_G19, 2)

#define TMS570_BALL_H18 TMS570_BALL_WITH_MMR(12, 3)
#define TMS570_BALL_H18_MIBSPI5NENA TMS570_PIN_AND_FNC(TMS570_BALL_H18, 0)
#define TMS570_BALL_H18_DMM_DATA_7 TMS570_PIN_AND_FNC(TMS570_BALL_H18, 1)
#define TMS570_BALL_H18_MII_RXD_3 TMS570_PIN_AND_FNC(TMS570_BALL_H18, 2)

#define TMS570_BALL_J18 TMS570_BALL_WITH_MMR(13, 0)
#define TMS570_BALL_J18_MIBSPI5SOMI_0 TMS570_PIN_AND_FNC(TMS570_BALL_J18, 0)
#define TMS570_BALL_J18_DMM_DATA_12 TMS570_PIN_AND_FNC(TMS570_BALL_J18, 1)
#define TMS570_BALL_J18_MII_TXD_0 TMS570_PIN_AND_FNC(TMS570_BALL_J18, 2)
#define TMS570_BALL_J18_RMII_TXD_0 TMS570_PIN_AND_FNC(TMS570_BALL_J18, 3)

#define TMS570_BALL_J19 TMS570_BALL_WITH_MMR(13, 1)
#define TMS570_BALL_J19_MIBSPI5SIMO_0 TMS570_PIN_AND_FNC(TMS570_BALL_J19, 0)
#define TMS570_BALL_J19_DMM_DATA_8 TMS570_PIN_AND_FNC(TMS570_BALL_J19, 1)
#define TMS570_BALL_J19_MII_TXD_1 TMS570_PIN_AND_FNC(TMS570_BALL_J19, 2)
#define TMS570_BALL_J19_RMII_TXD_1 TMS570_PIN_AND_FNC(TMS570_BALL_J19, 3)

#define TMS570_BALL_H19 TMS570_BALL_WITH_MMR(13, 2)
#define TMS570_BALL_H19_MIBSPI5CLK TMS570_PIN_AND_FNC(TMS570_BALL_H19, 0)
#define TMS570_BALL_H19_DMM_DATA_4 TMS570_PIN_AND_FNC(TMS570_BALL_H19, 1)
#define TMS570_BALL_H19_MII_TXEN TMS570_PIN_AND_FNC(TMS570_BALL_H19, 2)
#define TMS570_BALL_H19_RMII_TXEN TMS570_PIN_AND_FNC(TMS570_BALL_H19, 3)

#define TMS570_BALL_R2 TMS570_BALL_WITH_MMR(13, 3)
#define TMS570_BALL_R2_MIBSPI1NCS_0 TMS570_PIN_AND_FNC(TMS570_BALL_R2, 0)
#define TMS570_BALL_R2_MIBSPI1SOMI_1 TMS570_PIN_AND_FNC(TMS570_BALL_R2, 1)
#define TMS570_BALL_R2_MII_TXD_2 TMS570_PIN_AND_FNC(TMS570_BALL_R2, 2)

#define TMS570_BALL_E18 TMS570_BALL_WITH_MMR(14, 0)
#define TMS570_BALL_E18_HET1_08 TMS570_PIN_AND_FNC(TMS570_BALL_E18, 0)
#define TMS570_BALL_E18_MIBSPI1SIMO_1 TMS570_PIN_AND_FNC(TMS570_BALL_E18, 1)
#define TMS570_BALL_E18_MII_TXD_3 TMS570_PIN_AND_FNC(TMS570_BALL_E18, 2)

#define TMS570_BALL_K19 TMS570_BALL_WITH_MMR(14, 1)
#define TMS570_BALL_K19_HET1_28 TMS570_PIN_AND_FNC(TMS570_BALL_K19, 0)
#define TMS570_BALL_K19_MII_RXCLK TMS570_PIN_AND_FNC(TMS570_BALL_K19, 1)
#define TMS570_BALL_K19_RMII_REFCLK TMS570_PIN_AND_FNC(TMS570_BALL_K19, 2)
#define TMS570_BALL_K19_MII_RX_AVCLK4 TMS570_PIN_AND_FNC(TMS570_BALL_K19, 3)

#define TMS570_BALL_D17 TMS570_BALL_WITH_MMR(14, 2)
#define TMS570_BALL_D17_EMIF_nWE TMS570_PIN_AND_FNC(TMS570_BALL_D17, 0)
#define TMS570_BALL_D17_EMIF_RNW TMS570_PIN_AND_FNC(TMS570_BALL_D17, 1)

#define TMS570_BALL_D16 TMS570_BALL_WITH_MMR(14, 3)
#define TMS570_BALL_D16_EMIF_BA_1 TMS570_PIN_AND_FNC(TMS570_BALL_D16, 0)
#define TMS570_BALL_D16_HET2_05 TMS570_PIN_AND_FNC(TMS570_BALL_D16, 1)

#define TMS570_BALL_C17 TMS570_BALL_WITH_MMR(15, 0)
#define TMS570_BALL_C17_EMIF_ADDR_21 TMS570_PIN_AND_FNC(TMS570_BALL_C17, 0)
#define TMS570_BALL_C17_RTP_CLK TMS570_PIN_AND_FNC(TMS570_BALL_C17, 1)

#define TMS570_BALL_C16 TMS570_BALL_WITH_MMR(15, 1)
#define TMS570_BALL_C16_EMIF_ADDR_20 TMS570_PIN_AND_FNC(TMS570_BALL_C16, 0)
#define TMS570_BALL_C16_RTP_nSYNC TMS570_PIN_AND_FNC(TMS570_BALL_C16, 1)

#define TMS570_BALL_C15 TMS570_BALL_WITH_MMR(15, 2)
#define TMS570_BALL_C15_EMIF_ADDR_19 TMS570_PIN_AND_FNC(TMS570_BALL_C15, 0)
#define TMS570_BALL_C15_RTP_nENA TMS570_PIN_AND_FNC(TMS570_BALL_C15, 1)

#define TMS570_BALL_D15 TMS570_BALL_WITH_MMR(15, 3)
#define TMS570_BALL_D15_EMIF_ADDR_18 TMS570_PIN_AND_FNC(TMS570_BALL_D15, 0)
#define TMS570_BALL_D15_RTP_DATA_0 TMS570_PIN_AND_FNC(TMS570_BALL_D15, 1)

#define TMS570_BALL_E13 TMS570_BALL_WITH_MMR(16, 0)
#define TMS570_BALL_E13_ETMDATA_12 TMS570_PIN_AND_FNC(TMS570_BALL_E13, 0)
#define TMS570_BALL_E13_EMIF_BA_0 TMS570_PIN_AND_FNC(TMS570_BALL_E13, 1)

#define TMS570_BALL_C14 TMS570_BALL_WITH_MMR(16, 1)
#define TMS570_BALL_C14_EMIF_ADDR_17 TMS570_PIN_AND_FNC(TMS570_BALL_C14, 0)
#define TMS570_BALL_C14_RTP_DATA_01 TMS570_PIN_AND_FNC(TMS570_BALL_C14, 1)

#define TMS570_BALL_D14 TMS570_BALL_WITH_MMR(16, 2)
#define TMS570_BALL_D14_EMIF_ADDR_16 TMS570_PIN_AND_FNC(TMS570_BALL_D14, 0)
#define TMS570_BALL_D14_RTP_DATA_02 TMS570_PIN_AND_FNC(TMS570_BALL_D14, 1)

#define TMS570_BALL_E12 TMS570_BALL_WITH_MMR(16, 3)
#define TMS570_BALL_E12_ETMDATA_13 TMS570_PIN_AND_FNC(TMS570_BALL_E12, 0)
#define TMS570_BALL_E12_EMIF_nOE TMS570_PIN_AND_FNC(TMS570_BALL_E12, 1)

#define TMS570_BALL_D19 TMS570_BALL_WITH_MMR(17, 0)
#define TMS570_BALL_D19_HET1_10 TMS570_PIN_AND_FNC(TMS570_BALL_D19, 0)
#define TMS570_BALL_D19_MII_TX_CLK TMS570_PIN_AND_FNC(TMS570_BALL_D19, 1)
#define TMS570_BALL_D19_MII_TX_AVCLK4 TMS570_PIN_AND_FNC(TMS570_BALL_D19, 3)

#define TMS570_BALL_E11 TMS570_BALL_WITH_MMR(17, 1)
#define TMS570_BALL_E11_ETMDATA_14 TMS570_PIN_AND_FNC(TMS570_BALL_E11, 0)
#define TMS570_BALL_E11_EMIF_nDQM_1 TMS570_PIN_AND_FNC(TMS570_BALL_E11, 1)

#define TMS570_BALL_B4 TMS570_BALL_WITH_MMR(17, 2)
#define TMS570_BALL_B4_HET1_12 TMS570_PIN_AND_FNC(TMS570_BALL_B4, 0)
#define TMS570_BALL_B4_MII_CRS TMS570_PIN_AND_FNC(TMS570_BALL_B4, 1)
#define TMS570_BALL_B4_RMII_CRS_DV TMS570_PIN_AND_FNC(TMS570_BALL_B4, 2)

#define TMS570_BALL_E9 TMS570_BALL_WITH_MMR(17, 3)
#define TMS570_BALL_E9_ETMDATA_8 TMS570_PIN_AND_FNC(TMS570_BALL_E9, 0)
#define TMS570_BALL_E9_EMIF_ADDR_5 TMS570_PIN_AND_FNC(TMS570_BALL_E9, 1)

#define TMS570_BALL_C13 TMS570_BALL_WITH_MMR(18, 0)
#define TMS570_BALL_C13_EMIF_ADDR_15 TMS570_PIN_AND_FNC(TMS570_BALL_C13, 0)
#define TMS570_BALL_C13_RTP_DATA_03 TMS570_PIN_AND_FNC(TMS570_BALL_C13, 1)

#define TMS570_BALL_A11 TMS570_BALL_WITH_MMR(18, 1)
#define TMS570_BALL_A11_HET1_14 TMS570_PIN_AND_FNC(TMS570_BALL_A11, 0)

#define TMS570_BALL_C12 TMS570_BALL_WITH_MMR(18, 2)
#define TMS570_BALL_C12_EMIF_ADDR_14 TMS570_PIN_AND_FNC(TMS570_BALL_C12, 0)
#define TMS570_BALL_C12_RTP_DATA_04 TMS570_PIN_AND_FNC(TMS570_BALL_C12, 1)

#define TMS570_BALL_M2 TMS570_BALL_WITH_MMR(18, 3)
#define TMS570_BALL_M2_GIOB_0 TMS570_PIN_AND_FNC(TMS570_BALL_M2, 0)

#define TMS570_BALL_E8 TMS570_BALL_WITH_MMR(19, 0)
#define TMS570_BALL_E8_ETMDATA_09 TMS570_PIN_AND_FNC(TMS570_BALL_E8, 0)
#define TMS570_BALL_E8_EMIF_ADDR_4 TMS570_PIN_AND_FNC(TMS570_BALL_E8, 1)

#define TMS570_BALL_B11 TMS570_BALL_WITH_MMR(19, 1)
#define TMS570_BALL_B11_HET1_30 TMS570_PIN_AND_FNC(TMS570_BALL_B11, 0)
#define TMS570_BALL_B11_MII_RX_DV TMS570_PIN_AND_FNC(TMS570_BALL_B11, 1)

#define TMS570_BALL_E10 TMS570_BALL_WITH_MMR(19, 2)
#define TMS570_BALL_E10_ETMDATA_15 TMS570_PIN_AND_FNC(TMS570_BALL_E10, 0)
#define TMS570_BALL_E10_EMIF_nDQM_0 TMS570_PIN_AND_FNC(TMS570_BALL_E10, 1)

#define TMS570_BALL_E7 TMS570_BALL_WITH_MMR(19, 3)
#define TMS570_BALL_E7_ETMDATA_10 TMS570_PIN_AND_FNC(TMS570_BALL_E7, 0)
#define TMS570_BALL_E7_EMIF_ADDR_3 TMS570_PIN_AND_FNC(TMS570_BALL_E7, 1)

#define TMS570_BALL_C11 TMS570_BALL_WITH_MMR(20, 0)
#define TMS570_BALL_C11_EMIF_ADDR_13 TMS570_PIN_AND_FNC(TMS570_BALL_C11, 0)
#define TMS570_BALL_C11_RTP_DATA_05 TMS570_PIN_AND_FNC(TMS570_BALL_C11, 1)

#define TMS570_BALL_C10 TMS570_BALL_WITH_MMR(20, 1)
#define TMS570_BALL_C10_EMIF_ADDR_12 TMS570_PIN_AND_FNC(TMS570_BALL_C10, 0)
#define TMS570_BALL_C10_RTP_DATA_06 TMS570_PIN_AND_FNC(TMS570_BALL_C10, 1)

#define TMS570_BALL_F3 TMS570_BALL_WITH_MMR(20, 2)
#define TMS570_BALL_F3_MIBSPI1NCS_1 TMS570_PIN_AND_FNC(TMS570_BALL_F3, 0)
#define TMS570_BALL_F3_HET1_17 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_F3, 1), \
                TMS570_PIN_AND_FNC(TMS570_BALL_A13, 0) | TMS570_PIN_CLEAR_RQ_MASK)
#define TMS570_BALL_F3_MII_COL TMS570_PIN_AND_FNC(TMS570_BALL_F3, 2)

#define TMS570_BALL_C9 TMS570_BALL_WITH_MMR(20, 3)
#define TMS570_BALL_C9_EMIF_ADDR_11 TMS570_PIN_AND_FNC(TMS570_BALL_C9, 0)
#define TMS570_BALL_C9_RTP_DATA_08 TMS570_PIN_AND_FNC(TMS570_BALL_C9, 1)

#define TMS570_BALL_D5 TMS570_BALL_WITH_MMR(21, 0)
#define TMS570_BALL_D5_EMIF_ADDR_1 TMS570_PIN_AND_FNC(TMS570_BALL_D5, 0)
#define TMS570_BALL_D5_HET2_03 TMS570_PIN_AND_FNC(TMS570_BALL_D5, 1)

#define TMS570_BALL_K2 TMS570_BALL_WITH_MMR(21, 1)
#define TMS570_BALL_K2_GIOB_1 TMS570_PIN_AND_FNC(TMS570_BALL_K2, 0)

#define TMS570_BALL_C8 TMS570_BALL_WITH_MMR(21, 2)
#define TMS570_BALL_C8_EMIF_ADDR_10 TMS570_PIN_AND_FNC(TMS570_BALL_C8, 0)
#define TMS570_BALL_C8_RTP_DATA_09 TMS570_PIN_AND_FNC(TMS570_BALL_C8, 1)

#define TMS570_BALL_C7 TMS570_BALL_WITH_MMR(21, 3)
#define TMS570_BALL_C7_EMIF_ADDR_9 TMS570_PIN_AND_FNC(TMS570_BALL_C7, 0)
#define TMS570_BALL_C7_RTP_DATA_10 TMS570_PIN_AND_FNC(TMS570_BALL_C7, 1)

#define TMS570_BALL_D4 TMS570_BALL_WITH_MMR(22, 0)
#define TMS570_BALL_D4_EMIF_ADDR_0 TMS570_PIN_AND_FNC(TMS570_BALL_D4, 0)
#define TMS570_BALL_D4_HET2_01 TMS570_PIN_AND_FNC(TMS570_BALL_D4, 1)

#define TMS570_BALL_C5 TMS570_BALL_WITH_MMR(22, 1)
#define TMS570_BALL_C5_EMIF_ADDR_7 TMS570_PIN_AND_FNC(TMS570_BALL_C5, 0)
#define TMS570_BALL_C5_RTP_DATA_12 TMS570_PIN_AND_FNC(TMS570_BALL_C5, 1)
#define TMS570_BALL_C5_HET2_13 TMS570_PIN_AND_FNC(TMS570_BALL_C5, 2)

#define TMS570_BALL_C4 TMS570_BALL_WITH_MMR(22, 2)
#define TMS570_BALL_C4_EMIF_ADDR_6 TMS570_PIN_AND_FNC(TMS570_BALL_C4, 0)
#define TMS570_BALL_C4_RTP_DATA_13 TMS570_PIN_AND_FNC(TMS570_BALL_C4, 1)
#define TMS570_BALL_C4_HET2_11 TMS570_PIN_AND_FNC(TMS570_BALL_C4, 2)

#define TMS570_BALL_E6 TMS570_BALL_WITH_MMR(22, 3)
#define TMS570_BALL_E6_ETMDATA_11 TMS570_PIN_AND_FNC(TMS570_BALL_E6, 0)
#define TMS570_BALL_E6_EMIF_ADDR_2 TMS570_PIN_AND_FNC(TMS570_BALL_E6, 1)

#define TMS570_BALL_C6 TMS570_BALL_WITH_MMR(23, 0)
#define TMS570_BALL_C6_EMIF_ADDR_8 TMS570_PIN_AND_FNC(TMS570_BALL_C6, 0)
#define TMS570_BALL_C6_RTP_DATA_11 TMS570_PIN_AND_FNC(TMS570_BALL_C6, 1)
#define TMS570_BALL_C6_HET2_15 TMS570_PIN_AND_FNC(TMS570_BALL_C6, 2)

#define TMS570_MMR_SELECT_SPI4CLK TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(23, 1), 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_K18, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_MMR_SELECT_SPI4SIMO TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(23, 2), 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_W5, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_MMR_SELECT_SPI4SOMI TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(23, 3), 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_V6, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_MMR_SELECT_SPI4NENA TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(24, 0), 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_V2, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_MMR_SELECT_SPI4NCS_0 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(24, 1), 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_U1, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_A13 TMS570_BALL_WITH_MMR(24, 2)
#define TMS570_BALL_A13_HET1_17 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_A13, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_F3, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_B13 TMS570_BALL_WITH_MMR(24, 3)
#define TMS570_BALL_B13_HET1_19 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_B13, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_G3, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_H4 TMS570_BALL_WITH_MMR(25, 0)
#define TMS570_BALL_H4_HET1_21 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_H4, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_J3, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_J4 TMS570_BALL_WITH_MMR(25, 1)
#define TMS570_BALL_J4_HET1_23 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_J4, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_G19, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_M3 TMS570_BALL_WITH_MMR(25, 2)
#define TMS570_BALL_M3_HET1_25 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_M3, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_V5, 1) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_A9 TMS570_BALL_WITH_MMR(25, 3)
#define TMS570_BALL_A9_HET1_27 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_A9, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_B2, 2) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_A3 TMS570_BALL_WITH_MMR(26, 0)
#define TMS570_BALL_A3_HET1_29 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_A3, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_C3, 2) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_J17 TMS570_BALL_WITH_MMR(26, 1)
#define TMS570_BALL_J17_HET1_31 TMS570_PIN_WITH_IN_ALT( \
                TMS570_PIN_AND_FNC(TMS570_BALL_J17, 0), \
                TMS570_PIN_AND_FNC(TMS570_BALL_W9, 2) | TMS570_PIN_CLEAR_RQ_MASK)

#define TMS570_BALL_W6 TMS570_BALL_WITH_MMR(26, 2)
#define TMS570_BALL_W6_MIBSPI5NCS_2 TMS570_PIN_AND_FNC(TMS570_BALL_W6, 0)
#define TMS570_BALL_W6_DMM_DATA_2 TMS570_PIN_AND_FNC(TMS570_BALL_W6, 1)

#define TMS570_BALL_T12 TMS570_BALL_WITH_MMR(26, 3)
#define TMS570_BALL_T12_MIBSPI5NCS_3 TMS570_PIN_AND_FNC(TMS570_BALL_T12, 0)
#define TMS570_BALL_T12_DMM_DATA_3 TMS570_PIN_AND_FNC(TMS570_BALL_T12, 1)

#define TMS570_BALL_E19 TMS570_BALL_WITH_MMR(27, 0)
#define TMS570_BALL_E19_MIBSPI5NCS_0 TMS570_PIN_AND_FNC(TMS570_BALL_E19, 0)
#define TMS570_BALL_E19_DMM_DATA_5 TMS570_PIN_AND_FNC(TMS570_BALL_E19, 1)

#define TMS570_BALL_B6 TMS570_BALL_WITH_MMR(27, 1)
#define TMS570_BALL_B6_MIBSPI5NCS_1 TMS570_PIN_AND_FNC(TMS570_BALL_B6, 0)
#define TMS570_BALL_B6_DMM_DATA_6 TMS570_PIN_AND_FNC(TMS570_BALL_B6, 1)

#define TMS570_BALL_E16 TMS570_BALL_WITH_MMR(27, 2)
#define TMS570_BALL_E16_MIBSPI5SIMO_1 TMS570_PIN_AND_FNC(TMS570_BALL_E16, 0)
#define TMS570_BALL_E16_DMM_DATA_9 TMS570_PIN_AND_FNC(TMS570_BALL_E16, 1)

#define TMS570_BALL_H17 TMS570_BALL_WITH_MMR(27, 3)
#define TMS570_BALL_H17_MIBSPI5SIMO_2 TMS570_PIN_AND_FNC(TMS570_BALL_H17, 0)
#define TMS570_BALL_H17_DMM_DATA_10 TMS570_PIN_AND_FNC(TMS570_BALL_H17, 1)

#define TMS570_BALL_G17 TMS570_BALL_WITH_MMR(28, 0)
#define TMS570_BALL_G17_MIBSPI5SIMO_3 TMS570_PIN_AND_FNC(TMS570_BALL_G17, 0)
#define TMS570_BALL_G17_DMM_DATA_11 TMS570_PIN_AND_FNC(TMS570_BALL_G17, 1)

#define TMS570_BALL_E17 TMS570_BALL_WITH_MMR(28, 1)
#define TMS570_BALL_E17_MIBSPI5SOMI_1 TMS570_PIN_AND_FNC(TMS570_BALL_E17, 0)
#define TMS570_BALL_E17_DMM_DATA_13 TMS570_PIN_AND_FNC(TMS570_BALL_E17, 1)

#define TMS570_BALL_H16 TMS570_BALL_WITH_MMR(28, 2)
#define TMS570_BALL_H16_MIBSPI5SOMI_2 TMS570_PIN_AND_FNC(TMS570_BALL_H16, 0)
#define TMS570_BALL_H16_DMM_DATA_14 TMS570_PIN_AND_FNC(TMS570_BALL_H16, 1)

#define TMS570_BALL_G16 TMS570_BALL_WITH_MMR(28, 3)
#define TMS570_BALL_G16_MIBSPI5SOMI_3 TMS570_PIN_AND_FNC(TMS570_BALL_G16, 0)
#define TMS570_BALL_G16_DMM_DATA_15 TMS570_PIN_AND_FNC(TMS570_BALL_G16, 1)

#define TMS570_BALL_D3 TMS570_BALL_WITH_MMR(29, 0)
#define TMS570_BALL_D3_SPI2NENA TMS570_PIN_AND_FNC(TMS570_BALL_D3, 0)
#define TMS570_BALL_D3_SPI2NCS_1 TMS570_PIN_AND_FNC(TMS570_BALL_D3, 1)

#define TMS570_MMR_SELECT_EMIF_CLK_SEL TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(29, 1), 0)

#define TMS570_BALL_F2 TMS570_BALL_WITH_MMR(29, 2)
#define TMS570_BALL_F2_GIOB_2 TMS570_PIN_AND_FNC(TMS570_BALL_F2, 0)

#define TMS570_MMR_SELECT_MII_MODE \
  TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(29, 3), 0)
#define TMS570_MMR_SELECT_RMII_MODE \
  TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(29, 3), TMS570_PIN_FNC_CLEAR)

#define TMS570_MMR_SELECT_ADC_TRG1 TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(30, 0), 0)
#define TMS570_MMR_SELECT_ADC_TRG2 TMS570_PIN_AND_FNC(TMS570_BALL_WITH_MMR(30, 0), 1)

/* Default pinmux select */

#define TMS570_PINMMR_DEFAULT_INIT_LIST(per_pin_action, common_arg) \
  per_pin_action(common_arg, TMS570_BALL_W10_GIOB_3) \
  per_pin_action(common_arg, TMS570_BALL_A5_GIOA_0) \
  per_pin_action(common_arg, TMS570_BALL_C3_MIBSPI3NCS_3) \
  per_pin_action(common_arg, TMS570_BALL_B2_MIBSPI3NCS_2) \
  per_pin_action(common_arg, TMS570_BALL_C2_GIOA_1) \
  per_pin_action(common_arg, TMS570_BALL_E3_HET1_11) \
  per_pin_action(common_arg, TMS570_BALL_E5_ETMDATA_20) \
  per_pin_action(common_arg, TMS570_BALL_F5_ETMDATA_21) \
  per_pin_action(common_arg, TMS570_BALL_C1_GIOA_2) \
  per_pin_action(common_arg, TMS570_BALL_G5_ETMDATA_22) \
  per_pin_action(common_arg, TMS570_BALL_E1_GIOA_3) \
  per_pin_action(common_arg, TMS570_BALL_B5_GIOA_5) \
  per_pin_action(common_arg, TMS570_BALL_K5_ETMDATA_23) \
  per_pin_action(common_arg, TMS570_BALL_B3_HET1_22) \
  per_pin_action(common_arg, TMS570_BALL_H3_GIOA_6) \
  per_pin_action(common_arg, TMS570_BALL_L5_ETMDATA_24) \
  per_pin_action(common_arg, TMS570_BALL_M1_GIOA_7) \
  per_pin_action(common_arg, TMS570_BALL_M5_ETMDATA_25) \
  per_pin_action(common_arg, TMS570_BALL_V2_HET1_01) \
  per_pin_action(common_arg, TMS570_BALL_U1_HET1_03) \
  per_pin_action(common_arg, TMS570_BALL_K18_HET1_00) \
  per_pin_action(common_arg, TMS570_BALL_W5_HET1_02) \
  per_pin_action(common_arg, TMS570_BALL_V6_HET1_05) \
  per_pin_action(common_arg, TMS570_BALL_N5_ETMDATA_26) \
  per_pin_action(common_arg, TMS570_BALL_T1_HET1_07) \
  per_pin_action(common_arg, TMS570_BALL_P5_ETMDATA_27) \
  per_pin_action(common_arg, TMS570_BALL_V7_HET1_09) \
  per_pin_action(common_arg, TMS570_BALL_R5_ETMDATA_28) \
  per_pin_action(common_arg, TMS570_BALL_R6_ETMDATA_29) \
  per_pin_action(common_arg, TMS570_BALL_V5_MIBSPI3NCS_1) \
  per_pin_action(common_arg, TMS570_BALL_W3_HET1_06) \
  per_pin_action(common_arg, TMS570_BALL_R7_ETMDATA_30) \
  per_pin_action(common_arg, TMS570_BALL_N2_HET1_13) \
  per_pin_action(common_arg, TMS570_BALL_G3_MIBSPI1NCS_2) \
  per_pin_action(common_arg, TMS570_BALL_N1_HET1_15) \
  per_pin_action(common_arg, TMS570_BALL_R8_ETMDATA_31) \
  per_pin_action(common_arg, TMS570_BALL_R9_ETMTRACECLKIN) \
  per_pin_action(common_arg, TMS570_BALL_W9_MIBSPI3NENA) \
  per_pin_action(common_arg, TMS570_BALL_V10_MIBSPI3NCS_0) \
  per_pin_action(common_arg, TMS570_BALL_J3_MIBSPI1NCS_3) \
  per_pin_action(common_arg, TMS570_BALL_N19_AD1EVT) \
  per_pin_action(common_arg, TMS570_BALL_N15_ETMDATA_19) \
  per_pin_action(common_arg, TMS570_BALL_N17_EMIF_nCS_0) \
  per_pin_action(common_arg, TMS570_BALL_M15_ETMDATA_18) \
  per_pin_action(common_arg, TMS570_BALL_K17_EMIF_nCS_3) \
  per_pin_action(common_arg, TMS570_BALL_M17_EMIF_nCS_4) \
  per_pin_action(common_arg, TMS570_BALL_L15_ETMDATA_17) \
  per_pin_action(common_arg, TMS570_BALL_P1_HET1_24) \
  per_pin_action(common_arg, TMS570_BALL_A14_HET1_26) \
  per_pin_action(common_arg, TMS570_BALL_K15_ETMDATA_16) \
  per_pin_action(common_arg, TMS570_BALL_G19_MIBSPI1NENA) \
  per_pin_action(common_arg, TMS570_BALL_H18_MIBSPI5NENA) \
  per_pin_action(common_arg, TMS570_BALL_J18_MIBSPI5SOMI_0) \
  per_pin_action(common_arg, TMS570_BALL_J19_MIBSPI5SIMO_0) \
  per_pin_action(common_arg, TMS570_BALL_H19_MIBSPI5CLK) \
  per_pin_action(common_arg, TMS570_BALL_R2_MIBSPI1NCS_0) \
  per_pin_action(common_arg, TMS570_BALL_E18_HET1_08) \
  per_pin_action(common_arg, TMS570_BALL_K19_HET1_28) \
  per_pin_action(common_arg, TMS570_BALL_D17_EMIF_nWE) \
  per_pin_action(common_arg, TMS570_BALL_D16_EMIF_BA_1) \
  per_pin_action(common_arg, TMS570_BALL_C17_EMIF_ADDR_21) \
  per_pin_action(common_arg, TMS570_BALL_C16_EMIF_ADDR_20) \
  per_pin_action(common_arg, TMS570_BALL_C15_EMIF_ADDR_19) \
  per_pin_action(common_arg, TMS570_BALL_D15_EMIF_ADDR_18) \
  per_pin_action(common_arg, TMS570_BALL_E13_ETMDATA_12) \
  per_pin_action(common_arg, TMS570_BALL_C14_EMIF_ADDR_17) \
  per_pin_action(common_arg, TMS570_BALL_D14_EMIF_ADDR_16) \
  per_pin_action(common_arg, TMS570_BALL_E12_ETMDATA_13) \
  per_pin_action(common_arg, TMS570_BALL_D19_HET1_10) \
  per_pin_action(common_arg, TMS570_BALL_E11_ETMDATA_14) \
  per_pin_action(common_arg, TMS570_BALL_B4_HET1_12) \
  per_pin_action(common_arg, TMS570_BALL_E9_ETMDATA_8) \
  per_pin_action(common_arg, TMS570_BALL_C13_EMIF_ADDR_15) \
  per_pin_action(common_arg, TMS570_BALL_A11_HET1_14) \
  per_pin_action(common_arg, TMS570_BALL_C12_EMIF_ADDR_14) \
  per_pin_action(common_arg, TMS570_BALL_M2_GIOB_0) \
  per_pin_action(common_arg, TMS570_BALL_E8_ETMDATA_09) \
  per_pin_action(common_arg, TMS570_BALL_B11_HET1_30) \
  per_pin_action(common_arg, TMS570_BALL_E10_ETMDATA_15) \
  per_pin_action(common_arg, TMS570_BALL_E7_ETMDATA_10) \
  per_pin_action(common_arg, TMS570_BALL_C11_EMIF_ADDR_13) \
  per_pin_action(common_arg, TMS570_BALL_C10_EMIF_ADDR_12) \
  per_pin_action(common_arg, TMS570_BALL_F3_MIBSPI1NCS_1) \
  per_pin_action(common_arg, TMS570_BALL_C9_EMIF_ADDR_11) \
  per_pin_action(common_arg, TMS570_BALL_D5_EMIF_ADDR_1) \
  per_pin_action(common_arg, TMS570_BALL_K2_GIOB_1) \
  per_pin_action(common_arg, TMS570_BALL_C8_EMIF_ADDR_10) \
  per_pin_action(common_arg, TMS570_BALL_C7_EMIF_ADDR_9) \
  per_pin_action(common_arg, TMS570_BALL_D4_EMIF_ADDR_0) \
  per_pin_action(common_arg, TMS570_BALL_C5_EMIF_ADDR_7) \
  per_pin_action(common_arg, TMS570_BALL_C4_EMIF_ADDR_6) \
  per_pin_action(common_arg, TMS570_BALL_E6_ETMDATA_11) \
  per_pin_action(common_arg, TMS570_BALL_C6_EMIF_ADDR_8) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_SPI4CLK) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_SPI4SIMO) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_SPI4SOMI) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_SPI4NENA) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_SPI4NCS_0) \
  per_pin_action(common_arg, TMS570_BALL_A13_HET1_17) \
  per_pin_action(common_arg, TMS570_BALL_B13_HET1_19) \
  per_pin_action(common_arg, TMS570_BALL_H4_HET1_21) \
  per_pin_action(common_arg, TMS570_BALL_J4_HET1_23) \
  per_pin_action(common_arg, TMS570_BALL_M3_HET1_25) \
  per_pin_action(common_arg, TMS570_BALL_A9_HET1_27) \
  per_pin_action(common_arg, TMS570_BALL_A3_HET1_29) \
  per_pin_action(common_arg, TMS570_BALL_J17_HET1_31) \
  per_pin_action(common_arg, TMS570_BALL_W6_MIBSPI5NCS_2) \
  per_pin_action(common_arg, TMS570_BALL_T12_MIBSPI5NCS_3) \
  per_pin_action(common_arg, TMS570_BALL_E19_MIBSPI5NCS_0) \
  per_pin_action(common_arg, TMS570_BALL_B6_MIBSPI5NCS_1) \
  per_pin_action(common_arg, TMS570_BALL_E16_MIBSPI5SIMO_1) \
  per_pin_action(common_arg, TMS570_BALL_H17_MIBSPI5SIMO_2) \
  per_pin_action(common_arg, TMS570_BALL_G17_MIBSPI5SIMO_3) \
  per_pin_action(common_arg, TMS570_BALL_E17_MIBSPI5SOMI_1) \
  per_pin_action(common_arg, TMS570_BALL_H16_MIBSPI5SOMI_2) \
  per_pin_action(common_arg, TMS570_BALL_G16_MIBSPI5SOMI_3) \
  per_pin_action(common_arg, TMS570_BALL_D3_SPI2NENA) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_EMIF_CLK_SEL) \
  per_pin_action(common_arg, TMS570_BALL_F2_GIOB_2) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_GMII_SEL) \
  per_pin_action(common_arg, TMS570_MMR_SELECT_ADC_TRG1) \

/* End of default PINMMR list */

#endif /*LIBBSP_ARM_TMS570_TMS570LS3137ZWT_PINS_H*/
