/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _HCI_HAL_INIT_C_

#include <osdep_service.h>
#include <drv_types.h>
#include <rtw_efuse.h>
#include <fw.h>
#include <rtl8188e_hal.h>
#include <rtl8188e_led.h>
#include <rtw_iol.h>
#include <usb_hal.h>
#include <phy.h>

#define		HAL_BB_ENABLE		1

static void _ConfigNormalChipOutEP_8188E(struct ieee80211_hw *hw, u8 NumOutPipe)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_usb_priv *usb_priv = rtl_usbpriv(hw);
	struct rtl_usb *rtlusb = rtl_usbdev(usb_priv);

	switch (NumOutPipe) {
	case	3:
		rtlusb->out_queue_sel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		haldata->OutEpNumber = 3;
		break;
	case	2:
		rtlusb->out_queue_sel = TX_SELE_HQ | TX_SELE_NQ;
		haldata->OutEpNumber = 2;
		break;
	case	1:
		rtlusb->out_queue_sel = TX_SELE_HQ;
		haldata->OutEpNumber = 1;
		break;
	default:
		break;
	}
	DBG_88E("%s OutEpQueueSel(0x%02x), OutEpNumber(%d)\n", __func__, rtlusb->out_queue_sel, haldata->OutEpNumber);
}

static bool HalUsbSetQueuePipeMapping8188EUsb(struct ieee80211_hw *hw, u8 NumInPipe, u8 NumOutPipe)
{
	bool result = false;

	_ConfigNormalChipOutEP_8188E(hw, NumOutPipe);
	
	/* TODO */
	/*  Normal chip with one IN and one OUT doesn't have interrupt IN EP. */
	if (1 == haldata->OutEpNumber) {
		if (1 != NumInPipe)
			return result;
	}

	/*  All config other than above support one Bulk IN and one Interrupt IN. */

	result = Hal_MappingOutPipe(hw, NumOutPipe);

	return result;
}

static void rtl8188eu_interface_configure(struct ieee80211_hw *hw)
{
	/* TODO */
#if 0

	if (pdvobjpriv->ishighspeed)
		haldata->UsbBulkOutSize = USB_HIGH_SPEED_BULK_SIZE;/* 512 bytes */
	else
		haldata->UsbBulkOutSize = USB_FULL_SPEED_BULK_SIZE;/* 64 bytes */

	haldata->interfaceIndex = pdvobjpriv->InterfaceNumber;

	haldata->UsbTxAggMode		= 1;
	haldata->UsbTxAggDescNum	= 0x6;	/*  only 4 bits */

	haldata->UsbRxAggMode		= USB_RX_AGG_DMA;/*  USB_RX_AGG_DMA; */
	haldata->UsbRxAggBlockCount	= 8; /* unit : 512b */
	haldata->UsbRxAggBlockTimeout	= 0x6;
	haldata->UsbRxAggPageCount	= 48; /* uint :128 b 0x0A;	10 = MAX_RX_DMA_BUFFER_SIZE/2/haldata->UsbBulkOutSize */
	haldata->UsbRxAggPageTimeout	= 0x4; /* 6, absolute time = 34ms/(2^6) */

	HalUsbSetQueuePipeMapping8188EUsb(hw,
				pdvobjpriv->RtNumInPipes, pdvobjpriv->RtNumOutPipes);
#endif
}

static u32 rtl8188eu_InitPowerOn(struct ieee80211_hw *hw)
{
	u16 value16;
	/*  HW Power on sequence */
#if 0
	if (haldata->bMacPwrCtrlOn)
		return true;
#endif

	if (!rtl88eu_pwrseqcmdparsing(hw, PWR_CUT_ALL_MSK,
				      PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK,
				      Rtl8188E_NIC_PWR_ON_FLOW)) {
		DBG_88E(KERN_ERR "%s: run power on flow fail\n", __func__);
		return false;
	}

	/*  Enable MAC DMA/WMAC/SCHEDULE/SEC block */
	/*  Set CR bit10 to enable 32k calibration. Suggested by SD1 Gimmy. Added by tynli. 2011.08.31. */
	rtl_write_word(rtlpriv, REG_CR, 0x00);  /* suggseted by zhouzhou, by page, 20111230 */

		/*  Enable MAC DMA/WMAC/SCHEDULE/SEC block */
	value16 = rtl_read_word(rtlpriv, REG_CR);
	value16 |= (HCI_TXDMA_EN | HCI_RXDMA_EN | TXDMA_EN | RXDMA_EN
				| PROTOCOL_EN | SCHEDULE_EN | ENSEC | CALTMR_EN);
	/*  for SDIO - Set CR bit10 to enable 32k calibration. Suggested by SD1 Gimmy. Added by tynli. 2011.08.31. */

	rtl_write_word(rtlpriv, REG_CR, value16);
	/* TODO */
#if 0
	haldata->bMacPwrCtrlOn = true;
#endif

	return true;
}

/*  Shall USB interface init this? */
static void _InitInterrupt(struct ieee80211_hw *hw)
{
	u32 imr, imr_ex;
	u8  usb_opt;

	/* HISR write one to clear */
	rtl_write_dword(rtlpriv, REG_HISR_88E, 0xFFFFFFFF);
	/*  HIMR - */
	imr = IMR_PSTIMEOUT_88E | IMR_TBDER_88E | IMR_CPWM_88E | IMR_CPWM2_88E;
	rtl_write_dword(rtlpriv, REG_HIMR_88E, imr);
	rtlusb->irq_mask[0] = imr;

	imr_ex = IMR_TXERR_88E | IMR_RXERR_88E | IMR_TXFOVW_88E | IMR_RXFOVW_88E;
	rtl_write_dword(rtlpriv, REG_HIMRE_88E, imr_ex);
	rtlusb->irq_mask[1] = imr_ex;

	/*  REG_USB_SPECIAL_OPTION - BIT(4) */
	/*  0; Use interrupt endpoint to upload interrupt pkt */
	/*  1; Use bulk endpoint to upload interrupt pkt, */
	usb_opt = rtl_read_byte(rtlpriv, REG_USB_SPECIAL_OPTION);

	if (!IS_HIGH_SPEED_USB(rtlusb->udev)
		usb_opt = usb_opt & (~INT_BULK_SEL);
	else
		usb_opt = usb_opt | (INT_BULK_SEL);

	rtl_write_byte(rtlpriv, REG_USB_SPECIAL_OPTION, usb_opt);
}

static void _InitQueueReservedPage(struct ieee80211_hw *hw)
{
	u32 numHQ	= 0;
	u32 numLQ	= 0;
	u32 numNQ	= 0;
	u32 numPubQ;
	u32 value32;
	u8 value8;
	bool bWiFiConfig = rtlusb->wmm_enable;

	if (bWiFiConfig) {
		if (rtlusb->out_queue_sel & TX_SELE_HQ)
			numHQ =  0x29;

		if (rtlusb->out_queue_sel & TX_SELE_LQ)
			numLQ = 0x1C;

		/*  NOTE: This step shall be proceed before writting REG_RQPN. */
		if (rtlusb->out_queue_sel & TX_SELE_NQ)
			numNQ = 0x1C;
		value8 = (u8)_NPQ(numNQ);
		rtl_write_byte(rtlpriv, REG_RQPN_NPQ, value8);

		numPubQ = 0xA8 - numHQ - numLQ - numNQ;

		/*  TX DMA */
		value32 = _HPQ(numHQ) | _LPQ(numLQ) | _PUBQ(numPubQ) | LD_RQPN;
		rtl_write_dword(rtlpriv, REG_RQPN, value32);
	} else {
		rtl_write_word(rtlpriv, REG_RQPN_NPQ, 0x0000);/* Just follow MP Team,??? Georgia 03/28 */
		rtl_write_word(rtlpriv, REG_RQPN_NPQ, 0x0d);
		rtl_write_dword(rtlpriv, REG_RQPN, 0x808E000d);/* reserve 7 page for LPS */
	}
}

static void _InitTxBufferBoundary(struct ieee80211_hw *hw, u8 txpktbuf_bndy)
{
	rtl_write_byte(rtlpriv, REG_TXPKTBUF_BCNQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TXPKTBUF_MGQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TXPKTBUF_WMAC_LBK_BF_HD, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TRXFF_BNDY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TDECTRL+1, txpktbuf_bndy);
}

static void _InitPageBoundary(struct ieee80211_hw *hw)
{
	/*  RX Page Boundary */
	/*  */
	u16 rxff_bndy = MAX_RX_DMA_BUFFER_SIZE_88E-1;

	rtl_write_word(rtlpriv, (REG_TRXFF_BNDY + 2), rxff_bndy);
}

static void _InitNormalChipRegPriority(struct ieee80211_hw *hw, u16 beQ,
				       u16 bkQ, u16 viQ, u16 voQ, u16 mgtQ,
				       u16 hiQ)
{
	u16 value16	= (rtl_read_word(rtlpriv, REG_TRXDMA_CTRL) & 0x7);

	value16 |= _TXDMA_BEQ_MAP(beQ)	| _TXDMA_BKQ_MAP(bkQ) |
		   _TXDMA_VIQ_MAP(viQ)	| _TXDMA_VOQ_MAP(voQ) |
		   _TXDMA_MGQ_MAP(mgtQ) | _TXDMA_HIQ_MAP(hiQ);

	rtl_write_word(rtlpriv, REG_TRXDMA_CTRL, value16);
}

static void _InitNormalChipOneOutEpPriority(struct ieee80211_hw *hw)
{

	u16 value = 0;
	switch (rtlusb->out_queue_sel) {
	case TX_SELE_HQ:
		value = QUEUE_HIGH;
		break;
	case TX_SELE_LQ:
		value = QUEUE_LOW;
		break;
	case TX_SELE_NQ:
		value = QUEUE_NORMAL;
		break;
	default:
		break;
	}
	_InitNormalChipRegPriority(hw, value, value, value, value,
				   value, value);
}

static void _InitNormalChipTwoOutEpPriority(struct ieee80211_hw *hw)
{
	u16 beQ, bkQ, viQ, voQ, mgtQ, hiQ;
	u16 valueHi = 0;
	u16 valueLow = 0;

	switch (rtlusb->out_queue_sel) {
	case (TX_SELE_HQ | TX_SELE_LQ):
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_LOW;
		break;
	case (TX_SELE_NQ | TX_SELE_LQ):
		valueHi = QUEUE_NORMAL;
		valueLow = QUEUE_LOW;
		break;
	case (TX_SELE_HQ | TX_SELE_NQ):
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_NORMAL;
		break;
	default:
		break;
	}

	if (!rtlusb->wmm_enable) {
		beQ	= valueLow;
		bkQ	= valueLow;
		viQ	= valueHi;
		voQ	= valueHi;
		mgtQ	= valueHi;
		hiQ	= valueHi;
	} else {/* for WMM ,CONFIG_OUT_EP_WIFI_MODE */
		beQ	= valueLow;
		bkQ	= valueHi;
		viQ	= valueHi;
		voQ	= valueLow;
		mgtQ	= valueHi;
		hiQ	= valueHi;
	}
	_InitNormalChipRegPriority(hw, beQ, bkQ, viQ, voQ, mgtQ, hiQ);
}

static void _InitNormalChipThreeOutEpPriority(struct ieee80211_hw *hw)
{
	u16 beQ, bkQ, viQ, voQ, mgtQ, hiQ;

	if (!rtlusb->wmm_enable) {/*  typical setting */
		beQ	= QUEUE_LOW;
		bkQ	= QUEUE_LOW;
		viQ	= QUEUE_NORMAL;
		voQ	= QUEUE_HIGH;
		mgtQ	= QUEUE_HIGH;
		hiQ	= QUEUE_HIGH;
	} else {/*  for WMM */
		beQ	= QUEUE_LOW;
		bkQ	= QUEUE_NORMAL;
		viQ	= QUEUE_NORMAL;
		voQ	= QUEUE_HIGH;
		mgtQ	= QUEUE_HIGH;
		hiQ	= QUEUE_HIGH;
	}
	_InitNormalChipRegPriority(hw, beQ, bkQ, viQ, voQ, mgtQ, hiQ);
}

static void _InitQueuePriority(struct ieee80211_hw *hw)
{

	switch (haldata->OutEpNumber) {
	case 1:
		_InitNormalChipOneOutEpPriority(hw);
		break;
	case 2:
		_InitNormalChipTwoOutEpPriority(hw);
		break;
	case 3:
		_InitNormalChipThreeOutEpPriority(hw);
		break;
	default:
		break;
	}
}

static void _InitNetworkType(struct ieee80211_hw *hw)
{
	u32 value32;

	value32 = rtl_read_dword(rtlpriv, REG_CR);
	/*  TODO: use the other function to set network type */
	value32 = (value32 & ~MASK_NETTYPE) | _NETTYPE(NT_LINK_AP);

	rtl_write_dword(rtlpriv, REG_CR, value32);
}

static void _InitTransferPageSize(struct ieee80211_hw *hw)
{
	/*  Tx page size is always 128. */

	u8 value8;
	value8 = _PSRX(PBP_128) | _PSTX(PBP_128);
	rtl_write_byte(rtlpriv, REG_PBP, value8);
}

static void _InitDriverInfoSize(struct ieee80211_hw *hw, u8 drvInfoSize)
{
	rtl_write_byte(rtlpriv, REG_RX_DRVINFO_SZ, drvInfoSize);
}

static void _InitWMACSetting(struct ieee80211_hw *hw)
{
	mac->rx_conf = RCR_AAP | RCR_APM | RCR_AM | RCR_AB |
				  RCR_CBSSID_DATA | RCR_CBSSID_BCN |
				  RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL |
				  RCR_APP_MIC | RCR_APP_PHYSTS;

	/*  some REG_RCR will be modified later by phy_ConfigMACWithHeaderFile() */
	rtl_write_dword(rtlpriv, REG_RCR, mac->rx_conf);

	/*  Accept all multicast address */
	rtl_write_dword(rtlpriv, REG_MAR, 0xFFFFFFFF);
	rtl_write_dword(rtlpriv, REG_MAR + 4, 0xFFFFFFFF);
}

static void _InitAdaptiveCtrl(struct ieee80211_hw *hw)
{
	u16 value16;
	u32 value32;

	/*  Response Rate Set */
	value32 = rtl_read_dword(rtlpriv, REG_RRSR);
	value32 &= ~RATE_BITMAP_ALL;
	value32 |= RATE_RRSR_CCK_ONLY_1M;
	rtl_write_dword(rtlpriv, REG_RRSR, value32);

	/*  CF-END Threshold */

	/*  SIFS (used in NAV) */
	value16 = _SPEC_SIFS_CCK(0x10) | _SPEC_SIFS_OFDM(0x10);
	rtl_write_word(rtlpriv, REG_SPEC_SIFS, value16);

	/*  Retry Limit */
	value16 = _LRL(0x30) | _SRL(0x30);
	rtl_write_word(rtlpriv, REG_RL, value16);
}

static void _InitEDCA(struct ieee80211_hw *hw)
{
	/*  Set Spec SIFS (used in NAV) */
	rtl_write_word(rtlpriv, REG_SPEC_SIFS, 0x100a);
	rtl_write_word(rtlpriv, REG_MAC_SPEC_SIFS, 0x100a);

	/*  Set SIFS for CCK */
	rtl_write_word(rtlpriv, REG_SIFS_CTX, 0x100a);

	/*  Set SIFS for OFDM */
	rtl_write_word(rtlpriv, REG_SIFS_TRX, 0x100a);

	/*  TXOP */
	rtl_write_dword(rtlpriv, REG_EDCA_BE_PARAM, 0x005EA42B);
	rtl_write_dword(rtlpriv, REG_EDCA_BK_PARAM, 0x0000A44F);
	rtl_write_dword(rtlpriv, REG_EDCA_VI_PARAM, 0x005EA324);
	rtl_write_dword(rtlpriv, REG_EDCA_VO_PARAM, 0x002FA226);
}

static void _InitRDGSetting(struct ieee80211_hw *hw)
{
	rtl_write_byte(rtlpriv, REG_RD_CTRL, 0xFF);
	rtl_write_word(rtlpriv, REG_RD_NAV_NXT, 0x200);
	rtl_write_byte(rtlpriv, REG_RD_RESP_PKT_TH, 0x05);
}

static void _InitRxSetting(struct ieee80211_hw *hw)
{
	rtl_write_dword(rtlpriv, REG_MACID, 0x87654321);
	rtl_write_dword(rtlpriv, 0x0700, 0x87654321);
}

static void _InitRetryFunction(struct ieee80211_hw *hw)
{
	u8 value8;

	value8 = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL);
	value8 |= EN_AMPDU_RTY_NEW;
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL, value8);

	/*  Set ACK timeout */
	rtl_write_byte(rtlpriv, REG_ACKTO, 0x40);
}

/*-----------------------------------------------------------------------------
 * Function:	usb_AggSettingTxUpdate()
 *
 * Overview:	Separate TX/RX parameters update independent for TP detection and
 *			dynamic TX/RX aggreagtion parameters update.
 *
 * Input:			struct ieee80211_hw *
 *
 * Output/Return:	NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	12/10/2010	MHC		Separate to smaller function.
 *
 *---------------------------------------------------------------------------*/
static void usb_AggSettingTxUpdate(struct ieee80211_hw *hw)
{
	/* TODO */
#if 0
	u32 value32;

	if (Adapter->registrypriv.wifi_spec)
		haldata->UsbTxAggMode = false;

	if (haldata->UsbTxAggMode) {
		value32 = rtl_read_dword(rtlpriv, REG_TDECTRL);
		value32 = value32 & ~(BLK_DESC_NUM_MASK << BLK_DESC_NUM_SHIFT);
		value32 |= ((haldata->UsbTxAggDescNum & BLK_DESC_NUM_MASK) << BLK_DESC_NUM_SHIFT);

		rtl_write_dword(rtlpriv, REG_TDECTRL, value32);
	}
#endif
}	/*  usb_AggSettingTxUpdate */

/*-----------------------------------------------------------------------------
 * Function:	usb_AggSettingRxUpdate()
 *
 * Overview:	Separate TX/RX parameters update independent for TP detection and
 *			dynamic TX/RX aggreagtion parameters update.
 *
 * Input:			struct ieee80211_hw *
 *
 * Output/Return:	NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	12/10/2010	MHC		Separate to smaller function.
 *
 *---------------------------------------------------------------------------*/
static void
usb_AggSettingRxUpdate(
		struct ieee80211_hw *hw
	)
{
	/* TODO */
#if 0
	u8 valueDMA;
	u8 valueUSB;

	valueDMA = rtl_read_byte(rtlpriv, REG_TRXDMA_CTRL);
	valueUSB = rtl_read_byte(rtlpriv, REG_USB_SPECIAL_OPTION);

	switch (haldata->UsbRxAggMode) {
	case USB_RX_AGG_DMA:
		valueDMA |= RXDMA_AGG_EN;
		valueUSB &= ~USB_AGG_EN;
		break;
	case USB_RX_AGG_USB:
		valueDMA &= ~RXDMA_AGG_EN;
		valueUSB |= USB_AGG_EN;
		break;
	case USB_RX_AGG_MIX:
		valueDMA |= RXDMA_AGG_EN;
		valueUSB |= USB_AGG_EN;
		break;
	case USB_RX_AGG_DISABLE:
	default:
		valueDMA &= ~RXDMA_AGG_EN;
		valueUSB &= ~USB_AGG_EN;
		break;
	}

	rtl_write_byte(rtlpriv, REG_TRXDMA_CTRL, valueDMA);
	rtl_write_byte(rtlpriv, REG_USB_SPECIAL_OPTION, valueUSB);

	switch (haldata->UsbRxAggMode) {
	case USB_RX_AGG_DMA:
		rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH, haldata->UsbRxAggPageCount);
		rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH+1, haldata->UsbRxAggPageTimeout);
		break;
	case USB_RX_AGG_USB:
		rtl_write_byte(rtlpriv, REG_USB_AGG_TH, haldata->UsbRxAggBlockCount);
		rtl_write_byte(rtlpriv, REG_USB_AGG_TO, haldata->UsbRxAggBlockTimeout);
		break;
	case USB_RX_AGG_MIX:
		rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH, haldata->UsbRxAggPageCount);
		rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH+1, (haldata->UsbRxAggPageTimeout & 0x1F));/* 0x280[12:8] */
		rtl_write_byte(rtlpriv, REG_USB_AGG_TH, haldata->UsbRxAggBlockCount);
		rtl_write_byte(rtlpriv, REG_USB_AGG_TO, haldata->UsbRxAggBlockTimeout);
		break;
	case USB_RX_AGG_DISABLE:
	default:
		/*  TODO: */
		break;
	}

	switch (PBP_128) {
	case PBP_128:
		haldata->HwRxPageSize = 128;
		break;
	case PBP_64:
		haldata->HwRxPageSize = 64;
		break;
	case PBP_256:
		haldata->HwRxPageSize = 256;
		break;
	case PBP_512:
		haldata->HwRxPageSize = 512;
		break;
	case PBP_1024:
		haldata->HwRxPageSize = 1024;
		break;
	default:
		break;
	}
#endif
}	/*  usb_AggSettingRxUpdate */

static void InitUsbAggregationSetting(struct ieee80211_hw *hw)
{

	/* TODO */
#if 0
	/*  Tx aggregation setting */
	usb_AggSettingTxUpdate(Adapter);

	/*  Rx aggregation setting */
	usb_AggSettingRxUpdate(Adapter);

	/*  201/12/10 MH Add for USB agg mode dynamic switch. */
	haldata->UsbRxHighSpeedMode = false;
#endif
}

static void _InitBeaconParameters(struct ieee80211_hw *hw)
{

	rtl_write_word(rtlpriv, REG_BCN_CTRL, 0x1010);

	/*  TODO: Remove these magic number */
	rtl_write_word(rtlpriv, REG_TBTT_PROHIBIT, 0x6404);/*  ms */
	rtl_write_byte(rtlpriv, REG_DRVERLYINT, DRIVER_EARLY_INT_TIME);/*  5ms */
	rtl_write_byte(rtlpriv, REG_BCNDMATIM, BCN_DMA_ATIME_INT_TIME); /*  2ms */

	/*  Suggested by designer timchen. Change beacon AIFS to the largest number */
	/*  beacause test chip does not contension before sending beacon. by tynli. 2009.11.03 */
	rtl_write_word(rtlpriv, REG_BCNTCFG, 0x660F);

	/* TODO */
	rtlusb->reg_bcn_ctrl_val = rtl_read_byte(rtlpriv, REG_BCN_CTRL);
#if 0
	haldata->RegTxPause = rtl_read_byte(rtlpriv, REG_TXPAUSE);
	haldata->RegFwHwTxQCtrl = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL+2);
	haldata->RegReg542 = rtl_read_byte(rtlpriv, REG_TBTT_PROHIBIT+2);
	haldata->RegCR_1 = rtl_read_byte(rtlpriv, REG_CR+1);
#endif
}

static void _BeaconFunctionEnable(struct ieee80211_hw *hw,
				  bool Enable, bool Linked)
{
	rtl_write_byte(rtlpriv, REG_BCN_CTRL, (BIT(4) | BIT(3) | BIT(1)));

	rtl_write_byte(rtlpriv, REG_RD_CTRL+1, 0x6F);
}

/*  Set CCK and OFDM Block "ON" */
static void _BBTurnOnBlock(struct ieee80211_hw *hw)
{
	rtl_set_bbreg(hw, rFPGA0_RFMOD, bCCKEn, 0x1);
	rtl_set_bbreg(hw, rFPGA0_RFMOD, bOFDMEn, 0x1);
}

enum {
	Antenna_Lfet = 1,
	Antenna_Right = 2,
};

static void _InitAntenna_Selection(struct ieee80211_hw *hw)
{

	if (rtlefuse->antenna_div_cfg == 0)
		return;
	DBG_88E("==>  %s ....\n", __func__);

	rtl_write_dword(rtlpriv, REG_LEDCFG0, rtl_read_dword(rtlpriv, REG_LEDCFG0)|BIT(23));
	rtl_set_bbreg(hw, rFPGA0_XAB_RFParameter, BIT(13), 0x01);

	if (rtl_get_bbreg(hw, rFPGA0_XA_RFInterfaceOE, 0x300) == Antenna_A)
		haldata->CurAntenna = Antenna_A;
	else
		haldata->CurAntenna = Antenna_B;
	DBG_88E("%s,Cur_ant:(%x)%s\n", __func__, haldata->CurAntenna, (haldata->CurAntenna == Antenna_A) ? "Antenna_A" : "Antenna_B");
}

/*-----------------------------------------------------------------------------
 * Function:	HwSuspendModeEnable92Cu()
 *
 * Overview:	HW suspend mode switch.
 *
 * Input:		NONE
 *
 * Output:	NONE
 *
 * Return:	NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	08/23/2010	MHC		HW suspend mode switch test..
 *---------------------------------------------------------------------------*/
enum rt_rf_power_state RfOnOffDetect(struct ieee80211_hw *hw)
{
	u8 val8;
	enum rt_rf_power_state rfpowerstate = ERFOFF;

	if (ppsc->pwrdown_mode) {
		val8 = rtl_read_byte(rtlpriv, REG_HSISR);
		DBG_88E("pwrdown, 0x5c(BIT(7))=%02x\n", val8);
		rfpowerstate = (val8 & BIT(7)) ? ERFOFF : ERFON;
	} else { /*  rf on/off */
		rtl_write_byte(rtlpriv, REG_MAC_PINMUX_CFG, rtl_read_byte(rtlpriv, REG_MAC_PINMUX_CFG)&~(BIT(3)));
		val8 = rtl_read_byte(rtlpriv, REG_GPIO_IO_SEL);
		DBG_88E("GPIO_IN=%02x\n", val8);
		rfpowerstate = (val8 & BIT(3)) ? ERFON: ERFOFF;
	}
	return rfpowerstate;
}	/*  HalDetectPwrDownMode */


static u32 rtl8188eu_hal_init(struct ieee80211_hw *hw)
{
	u8 value8 = 0;
	u16  value16;
	u8 txpktbuf_bndy;
	u32 status = true;
	u32 init_start_time = jiffies;

	#define HAL_INIT_PROFILE_TAG(stage) do {} while (0)

	if (ppsc->rfpwr_state == ERFON) {

		if (rtlphy->iqk_initialized) {
			rtl88eu_phy_iq_calibrate(hw, true);
		} else {
			rtl88eu_phy_iq_calibrate(hw, false);
			rtlphy->iqk_initialized = true;
		}

		ODM_TXPowerTrackingCheck(&haldata->odmpriv);
		rtl88eu_phy_lc_calibrate(hw);

		goto exit;
	}

	status = rtl8188eu_InitPowerOn(hw);
	if (status == false) {
		RT_TRACE(_module_hci_hal_init_c_, _drv_err_, ("Failed to init power on!\n"));
		goto exit;
	}

	/*  Save target channel */
	rtlphy->current_channel = 6;/* default set to 6 */

	/* TODO */
#if 0
	if (pwrctrlpriv->reg_rfoff) {
		pwrctrlpriv->rf_pwrstate = ERFOFF;
	}
#endif

	/*  2010/08/09 MH We need to check if we need to turnon or off RF after detecting */
	/*  HW GPIO pin. Before PHY_RFConfig8192C. */
	/*  2010/08/26 MH If Efuse does not support sective suspend then disable the function. */

	if (!rtlusb->wmm_enable) {
		txpktbuf_bndy = TX_PAGE_BOUNDARY_88E;
	} else {
		/*  for WMM */
		txpktbuf_bndy = WMM_NORMAL_TX_PAGE_BOUNDARY_88E;
	}

	_InitQueueReservedPage(hw);
	_InitQueuePriority(hw);
	_InitPageBoundary(hw);
	_InitTransferPageSize(hw);

	_InitTxBufferBoundary(hw, 0);

	status = rtl88eu_download_fw(hw);
	if (status) {
		DBG_88E("%s: Download Firmware failed!!\n", __func__);
		rtlhal->fw_ready = false;
		return status;
	} 
	
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("Initializeadapt8192CSdio(): Download Firmware Success!!\n"));
	rtlhal->fw_ready = true;
	rtl8188e_InitializeFirmwareVars(hw);
	rtl88eu_phy_mac_config(hw);
	rtl88eu_phy_bb_config(hw);
	rtl88eu_phy_rf_config(hw);
	status = rtl8188e_iol_efuse_patch(hw);
	if (status == false) {
		DBG_88E("%s  rtl8188e_iol_efuse_patch failed\n", __func__);
		goto exit;
	}

	_InitTxBufferBoundary(hw, txpktbuf_bndy);

	status =  InitLLTTable(hw, txpktbuf_bndy);
	if (status == false) {
		RT_TRACE(_module_hci_hal_init_c_, _drv_err_, ("Failed to init LLT table\n"));
		goto exit;
	}

	/*  Get Rx PHY status in order to report RSSI and others. */
	_InitDriverInfoSize(hw, DRVINFO_SZ);

	_InitInterrupt(hw);
	hal_init_macaddr(hw);/* set mac_address */
	_InitNetworkType(hw);/* set msr */
	_InitWMACSetting(hw);
	_InitAdaptiveCtrl(hw);
	_InitEDCA(hw);
	_InitRetryFunction(hw);
	InitUsbAggregationSetting(hw);
	_InitBeaconParameters(hw);
	/*  Init CR MACTXEN, MACRXEN after setting RxFF boundary REG_TRXFF_BNDY to patch */
	/*  Hw bug which Hw initials RxFF boundary size to a value which is larger than the real Rx buffer size in 88E. */
	/*  Enable MACTXEN/MACRXEN block */
	value16 = rtl_read_word(rtlpriv, REG_CR);
	value16 |= (MACTXEN | MACRXEN);
	rtl_write_byte(rtlpriv, REG_CR, value16);

	/* TODO */
#if 0
	if (haldata->bRDGEnable)
		_InitRDGSetting(Adapter);
#endif

	/* Enable TX Report */
	/* Enable Tx Report Timer */
	value8 = rtl_read_byte(rtlpriv, REG_TX_RPT_CTRL);
	rtl_write_byte(rtlpriv,  REG_TX_RPT_CTRL, (value8|BIT(1)|BIT(0)));
	/* Set MAX RPT MACID */
	rtl_write_byte(rtlpriv,  REG_TX_RPT_CTRL+1, 2);/* FOR sta mode ,0: bc/mc ,1:AP */
	/* Tx RPT Timer. Unit: 32us */
	rtl_write_word(rtlpriv, REG_TX_RPT_TIME, 0xCdf0);

	rtl_write_byte(rtlpriv, REG_EARLY_MODE_CONTROL, 0);

	rtl_write_word(rtlpriv, REG_PKT_VO_VI_LIFE_TIME, 0x0400);	/*  unit: 256us. 256ms */
	rtl_write_word(rtlpriv, REG_PKT_BE_BK_LIFE_TIME, 0x0400);	/*  unit: 256us. 256ms */

	/* Keep RfRegChnlVal for later use. */
	rtlphy->rfreg_chnlval[0] = phy_query_rf_reg(hw, (enum rf_radio_path)0, RF_CHNLBW, bRFRegOffsetMask);
	rtlphy->rfreg_chnlval[1] = phy_query_rf_reg(hw, (enum rf_radio_path)1, RF_CHNLBW, bRFRegOffsetMask);

	_BBTurnOnBlock(hw);

	invalidate_cam_all(hw);

	/*  2010/12/17 MH We need to set TX power according to EFUSE content at first. */
	phy_set_tx_power_level(hw, rtlphy->current_channel);

/*  Move by Neo for USB SS to below setp */
/* _RfPowerSave(Adapter); */

	_InitAntenna_Selection(hw);

	/*  */
	/*  Disable BAR, suggested by Scott */
	/*  2010.04.09 add by hpfan */
	/*  */
	rtl_write_dword(rtlpriv, REG_BAR_MODE_CTRL, 0x0201ffff);

	/*  HW SEQ CTRL */
	/* set 0x0 to 0xFF by tynli. Default enable HW SEQ NUM. */
	rtl_write_byte(rtlpriv, REG_HWSEQ_CTRL, 0xFF);

	if (rtlusb->wmm_enable)
		rtl_write_word(rtlpriv, REG_FAST_EDCA_CTRL, 0);

	/* Nav limit , suggest by scott */
	rtl_write_byte(rtlpriv, 0x652, 0x0);

	rtl8188e_InitHalDm(hw);

	/*  2010/08/11 MH Merge from 8192SE for Minicard init. We need to confirm current radio status */
	/*  and then decide to enable RF or not.!!!??? For Selective suspend mode. We may not */
	/*  call initstruct ieee80211_hw. May cause some problem?? */
	/*  Fix the bug that Hw/Sw radio off before S3/S4, the RF off action will not be executed */
	/*  in MgntActSet_RF_State() after wake up, because the value of haldata->eRFPowerState */
	/*  is the same as eRfOff, we should change it to eRfOn after we config RF parameters. */
	/*  Added by tynli. 2010.03.30. */
	pwrctrlpriv->rf_pwrstate = ERFON;

	/*  enable Tx report. */
	rtl_write_byte(rtlpriv,  REG_FWHW_TXQ_CTRL+1, 0x0F);

	/*  Suggested by SD1 pisa. Added by tynli. 2011.10.21. */
	rtl_write_byte(rtlpriv, REG_EARLY_MODE_CONTROL+3, 0x01);/* Pretx_en, for WEP/TKIP SEC */

	/* tynli_test_tx_report. */
	rtl_write_word(rtlpriv, REG_TX_RPT_TIME, 0x3DF0);

	/* enable tx DMA to drop the redundate data of packet */
	rtl_write_word(rtlpriv, REG_TXDMA_OFFSET_CHK, (rtl_read_word(rtlpriv, REG_TXDMA_OFFSET_CHK) | DROP_DATA_EN));

	/*  2010/08/26 MH Merge from 8192CE. */
	if (pwrctrlpriv->rf_pwrstate == ERFON) {
		if (rtlphy->iqk_initialized) {
				rtl88eu_phy_iq_calibrate(hw, true);
		} else {
			rtl88eu_phy_iq_calibrate(hw, false);
			rtlphy->iqk_initialized = true;
		}
		ODM_TXPowerTrackingCheck(&haldata->odmpriv);
		rtl88eu_phy_lc_calibrate(hw);
	}

/* HAL_INIT_PROFILE_TAG(HAL_INIT_STAGES_INIT_PABIAS); */
/*	_InitPABias(Adapter); */
	rtl_write_byte(rtlpriv, REG_USB_HRPWM, 0);

	/* ack for xmit mgmt frames. */
	rtl_write_dword(rtlpriv, REG_FWHW_TXQ_CTRL, rtl_read_dword(rtlpriv, REG_FWHW_TXQ_CTRL)|BIT(12));

exit:
	DBG_88E("%s in %dms\n", __func__, rtw_get_passing_time_ms(init_start_time));
	return status;
}

static void CardDisableRTL8188EU(struct ieee80211_hw *hw)
{
	u8 val8;

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("CardDisableRTL8188EU\n"));

	/* Stop Tx Report Timer. 0x4EC[Bit1]=b'0 */
	val8 = rtl_read_byte(rtlpriv, REG_TX_RPT_CTRL);
	rtl_write_byte(rtlpriv, REG_TX_RPT_CTRL, val8&(~BIT(1)));

	/*  stop rx */
	rtl_write_byte(rtlpriv, REG_CR, 0x0);

	/*  Run LPS WL RFOFF flow */
	rtl88eu_pwrseqcmdparsing(hw, PWR_CUT_ALL_MSK,
				 PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK,
				 Rtl8188E_NIC_LPS_ENTER_FLOW);

	/*  2. 0x1F[7:0] = 0		turn off RF */

	val8 = rtl_read_byte(rtlpriv, REG_MCUFWDL);
	if ((val8 & RAM_DL_SEL) && rtlhal->fw_ready) { /* 8051 RAM code */
		/*  Reset MCU 0x2[10]=0. */
		val8 = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN+1);
		val8 &= ~BIT(2);	/*  0x2[10], FEN_CPUEN */
		rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN+1, val8);
	}

	/*  reset MCU ready status */
	rtl_write_byte(rtlpriv, REG_MCUFWDL, 0);

	/* YJ,add,111212 */
	/* Disable 32k */
	val8 = rtl_read_byte(rtlpriv, REG_32K_CTRL);
	rtl_write_byte(rtlpriv, REG_32K_CTRL, val8&(~BIT(0)));

	/*  Card disable power action flow */
	rtl88eu_pwrseqcmdparsing(hw, PWR_CUT_ALL_MSK,
				 PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK,
				 Rtl8188E_NIC_DISABLE_FLOW);

	/*  Reset MCU IO Wrapper */
	val8 = rtl_read_byte(rtlpriv, REG_RSV_CTRL+1);
	rtl_write_byte(rtlpriv, REG_RSV_CTRL+1, (val8&(~BIT(3))));
	val8 = rtl_read_byte(rtlpriv, REG_RSV_CTRL+1);
	rtl_write_byte(rtlpriv, REG_RSV_CTRL+1, val8|BIT(3));

	/* YJ,test add, 111207. For Power Consumption. */
	val8 = rtl_read_byte(rtlpriv, GPIO_IN);
	rtl_write_byte(rtlpriv, GPIO_OUT, val8);
	rtl_write_byte(rtlpriv, GPIO_IO_SEL, 0xFF);/* Reg0x46 */

	val8 = rtl_read_byte(rtlpriv, REG_GPIO_IO_SEL);
	rtl_write_byte(rtlpriv, REG_GPIO_IO_SEL, (val8<<4));
	val8 = rtl_read_byte(rtlpriv, REG_GPIO_IO_SEL+1);
	rtl_write_byte(rtlpriv, REG_GPIO_IO_SEL+1, val8|0x0F);/* Reg0x43 */
	rtl_write_dword(rtlpriv, REG_BB_PAD_CTRL, 0x00080808);/* set LNA ,TRSW,EX_PA Pin to output mode */
	/* TODO */
#if 0
	haldata->bMacPwrCtrlOn = false;
#endif
	rtlhal->fw_ready = false;
}
static void rtl8192cu_hw_power_down(struct ieee80211_hw *hw)
{
	/*  2010/-8/09 MH For power down module, we need to enable register block contrl reg at 0x1c. */
	/*  Then enable power down control bit of register 0x04 BIT(4) and BIT(15) as 1. */

	/*  Enable register area 0x0-0xc. */
	rtl_write_byte(rtlpriv, REG_RSV_CTRL, 0x0);
	rtl_write_word(rtlpriv, REG_APS_FSMCO, 0x8812);
}

/* TODO */
static u32 rtl8188eu_hal_deinit(struct ieee80211_hw *hw)
{

	DBG_88E("==> %s\n", __func__);

	rtl_write_dword(rtlpriv, REG_HIMR_88E, IMR_DISABLED_88E);
	rtl_write_dword(rtlpriv, REG_HIMRE_88E, IMR_DISABLED_88E);

	DBG_88E("bkeepfwalive(%x)\n", ppsc->rfpwr_state);
	if (ppsc->rfpwr_state == ERFON) {
		if ((Adapter->pwrctrlpriv.bHWPwrPindetect) && (Adapter->pwrctrlpriv.bHWPowerdown))
			rtl8192cu_hw_power_down(hw);
	} else {
		if (Adapter->hw_init_completed) {
			CardDisableRTL8188EU(hw);

			if ((Adapter->pwrctrlpriv.bHWPwrPindetect) && (Adapter->pwrctrlpriv.bHWPowerdown))
				rtl8192cu_hw_power_down(hw);
		}
	}
	return true;
 }

/* TODO */
#if 0
static unsigned int rtl8188eu_inirp_init(struct ieee80211_hw *hw)
{
	u8 i;
	struct recv_buf *precvbuf;
	uint	status;

	status = true;

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("===> usb_inirp_init\n"));

	precvpriv->ff_hwaddr = RECV_BULK_IN_ADDR;

	/* issue Rx irp to receive data */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	for (i = 0; i < NR_RECVBUFF; i++) {
		if (usb_read_port(Adapter, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf) == false) {
			RT_TRACE(_module_hci_hal_init_c_, _drv_err_, ("usb_rx_init: usb_read_port error\n"));
			status = false;
			goto exit;
		}

		precvbuf++;
		precvpriv->free_recv_buf_queue_cnt--;
	}

exit:

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("<=== usb_inirp_init\n"));


	return status;
}

static unsigned int rtl8188eu_inirp_deinit(struct ieee80211_hw *hw)
{
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("\n ===> usb_rx_deinit\n"));

	usb_read_port_cancel(Adapter);

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("\n <=== usb_rx_deinit\n"));

	return true;
}
#endif
/*  */
/*  */
/*	EEPROM/EFUSE Content Parsing */
/*  */
/*  */
static void Hal_EfuseParsePIDVID_8188EU(struct ieee80211_hw *hw, u8 *hwinfo, bool AutoLoadFail)
{

	if (!AutoLoadFail) {
		/*  VID, PID */
		rtlefuse->eeprom_vid = EF2BYTE(*(__le16 *)&hwinfo[EEPROM_VID_88EU]);
		rtlefuse->eeprom_did = EF2BYTE(*(__le16 *)&hwinfo[EEPROM_PID_88EU]);

		/*  Customer ID, 0x00 and 0xff are reserved for Realtek. */
		rtlefuse->eeprom_oemid = *(u8 *)&hwinfo[EEPROM_CUSTOMERID_88E];
	} else {
		rtlefuse->eeprom_vid			= EEPROM_Default_VID;
		rtlefuse->eeprom_did			= EEPROM_Default_PID;

		/*  Customer ID, 0x00 and 0xff are reserved for Realtek. */
		rtlefuse->eeprom_oemid		= EEPROM_Default_CustomerID;
	}

	DBG_88E("VID = 0x%04X, PID = 0x%04X\n", rtlefuse->eeprom_vid, rtlefuse->eeprom_did);
	DBG_88E("Customer ID: 0x%02X, SubCustomer ID: 0x%02X\n", rtlefuse->eeprom_oemid, haldata->EEPROMSubCustomerID);
}

static void Hal_EfuseParseMACAddr_8188EU(struct ieee80211_hw *hw, u8 *hwinfo, bool AutoLoadFail)
{
	u16 i;
	u8 sMacAddr[6] = {0x00, 0xE0, 0x4C, 0x81, 0x88, 0x02};

	if (AutoLoadFail) {
		for (i = 0; i < 6; i++)
			rtlefuse->dev_addr[i] = sMacAddr[i];
	} else {
		/* Read Permanent MAC address */
		memcpy(rtlefuse->dev_addr, &hwinfo[EEPROM_MAC_ADDR_88EU], ETH_ALEN);
	}
	RT_TRACE(_module_hci_hal_init_c_, _drv_notice_,
		 ("Hal_EfuseParseMACAddr_8188EU: Permanent Address = %pM\n",
		 eeprom->mac_addr));
}

static void readAdapterInfo_8188EU(
		struct ieee80211_hw *hw
	)
{

	/* parse the eeprom/efuse content */
	Hal_EfuseParseIDCode88E(hw, eeprom->efuse_eeprom_data);
	Hal_EfuseParsePIDVID_8188EU(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_EfuseParseMACAddr_8188EU(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);

	Hal_ReadPowerSavingMode88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_ReadTxPowerInfo88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_EfuseParseEEPROMVer88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	rtl8188e_EfuseParseChnlPlan(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_EfuseParseXtal_8188E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_EfuseParseCustomerID88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_ReadAntennaDiversity88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_EfuseParseBoardType88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);
	Hal_ReadThermalMeter_88E(hw, eeprom->efuse_eeprom_data, eeprom->bautoload_fail_flag);

}

static void _ReadPROMContent(
	struct ieee80211_hw *hw
	)
{
	u8 eeValue;

	/* check system boot selection */
	eeValue = rtl_read_byte(rtlpriv, REG_9346CR);
	
	if (eeValue & BIT(4)) {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_DMESG, "Boot from EEPROM\n");
		rtlefuse->epromtype = EEPROM_93C46;
	} else {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_DMESG, "Boot from EFUSE\n");
		rtlefuse->epromtype = EEPROM_BOOT_EFUSE;
	}
	if (eeValue & BIT(5)) {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "Autoload OK\n");
		rtlefuse->autoload_failflag = false;
	} else {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_EMERG, "Autoload ERR!!\n");
	}

	Hal_InitPGData88E(hw);
	readAdapterInfo_8188EU(hw);
}

static void _ReadRFType(struct ieee80211_hw *hw)
{
	/* TODO */
#if 0
	haldata->rf_chip = RF_6052;
#endif
}

static void _ReadAdapterInfo8188EU(struct ieee80211_hw *hw)
{
	u32 start = jiffies;

	MSG_88E("====> %s\n", __func__);

	_ReadRFType(hw);/* rf_chip -> _InitRFType() */
	_ReadPROMContent(hw);

	MSG_88E("<==== %s in %d ms\n", __func__, rtw_get_passing_time_ms(start));
}

#define GPIO_DEBUG_PORT_NUM 0
static void rtl8192cu_trigger_gpio_0(struct ieee80211_hw *hw)
{
}

static void ResumeTxBeacon(struct ieee80211_hw *hw)
{

	/*  2010.03.01. Marked by tynli. No need to call workitem beacause we record the value */
	/*  which should be read from register to a global variable. */

	u8 tmp1byte;
	tmp1byte = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2);
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL+2, tmp1byte | BIT(6));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+1, 0xff);
	tmp1byte = rtl_read_byte(rtlpriv, REG_TBTT_PROHIBIT + 2);
	tmp1byte |= BIT(0);
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+2, tmp1byte);
}

static void StopTxBeacon(struct ieee80211_hw *hw)
{
	/*  2010.03.01. Marked by tynli. No need to call workitem beacause we record the value */
	/*  which should be read from register to a global variable. */

	u8 tmp1byte;
	tmp1byte = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2);
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL+2, tmp1byte & (~BIT(6)));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+1, 0x64);
	tmp1byte = rtl_read_byte(rtlpriv, REG_TBTT_PROHIBIT + 2);
	tmp1byte &= ~(BIT(0));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+2, tmp1byte);

	 /* todo: CheckFwRsvdPageContent(Adapter);  2010.06.23. Added by tynli. */
}

static void hw_var_set_opmode(struct ieee80211_hw *hw, u8 variable, u8 *val)
{
	u8 val8;
	u8 mode = *((u8 *)val);

	/*  disable Port0 TSF update */
	rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|BIT(4));

	/*  set net_type */
	val8 = rtl_read_byte(rtlpriv, MSR)&0x0c;
	val8 |= mode;
	rtl_write_byte(rtlpriv, MSR, val8);

	DBG_88E("%s()-%d mode = %d\n", __func__, __LINE__, mode);

	if ((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_)) {
		StopTxBeacon(hw);

		rtl_write_byte(rtlpriv, REG_BCN_CTRL, 0x19);/* disable atim wnd */
	} else if ((mode == _HW_STATE_ADHOC_)) {
		ResumeTxBeacon(hw);
		rtl_write_byte(rtlpriv, REG_BCN_CTRL, 0x1a);
	} else if (mode == _HW_STATE_AP_) {
		ResumeTxBeacon(hw);

		rtl_write_byte(rtlpriv, REG_BCN_CTRL, 0x12);

		/* Set RCR */
		rtl_write_dword(rtlpriv, REG_RCR, 0x7000208e);/* CBSSID_DATA must set to 0,reject ICV_ERR packet */
		/* enable to rx data frame */
		rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);
		/* enable to rx ps-poll */
		rtl_write_word(rtlpriv, REG_RXFLTMAP1, 0x0400);

		/* Beacon Control related register for first time */
		rtl_write_byte(rtlpriv, REG_BCNDMATIM, 0x02); /*  2ms */

		rtl_write_byte(rtlpriv, REG_ATIMWND, 0x0a); /*  10ms */
		rtl_write_word(rtlpriv, REG_BCNTCFG, 0x00);
		rtl_write_word(rtlpriv, REG_TBTT_PROHIBIT, 0xff04);
		rtl_write_word(rtlpriv, REG_TSFTR_SYN_OFFSET, 0x7fff);/*  +32767 (~32ms) */

		/* reset TSF */
		rtl_write_byte(rtlpriv, REG_DUAL_TSF_RST, BIT(0));

		/* BIT(3) - If set 0, hw will clr bcnq when tx becon ok/fail or port 0 */
		rtl_write_byte(rtlpriv, REG_MBID_NUM, rtl_read_byte(rtlpriv, REG_MBID_NUM) | BIT(3) | BIT(4));

		/* enable BCN0 Function for if1 */
		/* don't enable update TSF0 for if1 (due to TSF update when beacon/probe rsp are received) */
		rtl_write_byte(rtlpriv, REG_BCN_CTRL, (DIS_TSF_UDT0_NORMAL_CHIP|EN_BCN_FUNCTION | BIT(1)));

		/* dis BCN1 ATIM  WND if if2 is station */
		rtl_write_byte(rtlpriv, REG_BCN_CTRL_1, rtl_read_byte(rtlpriv, REG_BCN_CTRL_1) | BIT(0));
	}
}

static void hw_var_set_macaddr(struct ieee80211_hw *hw, u8 variable, u8 *val)
{
	u8 idx = 0;
	u32 reg_macid;

	reg_macid = REG_MACID;

	for (idx = 0; idx < 6; idx++)
		rtl_write_byte(rtlpriv, (reg_macid+idx), val[idx]);
}

static void hw_var_set_bssid(struct ieee80211_hw *hw, u8 variable, u8 *val)
{
	u8 idx = 0;
	u32 reg_bssid;

	reg_bssid = REG_BSSID;

	for (idx = 0; idx < 6; idx++)
		rtl_write_byte(rtlpriv, (reg_bssid+idx), val[idx]);
}

static void hw_var_set_bcn_func(struct ieee80211_hw *hw, u8 variable, u8 *val)
{
	u32 bcn_ctrl_reg;

	bcn_ctrl_reg = REG_BCN_CTRL;

	if (*((u8 *)val))
		rtl_write_byte(rtlpriv, bcn_ctrl_reg, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
	else
		rtl_write_byte(rtlpriv, bcn_ctrl_reg, rtl_read_byte(rtlpriv, bcn_ctrl_reg)&(~(EN_BCN_FUNCTION | EN_TXBCN_RPT)));
}

static void SetHwReg8188EU(struct ieee80211_hw *hw, u8 variable, u8 *val)
{

	switch (variable) {
	case HW_VAR_MEDIA_STATUS:
		{
			u8 val8;

			val8 = rtl_read_byte(rtlpriv, MSR)&0x0c;
			val8 |= *((u8 *)val);
			rtl_write_byte(rtlpriv, MSR, val8);
		}
		break;
	case HW_VAR_MEDIA_STATUS1:
		{
			u8 val8;

			val8 = rtl_read_byte(rtlpriv, MSR) & 0x03;
			val8 |= *((u8 *)val) << 2;
			rtl_write_byte(rtlpriv, MSR, val8);
		}
		break;
	case HW_VAR_SET_OPMODE:
		hw_var_set_opmode(hw, variable, val);
		break;
	case HW_VAR_MAC_ADDR:
		hw_var_set_macaddr(hw, variable, val);
		break;
	case HW_VAR_BSSID:
		hw_var_set_bssid(hw, variable, val);
		break;
	case HW_VAR_BASIC_RATE:
		{
			u16 BrateCfg = 0;
			u8 RateIndex = 0;

			/*  2007.01.16, by Emily */
			/*  Select RRSR (in Legacy-OFDM and CCK) */
			/*  For 8190, we select only 24M, 12M, 6M, 11M, 5.5M, 2M, and 1M from the Basic rate. */
			/*  We do not use other rates. */
			HalSetBrateCfg(hw, val, &BrateCfg);
			DBG_88E("HW_VAR_BASIC_RATE: BrateCfg(%#x)\n", BrateCfg);

			/* 2011.03.30 add by Luke Lee */
			/* CCK 2M ACK should be disabled for some BCM and Atheros AP IOT */
			/* because CCK 2M has poor TXEVM */
			/* CCK 5.5M & 11M ACK should be enabled for better performance */

			BrateCfg = (BrateCfg | 0xd) & 0x15d;

			BrateCfg |= 0x01; /*  default enable 1M ACK rate */
			/*  Set RRSR rate table. */
			rtl_write_byte(rtlpriv, REG_RRSR, BrateCfg & 0xff);
			rtl_write_byte(rtlpriv, REG_RRSR+1, (BrateCfg >> 8) & 0xff);
			rtl_write_byte(rtlpriv, REG_RRSR+2, rtl_read_byte(rtlpriv, REG_RRSR+2)&0xf0);

			/*  Set RTS initial rate */
			while (BrateCfg > 0x1) {
				BrateCfg >>= 1;
				RateIndex++;
			}
			/*  Ziv - Check */
			rtl_write_byte(rtlpriv, REG_INIRTS_RATE_SEL, RateIndex);
		}
		break;
	case HW_VAR_TXPAUSE:
		rtl_write_byte(rtlpriv, REG_TXPAUSE, *((u8 *)val));
		break;
	case HW_VAR_BCN_FUNC:
		hw_var_set_bcn_func(hw, variable, val);
		break;
	case HW_VAR_CORRECT_TSF:
		{
			u64	tsf;
			tsf = pmlmeext->TSFValue - rtw_modular64(pmlmeext->TSFValue, (pmlmeinfo->bcn_interval*1024)) - 1024; /* us */

			if (((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
				StopTxBeacon(hw);

			/* disable related TSF function */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)&(~BIT(3)));

			rtl_write_dword(rtlpriv, REG_TSFTR, tsf);
			rtl_write_dword(rtlpriv, REG_TSFTR+4, tsf>>32);

			/* enable related TSF function */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|BIT(3));

			if (((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
				ResumeTxBeacon(hw);
		}
		break;
	case HW_VAR_CHECK_BSSID:
		if (*((u8 *)val)) {
			rtl_write_dword(rtlpriv, REG_RCR, rtl_read_dword(rtlpriv, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);
		} else {
			u32 val32;

			val32 = rtl_read_dword(rtlpriv, REG_RCR);

			val32 &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);

			rtl_write_dword(rtlpriv, REG_RCR, val32);
		}
		break;
	case HW_VAR_MLME_DISCONNECT:
		/* Set RCR to not to receive data frame when NO LINK state */
		/* reject all data frames */
		rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0x00);

		/* reset TSF */
		rtl_write_byte(rtlpriv, REG_DUAL_TSF_RST, (BIT(0)|BIT(1)));

		/* disable update TSF */
		rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|BIT(4));
		break;
	case HW_VAR_MLME_SITESURVEY:
/* TODO */
#if 0
		if (*((u8 *)val)) { /* under sitesurvey */
			/* config RCR to receive different BSSID & not to receive data frame */
			u32 v = rtl_read_dword(rtlpriv, REG_RCR);
			v &= ~(RCR_CBSSID_BCN);
			rtl_write_dword(rtlpriv, REG_RCR, v);
			/* reject all data frame */
			rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0x00);

			/* disable update TSF */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|BIT(4));
		} else { /* sitesurvey done */
			struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;
			struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

			if ((is_client_associated_to_ap(Adapter)) ||
			    ((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)) {
				/* enable to rx data frame */
				rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);

				/* enable update TSF */
				rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)&(~BIT(4)));
			} else if ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE) {
				rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);
				/* enable update TSF */
				rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)&(~BIT(4)));
			}
			if ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE) {
				rtl_write_dword(rtlpriv, REG_RCR, rtl_read_dword(rtlpriv, REG_RCR)|RCR_CBSSID_BCN);
			} else {
				if (Adapter->in_cta_test) {
					u32 v = rtl_read_dword(rtlpriv, REG_RCR);
					v &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);/*  RCR_ADF */
					rtl_write_dword(rtlpriv, REG_RCR, v);
				} else {
					rtl_write_dword(rtlpriv, REG_RCR, rtl_read_dword(rtlpriv, REG_RCR)|RCR_CBSSID_BCN);
				}
			}
		}
		break;
	case HW_VAR_MLME_JOIN:
		{
			u8 RetryLimit = 0x30;
			u8 type = *((u8 *)val);
			struct mlme_priv	*pmlmepriv = &Adapter->mlmepriv;

			if (type == 0) { /*  prepare to join */
				/* enable to rx data frame.Accept all data frame */
				rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);

				if (Adapter->in_cta_test) {
					u32 v = rtl_read_dword(rtlpriv, REG_RCR);
					v &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);/*  RCR_ADF */
					rtl_write_dword(rtlpriv, REG_RCR, v);
				} else {
					rtl_write_dword(rtlpriv, REG_RCR, rtl_read_dword(rtlpriv, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);
				}

				if (check_fwstate(pmlmepriv, WIFI_STATION_STATE))
					RetryLimit = (haldata->CustomerID == RT_CID_CCX) ? 7 : 48;
				else /*  Ad-hoc Mode */
					RetryLimit = 0x7;
			} else if (type == 1) {
				/* joinbss_event call back when join res < 0 */
				rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0x00);
			} else if (type == 2) {
				/* sta add event call back */
				/* enable update TSF */
				rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)&(~BIT(4)));

				if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE))
					RetryLimit = 0x7;
			}
			rtl_write_word(rtlpriv, REG_RL, RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT);
		}
		break;
#endif
	case HW_VAR_BEACON_INTERVAL:
		rtl_write_word(rtlpriv, REG_BCN_INTERVAL, *((u16 *)val));
		break;
	case HW_VAR_SLOT_TIME:
		{
			u8 u1bAIFS, aSifsTime;
			rtl_write_byte(rtlpriv, REG_SLOT, val[0]);

			if (rtlusb->wmm_enable == 0) {
				/* TODO */
#if 0
				if (pmlmeext->cur_wireless_mode == WIRELESS_11B)
					aSifsTime = 10;
				else
					aSifsTime = 16;

				u1bAIFS = aSifsTime + (2 * pmlmeinfo->slotTime);

				/*  <Roger_EXP> Temporary removed, 2008.06.20. */
				rtl_write_byte(rtlpriv, REG_EDCA_VO_PARAM, u1bAIFS);
				rtl_write_byte(rtlpriv, REG_EDCA_VI_PARAM, u1bAIFS);
				rtl_write_byte(rtlpriv, REG_EDCA_BE_PARAM, u1bAIFS);
				rtl_write_byte(rtlpriv, REG_EDCA_BK_PARAM, u1bAIFS);
#endif
			}
		}
		break;
	case HW_VAR_RESP_SIFS:
		/* RESP_SIFS for CCK */
		rtl_write_byte(rtlpriv, REG_R2T_SIFS, val[0]); /*  SIFS_T2T_CCK (0x08) */
		rtl_write_byte(rtlpriv, REG_R2T_SIFS+1, val[1]); /* SIFS_R2T_CCK(0x08) */
		/* RESP_SIFS for OFDM */
		rtl_write_byte(rtlpriv, REG_T2T_SIFS, val[2]); /* SIFS_T2T_OFDM (0x0a) */
		rtl_write_byte(rtlpriv, REG_T2T_SIFS+1, val[3]); /* SIFS_R2T_OFDM(0x0a) */
		break;
	case HW_VAR_ACK_PREAMBLE:
		{
			u8 regTmp;
			u8 bShortPreamble = *((bool *)val);
			/*  Joseph marked out for Netgear 3500 TKIP channel 7 issue.(Temporarily) */
			regTmp = (haldata->nCur40MhzPrimeSC)<<5;
			if (bShortPreamble)
				regTmp |= 0x80;

			rtl_write_byte(rtlpriv, REG_RRSR+2, regTmp);
		}
		break;
	case HW_VAR_SEC_CFG:
		rtl_write_byte(rtlpriv, REG_SECCFG, *((u8 *)val));
		break;
	case HW_VAR_DM_FLAG:
		podmpriv->SupportAbility = *((u8 *)val);
		break;
	case HW_VAR_DM_FUNC_OP:
		if (val[0])
			podmpriv->BK_SupportAbility = podmpriv->SupportAbility;
		else
			podmpriv->SupportAbility = podmpriv->BK_SupportAbility;
		break;
	case HW_VAR_DM_FUNC_SET:
		if (*((u32 *)val) == DYNAMIC_ALL_FUNC_ENABLE) {
			pdmpriv->DMFlag = pdmpriv->InitDMFlag;
			podmpriv->SupportAbility =	pdmpriv->InitODMFlag;
		} else {
			podmpriv->SupportAbility |= *((u32 *)val);
		}
		break;
	case HW_VAR_DM_FUNC_CLR:
		podmpriv->SupportAbility &= *((u32 *)val);
		break;
	case HW_VAR_CAM_EMPTY_ENTRY:
		{
			u8 ucIndex = *((u8 *)val);
			u8 i;
			u32 ulCommand = 0;
			u32 ulContent = 0;
			u32 ulEncAlgo = CAM_AES;

			for (i = 0; i < CAM_CONTENT_COUNT; i++) {
				/*  filled id in CAM config 2 byte */
				if (i == 0)
					ulContent |= (ucIndex & 0x03) | ((u16)(ulEncAlgo)<<2);
				else
					ulContent = 0;
				/*  polling bit, and No Write enable, and address */
				ulCommand = CAM_CONTENT_COUNT*ucIndex+i;
				ulCommand = ulCommand | CAM_POLLINIG|CAM_WRITE;
				/*  write content 0 is equall to mark invalid */
				rtl_write_dword(rtlpriv, WCAMI, ulContent);  /* delay_ms(40); */
				rtl_write_dword(rtlpriv, RWCAM, ulCommand);  /* delay_ms(40); */
			}
		}
		break;
	case HW_VAR_CAM_INVALID_ALL:
		rtl_write_dword(rtlpriv, RWCAM, BIT(31)|BIT(30));
		break;
	case HW_VAR_CAM_WRITE:
		{
			u32 cmd;
			u32 *cam_val = (u32 *)val;
			rtl_write_dword(rtlpriv, WCAMI, cam_val[0]);

			cmd = CAM_POLLINIG | CAM_WRITE | cam_val[1];
			rtl_write_dword(rtlpriv, RWCAM, cmd);
		}
		break;
	case HW_VAR_AC_PARAM_VO:
		rtl_write_dword(rtlpriv, REG_EDCA_VO_PARAM, ((u32 *)(val))[0]);
		break;
	case HW_VAR_AC_PARAM_VI:
		rtl_write_dword(rtlpriv, REG_EDCA_VI_PARAM, ((u32 *)(val))[0]);
		break;
	case HW_VAR_AC_PARAM_BE:
		haldata->AcParam_BE = ((u32 *)(val))[0];
		rtl_write_dword(rtlpriv, REG_EDCA_BE_PARAM, ((u32 *)(val))[0]);
		break;
	case HW_VAR_AC_PARAM_BK:
		rtl_write_dword(rtlpriv, REG_EDCA_BK_PARAM, ((u32 *)(val))[0]);
		break;
	case HW_VAR_ACM_CTRL:
		{
			u8 acm_ctrl = *((u8 *)val);
			u8 AcmCtrl = rtl_read_byte(rtlpriv, REG_ACMHWCTRL);

			if (acm_ctrl > 1)
				AcmCtrl = AcmCtrl | 0x1;

			if (acm_ctrl & BIT(3))
				AcmCtrl |= AcmHw_VoqEn;
			else
				AcmCtrl &= (~AcmHw_VoqEn);

			if (acm_ctrl & BIT(2))
				AcmCtrl |= AcmHw_ViqEn;
			else
				AcmCtrl &= (~AcmHw_ViqEn);

			if (acm_ctrl & BIT(1))
				AcmCtrl |= AcmHw_BeqEn;
			else
				AcmCtrl &= (~AcmHw_BeqEn);

			DBG_88E("[HW_VAR_ACM_CTRL] Write 0x%X\n", AcmCtrl);
			rtl_write_byte(rtlpriv, REG_ACMHWCTRL, AcmCtrl);
		}
		break;
	case HW_VAR_AMPDU_MIN_SPACE:
		{
			u8 MinSpacingToSet;
			u8 SecMinSpace;

			MinSpacingToSet = *((u8 *)val);
			if (MinSpacingToSet <= 7) {
				switch (Adapter->securitypriv.dot11PrivacyAlgrthm) {
				case _NO_PRIVACY_:
				case _AES_:
					SecMinSpace = 0;
					break;
				case _WEP40_:
				case _WEP104_:
				case _TKIP_:
				case _TKIP_WTMIC_:
					SecMinSpace = 6;
					break;
				default:
					SecMinSpace = 7;
					break;
				}
				if (MinSpacingToSet < SecMinSpace)
					MinSpacingToSet = SecMinSpace;
				rtl_write_byte(rtlpriv, REG_AMPDU_MIN_SPACE, (rtl_read_byte(rtlpriv, REG_AMPDU_MIN_SPACE) & 0xf8) | MinSpacingToSet);
			}
		}
		break;
	case HW_VAR_AMPDU_FACTOR:
		{
			u8 RegToSet_Normal[4] = {0x41, 0xa8, 0x72, 0xb9};
			u8 FactorToSet;
			u8 *pRegToSet;
			u8 index = 0;

			pRegToSet = RegToSet_Normal; /*  0xb972a841; */
			FactorToSet = *((u8 *)val);
			if (FactorToSet <= 3) {
				FactorToSet = 1 << (FactorToSet + 2);
				if (FactorToSet > 0xf)
					FactorToSet = 0xf;

				for (index = 0; index < 4; index++) {
					if ((pRegToSet[index] & 0xf0) > (FactorToSet<<4))
						pRegToSet[index] = (pRegToSet[index] & 0x0f) | (FactorToSet<<4);

					if ((pRegToSet[index] & 0x0f) > FactorToSet)
						pRegToSet[index] = (pRegToSet[index] & 0xf0) | (FactorToSet);

					rtl_write_byte(rtlpriv, (REG_AGGLEN_LMT+index), pRegToSet[index]);
				}
			}
		}
		break;
	case HW_VAR_RXDMA_AGG_PG_TH:
		{
			u8 threshold = *((u8 *)val);
			if (threshold == 0)
				threshold = haldata->UsbRxAggPageCount;
			rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH, threshold);
		}
		break;
	case HW_VAR_SET_RPWM:
		break;
	case HW_VAR_H2C_FW_PWRMODE:
		{
			u8 psmode = (*(u8 *)val);

			/*  Forece leave RF low power mode for 1T1R to prevent conficting setting in Fw power */
			/*  saving sequence. 2010.06.07. Added by tynli. Suggested by SD3 yschang. */
			if ((psmode != PS_MODE_ACTIVE) && (!IS_92C_SERIAL(haldata->VersionID)))
				ODM_RF_Saving(podmpriv, true);
			rtl8188e_set_FwPwrMode_cmd(hw, psmode);
		}
		break;
	case HW_VAR_H2C_FW_JOINBSSRPT:
		{
			u8 mstatus = (*(u8 *)val);
			rtl8188e_set_FwJoinBssReport_cmd(hw, mstatus);
		}
		break;
	case HW_VAR_INITIAL_GAIN:
		{
			struct rtw_dig *pDigTable = &podmpriv->DM_DigTable;
			u32 rx_gain = ((u32 *)(val))[0];

			if (rx_gain == 0xff) {/* restore rx gain */
				ODM_Write_DIG(podmpriv, pDigTable->BackupIGValue);
			} else {
				pDigTable->BackupIGValue = pDigTable->CurIGValue;
				ODM_Write_DIG(podmpriv, rx_gain);
			}
		}
		break;
	case HW_VAR_TRIGGER_GPIO_0:
		rtl8192cu_trigger_gpio_0(hw);
		break;
	case HW_VAR_RPT_TIMER_SETTING:
		{
			u16 min_rpt_time = (*(u16 *)val);
			ODM_RA_Set_TxRPT_Time(podmpriv, min_rpt_time);
		}
		break;
	case HW_VAR_ANTENNA_DIVERSITY_SELECT:
		{
			u8 Optimum_antenna = (*(u8 *)val);
			u8 Ant;
			/* switch antenna to Optimum_antenna */
			if (haldata->CurAntenna !=  Optimum_antenna) {
				Ant = (Optimum_antenna == 2) ? MAIN_ANT : AUX_ANT;
				rtl88eu_dm_update_rx_idle_ant(&haldata->odmpriv, Ant);

				haldata->CurAntenna = Optimum_antenna;
			}
		}
		break;
	case HW_VAR_EFUSE_BYTES: /*  To set EFUE total used bytes, added by Roger, 2008.12.22. */
		haldata->EfuseUsedBytes = *((u16 *)val);
		break;
	case HW_VAR_FIFO_CLEARN_UP:
		{
			struct pwrctrl_priv *pwrpriv = &Adapter->pwrctrlpriv;
			u8 trycnt = 100;

			/* pause tx */
			rtl_write_byte(rtlpriv, REG_TXPAUSE, 0xff);

			/* keep sn */
			Adapter->xmitpriv.nqos_ssn = rtl_read_word(rtlpriv, REG_NQOS_SEQ);

			if (!pwrpriv->bkeepfwalive) {
				/* RX DMA stop */
				rtl_write_dword(rtlpriv, REG_RXPKT_NUM, (rtl_read_dword(rtlpriv, REG_RXPKT_NUM)|RW_RELEASE_EN));
				do {
					if (!(rtl_read_dword(rtlpriv, REG_RXPKT_NUM)&RXDMA_IDLE))
						break;
				} while (trycnt--);
				if (trycnt == 0)
					DBG_88E("Stop RX DMA failed......\n");

				/* RQPN Load 0 */
				rtl_write_word(rtlpriv, REG_RQPN_NPQ, 0x0);
				rtl_write_dword(rtlpriv, REG_RQPN, 0x80000000);
				mdelay(10);
			}
		}
		break;
	case HW_VAR_CHECK_TXBUF:
		break;
	case HW_VAR_APFM_ON_MAC:
		haldata->bMacPwrCtrlOn = *val;
		DBG_88E("%s: bMacPwrCtrlOn=%d\n", __func__, haldata->bMacPwrCtrlOn);
		break;
	case HW_VAR_TX_RPT_MAX_MACID:
		{
			u8 maxMacid = *val;
			DBG_88E("### MacID(%d),Set Max Tx RPT MID(%d)\n", maxMacid, maxMacid+1);
			rtl_write_byte(rtlpriv, REG_TX_RPT_CTRL+1, maxMacid+1);
		}
		break;
	case HW_VAR_H2C_MEDIA_STATUS_RPT:
		rtl8188e_set_FwMediaStatus_cmd(hw, (*(__le16 *)val));
		break;
	case HW_VAR_BCN_VALID:
		/* BCN_VALID, BIT(16) of REG_TDECTRL = BIT(0) of REG_TDECTRL+2, write 1 to clear, Clear by sw */
		rtl_write_byte(rtlpriv, REG_TDECTRL+2, rtl_read_byte(rtlpriv, REG_TDECTRL+2) | BIT(0));
		break;
	default:
		break;
	}
}

static void GetHwReg8188EU(struct ieee80211_hw *hw, u8 variable, u8 *val)
{

	switch (variable) {
	case HW_VAR_BASIC_RATE:
		*((u16 *)(val)) = haldata->BasicRateSet;
	case HW_VAR_TXPAUSE:
		val[0] = rtl_read_byte(rtlpriv, REG_TXPAUSE);
		break;
	case HW_VAR_BCN_VALID:
		/* BCN_VALID, BIT(16) of REG_TDECTRL = BIT(0) of REG_TDECTRL+2 */
		val[0] = (BIT(0) & rtl_read_byte(rtlpriv, REG_TDECTRL+2)) ? true : false;
		break;
	case HW_VAR_DM_FLAG:
		val[0] = podmpriv->SupportAbility;
		break;
	case HW_VAR_RF_TYPE:
		val[0] = rtlphy->rf_type;
		break;
	case HW_VAR_FWLPS_RF_ON:
		{
			/* When we halt NIC, we should check if FW LPS is leave. */
			if (ppsc->rfpwr_state == ERFOFF) {
				/*  If it is in HW/SW Radio OFF or IPS state, we do not check Fw LPS Leave, */
				/*  because Fw is unload. */
				val[0] = true;
			} else {
				u32 valRCR;
				valRCR = rtl_read_dword(rtlpriv, REG_RCR);
				valRCR &= 0x00070000;
				if (valRCR)
					val[0] = false;
				else
					val[0] = true;
			}
		}
		break;
	case HW_VAR_CURRENT_ANTENNA:
		val[0] = haldata->CurAntenna;
		break;
	case HW_VAR_EFUSE_BYTES: /*  To get EFUE total used bytes, added by Roger, 2008.12.22. */
		*((u16 *)(val)) = haldata->EfuseUsedBytes;
		break;
	case HW_VAR_APFM_ON_MAC:
		*val = haldata->bMacPwrCtrlOn;
		break;
	case HW_VAR_CHK_HI_QUEUE_EMPTY:
		*val = ((rtl_read_dword(rtlpriv, REG_HGQ_INFORMATION)&0x0000ff00) == 0) ? true : false;
		break;
	default:
		break;
	}

}

/*  */
/*	Description: */
/*		Query setting of specified variable. */
/*  */
static u8
GetHalDefVar8188EUsb(
		struct ieee80211_hw *hw,
		enum hal_def_variable eVariable,
		void *pValue
	)
{
	u8 bResult = true;

	switch (eVariable) {
	case HAL_DEF_UNDERCORATEDSMOOTHEDPWDB:
		{
			struct mlme_priv *pmlmepriv = &Adapter->mlmepriv;
			struct sta_priv *pstapriv = &Adapter->stapriv;
			struct sta_info *psta;
			psta = rtw_get_stainfo(pstapriv, pmlmepriv->cur_network.network.MacAddress);
			if (psta)
				*((int *)pValue) = psta->rssi_stat.UndecoratedSmoothedPWDB;
		}
		break;
	case HAL_DEF_IS_SUPPORT_ANT_DIV:
		*((u8 *)pValue) = (rtlefuse->antenna_div_cfg == 0) ? false : true;
		break;
	case HAL_DEF_CURRENT_ANTENNA:
		*((u8 *)pValue) = haldata->CurAntenna;
		break;
	case HAL_DEF_DRVINFO_SZ:
		*((u32 *)pValue) = DRVINFO_SZ;
		break;
	case HAL_DEF_MAX_RECVBUF_SZ:
		*((u32 *)pValue) = MAX_RECVBUF_SZ;
		break;
	case HAL_DEF_RX_PACKET_OFFSET:
		*((u32 *)pValue) = RXDESC_SIZE + DRVINFO_SZ;
		break;
	case HAL_DEF_DBG_DM_FUNC:
		*((u32 *)pValue) = haldata->odmpriv.SupportAbility;
		break;
	case HAL_DEF_RA_DECISION_RATE:
		{
			u8 MacID = *((u8 *)pValue);
			*((u8 *)pValue) = ODM_RA_GetDecisionRate_8188E(&(haldata->odmpriv), MacID);
		}
		break;
	case HAL_DEF_RA_SGI:
		{
			u8 MacID = *((u8 *)pValue);
			*((u8 *)pValue) = ODM_RA_GetShortGI_8188E(&(haldata->odmpriv), MacID);
		}
		break;
	case HAL_DEF_PT_PWR_STATUS:
		{
			u8 MacID = *((u8 *)pValue);
			*((u8 *)pValue) = ODM_RA_GetHwPwrStatus_8188E(&(haldata->odmpriv), MacID);
		}
		break;
	case HW_VAR_MAX_RX_AMPDU_FACTOR:
		*((u32 *)pValue) = MAX_AMPDU_FACTOR_64K;
		break;
	case HW_DEF_RA_INFO_DUMP:
		{
			u8 entry_id = *((u8 *)pValue);
			if (check_fwstate(&Adapter->mlmepriv, _FW_LINKED)) {
				DBG_88E("============ RA status check ===================\n");
				DBG_88E("Mac_id:%d , RateID = %d, RAUseRate = 0x%08x, RateSGI = %d, DecisionRate = 0x%02x ,PTStage = %d\n",
					entry_id,
					haldata->odmpriv.RAInfo[entry_id].RateID,
					haldata->odmpriv.RAInfo[entry_id].RAUseRate,
					haldata->odmpriv.RAInfo[entry_id].RateSGI,
					haldata->odmpriv.RAInfo[entry_id].DecisionRate,
					haldata->odmpriv.RAInfo[entry_id].PTStage);
			}
		}
		break;
	case HW_DEF_ODM_DBG_FLAG:
		{
			struct odm_dm_struct *dm_ocm = &(haldata->odmpriv);
			pr_info("dm_ocm->DebugComponents = 0x%llx\n", dm_ocm->DebugComponents);
		}
		break;
	case HAL_DEF_DBG_DUMP_RXPKT:
		*((u8 *)pValue) = haldata->bDumpRxPkt;
		break;
	case HAL_DEF_DBG_DUMP_TXPKT:
		*((u8 *)pValue) = haldata->bDumpTxPkt;
		break;
	default:
		bResult = false;
		break;
	}

	return bResult;
}

/*  */
/*	Description: */
/*		Change default setting of specified variable. */
/*  */
/* TODO */
#if 0
static u8 SetHalDefVar8188EUsb(struct ieee80211_hw *hw, enum hal_def_variable eVariable, void *pValue)
{
	struct hal_data_8188e	*haldata = GET_HAL_DATA(Adapter);
	u8 bResult = true;

	switch (eVariable) {
	case HAL_DEF_DBG_DM_FUNC:
		{
			u8 dm_func = *((u8 *)pValue);
			struct odm_dm_struct *podmpriv = &haldata->odmpriv;

			if (dm_func == 0) { /* disable all dynamic func */
				podmpriv->SupportAbility = DYNAMIC_FUNC_DISABLE;
				DBG_88E("==> Disable all dynamic function...\n");
			} else if (dm_func == 1) {/* disable DIG */
				podmpriv->SupportAbility  &= (~DYNAMIC_BB_DIG);
				DBG_88E("==> Disable DIG...\n");
			} else if (dm_func == 2) {/* disable High power */
				podmpriv->SupportAbility  &= (~DYNAMIC_BB_DYNAMIC_TXPWR);
			} else if (dm_func == 3) {/* disable tx power tracking */
				podmpriv->SupportAbility  &= (~DYNAMIC_RF_CALIBRATION);
				DBG_88E("==> Disable tx power tracking...\n");
			} else if (dm_func == 5) {/* disable antenna diversity */
				podmpriv->SupportAbility  &= (~DYNAMIC_BB_ANT_DIV);
			} else if (dm_func == 6) {/* turn on all dynamic func */
				if (!(podmpriv->SupportAbility  & DYNAMIC_BB_DIG)) {
					struct rtw_dig *pDigTable = &podmpriv->DM_DigTable;
					pDigTable->CurIGValue = rtl_read_byte(rtlpriv, 0xc50);
				}
				podmpriv->SupportAbility = DYNAMIC_ALL_FUNC_ENABLE;
				DBG_88E("==> Turn on all dynamic function...\n");
			}
		}
		break;
	case HAL_DEF_DBG_DUMP_RXPKT:
		haldata->bDumpRxPkt = *((u8 *)pValue);
		break;
	case HAL_DEF_DBG_DUMP_TXPKT:
		haldata->bDumpTxPkt = *((u8 *)pValue);
		break;
	case HW_DEF_FA_CNT_DUMP:
		{
			u8 bRSSIDump = *((u8 *)pValue);
			struct odm_dm_struct *dm_ocm = &(haldata->odmpriv);
			if (bRSSIDump)
				dm_ocm->DebugComponents	=	ODM_COMP_DIG|ODM_COMP_FA_CNT;
			else
				dm_ocm->DebugComponents	= 0;
		}
		break;
	case HW_DEF_ODM_DBG_FLAG:
		{
			u64	DebugComponents = *((u64 *)pValue);
			struct odm_dm_struct *dm_ocm = &(haldata->odmpriv);
			dm_ocm->DebugComponents = DebugComponents;
		}
		break;
	default:
		bResult = false;
		break;
	}

	return bResult;
}

static void UpdateHalRAMask8188EUsb(struct ieee80211_hw *hw, u32 mac_id, u8 rssi_level)
{
	u8 init_rate = 0;
	u8 networkType, raid;
	u32 mask, rate_bitmap;
	u8 shortGIrate = false;
	int	supportRateNum = 0;

	if (mac_id >= NUM_STA) /* CAM_SIZE */
		return;
	psta = pmlmeinfo->FW_sta_info[mac_id].psta;
	if (psta == NULL)
		return;
	switch (mac_id) {
	case 0:/*  for infra mode */
		supportRateNum = rtw_get_rateset_len(cur_network->SupportedRates);
		networkType = judge_network_type(adapt, cur_network->SupportedRates, supportRateNum) & 0xf;
		raid = networktype_to_raid(networkType);
		mask = update_supported_rate(cur_network->SupportedRates, supportRateNum);
		mask |= (pmlmeinfo->HT_enable) ? update_MSC_rate(&(pmlmeinfo->HT_caps)) : 0;
		if (support_short_GI(adapt, &(pmlmeinfo->HT_caps)))
			shortGIrate = true;
		break;
	case 1:/* for broadcast/multicast */
		supportRateNum = rtw_get_rateset_len(pmlmeinfo->FW_sta_info[mac_id].SupportedRates);
		if (pmlmeext->cur_wireless_mode & WIRELESS_11B)
			networkType = WIRELESS_11B;
		else
			networkType = WIRELESS_11G;
		raid = networktype_to_raid(networkType);
		mask = update_basic_rate(cur_network->SupportedRates, supportRateNum);
		break;
	default: /* for each sta in IBSS */
		supportRateNum = rtw_get_rateset_len(pmlmeinfo->FW_sta_info[mac_id].SupportedRates);
		networkType = judge_network_type(adapt, pmlmeinfo->FW_sta_info[mac_id].SupportedRates, supportRateNum) & 0xf;
		raid = networktype_to_raid(networkType);
		mask = update_supported_rate(cur_network->SupportedRates, supportRateNum);

		/* todo: support HT in IBSS */
		break;
	}

	rate_bitmap = 0x0fffffff;
	rate_bitmap = ODM_Get_Rate_Bitmap(&haldata->odmpriv, mac_id, mask, rssi_level);
	DBG_88E("%s => mac_id:%d, networkType:0x%02x, mask:0x%08x\n\t ==> rssi_level:%d, rate_bitmap:0x%08x\n",
		__func__, mac_id, networkType, mask, rssi_level, rate_bitmap);

	mask &= rate_bitmap;

	init_rate = get_highest_rate_idx(mask)&0x3f;

	if (haldata->fw_ractrl) {
		u8 arg;

		arg = mac_id & 0x1f;/* MACID */
		arg |= BIT(7);
		if (shortGIrate)
			arg |= BIT(5);
		mask |= ((raid << 28) & 0xf0000000);
		DBG_88E("update raid entry, mask=0x%x, arg=0x%x\n", mask, arg);
		psta->ra_mask = mask;
		mask |= ((raid << 28) & 0xf0000000);

		/* to do ,for 8188E-SMIC */
		rtl8188e_set_raid_cmd(adapt, mask);
	} else {
		ODM_RA_UpdateRateInfo_8188E(&(haldata->odmpriv),
				mac_id,
				raid,
				mask,
				shortGIrate
				);
	}
	/* set ra_id */
	psta->raid = raid;
	psta->init_rate = init_rate;
}
#endif

static void SetBeaconRelatedRegisters8188EUsb(struct ieee80211_hw *hw)
{
	u32 value32;
	u32 bcn_ctrl_reg			= REG_BCN_CTRL;
	/* reset TSF, enable update TSF, correcting TSF On Beacon */

	/* BCN interval */
	rtl_write_word(rtlpriv, REG_BCN_INTERVAL, mac->beacon_interval);
	rtl_write_byte(rtlpriv, REG_ATIMWND, 0x02);/*  2ms */

	_InitBeaconParameters(hw);

	rtl_write_byte(rtlpriv, REG_SLOT, 0x09);

	value32 = rtl_read_dword(rtlpriv, REG_TCR);
	value32 &= ~TSFRST;
	rtl_write_dword(rtlpriv,  REG_TCR, value32);

	value32 |= TSFRST;
	rtl_write_dword(rtlpriv, REG_TCR, value32);

	/*  NOTE: Fix test chip's bug (about contention windows's randomness) */
	rtl_write_byte(rtlpriv,  REG_RXTSF_OFFSET_CCK, 0x50);
	rtl_write_byte(rtlpriv, REG_RXTSF_OFFSET_OFDM, 0x50);

	_BeaconFunctionEnable(hw, true, true);

	ResumeTxBeacon(hw);

	rtl_write_byte(rtlpriv, bcn_ctrl_reg, rtl_read_byte(rtlpriv, bcn_ctrl_reg)|BIT(1));
}

static void rtl8188eu_init_default_value(struct ieee80211_hw *hw)
{
	u8 i;

	/* init default value */
	/* TODO */
#if 0
	haldata->fw_ractrl = false;
	if (!pwrctrlpriv->bkeepfwalive)
		rtlhal->last_hmeboxnum = 0;
#endif

	/* init dm default value */
	rtlphy->iqk_initialized = false;
	/* TODO */
#if 0
	haldata->odmpriv.RFCalibrateInfo.TM_Trigger = 0;/* for IQK */
#endif
	rtlphy->pwrgroup_cnt = 0;
	/* TODO */
#if 0
	haldata->PGMaxGroup = 13;
	haldata->odmpriv.RFCalibrateInfo.ThermalValue_HP_index = 0;
	for (i = 0; i < HP_THERMAL_NUM; i++)
		haldata->odmpriv.RFCalibrateInfo.ThermalValue_HP[i] = 0;
#endif
}

