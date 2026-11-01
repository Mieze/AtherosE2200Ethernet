//
//  AtherosE2200Hardware.cpp
//  AtherosE2200EthernetV2
//
//  Created by Laura Müller on 25.10.26.
//  Copyright © 2026 Laura Müller. All rights reserved.
//

#include "AtherosE2200Ethernet.h"

#pragma mark --- hardware initialization methods ---

bool AtherosE2200::initPCIConfigSpace(IOPCIDevice *provider)
{
    UInt32 pcieLinkCap;
    UInt16 pcieLinkCtl;
    UInt16 cmdReg;
    UInt16 pmCap;
    UInt8 pmCapOffset;
    bool result = false;
    
    /* Get vendor and device info. */
    pciDeviceData.vendor = provider->extendedConfigRead16(kIOPCIConfigVendorID);
    pciDeviceData.device = provider->extendedConfigRead16(kIOPCIConfigDeviceID);
    pciDeviceData.subsystem_vendor = provider->extendedConfigRead16(kIOPCIConfigSubSystemVendorID);
    pciDeviceData.subsystem_device = provider->extendedConfigRead16(kIOPCIConfigSubSystemID);
    pciDeviceData.revision = provider->extendedConfigRead8(kIOPCIConfigRevisionID);

    /* Identify the chipset. */
    if (!alxIdentifyChip())
        goto done;
    
    /* Setup power management. */
    if (provider->findPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
        pmCap = provider->extendedConfigRead16(pmCapOffset + kIOPCIPMCapability);
        DebugLog("PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & (kPCIPMCPMESupportFromD3Cold | kPCIPMCPMESupportFromD3Hot)) {
            wolCapable = true;
            DebugLog("PME# from D3 (cold/hot) supported.\n");
        }
        pciPMCtrlOffset = pmCapOffset + kIOPCIPMControl;
        
        /* Make sure the device is in D0 power state. */
        provider->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
        setPowerStateWakeAction(this, NULL, NULL, NULL, NULL);
    } else {
        IOLog("PCI power management unsupported.\n");
    }
    
    /* Get PCIe link information. */
    if (provider->findPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
        pcieLinkCap = provider->extendedConfigRead32(pcieCapOffset + kIOPCIELinkCapability);
        pcieLinkCtl = provider->extendedConfigRead16(pcieCapOffset + kIOPCIELinkControl);
        DebugLog("PCIe device capabilities: 0x%08x.\n", provider->extendedConfigRead32(pcieCapOffset + kIOPCIEDevCapability));
        DebugLog("PCIe link capabilities: 0x%08x, link control: 0x%04x.\n", pcieLinkCap, pcieLinkCtl);
        
#ifdef DEBUG
        if (pcieLinkCtl & kIOPCIELinkCtlASPM)
            IOLog("PCIe ASPM enabled.\n");
#endif  /* DEBUG */
        
    }
    /* Enable the device. */
    cmdReg    = provider->extendedConfigRead16(kIOPCIConfigCommand);
    cmdReg    |= kALXPCICommand;
    provider->extendedConfigWrite16(kIOPCIConfigCommand, cmdReg);
    
    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("region #0 not an MMIO resource, aborting.\n");
        goto done;
    }
    baseAddr = reinterpret_cast<volatile void *>(baseMap->getVirtualAddress());
    hw.hw_addr = (u8 __iomem *)baseAddr;
    result = true;
    
done:
    return result;
}

bool AtherosE2200::alxResetPCIe()
{
    UInt8 rev = alx_hw_revision(&hw);
    UInt32 val;
    UInt16 val16;
    bool result = false;

    /* Workaround for PCI problem when BIOS sets MMRBC incorrectly. */
    val16 = pciDevice->extendedConfigRead16(kIOPCIConfigCommand);
    
    if (!(val16 & kALXPCICommand) || (val16 & kIOPCICommandInterruptDisable)) {
        val16 = ((val16 | kALXPCICommand) & ~kIOPCICommandInterruptDisable);
        pciDevice->extendedConfigWrite16(kIOPCIConfigCommand, val16);
        DebugLog("Restored PCI command register.\n");
    }
    /* Check if the NIC has been disabled by the BIOS. */
    val = alxReadMem32(ALX_DRV);
    
    if (val & ALX_DRV_DISABLE) {
        IOLog("NIC disabled by BIOS, aborting.\n");
        goto done;
    }
    /* clear WoL setting/status */
    val = alxReadMem32(ALX_WOL0);
    alxWriteMem32(ALX_WOL0, 0);
    
    val = alxReadMem32(ALX_PDLL_TRNS1);
    alxWriteMem32(ALX_PDLL_TRNS1, val & ~ALX_PDLL_TRNS1_D3PLLOFF_EN);
    
    /* mask some pcie error bits */
    val = alxReadMem32(ALX_UE_SVRT);
    val &= ~(ALX_UE_SVRT_DLPROTERR | ALX_UE_SVRT_FCPROTERR);
    alxWriteMem32(ALX_UE_SVRT, val);
    
    /* wol 25M & pclk */
    val = alxReadMem32(ALX_MASTER);
    if (alx_is_rev_a(rev) && alx_hw_with_cr(&hw)) {
        if ((val & ALX_MASTER_WAKEN_25M) == 0 ||
            (val & ALX_MASTER_PCLKSEL_SRDS) == 0)
            alxWriteMem32(ALX_MASTER, val | ALX_MASTER_PCLKSEL_SRDS | ALX_MASTER_WAKEN_25M);
    } else {
        if ((val & ALX_MASTER_WAKEN_25M) == 0 ||
            (val & ALX_MASTER_PCLKSEL_SRDS) != 0)
            alxWriteMem32(ALX_MASTER, (val & ~ALX_MASTER_PCLKSEL_SRDS) | ALX_MASTER_WAKEN_25M);
    }
    /* ASPM setting */
    alx_enable_aspm(&hw, true, true);
    pciDevice->setASPMState(this, kIOPCIELinkCtlL0s | kIOPCIELinkCtlL1);
    
    result = true;
    IODelay(10);
    
done:
    return result;
}

IOReturn AtherosE2200::setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    AtherosE2200 *ethCtlr = OSDynamicCast(AtherosE2200, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        val16 |= kPCIPMCSPowerStateD0;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

IOReturn AtherosE2200::setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    AtherosE2200 *ethCtlr = OSDynamicCast(AtherosE2200, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;

    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;

        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        
        if (ethCtlr->hw.sleep_ctrl & ALX_SLEEP_ACTIVE)
            val16 |= (kPCIPMCSPMEStatus | kPCIPMCSPMEEnable | kPCIPMCSPowerStateD3);
        else
            val16 |= kPCIPMCSPowerStateD3;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

bool AtherosE2200::alxLoadDefaultAddress()
{
    UInt32 val;
    bool result = false;
    
    DebugLog("alxLoadDefaultAddress() ===>\n");
    
    /* try to get it from register first *//*
    if (getHardwareAddress(&origMacAddr) == kIOReturnSuccess) {
        DebugLog("Got MAC address from register.\n");
        result = true;
        goto done;
    }*/
    /* try to load from efuse */
    if (!alx_wait_reg(&hw, ALX_SLD, ALX_SLD_STAT | ALX_SLD_START, &val))
        goto done;
    
    alxWriteMem32(ALX_SLD, val | ALX_SLD_START);
    
    if (!alx_wait_reg(&hw, ALX_SLD, ALX_SLD_START, NULL))
        goto done;

    if (getHardwareAddress(&origMacAddr) == kIOReturnSuccess) {
        DebugLog("Got MAC address from efuse.\n");
        result = true;
        goto done;
    }
    /* try to load from flash/eeprom (if present) */
    val = alxReadMem32(ALX_EFLD);
    
    if (val & (ALX_EFLD_F_EXIST | ALX_EFLD_E_EXIST)) {
        if (!alx_wait_reg(&hw, ALX_EFLD, ALX_EFLD_STAT | ALX_EFLD_START, &val))
            goto done;
        
        alxWriteMem32(ALX_EFLD, val | ALX_EFLD_START);
        
        if (!alx_wait_reg(&hw, ALX_EFLD, ALX_EFLD_START, NULL))
            goto done;
        
        if (getHardwareAddress(&origMacAddr) == kIOReturnSuccess) {
            DebugLog("Got MAC address from EEPROM.\n");
            result = true;
        }
    }
    
done:
    if (result)
        memcpy(&currMacAddr.bytes[0], &origMacAddr.bytes[0], kIOEthernetAddressSize);

    DebugLog("alxLoadDefaultAddress() <===\n");
    
    return result;
}

IOReturn AtherosE2200::alxSetHardwareAddress(const IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnSuccess;
    UInt32 mac0, mac1;
    
    mac0 = OSSwapBigToHostInt32(*((UInt32 *)&addr->bytes[2]));
    alxWriteMem32(ALX_STAD0, mac0);
    mac1 = OSSwapBigToHostInt16(*((UInt16 *)&addr->bytes[0]));
    alxWriteMem32(ALX_STAD1, mac1);
    
    if ((alxReadMem32(ALX_STAD0) != mac0) || (alxReadMem32(ALX_STAD1) != mac1)) {
        alxLoadDefaultAddress();
        IOLog("Failed to set MAC address. Permanent address restored.\n");
        result = kIOReturnError;
    } else {
        memcpy(&currMacAddr.bytes[0], &addr->bytes[0], kIOEthernetAddressSize);
    }
    return result;
}

void AtherosE2200::alxEnable()
{
    const IONetworkMedium *selectedMedium = getSelectedMedium();
    UInt32 msiControl = ((hw.imt >> 1) << ALX_MSI_RETRANS_TM_SHIFT);
    
    if (!selectedMedium) {
        DebugLog("No medium selected. Falling back to autonegotiation.\n");
        selectedMedium = mediumTable[MEDIUM_INDEX_AUTO];
        setCurrentMedium(selectedMedium);
    }
    setLinkStatus(kIONetworkLinkValid);

    polling = false;

    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;

    alxSpeedDuplexForMedium(selectedMedium);
    alxSetupSpeedDuplex(hw.adv_cfg, eeeAdv, hw.flowctrl);

    alxResetPCIe();
    alxResetPhy();
    alx_reset_mac(&hw);
    alxConfigure();
    
    /* We are always using the the MSI interrupt. */
    alxWriteMem32(ALX_MSI_RETRANS_TIMER, msiControl | ALX_MSI_MASK_SEL_LINE);
        
    /* Configure vector mapping. */
    alxWriteMem32(ALX_MSI_MAP_TBL1, 0);
    alxWriteMem32(ALX_MSI_MAP_TBL2, 0);
    alxWriteMem32(ALX_MSI_ID_MAP, 0);

    alx_enable_aspm(&hw, false, true);
    pciDevice->setASPMState(this, kIOPCIELinkCtlL1);

    /* clear old interrupts */
    alxWriteMem32(ALX_ISR, ~(UInt32)ALX_ISR_DIS);

    /* Enable all known interrupts by setting the interrupt mask. */
    alxEnableIRQ();
}

int AtherosE2200::alxDisable()
{
    int error;
    int speed = 0;
    UInt8 duplex = 0;

    alxDisableIRQ();
    
    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;
    
    polling = false;

    alx_reset_mac(&hw);
    
    /* disable l0s/l1 */
    alx_enable_aspm(&hw, false, false);
    pciDevice->setASPMState(this, 0);

    if (hw.sleep_ctrl & ALX_SLEEP_ACTIVE) {
        error = alxSelectPowersavingSpeed(&speed, &duplex);
        
        if (error) {
            DebugLog("alx_select_powersaving_speed() failed.\n");
            goto done;
        }
        error = alx_clear_phy_intr(&hw);
        
        if (error) {
            DebugLog("alx_clear_phy_intr() failed.\n");
            goto done;
        }
        error = alx_pre_suspend(&hw, speed, duplex);
        
        if (error) {
            DebugLog("alx_pre_suspend() failed.\n");
            goto done;
        }
        error = alx_config_wol(&hw);
        
        if (error) {
            DebugLog("alx_config_wol() failed.\n");
            goto done;
        }
    }
    error = 0;
    
done:
    if (linkUp) {
        linkUp = false;
        setLinkStatus(kIONetworkLinkValid);
        IOLog("Link down on en%u\n", netif->getUnitNumber());
    }
    return error;
}

/* Reset the NIC in case a tx deadlock or a pci error occurred. timerSource and txQueue
 * are stopped immediately but will be restarted by checkLinkStatus() when the link has
 * been reestablished.
 */

void AtherosE2200::alxRestart()
{
    
    /* Stop output thread and flush txQueue */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    /*  Also set the link status to down. */
    linkUp = false;
    setLinkStatus(kIONetworkLinkValid);

    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;

    /* Reset NIC and cleanup both descriptor rings. */
    alxDisableIRQ();
    alx_reset_mac(&hw);
    intrMask = (ALX_ISR_MISC | ALX_ISR_PHY);

    /* disable l0s/l1 */
    alx_enable_aspm(&hw, false, false);
    pciDevice->setASPMState(this, 0);

    clearDescriptors();
    deadlockWarn = 0;
    
    /* Reinitialize NIC. */
    alxEnable();
}

void AtherosE2200::alxConfigure()
{
    alxInitDescRings();
    alxConfigureBasic();
    
#ifdef CONFIG_RSS

    alxConfigureRSS(false);

#else
    alx_disable_rss(&hw);

#endif  /* CONFIG_RSS */
    
    setMulticastMode(multicastMode);
    
    /*
     * Make sure that the state of promiscous mode is still
     * valid after wakeup from sleep.
     */
    if (promiscusMode)
        setPromiscuousMode(true);

    alxWriteMem32(ALX_MAC_CTRL, hw.rx_ctrl);
}

void AtherosE2200::alxConfigureBasic()
{
    UInt32 val, rawMTU, maxPayload;
    UInt16 val16;
    u8 chipRev = alx_hw_revision(&hw);
        
    alxWriteMem32(ALX_CLK_GATE, ALX_CLK_GATE_ALL);
    
    /* idle timeout to switch clk_125M */
    if (chipRev >= ALX_REV_B0)
        alxWriteMem32(ALX_IDLE_DECISN_TIMER, ALX_IDLE_DECISN_TIMER_DEF);
    
    alxWriteMem32(ALX_SMB_TIMER, hw.smb_timer * 500UL);
    
    val = alxReadMem32(ALX_MASTER);
    val |= ALX_MASTER_IRQMOD2_EN | ALX_MASTER_IRQMOD1_EN | ALX_MASTER_SYSALVTIMER_EN;
    alxWriteMem32(ALX_MASTER, val);
    alxWriteMem32(ALX_IRQ_MODU_TIMER, (hw.imt >> 1) << ALX_IRQ_MODU_TIMER1_SHIFT);
    /* intr re-trig timeout */
    alxWriteMem32(ALX_INT_RETRIG, ALX_INT_RETRIG_TO);
    /* tpd threshold to trig int */
    alxWriteMem32(ALX_TINT_TPD_THRSHLD, hw.ith_tpd);
    alxWriteMem32(ALX_TINT_TIMER, hw.imt);
    
    rawMTU = ALX_RAW_MTU(hw.mtu);
    alxWriteMem32(ALX_MTU, rawMTU);
    
    if (rawMTU > (ALX_MTU_JUMBO_TH + ETH_FCS_LEN + VLAN_HLEN))
        hw.rx_ctrl &= ~ALX_MAC_CTRL_FAST_PAUSE;
    else
        hw.rx_ctrl |= ALX_MAC_CTRL_FAST_PAUSE;

    if (rawMTU < ALX_TXQ1_JUMBO_TSO_TH)
        val = (rawMTU + 7) >> 3;
    else
        val = ALX_TXQ1_JUMBO_TSO_TH >> 3;
    
    alxWriteMem32(ALX_TXQ1, val | ALX_TXQ1_ERRLGPKT_DROP_EN);

    val16 = pciDevice->extendedConfigRead16(pcieCapOffset + kIOPCIEDeviceControl);
    maxPayload = ((val16 & kIOPCIEDevCtlReadQ) >> 12);
    /*
     * if BIOS had changed the default dma read max length,
     * restore it to default value
     */
    if (maxPayload < ALX_DEV_CTRL_MAXRRS_MIN) {
        val16 &= ~kIOPCIEDevCtlReadQ;
        val16 |= (ALX_DEV_CTRL_MAXRRS_MIN << 12);
        pciDevice->extendedConfigWrite16(pcieCapOffset + kIOPCIEDeviceControl, val16);
        DebugLog("Restore dma read max length: 0x%x.\n", val16);
    }
    val = ALX_TXQ_TPD_BURSTPREF_DEF << ALX_TXQ0_TPD_BURSTPREF_SHIFT | ALX_TXQ0_MODE_ENHANCE | ALX_TXQ0_LSO_8023_EN |    ALX_TXQ0_SUPT_IPOPT | ALX_TXQ_TXF_BURST_PREF_DEF << ALX_TXQ0_TXF_BURST_PREF_SHIFT;
    alxWriteMem32(ALX_TXQ0, val);
    val = ALX_TXQ_TPD_BURSTPREF_DEF << ALX_HQTPD_Q1_NUMPREF_SHIFT | ALX_TXQ_TPD_BURSTPREF_DEF << ALX_HQTPD_Q2_NUMPREF_SHIFT | ALX_TXQ_TPD_BURSTPREF_DEF << ALX_HQTPD_Q3_NUMPREF_SHIFT | ALX_HQTPD_BURST_EN;
    alxWriteMem32(ALX_HQTPD, val);
    
    /* rxq, flow control */
    val = alxReadMem32(ALX_SRAM5);
    val = ALX_GET_FIELD(val, ALX_SRAM_RXF_LEN) << 3;
    
    if (val > ALX_SRAM_RXF_LEN_8K) {
        val16 = ALX_MTU_STD_ALGN >> 3;
        val = (val - ALX_RXQ2_RXF_FLOW_CTRL_RSVD) >> 3;
    } else {
        val16 = ALX_MTU_STD_ALGN >> 3;
        val = (val - ALX_MTU_STD_ALGN) >> 3;
    }
    alxWriteMem32(ALX_RXQ2, val16 << ALX_RXQ2_RXF_XOFF_THRESH_SHIFT | val << ALX_RXQ2_RXF_XON_THRESH_SHIFT);
    val = ALX_RXQ0_NUM_RFD_PREF_DEF << ALX_RXQ0_NUM_RFD_PREF_SHIFT | ALX_RXQ0_RSS_MODE_DIS << ALX_RXQ0_RSS_MODE_SHIFT |ALX_RXQ0_IDT_TBL_SIZE_DEF << ALX_RXQ0_IDT_TBL_SIZE_SHIFT | ALX_RXQ0_RSS_HSTYP_ALL | ALX_RXQ0_RSS_HASH_EN |    ALX_RXQ0_IPV6_PARSE_EN;
    
    if (alx_hw_giga(&hw))
        ALX_SET_FIELD(val, ALX_RXQ0_ASPM_THRESH, ALX_RXQ0_ASPM_THRESH_100M);
    
    alxWriteMem32(ALX_RXQ0, val);
    
    val = alxReadMem32(ALX_DMA);
    val = ALX_DMA_RORDER_MODE_OUT << ALX_DMA_RORDER_MODE_SHIFT | ALX_DMA_RREQ_PRI_DATA | maxPayload << ALX_DMA_RREQ_BLEN_SHIFT | ALX_DMA_WDLY_CNT_DEF << ALX_DMA_WDLY_CNT_SHIFT | ALX_DMA_RDLY_CNT_DEF << ALX_DMA_RDLY_CNT_SHIFT | (hw.dma_chnl - 1) << ALX_DMA_RCHNL_SEL_SHIFT;
    alxWriteMem32(ALX_DMA, val);
    
    /* default multi-tx-q weights */
    val = ALX_WRR_PRI_RESTRICT_NONE << ALX_WRR_PRI_SHIFT | 4 << ALX_WRR_PRI0_SHIFT | 4 << ALX_WRR_PRI1_SHIFT | 4 << ALX_WRR_PRI2_SHIFT | 4 << ALX_WRR_PRI3_SHIFT;
    alxWriteMem32(ALX_WRR, val);
}

#ifdef CONFIG_RSS

static const UInt8 rssKey[40] = {
    0xE2, 0x91, 0xD7, 0x3D, 0x18, 0x05, 0xEC, 0x6C,
    0x2A, 0x94, 0xB3, 0x0D, 0xA5, 0x4F, 0x2B, 0xEC,
    0xEA, 0x49, 0xAF, 0x7C, 0xE2, 0x14, 0xAD, 0x3D,
    0xB8, 0x55, 0xAA, 0xBE, 0x6A, 0x3E, 0x67, 0xEA,
    0x14, 0x36, 0x4D, 0x17, 0x3B, 0xED, 0x20, 0x0D
};

void AtherosE2200::alxConfigureRSS(bool enable)
{
    UInt32 val = 0;
    UInt32 len = sizeof(rssKey);
    int i, j;

    /* Initialise RSS hash type and IDT table size. */
    //rssHashType = ALX_RSS_HSTYP_ALL_EN;
    rssIdtSize = ALX_RXQ0_IDT_TBL_SIZE_DEF;
    
    /* Fill out the redirection table. */
    memset(rssIdt, 0x0, sizeof(rssIdt));

    for (i = 0, j = 0; i < 256; i++, j++) {
        if (j == 1)
            j = 0;
        
        val |= (j << ((i & 7) * 4));
        
        if ((i & 7) == 7) {
            rssIdt[i >> 3] = val;
            val = 0;
        }
    }
    /* Fill out hash function keys. */
    for (i = 0; i < len; i++) {
        alxWriteMem8(ALX_RSS_KEY0 + i, rssKey[len - i - 1]);
    }
    len = sizeof(rssIdt) / sizeof(UInt32);
    
    /* Fill out redirection table. */
    for (i = 0; i < len; i++)
        alxWriteMem32(ALX_RSS_IDT_TBL0 + (i * sizeof(UInt32)), rssIdt[i]);
    
    alxWriteMem32(ALX_RSS_BASE_CPU_NUM, rssBaseCPU);
    
    val = alxReadMem32(ALX_RXQ0);
    
    if (enable)
        val |= ALX_RXQ0_RSS_HASH_EN;
    else
        val &= ~ALX_RXQ0_RSS_HASH_EN;
    
    alxWriteMem32(ALX_RXQ0, val);
}

#endif  /* CONFIG_RSS */

void AtherosE2200::alxInitDescRings()
{
    UInt32 addrHigh = (rxRetPhyAddr >> 32);
    UInt32 addrLow;
    
    txDirtyDescIndex = txNextDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    rxNextDescIndex = 0;
    rxCleanedCount = rxMapNextIndex = 0;

    addrLow = (UInt32)(rxRetPhyAddr & 0xffffffff);
    alxWriteMem32(ALX_RX_BASE_ADDR_HI, addrHigh);
    alxWriteMem32(ALX_RRD_ADDR_LO, addrLow);
    alxWriteMem32(ALX_RRD_RING_SZ, kNumRxDesc);
    
    addrLow = (UInt32)(rxFreePhyAddr & 0xffffffff);
    alxWriteMem32(ALX_RFD_ADDR_LO, addrLow);
    alxWriteMem32(ALX_RFD_RING_SZ, kNumRxDesc);
    alxWriteMem32(ALX_RFD_BUF_SZ, kRxBufferSize);

    addrHigh = (txPhyAddr >> 32);
    addrLow = (UInt32)(txPhyAddr & 0xffffffff);
    alxWriteMem32(ALX_TX_BASE_ADDR_HI, addrHigh);
    alxWriteMem32(ALX_TPD_PRI0_ADDR_LO, addrLow);
    alxWriteMem32(ALX_TPD_RING_SZ, kNumTxDesc);
    
    /* load these pointers into the chip */
    alxWriteMem32(ALX_SRAM9, ALX_SRAM_LOAD_PTR);
    
    alxWriteMem16(ALX_RFD_PIDX, kRxLastDesc);
}

inline void AtherosE2200::alxEnableIRQ()
{
    /* level-1 interrupt switch */
    alxWriteMem32(ALX_ISR, 0);
    alxWriteMem32(ALX_IMR, intrMask);
    alxPostWrite();
}

inline void AtherosE2200::alxDisableIRQ()
{
    alxWriteMem32(ALX_ISR, ALX_ISR_DIS);
    alxWriteMem32(ALX_IMR, 0);
    alxPostWrite();
}

bool AtherosE2200::alxIdentifyChip()
{
    int rev;
    bool result = false;
    
    switch (pciDeviceData.device) {
        case ALX_DEV_ID_AR8162:
            chip = kChipAR8162;
            gbCapable = false;
            eeeCap = ALX_LOCAL_EEEADV_100BT;
            DebugLog("Found AR8162.\n");
            break;
            
        case ALX_DEV_ID_AR8161:
            chip = kChipAR8161;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Found AR8161.\n");
            break;
            
        case ALX_DEV_ID_AR8172:
            chip = kChipAR8172;
            gbCapable = false;
            eeeCap = ALX_LOCAL_EEEADV_100BT;
            DebugLog("Found AR8172.\n");
            break;
            
        case ALX_DEV_ID_AR8171:
            chip = kChipAR8171;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Found AR8171.\n");
            break;
            
        case ALX_DEV_ID_E2200:
            chip = kChipKillerE2200;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Found Killer E2200.\n");
            break;
            
        case ALX_DEV_ID_E2400:
            chip = kChipKillerE2400;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Found Killer E2400.\n");
            break;
            
        case ALX_DEV_ID_E2500:
            chip = kChipKillerE2500;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Found Killer E2500.\n");
            break;
            
        default:
            IOLog("Unknown chip. Aborting.\n");
            goto done;
            break;
    }
    rev = alx_hw_revision(&hw);
    
    if (rev > ALX_REV_C0)
        goto done;
    
    hw.max_dma_chnl = (rev >= ALX_REV_B0) ? 4 : 2;
    result = true;
    
done:
    return result;
}

void AtherosE2200::alxGetChkSumCommand(UInt32 *cmd, mbuf_csum_request_flags_t checksums)
{
    if (checksums & kChecksumTCP)
        *cmd = (TPD_IPV4 | TPD_TCP_XSUM | TPD_IP_XSUM | kMinL4HdrOffsetV4);
    else if (checksums & kChecksumUDP)
        *cmd = (TPD_IPV4 | TPD_UDP_XSUM | TPD_IP_XSUM | kMinL4HdrOffsetV4);
    else if (checksums & kChecksumIP)
        *cmd = (TPD_IPV4 | TPD_IP_XSUM);
    else if (checksums & kChecksumTCPIPv6)
        *cmd = (TPD_TCP_XSUM | kMinL4HdrOffsetV6);
    else if (checksums & kChecksumUDPIPv6)
        *cmd = (TPD_UDP_XSUM | kMinL4HdrOffsetV6);
}

#pragma mark --- phy access methods ---

int AtherosE2200::alxReadPhyLink()
{
    int error = 0;
    UInt16 bmsr, giga, lpa;
    
    error = alx_read_phy_reg(&hw, MII_BMSR, &bmsr);
    
    if (error)
        goto done;
    
    error = alx_read_phy_reg(&hw, MII_BMSR, &bmsr);
    
    if (error)
        goto done;
    
    if (!(bmsr & BMSR_LSTATUS)) {
        hw.link_speed = SPEED_UNKNOWN;
        hw.duplex = DUPLEX_UNKNOWN;
        goto done;
    }
    /* speed/duplex result is saved in PHY Specific Status Register */
    error = alx_read_phy_reg(&hw, ALX_MII_GIGA_PSSR, &giga);
    
    if (error)
        goto done;
    
    if (!(giga & ALX_GIGA_PSSR_SPD_DPLX_RESOLVED))
        goto wrong_speed;
    
    switch (giga & ALX_GIGA_PSSR_SPEED) {
        case ALX_GIGA_PSSR_1000MBS:
            hw.link_speed = SPEED_1000;
            break;
            
        case ALX_GIGA_PSSR_100MBS:
            hw.link_speed = SPEED_100;
            break;
            
        case ALX_GIGA_PSSR_10MBS:
            hw.link_speed = SPEED_10;
            break;
            
        default:
            goto wrong_speed;
    }
    hw.duplex = (giga & ALX_GIGA_PSSR_DPLX) ? DUPLEX_FULL : DUPLEX_HALF;
    
    /* Get the flow control settings. */
    flowControl = 0;
    
    error = alx_read_phy_reg(&hw, MII_LPA, &lpa);
    
    if (error)
        goto done;
    
    if (lpa & LPA_PAUSE_CAP)
        flowControl = (ALX_FC_RX | ALX_FC_TX) & hw.flowctrl;
    
    eeeLpa = 0;
    
    error = alx_read_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_REMOTE_EEEADV, &eeeLpa);
    
    if (error)
        goto done;
    
    DebugLog("EEE link partner: 0x%04x.\n", eeeLpa);
    
done:
    return error;
    
wrong_speed:
    IOLog("Invalid PHY speed/duplex: 0x%x\n", giga);
    error = -EINVAL;
    goto done;
}

void AtherosE2200::alxResetPhy()
{
    int i;
    UInt32 val;
    UInt16 phy_val;
    
    /* (DSP)reset PHY core */
    val = alxReadMem32(ALX_PHY_CTRL);
    val &= ~(ALX_PHY_CTRL_DSPRST_OUT | ALX_PHY_CTRL_IDDQ |
             ALX_PHY_CTRL_GATE_25M | ALX_PHY_CTRL_POWER_DOWN |
             ALX_PHY_CTRL_CLS);
    val |= ALX_PHY_CTRL_RST_ANALOG;
    
    val |= (ALX_PHY_CTRL_HIB_PULSE | ALX_PHY_CTRL_HIB_EN);
    alxWriteMem32(ALX_PHY_CTRL, val);
    udelay(10);
    alxWriteMem32(ALX_PHY_CTRL, val | ALX_PHY_CTRL_DSPRST_OUT);
    
    for (i = 0; i < ALX_PHY_CTRL_DSPRST_TO; i++)
        udelay(10);
    
    /* phy power saving & hib */
    alx_write_phy_dbg(&hw, ALX_MIIDBG_LEGCYPS, ALX_LEGCYPS_DEF);
    alx_write_phy_dbg(&hw, ALX_MIIDBG_SYSMODCTRL, ALX_SYSMODCTRL_IECHOADJ_DEF);
    alx_write_phy_ext(&hw, ALX_MIIEXT_PCS, ALX_MIIEXT_VDRVBIAS, ALX_VDRVBIAS_DEF);
    
    /* EEE advertisement */
    if (eeeAdv) {
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_LOCAL_EEEADV, eeeAdv);
        
        /* half amplify */
        alx_write_phy_dbg(&hw, ALX_MIIDBG_AZ_ANADECT, ALX_AZ_ANADECT_DEF);
        
        alx_read_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_EEE_ANEG, &phy_val);
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_EEE_ANEG, phy_val | eeeAdv);
    } else {
        val = alxReadMem32(ALX_LPI_CTRL);
        alxWriteMem32(ALX_LPI_CTRL, val & ~ALX_LPI_CTRL_EN);
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_LOCAL_EEEADV, 0);
    }
    
    /* phy power saving */
    alx_write_phy_dbg(&hw, ALX_MIIDBG_TST10BTCFG, ALX_TST10BTCFG_DEF);
    alx_write_phy_dbg(&hw, ALX_MIIDBG_SRDSYSMOD, ALX_SRDSYSMOD_DEF);
    alx_write_phy_dbg(&hw, ALX_MIIDBG_TST100BTCFG, ALX_TST100BTCFG_DEF);
    alx_write_phy_dbg(&hw, ALX_MIIDBG_ANACTRL, ALX_ANACTRL_DEF);
    alx_read_phy_dbg(&hw, ALX_MIIDBG_GREENCFG2, &phy_val);
    alx_write_phy_dbg(&hw, ALX_MIIDBG_GREENCFG2, phy_val & ~ALX_GREENCFG2_GATE_DFSE_EN);
    
    /* rtl8139c, 120m issue */
    alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_NLP78, ALX_MIIEXT_NLP78_120M_DEF);
    alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_S3DIG10, ALX_MIIEXT_S3DIG10_DEF);
    
    if (hw.lnk_patch) {
        /* Turn off half amplitude */
        alx_read_phy_ext(&hw, ALX_MIIEXT_PCS, ALX_MIIEXT_CLDCTRL3, &phy_val);
        alx_write_phy_ext(&hw, ALX_MIIEXT_PCS, ALX_MIIEXT_CLDCTRL3, phy_val | ALX_CLDCTRL3_BP_CABLE1TH_DET_GT);
        
        /* Turn off Green feature */
        alx_read_phy_dbg(&hw, ALX_MIIDBG_GREENCFG2, &phy_val);
        alx_write_phy_dbg(&hw, ALX_MIIDBG_GREENCFG2, phy_val | ALX_GREENCFG2_BP_GREEN);
        
        /* Turn off half Bias */
        alx_read_phy_ext(&hw, ALX_MIIEXT_PCS, ALX_MIIEXT_CLDCTRL5, &phy_val);
        alx_write_phy_ext(&hw, ALX_MIIEXT_PCS, ALX_MIIEXT_CLDCTRL5, phy_val | ALX_CLDCTRL5_BP_VD_HLFBIAS);
    }
    
    /* set phy interrupt mask */
    alx_write_phy_reg(&hw, ALX_MII_IER, ALX_IER_LINK_UP | ALX_IER_LINK_DOWN);
}

void AtherosE2200::alxPostPhyLink()
{
    UInt16 phy_val, len, agc;
    UInt8 revid = alx_hw_revision(&hw);
    bool adj_th = revid == ALX_REV_B0;
    
    if (revid != ALX_REV_B0 && !alx_is_rev_a(revid))
        return;
    
    /* 1000BT/AZ, wrong cable length */
    if (hw.link_speed != SPEED_UNKNOWN) {
        alx_read_phy_ext(&hw, ALX_MIIEXT_PCS, ALX_MIIEXT_CLDCTRL6, &phy_val);
        len = ALX_GET_FIELD(phy_val, ALX_CLDCTRL6_CAB_LEN);
        alx_read_phy_dbg(&hw, ALX_MIIDBG_AGC, &phy_val);
        agc = ALX_GET_FIELD(phy_val, ALX_AGC_2_VGA);
        
        if ((hw.link_speed == SPEED_1000 && (len > ALX_CLDCTRL6_CAB_LEN_SHORT1G || (len == 0 && agc > ALX_AGC_LONG1G_LIMT))) ||
            (hw.link_speed == SPEED_100 && (len > ALX_CLDCTRL6_CAB_LEN_SHORT100M || (len == 0 && agc > ALX_AGC_LONG100M_LIMT)))) {
                 alx_write_phy_dbg(&hw, ALX_MIIDBG_AZ_ANADECT, ALX_AZ_ANADECT_LONG);
                 alx_read_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_AFE, &phy_val);
                 alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_AFE, phy_val | ALX_AFE_10BT_100M_TH);
        } else {
                 alx_write_phy_dbg(&hw, ALX_MIIDBG_AZ_ANADECT, ALX_AZ_ANADECT_DEF);
                 alx_read_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_AFE, &phy_val);
                 alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_AFE, phy_val & ~ALX_AFE_10BT_100M_TH);
        }
        /* threshold adjust */
        if (adj_th && hw.lnk_patch) {
            if (hw.link_speed == SPEED_100) {
                alx_write_phy_dbg(&hw, ALX_MIIDBG_MSE16DB, ALX_MSE16DB_UP);
            } else if (hw.link_speed == SPEED_1000) {
                /*
                 * Giga link threshold, raise the tolerance of
                 * noise 50%
                 */
                alx_read_phy_dbg(&hw, ALX_MIIDBG_MSE20DB, &phy_val);
                ALX_SET_FIELD(phy_val, ALX_MSE20DB_TH, ALX_MSE20DB_TH_HI);
                alx_write_phy_dbg(&hw, ALX_MIIDBG_MSE20DB, phy_val);
            }
        }
        /* phy link-down in 1000BT/AZ mode */
        if (eeeEnable && revid == ALX_REV_B0 && hw.link_speed == SPEED_1000) {
            alx_write_phy_dbg(&hw, ALX_MIIDBG_SRDSYSMOD, ALX_SRDSYSMOD_DEF & ~ALX_SRDSYSMOD_DEEMP_EN);
        }
    } else {
        alx_read_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_AFE, &phy_val);
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_AFE, phy_val & ~ALX_AFE_10BT_100M_TH);
        
        if (adj_th && hw.lnk_patch) {
            alx_write_phy_dbg(&hw, ALX_MIIDBG_MSE16DB, ALX_MSE16DB_DOWN);
            alx_read_phy_dbg(&hw, ALX_MIIDBG_MSE20DB, &phy_val);
            ALX_SET_FIELD(phy_val, ALX_MSE20DB_TH, ALX_MSE20DB_TH_DEF);
            alx_write_phy_dbg(&hw, ALX_MIIDBG_MSE20DB, phy_val);
        }
        if (eeeCap && revid == ALX_REV_B0) {
            alx_write_phy_dbg(&hw, ALX_MIIDBG_SRDSYSMOD, ALX_SRDSYSMOD_DEF);
        }
    }
}

int AtherosE2200::alxSetupSpeedDuplex(UInt32 ethadv, UInt16 eeeadv, UInt8 flowctrl)
{
    UInt16 adv, giga, cr;
    UInt16 phy_val;
    UInt32 val;
    int err = 0;
    
    alx_write_phy_reg(&hw, ALX_MII_DBG_ADDR, 0);
    val = alxReadMem32(ALX_DRV);
    ALX_SET_FIELD(val, ALX_DRV_PHY, 0);
    
    /* EEE advertisement */
    if (eeeadv) {
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_LOCAL_EEEADV, eeeadv);
        
        /* half amplify */
        alx_write_phy_dbg(&hw, ALX_MIIDBG_AZ_ANADECT, ALX_AZ_ANADECT_DEF);
        
        alx_read_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_EEE_ANEG, &phy_val);
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_EEE_ANEG, phy_val | eeeadv);
    } else {
        alx_write_phy_ext(&hw, ALX_MIIEXT_ANEG, ALX_MIIEXT_LOCAL_EEEADV, 0);
    }

    if (ethadv & ADVERTISED_Autoneg) {
        adv = ADVERTISE_CSMA;
        adv |= ethtool_adv_to_mii_adv_t(ethadv);
        
        if (flowctrl & ALX_FC_ANEG) {
            if (flowctrl & ALX_FC_RX) {
                adv |= ADVERTISED_Pause;
                if (!(flowctrl & ALX_FC_TX))
                    adv |= ADVERTISED_Asym_Pause;
            } else if (flowctrl & ALX_FC_TX) {
                adv |= ADVERTISED_Asym_Pause;
            }
        }
        giga = 0;
        if (alx_hw_giga(&hw))
            giga = ethtool_adv_to_mii_ctrl1000_t(ethadv);
        
        cr = BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART;
        
        if (alx_write_phy_reg(&hw, MII_ADVERTISE, adv) ||
            alx_write_phy_reg(&hw, MII_CTRL1000, giga) ||
            alx_write_phy_reg(&hw, MII_BMCR, cr))
            err = -EBUSY;
    } else {
        cr = BMCR_RESET;
        if (ethadv == ADVERTISED_100baseT_Half ||
            ethadv == ADVERTISED_100baseT_Full)
            cr |= BMCR_SPEED100;
        if (ethadv == ADVERTISED_10baseT_Full ||
            ethadv == ADVERTISED_100baseT_Full)
            cr |= BMCR_FULLDPLX;
        
        err = alx_write_phy_reg(&hw, MII_BMCR, cr);
    }
    
    if (!err) {
        alx_write_phy_reg(&hw, ALX_MII_DBG_ADDR, ALX_PHY_INITED);
        val |= ethadv_to_hw_cfg(&hw, ethadv);
        
        if (eeeadv)
            val |= ALX_DRV_PHY_EEE;
    }
    
    alxWriteMem32(ALX_DRV, val);
    
    return err;
}

int AtherosE2200::alxSelectPowersavingSpeed(int *speed, UInt8 *duplex)
{
    int i, error;
    UInt16 lpa;
    UInt16 eee = 0;
    
    error = alxReadPhyLink();
    
    if (error)
        return error;
    
    if (hw.link_speed == SPEED_UNKNOWN) {
        *speed = SPEED_UNKNOWN;
        *duplex = DUPLEX_UNKNOWN;
        return 0;
    }
    
    error = alx_read_phy_reg(&hw, MII_LPA, &lpa);
    
    if (error)
        return error;
    
    if (!(lpa & LPA_LPACK)) {
        *speed = hw.link_speed;
        return 0;
    }
    
    if (lpa & LPA_10FULL) {
        *speed = SPEED_10;
        *duplex = DUPLEX_FULL;
    } else if (lpa & LPA_10HALF) {
        *speed = SPEED_10;
        *duplex = DUPLEX_HALF;
    } else if (lpa & LPA_100FULL) {
        *speed = SPEED_100;
        *duplex = DUPLEX_FULL;
        eee = ALX_LOCAL_EEEADV_100BT;
    } else {
        *speed = SPEED_100;
        *duplex = DUPLEX_HALF;
    }
    
    if (*speed == hw.link_speed && *duplex == hw.duplex)
        return 0;
    
    error = alx_write_phy_reg(&hw, ALX_MII_IER, 0);
    
    if (error)
        return error;
    
    error = alxSetupSpeedDuplex(alx_speed_to_ethadv(*speed, *duplex) | ADVERTISED_Autoneg, eee, ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX);
    
    if (error)
        return error;
    
    /* wait for linkup */
    for (i = 0; i < ALX_MAX_SETUP_LNK_CYCLE; i++) {
        IOSleep(100);
        
        error = alxReadPhyLink();
        
        if (error < 0)
            return error;
        
        if (hw.link_speed != SPEED_UNKNOWN)
            break;
    }
    if (i == ALX_MAX_SETUP_LNK_CYCLE)
        return -ETIMEDOUT;
    
    return 0;
}

void AtherosE2200::alxSpeedDuplexForMedium(const IONetworkMedium *medium)
{
    hw.flowctrl = (ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX);
    eeeAdv = 0;
    
    switch (medium->getIndex()) {
        case MEDIUM_INDEX_10HD:
            hw.adv_cfg = (ADVERTISED_10baseT_Half);
            break;
            
        case MEDIUM_INDEX_10FD:
            hw.adv_cfg = (ADVERTISED_10baseT_Full);
            break;
            
        case MEDIUM_INDEX_100HD:
            hw.adv_cfg = (ADVERTISED_100baseT_Half);
            break;
            
        case MEDIUM_INDEX_100FD:
            hw.adv_cfg = (ADVERTISED_100baseT_Full);
            break;
            
        case MEDIUM_INDEX_100FDFC:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_100baseT_Full);
            break;
            
        case MEDIUM_INDEX_100FDEEE:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_100baseT_Full);
            hw.flowctrl = ALX_FC_ANEG;
            eeeAdv = eeeCap;
            break;
            
        case MEDIUM_INDEX_100FDFCEEE:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_100baseT_Full);
            eeeAdv = eeeCap;
            break;
            
        case MEDIUM_INDEX_1000FD:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_1000baseT_Full);
            hw.flowctrl = ALX_FC_ANEG;
            break;
            
        case MEDIUM_INDEX_1000FDFC:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_1000baseT_Full);
            break;
            
        case MEDIUM_INDEX_1000FDEEE:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_1000baseT_Full);
            hw.flowctrl = ALX_FC_ANEG;
            eeeAdv = eeeCap;
            break;
            
        case MEDIUM_INDEX_1000FDFCEEE:
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_1000baseT_Full);
            eeeAdv = eeeCap;
            break;
            
        default:    /* MEDIUM_INDEX_AUTO */
            hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half);
            eeeAdv = eeeCap;
            
            if (gbCapable)
                hw.adv_cfg |= ADVERTISED_1000baseT_Full;
            break;
    }
}

IOReturn AtherosE2200::alxActiveMediumIndex(UInt32 *index)
{
    IOReturn result = kIOReturnSuccess;
    
    eeeEnable = 0;
    
    /* Get link speed, duplex and flow-control mode. */
    if (hw.link_speed == SPEED_1000) {
        eeeEnable = eeeAdv & eeeLpa;
        
        if (flowControl) {
            if (eeeEnable) {
                *index = MEDIUM_INDEX_1000FDFCEEE;
            } else {
                *index = MEDIUM_INDEX_1000FDFC;
            }
        } else {
            if (eeeEnable) {
                *index = MEDIUM_INDEX_1000FDEEE;
            } else {
                *index = MEDIUM_INDEX_1000FD;
            }
        }
    } else if (hw.link_speed == SPEED_100) {
        if (hw.duplex == DUPLEX_FULL) {
            eeeEnable = eeeAdv & eeeLpa;
            
            if (flowControl) {
                if (eeeEnable) {
                    *index = MEDIUM_INDEX_100FDFCEEE;
                } else {
                    *index = MEDIUM_INDEX_100FDFC;
                }
            } else {
                if (eeeEnable) {
                    *index = MEDIUM_INDEX_100FDEEE;
                } else {
                    *index = MEDIUM_INDEX_100FD;
                }
            }
        } else {
            *index = MEDIUM_INDEX_100HD;
        }
    } else {
        if (hw.duplex == DUPLEX_FULL) {
            *index = MEDIUM_INDEX_10FD;
        } else {
            *index = MEDIUM_INDEX_10HD;
        }
    }
    return result;
}
