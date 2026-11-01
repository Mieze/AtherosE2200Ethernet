/* AtherosE2200Ethernet.cpp -- AtherosE2200 driver class implementation.
 *
 * Copyright (c) 2014 Laura Müller <laura-mueller@uni-duesseldorf.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Driver for Atheros Killer E2200 PCIe ethernet controllers.
 *
 * This driver is based on Johannes Berg's alx driver for Linux.
 */


#include "AtherosE2200Ethernet.h"
#include "AtherosE2200RxPool.hpp"

#pragma mark --- function prototypes ---

static inline void adjustIPv4Header(mbuf_t m);
static inline UInt32 adjustIPv6Header(mbuf_t m);

static inline u32 ether_crc(int length, unsigned char *data);

#pragma mark --- private data ---

static const char *chipNames[] = {
    "Unkown",
    "AR8161",
    "AR8162",
    "AR8171",
    "AR8172",
    "Killer E2200",
    "Killer E2400",
    "Killer E2500",
};

static const char *mediumDescriptionArray[MEDIUM_INDEX_COUNT] = {
    "",
    "10-Megabit, half-duplex",
    "10-Megabit, full-duplex",
    "100-Megabit, half-duplex",
    "100-Megabit, full-duplex, no flow-control",
    "100-Megabit, full-duplex, Rx/Tx flow-control",
    "100-Megabit, full-duplex, no flow-control, energy-efficient-ethernet",
    "100-Megabit, full-duplex, Rx/Tx flow-control, energy-efficient-ethernet",
    "1-Gigabit, full-duplex, no flow-control",
    "1-Gigabit, full-duplex, Rx/Tx flow-control",
    "1-Gigabit, full-duplex, no flow-control, energy-efficient-ethernet",
    "1-Gigabit, full-duplex, Rx/Tx flow-control, energy-efficient-ethernet",
};

/* Power Management Support */
static IOPMPowerState powerStateArray[kPowerStateCount] =
{
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

static unsigned const ethernet_polynomial = 0x04c11db7U;

#pragma mark --- public methods ---

OSDefineMetaClassAndStructors(AtherosE2200, super)

/* IOService (or its superclass) methods. */

bool AtherosE2200::init(OSDictionary *properties)
{
    bool result;
    
    result = super::init(properties);
    
    if (result) {
        workLoop = NULL;
        commandGate = NULL;
        pciDevice = NULL;
        mediumDict = NULL;
        txQueue = NULL;
        interruptSource = NULL;
        timerSource = NULL;
        netif = NULL;
        netStats = NULL;
        etherStats = NULL;
        baseMap = NULL;
        baseAddr = NULL;
        mapper = NULL;
        
        txBufDesc = NULL;
        txPhyAddr = 0;
        txDescDmaCmd = NULL;
        txDescArray = NULL;
        txMbufCursor = NULL;
        txMbufArray = NULL;
        txBufArrayMem = NULL;
        txMapInfo = NULL;
        txMapMem = NULL;
        txDescDoneCount = 0;
        txDescDoneLast = 0;
        txNumFreeDesc = 0;
        txNextDescIndex = 0;
        txDirtyDescIndex = 0;

        rxBufDesc = NULL;
        rxRetPhyAddr = NULL;
        rxFreePhyAddr = NULL;
        rxDescDmaCmd = NULL;
        rxRetDescArray = NULL;
        rxFreeDescArray = NULL;
        rxPool = NULL;
        rxBufArray = NULL;
        rxBufArrayMem = NULL;

        isEnabled = false;
        promiscusMode = false;
        multicastMode = false;
        linkUp = false;
        polling = false;
        chip = kChipUnkown;
        eeeCap = 0;
        powerState = 0;
        pciDeviceData.vendor = 0;
        pciDeviceData.device = 0;
        pciDeviceData.subsystem_vendor = 0;
        pciDeviceData.subsystem_device = 0;
        pciDeviceData.revision = 0;
        hw.pdev = &pciDeviceData;
        wolCapable = false;
        gbCapable = false;
        enableTSO4 = false;
        enableTSO6 = false;
        enableCSO6 = false;
        useAppleVTD = false;
        flowControl = 0;
        pciPMCtrlOffset = 0;
    }
    
done:
    return result;
}

void AtherosE2200::free()
{
    UInt32 i;
    
    DebugLog("free() ===>\n");
    
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediumDict);
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    RELEASE(baseMap);
    baseAddr = NULL;
    
    RELEASE(pciDevice);

    DebugLog("free() <===\n");
    
    super::free();
}

bool AtherosE2200::start(IOService *provider)
{
    OSString *versionString;
    UInt32 newIntrRate;
    bool result;
    
    result = super::start(provider);
    
    if (!result) {
        IOLog("IOEthernetController::start failed.\n");
        goto done;
    }
    multicastMode = false;
    promiscusMode = false;
    multicastFilter[0] = multicastFilter[1] = 0;
    
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    
    if (!pciDevice) {
        IOLog("No provider.\n");
        goto done;
    }
    pciDevice->retain();
    
    if (!pciDevice->open(this)) {
        IOLog("Failed to open provider.\n");
        goto error_open;
    }
    mapper = IOMapper::copyMapperForDevice(pciDevice);

    if (!initPCIConfigSpace(pciDevice)) {
        goto error_cfg;
    }
    /* Get the config values from Info.plist. */
    getParams(&newIntrRate);
    
    if (!alxStart(newIntrRate)) {
        goto error_cfg;
    }
    versionString = OSDynamicCast(OSString, getProperty(kDriverVersionName));
    newIntrRate = 1000000 / hw.imt;
    
    if (versionString)
        IOLog("Version %s using max interrupt rate %u. Please don't support tonymacx86.com!\n", versionString->getCStringNoCopy(), newIntrRate);
    else
        IOLog("Using max interrupt rate %u. Please don't support tonymacx86.com!\n", newIntrRate);

    if (!setupMediumDict()) {
        IOLog("Failed to setup medium dictionary.\n");
        goto error_cfg;
    }
    commandGate = getCommandGate();
    
    if (!commandGate) {
        IOLog("getCommandGate() failed.\n");
        goto error_gate;
    }
    commandGate->retain();
    
    if (!setupTxResources()) {
        IOLog("Error allocating Tx resources.\n");
        goto error_dma1;
    }

    if (!setupRxResources()) {
        IOLog("Error allocating Rx resources.\n");
        goto error_dma2;
    }

    if (!initEventSources(provider)) {
        IOLog("initEventSources() failed.\n");
        goto error_src;
    }
    
    result = attachInterface(reinterpret_cast<IONetworkInterface**>(&netif));
    
    if (!result) {
        IOLog("attachInterface() failed.\n");
        goto error_dma1;
    }
    pciDevice->close(this);
    result = true;
    
done:
    return result;
    
error_src:
    freeRxResources();

error_dma2:
    freeTxResources();
    
error_dma1:
    RELEASE(commandGate);

error_gate:
    RELEASE(mediumDict);

error_cfg:
    pciDevice->close(this);
    
error_open:
    pciDevice->release();
    pciDevice = NULL;
    goto done;
}

void AtherosE2200::stop(IOService *provider)
{
    UInt32 i;
    
    if (netif) {
        detachInterface(netif);
        RELEASE(netif);
    }
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediumDict);
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    freeRxResources();
    freeTxResources();
    RELEASE(baseMap);
    baseAddr = NULL;
    
    RELEASE(pciDevice);
    
    super::stop(provider);
}

IOReturn AtherosE2200::registerWithPolicyMaker(IOService *policyMaker)
{
    DebugLog("registerWithPolicyMaker() ===>\n");
    
    powerState = kPowerStateOn;
    
    DebugLog("registerWithPolicyMaker() <===\n");
    
    return policyMaker->registerPowerDriver(this, powerStateArray, kPowerStateCount);
}

IOReturn AtherosE2200::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
    IOReturn result = IOPMAckImplied;
    
    DebugLog("setPowerState() ===>\n");
    
    if (powerStateOrdinal == powerState) {
        DebugLog("Already in power state %lu.\n", powerStateOrdinal);
        goto done;
    }
    DebugLog("Switching to power state %lu.\n", powerStateOrdinal);
    
    if (powerStateOrdinal == kPowerStateOff)
        commandGate->runAction(setPowerStateSleepAction);
    else
        commandGate->runAction(setPowerStateWakeAction);
    
    powerState = powerStateOrdinal;
    
done:
    DebugLog("setPowerState() <===\n");
    
    return result;
}

void AtherosE2200::systemWillShutdown(IOOptionBits specifier)
{
    DebugLog("systemWillShutdown() ===>\n");
    
    if ((kIOMessageSystemWillPowerOff | kIOMessageSystemWillRestart) & specifier) {
        disable(netif);
        
        /* Restore the original MAC address. */
        alxLoadDefaultAddress();
    }
    
    DebugLog("systemWillShutdown() <===\n");
    
    /* Must call super on shutdown or system will stall. */
    super::systemWillShutdown(specifier);
}

/* IONetworkController methods. */
IOReturn AtherosE2200::enable(IONetworkInterface *netif)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("enable() ===>\n");
    
    if (isEnabled) {
        DebugLog("Interface already enabled.\n");
        result = kIOReturnSuccess;
        goto done;
    }
    if (!pciDevice || pciDevice->isOpen()) {
        IOLog("Unable to open PCI device.\n");
        goto done;
    }
    pciDevice->open(this);
    
    txDescDoneCount = txDescDoneLast = 0;
    deadlockWarn = 0;
    rxNextDescIndex = 0;
    rxCleanedCount = rxMapNextIndex = 0;

    alxEnable();
    
    /* Enable the MSI interrupt. */
    interruptSource->enable();
    

    polling = false;
    isEnabled = true;
    
    result = kIOReturnSuccess;
    
    DebugLog("enable() <===\n");
    
done:
    return result;
}

IOReturn AtherosE2200::disable(IONetworkInterface *netif)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("disable() ===>\n");
    
    if (!isEnabled)
        goto done;
    
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    polling = false;

    timerSource->cancelTimeout();
    txDescDoneCount = txDescDoneLast = 0;
    multicastFilter[0] = multicastFilter[1] = 0;
    isEnabled = false;

    /* Disable the MSI interrupt. */
    interruptSource->disable();
    
    alxDisable();
    clearDescriptors();
    
    if (pciDevice && pciDevice->isOpen())
        pciDevice->close(this);
    
    DebugLog("disable() <===\n");
    
done:
    return result;
}

IOReturn AtherosE2200::outputStart(IONetworkInterface *interface, IOOptionBits options )
{
    IOPhysicalSegment txSegments[kMaxSegs];
    mbuf_t m;
    QCATxDesc *desc;
    IOReturn result = kIOReturnNoResources;
    UInt32 numDescs;
    UInt32 cmd;
    UInt32 totalLen;
    UInt32 mssValue;
    UInt32 word1;
    UInt32 numSegs;
    UInt32 lastSeg;
    UInt32 index;
    mbuf_tso_request_flags_t tsoFlags;
    mbuf_csum_request_flags_t checksums;
    UInt16 vlanTag;
    UInt16 segLen;
    UInt16 i;
    UInt16 count;
    
    //DebugLog("outputPacket() ===>\n");
    count = 0;

    if (!(isEnabled && linkUp)) {
        DebugLog("Interface down. Dropping packets.\n");
        goto done;
    }
    while ((txNumFreeDesc > (kMaxSegs + 3)) && (interface->dequeueOutputPackets(1, &m, NULL, NULL, NULL) == kIOReturnSuccess)) {
        numDescs = 0;
        cmd = 0;
        totalLen = 0;

        if (mbuf_get_tso_requested(m, &tsoFlags, &mssValue)) {
            DebugLog("mbuf_get_tso_requested() failed. Dropping packet.\n");
            freePacket(m);
            continue;
        }
        /* First prepare the header and the command bits. */
        if (tsoFlags & (MBUF_TSO_IPV4 | MBUF_TSO_IPV6)) {
            if (tsoFlags & MBUF_TSO_IPV4) {
                /* Correct the pseudo header checksum. */
                adjustIPv4Header(m);
                
                /* Setup the command bits for TSO over IPv4. */
                cmd = (((mssValue & TPD_MSS_MASK) << TPD_MSS_SHIFT) | TPD_IPV4 | TPD_LSO_EN | kMinL4HdrOffsetV4);
            } else {
                /* Correct the pseudo header checksum and get the size of the packet including all headers. */
                totalLen = adjustIPv6Header(m);
                
                /* Setup the command bits for TSO over IPv6. */
                cmd = (((mssValue & TPD_MSS_MASK) << TPD_MSS_SHIFT) | TPD_LSO_V2 | TPD_LSO_EN | kMinL4HdrOffsetV6);
                numDescs = 1;
            }
        } else {
            /* We use mssValue as a dummy here because we don't need it anymore. */
            mbuf_get_csum_requested(m, &checksums, &mssValue);
            
            /* Next setup the checksum command bits. */
            alxGetChkSumCommand(&cmd, checksums);
        }
        /* Next get the VLAN tag and command bit. */
        cmd |= (!mbuf_get_vlan_tag(m, &vlanTag)) ? TPD_INS_VLTAG : 0;
        
        /* Finally get the physical segments. */
        if (useAppleVTD)
            numSegs = txMapPacket(m, txSegments, kMaxSegs);
        else
            numSegs = txMbufCursor->getPhysicalSegmentsWithCoalesce(m, txSegments, kMaxSegs);
        
        numDescs += numSegs;
        
        if (!numSegs) {
            DebugLog("getPhysicalSegmentsWithCoalesce() failed. Dropping packet.\n");
            etherStats->dot3TxExtraEntry.resourceErrors++;
            freePacket(m);
            continue;
        }
        OSAddAtomic(-numDescs, &txNumFreeDesc);
        index = txNextDescIndex;
        txNextDescIndex = (txNextDescIndex + numDescs) & kTxDescMask;
        lastSeg = numSegs - 1;
        
        /* Setup the context descriptor for TSO over IPv6. */
        if (tsoFlags & MBUF_TSO_IPV6) {
            desc = &txDescArray[index];
            
            desc->vlanTag = OSSwapHostToBigInt16(vlanTag);
            desc->word1 = OSSwapHostToLittleInt32(cmd);
            desc->adrl.l.pktLength = OSSwapHostToLittleInt32(totalLen);
            
            ++index &= kTxDescMask;
        }
        /* And finally fill in the data descriptors. */
        for (i = 0; i < numSegs; i++) {
            desc = &txDescArray[index];
            word1 = cmd;
            segLen = (UInt16)txSegments[i].length;
            
            if (i == lastSeg) {
                word1 |= TPD_EOP;
                txMbufArray[index] = m;
            } else {
                txMbufArray[index] = NULL;
            }
            desc->vlanTag = OSSwapHostToBigInt16(vlanTag);
            desc->length = OSSwapHostToLittleInt16(segLen);
            desc->word1 = OSSwapHostToLittleInt32(word1);
            desc->adrl.addr = OSSwapHostToLittleInt64(txSegments[i].location);
            
            ++index &= kTxDescMask;
        }
        count++;
    }
    if (count) {
        /* flush updates before updating hardware */
        OSSynchronizeIO();
        alxWriteMem16(ALX_TPD_PRI0_PIDX, txNextDescIndex);
    }
    result = (txNumFreeDesc > (kMaxSegs + 3)) ? kIOReturnSuccess : kIOReturnNoResources;

done:
    //DebugLog("outputStart() <===\n");
    
    return result;
}

/* 
 * We are enforcing alignment by requesting packets of excatly 2048 bytes which is
 * the size of a cluster so that all packets will be 2k aligned. As the kernel adds
 * aligment constraints to the requested size we have to claim that there are no
 * aligment constraints in order to perfectly match the requested packet size.
 */

void AtherosE2200::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
    DebugLog("getPacketBufferConstraints() ===>\n");
    
	constraints->alignStart = kIOPacketBufferAlign1;
	constraints->alignLength = kIOPacketBufferAlign1;
    
    DebugLog("getPacketBufferConstraints() <===\n");
}

IOOutputQueue* AtherosE2200::createOutputQueue()
{
    DebugLog("createOutputQueue() ===>\n");
    
    DebugLog("createOutputQueue() <===\n");
    
    return IOBasicOutputQueue::withTarget(this);
}

const OSString* AtherosE2200::newVendorString() const
{
    DebugLog("newVendorString() ===>\n");
    
    DebugLog("newVendorString() <===\n");
    
    return OSString::withCString("Qualcomm Atheros");
}

const OSString* AtherosE2200::newModelString() const
{
    DebugLog("newModelString() ===>\n");
    DebugLog("newModelString() <===\n");
    
    return OSString::withCString(chipNames[chip]);
}

bool AtherosE2200::configureInterface(IONetworkInterface *interface)
{
    char modelName[kNameLenght];
    IONetworkData *data;
    IOReturn error;
    bool result;
    
    DebugLog("configureInterface() ===>\n");
    
    result = super::configureInterface(interface);
    
    if (!result)
        goto done;
	
    /* Get the generic network statistics structure. */
    data = interface->getParameter(kIONetworkStatsKey);
    
    if (data) {
        netStats = (IONetworkStats *)data->getBuffer();
        
        if (!netStats) {
            IOLog("Error getting IONetworkStats\n.");
            result = false;
            goto done;
        }
    }
    /* Get the Ethernet statistics structure. */
    data = interface->getParameter(kIOEthernetStatsKey);
    
    if (data) {
        etherStats = (IOEthernetStats *)data->getBuffer();
        
        if (!etherStats) {
            IOLog("Error getting IOEthernetStats\n.");
            result = false;
            goto done;
        }
    }
    
    error = interface->configureOutputPullModel(512, 0, 0, IONetworkInterface::kOutputPacketSchedulingModelNormal);
    
    if (error != kIOReturnSuccess) {
        IOLog("configureOutputPullModel() failed\n.");
        result = false;
        goto done;
    }
    error = interface->configureInputPacketPolling(kNumRxDesc, kIONetworkWorkLoopSynchronous);
    
    if (error != kIOReturnSuccess) {
        IOLog("configureInputPacketPolling() failed\n.");
        result = false;
        goto done;
    }

    if ((chip == kChipAR8162) || (chip == kChipAR8172))
        snprintf(modelName, kNameLenght, "Qualcomm Atheros %s PCIe Fast Ethernet", chipNames[chip]);
    else
        snprintf(modelName, kNameLenght, "Qualcomm Atheros %s PCIe Gigabit Ethernet", chipNames[chip]);
    
    setProperty("model", modelName);
    
    DebugLog("configureInterface() <===\n");
    
done:
    return result;
}

bool AtherosE2200::createWorkLoop()
{
    DebugLog("createWorkLoop() ===>\n");
    
    workLoop = IOWorkLoop::workLoop();
    
    DebugLog("createWorkLoop() <===\n");
    
    return workLoop ? true : false;
}

IOWorkLoop* AtherosE2200::getWorkLoop() const
{
    DebugLog("getWorkLoop() ===>\n");
    
    DebugLog("getWorkLoop() <===\n");
    
    return workLoop;
}

IOReturn AtherosE2200::setPromiscuousMode(bool active)
{
    UInt32 mcFilter[2];
    
    DebugLog("setPromiscuousMode() ===>\n");

    hw.rx_ctrl &= ~(ALX_MAC_CTRL_MULTIALL_EN | ALX_MAC_CTRL_PROMISC_EN);

    if (active) {
        DebugLog("Promiscuous mode enabled.\n");
        hw.rx_ctrl |= ALX_MAC_CTRL_PROMISC_EN;
        mcFilter[1] = mcFilter[0] = 0xffffffff;
    } else {
        DebugLog("Promiscuous mode disabled.\n");
        mcFilter[0] = multicastFilter[0];
        mcFilter[1] = multicastFilter[1];
        
        if ((mcFilter[0] == 0xffffffff) && (mcFilter[1] == 0xffffffff))
            hw.rx_ctrl |= ALX_MAC_CTRL_MULTIALL_EN;
    }
    promiscusMode = active;
    alxWriteMem32(ALX_HASH_TBL0, mcFilter[0]);
    alxWriteMem32(ALX_HASH_TBL1, mcFilter[1]);
    alxWriteMem32(ALX_MAC_CTRL, hw.rx_ctrl);

    DebugLog("setPromiscuousMode() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn AtherosE2200::setMulticastMode(bool active)
{
    UInt32 mcFilter[2];
    
    DebugLog("setMulticastMode() ===>\n");

    hw.rx_ctrl &= ~(ALX_MAC_CTRL_MULTIALL_EN | ALX_MAC_CTRL_PROMISC_EN);

    if (active) {
        mcFilter[0] = multicastFilter[0];
        mcFilter[1] = multicastFilter[1];
        
        if ((mcFilter[0] == 0xffffffff) && (mcFilter[1] == 0xffffffff))
            hw.rx_ctrl |= ALX_MAC_CTRL_MULTIALL_EN;
    } else{
        mcFilter[1] = mcFilter[0] = 0;
    }
    multicastMode = active;
    alxWriteMem32(ALX_HASH_TBL0, mcFilter[0]);
    alxWriteMem32(ALX_HASH_TBL1, mcFilter[1]);
    alxWriteMem32(ALX_MAC_CTRL, hw.rx_ctrl);

    DebugLog("setMulticastMode() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn AtherosE2200::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
    UInt32 crc32, bit, reg, i;

    DebugLog("setMulticastList() ===>\n");

    if (count <= kMCFilterLimit) {
        multicastFilter[0] = multicastFilter[1] = 0;

        for (i = 0; i < count; i++, addrs++) {
            crc32 = ether_crc(ETHER_ADDR_LEN, reinterpret_cast<unsigned char *>(addrs));
            reg = (crc32 >> 31) & 0x1;
            bit = (crc32 >> 26) & 0x1F;
            multicastFilter[reg] |= BIT(bit);
        }
        hw.rx_ctrl &= ~ALX_MAC_CTRL_MULTIALL_EN;
    } else {
        multicastFilter[0] = multicastFilter[1] = 0xffffffff;
        hw.rx_ctrl |= ALX_MAC_CTRL_MULTIALL_EN;
    }
    alxWriteMem32(ALX_HASH_TBL0, multicastFilter[0]);
    alxWriteMem32(ALX_HASH_TBL1, multicastFilter[1]);
    alxWriteMem32(ALX_MAC_CTRL, hw.rx_ctrl);
    
    DebugLog("setMulticastList() <===\n");

    return kIOReturnSuccess;
}

IOReturn AtherosE2200::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("getChecksumSupport() ===>\n");
    
    if ((checksumFamily == kChecksumFamilyTCPIP) && checksumMask) {
        if (isOutput) {
            *checksumMask = (enableCSO6) ? (kChecksumTCP | kChecksumUDP | kChecksumIP | kChecksumTCPIPv6 | kChecksumUDPIPv6) : (kChecksumTCP | kChecksumUDP | kChecksumIP);
        } else {
            *checksumMask = (kChecksumTCP | kChecksumUDP | kChecksumIP | kChecksumTCPIPv6 | kChecksumUDPIPv6);
        }
        result = kIOReturnSuccess;
    }
    DebugLog("getChecksumSupport() <===\n");
    
    return result;
}

UInt32 AtherosE2200::getFeatures() const
{
    UInt32 features = (kIONetworkFeatureMultiPages | kIONetworkFeatureHardwareVlan);
    
    DebugLog("getFeatures() ===>\n");
    
    if (enableTSO4)
        features |= kIONetworkFeatureTSOIPv4;
    
    if (enableTSO6)
        features |= kIONetworkFeatureTSOIPv6;
    
    DebugLog("getFeatures() <===\n");
    
    return features;
}

IOReturn AtherosE2200::setWakeOnMagicPacket(bool active)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("setWakeOnMagicPacket() ===>\n");
    
    if (wolCapable) {
        hw.sleep_ctrl = active ? (ALX_SLEEP_WOL_MAGIC | ALX_SLEEP_CIFS) : 0;
        DebugLog("Wake on magic packet %s.\n", active ? "enabled" : "disabled");
        result = kIOReturnSuccess;
    }
    
    DebugLog("setWakeOnMagicPacket() <===\n");
    
    return result;
}

IOReturn AtherosE2200::getPacketFilters(const OSSymbol *group, UInt32 *filters) const
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("getPacketFilters() ===>\n");
    
    if ((group == gIOEthernetWakeOnLANFilterGroup) && wolCapable) {
        *filters = kIOEthernetWakeOnMagicPacket;
        DebugLog("kIOEthernetWakeOnMagicPacket added to filters.\n");
    } else {
        result = super::getPacketFilters(group, filters);
    }
    
    DebugLog("getPacketFilters() <===\n");
    
    return result;
}

IOReturn AtherosE2200::setHardwareAddress(const IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("setHardwareAddress() ===>\n");

    if (addr && ether_addr_equal(&addr->bytes[0], &origMacAddr.bytes[0])) {
        alxLoadDefaultAddress();
    } else {
        result = alxSetHardwareAddress(addr);
    }

    DebugLog("setHardwareAddress() <===\n");
    
    return result;
}

/* Methods inherited from IOEthernetController. */
IOReturn AtherosE2200::getHardwareAddress(IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnError;
    UInt32 mac0, mac1;
    
    DebugLog("getHardwareAddress() ===>\n");
    
    if (addr) {
        mac0 = alxReadMem32(ALX_STAD0);
        mac1 = alxReadMem32(ALX_STAD1);
        
        addr->bytes[0] = ((mac1 >> 8) & 0xff);
        addr->bytes[1] = (mac1 & 0xff);
        addr->bytes[2] = ((mac0 >> 24) & 0xff);
        addr->bytes[3] = ((mac0 >> 16) & 0xff);
        addr->bytes[4] = ((mac0 >> 8) & 0xff);
        addr->bytes[5] = (mac0 & 0xff);
        
        if (is_valid_ether_addr(&addr->bytes[0]))
            result = kIOReturnSuccess;
    }
    
    DebugLog("getHardwareAddress() <===\n");
    
    return result;
}

IOReturn AtherosE2200::selectMedium(const IONetworkMedium *medium)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("selectMedium() ===>\n");
    
    if (medium) {
        alxSpeedDuplexForMedium(medium);
        setLinkDown();
        alxSetupSpeedDuplex(hw.adv_cfg, eeeAdv, hw.flowctrl);
        setCurrentMedium(medium);
    }
    
    DebugLog("selectMedium() <===\n");
    
done:
    return result;
}

IOReturn AtherosE2200::getMaxPacketSize(UInt32 * maxSize) const
{
    DebugLog("getMaxPacketSize() ===>\n");

    *maxSize = kMaxPacketSize;
    
    DebugLog("getMaxPacketSize() <===\n");

    return kIOReturnSuccess;
}

IOReturn AtherosE2200::setMaxPacketSize(UInt32 maxSize)
{
    IOReturn result = kIOReturnError;
    ifnet_t ifnet = netif->getIfnet();
    ifnet_offload_t offload;
    UInt32 mask = 0;

    DebugLog("setMaxPacketSize() ===>\n");
    
    if (maxSize <= kMaxPacketSize) {
        hw.mtu = maxSize - (ETH_HLEN + ETH_FCS_LEN);
        
        DebugLog("maxSize: %u, mtu: %u\n", maxSize, hw.mtu);

        if(enableTSO4)
            mask |= IFNET_TSO_IPV4;
        
        if(enableTSO6)
            mask |= IFNET_TSO_IPV6;
        
        offload = ifnet_offload(ifnet);
        
        if (hw.mtu > ALX_MAX_TSO_PKT_SIZE) {
            offload &= ~mask;
            DebugLog("Disable hardware offload features: %x!\n", mask);
        } else {
            offload |= mask;
            DebugLog("Enable hardware offload features: %x!\n", mask);
        }
        if (ifnet_set_offload(ifnet, offload))
            IOLog("Error setting hardware offload: %x!\n", offload);

        /* Force reinitialization. */
        setLinkDown();
        alxSetupSpeedDuplex(hw.adv_cfg, eeeAdv, hw.flowctrl);
        
        result = kIOReturnSuccess;
    }
    
    DebugLog("setMaxPacketSize() <===\n");
    
    return result;
}

#pragma mark --- rx poll methods ---

IOReturn AtherosE2200::setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled)
{
    //DebugLog("setInputPacketPollingEnable() ===>\n");
    
    if (isEnabled) {
        if (enabled) {
            intrMask = (ALX_ISR_MISC | ALX_ISR_PHY);
        } else {
            intrMask = (ALX_ISR_MISC | ALX_ISR_PHY | ALX_ISR_RX_Q0 | ALX_ISR_TX_Q0);
        }
        polling = enabled;
        alxWriteMem32(ALX_IMR, intrMask);
    }
    //DebugLog("input polling %s.\n", enabled ? "enabled" : "disabled");
    
    //DebugLog("setInputPacketPollingEnable() <===\n");
    
    return kIOReturnSuccess;
}

void AtherosE2200::pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context )
{
    //DebugLog("pollInputPackets() ===>\n");
    
    if (polling) {
        if (useAppleVTD)
            rxInterruptVTD(interface, maxCount, pollQueue, context);
        else
            rxInterrupt(interface, maxCount, pollQueue, context);
    
        /* Finally cleanup the transmitter ring. */
        txInterrupt();
    }
    //DebugLog("pollInputPackets() <===\n");
}


#pragma mark --- common interrupt methods ---

void AtherosE2200::txInterrupt()
{
    UInt16 newDirtyIndex = alxReadMem16(ALX_TPD_PRI0_CIDX);
    
    //DebugLog("txInterrupt oldIndex=%u newIndex=%u\n", txDirtyDescIndex, newDirtyIndex);

    if (txDirtyDescIndex != newDirtyIndex) {
        while (txDirtyDescIndex != newDirtyIndex) {
            if (txMbufArray[txDirtyDescIndex]) {
                if (useAppleVTD)
                    txUnmapPacket();
                
                freePacket(txMbufArray[txDirtyDescIndex], kDelayFree);
                txMbufArray[txDirtyDescIndex] = NULL;
            }
            txDescDoneCount++;
            OSIncrementAtomic(&txNumFreeDesc);
            ++txDirtyDescIndex &= kTxDescMask;
        }
        releaseFreePackets();
        
        if (txNumFreeDesc > kTxQueueWakeTreshhold)
            netif->signalOutputThread();
        
        if (!polling)
            etherStats->dot3TxExtraEntry.interrupts++;
    }
}

UInt32 AtherosE2200::rxInterrupt(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    QCARxRetDesc *desc = &rxRetDescArray[rxNextDescIndex];
    mbuf_t bufPkt, newPkt;
    mbuf_t extraPkt, tailPkt;
    UInt64 addr;
    UInt32 status0, status2, status3;
    UInt32 pktSize;
    UInt32 validMask;
    SInt32 extraSize;
    UInt16 index;
    UInt16 lastIndex;
    UInt16 vlanTag;
    UInt16 goodPkts = 0;
    bool replaced;
    
    //DebugLog("rxInterrupt()\n");
    
    while (((status3 = OSSwapLittleToHostInt32(desc->word3)) & RRD_UPDATED) && (goodPkts < maxCount)) {
        status0 = OSSwapLittleToHostInt32(desc->word0);
        status2 = OSSwapLittleToHostInt32(desc->word2);
        pktSize = (status3 & RRD_PKTLEN_MASK) - kIOEthernetCRCSize;
        index = (status0 >> RRD_SI_SHIFT) & RRD_SI_MASK;
        lastIndex = (index + (((status0 >> RRD_NOR_SHIFT) & 0x000F) - 1)) & kRxDescMask;
        vlanTag = (status3 & RRD_VLTAGGED) ? OSSwapBigToHostInt16(status2 & RRD_VLTAG_MASK) : 0;
        bufPkt = rxBufArray[index].mbuf;
        extraSize = pktSize - kRxBufferSize;

        //DebugLog("Packet with index=%u, numBufs=%u, pktSize=%u, errors=0x%x\n", index, numBufs, pktSize, errors);
        
        /* Skip bad packet. */
        if (status3 & RRD_ERR_MASK) {
            DebugLog("Bad packet. error: 0x%x\n", (status3 & RRD_ERR_MASK) >> RRD_ERR_FCS_SHIFT);
            etherStats->dot3StatsEntry.internalMacReceiveErrors++;
            goto nextDesc;
        }
        tailPkt = newPkt = rxPool->replaceOrCopyPacket(&bufPkt, pktSize, &replaced);
        
        if (!newPkt) {
            /* Allocation of a new packet failed so that we must leave the original packet in place. */
            DebugLog("replaceOrCopyPacket() failed.\n");
            etherStats->dot3RxExtraEntry.resourceErrors++;
            goto nextDesc;
        }
handle_pkt:
        /* If the packet was replaced we have to update the free descriptor's buffer address. */
        if (replaced) {
            if (mbuf_next(bufPkt) != NULL) {
                DebugLog("getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                mbuf_freem_list(bufPkt);
                goto nextDesc;
            }
            addr = mbuf_data_to_physical(mbuf_data(bufPkt));
            rxBufArray[index].mbuf = bufPkt;
            rxBufArray[index].phyAddr = addr;

            rxFreeDescArray[index].addr = OSSwapHostToLittleInt64(addr);
        }
        while (extraSize > 0) {
            ++index &= kRxDescMask;
            bufPkt = rxBufArray[index].mbuf;
            extraPkt = rxPool->replaceOrCopyPacket(&bufPkt, kRxBufferSize, &replaced);
            
            if (!extraPkt) {
                /* Allocation of a new packet failed so that we must leave the original packet in place. */
                DebugLog("getPhysicalSegments() failed for jumbo frame.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                freePacket(newPkt);
                goto nextDesc;
            }
            mbuf_setflags_mask(extraPkt, 0, MBUF_PKTHDR);
            mbuf_setlen(extraPkt, (extraSize > kRxBufferSize) ? kRxBufferSize : extraSize);
            mbuf_setnext(tailPkt, extraPkt);
            
            if (replaced) {
                addr = mbuf_data_to_physical(mbuf_data(bufPkt));
                
                if (!addr) {
                    DebugLog("getPhysicalSegments() failed for jumbo frame.\n");
                    etherStats->dot3RxExtraEntry.resourceErrors++;
                    freePacket(bufPkt);
                    goto nextDesc;
                }
                rxBufArray[index].mbuf = bufPkt;
                rxBufArray[index].phyAddr = addr;
                rxFreeDescArray[index].addr = OSSwapHostToLittleInt64(addr);
            }
            extraSize -= kRxBufferSize;
            tailPkt = extraPkt;
        }
        switch (getProtocolID(status2)) {
            case RRD_PID_IPV4:
                validMask = (status3 & RRD_ERR_IPV4) ? 0 : kChecksumIP;
                break;
                
            case RRD_PID_IPV6TCP:
                validMask = (status3 & RRD_ERR_L4) ? 0 : kChecksumTCPIPv6;
                break;
                
            case RRD_PID_IPV4TCP:
                validMask = (status3 & (RRD_ERR_L4 | RRD_ERR_IPV4)) ? 0 : (kChecksumTCP | kChecksumIP);
                break;
                
            case RRD_PID_IPV6UDP:
                validMask = (status3 & RRD_ERR_L4) ? 0 : kChecksumUDPIPv6;
                break;
                
            case RRD_PID_IPV4UDP:
                validMask = (status3 & (RRD_ERR_L4 | RRD_ERR_IPV4)) ? 0 : (kChecksumUDP | kChecksumIP);
                break;
                
            default:
                validMask = 0;
        }
        if (validMask)
            setChecksumResult(newPkt, kChecksumFamilyTCPIP, validMask, validMask);
        
        /* Also get the VLAN tag if there is any. */
        if (vlanTag)
            setVlanTag(newPkt, vlanTag);
        
        mbuf_pkthdr_setlen(newPkt, pktSize);
        mbuf_setlen(newPkt, (pktSize > kRxBufferSize) ? kRxBufferSize : pktSize);
        interface->enqueueInputPacket(newPkt, pollQueue);
        goodPkts++;
        
        /* Finally update the descriptor and get the next one to examine. */
    nextDesc:
        desc->word3 = OSSwapHostToLittleInt32(status3 & ~RRD_UPDATED);
        
        ++rxNextDescIndex &= kRxDescMask;
        desc = &rxRetDescArray[rxNextDescIndex];
        
        alxWriteMem16(ALX_RFD_PIDX, lastIndex);
    }
    return goodPkts;
}

void AtherosE2200::checkLinkStatus()
{
	int oldSpeed;
    
	/* clear PHY internal interrupt status, otherwise the main
	 * interrupt status will be asserted forever
	 */
	alx_clear_phy_intr(&hw);
    
	oldSpeed = hw.link_speed;
    
	if (alxReadPhyLink() == 0) {
        if (oldSpeed != hw.link_speed) {
            if (hw.link_speed != SPEED_UNKNOWN)
                setLinkUp();
            else
                setLinkDown();
        }
    }
}

void AtherosE2200::interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count)
{
	UInt32 status = alxReadMem32(ALX_ISR);
    UInt32 packets;
    
    /* hotplug/major error/no more work/shared irq */
	if (status & ALX_ISR_DIS || !(status & intrMask))
        goto done;

    /* ACK interrupt */
	alxWriteMem32(ALX_ISR, status | ALX_ISR_DIS);

	if (status & ALX_ISR_FATAL) {
        IOLog("Fatal interrupt. Reseting chip. ISR=0x%x\n", status);
        etherStats->dot3TxExtraEntry.resets++;
        etherStats->dot3RxExtraEntry.resets++;
		alxRestart();
		return;
	}
    if (status & ALX_ISR_ALERT) {
        IOLog("Alert interrupt. Reseting chip. ISR=0x%x\n", status);
        etherStats->dot3TxExtraEntry.resets++;
        etherStats->dot3RxExtraEntry.resets++;
        alxRestart();
        return;
    }

    if (!polling) {
        if (status & ALX_ISR_TX_Q0)
            txInterrupt();
        
        if (status & ALX_ISR_RX_Q0) {
            packets = rxInterrupt(netif, kNumRxDesc, NULL, NULL);
            
            if (packets)
                netif->flushInputQueue();
        }
    }
	if (status & ALX_ISR_PHY)
        checkLinkStatus();
	   
done:
    alxWriteMem32(ALX_ISR, 0);
}

bool AtherosE2200::checkForDeadlock()
{
    bool deadlock = false;
    
    if ((txDescDoneCount == txDescDoneLast) && (txNumFreeDesc < kNumTxDesc)) {
        if (++deadlockWarn >= kTxDeadlockTreshhold) {
#ifdef DEBUG
            UInt16 i, index;
            UInt16 stalledIndex = alxReadMem16(ALX_TPD_PRI0_CIDX);

            for (i = 0; i < 10; i++) {
                index = ((stalledIndex - 4 + i) & kTxDescMask);
                IOLog("desc[%u]: lenght=0x%x, vlanTag=0x%x, word1=0x%x, addr=0x%llx.\n", index, txDescArray[index].length, txDescArray[index].vlanTag, txDescArray[index].word1, txDescArray[index].adrl.addr);
            }
#endif
            IOLog("Tx stalled? Resetting chipset. ISR=0x%x, IMR=0x%x.\n", alxReadMem32(ALX_ISR), alxReadMem32(ALX_IMR));
            etherStats->dot3TxExtraEntry.resets++;
            alxRestart();
            deadlock = true;
        }
    } else {
        deadlockWarn = 0;
    }
    return deadlock;
}

#pragma mark --- link status change methods ---

/* AtherosE2200::setLinkUp()
 *
 * Establish link speed, duplex and flow control settings. Programm the MAC
 * according to the new settings and start receive and transmit. In case the
 * output queue was stalled, restart it too.
 */

void AtherosE2200::setLinkUp()
{
    UInt32 mediumIndex;

    alxActiveMediumIndex(&mediumIndex);
    
    intrMask = (ALX_ISR_MISC | ALX_ISR_PHY | ALX_ISR_RX_Q0 | ALX_ISR_TX_Q0);
    alxWriteMem32(ALX_IMR, intrMask);
    
    alxPostPhyLink();
    alx_enable_aspm(&hw, false, false);
    pciDevice->setASPMState(this, 0);

    /* Adjust MAC's speed, duplex and flow control settings. */
    alx_start_mac(&hw);
    alx_cfg_mac_flowcontrol(&hw, flowControl);
    
    linkUp = true;
    setLinkStatus((kIONetworkLinkValid | kIONetworkLinkActive), mediumTable[mediumIndex]);
    
    /* Update poll params according to link speed. */
    bzero(&pollParams, sizeof(IONetworkPacketPollingParameters));
    
    if (hw.link_speed == SPEED_10) {
        pollParams.lowThresholdPackets = 2;
        pollParams.highThresholdPackets = 8;
        pollParams.lowThresholdBytes = 0x400;
        pollParams.highThresholdBytes = 0x1800;
        pollParams.pollIntervalTime = 1000000;  /* 1ms */
    } else {
        pollParams.lowThresholdPackets = 10;
        pollParams.highThresholdPackets = 40;
        pollParams.lowThresholdBytes = 0x1000;
        pollParams.highThresholdBytes = 0x10000;
        pollParams.pollIntervalTime = (hw.link_speed == SPEED_1000) ? 170000 : 1000000;  /* 170µs / 1ms */
    }
    netif->setPacketPollingParameters(&pollParams, 0);

        /* Start output thread, statistics update and watchdog. */
    netif->startOutputThread();
    timerSource->setTimeoutMS(kTimeoutMS);

    IOLog("Link up on en%u, %s\n", netif->getUnitNumber(), mediumDescriptionArray[mediumIndex]);
}

/* AtherosE2200::setLinkDown()
 * 
 * Stop output queue, watchdog and statistics updates. Also reset the MAC, clear the
 * tx descriptor ring and reinitialize the MAC.
 */

void AtherosE2200::setLinkDown()
{
    timerSource->cancelTimeout();

    deadlockWarn = 0;
    
    /* Stop output thread and flush output queue. */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    /* Update link status. */
    linkUp = false;
    setLinkStatus(kIONetworkLinkValid);
    
    alx_reset_mac(&hw);
    
    intrMask = (ALX_ISR_MISC | ALX_ISR_PHY);
    alxWriteMem32(ALX_IMR, intrMask);

    /* Cleanup transmitter ring. */
    clearDescriptors();

    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;

    /* MAC reset causes all HW settings to be lost, restore all */
    alxConfigure();
    alx_enable_aspm(&hw, false, true);
    pciDevice->setASPMState(this, kIOPCIELinkCtlL1);

    alxPostPhyLink();
    
    IOLog("Link down on en%u\n", netif->getUnitNumber());
}

#pragma mark --- hardware initialization methods ---

bool AtherosE2200::alxStart(UInt32 maxIntrRate)
{
    int error;
    bool result = false;
    bool phyConfigured;

    if (maxIntrRate < 2500)
        maxIntrRate = 2500;
    else if (maxIntrRate > 10000)
        maxIntrRate = 10000;

    maxIntrRate = (1000000 / maxIntrRate);

    hw.lnk_patch = ((pciDeviceData.device == ALX_DEV_ID_AR8161) && (pciDeviceData.subsystem_vendor == 0x1969) && (pciDeviceData.subsystem_device == 0x0091) && (pciDeviceData.revision == 0));
    
    hw.smb_timer = 400;
    hw.mtu = ETHERMTU;
    hw.sleep_ctrl = 0;
    hw.imt = (UInt16)maxIntrRate;
    intrMask = (ALX_ISR_MISC | ALX_ISR_PHY);
    hw.dma_chnl = hw.max_dma_chnl;
    hw.ith_tpd = 192;
    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;
    hw.adv_cfg = (ADVERTISED_Autoneg | ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half | ADVERTISED_1000baseT_Full);
    hw.flowctrl = (ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX);
    
    hw.rx_ctrl = (ALX_MAC_CTRL_WOLSPED_SWEN | ALX_MAC_CTRL_BRD_EN | ALX_MAC_CTRL_RX_XSUM_EN | ALX_MAC_CTRL_VLANSTRIP | ALX_MAC_CTRL_MHASH_ALG_HI5B | ALX_MAC_CTRL_PCRCE | ALX_MAC_CTRL_CRCE | ALX_MAC_CTRL_RXFC_EN | ALX_MAC_CTRL_TXFC_EN | (7 << ALX_MAC_CTRL_PRMBLEN_SHIFT));
    
    eeeAdv = eeeCap;

    if (!alxResetPCIe())
        goto done;
    
    phyConfigured = alx_phy_configured(&hw);
    
    if (!phyConfigured)
        alxResetPhy();
    
    if (alx_reset_mac(&hw)) {
        IOLog("Failed to reset MAC.\n");
        //goto done;
    }
    /* setup link to put it in a known good starting state */
    if (!phyConfigured) {
        error = alxSetupSpeedDuplex(hw.adv_cfg, eeeAdv, hw.flowctrl);
        
        if (error) {
            IOLog("Failed to configure PHY speed/duplex: %d.\n", error);
            goto done;
        }
    }
    if (!alxLoadDefaultAddress()) {
        IOLog("Failed to get permanent MAC address.\n");
        goto done;
    }
    hw.mdio.prtad = 0;
    hw.mdio.mmds = 0;
    hw.mdio.dev = NULL;
    hw.mdio.mode_support = (MDIO_SUPPORTS_C45 | MDIO_SUPPORTS_C22 | MDIO_EMULATE_C22);
    hw.mdio.mdio_read = NULL;
    hw.mdio.mdio_write = NULL;
    
    if (!alx_get_phy_info(&hw)) {
        IOLog("Failed to identify PHY.\n");
        goto done;
    }
    IOLog("%s: (Rev. %u) at 0x%p, %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
          chipNames[chip], pciDeviceData.revision, baseAddr,
          origMacAddr.bytes[0], origMacAddr.bytes[1],
          origMacAddr.bytes[2], origMacAddr.bytes[3],
          origMacAddr.bytes[4], origMacAddr.bytes[5]);
    result = true;
    
done:
    return result;
}

#pragma mark --- timer action methods ---

void AtherosE2200::timerAction(IOTimerEventSource *timer)
{
    UInt32 lpi;
    
    if (!linkUp) {
        DebugLog("Timer fired while link down.\n");
        goto done;
    }
    /* Check for tx deadlock. */
    if (checkForDeadlock())
        goto done;
    
    updateStatitics();
    timerSource->setTimeoutMS(kTimeoutMS);

    if (eeeEnable) {
        eeeEnable = 0;
        lpi = alxReadMem32(ALX_LPI_CTRL);
        lpi |= ALX_LPI_CTRL_EN;
        alxWriteMem32(ALX_LPI_CTRL, lpi);
        DebugLog("Enable LPI: ALX_LPI_CTRL=0x%08x.\n", lpi);
    }
done:
    txDescDoneLast = txDescDoneCount;
    
    //DebugLog("timerAction() <===\n");
}

void AtherosE2200::updateStatitics()
{
    alx_update_hw_stats(&hw);
    
    netStats->inputPackets = (UInt32)hw.stats.rx_ok;
    netStats->inputErrors = (UInt32)(hw.stats.rx_frag + hw.stats.rx_fcs_err + hw.stats.rx_len_err + hw.stats.rx_ov_sz + hw.stats.rx_ov_rrd + hw.stats.rx_align_err + hw.stats.rx_ov_rxf);
    netStats->outputPackets = (UInt32)hw.stats.tx_ok;
    netStats->outputErrors = (UInt32)(hw.stats.tx_late_col + hw.stats.tx_abort_col + hw.stats.tx_underrun + hw.stats.tx_trunc);
    
    netStats->collisions = (UInt32)(hw.stats.tx_single_col + hw.stats.tx_multi_col + hw.stats.tx_late_col + hw.stats.tx_abort_col);
    
    etherStats->dot3StatsEntry.singleCollisionFrames = (UInt32)(hw.stats.tx_single_col);
    etherStats->dot3StatsEntry.multipleCollisionFrames = (UInt32)hw.stats.tx_multi_col;
    etherStats->dot3StatsEntry.alignmentErrors = (UInt32)hw.stats.rx_align_err;
    etherStats->dot3StatsEntry.missedFrames = (UInt32)(hw.stats.rx_ov_rrd + hw.stats.rx_ov_rrd);
    etherStats->dot3TxExtraEntry.underruns = (UInt32)hw.stats.tx_underrun;
}

#pragma mark --- miscellaneous functions ---

static inline u32 ether_crc(int length, unsigned char *data)
{
    int crc = -1;
    
    while(--length >= 0) {
        unsigned char current_octet = *data++;
        int bit;
        for (bit = 0; bit < 8; bit++, current_octet >>= 1) {
            crc = (crc << 1) ^
            ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
        }
    }
    return crc;
}

static inline void adjustIPv4Header(mbuf_t m)
{
    struct iphdr *ipHdr;
    struct tcphdr *tcpHdr;
    int i = (int)mbuf_len(m);
    UInt32 plen;
    UInt32 csum;

    /*
     * VLAN packets have the L3 and L4 headers in the second mbuf.
     * The first one carries only the L2 header.
     */
    if (i == ETH_HLEN)
        ipHdr = (struct iphdr *)((UInt8 *)mbuf_data(mbuf_next(m)) + ETHER_HDR_LEN);
    else
        ipHdr = (struct iphdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);

    tcpHdr = (struct tcphdr *)((UInt8 *)ipHdr + sizeof(struct iphdr));
    plen = ntohs(ipHdr->tot_len) - sizeof(struct iphdr);
    csum = ntohs(tcpHdr->th_sum) - plen;
    csum += (csum >> 16);
    tcpHdr->th_sum = htons((UInt16)csum);
}

static inline UInt32 adjustIPv6Header(mbuf_t m)
{
    struct ip6_hdr *ip6Hdr;
    struct tcphdr *tcpHdr;
    int i = (int)mbuf_len(m);
    UInt32 plen;
    UInt32 csum;
    
    /*
     * VLAN packets have the L3 and L4 headers in the second mbuf.
     * The first one carries only the L2 header.
     */
    if (i == ETH_HLEN)
        ip6Hdr = (struct ip6_hdr *)((UInt8 *)mbuf_data(mbuf_next(m)) + ETHER_HDR_LEN);
    else
        ip6Hdr = (struct ip6_hdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);

    tcpHdr = (struct tcphdr *)((UInt8 *)ip6Hdr + sizeof(struct ip6_hdr));
    plen = ntohs(ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen);
    csum = ntohs(tcpHdr->th_sum) - plen;
    
    csum += (csum >> 16);
    ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen = 0;
    tcpHdr->th_sum = htons((UInt16)csum);

    return (plen + kMinL4HdrOffsetV6);
}

