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
};

static const char *onName = "enabled";
static const char *offName = "disabled";

/* Power Management Support */
static IOPMPowerState powerStateArray[kPowerStateCount] =
{
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

static IOMediumType mediumTypeArray[MEDIUM_INDEX_COUNT] = {
    kIOMediumEthernetAuto,
    (kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex),
    (kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionEEE),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl | kIOMediumOptionEEE),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionEEE),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl | kIOMediumOptionEEE),

};

static UInt32 mediumSpeedArray[MEDIUM_INDEX_COUNT] = {
    0,
    10 * MBit,
    10 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit,
    1000 * MBit,
    1000 * MBit,
    1000 * MBit,
    1000 * MBit
};

static const char *speed1GName = "1-Gigabit";
static const char *speed100MName = "100-Megabit";
static const char *speed10MName = "10-Megabit";
static const char *duplexFullName = "full-duplex";
static const char *duplexHalfName = "half-duplex";

static const char *flowControlNames[] = {
    "no flow-control",
    "Rx/Tx flow-control",
};

static const char* eeeNames[kEEETypeCount] = {
    "",
    ", energy-efficient-ethernet"
};

static unsigned const ethernet_polynomial = 0x04c11db7U;

#ifdef CONFIG_RSS

static const UInt8 rssKey[40] = {
    0xE2, 0x91, 0xD7, 0x3D, 0x18, 0x05, 0xEC, 0x6C,
    0x2A, 0x94, 0xB3, 0x0D, 0xA5, 0x4F, 0x2B, 0xEC,
    0xEA, 0x49, 0xAF, 0x7C, 0xE2, 0x14, 0xAD, 0x3D,
    0xB8, 0x55, 0xAA, 0xBE, 0x6A, 0x3E, 0x67, 0xEA,
    0x14, 0x36, 0x4D, 0x17, 0x3B, 0xED, 0x20, 0x0D
};

#endif  /* CONFIG_RSS */

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
        rxMbufCursor = NULL;
        txMbufCursor = NULL;
        isEnabled = false;
        promiscusMode = false;
        multicastMode = false;
        linkUp = false;
        
#ifdef __PRIVATE_SPI__
        linkOpts = 0;
        polling = false;
        rxPoll = false;
#else
        stalled = false;
#endif /* __PRIVATE_SPI__ */
        
        useMSI = false;
        chip = kChipUnkown;
        eeeCap = 0;
        powerState = 0;
        pciDeviceData.vendor = 0;
        pciDeviceData.device = 0;
        pciDeviceData.subsystem_vendor = 0;
        pciDeviceData.subsystem_device = 0;
        pciDeviceData.revision = 0;
        hw.pdev = &pciDeviceData;
        
#ifndef __PRIVATE_SPI__
        txStallCount = 0;
        txStallLast = 0;
#endif /* __PRIVATE_SPI__ */

        wolCapable = false;
        gbCapable = false;
        enableTSO4 = false;
        enableTSO6 = false;
        enableCSO6 = false;
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
    freeDMADescriptors();
    
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
        IOLog("Ethernet [AtherosE2200]: IOEthernetController::start failed.\n");
        goto done;
    }
    multicastMode = false;
    promiscusMode = false;
    multicastFilter[0] = multicastFilter[1] = 0;
    
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    
    if (!pciDevice) {
        IOLog("Ethernet [AtherosE2200]: No provider.\n");
        goto done;
    }
    pciDevice->retain();
    
    if (!pciDevice->open(this)) {
        IOLog("Ethernet [AtherosE2200]: Failed to open provider.\n");
        goto error1;
    }
    if (!initPCIConfigSpace(pciDevice)) {
        goto error2;
    }
    /* Get the config values from Info.plist. */
    getParams(&newIntrRate);
    
    if (!alxStart(newIntrRate)) {
        goto error2;
    }
    versionString = OSDynamicCast(OSString, getProperty(kDriverVersionName));
    newIntrRate = 1000000 / hw.imt;
    
    if (versionString)
        IOLog("Ethernet [AtherosE2200]: Version %s using max interrupt rate %u. Please don't support tonymacx86.com!\n", versionString->getCStringNoCopy(), newIntrRate);
    else
        IOLog("Ethernet [AtherosE2200]: Using max interrupt rate %u. Please don't support tonymacx86.com!\n", newIntrRate);

    if (!setupMediumDict()) {
        IOLog("Ethernet [AtherosE2200]: Failed to setup medium dictionary.\n");
        goto error2;
    }
    commandGate = getCommandGate();
    
    if (!commandGate) {
        IOLog("Ethernet [AtherosE2200]: getCommandGate() failed.\n");
        goto error3;
    }
    commandGate->retain();
    
    if (!initEventSources(provider)) {
        IOLog("Ethernet [AtherosE2200]: initEventSources() failed.\n");
        goto error3;
    }
    
    result = attachInterface(reinterpret_cast<IONetworkInterface**>(&netif));
    
    if (!result) {
        IOLog("Ethernet [AtherosE2200]: attachInterface() failed.\n");
        goto error3;
    }
    pciDevice->close(this);
    result = true;
    
done:
    return result;
    
error3:
    RELEASE(commandGate);
    
error2:
    pciDevice->close(this);
    
error1:
    pciDevice->release();
    pciDevice = NULL;
    goto done;
}

void AtherosE2200::stop(IOService *provider)
{
    UInt32 i;
    
    if (netif) {
        detachInterface(netif);
        netif = NULL;
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
    
    freeDMADescriptors();
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
        DebugLog("Ethernet [AtherosE2200]: Already in power state %lu.\n", powerStateOrdinal);
        goto done;
    }
    DebugLog("Ethernet [AtherosE2200]: switching to power state %lu.\n", powerStateOrdinal);
    
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
        DebugLog("Ethernet [AtherosE2200]: Interface already enabled.\n");
        result = kIOReturnSuccess;
        goto done;
    }
    if (!pciDevice || pciDevice->isOpen()) {
        IOLog("Ethernet [AtherosE2200]: Unable to open PCI device.\n");
        goto done;
    }
    pciDevice->open(this);
    
    if (!setupDMADescriptors()) {
        IOLog("Ethernet [AtherosE2200]: Error allocating DMA descriptors.\n");
        goto done;
    }
    alxEnable();
    
    /* In case we are using an msi the interrupt hasn't been enabled by start(). */
    if (useMSI)
        interruptSource->enable();
    
    txDescDoneCount = txDescDoneLast = 0;
    deadlockWarn = 0;

#ifdef __PRIVATE_SPI__
    polling = false;
#else
    txQueue->setCapacity(kTransmitQueueCapacity);
    txStallCount = txStallLast = 0;
    stalled = false;
#endif /* __PRIVATE_SPI__ */
    
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
    
#ifdef __PRIVATE_SPI__
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    polling = false;
#else
    txQueue->stop();
    txQueue->flush();
    txQueue->setCapacity(0);
    txStallCount = txStallLast = 0;
    stalled = false;
#endif /* __PRIVATE_SPI__ */

    timerSource->cancelTimeout();
    txDescDoneCount = txDescDoneLast = 0;
    multicastFilter[0] = multicastFilter[1] = 0;
    isEnabled = false;

    /* In case we are using msi disable the interrupt. */
    if (useMSI)
        interruptSource->disable();
    
    alxDisable();
    txClearDescriptors();
    
    if (pciDevice && pciDevice->isOpen())
        pciDevice->close(this);
    
    freeDMADescriptors();
    
    DebugLog("disable() <===\n");
    
done:
    return result;
}

#ifdef __PRIVATE_SPI__

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
        DebugLog("Ethernet [AtherosE2200]: Interface down. Dropping packets.\n");
        goto done;
    }
    while ((txNumFreeDesc > (kMaxSegs + 3)) && (interface->dequeueOutputPackets(1, &m, NULL, NULL, NULL) == kIOReturnSuccess)) {
        numDescs = 0;
        cmd = 0;
        totalLen = 0;

        if (mbuf_get_tso_requested(m, &tsoFlags, &mssValue)) {
            DebugLog("Ethernet [AtherosE2200]: mbuf_get_tso_requested() failed. Dropping packet.\n");
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
        numSegs = txMbufCursor->getPhysicalSegmentsWithCoalesce(m, &txSegments[0], kMaxSegs);
        numDescs += numSegs;
        
        if (!numSegs) {
            DebugLog("Ethernet [AtherosE2200]: getPhysicalSegmentsWithCoalesce() failed. Dropping packet.\n");
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
            desc->vlanTag = OSSwapHostToLittleInt16(vlanTag);
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

#else

UInt32 AtherosE2200::outputPacket(mbuf_t m, void *param)
{
    IOPhysicalSegment txSegments[kMaxSegs];
    QCATxDesc *desc;
    UInt32 result = kIOReturnOutputDropped;
    UInt32 numDescs = 0;
    UInt32 cmd = 0;
    UInt32 totalLen = 0;
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
    
    //DebugLog("outputPacket() ===>\n");

    if (!(isEnabled && linkUp)) {
        DebugLog("Ethernet [AtherosE2200]: Interface down. Dropping packet.\n");
        goto error;
    }
    if (mbuf_get_tso_requested(m, &tsoFlags, &mssValue)) {
        DebugLog("Ethernet [AtherosE2200]: mbuf_get_tso_requested() failed. Dropping packet.\n");
        goto done;
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
    numSegs = txMbufCursor->getPhysicalSegmentsWithCoalesce(m, &txSegments[0], kMaxSegs);
    numDescs += numSegs;
    
    if (!numSegs) {
        DebugLog("Ethernet [AtherosE2200]: getPhysicalSegmentsWithCoalesce() failed. Dropping packet.\n");
        etherStats->dot3TxExtraEntry.resourceErrors++;
        goto error;
    }
    /* Alloc required number of descriptors. We leave at least two unused. */
    if ((txNumFreeDesc <= (numDescs + 1))) {
        DebugLog("Ethernet [AtherosE2200]: Not enough descriptors. Stalling.\n");
        result = kIOReturnOutputStall;
        stalled = true;
        txStallCount++;
        goto done;
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
        desc->vlanTag = OSSwapHostToLittleInt16(vlanTag);
        desc->length = OSSwapHostToLittleInt16(segLen);
        desc->word1 = OSSwapHostToLittleInt32(word1);
        desc->adrl.addr = OSSwapHostToLittleInt64(txSegments[i].location);
        
        ++index &= kTxDescMask;
    }
	/* flush updates before updating hardware */
	OSSynchronizeIO();
	alxWriteMem16(ALX_TPD_PRI0_PIDX, txNextDescIndex);
    
    result = kIOReturnOutputSuccess;

done:
    //DebugLog("outputPacket() <===\n");
    
    return result;
    
error:
    freePacket(m);
    goto done;
}

#endif /* __PRIVATE_SPI__ */

void AtherosE2200::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
    DebugLog("getPacketBufferConstraints() ===>\n");
    
	constraints->alignStart = kIOPacketBufferAlign8;
	constraints->alignLength = kIOPacketBufferAlign8;
    
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
    
#ifdef __PRIVATE_SPI__
    IOReturn error;
#endif /* __PRIVATE_SPI__ */

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
            IOLog("Ethernet [AtherosE2200]: Error getting IONetworkStats\n.");
            result = false;
            goto done;
        }
    }
    /* Get the Ethernet statistics structure. */
    data = interface->getParameter(kIOEthernetStatsKey);
    
    if (data) {
        etherStats = (IOEthernetStats *)data->getBuffer();
        
        if (!etherStats) {
            IOLog("Ethernet [AtherosE2200]: Error getting IOEthernetStats\n.");
            result = false;
            goto done;
        }
    }
    
#ifdef __PRIVATE_SPI__
    error = interface->configureOutputPullModel(512, 0, 0, IONetworkInterface::kOutputPacketSchedulingModelNormal);
    
    if (error != kIOReturnSuccess) {
        IOLog("Ethernet [AtherosE2200]: configureOutputPullModel() failed\n.");
        result = false;
        goto done;
    }
    if (rxPoll) {
        error = interface->configureInputPacketPolling(kNumRxDesc, kIONetworkWorkLoopSynchronous);
        
        if (error != kIOReturnSuccess) {
            IOLog("Ethernet [AtherosE2200]: configureInputPacketPolling() failed\n.");
            result = false;
            goto done;
        }
    }

#endif /* __PRIVATE_SPI__ */

    if ((chip == kChipAR8162) || (chip == kChipAR8172))
        snprintf(modelName, kNameLenght, "Qualcomm Atheros %s PCI Express Fast Ethernet", chipNames[chip]);
    else
        snprintf(modelName, kNameLenght, "Qualcomm Atheros %s PCI Express Gigabit Ethernet", chipNames[chip]);
    
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
        DebugLog("Ethernet [AtherosE2200]: Promiscuous mode enabled.\n");
        hw.rx_ctrl |= ALX_MAC_CTRL_PROMISC_EN;
        mcFilter[1] = mcFilter[0] = 0xffffffff;
    } else {
        DebugLog("Ethernet [AtherosE2200]: Promiscuous mode disabled.\n");
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
        hw.sleep_ctrl = active ? (ALX_SLEEP_WOL_MAGIC) : 0;
        DebugLog("Ethernet [AtherosE2200]: Wake on magic packet %s.\n", active ? "enabled" : "disabled");
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
        DebugLog("Ethernet [AtherosE2200]: kIOEthernetWakeOnMagicPacket added to filters.\n");
    } else {
        result = super::getPacketFilters(group, filters);
    }
    
    DebugLog("getPacketFilters() <===\n");
    
    return result;
}

/* There seems to be a hardware bug which prevents the Killer and AR816x/AR817x
 * NICs from changing their MAC address. Writes to the MAC adddress registers
 * result in a messed up address register effectively killing unicast packet
 * reception until the next reset. As a workaround, we acknowledge the request
 * without performing any action as refusing it would break LACP.
 */
IOReturn AtherosE2200::setHardwareAddress(const IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("setHardwareAddress() ===>\n");
/*
    if (addr && ether_addr_equal(&addr->bytes[0], &origMacAddr.bytes[0])) {
        memcpy(&currMacAddr.bytes[0], &addr->bytes[0], kIOEthernetAddressSize);
        alxSetHardwareAddress(addr);

        result = kIOReturnSuccess;
    }
*/
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

#pragma mark --- rx poll methods ---

#ifdef __PRIVATE_SPI__

IOReturn AtherosE2200::setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled)
{
    //DebugLog("setInputPacketPollingEnable() ===>\n");
    
    if (enabled) {
        intrMask = (ALX_ISR_MISC | ALX_ISR_PHY);
        polling = true;
    } else {
        intrMask = (ALX_ISR_MISC | ALX_ISR_PHY | ALX_ISR_RX_Q0 | ALX_ISR_TX_Q0);
        polling = false;
    }
    if(isEnabled)
        alxWriteMem32(ALX_IMR, intrMask);
    
    //DebugLog("input polling %s.\n", enabled ? "enabled" : "disabled");
    
    //DebugLog("setInputPacketPollingEnable() <===\n");
    
    return kIOReturnSuccess;
}

void AtherosE2200::pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context )
{
    //DebugLog("pollInputPackets() ===>\n");
    
    rxInterrupt(interface, maxCount, pollQueue, context);
    
    /* Finally cleanup the transmitter ring. */
    txInterrupt();
    
    //DebugLog("pollInputPackets() <===\n");
}

#endif /* __PRIVATE_SPI__ */

#pragma mark --- data structure initialization methods ---

bool AtherosE2200::setupMediumDict()
{
	IONetworkMedium *medium;
    UInt32 count = gbCapable ? MEDIUM_INDEX_COUNT : (MEDIUM_INDEX_COUNT - 2);
    UInt32 i;
    bool result = false;
    
    mediumDict = OSDictionary::withCapacity(count + 1);
    
    if (mediumDict) {
        for (i = MEDIUM_INDEX_AUTO; i < count; i++) {
            medium = IONetworkMedium::medium(mediumTypeArray[i], mediumSpeedArray[i], 0, i);
            
            if (!medium)
                goto error1;
            
            result = IONetworkMedium::addMedium(mediumDict, medium);
            medium->release();
            
            if (!result)
                goto error1;
            
            mediumTable[i] = medium;
        }
    }
    result = publishMediumDictionary(mediumDict);
    
    if (!result)
        goto error1;
    
done:
    return result;
    
error1:
    IOLog("Ethernet [AtherosE2200]: Error creating medium dictionary.\n");
    mediumDict->release();
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    goto done;
}

bool AtherosE2200::initEventSources(IOService *provider)
{
    IOReturn intrResult;
    int msiIndex = -1;
    int intrIndex = 0;
    int intrType = 0;
    bool result = false;
    
    txQueue = reinterpret_cast<IOBasicOutputQueue *>(getOutputQueue());
    
    if (txQueue == NULL) {
        IOLog("Ethernet [AtherosE2200]: Failed to get output queue.\n");
        goto done;
    }
    txQueue->retain();
    
    while ((intrResult = pciDevice->getInterruptType(intrIndex, &intrType)) == kIOReturnSuccess) {
        if (intrType & kIOInterruptTypePCIMessaged){
            msiIndex = intrIndex;
            break;
        }
        intrIndex++;
    }
    if (msiIndex != -1) {
        DebugLog("Ethernet [AtherosE2200]: MSI interrupt index: %d\n", msiIndex);
        
        interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosE2200::interruptOccurred), provider, msiIndex);
    }
    if (!interruptSource) {
        DebugLog("Ethernet [AtherosE2200]: Warning: MSI index was not found or MSI interrupt could not be enabled.\n");
        
        interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosE2200::interruptOccurred), provider);
        
        useMSI = false;
    } else {
        useMSI = true;
    }
    if (!interruptSource)
        goto error1;
    
    workLoop->addEventSource(interruptSource);
    
    /*
     * This is important. If the interrupt line is shared with other devices,
	 * then the interrupt vector will be enabled only if all corresponding
	 * interrupt event sources are enabled. To avoid masking interrupts for
	 * other devices that are sharing the interrupt line, the event source
	 * is enabled immediately.
     */
    if (!useMSI)
        interruptSource->enable();
    
    timerSource = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &AtherosE2200::timerAction));
    
    if (!timerSource) {
        IOLog("Ethernet [AtherosE2200]: Failed to create IOTimerEventSource.\n");
        goto error2;
    }
    workLoop->addEventSource(timerSource);
    
    result = true;
    
done:
    return result;
    
error2:
    workLoop->removeEventSource(interruptSource);
    RELEASE(interruptSource);
    
error1:
    IOLog("Ethernet [AtherosE2200]: Error initializing event sources.\n");
    txQueue->release();
    txQueue = NULL;
    goto done;
}

bool AtherosE2200::setupDMADescriptors()
{
    mbuf_t spareMbuf[kRxNumSpareMbufs];
    mbuf_t m;
    IOPhysicalSegment rxSegment;
    QCARxTxDescArray *descArray;
    UInt32 i;
    //UInt32 opts1;
    bool result = false;
    
    /* Create descriptor arrays. */
    bufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), sizeof(QCARxTxDescArray), 0xFFFFFFFFFFFFFF00ULL);
    
    if (!bufDesc) {
        IOLog("Ethernet [AtherosE2200]: Couldn't alloc bufDesc.\n");
        goto done;
    }
    if (bufDesc->prepare() != kIOReturnSuccess) {
        IOLog("Ethernet [AtherosE2200]: bufDesc->prepare() failed.\n");
        goto error1;
    }
    descArray = (QCARxTxDescArray *)bufDesc->getBytesNoCopy();
    txDescArray = &descArray->txDesc[0];
    txPhyAddr = bufDesc->getPhysicalAddress();
    
    /* Initialize txDescArray. */
    bzero(&descArray->txDesc[0], kTxDescSize);
    
    for (i = 0; i < kNumTxDesc; i++) {
        txMbufArray[i] = NULL;
    }
    txNextDescIndex = txDirtyDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(0x4000, kMaxSegs);
    
    if (!txMbufCursor) {
        IOLog("Ethernet [AtherosE2200]: Couldn't create txMbufCursor.\n");
        goto error2;
    }
    
    /* Setup receiver return descriptor array. */
    rxRetDescArray = &descArray->rxRetDesc[0];
    rxRetPhyAddr = txPhyAddr + offsetof(QCARxTxDescArray, rxRetDesc);
    
    /* Initialize rxRetDescArray. */
    bzero(rxRetDescArray, kRxRetDescSize);

    /* Setup receiver free descriptor array. */
    rxFreeDescArray = &descArray->rxFreeDesc[0];
    rxFreePhyAddr = txPhyAddr + offsetof(QCARxTxDescArray, rxFreeDesc);
    
    /* Initialize rxFreeDescArray. */
    bzero(rxFreeDescArray, kRxFreeDescSize);
    
    for (i = 0; i < kNumRxDesc; i++) {
        rxMbufArray[i] = NULL;
    }
    rxNextDescIndex = 0;
    
    rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, 1);
    
    if (!rxMbufCursor) {
        IOLog("Ethernet [AtherosE2200]: Couldn't create rxMbufCursor.\n");
        goto error3;
    }
    /* Alloc receive buffers. */
    for (i = 0; i < kNumRxDesc; i++) {
        m = allocatePacket(kRxBufferPktSize);
        
        if (!m) {
            IOLog("Ethernet [AtherosE2200]: Couldn't alloc receive buffer.\n");
            goto error4;
        }
        rxMbufArray[i] = m;
        
        if (rxMbufCursor->getPhysicalSegments(m, &rxSegment, 1) != 1) {
            IOLog("Ethernet [AtherosE2200]: getPhysicalSegments() for receive buffer failed.\n");
            goto error4;
        }
        rxFreeDescArray[i].addr = OSSwapHostToLittleInt64(rxSegment.location);
    }
    /* Allocate some spare mbufs and free them in order to increase the buffer pool.
     * This seems to avoid the replaceOrCopyPacket() errors under heavy load.
     */
    for (i = 0; i < kRxNumSpareMbufs; i++)
        spareMbuf[i] = allocatePacket(kRxBufferPktSize);
    
    for (i = 0; i < kRxNumSpareMbufs; i++) {
        if (spareMbuf[i])
            freePacket(spareMbuf[i]);
    }
    result = true;
    
done:
    return result;
    
error4:
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxMbufArray[i]) {
            freePacket(rxMbufArray[i]);
            rxMbufArray[i] = NULL;
        }
    }
    RELEASE(rxMbufCursor);
    
error3:
    RELEASE(txMbufCursor);
    
error2:
    bufDesc->complete();
    
error1:
    bufDesc->release();
    bufDesc = NULL;
    goto done;
}

void AtherosE2200::freeDMADescriptors()
{
    UInt32 i;
    
    if (bufDesc) {
        bufDesc->complete();
        bufDesc->release();
        bufDesc = NULL;
        txPhyAddr = NULL;
    }
    RELEASE(txMbufCursor);
    RELEASE(rxMbufCursor);
    
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxMbufArray[i]) {
            freePacket(rxMbufArray[i]);
            rxMbufArray[i] = NULL;
        }
    }
}

void AtherosE2200::txClearDescriptors()
{
    mbuf_t m;
    UInt32 i;
    
    DebugLog("txClearDescriptors() ===>\n");
    
    for (i = 0; i < kNumTxDesc; i++) {
        m = txMbufArray[i];
        
        if (m) {
            freePacket(m);
            txMbufArray[i] = NULL;
        }
    }
    txDirtyDescIndex = txNextDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    
    DebugLog("txClearDescriptors() <===\n");
}

void AtherosE2200::getParams(UInt32 *intrLimit)
{
    OSNumber *intrRate;
    OSBoolean *tso4;
    OSBoolean *tso6;
    OSBoolean *csoV6;
    
#ifdef __PRIVATE_SPI__
    OSBoolean *poll;
#endif /* __PRIVATE_SPI__ */

#ifdef __PRIVATE_SPI__
    poll = OSDynamicCast(OSBoolean, getProperty(kEnableRxPollName));
    rxPoll = (poll) ? poll->getValue() : false;
    
    IOLog("Ethernet [AtherosE2200]: RxPoll support %s.\n", rxPoll ? onName : offName);
#endif /* __PRIVATE_SPI__ */

    tso4 = OSDynamicCast(OSBoolean, getProperty(kEnableTSO4Name));
    enableTSO4 = (tso4) ? tso4->getValue() : false;
    
    IOLog("Ethernet [AtherosE2200]: TCP/IPv4 segmentation offload %s.\n", enableTSO4 ? onName : offName);
    
    tso6 = OSDynamicCast(OSBoolean, getProperty(kEnableTSO6Name));
    enableTSO6 = (tso6) ? tso6->getValue() : false;
    
    IOLog("Ethernet [AtherosE2200]: TCP/IPv6 segmentation offload %s.\n", enableTSO6 ? onName : offName);
    
    csoV6 = OSDynamicCast(OSBoolean, getProperty(kEnableCSO6Name));
    enableCSO6 = (csoV6) ? csoV6->getValue() : false;
    
    IOLog("Ethernet [AtherosE2200]: TCP/IPv6 checksum offload %s.\n", enableCSO6 ? onName : offName);
    
    intrRate = OSDynamicCast(OSNumber, getProperty(kIntrRateName));
    *intrLimit = 5000;
    
    if (intrRate)
        *intrLimit = intrRate->unsigned32BitValue();
}

#pragma mark --- common interrupt methods ---

void AtherosE2200::txInterrupt()
{
    UInt16 newDirtyIndex = alxReadMem16(ALX_TPD_PRI0_CIDX);
    
    //DebugLog("Ethernet [AtherosE2200]: txInterrupt oldIndex=%u newIndex=%u\n", txDirtyDescIndex, newDirtyIndex);

    if (txDirtyDescIndex != newDirtyIndex) {
        while (txDirtyDescIndex != newDirtyIndex) {
            if (txMbufArray[txDirtyDescIndex]) {
                freePacket(txMbufArray[txDirtyDescIndex], kDelayFree);
                txMbufArray[txDirtyDescIndex] = NULL;
            }
            txDescDoneCount++;
            OSIncrementAtomic(&txNumFreeDesc);
            ++txDirtyDescIndex &= kTxDescMask;
        }
        releaseFreePackets();
        
#ifdef __PRIVATE_SPI__
        if (txNumFreeDesc > kTxQueueWakeTreshhold)
            netif->signalOutputThread();
        
        if (!polling)
            etherStats->dot3TxExtraEntry.interrupts++;
#else
        if (stalled && (txNumFreeDesc > kTxQueueWakeTreshhold)) {
            DebugLog("Ethernet [AtherosE2200]: Restart stalled queue!\n");
            txQueue->service(IOBasicOutputQueue::kServiceAsync);
            stalled = false;
        }
        etherStats->dot3TxExtraEntry.interrupts++;
#endif /* __PRIVATE_SPI__ */

    }
}

#ifdef __PRIVATE_SPI__

UInt32 AtherosE2200::rxInterrupt(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    IOPhysicalSegment rxSegment;
    QCARxRetDesc *desc = &rxRetDescArray[rxNextDescIndex];
    mbuf_t bufPkt, newPkt;
    UInt32 status0, status2, status3;
    UInt32 pktSize;
    UInt32 validMask;
    UInt16 index, numBufs;
    UInt16 vlanTag;
    UInt16 goodPkts = 0;
    bool replaced;
    
    //DebugLog("Ethernet [AtherosE2200]: rxInterrupt()\n");
    
    while (((status3 = OSSwapLittleToHostInt32(desc->word3)) & RRD_UPDATED) && (goodPkts < maxCount)) {
        status0 = OSSwapLittleToHostInt32(desc->word0);
        status2 = OSSwapLittleToHostInt32(desc->word2);
        pktSize = (status3 & RRD_PKTLEN_MASK) - kIOEthernetCRCSize;
        index = (status0 >> RRD_SI_SHIFT) & RRD_SI_MASK;
        numBufs = (status0 >> RRD_NOR_SHIFT) & 0x000F;
        vlanTag = (status3 & RRD_VLTAGGED) ? OSSwapBigToHostInt16(status2 & RRD_VLTAG_MASK) : 0;
        bufPkt = rxMbufArray[index];
        
        //DebugLog("Ethernet [AtherosE2200]: Packet with index=%u, numBufs=%u, pktSize=%u, errors=0x%x\n", index, numBufs, pktSize, errors);
        
        /* As we don't support jumbo frames we consider fragmented packets as errors. */
        if (numBufs > 1) {
            DebugLog("Ethernet [AtherosE2200]: Fragmented packet.\n");
            etherStats->dot3StatsEntry.frameTooLongs++;
            index =  (index + (numBufs - 1)) & kRxDescMask;
            goto nextDesc;
        }
        /* Skip bad packet. */
        if (status3 & RRD_ERR_MASK) {
            DebugLog("Ethernet [AtherosE2200]: Bad packet.\n");
            etherStats->dot3StatsEntry.internalMacReceiveErrors++;
            goto nextDesc;
        }
        newPkt = replaceOrCopyPacket(&bufPkt, pktSize, &replaced);
        
        if (!newPkt) {
            /* Allocation of a new packet failed so that we must leave the original packet in place. */
            DebugLog("Ethernet [AtherosE2200]: replaceOrCopyPacket() failed.\n");
            etherStats->dot3RxExtraEntry.resourceErrors++;
            goto nextDesc;
        }
        /* If the packet was replaced we have to update the free descriptor's buffer address. */
        if (replaced) {
            if (rxMbufCursor->getPhysicalSegments(bufPkt, &rxSegment, 1) != 1) {
                DebugLog("Ethernet [AtherosE2200]: getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                freePacket(bufPkt);
                goto nextDesc;
            }
            rxMbufArray[index] = bufPkt;
            rxFreeDescArray[index].addr = OSSwapHostToLittleInt64(rxSegment.location);
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
        mbuf_setlen(newPkt, pktSize);
        interface->enqueueInputPacket(newPkt, pollQueue);
        goodPkts++;
        
        /* Finally update the descriptor and get the next one to examine. */
    nextDesc:
        desc->word3 = OSSwapHostToLittleInt32(status3 & ~RRD_UPDATED);
        
        ++rxNextDescIndex &= kRxDescMask;
        desc = &rxRetDescArray[rxNextDescIndex];
        
        alxWriteMem16(ALX_RFD_PIDX, index);
    }
    return goodPkts;
}

#else

void AtherosE2200::rxInterrupt()
{
    IOPhysicalSegment rxSegment;
    QCARxRetDesc *desc = &rxRetDescArray[rxNextDescIndex];
    mbuf_t bufPkt, newPkt;
    UInt32 status0, status2, status3;
    UInt32 pktSize;
    UInt32 validMask;
    UInt16 goodPkts = 0;
    UInt16 index, numBufs;
    UInt16 vlanTag;
    bool replaced;
    
    //DebugLog("Ethernet [AtherosE2200]: rxInterrupt()\n");

    while ((status3 = OSSwapLittleToHostInt32(desc->word3)) & RRD_UPDATED) {
        status0 = OSSwapLittleToHostInt32(desc->word0);
        status2 = OSSwapLittleToHostInt32(desc->word2);
        pktSize = (status3 & RRD_PKTLEN_MASK) - kIOEthernetCRCSize;
        index = (status0 >> RRD_SI_SHIFT) & RRD_SI_MASK;
        numBufs = (status0 >> RRD_NOR_SHIFT) & 0x000F;
        vlanTag = (status3 & RRD_VLTAGGED) ? OSSwapBigToHostInt16(status2 & RRD_VLTAG_MASK) : 0;
        bufPkt = rxMbufArray[index];

        //DebugLog("Ethernet [AtherosE2200]: Packet with index=%u, numBufs=%u, pktSize=%u, errors=0x%x\n", index, numBufs, pktSize, errors);

        /* As we don't support jumbo frames we consider fragmented packets as errors. */
        if (numBufs > 1) {
            DebugLog("Ethernet [AtherosE2200]: Fragmented packet.\n");
            etherStats->dot3StatsEntry.frameTooLongs++;
            index =  (index + (numBufs - 1)) & kRxDescMask;
            goto nextDesc;
        }
        /* Skip bad packet. */
        if (status3 & RRD_ERR_MASK) {
            DebugLog("Ethernet [AtherosE2200]: Bad packet.\n");
            etherStats->dot3StatsEntry.internalMacReceiveErrors++;
            goto nextDesc;
        }
        newPkt = replaceOrCopyPacket(&bufPkt, pktSize, &replaced);
        
        if (!newPkt) {
            /* Allocation of a new packet failed so that we must leave the original packet in place. */
            DebugLog("Ethernet [AtherosE2200]: replaceOrCopyPacket() failed.\n");
            etherStats->dot3RxExtraEntry.resourceErrors++;
            goto nextDesc;
        }
        /* If the packet was replaced we have to update the free descriptor's buffer address. */
        if (replaced) {
            if (rxMbufCursor->getPhysicalSegments(bufPkt, &rxSegment, 1) != 1) {
                DebugLog("Ethernet [AtherosE2200]: getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                freePacket(bufPkt);
                goto nextDesc;
            }
            rxMbufArray[index] = bufPkt;
            rxFreeDescArray[index].addr = OSSwapHostToLittleInt64(rxSegment.location);
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
        
        netif->inputPacket(newPkt, pktSize, IONetworkInterface::kInputOptionQueuePacket);
        goodPkts++;
        
        /* Finally update the descriptor and get the next one to examine. */
nextDesc:
        desc->word3 = OSSwapHostToLittleInt32(status3 & ~RRD_UPDATED);
        
        ++rxNextDescIndex &= kRxDescMask;
        desc = &rxRetDescArray[rxNextDescIndex];
        
        alxWriteMem16(ALX_RFD_PIDX, index);
    }
    if (goodPkts) {
        //DebugLog("Ethernet [AtherosE2200]: Received %u good packets.\n", goodPkts);
        netif->flushInputQueue();
    }
    //etherStats->dot3RxExtraEntry.interrupts++;
}

#endif /* __PRIVATE_SPI__ */

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
        IOLog("Ethernet [AtherosE2200]: Fatal interrupt. Reseting chip. ISR=0x%x\n", status);
        etherStats->dot3TxExtraEntry.resets++;
		alxRestart();
		return;
	}
	if (status & ALX_ISR_ALERT)
        IOLog("Ethernet [AtherosE2200]: Alert interrupt. ISR=0x%x\n", status);

#ifdef __PRIVATE_SPI__
    if (!polling) {
        if (status & ALX_ISR_TX_Q0)
            txInterrupt();
        
        if (status & ALX_ISR_RX_Q0) {
            packets = rxInterrupt(netif, kNumRxDesc, NULL, NULL);
            
            if (packets)
                netif->flushInputQueue();
        }
    }
#else
    if (status & ALX_ISR_TX_Q0)
        txInterrupt();
    
    if (status & ALX_ISR_RX_Q0)
        rxInterrupt();
#endif /* __PRIVATE_SPI__ */

	if (status & ALX_ISR_PHY)
        checkLinkStatus();
	   
done:
    alxWriteMem32(ALX_ISR, 0);
}

bool AtherosE2200::checkForDeadlock()
{
    bool deadlock = false;
    
#ifdef __PRIVATE_SPI__
    if ((txDescDoneCount == txDescDoneLast) && (txNumFreeDesc < kNumTxDesc))
#else
    if (((txDescDoneCount == txDescDoneLast) && (txNumFreeDesc < kNumTxDesc)) || (stalled && (txStallCount ==txStallLast)))
#endif /* __PRIVATE_SPI__ */
    {
        if (++deadlockWarn >= kTxDeadlockTreshhold) {
#ifdef DEBUG
            UInt16 i, index;
            UInt16 stalledIndex = alxReadMem16(ALX_TPD_PRI0_CIDX);

            for (i = 0; i < 10; i++) {
                index = ((stalledIndex - 4 + i) & kTxDescMask);
                IOLog("Ethernet [AtherosE2200]: desc[%u]: lenght=0x%x, vlanTag=0x%x, word1=0x%x, addr=0x%llx.\n", index, txDescArray[index].length, txDescArray[index].vlanTag, txDescArray[index].word1, txDescArray[index].adrl.addr);
            }
#endif
            IOLog("Ethernet [AtherosE2200]: Tx stalled? Resetting chipset. ISR=0x%x, IMR=0x%x.\n", alxReadMem32(ALX_ISR), alxReadMem32(ALX_IMR));
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
    UInt64 mediumSpeed;
    UInt32 mediumIndex = MEDIUM_INDEX_AUTO;
    const char *flowName;
    const char *speedName;
    const char *duplexName;
    const char *eeeName;

    eeeEnable = 0;
    eeeName = eeeNames[kEEETypeNo];

    /* Get link speed, duplex and flow-control mode. */
    if (hw.link_speed == SPEED_1000) {
        mediumSpeed = kSpeed1000MBit;
        speedName = speed1GName;
        duplexName = duplexFullName;
        
        eeeEnable = eeeAdv & eeeLpa;

        if (flowControl) {
            if (eeeEnable) {
                mediumIndex = MEDIUM_INDEX_1000FDFCEEE;
                eeeName = eeeNames[kEEETypeYes];
            } else {
                mediumIndex = MEDIUM_INDEX_1000FDFC;
            }
        } else {
            if (eeeEnable) {
                mediumIndex = MEDIUM_INDEX_1000FDEEE;
                eeeName = eeeNames[kEEETypeYes];
            } else {
                mediumIndex = MEDIUM_INDEX_1000FD;
            }
        }
    } else if (hw.link_speed == SPEED_100) {
        mediumSpeed = kSpeed100MBit;
        speedName = speed100MName;
        
        if (hw.duplex == DUPLEX_FULL) {
            duplexName = duplexFullName;

            eeeEnable = eeeAdv & eeeLpa;
            
            if (flowControl) {
                if (eeeEnable) {
                    mediumIndex = MEDIUM_INDEX_100FDFCEEE;
                    eeeName = eeeNames[kEEETypeYes];
                } else {
                    mediumIndex = MEDIUM_INDEX_100FDFC;
                }
            } else {
                if (eeeEnable) {
                    mediumIndex = MEDIUM_INDEX_100FDEEE;
                    eeeName = eeeNames[kEEETypeYes];
                } else {
                    mediumIndex = MEDIUM_INDEX_100FD;
                }
            }
        } else {
            mediumIndex = MEDIUM_INDEX_100HD;
            duplexName = duplexHalfName;
        }
    } else {
        mediumSpeed = kSpeed10MBit;
        speedName = speed10MName;
        
        if (hw.duplex == DUPLEX_FULL) {
            mediumIndex = MEDIUM_INDEX_10FD;
            duplexName = duplexFullName;
        } else {
            mediumIndex = MEDIUM_INDEX_10HD;
            duplexName = duplexHalfName;
        }
    }
    if (flowControl & (ALX_FC_RX | ALX_FC_TX))
        flowName = flowControlNames[1];
    else
        flowName = flowControlNames[0];

    intrMask = (ALX_ISR_MISC | ALX_ISR_PHY | ALX_ISR_RX_Q0 | ALX_ISR_TX_Q0);
    alxWriteMem32(ALX_IMR, intrMask);
    
    alxPostPhyLink();
    alx_enable_aspm(&hw, false, false);
    pciDevice->setASPMState(this, 0);

    /* Adjust MAC's speed, duplex and flow control settings. */
    alx_start_mac(&hw);
    alx_cfg_mac_flowcontrol(&hw, flowControl);
    
    linkUp = true;
    
#ifdef __PRIVATE_SPI__
    setLinkStatus((kIONetworkLinkValid | kIONetworkLinkActive | linkOpts), mediumTable[mediumIndex], mediumSpeed, NULL);
    linkOpts = 0;
    
    if (rxPoll) {
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
            pollParams.pollIntervalTime = (hw.link_speed == SPEED_1000) ? 200000 : 1000000;  /* 200µs / 1ms */
        }
        netif->setPacketPollingParameters(&pollParams, 0);
    }
    /* Start output thread, statistics update and watchdog. */
    netif->startOutputThread();
#else
    setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, mediumTable[mediumIndex], mediumSpeed, NULL);

    /* Restart txQueue, statistics updates and watchdog. */
    txQueue->start();
    
    if (stalled) {
        txQueue->service();
        stalled = false;
        DebugLog("Ethernet [AtherosE2200]: Restart stalled queue!\n");
    }
#endif /* __PRIVATE_SPI__ */

    timerSource->setTimeoutMS(kTimeoutMS);

    IOLog("Ethernet [AtherosE2200]: Link up on en%u, %s, %s, %s%s\n", netif->getUnitNumber(), speedName, duplexName, flowName, eeeName);
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
    
#ifdef __PRIVATE_SPI__
    /* Stop output thread and flush output queue. */
    netif->stopOutputThread();
    netif->flushOutputQueue();
#else
    /* Stop txQueue. */
    txQueue->stop();
    txQueue->flush();
#endif /* __PRIVATE_SPI__ */
    
    /* Update link status. */
    linkUp = false;
    setLinkStatus(kIONetworkLinkValid);
    
    alx_reset_mac(&hw);
    
    intrMask = (ALX_ISR_MISC | ALX_ISR_PHY);
    alxWriteMem32(ALX_IMR, intrMask);

    /* Cleanup transmitter ring. */
    txClearDescriptors();

    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;

    /* MAC reset causes all HW settings to be lost, restore all */
    alxConfigure();
    alx_enable_aspm(&hw, false, true);
    pciDevice->setASPMState(this, kIOPCIELinkCtlL1);

    alxPostPhyLink();
    
    IOLog("Ethernet [AtherosE2200]: Link down on en%u\n", netif->getUnitNumber());
}

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
        DebugLog("Ethernet [AtherosE2200]: PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & (kPCIPMCPMESupportFromD3Cold | kPCIPMCPMESupportFromD3Hot)) {
            wolCapable = true;
            DebugLog("Ethernet [AtherosE2200]: PME# from D3 (cold/hot) supported.\n");
        }
        pciPMCtrlOffset = pmCapOffset + kIOPCIPMControl;
        
        /* Make sure the device is in D0 power state. */
        provider->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
        setPowerStateWakeAction(this, NULL, NULL, NULL, NULL);
    } else {
        IOLog("Ethernet [AtherosE2200]: PCI power management unsupported.\n");
    }
    
    /* Get PCIe link information. */
    if (provider->findPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
        pcieLinkCap = provider->extendedConfigRead32(pcieCapOffset + kIOPCIELinkCapability);
        pcieLinkCtl = provider->extendedConfigRead16(pcieCapOffset + kIOPCIELinkControl);
        DebugLog("Ethernet [AtherosE2200]: PCIe device capabilities: 0x%08x.\n", provider->extendedConfigRead32(pcieCapOffset + kIOPCIEDevCapability));
        DebugLog("Ethernet [AtherosE2200]: PCIe link capabilities: 0x%08x, link control: 0x%04x.\n", pcieLinkCap, pcieLinkCtl);
        
#ifdef DEBUG
        if (pcieLinkCtl & kIOPCIELinkCtlASPM)
            IOLog("Ethernet [AtherosE2200]: PCIe ASPM enabled.\n");
#endif  /* DEBUG */
        
    }
    /* Enable the device. */
    cmdReg	= provider->extendedConfigRead16(kIOPCIConfigCommand);
    cmdReg	|= kALXPCICommand;
	provider->extendedConfigWrite16(kIOPCIConfigCommand, cmdReg);
    
    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("Ethernet [AtherosE2200]: region #0 not an MMIO resource, aborting.\n");
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
        DebugLog("Ethernet [AtherosE2200]: Restored PCI command register.\n");
	}
    /* Check if the NIC has been disabled by the BIOS. */
    val = alxReadMem32(ALX_DRV);
    
    if (val & ALX_DRV_DISABLE) {
        IOLog("Ethernet [AtherosE2200]: NIC disabled by BIOS, aborting.\n");
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
        DebugLog("Ethernet [AtherosE2200]: Got MAC address from register.\n");
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
        DebugLog("Ethernet [AtherosE2200]: Got MAC address from efuse.\n");
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
            DebugLog("Ethernet [AtherosE2200]: Got MAC address from EEPROM.\n");
            result = true;
        }
    }
    
done:
    if (result)
        memcpy(&currMacAddr.bytes[0], &origMacAddr.bytes[0], kIOEthernetAddressSize);

    DebugLog("alxLoadDefaultAddress() <===\n");
    
    return result;
}

void AtherosE2200::alxSetHardwareAddress(const IOEthernetAddress *addr)
{
    UInt32 mac0, mac1;
    
    mac0 = ((addr->bytes[2] << 24) || (addr->bytes[3] << 16) || (addr->bytes[4] << 8) || addr->bytes[5]);
    alxWriteMem32(ALX_STAD0, mac0);
    mac1 = ((addr->bytes[0] << 8) || addr->bytes[1]);
    alxWriteMem32(ALX_STAD1, mac1);
    
    if ((alxReadMem32(ALX_STAD0) != mac0) || (alxReadMem32(ALX_STAD1) != mac1))
        IOLog("Ethernet [AtherosE2200]: Failed to set MAC address.\n");
}

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
    
	hw.rx_ctrl = (ALX_MAC_CTRL_WOLSPED_SWEN | ALX_MAC_CTRL_BRD_EN | ALX_MAC_CTRL_VLANSTRIP | ALX_MAC_CTRL_MHASH_ALG_HI5B | ALX_MAC_CTRL_PCRCE | ALX_MAC_CTRL_CRCE | ALX_MAC_CTRL_RXFC_EN | ALX_MAC_CTRL_TXFC_EN | (7 << ALX_MAC_CTRL_PRMBLEN_SHIFT));
    
    eeeAdv = eeeCap;
    
#ifdef __PRIVATE_SPI__
    linkOpts = 0;
#endif /* __PRIVATE_SPI__ */

    if (!alxResetPCIe())
        goto done;
    
	phyConfigured = alx_phy_configured(&hw);
    
	if (!phyConfigured)
		alxResetPhy();
    
	if (alx_reset_mac(&hw)) {
        IOLog("Ethernet [AtherosE2200]: Failed to reset MAC.\n");
        //goto done;
    }
	/* setup link to put it in a known good starting state */
	if (!phyConfigured) {
        error = alxSetupSpeedDuplex(hw.adv_cfg, eeeAdv, hw.flowctrl);
        
		if (error) {
            IOLog("Ethernet [AtherosE2200]: Failed to configure PHY speed/duplex: %d.\n", error);
            goto done;
        }
	}
	if (!alxLoadDefaultAddress()) {
        IOLog("Ethernet [AtherosE2200]: Failed to get permanent MAC address.\n");
        goto done;
	}
	hw.mdio.prtad = 0;
	hw.mdio.mmds = 0;
	hw.mdio.dev = NULL;
	hw.mdio.mode_support = (MDIO_SUPPORTS_C45 | MDIO_SUPPORTS_C22 | MDIO_EMULATE_C22);
	hw.mdio.mdio_read = NULL;
	hw.mdio.mdio_write = NULL;
    
	if (!alx_get_phy_info(&hw)) {
        IOLog("Ethernet [AtherosE2200]: Failed to identify PHY.\n");
        goto done;
	}
    IOLog("Ethernet [AtherosE2200]: %s: (Rev. %u) at 0x%lx, %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
          chipNames[chip], pciDeviceData.revision, (unsigned long)baseAddr,
          origMacAddr.bytes[0], origMacAddr.bytes[1],
          origMacAddr.bytes[2], origMacAddr.bytes[3],
          origMacAddr.bytes[4], origMacAddr.bytes[5]);
    result = true;
    
done:
    return result;
}

void AtherosE2200::alxEnable()
{
    const IONetworkMedium *selectedMedium = getSelectedMedium();
    UInt32 msiControl = ((hw.imt >> 1) << ALX_MSI_RETRANS_TM_SHIFT);
    
    if (!selectedMedium) {
        DebugLog("Ethernet [AtherosE2200]: No medium selected. Falling back to autonegotiation.\n");
        selectedMedium = mediumTable[MEDIUM_INDEX_AUTO];
        setCurrentMedium(selectedMedium);
    }

#ifdef __PRIVATE_SPI__
    /* Check if we re waking up from sleep with WoL enabled and still have a valid link. */
    if (!linkOpts || alxReadPhyLink() || (hw.link_speed == SPEED_UNKNOWN)) {
        linkOpts = 0;
        setLinkStatus(kIONetworkLinkValid);
    }
    polling = false;
#else
    setLinkStatus(kIONetworkLinkValid);
#endif /* __PRIVATE_SPI__ */

    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;

    alxSpeedDuplexForMedium(selectedMedium);
    alxSetupSpeedDuplex(hw.adv_cfg, eeeAdv, hw.flowctrl);

    alxResetPCIe();
    alxResetPhy();
    alx_reset_mac(&hw);
	alxConfigure();
    
	if (useMSI) {
		alxWriteMem32(ALX_MSI_RETRANS_TIMER, msiControl | ALX_MSI_MASK_SEL_LINE);
        
        /* Configure vector mapping. */
        alxWriteMem32(ALX_MSI_MAP_TBL1, 0);
        alxWriteMem32(ALX_MSI_MAP_TBL2, 0);
        alxWriteMem32(ALX_MSI_ID_MAP, 0);
	} else {
        alxWriteMem32(ALX_MSI_RETRANS_TIMER, 0);
    }
    alx_enable_aspm(&hw, false, true);
    pciDevice->setASPMState(this, kIOPCIELinkCtlL1);

    /* clear old interrupts */
	alxWriteMem32(ALX_ISR, ~(UInt32)ALX_ISR_DIS);

    /* Enable all known interrupts by setting the interrupt mask. */
    alxEnableIRQ();
}

int AtherosE2200::alxDisable()
{
    UInt32 status = kIONetworkLinkValid;
    int error;
    int speed = 0;
    UInt8 duplex = 0;

    alxDisableIRQ();
    
    hw.link_speed = SPEED_UNKNOWN;
    hw.duplex = DUPLEX_UNKNOWN;
    
#ifdef __PRIVATE_SPI__
    polling = false;
    linkOpts = 0;
#endif /* __PRIVATE_SPI__ */

	alx_reset_mac(&hw);
    
	/* disable l0s/l1 */
	alx_enable_aspm(&hw, false, false);
    pciDevice->setASPMState(this, 0);

    if (hw.sleep_ctrl & ALX_SLEEP_ACTIVE) {
        error = alxSelectPowersavingSpeed(&speed, &duplex);
        
        if (error) {
            DebugLog("Ethernet [AtherosE2200]: alx_select_powersaving_speed() failed.\n");
            goto done;
        }
        error = alx_clear_phy_intr(&hw);
        
        if (error) {
            DebugLog("Ethernet [AtherosE2200]: alx_clear_phy_intr() failed.\n");
            goto done;
        }
        error = alx_pre_suspend(&hw, speed, duplex);
        
        if (error) {
            DebugLog("Ethernet [AtherosE2200]: alx_pre_suspend() failed.\n");
            goto done;
        }
        error = alx_config_wol(&hw);
        
        if (error) {
            DebugLog("Ethernet [AtherosE2200]: alx_config_wol() failed.\n");
            goto done;
        }
#ifdef __PRIVATE_SPI__
        linkOpts = kIONetworkLinkNoNetworkChange;
        status |= linkOpts;
#endif /* __PRIVATE_SPI__ */
    }
    error = 0;
    
done:
    if (linkUp) {
        linkUp = false;
        setLinkStatus(status);
        IOLog("Ethernet [AtherosE2200]: Link down on en%u\n", netif->getUnitNumber());
    }
    return error;
}

/* Reset the NIC in case a tx deadlock or a pci error occurred. timerSource and txQueue
 * are stopped immediately but will be restarted by checkLinkStatus() when the link has
 * been reestablished.
 */

void AtherosE2200::alxRestart()
{
    
#ifdef __PRIVATE_SPI__
    /* Stop output thread and flush txQueue */
    netif->stopOutputThread();
    netif->flushOutputQueue();
#else
    /* Stop and cleanup txQueue. */
    txQueue->stop();
    txQueue->flush();
#endif /* __PRIVATE_SPI__ */
    
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

    txClearDescriptors();
    rxNextDescIndex = 0;
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
    
	rawMTU = hw.mtu + ETHER_HDR_LEN;
	alxWriteMem32(ALX_MTU, rawMTU + 8);
    
	if (rawMTU > ALX_MTU_JUMBO_TH)
		hw.rx_ctrl &= ~ALX_MAC_CTRL_FAST_PAUSE;
    else
        hw.rx_ctrl |= ALX_MAC_CTRL_FAST_PAUSE;

	if ((rawMTU + 8) < ALX_TXQ1_JUMBO_TSO_TH)
		val = (rawMTU + 8 + 7) >> 3;
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
        DebugLog("Ethernet [AtherosE2200]: Restore dma read max length: 0x%x.\n", val16);
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
	UInt32 addrHigh = (txPhyAddr >> 32);
    UInt32 addrLow;
    
    txDirtyDescIndex = txNextDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    rxNextDescIndex = 0;

    addrLow = (UInt32)(rxRetPhyAddr & 0xffffffff);
	alxWriteMem32(ALX_RX_BASE_ADDR_HI, addrHigh);
	alxWriteMem32(ALX_RRD_ADDR_LO, addrLow);
	alxWriteMem32(ALX_RRD_RING_SZ, kNumRxDesc);
    
    addrLow = (UInt32)(rxFreePhyAddr & 0xffffffff);
	alxWriteMem32(ALX_RFD_ADDR_LO, addrLow);
	alxWriteMem32(ALX_RFD_RING_SZ, kNumRxDesc);
	alxWriteMem32(ALX_RFD_BUF_SZ, kRxBufferPktSize);
    
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
            DebugLog("Ethernet [AtherosE2200]: Found AR8162.\n");
            break;
            
        case ALX_DEV_ID_AR8161:
            chip = kChipAR8161;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Ethernet [AtherosE2200]: Found AR8161.\n");
            break;
            
        case ALX_DEV_ID_AR8172:
            chip = kChipAR8172;
            gbCapable = false;
            eeeCap = ALX_LOCAL_EEEADV_100BT;
            DebugLog("Ethernet [AtherosE2200]: Found AR8172.\n");
            break;
            
        case ALX_DEV_ID_AR8171:
            chip = kChipAR8171;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Ethernet [AtherosE2200]: Found AR8171.\n");
            break;
            
        case ALX_DEV_ID_E2200:
            chip = kChipKillerE2200;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Ethernet [AtherosE2200]: Found Killer E2200.\n");
            break;
            
        case ALX_DEV_ID_E2400:
            chip = kChipKillerE2400;
            gbCapable = true;
            eeeCap = (ALX_LOCAL_EEEADV_100BT | ALX_LOCAL_EEEADV_1000BT);
            DebugLog("Ethernet [AtherosE2200]: Found Killer E2400.\n");
            break;
            
        default:
            IOLog("Ethernet [AtherosE2200]: Unknown chip. Aborting.\n");
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

inline void AtherosE2200::alxGetChkSumCommand(UInt32 *cmd, mbuf_csum_request_flags_t checksums)
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
    
    DebugLog("Ethernet [AtherosE2200]: EEE link partner: 0x%04x.\n", eeeLpa);
    
done:
	return error;
    
wrong_speed:
    IOLog("Ethernet [AtherosE2200]: Invalid PHY speed/duplex: 0x%x\n", giga);
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

#pragma mark --- timer action methods ---

void AtherosE2200::timerAction(IOTimerEventSource *timer)
{
    UInt32 lpi;
    
    if (!linkUp) {
        DebugLog("Ethernet [AtherosE2200]: Timer fired while link down.\n");
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
        DebugLog("Ethernet [AtherosE2200]: Enable LPI: ALX_LPI_CTRL=0x%08x.\n", lpi);
    }
done:
    txDescDoneLast = txDescDoneCount;
    
#ifndef __PRIVATE_SPI__
    txStallLast = txStallCount;
#endif /* __PRIVATE_SPI__ */

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
    struct iphdr *ipHdr = (struct iphdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);
    struct tcphdr *tcpHdr = (struct tcphdr *)((UInt8 *)ipHdr + sizeof(struct iphdr));
    UInt32 plen = ntohs(ipHdr->tot_len) - sizeof(struct iphdr);
    UInt32 csum = ntohs(tcpHdr->th_sum) - plen;
    
    csum += (csum >> 16);
    tcpHdr->th_sum = htons((UInt16)csum);
}

static inline UInt32 adjustIPv6Header(mbuf_t m)
{
    struct ip6_hdr *ip6Hdr = (struct ip6_hdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);
    struct tcphdr *tcpHdr = (struct tcphdr *)((UInt8 *)ip6Hdr + sizeof(struct ip6_hdr));
    UInt32 plen = ntohs(ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen);
    UInt32 csum = ntohs(tcpHdr->th_sum) - plen;
    
    csum += (csum >> 16);
    ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen = 0;
    tcpHdr->th_sum = htons((UInt16)csum);

    return (plen + kMinL4HdrOffsetV6);
}

