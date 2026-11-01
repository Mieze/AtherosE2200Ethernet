/* AtherosE2200Setup.cpp -- AtherosE2200 resource allocation.
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

static const char *onName = "enabled";
static const char *offName = "disabled";

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


#pragma mark --- data structure initialization methods ---

void AtherosE2200::getParams(UInt32 *intrLimit)
{
    OSDictionary *params;
    OSIterator *iterator;
    OSNumber *intrRate;
    OSBoolean *tso4;
    OSBoolean *tso6;
    OSBoolean *csoV6;
    OSString *versionString;

    if (version_major >= Tahoe) {
        params = serviceMatching("AppleVTD");
        
        if (params) {
            iterator = IOService::getMatchingServices(params);
            
            if (iterator) {
                IOMapper *mp = OSDynamicCast(IOMapper, iterator->getNextObject());
                
                if (mp) {
                    IOLog("AppleVTD is enabled.");
                    useAppleVTD = true;
                }
                iterator->release();
            }
            params->release();
        }
    }
    versionString = OSDynamicCast(OSString, getProperty(kDriverVersionName));

    params = OSDynamicCast(OSDictionary, getProperty(kParamName));
    
    if (params) {
        tso4 = OSDynamicCast(OSBoolean, params->getObject(kEnableTSO4Name));
        enableTSO4 = (tso4 != NULL) ? tso4->getValue() : false;
        
        IOLog("TCP/IPv4 segmentation offload %s.\n", enableTSO4 ? onName : offName);
        
        tso6 = OSDynamicCast(OSBoolean, params->getObject(kEnableTSO6Name));
        enableTSO6 = (tso6 != NULL) ? tso6->getValue() : false;
        
        IOLog("TCP/IPv6 segmentation offload %s.\n", enableTSO6 ? onName : offName);
        
        csoV6 = OSDynamicCast(OSBoolean, params->getObject(kEnableCSO6Name));
        enableCSO6 = (csoV6 != NULL) ? csoV6->getValue() : false;
        
        IOLog("TCP/IPv6 checksum offload %s.\n", enableCSO6 ? onName : offName);
        
        intrRate = OSDynamicCast(OSNumber, params->getObject(kIntrRateName));
        *intrLimit = 5000;
        
        if (intrRate)
            *intrLimit = intrRate->unsigned32BitValue();
    }
}

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
    IOLog("Error creating medium dictionary.\n");
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
        IOLog("Failed to get output queue.\n");
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
        DebugLog("MSI interrupt index: %d\n", msiIndex);
        
        if (useAppleVTD) {
            interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosE2200::interruptOccurredVTD), provider, msiIndex);
        } else {
            interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosE2200::interruptOccurred), provider, msiIndex);
        }
    }
    if (!interruptSource) {
        IOLog("MSI interrupt could not be enabled.\n");

        goto error1;
    }
    workLoop->addEventSource(interruptSource);
    
    interruptSource->enable();
    
    timerSource = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &AtherosE2200::timerAction));
    
    if (!timerSource) {
        IOLog("Failed to create IOTimerEventSource.\n");
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
    IOLog("Error initializing event sources.\n");
    txQueue->release();
    txQueue = NULL;
    goto done;
}

bool AtherosE2200::setupRxResources()
{
    IODMACommand::Segment64 seg;
    IOPhysicalAddress64 pa = 0;
    QCARxDescArray *descArray;
    mbuf_t m;
    UInt64 offset = 0;
    UInt32 numSegs = 1;
    UInt32 i;
    bool result = false;
    
    /* Alloc rx mbuf_t array. */
    rxBufArrayMem = IOMallocZero(kRxBufArraySize);
    
    if (!rxBufArrayMem) {
        IOLog("Couldn't alloc receive buffer array.\n");
        goto done;
    }
    rxBufArray = (athRxBufferInfo *)rxBufArrayMem;

    /* Create receiver descriptor array. */
    rxBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMemoryHostPhysicallyContiguous | kIOMapInhibitCache), kRxDescArraySize, 0xFFFFFFFFFFFFF000ULL);
    
    if (!rxBufDesc) {
        IOLog("Couldn't alloc rxBufDesc.\n");
        goto error_buff;
    }
    if (rxBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("rxBufDesc->prepare() failed.\n");
        goto error_prep;
    }

    rxDescDmaCmd = IODMACommand::withSpecification(kIODMACommandOutputHost64, 64, 0, IODMACommand::kMapped, 0, 1, mapper, NULL);
    
    if (!rxDescDmaCmd) {
        IOLog("Couldn't alloc rxDescDmaCmd.\n");
        goto error_dma;
    }
    
    if (rxDescDmaCmd->setMemoryDescriptor(rxBufDesc) != kIOReturnSuccess) {
        IOLog("setMemoryDescriptor() failed.\n");
        goto error_set_desc;
    }
    
    if (rxDescDmaCmd->gen64IOVMSegments(&offset, &seg, &numSegs) != kIOReturnSuccess) {
        IOLog("gen64IOVMSegments() failed.\n");
        goto error_segm;
    }
    /* And the rx ring's physical address too. */
    rxRetPhyAddr = seg.fIOVMAddr;
    
    /* Setup receiver return descriptor array. */
    descArray = (QCARxDescArray *)rxBufDesc->getBytesNoCopy();
    rxRetDescArray = &descArray->rxRetDesc[0];
    
    /* Initialize rxRetDescArray. */
    bzero(rxRetDescArray, kRxRetDescArraySize);

    /* Setup receiver free descriptor array. */
    rxFreeDescArray =&descArray->rxFreeDesc[0];
    rxFreePhyAddr = rxRetPhyAddr + offsetof(QCARxDescArray, rxFreeDesc);
    
    /* Initialize rxFreeDescArray. */
    bzero(rxFreeDescArray, kRxFreeDescArraySize);

    for (i = 0; i < kNumRxDesc; i++) {
        rxBufArray[i].mbuf = NULL;
        rxBufArray[i].phyAddr = 0;
    }
    rxNextDescIndex = 0;
    rxCleanedCount = rxMapNextIndex = 0;

    rxPool = AtherosE2200RxPool::withCapacity(kRxPoolMbufCap, kRxPoolClstCap);

    if (!rxPool) {
        IOLog("Couldn't alloc receive buffer pool.\n");
        goto error_rx_pool;
    }

    /* Alloc receive buffers. */
    for (i = 0; i < kNumRxDesc; i++) {
        m = rxPool->getPacket(kRxBufferSize, MBUF_WAITOK);
        
        if (!m) {
            IOLog("Couldn't alloc receive buffer.\n");
            goto error_rx_buf;
        }
        rxBufArray[i].mbuf = m;

        if (!useAppleVTD) {
            pa = mbuf_data_to_physical(mbuf_data(m));
            
            /* We have to keep the physical address of the buffer too
             * as descriptor write back overwrites it in the descriptor
             * so that it must be refreshed when the descriptor is
             * prepared for reuse.
             */
            rxBufArray[i].phyAddr = pa;

            rxFreeDescArray[i].addr = OSSwapHostToLittleInt64(pa);
        }
    }
    if (useAppleVTD) {
        result = setupRxMap();
        
        if (!result)
            goto error_rx_buf;

    } else {
        result = true;
    }

done:
    return result;
    
error_rx_buf:
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxBufArray[i].mbuf) {
            mbuf_freem_list(rxBufArray[i].mbuf);
            rxBufArray[i].mbuf = NULL;
            rxBufArray[i].phyAddr = 0;
        }
    }
    RELEASE(rxPool);

error_rx_pool:
    rxRetPhyAddr = NULL;
    rxFreePhyAddr = NULL;
    
error_segm:
    rxDescDmaCmd->clearMemoryDescriptor();

error_set_desc:
    RELEASE(rxDescDmaCmd);

error_dma:
    rxBufDesc->complete();
    
error_prep:
    RELEASE(rxBufDesc);

error_buff:
    IOFree(rxBufArrayMem, kRxBufArraySize);
    rxBufArrayMem = NULL;
    rxBufArray = NULL;

    goto done;
}

bool AtherosE2200::setupTxResources()
{
    IODMACommand::Segment64 seg;
    UInt64 offset = 0;
    UInt32 numSegs = 1;
    UInt32 i;
    bool result = false;
    
    /* Alloc tx mbuf_t array. */
    txBufArrayMem = IOMallocZero(kTxBufArraySize);
    
    if (!txBufArrayMem) {
        IOLog("Couldn't alloc transmit mbuf array.\n");
        goto done;
    }
    txMbufArray = (mbuf_t *)txBufArrayMem;
    
    /* Create transmitter descriptor array. */
    txBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMemoryHostPhysicallyContiguous | kIOMapInhibitCache), kTxDescArraySize, 0xFFFFFFFFFFFFFF00ULL);
                
    if (!txBufDesc) {
        IOLog("Couldn't alloc txBufDesc.\n");
        goto error_buff;
    }
    if (txBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("txBufDesc->prepare() failed.\n");
        goto error_prep;
    }
    txDescArray = (QCATxDesc *)txBufDesc->getBytesNoCopy();

    txDescDmaCmd = IODMACommand::withSpecification(kIODMACommandOutputHost64, 64, 0, IODMACommand::kMapped, 0, 1, mapper, NULL);
    
    if (!txDescDmaCmd) {
        IOLog("Couldn't alloc txDescDmaCmd.\n");
        goto error_dma;
    }
    
    if (txDescDmaCmd->setMemoryDescriptor(txBufDesc) != kIOReturnSuccess) {
        IOLog("setMemoryDescriptor() failed.\n");
        goto error_set_desc;
    }
    
    if (txDescDmaCmd->gen64IOVMSegments(&offset, &seg, &numSegs) != kIOReturnSuccess) {
        IOLog("gen64IOVMSegments() failed.\n");
        goto error_segm;
    }
    /* Now get tx ring's physical address. */
    txPhyAddr = seg.fIOVMAddr;
    
    /* Initialize txDescArray. */
    bzero(txDescArray, kTxDescArraySize);
    
    for (i = 0; i < kNumTxDesc; i++) {
        txMbufArray[i] = NULL;
    }
    txNextDescIndex = txDirtyDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    
    if (useAppleVTD) {
        result = setupTxMap();
        
        if (!result)
            goto error_tx_cursor;
    } else {
        txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, kMaxSegs);
        
        if (!txMbufCursor) {
            IOLog("Couldn't create txMbufCursor.\n");
            goto error_tx_cursor;
        }
    }
    result = true;
    
done:
    return result;
    
error_tx_cursor:
    txPhyAddr = 0;

error_segm:
    txDescDmaCmd->clearMemoryDescriptor();

error_set_desc:
    RELEASE(txDescDmaCmd);
    
error_dma:
    txBufDesc->complete();

error_prep:
    RELEASE(txBufDesc);
    
error_buff:
    IOFree(txBufArrayMem, kTxBufArraySize);
    txBufArrayMem = NULL;
    txMbufArray = NULL;
    
    goto done;
}

void AtherosE2200::freeRxResources()
{
    UInt32 i;
    
    if (useAppleVTD)
        freeRxMap();

    if (rxDescDmaCmd) {
        rxDescDmaCmd->complete();
        rxDescDmaCmd->clearMemoryDescriptor();
        rxDescDmaCmd->release();
        rxDescDmaCmd = NULL;
    }
    if (rxBufDesc) {
        rxBufDesc->complete();
        rxBufDesc->release();
        rxBufDesc = NULL;;
        rxRetPhyAddr = (IOPhysicalAddress64)NULL;
        rxFreePhyAddr = (IOPhysicalAddress64)NULL;
    }
    
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxBufArray[i].mbuf) {
            freePacket(rxBufArray[i].mbuf);
            rxBufArray[i].mbuf = NULL;
            rxBufArray[i].phyAddr = 0;
        }
    }
    if (rxBufArrayMem) {
        IOFree(rxBufArrayMem, kRxBufArraySize);
        rxBufArrayMem = NULL;
        rxBufArray = NULL;
    }
}

void AtherosE2200::freeTxResources()
{
    if (useAppleVTD)
        freeTxMap();

    if (txDescDmaCmd) {
        txDescDmaCmd->complete();
        txDescDmaCmd->clearMemoryDescriptor();
        txDescDmaCmd->release();
        txDescDmaCmd = NULL;
    }
    if (txBufDesc) {
        txBufDesc->complete();
        txBufDesc->release();
        txBufDesc = NULL;
        txPhyAddr = (IOPhysicalAddress64)NULL;
    }
    if (txBufArrayMem) {
        IOFree(txBufArrayMem, kTxBufArraySize);
        txBufArrayMem = NULL;
        txMbufArray = NULL;
    }
    RELEASE(txMbufCursor);
}

void AtherosE2200::clearDescriptors()
{
    IOMemoryDescriptor *md;
    mbuf_t m;
    UInt32 i;
    
    DebugLog("clearDescriptors() ===>\n");
    
    /* Cleanup txMbufArray. */
    for (i = 0; i < kNumTxDesc; i++) {
        m = txMbufArray[i];
        
        if (m) {
            freePacket(m);
            txMbufArray[i] = NULL;
        }
    }
    if (useAppleVTD) {
        for (i = 0; i < kNumTxMemDesc; i++) {
            md = txMapInfo->txMemIO[i];
            
            if (md && (md->getTag() == kIOMemoryActive)) {
                md->complete();
                md->setTag(kIOMemoryInactive);
            }
        }
        txMapInfo->txNextMem2Use = txMapInfo->txNextMem2Free = 0;
        txMapInfo->txNumFreeMem = kNumTxMemDesc;
    }
    txDirtyDescIndex = txNextDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    
    if (useAppleVTD) {
        rxMapNextIndex = 0;
        rxMapBuffers(0, kNumRxMemDesc, false);
    }

    /* Cleanup rxRetDescArray. */
    bzero(rxRetDescArray, kRxRetDescArraySize);
    
    rxNextDescIndex = 0;
    rxCleanedCount = rxMapNextIndex = 0;

    DebugLog("clearDescriptors() <===\n");
}
