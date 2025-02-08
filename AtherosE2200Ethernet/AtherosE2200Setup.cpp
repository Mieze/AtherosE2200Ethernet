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
        
        interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosE2200::interruptOccurred), provider, msiIndex);
    }
    if (!interruptSource) {
        DebugLog("Warning: MSI index was not found or MSI interrupt could not be enabled.\n");
        
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
    IOPhysicalSegment rxSegment;
    IODMACommand::Segment64 seg;
    QCARxDescArray *descArray;
    mbuf_t spareMbuf[kRxNumSpareMbufs];
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
    rxMbufArray = (mbuf_t *)rxBufArrayMem;

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
        rxMbufArray[i] = NULL;
    }
    rxNextDescIndex = 0;
    
    rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, 1);
    
    if (!rxMbufCursor) {
        IOLog("Couldn't create rxMbufCursor.\n");
        goto error_segm;
    }

    /* Alloc receive buffers. */
    for (i = 0; i < kNumRxDesc; i++) {
        m = allocatePacket(kRxBufferPktSize);
        
        if (!m) {
            IOLog("Couldn't alloc receive buffer.\n");
            goto error_buf;
        }
        rxMbufArray[i] = m;
        
        if (rxMbufCursor->getPhysicalSegments(m, &rxSegment, 1) != 1) {
            IOLog("getPhysicalSegments() for receive buffer failed.\n");
            goto error_buf;
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
    
error_buf:
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxMbufArray[i]) {
            freePacket(rxMbufArray[i]);
            rxMbufArray[i] = NULL;
        }
    }
    RELEASE(rxMbufCursor);

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
    rxMbufArray = NULL;

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
        IOLog("Couldn't alloc transmit buffer array.\n");
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
    txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(0x1000, kMaxSegs);

    if (!txMbufCursor) {
        IOLog("Couldn't create txMbufCursor.\n");
        goto error_segm;
    }
    result = true;
    
done:
    return result;
    
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
    RELEASE(rxMbufCursor);
    
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxMbufArray[i]) {
            freePacket(rxMbufArray[i]);
            rxMbufArray[i] = NULL;
        }
    }
    if (rxBufArrayMem) {
        IOFree(rxBufArrayMem, kRxBufArraySize);
        rxBufArrayMem = NULL;
        rxMbufArray = NULL;
    }
}

void AtherosE2200::freeTxResources()
{
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
    RELEASE(txMbufCursor);

    if (txBufArrayMem) {
        IOFree(txBufArrayMem, kTxBufArraySize);
        txBufArrayMem = NULL;
        txMbufArray = NULL;
    }
}

void AtherosE2200::clearDescriptors()
{
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
    txDirtyDescIndex = txNextDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    
    /* Cleanup rxRetDescArray. */
    bzero(rxRetDescArray, kRxRetDescArraySize);
    rxNextDescIndex = 0;

    DebugLog("clearDescriptors() <===\n");
}

void AtherosE2200::getParams(UInt32 *intrLimit)
{
    OSNumber *intrRate;
    OSBoolean *tso4;
    OSBoolean *tso6;
    OSBoolean *csoV6;
    OSBoolean *poll;

    poll = OSDynamicCast(OSBoolean, getProperty(kEnableRxPollName));
    rxPoll = (poll) ? poll->getValue() : false;
    
    IOLog("RxPoll support %s.\n", rxPoll ? onName : offName);

    tso4 = OSDynamicCast(OSBoolean, getProperty(kEnableTSO4Name));
    enableTSO4 = (tso4) ? tso4->getValue() : false;
    
    IOLog("TCP/IPv4 segmentation offload %s.\n", enableTSO4 ? onName : offName);
    
    tso6 = OSDynamicCast(OSBoolean, getProperty(kEnableTSO6Name));
    enableTSO6 = (tso6) ? tso6->getValue() : false;
    
    IOLog("TCP/IPv6 segmentation offload %s.\n", enableTSO6 ? onName : offName);
    
    csoV6 = OSDynamicCast(OSBoolean, getProperty(kEnableCSO6Name));
    enableCSO6 = (csoV6) ? csoV6->getValue() : false;
    
    IOLog("TCP/IPv6 checksum offload %s.\n", enableCSO6 ? onName : offName);
    
    intrRate = OSDynamicCast(OSNumber, getProperty(kIntrRateName));
    *intrLimit = 5000;
    
    if (intrRate)
        *intrLimit = intrRate->unsigned32BitValue();
}
