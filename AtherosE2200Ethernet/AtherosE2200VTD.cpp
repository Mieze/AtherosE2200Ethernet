//
//  AtherosE2200VTD.cpp
//  AtherosE2200EthernetV2
//
//  Created by Laura Müller on 25.10.26.
//  Copyright © 2026 Laura Müller. All rights reserved.
//
/* AtherosE2200VTD.cpp -- AppleVTD support of AtherosE2200 for Tahoe.
 *
 * Copyright (c) 2026 Laura Müller <laura-mueller@uni-duesseldorf.de>
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

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif /* MIN */

#define next_page(x) trunc_page(x + PAGE_SIZE)

#pragma mark --- initialisation methods for AppleVTD support ---

bool AtherosE2200::setupRxMap()
{
    IOMemoryDescriptor *md;
    IOPhysicalAddress pa;
    IOByteCount offset;
    UInt32 end;
    UInt32 i, n, idx;
    bool result = false;

    /* Alloc ixgbeRxBufferInfo. */
    rxMapMem = IOMallocZero(kRxMapMemSize);
    
    if (!rxMapMem) {
        IOLog("Couldn't alloc rx map.\n");
        goto done;
    }
    rxMapInfo = (athRxMapInfo *)rxMapMem;
    
    /* Setup Ranges for IOMemoryDescriptors. */
    for (i = 0; i < kNumRxDesc; i++) {
        rxMapInfo->rxMemRange[i].address = (IOVirtualAddress)mbuf_datastart(rxBufArray[i].mbuf);
        rxMapInfo->rxMemRange[i].length = PAGE_SIZE;
    }

    /* Alloc IOMemoryDescriptors. */
    for (i = 0, idx = 0; i < kNumRxMemDesc; i++, idx += kRxMemBatchSize) {
        md = IOMemoryDescriptor::withOptions(&rxMapInfo->rxMemRange[idx], kRxMemBatchSize, 0, kernel_task, (kIOMemoryTypeVirtual | kIODirectionIn | kIOMemoryAsReference), mapper);
        
        if (!md) {
            IOLog("Couldn't alloc IOMemoryDescriptor.\n");
            goto error_rx_desc;
        }
        if (md->prepare() != kIOReturnSuccess) {
            IOLog("IOMemoryDescriptor::prepare() failed.\n");
            goto error_prep;
        }
        rxMapInfo->rxMemIO[i] = md;
        offset = 0;
        end = idx + kRxMemBatchSize;

        for (n = idx; n < end; n++) {
            pa = md->getPhysicalSegment(offset, NULL);
            rxBufArray[n].phyAddr = pa;
            
            rxFreeDescArray[n].addr = OSSwapHostToLittleInt64(pa);

            offset += PAGE_SIZE;
        }
    }
    result = true;
    
done:
    return result;
            
error_prep:
    md->complete();
    RELEASE(md);

error_rx_desc:
    if (rxMapMem) {
        for (i = 0; i < kNumRxMemDesc; i++) {
            md = rxMapInfo->rxMemIO[i];
                            
            if (md) {
                md->complete();
                md->release();
            }
            rxMapInfo->rxMemIO[i] = NULL;
        }
        IOFree(rxMapMem, kRxMapMemSize);
        rxMapMem = NULL;
    }
    goto done;
}

void AtherosE2200::freeRxMap()
{
    IOMemoryDescriptor *md;
    UInt32 i;

    if (rxMapMem) {
        for (i = 0; i < kNumRxMemDesc; i++) {
            md = rxMapInfo->rxMemIO[i];
                            
            if (md) {
                md->complete();
                md->release();
            }
            rxMapInfo->rxMemIO[i] = NULL;
        }
        IOFree(rxMapMem, kRxMapMemSize);
        rxMapMem = NULL;
    }
}

bool AtherosE2200::setupTxMap()
{
    bool result = false;

    txMapMem = IOMallocZero(kTxMapMemSize);
    
    if (!txMapMem) {
        IOLog("Couldn't alloc memory for tx map.\n");
        goto done;
    }
    txMapInfo = (athTxMapInfo *)txMapMem;
    
    txMapInfo->txNextMem2Use = 0;
    txMapInfo->txNextMem2Free = 0;
    txMapInfo->txNumFreeMem = kNumTxMemDesc;

    result = true;
    
done:
    return result;
}

void AtherosE2200::freeTxMap()
{
    UInt32 i;

    if (txMapMem) {
        for (i = 0; i < kNumTxMemDesc; i++) {
            if (txMapInfo->txMemIO[i]) {
                txMapInfo->txMemIO[i]->complete();
                txMapInfo->txMemIO[i]->release();
                txMapInfo->txMemIO[i] = NULL;
            }
        }
        IOFree(txMapMem, kTxMapMemSize);
        txMapMem = NULL;
    }
}

#pragma mark --- interrupt methods for AppleVTD support ---

void AtherosE2200::interruptOccurredVTD(OSObject *client, IOInterruptEventSource *src, int count)
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
            packets = rxInterruptVTD(netif, kNumRxDesc, NULL, NULL);
            
            if (packets)
                netif->flushInputQueue();
        }
    }
    if (status & ALX_ISR_PHY)
        checkLinkStatus();
       
done:
    alxWriteMem32(ALX_ISR, 0);
}


#pragma mark --- tx methods for AppleVTD support ---

/*
 * Map a tx packet for read DMA access by the NIC.
 * The packet is split up into physical contiguous segments
 * and an IOMemoryDescriptor is used to map all segments for
 * DMA access.
 */
UInt32 AtherosE2200::txMapPacket(mbuf_t packet,
                            IOPhysicalSegment *vector,
                            UInt32 maxSegs)
{
    IOMemoryDescriptor *md = NULL;
    IOAddressRange *srcRange;
    IOAddressRange *dstRange;
    mbuf_t m;
    IOVirtualAddress d;
    IOByteCount offset;
    UInt64 len, l;
    UInt32 segIndex = 0;
    UInt32 i;
    UInt16 saveMem;
    bool result = false;

    if (packet && vector && maxSegs) {
        srcRange = txMapInfo->txSCRange;
        m = packet;
        
        /*
         * Split up the packet into virtual contiguos segments.
         */
        if (mbuf_next(m) == 0) {
            d = (IOVirtualAddress)mbuf_data(m);
            len = mbuf_len(m);
            
            if ( trunc_page(d) == trunc_page(d + len - 1) ) {
                srcRange[0].address = d;
                srcRange[0].length = len;
                segIndex = 1;
                goto map;
            }
        }
        do {
            d = (IOVirtualAddress)mbuf_data(m);
            
            for (len = mbuf_len(m); len; d += l, len -= l) {
                l = MIN(len, PAGE_SIZE);
                l = MIN(next_page(d), d + l) - d;
                
                if (segIndex < maxSegs) {
                    srcRange[segIndex].address = d;
                    srcRange[segIndex].length = l;
                } else {
                    segIndex = 0;
                    goto done;
                }
                segIndex++;
            }
            m = mbuf_next(m);
        } while (m);
map:
        /*
         * Get IORanges, fill in the virtual segments and grab
         * an IOMemoryDescriptor to map the packet.
         */
        if (txMapInfo->txNumFreeMem > 1) {
            dstRange = &txMapInfo->txMemRange[txNextDescIndex];
            
            for (i = 0; i < segIndex; i++) {
                dstRange[i].address = (srcRange[i].address & ~PAGE_MASK);
                dstRange[i].length = PAGE_SIZE;
                srcRange[i].address &= PAGE_MASK;
            }
            OSAddAtomic16(-1, &txMapInfo->txNumFreeMem);
            saveMem = txMapInfo->txNextMem2Use++;
            txMapInfo->txNextMem2Use &= kTxMemDescMask;
            md = txMapInfo->txMemIO[saveMem];
            
            if (md) {
                result = md->initWithOptions(dstRange, segIndex, 0, kernel_task, (kIOMemoryTypeVirtual | kIODirectionOut | kIOMemoryAsReference), mapper);
            } else {
                md = IOMemoryDescriptor::withAddressRanges(dstRange, segIndex, (kIOMemoryTypeVirtual | kIODirectionOut | kIOMemoryAsReference), kernel_task);
                
                if (!md) {
                    DebugLog("Couldn't alloc IOMemoryDescriptor for tx packet.");
                    goto error_map;
                }
                txMapInfo->txMemIO[saveMem] = md;
                result = true;
            }
            if (!result) {
                DebugLog("Failed to init IOMemoryDescriptor for tx packet.");
                goto error_map;
            }
            if (md->prepare() != kIOReturnSuccess) {
                DebugLog("Failed to prepare() tx packet.");
                goto error_map;
            }
            md->setTag(kIOMemoryActive);
            offset = 0;

            /*
             * Get the physical segments and fill in the vector.
             */
            for (i = 0; i < segIndex; i++) {
                vector[i].location = md->getPhysicalSegment(offset, NULL) + srcRange[i].address;
                vector[i].length = srcRange[i].length;

                //DebugLog("Phy. Segment %u addr: %llx, len: %llu\n", i, vector[i].location, vector[i].length);
                offset += PAGE_SIZE;
            }
        }
    }
    
done:
    return segIndex;

error_map:
    txMapInfo->txNextMem2Use = saveMem;
    OSAddAtomic16(1, &txMapInfo->txNumFreeMem);

    segIndex = 0;
    goto done;
}

/*
 * Unmap a tx packet. Complete the IOMemoryDecriptor and free it
 * for reuse.
 */
void AtherosE2200::txUnmapPacket()
{
    IOMemoryDescriptor *md = txMapInfo->txMemIO[txMapInfo->txNextMem2Free];
    
    md->complete();
    md->setTag(kIOMemoryInactive);
    
    ++(txMapInfo->txNextMem2Free) &= kTxMemDescMask;
    OSAddAtomic16(1, &txMapInfo->txNumFreeMem);
}

#pragma mark --- rx methods for AppleVTD support ---

/*
 * Unmap a batch of rx buffers, replace them with new ones and map them.
 * @ring        The ring to map for
 * @index       The index of the first buffer in a batch to map.
 * @count       Number of batches to map.
 * @result      The index of the next batch to map.
 */
UInt16 AtherosE2200::rxMapBuffers(UInt16 index, UInt16 count, bool update)
{
    IOPhysicalAddress pa;
    IOMemoryDescriptor *md;
    IOByteCount offset;
    UInt32 batch = count;
    UInt16 end, i;
    UInt16 pidx = 0;
    bool result;
    
    while (batch--) {
        /*
         * Get the coresponding IOMemoryDescriptor and complete
         * the mapping;
         */
        md = rxMapInfo->rxMemIO[index >> kRxMemBaseShift];
        md->complete();
        
        /*
         * Update IORanges with the addresses of the replaced buffers.
         */
        for (i = index, end = index + kRxMemBatchSize; i < end; i++) {
            if (rxBufArray[i].phyAddr == 0) {
                rxMapInfo->rxMemRange[i].address = (IOVirtualAddress)mbuf_datastart(rxBufArray[i].mbuf);
                rxMapInfo->rxMemRange[i].length = PAGE_SIZE;
            }
        }
        /*
         * Prepare IOMemoryDescriptor with updated buffer addresses.
         */
        result = md->initWithOptions(&rxMapInfo->rxMemRange[index], kRxMemBatchSize, 0, kernel_task, kIOMemoryTypeVirtual | kIODirectionIn | kIOMemoryAsReference, mapper);

        if (!result) {
            IOLog("Failed to reinit rx IOMemoryDescriptor.\n");
            goto done;
        }
        if (md->prepare() != kIOReturnSuccess) {
            IOLog("Failed to prepare rx IOMemoryDescriptor.\n");
            goto done;
        }
        /*
         * Get physical addresses of the buffers and update buffer info,
         * as well as the descriptor ring with new addresses.
         */
        offset = 0;

        for (i = index, end = index + kRxMemBatchSize; i < end; i++) {
            pa = md->getPhysicalSegment(offset, NULL);
            rxBufArray[i].phyAddr = pa;
            
            rxFreeDescArray[i].addr = OSSwapHostToLittleInt64(pa);

            //DebugLog("rxFreeDescArray[%u]: %llx\n", i, pa);
            offset += PAGE_SIZE;
        }
        wmb();
        
next_batch:
        pidx = index + (kRxMemBatchSize - 2);
        index = (index + kRxMemBatchSize) & kRxDescMask;
        rxMapNextIndex = index;
        
        if (update) {
            /*
             * Update index of last rx buffer to be reused by hardware.
             */
            //DebugLog("Update ALX_RFD_PIDX=%u, rxMapNextIndex=%u\n", pidx, rxMapNextIndex);

            alxWriteMem16(ALX_RFD_PIDX, pidx);
        }
    }

done:
    return index;
}

UInt32 AtherosE2200::rxInterruptVTD(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    QCARxRetDesc *desc = &rxRetDescArray[rxNextDescIndex];
    mbuf_t bufPkt, newPkt;
    mbuf_t extraPkt, tailPkt;
    UInt64 addr;
    UInt32 status0, status2, status3;
    UInt32 pktSize;
    UInt32 validMask;
    SInt32 extraSize;
    UInt16 goodPkts = 0;
    UInt16 index;
    UInt16 num;
    UInt16 vlanTag;
    bool replaced;
    
    //DebugLog("rxInterrupt()\n");
    
    while (((status3 = OSSwapLittleToHostInt32(desc->word3)) & RRD_UPDATED) && (goodPkts < maxCount)) {
        status0 = OSSwapLittleToHostInt32(desc->word0);
        status2 = OSSwapLittleToHostInt32(desc->word2);
        pktSize = (status3 & RRD_PKTLEN_MASK) - kIOEthernetCRCSize;
        index = (status0 >> RRD_SI_SHIFT) & RRD_SI_MASK;
        vlanTag = (status3 & RRD_VLTAGGED) ? OSSwapBigToHostInt16(status2 & RRD_VLTAG_MASK) : 0;
        bufPkt = rxBufArray[index].mbuf;
        extraSize = pktSize - kRxBufferSize;

        num = ((status0 >> RRD_NOR_SHIFT) & 0x000F);
        rxCleanedCount += num;
        
        DebugLog("Packet with index=%u, num=%u, pktSize=%u status3=0x%x\n", index, num, pktSize, status3);
                
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
            rxBufArray[index].mbuf = bufPkt;
            rxBufArray[index].phyAddr = 0;
        }
        while (extraSize > 0) {
            ++index &= kRxDescMask;

            bufPkt = rxBufArray[index].mbuf;
            extraPkt = rxPool->replaceOrCopyPacket(&bufPkt, kRxBufferSize, &replaced);
            
            if (!extraPkt) {
                /* Allocation of a new packet failed so that we must leave the original packet in place. */
                DebugLog("replaceOrCopyPacket() failed.\n");
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
                rxBufArray[index].phyAddr = 0;
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
    }
    wmb();
    
    /*
     * Map complete batches of cleaned rx buffers if at least
     * one batch is ready to be mapped.
     */

    if (rxCleanedCount > kRxMemBatchSize) {
        num = (rxCleanedCount >> kRxMemBaseShift);
        rxCleanedCount -= (num << kRxMemBaseShift);

        DebugLog("rxMapBuffers(): rxMapNextIndex=%u, num=%u, rxCleanedCount=%u\n", rxMapNextIndex, num, rxCleanedCount);

        rxMapBuffers(rxMapNextIndex, num, true);
    }
    return goodPkts;
}
