/* AtherosE2200Ethernet.h -- AtherosE2200 driver class definition.
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

#include "reg.h"
#include "hw.h"

#ifdef DEBUG
#define DebugLog(args...) IOLog(args)
#else
#define DebugLog(args...)
#endif

#define	RELEASE(x)	if(x){(x)->release();(x)=NULL;}

#define alxWriteMem8(reg, val8)     _OSWriteInt8((baseAddr), (reg), (val8))
#define alxWriteMem16(reg, val16)   OSWriteLittleInt16((baseAddr), (reg), (val16))
#define alxWriteMem32(reg, val32)   OSWriteLittleInt32((baseAddr), (reg), (val32))
#define alxReadMem8(reg)            _OSReadInt8((baseAddr), (reg))
#define alxReadMem16(reg)           OSReadLittleInt16((baseAddr), (reg))
#define alxReadMem32(reg)           OSReadLittleInt32((baseAddr), (reg))
#define alxPostWrite()              OSReadLittleInt32((baseAddr), 0)

#define super IOEthernetController

enum
{
	MEDIUM_INDEX_AUTO = 0,
	MEDIUM_INDEX_10HD,
	MEDIUM_INDEX_10FD,
	MEDIUM_INDEX_100HD,
	MEDIUM_INDEX_100FD,
	MEDIUM_INDEX_100FDFC,
    MEDIUM_INDEX_100FDEEE,
    MEDIUM_INDEX_100FDFCEEE,
	MEDIUM_INDEX_1000FD,
	MEDIUM_INDEX_1000FDFC,
    MEDIUM_INDEX_1000FDEEE,
    MEDIUM_INDEX_1000FDFCEEE,
	MEDIUM_INDEX_COUNT
};

#define MBit 1000000

enum {
    kSpeed1000MBit = 1000*MBit,
    kSpeed100MBit = 100*MBit,
    kSpeed10MBit = 10*MBit,
};

enum {
    kEEETypeNo = 0,
    kEEETypeYes = 1,
    kEEETypeCount
};

enum {
    kChipUnkown = 0,
    kChipAR8161,
    kChipAR8162,
    kChipAR8171,
    kChipAR8172,
    kChipKillerE2200,
    kChipKillerE2400,
    kChipKillerE2500,
    kNumChips
};

/* Transmit descriptor. */
typedef struct QCATxDesc {
    UInt16 length;
    UInt16 vlanTag;
    UInt32 word1;
    union {
        UInt64 addr;
        struct {
            UInt32 pktLength;
            UInt32 reserved;
        } l;
    } adrl;
} QCATxDesc;

/* Receive Return descriptor */
typedef struct QCARxRetDesc {
    UInt32 word0;
    UInt32 rssHash;
    UInt32 word2;
    UInt32 word3;
} QCARxRetDesc;

#define TPD_IP_XSUM (1 << TPD_IP_XSUM_SHIFT)
#define TPD_TCP_XSUM (1 << TPD_TCP_XSUM_SHIFT)
#define TPD_UDP_XSUM (1 << TPD_UDP_XSUM_SHIFT)
#define TPD_LSO_EN (1 << TPD_LSO_EN_SHIFT)
#define TPD_LSO_V2 (1 << TPD_LSO_V2_SHIFT)
#define TPD_INS_VLTAG (1 << TPD_INS_VLTAG_SHIFT)
#define TPD_IPV4 (1 << TPD_IPV4_SHIFT)
#define TPD_EOP (1 << TPD_EOP_SHIFT)
#define TPD_ETHTYPE (1 << TPD_ETHTYPE_SHIFT)

#define RRD_UPDATED (1 << RRD_UPDATED_SHIFT)
#define RRD_VLTAGGED (1 << RRD_VLTAGGED_SHIFT)
#define RRD_ERR_L4 (1 << RRD_ERR_L4_SHIFT)
#define RRD_ERR_IPV4 (1 << RRD_ERR_IPV4_SHIFT)
#define RRD_ERR_FCS (1 << RRD_ERR_FCS_SHIFT)
#define RRD_ERR_FAE (1 << RRD_ERR_FAE_SHIFT)
#define RRD_ERR_TRUNC (1 << RRD_ERR_TRUNC_SHIFT)
#define RRD_ERR_ICMP (1 << RRD_ERR_ICMP_SHIFT)
#define RRD_ERR_FIFOV (1 << RRD_ERR_FIFOV_SHIFT)
#define RRD_ERR_LEN (1 << RRD_ERR_LEN_SHIFT)
#define RRD_ERR_MASK (RRD_ERR_FCS | RRD_ERR_FAE | RRD_ERR_TRUNC | RRD_ERR_ICMP | RRD_ERR_FIFOV | RRD_ERR_LEN)

#define getProtocolID(x) ((x >> RRD_PID_SHIFT) & RRD_PID_MASK)

/* PHY Specific Status Register */
#define ALX_GIGA_PSSR_FC_RXEN   0x0004
#define ALX_GIGA_PSSR_FC_TXEN   0x0008
#define ALX_GIGA_PSSR_FC_MASK   (ALX_GIGA_PSSR_FC_RXEN | ALX_GIGA_PSSR_FC_TXEN)

/* Receive Free descriptor. */
typedef struct QCARxFreeDesc {
    UInt64 addr;
} QCARxFreeDesc;

#define kTransmitQueueCapacity  1024

/* With up to 40 segments we should be on the save side. */
#define kMaxSegs 40

/* The number of descriptors must be a power of 2. */
#define kNumTxDesc      1024    /* Number of Tx descriptors */
#define kNumRxDesc      512     /* Number of Rx descriptors */
#define kTxLastDesc    (kNumTxDesc - 1)
#define kRxLastDesc    (kNumRxDesc - 1)
#define kTxDescMask    (kNumTxDesc - 1)
#define kRxDescMask    (kNumRxDesc - 1)
#define kTxDescSize    (kNumTxDesc*sizeof(QCATxDesc))
#define kRxRetDescSize    (kNumRxDesc*sizeof(QCARxRetDesc))
#define kRxFreeDescSize    (kNumRxDesc*sizeof(QCARxFreeDesc))

/* Complete Descriptor array */
typedef struct QCARxTxDescArray {
    QCATxDesc txDesc[kNumTxDesc];
    QCARxRetDesc rxRetDesc[kNumRxDesc];
    QCARxFreeDesc rxFreeDesc[kNumRxDesc];
} QCARxTxDescArray;

/* This is the receive buffer size (must be exactly 2048 bytes to match a cluster). */
#define kRxBufferPktSize 2048
#define kRxNumSpareMbufs 100
#define kMCFilterLimit 32
#define kMaxRxQueques 1
#define kMaxMtu 9000
#define kMaxPacketSize (kMaxMtu + ETH_HLEN + ETH_FCS_LEN)
#define kMaxTsoMtu 7000

/* statitics timer period in ms. */
#define kTimeoutMS 1000

/* Treshhold value to wake a stalled queue */
#define kTxQueueWakeTreshhold (kNumTxDesc / 4)

/* transmitter deadlock treshhold in seconds. */
#define kTxDeadlockTreshhold 5

/* IP specific stuff */
#define kMinL4HdrOffsetV4 34
#define kMinL4HdrOffsetV6 54

#define ALX_RSS_BASE_CPU_NUM            0x15B8

/* These definitions should have been in IOPCIDevice.h. */
enum
{
    kIOPCIPMCapability = 2,
    kIOPCIPMControl = 4,
};

enum
{
    kIOPCIEDevCapability = 4,
    kIOPCIEDeviceControl = 8,
    kIOPCIELinkCapability = 12,
    kIOPCIELinkControl = 16,
};

enum
{
    kIOPCIELinkCtlASPM = 0x0003,    /* ASPM Control */
    kIOPCIELinkCtlL0s = 0x0001,     /* L0s Enable */
    kIOPCIELinkCtlL1 = 0x0002,      /* L1 Enable */
    kIOPCIELinkCtlCcc = 0x0040,     /* Common Clock Configuration */
    kIOPCIELinkCtlClkReqEn = 0x100, /* Enable clkreq */
};

enum
{
    kIOPCIEDevCtlReadQ = 0x7000,
};

#define kALXPCICommand (kIOPCICommandBusMaster | kIOPCICommandMemorySpace | kIOPCICommandIOSpace)

enum
{
    kPowerStateOff = 0,
    kPowerStateOn,
    kPowerStateCount
};

#define kEnableCSO6Name "enableCSO6"
#define kEnableTSO4Name "enableTSO4"
#define kEnableTSO6Name "enableTSO6"
#define kIntrRateName "maxIntrRate"
#define kDriverVersionName "Driver_Version"
#define kNameLenght 64

#ifdef __PRIVATE_SPI__

#define kEnableRxPollName "rxPolling"

#endif /* __PRIVATE_SPI__ */

class AtherosE2200 : public super
{
	
	OSDeclareDefaultStructors(AtherosE2200)
	
public:
	/* IOService (or its superclass) methods. */
	virtual bool start(IOService *provider);
	virtual void stop(IOService *provider);
	virtual bool init(OSDictionary *properties);
	virtual void free();
	
	/* Power Management Support */
	virtual IOReturn registerWithPolicyMaker(IOService *policyMaker);
    virtual IOReturn setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker );
	virtual void systemWillShutdown(IOOptionBits specifier);
    
	/* IONetworkController methods. */
	virtual IOReturn enable(IONetworkInterface *netif);
	virtual IOReturn disable(IONetworkInterface *netif);
	
#ifdef __PRIVATE_SPI__
    virtual IOReturn outputStart(IONetworkInterface *interface, IOOptionBits options );
    virtual IOReturn setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled);
    virtual void pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context);
#else
    virtual UInt32 outputPacket(mbuf_t m, void *param);
#endif /* __PRIVATE_SPI__ */
	
	virtual void getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const;
	
	virtual IOOutputQueue* createOutputQueue();
	
	virtual const OSString* newVendorString() const;
	virtual const OSString* newModelString() const;
	
	virtual IOReturn selectMedium(const IONetworkMedium *medium);
	virtual bool configureInterface(IONetworkInterface *interface);
	
	virtual bool createWorkLoop();
	virtual IOWorkLoop* getWorkLoop() const;
	
	/* Methods inherited from IOEthernetController. */
	virtual IOReturn getHardwareAddress(IOEthernetAddress *addr);
	virtual IOReturn setHardwareAddress(const IOEthernetAddress *addr);
	virtual IOReturn setPromiscuousMode(bool active);
	virtual IOReturn setMulticastMode(bool active);
	virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count);
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput);
    virtual IOReturn getMaxPacketSize(UInt32 * maxSize) const;
    virtual IOReturn setMaxPacketSize(UInt32 maxSize);
    virtual IOReturn setWakeOnMagicPacket(bool active);
    virtual IOReturn getPacketFilters(const OSSymbol *group, UInt32 *filters) const;
    
    virtual UInt32 getFeatures() const;
    
private:
    bool initPCIConfigSpace(IOPCIDevice *provider);
    bool alxResetPCIe();
    static IOReturn setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);
    static IOReturn setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);
    void getParams(UInt32 *intrLimit);
    bool setupMediumDict();
    bool initEventSources(IOService *provider);
    void interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count);
    void txInterrupt();
    
#ifdef __PRIVATE_SPI__
    UInt32 rxInterrupt(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context);
#else
    void rxInterrupt();
#endif /* __PRIVATE_SPI__ */

    bool setupDMADescriptors();
    void freeDMADescriptors();
    void txClearDescriptors();
    void checkLinkStatus();
    void updateStatitics();
    void setLinkUp();
    void setLinkDown();
    bool checkForDeadlock();
    
    /* Hardware specific methods */
    IOReturn alxSetHardwareAddress(const IOEthernetAddress *addr);
    bool alxLoadDefaultAddress();
    bool alxStart(UInt32 maxIntrRate);
    void alxEnable();
    int alxDisable();
    void alxRestart();
    bool alxIdentifyChip();
    void alxInitDescRings();
    void alxConfigure();
    void alxConfigureBasic();
    void alxConfigureRSS(bool enable);
    inline void alxEnableIRQ();
    inline void alxDisableIRQ();
    inline void alxGetChkSumCommand(UInt32 *cmd, mbuf_csum_request_flags_t checksums);
    int alxReadPhyLink();
    void alxResetPhy();
    void alxPostPhyLink();
    int alxSetupSpeedDuplex(UInt32 ethadv, UInt16 eeeadv, UInt8 flowctrl);
    int alxSelectPowersavingSpeed(int *speed, UInt8 *duplex);
    void alxSpeedDuplexForMedium(const IONetworkMedium *medium);

    /* timer action */
    void timerAction(IOTimerEventSource *timer);
    
private:
	IOWorkLoop *workLoop;
    IOCommandGate *commandGate;
	IOPCIDevice *pciDevice;
	OSDictionary *mediumDict;
	IONetworkMedium *mediumTable[MEDIUM_INDEX_COUNT];
	IOBasicOutputQueue *txQueue;
	
	IOInterruptEventSource *interruptSource;
	IOTimerEventSource *timerSource;
	IOEthernetInterface *netif;
	IOMemoryMap *baseMap;
    volatile void *baseAddr;
    
    /* transmitter data */
    IOBufferMemoryDescriptor *bufDesc;
    IOPhysicalAddress64 txPhyAddr;
    QCATxDesc *txDescArray;
    IOMbufNaturalMemoryCursor *txMbufCursor;
    UInt64 txDescDoneCount;
    UInt64 txDescDoneLast;
    SInt32 txNumFreeDesc;
    
#ifndef __PRIVATE_SPI__
    UInt32 txStallCount;
    UInt32 txStallLast;
#endif /* __PRIVATE_SPI__ */

    UInt16 txNextDescIndex;
    UInt16 txDirtyDescIndex;
    
    /* receiver data */
    IOPhysicalAddress64 rxRetPhyAddr;
    IOPhysicalAddress64 rxFreePhyAddr;
    QCARxRetDesc *rxRetDescArray;
    QCARxFreeDesc *rxFreeDescArray;
	IOMbufNaturalMemoryCursor *rxMbufCursor;
    UInt32 multicastFilter[2];
    UInt16 rxNextDescIndex;
    
    /* EEE support */
    UInt16 eeeCap;
    UInt16 eeeAdv;
    UInt16 eeeLpa;
    UInt16 eeeEnable;
    
    /* power management data */
    unsigned long powerState;
    
    /* statistics data */
    UInt32 deadlockWarn;
    IONetworkStats *netStats;
	IOEthernetStats *etherStats;
    
    UInt32 chip;
    UInt32 intrMask;
    
#ifdef __PRIVATE_SPI__
    UInt32 linkOpts;
    IONetworkPacketPollingParameters pollParams;
#endif /* __PRIVATE_SPI__ */

    struct alx_hw hw;
    struct pci_dev pciDeviceData;
    struct IOEthernetAddress currMacAddr;
    struct IOEthernetAddress origMacAddr;
    UInt8 pcieCapOffset;
    UInt8 pciPMCtrlOffset;
    UInt8 flowControl;
    
    /* flags */
    bool isEnabled;
	bool promiscusMode;
	bool multicastMode;
    bool linkUp;
    
#ifdef __PRIVATE_SPI__
    bool rxPoll;
    bool polling;
#else
    bool stalled;
#endif /* __PRIVATE_SPI__ */
    
    bool useMSI;
    bool gbCapable;;
    bool wolCapable;
    bool enableTSO4;
    bool enableTSO6;
    bool enableCSO6;
    
    /* mbuf_t arrays */
    mbuf_t txMbufArray[kNumTxDesc];
    mbuf_t rxMbufArray[kNumRxDesc];
    
#ifdef CONFIG_RSS
    
    /* RSS parameter */
	//enum alx_rss_mode rss_mode;
	UInt32 rssIdt[32];
	UInt16 rssIdtSize;
	UInt8 rssHashType;
	UInt8 rssBaseCPU;

#endif  /* CONFIG_RSS */
};
