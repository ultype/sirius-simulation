Advantech DAQNavi Linux Drivers
Date: 10/2017

The Readme file contains the following section:

1. What's new
2. Installation
3. Supported distribution
4. Driver note
5. Hardware Support for Linux
6. System requirements

1. What's new

DAQNavi_SDK:
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1. Modified:Bug 10000: compiling error on linux kernel 4.11.         | libiodaq.so: V3.1.15.0   |
|               |              | 2. Modified:fix bug: Add coupling type and IEPE type ANSI-C API      |                          |
|               |              | 3. Modified:fix bug: compiling error for cross-compiling: amd64      |                          |
|               |              | 4. Modified:Bug 10657.Modify the return value of                     |                          |
|               |              |    AdxGetValueRangeInformation                                       |                          |
|               |              | 5. Modified:Bug 10216. Missing some AI channel methods,              |                          |
|               |              |    such as IEPE and CouplingType.                                    |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1. Modified:fix bug: Compile error in linux environment              | libiodaq.so: V3.1.14.0   |
--------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    3.2.7.0    |  2015-09-01  | 1. Modified:Bug 17996.Add parameter verify for method: di/do         | libiodaq.so: V3.1.13.0   |
|               |              |    read/write bit                                                    |                          |
|               |              | 2. Added: ModeWriteShared.                                           |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.6.0    |  2015-07-15  | 1. Modified:Bug 17206.add noise filter setting:blocking time.        | libiodaq.so: V3.1.12.0   |
|               |              | 2. Modified:Bug 17485.Add CouplingType and IEPE to AiFeature and     |                          |
|               |              |     AIChannel                                                        |                          |
|               |              | 3. Modified:Bug 17509.Add property id from trigger                   |                          |
|               |              | 3. Modified:Bug 17744.Modify Coupling&IEPE Property's definition     |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1. Modified:Bug 16823.fix fileno reveal.                             | libiodaq.so: V3.1.11.0   |
|               |              | 2. Modified:Bug 16984.modify DIO programmable method for devices     |                          |
|               |              |     which DIO ports have different programmable feature              |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1. added device module property/method.(Normal)                      | libiodaq.so: V3.1.10.0   |
|               |              | 2. Modified:Modify return ErrorCode when property CountingType is set|                          |
|               |              |    to invalid value                                                  |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1. Add: Dio read and write bit port function.(Normal)                | libiodaq.so: V3.1.9.0    |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1. Add: CSCL APIs for ANSI-C.(Normal)                                |                          |
|               |              | 2. Add: "ErrorFuncConflictWithBfdAi" and                             |                          |
|               |              |    "ErrorVrgNotAvailableInSeMode" Error code.(Normal)                | libiodaq.so: V3.1.8.0    |
|               |              | 3. Add: Add double trigger example.(Normal)                          |                          |
|               |              | 4. Modified: When driver load failed,will return right error code.   |                          |
|               |              |    (Normal)                                                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.7.0    |  2013-07-15  | 1. Add: Double trigger feature.(Normal)                              |                          |
|               |              | 2. Add: Add counter cache for get last data when disconnection.      |                          |
|               |              |    (Normal)                                                          | libiodaq.so: V3.1.7.0    |
|               |              | 3. Modified: Raising overrun and cache overflow event priority.      |                          |
|               |              |    (Normal)                                                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.6.0    |  2013-05-15  | 1. Add: example add Counter_ContinueCompare,Counter_SnapCounter,     |                          |
|               |              |    Counter_UpDownCounter.(Normal)                                    | libiodaq.so: V3.1.6.0    |
|               |              | 2. Modified: fix qt and c++ example bug.(Normal)                     |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.5.0    |  2013-03-15  | 1. Modified: fix qt example bug.(Normal)                             | libiodaq.so: V3.1.5.0    |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.4.0    |  2013-01-15  | 1. Modified: Adjust the header file.(Normal)                         | libiodaq.so: V3.1.4.0    |
|               |              | 2. Add: Add remaining qt examples.                                   |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2012-11-15  | 1. noise filter interface change.                                    |                          |
|               |              | 2. fix cscl thread lock bug.                                         |                          |
|               |              | 3. add feature for PCI1784.                                          | libiodaq.so: V3.1.3.0    |
|               |              | 4. fix multi BufferedAiCtrl's event will receive confusion.          |                          |
|               |              | 5. add up-down counting property.                                    |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2012-09-15  | 1. Fix triggerEdges feature bug in integration layer.                |                          |
|               |              | 2. Disable TimerEvent OneShotEvent and  OverflowEvent feature.       | libiodaq.so: V3.1.2.0    |
|               |              | 3. fix Linux CSCL open device bug.                                   |                          |
|               |              | 4. Modify the Examples bugs.                                         |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | libiodaq.so: V3.1.1.0    |
|               |              | 2. Support Linux Mint12,Redhat Enterprise Linux6.2.                  |                          |
|               |              | 3. Modify the Examples bugs.                                         |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Linux DAQNavi SDK Initial version.                                | libiodaq.so: V3.0.0100   |
|               |              | 2. Add Java,Qt,C++ examples for Linux.                               |                          |
|               |              | 3. Support Ubuntu10.04,Fedora16,OpenSUSE11.4                         |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------


DAQNavi_PCI1706:support PCI-1706
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1706.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1706.so: V3.1.3.0  |
|               |              | 3.Modified:bug 9679, compiling error due lib api:                    |                          |
|               |              |            daq_umem_alloc/daq_umem_free                              |                          |
|               |              | 4.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 5.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 20240.fix BufferedAI can't start work ,               | bio1706.ko: V3.1.3.0     |
|               |              |     and crashed error.                                               | libbio1706.so: V3.1.2.0  |
|               |              | 2.Modified the thread-safe bugs.                                     |                          |
|               |              | 3.Modified:Bug 19974: Modify the device description bug.             |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16566.fix InstantAO output data error.                | bio1706.ko: V3.1.2.0     |
|               |              | 2.Modified:Bug 16833.Add DO Write Bit Function.                      | libbio1706.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Added: Initial version.                                            | bio1706.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1706.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1711:support PCI-1711/L
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1711.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1711.so: V3.1.7.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1711.ko: V3.1.5.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1711.so: V3.1.6.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1711.ko: V3.1.4.0     |
|               |              | 2.Modified:Bug 16566.fix InstantAO output data error.                | libbio1711.so: V3.1.6.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 15214. Fix data count error in OneBufferedAI when    | bio1711.ko: V3.1.3.0     |
|               |              |             intervalCount is FIF0/2 and Sample is non integer times  | libbio1711.so: V3.1.5.0  |
|               |              |             to it .(Normal)                                          |                          |
|               |              | 2.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14651. Fix Readback ai convert frequency do not equal| bio1711.ko: V3.1.2.0     |
|               |              |             actually frequency after user set some frequency.(Normal)| libbio1711.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Fix Delayed Pulse Generation function error.(Critical)   | bio1711.ko: V3.1.2.0     |
|               |              | 2.Modified: Set AO external reference value invalid.(Critical)       | libbio1711.so: V3.1.3.0  |
|               |              | 3.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-07-15  | 1.Modified: Fix frequency measurement method error.(Critical)        | bio1711.ko: V3.1.1.0     |
|               |              | 2.Modified: Fix AO configure can not reset problem.(Normal)          | libbio1711.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-01-15  | 1.Added: Initial version.                                            | bio1711.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1711.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1713:support PCI-1713/U
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1713.ko: V3.1.3.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1713.so: V3.1.2.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio1713.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1713.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  |  1.Added: Initial version.                                           | bio1713.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1713.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------


DAQNavi_PCI1714_PCIE1744:support PCI-1714/UL,PCIE-1744,MIC-3714
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1714.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1714.so: V3.1.6.0  |
|               |              | 3.Modified:bug 9592, Wrong calibration was loaded when change the    |                          |
|               |              |            channel's value range                                     |                          |
|               |              | 4.Modified:bug 10259, wrong trigger edge was used.                   |                          |
|               |              | 5.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 6.Modified:bug 9679, compiling error due lib api:                    |                          |
|               |              |            daq_umem_alloc/daq_umem_free                              |                          |
|               |              | 7.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 8.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio1714.ko: V3.1.4.0     |
|               |              |                                                                      | libbio1714.so: V3.1.5.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      | bio1714.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1714.so: V3.1.5.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14651. Fix Readback ai convert frequency do not equal| bio1714.ko: V3.1.3.0     |
|               |              |             actually frequency after user set some frequency.(Normal)| libbio1714.so: V3.1.4.0  |
--------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Bug 13242. Program maybe get data before DelayToStart    | bio1714.ko: V3.1.3.0     |
|               |              |    trigger.(Critical)                                                | libbio1714.so: V3.1.3.0  |
|               |              | 2.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-03-15  | 1.Modified: Bug 10769. fix system crash error on 64bit OS.(Critical) | bio1714.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1714.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-01-15  | 1.Added: Initial version.                                            | bio1714.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1714.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------


DAQNavi_PCI1715:support PCI-1715
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1715.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1715.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio1715.ko: V3.1.4.0     |
|               |              |                                                                      | libbio1715.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 15214. Fix data count error in OneBufferedAI when    | bio1715.ko: V3.1.3.0     |
|               |              |             intervalCount is FIF0/2 and Sample is non integer times  | libbio1715.so: V3.1.3.0  |
|               |              |             to it .(Normal)                                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14651. Fix Readback ai convert frequency do not equal| bio1715.ko: V3.1.2.0     |
|               |              |             actually frequency after user set some frequency.(Normal)| libbio1715.so: V3.1.3.0  |
--------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Optimize FAI stopping process.(Normal)                   | bio1715.ko: V3.1.2.0     |
|               |              | 2.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      | libbio1715.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

|    3.1.1.0    |  2013-01-15  | 1.Added: Initial version.                                            | bio1715.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1715.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1716:support PCI1716/L,MIC-3716
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1716.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1716.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1716.ko: V3.1.4.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1716.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1716.ko: V3.1.3.0     |
|               |              | 1.Modified:Bug 16566.fix InstantAO output data error.                | libbio1716.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 15821. Fix data read by InstantAI function is        | bio1716.ko: V3.1.2.0     |
|               |              |             instable                                                 | libbio1716.so: V3.1.2.0  |
|               |              | 2.Modified: Bug 15214. Fix data count error in OneBufferedAI when    |                          |
|               |              |             intervalCount is FIF0/2 and Sample is non integer times  |                          |
|               |              |             to it .(Normal)                                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14651. Fix Readback ai convert frequency do not equal| bio1716.ko: V3.1.1.0     |
|               |              |             actually frequency after user set some frequency.(Normal)| libbio1716.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Added: Initial version.                                            | bio1716.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1716.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1720:support PCI-1720
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1720.ko: V3.1.3.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1720.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 20099.Modify the BUILT_MODULE_NAME.                   | bio1720.ko: V3.1.2.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1720.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16566.fix InstantAO output data error.                | bio1720.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1720.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Added: Initial version.                                            | bio1720.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1720.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1721:support PCI-1721
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1721.ko: V3.1.10.0    |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1721.so: V3.1.9.0  |
|               |              | 3.Modified:bug 9679, compiling error due lib api:                    |                          |
|               |              |            daq_umem_alloc/daq_umem_free                              |                          |
|               |              | 4.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 5.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1721.ko: V3.1.9.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1721.so.3.1.8.0    |
--------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    3.2.7.0    |  2015-09-05  | 1.Modified: BUG 18281. AO Updating Rate Cannot Be Set Higher than    | bio1721.ko: V3.1.8.0     |
|               |              |             1MHz in Linux                                            | libbio1721.so.3.1.8.0    |
--------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1721.ko: V3.1.8.0     |
|               |              | 1.Modified:Bug 16566.fix InstantAO output data error.                | libbio1721.so: V3.1.7.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 16114. Fix Transmitted Event can not be sent.(Normal)| bio1721.ko: V3.1.7.0     |
|               |              | 2.Modified: Bug 14759. Fix program DIO will excite a pules,when      | libbio1721.so: V3.1.6.0  |
|               |              |             execute reset function.(Normal)                          |                          |
|               |              | 3.Modified: Bug 13979. Add thread locker for FAO event.(Normal)      |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14651. Fix Readback ao convert frequency do not equal| bio1721.ko: V3.1.7.0     |
|               |              |             actually frequency after user set some frequency.(Normal)| libbio1721.so: V3.1.5.0  |
|               |              | 2.Modified: Bug 14759. Fix program DIO will excite a pules,when      |                          |
|               |              |             execute reset function.(Normal)                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Add check FAO stopping timeout.(Normal)                  | bio1721.ko: V3.1.7.0     |
|               |              | 2.Modified: Bug 13979. Add thread locker for FAO event.(Normal)      | libbio1721.so: V3.1.4.0  |
|               |              | 3.Modified: Bug 13268. Data maybe output error under StreamingAO mode|                          |
|               |              |   .(Critical)                                                        |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.6.0    |  2013-07-15  | 1.Modified: Fix buffered AO stop and start again AO output error, if | bio1721.ko: V3.1.6.0     |
|               |              |   do not exist problem.(Normal)                                      | libbio1721.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.5.0    |  2013-05-15  | 1.Modified: Bug 12114. Fix buffered AO will bluescreen in some time. | bio1721.ko: V3.1.5.0     |
|               |              |   (Critical)                                                         | libbio1721.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.4.0    |  2013-03-15  | 1.Modified: one-shot can't change to extern clock. (Normal)          | bio1721.ko: V3.1.4.0     |
|               |              |                                                                      | libbio1721.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2013-01-15  | 1.Modified: Bug 10769. fix dma transfer error on 64bit OS. (Critical)| bio1721.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1721.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2012-11-15  | 1. fix compiler warning.                                             | bio1721.ko: V3.1.2.0     |
|               |              | 2. counter must reset before device close.                           | libbio1721.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | bio1721.ko: V3.1.1.0     |
|               |              | 2. fixed general bugs.(bug 10903, 10908).                            | libbio1721.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Initial version.                                                  | bio1721.ko: V3.0.0100    |
|               |              |                                                                      | libbio1721.so: V3.0.0100 |
 --------------- -------------- ----------------------------------------------------------------------- --------------------------

 DAQNavi_PCI1723:support PCI-1723, MIC-3723
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1723.ko: V3.1.2.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1723.so: V3.1.2.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Initial version.                                                   | bio1723.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1723.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1724:support PCI-1724
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1724.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1724.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio1724.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1724.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.9.0    |  2016-07-14  | 1.Modified:Bug 19788.fix InstantAO initial value error.              | bio1724.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1724.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16566.fix InstantAO output data error.                | bio1724.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1724.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1724.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1724.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Remove unused variables.(Normal)                         | bio1724.ko: V3.1.2.0     |
|               |              | 2.Modified: Re-arrange the header files structure.(Normal)           | libbio1724.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-11-15  | 1. Initial version.                                                  | bio1724.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1724.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1727:support PCI-1727U
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1727.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1727.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1727.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1727.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1727.ko: V3.1.2.0     |
|               |              | 1.Modified:Bug 16566.fix InstantAO output data error.                | libbio1727.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1727.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1727.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-01-15  | 1.Added: Initial version.                                            | bio1727.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1727.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1730:support PCI-1730
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1730.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1730.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1730.ko: V3.1.5.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1730.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1730.ko: V3.1.4.0     |
|               |              |                                                                      | libbio1730.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1730.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1730.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2013-07-15  | 1. Modified: Port value is not correct when read back DO .(Normal)   | bio1730.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1730.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1. Modified: Re-arrange the header files structure.(Normal)          | bio1730.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1730.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-11-15  | 1. Initial version.                                                  | bio1730.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1730.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

 DAQNavi_PCI1733:support PCI-1733
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1733.ko: V3.1.2.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1733.so: V3.1.2.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Initial version.                                                   | bio1733.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1733.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1734:support PCI-1734
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1734.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1734.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1734.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1734.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1734.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1734.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1734.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1734.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-07-15  | 1.Added: Initial version.                                            | bio1734.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1734.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1737_PCI1739:support PCI-1737,PCI-1739
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Added: Initial version.                                            | bio1737.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1737.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
 
DAQNavi_PCI1741:support PCI-1741
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1741.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1741.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1741.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1741.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1741.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1741.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.1    |  2015-01-07  | 1.Modified:Bug 16566.fix InstantAO output data error.                | bio1741.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1741.so: V3.1.1.1  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Added: Initial version.                                            | bio1741.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1741.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1747:support PCI-1747
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1747.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1747.so: V3.1.5.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio1747.ko: V3.1.4.0     |
|               |              |                                                                      | libbio1747.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 15214. Fix data count error in OneBufferedAI when    | bio1747.ko: V3.1.3.0     |
|               |              |             intervalCount is FIF0/2 and Sample is non integer times  | libbio1747.so: V3.1.4.0  |
|               |              |             to it .(Normal)                                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14651. Fix Readback ai convert frequency do not equal| bio1747.ko: V3.1.2.0     |
|               |              |             actually frequency after user set some frequency.(Normal)| libbio1747.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      | bio1747.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1747.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Bug 10769. fix dma transfer error on 64bit OS. (Critical)| bio1747.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1747.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | bio1747.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1747.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Initial version.                                                  | bio1747.ko: V3.0.0100    |
|               |              |                                                                      | libbio1747.so: V3.0.0100 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1750:support PCI-1750
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1750.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1750.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1750.ko: V3.1.4.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1750.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1750.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1750.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1750.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1750.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Re-arrange the header files structure.(Normal)           | bio1750.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1750.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-11-15  | 1. Initial version.                                                  | bio1750.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1750.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1751:support PCI-1751
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1751.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1751.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 20245.Add board ID support.                           | bio1751.ko: V3.1.3.0     |
|               |              | 2.Modified the thread-safe bugs.                                     | libbio1751.so: V3.1.2.0  |
|               |              | 3.Modified:Bug 19974: Modify the device description bug.             |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1751.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1751.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Added: Initial version.                                            | bio1751.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1751.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------



DAQNavi_PCI1752:support PCI-1752
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1752.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1752.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1752.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1752.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1752.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1752.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1752.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1752.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-01-15  | 1.Added: Initial version.                                            | bio1752.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1752.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1753/PCM3753I:support PCI-1753,PCM-3753I
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1753.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1753.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1753.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1753.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1753.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1753.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14759. Fix program DIO will excite a pules,when      | bio1753.ko: V3.1.1.0     |
|               |              |             execute reset function.(Normal)                          | libbio1753.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Added: Initial version.                                            | bio1753.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1753.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

 DAQNavi_PCI1754:support PCI1754
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-09-14  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1754.ko: V3.1.3.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1754.so: V3.1.2.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1754.ko: V3.1.2.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1754.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1. Initial version.                                                  | bio1754.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1754.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

 DAQNavi_PCI1756:support PCI-1756,MIC-3756
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1756.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1756.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:Bug 8555:Fix wrong error code is returned                 |                          |
|               |              |            when dll driver doesn't match with kernel driver.         |                          |
|               |              | 5.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1756.ko: V3.1.4.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1756.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.7.0    |  2015-6-29   | 1.Added: ModeWriteShared.                                            | bio1756.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1756.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1756.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1756.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Added: Initial version.                                            | bio1756.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1756.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1757:support PCI-1757
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1757.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1757.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1757.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1757.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1757.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1757.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 15268. Fix read&Write EEPROM error                   | bio1757.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1757.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Added: Initial version.                                            | bio1757.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1757.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1758:support PCI-1758UDI,PCI-1758UDIO,PCI-1758UDO
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1758.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1758.so: V3.1.5.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:Bug 8581:Fix DO status would be impacted by other channel.|                          |
|               |              | 5.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1758.ko: V3.1.4.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1758.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  |  1.Modified:Bug 16833.Add Do write bit.                              | bio1758.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1758.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1758.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1758.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2013-01-15  | 1.Modified: Re-arrange the header files structure.(Normal)           | bio1758.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1758.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2012-11-15  | 1. Remove unused variables.                                          | bio1758.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1758.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | bio1758.ko: V3.1.1.0     |
|               |              | 2. Disable Interrupt feature.                                        | libbio1758.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Initial version.                                                  | bio1758.ko: V3.0.0100    |
|               |              |                                                                      | libbio1758.so: V3.0.0100 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1761:support PCI-1761,PCM-3761I,MIC-3761
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1761.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1761.so: V3.1.6.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1761.ko: V3.1.5.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1761.so: V3.1.5.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1761.ko: V3.1.4.0     |
|               |              |                                                                      | libbio1761.so: V3.1.5.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Added: support PCM-3761I.                                          | bio1761.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1761.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1761.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1761.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2013-01-15  | 1.Modified: Re-arrange the header files structure.(Normal)           | bio1761.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1761.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2012-11-15  | 1. Improve performanc.                                               | bio1761.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1761.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-09-15  | 1. Initial version.                                                  | bio1761.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1761.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCI1762:support PCI-1762
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1762.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1762.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1762.ko: V3.1.4.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1762.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio1762.ko: V3.1.3.0     |
|               |              |                                                                      | libbio1762.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio1762.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1762.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Re-arrange the header files structure.(Normal)           | bio1762.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1762.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-11-15  | 1. Initial version.                                                  | bio1762.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1762.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

 DAQNavi_PCI1784:support PCI1784
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1784.ko: V3.1.3.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1784.so: V3.1.2.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio1784.ko: V3.1.2.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio1784.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1. Initial version.                                                  | bio1784.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1784.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCM3810I:support PCM-3810I
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio3810.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio3810.so: V3.1.5.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:bug 9715. Fix a compiling error because of missing i387.h |                          |
|               |              | 5.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio3810.ko: V3.1.5.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio3810.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- -------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio3810.ko: V3.1.4.0     |
|               |              |                                                                      | libbio3810.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Modified: Bug 15214. Fix data count error in OneBufferedAI when    | bio3810.ko: V3.1.3.0     |
|               |              |             intervalCount is FIF0/2 and Sample is non integer times  | libbio3810.so: V3.1.3.0  |
|               |              |             to it .(Normal)                                          |                          |
|               |              | 2.Modified: Bug 13603. Fix AI Trigger delay time error(Normal)       |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14759. Fix program DIO will excite a pules,when      | bio3810.ko: V3.1.2.0     |
|               |              |             execute reset function.(Normal)                          | libbio3810.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Bug 13979. Add thread locker for FAO,FAI event.(Normal)  | bio3810.ko: V3.1.2.0     |
|               |              |                                                                      | libbio3810.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-07-15  | 1.Modified: Fix buffered AO stop and start again AO output error, if | bio3810.ko: V3.1.2.0     |
|               |              |   do not exist problem.(Normal)                                      | libbio3810.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-05-15  | 1. Initial version.                                                  | bio3810.ko: V3.1.1.0     |
|               |              |                                                                      | libbio3810.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCIGPDC:support PCIE-1756,PCIE-1760,PCIE-1730,PCIE-1751,PCIE-1752
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | biogpdc.ko: V3.1.8.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbiogpdc.so: V3.1.8.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | biogpdc.ko: V3.1.7.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbiogpdc.so: V3.1.7.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1.Added: support PCIE-1752.                                          | biogpdc.ko: V3.1.6.0     |
|               |              |                                                                      | libbiogpdc.so: V3.1.6.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.6.0    |  2015-07-15  | 1.Added: support PCIE-1751.                                          | biogpdc.ko: V3.1.5.0     |
|               |              |                                                                      | libbiogpdc.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | biogpdc.ko: V3.1.4.0     |
|               |              |                                                                      | libbiogpdc.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1.Added: support PCIE-1730.                                          | biogpdc.ko: V3.1.3.0     |
|               |              |                                                                      | libbiogpdc.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Added: support PCIE-1756.                                          | biogpdc.ko: V3.1.2.0     |
|               |              | 2.Modified: Bug 14759. Fix program DIO will excite a pules,when      | libbiogpdc.so: V3.1.2.0  |
|               |              |             execute reset function.(Normal)                          |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Added: Initial version, support PCIE-1760                          | biogpdc.ko: V3.1.1.0     |
|               |              |                                                                      | libbiogpdc.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

 DAQNavi_PCIE1802:support PCIE1802
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Initial version.                                                   | bio1802.ko: V3.1.2.0     |
|               |              |                                                                      | libbio1802.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCIE1810:support PCIE1810,MIOe-3810
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio1810.ko: V3.1.3.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio1810.so: V3.1.3.0  |
|               |              | 3.Modified:bug 10685, wrong trigger edge for ch8,ch16                |                          |
|               |              | 4.Add support of MIOe3810                                            |                          |
|               |              | 5.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 6.Modified:fix bug: Fix 1810 AI data has no SN field                 |                          |
|               |              | 7.Modified:fix bug: compiling error for cross-compiling: amd64.      |                          |
|               |              | 8.Modified:bug 10734. The description of PCIE1810 is wrong.          |                          |
|               |              | 9.Modified:bug 10679. Failed to set the trigger1 source              |                          |
|               |              |            as SigExtDigTrigger1.                                     |                          |
|               |              | 10.Modified:bug 10660. The AO out is wrong when using multi-channel. |                          |
|               |              | 11.Modified:bug 10652. The funciton of channel reentry is wrong.     |                          |
|               |              | 12.Modified:bug 10625. Failed to set the trigger1 source.            |                          |
|               |              | 13.Modified:bug 10546. Failed to update datas when                   |                          |
|               |              |            modifying valueRange.                                     |                          |
|               |              | 14.Modified:bug 10535. The return value of valueRange count is wrong.|                          |
|               |              | 15.Modified:Bug 10955: compiling error on kernel v4.13               |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1. Modified:Bug 20252.Fix crashed error when set DIO port direction. | bio1810.ko: V3.1.2.0     |
|               |              | 2. Modified the thread-safe bugs.                                    | libbio1810.so: V3.1.2.0  |
|               |              | 3. Modified:Bug 19974: Modify the device description bug.            |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1. Initial version.                                                  | bio1810.ko: V3.1.1.0     |
|               |              |                                                                      | libbio1810.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCIGPSCMF:support PCIE-1816,MIOe-3816
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | biogpscmf.ko: V3.1.4.0   |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbiogpscmf.so: V3.1.4.0|
|               |              | 3.Modified:bug 10685, wrong trigger edge for ch8,ch16                |                          |
|               |              | 4.Add support of MIOe3816                                            |                          |
|               |              | 5.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 6.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 7.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1. Modified:Bug 20252.Fix crashed error when set DIO port direction. | biogpscmf.ko: V3.1.3.0   |
|               |              | 2. Modified the thread-safe bugs.                                    | libbiogpscmf.so: V3.1.3.0|
|               |              | 3. Modified:Bug 19974: Modify the device description bug.            |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1. Modified£ºBug of MUX calculation                                  | biogpscmf.ko: V3.1.2.0   |
|               |              |                                                                      | libbiogpscmf.so: V3.1.2.0|
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.7.0    |  2015-09-17  | 1. Initial version.                                                  | biogpscmf.ko: V3.1.1.0   |
|               |              |                                                                      | libbiogpscmf.so: V3.1.1.0|
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4702/4704:support USB-4702,USB-4704
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4702.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4702.so: V3.1.6.0  |
|               |              | 3.Modified:fix bug: kernel warning of DO write port                  |                          |
|               |              | 4.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 5.Modified:bug 10099. compiling error due to the absence of i387.h   |                          |
|               |              | 6.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 7.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 8.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4702.ko: V3.1.5.0     |
|               |              |                                                                      | libbio4702.so: V3.1.5.0  |
--------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4702.ko: V3.1.4.0     |
|               |              | 1.Modified:Bug 16566.fix InstantAO output data error.                | libbio4702.so: V3.1.5.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      | bio4704.ko: V3.1.3.0     |
|               |              |                                                                      | libbio4704.so: V3.1.4.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2013-03-15  | 1.Modified:Bug 11889.Fix device can't run on max pacer rate.(Critical|                          |
|               |              |   )                                                                  | bio4704.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 12296.Fix device can't set convert clock rate.(Critica| libbio4704.so: V3.1.3.0  |
|               |              |   l)                                                                 |                          |
|               |              | 3.Modified:BUg 12329.Fix can not recive data ready event.(Critical)  |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Bug 11731.Fix interval count can't be 2 exponential power| bio4704.ko: V3.1.2.0     |
|               |              |   bug.(Critical)                                                     | libbio4704.so: V3.1.2.0  |
|               |              | 2.Modified: Bug 11114.If ai start after stopped faster, ai may be can|                          |
|               |              |   not start.(Critical)                                               |                          |
|               |              | 3.Modified: Bug 10769. fix dma transfer error on 64bit OS. (Critical)|                          |
|               |              | 4.Modified: Re-arrange the header files structure.(Normal)           |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-09-15  | 1. Initial version.                                                  | bio4704.ko: V3.1.1.0     |
|               |              |                                                                      | libbio4704.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4711A:support USB-4711A
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4711a.ko: V3.1.4.0    |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4711a.so: V3.1.4.0 |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4711a.ko: V3.1.3.0    |
|               |              |                                                                      | libbio4711a.so: V3.1.3.0 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4711a.ko: V3.1.2.0    |
|               |              | 1.Modified:Bug 16566.fix InstantAO output data error.                | libbio4711a.so: V3.1.3.0 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      | bio4711a.ko: V3.1.1.0    |
|               |              |                                                                      | libbio4711a.so: V3.1.2.0 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-07-15  | 1.Added: Initial version.                                            | bio4711a.ko: V3.1.1.0    |
|               |              |                                                                      | libbio4711a.so: V3.1.1.0 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4716:support USB-4716
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4716.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4716.so: V3.1.7.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4716.ko: V3.1.5.0     |
|               |              |                                                                      | libbio4716.so: V3.1.6.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4716.ko: V3.1.4.0     |
|               |              | 1.Modified:Bug 16566.fix InstantAO output data error.                | libbio4716.so: V3.1.6.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | 1.Modified: Bug 13979. Add thread locker for FAI event.(Normal)      | bio4716.ko: V3.1.3.0     |
|               |              |                                                                      | libbio4716.so: V3.1.5.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.4.0    |  2013-07-15  | 1.Modified: Bug 12605.If use extern clock, sometime last data maybe  |                          |
|               |              |   lost.(Critical)                                                    |                          |
|               |              | 2.Modified: Update Reconnection way.(Normal)                         | bio4716.ko: V3.1.3.0     |
|               |              | 3.Modified: Bug 13244.Setting SignalType error.(Normal)              | libbio4716.so: V3.1.4.0  |
|               |              | 4.Modified: Bug 13250.AI data error,when singal type is Differential.|                          |
|               |              |   (Critical)                                                         |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.3.0    |  2013-03-15  | 1.Modified:Bug 12296.Fix device can't set convert clock rate.(Critica| bio4716.ko: V3.1.2.0     |
|               |              |   l)                                                                 | libbio4716.so: V3.1.3.0  |
|               |              | 2.Modified:BUg 12329.Fix can not recive data ready event.(Critical)  |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Bug 11731.Fix interval count can't be 2 exponential power| bio4716.ko: V3.1.2.0     |
|               |              |   bug.(Critical)                                                     | libbio4716.so: V3.1.2.0  |
|               |              | 2.Modified: Bug 11114.If ai start after stopped faster, ai may be can|                          |
|               |              |   not start.(Critical)                                               |                          |
|               |              | 3.Modified: Bug 10769. fix dma transfer error on 64bit OS. (Critical)|                          |
|               |              | 4.Modified: Re-arrange the header files structure.(Normal)           |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | bio4716.ko: V3.1.1.0     |
|               |              | 2. fixed general bugs.(bug 10897, 10895).                            | libbio4716.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Initial version.                                                  | bio4716.ko: V3.0.0100    |
|               |              |                                                                      | libbio4716.so: V3.0.0100 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4718:support USB4718
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4718.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4718.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4718.ko: V3.1.4.0     |
|               |              |                                                                      | libbio4718.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4718.ko: V3.1.3.0     |
|               |              | 2.Modified:Bug 16826.Add DeviceRemoved event                         | libbio4718.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio4718.ko: V3.1.2.0     |
|               |              |                                                                      | libbio4718.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Re-arrange the header files structure.(Normal)           | bio4718.ko: V3.1.2.0     |
|               |              |                                                                      | libbio4718.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | bio4718.ko: V3.1.1.0     |
|               |              | 2. fixed general bugs.(bug 10906).                                   | libbio4718.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Initial version.                                                  | bio4718.ko: V3.0.0100    |
|               |              |                                                                      | libbio4718.so: V3.0.0100 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4750:support USB-4750
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4750.ko: V3.1.6.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4750.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4750.ko: V3.1.5.0     |
|               |              |                                                                      | libbio4750.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4750.ko: V3.1.4.0     |
|               |              |                                                                      | libbio4750.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1.Modified: Bug 14167. Change DO default state from 0xFF to 0x0.     | bio4750.ko: V3.1.3.0     |
|               |              |             (Normal)                                                 | libbio4750.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio4750.ko: V3.1.2.0     |
|               |              |                                                                      | libbio4750.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.2.0    |  2013-01-15  | 1.Modified: Re-arrange the header files structure.(Normal)           | bio4750.ko: V3.1.2.0     |
|               |              |                                                                      | libbio4750.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2012-07-15  | 1. Add 64bit architecture support.                                   | bio4750.ko: V3.1.1.0     |
|               |              | 2. fixed general bugs.(bug 10905).                                   | libbio4750.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.0.0100   |  2012-05-15  | 1. Initial version.                                                  | bio4750.ko: V3.0.0100    |
|               |              |                                                                      | libbio4750.so: V3.0.0100 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4751:support USB4751/L
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4751.ko: V3.1.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4751.so: V3.1.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4751.ko: V3.1.4.0     |
|               |              |                                                                      | libbio4751.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4751.ko: V3.1.3.0     |
|               |              |                                                                      | libbio4751.so: V3.1.3.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.2.0    |  2014-06-15  | 1. Initial version.                                                  | bio4751.ko: V3.1.2.0     |
|               |              |                                                                      | libbio4751.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_USB4761:support USB-4761
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio4761.ko: V3.1.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio4761.so: V3.1.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:bug 10100. usb communication error on kernel v4.11        |                          |
|               |              | 6.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified:Bug 19974: Modify the device description bug.             | bio4761.ko: V3.1.3.0     |
|               |              |                                                                      | libbio4761.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio4761.ko: V3.1.2.0     |
|               |              |                                                                      | libbio4761.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.1.0    |  2014-02-15  | Only update product version.                                         | bio4761.ko: V3.1.1.0     |
|               |              |                                                                      | libbio4761.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.1.1.0    |  2013-07-15  | 1.Added: Initial version.                                            | bio4761.ko: V3.1.1.0     |
|               |              |                                                                      | libbio4761.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCM3718:support PCM-3718,PCM-3718H,PCM-3718HG,PCM-3718HO
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio3718.ko: V3.2.5.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio3718.so: V3.2.4.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     |  bio3718.ko: V3.2.4.0    |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             |  libbio3718.so: V3.2.3.0 |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.8.0    |  2016-03-29  | 1.Modified:Bug 18431. Some channel return error code when set signal |  bio3718.ko: V3.2.3.0    |
|               |              |   type. Some channel return the same value with the start channel's  |  libbio3718.so: V3.2.3.0 |
|               |              |   when scanning multi channel                                        |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio3718.ko: V3.1.2.0     |
|               |              | 2.Modified:Bug 16946.Fix DMA Channel setting error                   | libbio3718.so: V3.1.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1. Initial version.                                                  | bio3718.ko: V3.1.1.0     |
|               |              |                                                                      | libbio3718.so: V3.1.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCM3725:support PCM-3725
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio3725.ko: V3.2.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio3725.so: V3.2.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio3725.ko: V3.2.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio3725.so: V3.2.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio3725.ko: V3.2.2.0     |
|               |              |                                                                      | libbio3725.so: V3.2.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1. Initial version.                                                  | bio3725.ko: V3.2.1.0     |
|               |              |                                                                      | libbio3725.so: V3.2.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

DAQNavi_PCM3730:support PCM-3730
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    Version    |     Date     |                             Description                              |          Updated         |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio3730.ko: V3.2.4.0     |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio3730.so: V3.2.3.0  |
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                          |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                          |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio3730.ko: V3.2.3.0     |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio3730.so: V3.2.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.5.0    |  2015-04-01  | 1.Modified:Bug 16833.Add Do write bit.                               | bio3730.ko: V3.2.2.0     |
|               |              |                                                                      | libbio3730.so: V3.2.2.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------
|    3.2.4.0    |  2015-01-15  | 1. Initial version.                                                  | bio3730.ko: V3.2.1.0     |
|               |              |                                                                      | libbio3730.so: V3.2.1.0  |
 --------------- -------------- ---------------------------------------------------------------------- --------------------------

 DAQNavi_PCM27D24DI:support PCM27D24DI
 --------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    Version    |     Date     |                             Description                              |          Updated          |
 --------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    3.2.11.0   |  2017-10-17  | 1.Modified:fix bug: compiling error on linux kernel 4.11             | bio27d24di.ko: V3.1.3.0   |
|               |              | 2.Modified:fix bug: compiling error for cross-compiling: arm64 oecore| libbio27d24di.so: V3.1.2.0|
|               |              | 3.Modified:bug 8825. Fix a compiling error after the kernel upgraded |                           |
|               |              | 4.Modified:fix bug: compiling error for cross-compiling: amd64       |                           |
|               |              | 5.Modified:Bug 10955: compiling error on kernel v4.13                |                          |
 --------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    3.2.10.0   |  2016-12-21  | 1.Modified the thread-safe bugs.                                     | bio27d24di.ko: V3.1.2.0   |
|               |              | 2.Modified:Bug 19974: Modify the device description bug.             | libbio27d24di.so: V3.1.1.0|
 --------------- -------------- ---------------------------------------------------------------------- ---------------------------
|    3.2.8.0    |  2016-03-29  | 1. Initial version.                                                  | bio27d24di.ko: V3.1.1.0   |
|               |              |                                                                      | libbio27d24di.so: V3.1.1.0|
 --------------- -------------- ---------------------------------------------------------------------- -------------------------- -


Other device driver by request.

2. Installation
(1) Please make sure that the kernel source and gcc are installed.
(2) Please make sure that the source of system software is up-to-date.
(3) Disable comedi driver, please refer to Driver note.
(4) Uninstall the adsapi32 driver, please refer to Driver note.
(5) Copy the driver compression package to home folder and decompression it.
(6) Get root permissions and open the directory to driver source. (example: cd linux_driver_source_3.2.9.3_64bit)
(7) Copy the header file to the system directory. (example: cp inc/* /usr/include/)
(8) Copy the dynamic library to the system directory. (example: cp libs/* /usr/lib/)
(9) Please compile and install biokernbase drivers. (example: cd drivers/driver_base/src/lnx_ko; make; make install)
(10) Compile and install DAQ device driver. (example: cd drivers/usb4716/src/lnx_ko; make; make install)

Through above steps, the SDK and device driver have been installed successfully, and the drivers will be loaded automatically when reboot the system. If the DAQ device is ISA device, such as PCM-3718, PCM-3725, PCM3730, you must perform the following additional steps.
(11) Set jumper and switch to config the io port address, irq, dma of PCM-3718HO. (example: PCM-3718HO)
(12) Config the resource by isa_tools and make sure the resource is consistent with the hardware configuration. (example: cd tools/isa_tool; ./isa_tool -a -n bio3718 -m pcm3718ho -p 0x200 -i 3 -d 1)

3. Supported distribution
 --------------------------------------- -------------- ---------
|             Distribution              |     Kernel version     |
 --------------------------------------- ------------------------
|  Ubuntu 12.04/14.04/15.10(Desktop)    |      3.2/3.13/4.2      |
 --------------------------------------- ------------------------
|  Redhat Enterprise Linux Server 7.2   |         3.10.0         |
 --------------------------------------- ------------------------
|  OpenSUSE Leap 42.1                   |         4.1.12         |
 --------------------------------------- ------------------------
|  Dabian 8.3                           |         3.16.0         |
 --------------------------------------- ------------------------
|  Fedora 23                            |         4.2.3          |
 --------------------------------------- ------------------------

4. Driver note
Please disable comedi driver, because this driver will conflict with some of DAQNavi's driver.
----------------------------------------------
(1) Please get root permissions.
(2) Open /etc/modprobe.d/blacklist.conf file.
(3) Add "blacklist comedi" "blacklist adv_pci_dio" "blacklist adv_pci1710" at end of the file.
(4) reboot system.

Please uninstall adsapi32 driver, because this driver will conflict with some of DAQNavi's driver.
----------------------------------------------
If needed to uninstall adsapi32 driver, please refer to their own uninstall instructions.

User manual
----------------------------------------------
Linux User Interface Manaul has been included in the development plan.If needed,please refer to
\Documents\DAQNavi_User_Interface_Manual_EN\DAQNavi_User_Interface.chm. You can install gnochm to browse
chm manual in linux OS.


5. Hardware Support

Advantech DAQNavi Linux Drivers supports the following devices:
--------------------------------------------------------------------------------
PCI-1706U
PCI-1711/U
PCI-1711L/UL
PCI-1713U
PCI-1714U
PCI-1714UL
PCI-1715U
PCI-1716
PCI-1716L
PCI-1720U
PCI-1721
PCI-1723
PCI-1724U
PCI-1727U
PCI-1730/U
PCI-1733
PCI-1734
PCI-1737
PCI-1739
PCI-1741U
PCI-1747U
PCI-1750
PCI-1751
PCI-1752U/USO
PCI-1753
PCI-1754
PCI-1756
PCI-1757UP
PCI-1758 series
PCI-1761
PCI-1762
PCI-1784
PCIE-1730
PCIE-1744
PCIE-1752
PCIE-1756
PCIE-1760
PCIE-1802
PCIE-1810
MIOe-3810
PCIE-1816
MIOe-3816
PCM-3718
PCM-3725
PCM-3730
PCM-3761I
PCM-3753I
PCM-3810I
PCM-27D24DI
USB-4702
USB-4704
USB-4711A
USB-4716
USB-4718
USB-4750
USB-4751
USB-4761
MIC-3714
MIC-3716
MIC-3723
MIC-3753
MIC-3756


6. System requirements

Advantech DAQNavi Linux Drivers system requirements:
--------------------------------------------------------------------------------
1. Kernel version must be higher than 2.6.18.
2. The glibc version must be higner than 2.11
