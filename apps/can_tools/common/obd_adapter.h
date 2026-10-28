#pragma once

#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import <IOBluetooth/IOBluetooth.h>
#include <stdbool.h>

#include "elm327.h"

typedef enum {
    OBD_ADAPTER_TRANSPORT_BLE = 0,
    OBD_ADAPTER_TRANSPORT_CLASSIC = 1
} OBDAdapterTransport;

typedef void (*obd_adapter_frame_callback_t)(const can_frame_t *frame, void *context);

/** BLE (SPP service FFF0, write FFF1, notify FFF2) or Classic RFCOMM channel 1
 *  ELM327/STN adapter. Delegate callbacks arrive on the main run loop; the
 *  caller pumps it (runLoopFor:, waitForBleReady:, sendCommand:...). Scan and
 *  connect policy lives in obd_connect.h. Safe to create and release inside a
 *  short-lived autorelease pool. */
@interface OBDAdapter : NSObject <CBCentralManagerDelegate,
                                  CBPeripheralDelegate,
                                  IOBluetoothDeviceInquiryDelegate,
                                  IOBluetoothRFCOMMChannelDelegate> {
@public
    char _responseBuf[4096];
    int _responseLen;
    BOOL _responseReady;
    BOOL _bleReady;
    char _canLineBuf[256];
    int _canLineLen;
}

@property (nonatomic, strong) CBCentralManager *central;
@property (nonatomic, strong) NSMutableArray<CBPeripheral *> *discoveredDevices;
@property (nonatomic, strong) NSMutableArray<NSString *> *discoveredNames;
@property (nonatomic, strong) NSMutableArray<NSNumber *> *discoveredRSSIs;
@property (nonatomic, strong) CBPeripheral *connectedPeripheral;
@property (nonatomic, strong) CBCharacteristic *rxCharacteristic;
@property (nonatomic, strong) CBCharacteristic *txCharacteristic;

@property (nonatomic, strong) IOBluetoothDeviceInquiry *classicInquiry;
@property (nonatomic, strong) NSMutableArray<IOBluetoothDevice *> *classicDevices;
@property (nonatomic, strong) IOBluetoothDevice *connectedClassicDevice;
@property (nonatomic, strong) IOBluetoothRFCOMMChannel *rfcommChannel;
@property (nonatomic, assign) BOOL classicInquiryComplete;

@property (nonatomic, strong) NSMutableArray<NSNumber *> *discoveredTransports;
@property (nonatomic, strong) NSMutableArray<NSNumber *> *discoveredSourceIndices;
@property (nonatomic, assign) OBDAdapterTransport activeTransport;

@property (nonatomic, assign) BOOL isScanning;
@property (nonatomic, assign) BOOL isConnected;
@property (nonatomic, assign) BOOL isConnecting;
@property (nonatomic, assign) BOOL serviceDiscoveryDone;
@property (nonatomic, assign) BOOL canMonitorActive;
@property (nonatomic, assign) BOOL quiet;
/** When YES (and not quiet), also report per-command failures on stderr:
 *  not connected, characteristic not ready, command timeout, invalid device
 *  index, already connected, missing FFF1/FFF2 characteristics. These are the
 *  diagnostics can_reader/can_discover printed before they used OBDAdapter.
 *  Default NO. */
@property (nonatomic, assign) BOOL logCommandErrors;
/** Case-insensitive substrings a BLE or Classic device name must contain to be
 *  listed; empty = list every named device. */
@property (nonatomic, strong) NSArray<NSString *> *nameHints;

@property (nonatomic, readonly) NSString *connectedDeviceName;

/** Name hints for ELM327/STN adapters: OBDLink, vLinker, ELM327, OBD, iOBD. */
+ (NSArray<NSString *> *)defaultNameHints;

- (instancetype)initWithNameHints:(NSArray<NSString *> *)nameHints quiet:(BOOL)quiet;
- (BOOL)isBleReady;
- (BOOL)waitForBleReady:(double)timeoutSeconds;
- (void)runLoopFor:(double)seconds;
- (void)startScan;
- (void)stopScan;
- (BOOL)connectToDevice:(int)index;
- (void)disconnect;
- (BOOL)sendCommand:(const char *)cmd response:(char *)resp maxLen:(int)maxLen;
- (BOOL)sendCommand:(const char *)cmd
            response:(char *)resp
              maxLen:(int)maxLen
             timeout:(double)timeoutSeconds;
- (void)sendRawString:(const char *)str;
/** Enter monitor parsing: every complete received line that parses as a CAN
 *  frame is timestamped (wall clock ms) and passed to callback. Clears the
 *  command response buffer. The caller sends the monitor command itself. */
- (void)startMonitorWithCallback:(obd_adapter_frame_callback_t)callback context:(void *)context;
/** Leave monitor parsing; received bytes go to the response buffer again.
 *  The caller sends "\r" to stop the adapter's monitor command. */
- (void)stopMonitor;

@end
