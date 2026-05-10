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
@property (nonatomic, strong) NSArray<NSString *> *nameHints;

@property (nonatomic, readonly) NSString *connectedDeviceName;

- (instancetype)initWithNameHints:(NSArray<NSString *> *)nameHints quiet:(BOOL)quiet;
- (BOOL)isBleReady;
- (BOOL)waitForBleReady:(double)timeoutSeconds;
- (void)runLoopFor:(double)seconds;
- (void)startScan;
- (void)stopScan;
- (BOOL)connectToDevice:(int)index;
- (BOOL)scanAndConnectMatching:(NSString *)namePart scanSeconds:(double)scanSeconds;
- (void)disconnect;
- (BOOL)sendCommand:(const char *)cmd response:(char *)resp maxLen:(int)maxLen;
- (BOOL)sendCommand:(const char *)cmd
            response:(char *)resp
              maxLen:(int)maxLen
             timeout:(double)timeoutSeconds;
- (void)sendRawString:(const char *)str;
- (void)startMonitorWithCallback:(obd_adapter_frame_callback_t)callback context:(void *)context;
- (void)stopMonitor;

@end
