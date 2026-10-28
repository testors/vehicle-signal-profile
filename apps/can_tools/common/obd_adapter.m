#import "obd_adapter.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

static NSString * const kOBDAdapterServiceUUID = @"FFF0";
static const NSTimeInterval kOBDAdapterDefaultTimeout = 3.0;

/* Ownership (compiled without ARC): every object ivar is retained by the
 * adapter and released in -dealloc. Connection objects are replaced through
 * obd_replace(), which releases the old object at the next autorelease pool
 * drain rather than immediately, so a delegate callback never frees the
 * object that is calling it. */
static void obd_replace(id *slot, id value) {
    if (*slot == value) return;
    [value retain];
    [*slot autorelease];
    *slot = value;
}

@implementation OBDAdapter {
    obd_adapter_frame_callback_t _frameCallback;
    void *_frameContext;
}

+ (NSArray<NSString *> *)defaultNameHints {
    return @[ @"OBDLink", @"vLinker", @"VLINKER", @"VLinker",
              @"ELM327", @"OBD", @"iOBD" ];
}

/* Per-command diagnostics (logCommandErrors) */
- (BOOL)_logErrors {
    return _logCommandErrors && !_quiet;
}

- (instancetype)initWithNameHints:(NSArray<NSString *> *)nameHints quiet:(BOOL)quiet {
    self = [super init];
    if (self) {
        _quiet = quiet;
        _nameHints = [(nameHints ?: @[]) copy];
        _discoveredDevices = [[NSMutableArray alloc] init];
        _discoveredNames = [[NSMutableArray alloc] init];
        _discoveredRSSIs = [[NSMutableArray alloc] init];
        _classicDevices = [[NSMutableArray alloc] init];
        _discoveredTransports = [[NSMutableArray alloc] init];
        _discoveredSourceIndices = [[NSMutableArray alloc] init];
        _central = [[CBCentralManager alloc] initWithDelegate:self
                                                        queue:nil
                                                      options:@{CBCentralManagerOptionShowPowerAlertKey: @YES}];
        _classicInquiry = [[IOBluetoothDeviceInquiry inquiryWithDelegate:self] retain];
        _classicInquiry.inquiryLength = 4;
        _classicInquiry.updateNewDeviceNames = YES;
    }
    return self;
}

- (void)dealloc {
    _frameCallback = NULL;
    _frameContext = NULL;
    _canMonitorActive = NO;
    if (_isScanning) [self stopScan];
    [_classicInquiry setDelegate:nil];
    [_classicInquiry release];
    _central.delegate = nil;
    [_central release];
    _connectedPeripheral.delegate = nil;
    [_connectedPeripheral release];
    [_rxCharacteristic release];
    [_txCharacteristic release];
    [_rfcommChannel setDelegate:nil];
    [_rfcommChannel release];
    [_connectedClassicDevice release];
    [_discoveredDevices release];
    [_discoveredNames release];
    [_discoveredRSSIs release];
    [_classicDevices release];
    [_discoveredTransports release];
    [_discoveredSourceIndices release];
    [_nameHints release];
    [super dealloc];
}

- (NSString *)connectedDeviceName {
    if (_activeTransport == OBD_ADAPTER_TRANSPORT_CLASSIC) {
        return _connectedClassicDevice.name;
    }
    return _connectedPeripheral.name;
}

- (BOOL)isBleReady {
    return _bleReady;
}

- (void)runLoopFor:(double)seconds {
    NSDate *until = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while ([[NSDate date] compare:until] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }
}

- (BOOL)waitForBleReady:(double)timeoutSeconds {
    NSDate *timeout = [NSDate dateWithTimeIntervalSinceNow:timeoutSeconds];
    while (!_bleReady && [[NSDate date] compare:timeout] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }
    return _bleReady;
}

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    (void)central;
    _bleReady = (_central.state == CBManagerStatePoweredOn);
    if (!_quiet && !_bleReady) {
        fprintf(stderr, "[WARN] BLE state: %ld\n", (long)_central.state);
    }
}

- (BOOL)_nameMatchesHints:(NSString *)name {
    if (!name || name.length == 0) return NO;
    if (_nameHints.count == 0) return YES;

    NSString *upper = [name uppercaseString];
    for (NSString *hint in _nameHints) {
        if ([upper containsString:[hint uppercaseString]]) {
            return YES;
        }
    }
    return NO;
}

- (void)_addClassicDeviceIfMatched:(IOBluetoothDevice *)device {
    NSString *name = device.name;
    if (![self _nameMatchesHints:name]) return;

    for (IOBluetoothDevice *existing in _classicDevices) {
        if ([existing.addressString isEqualToString:device.addressString]) return;
    }

    NSUInteger sourceIdx = _classicDevices.count;
    [_classicDevices addObject:device];
    [_discoveredNames addObject:name];
    [_discoveredRSSIs addObject:@(device.rawRSSI)];
    [_discoveredTransports addObject:@(OBD_ADAPTER_TRANSPORT_CLASSIC)];
    [_discoveredSourceIndices addObject:@(sourceIdx)];
}

- (void)centralManager:(CBCentralManager *)central
 didDiscoverPeripheral:(CBPeripheral *)peripheral
     advertisementData:(NSDictionary<NSString *,id> *)advertisementData
                  RSSI:(NSNumber *)RSSI {
    (void)central;
    (void)advertisementData;

    NSString *name = peripheral.name;
    if (![self _nameMatchesHints:name]) return;

    for (CBPeripheral *existing in _discoveredDevices) {
        if ([existing.identifier isEqual:peripheral.identifier]) return;
    }

    NSUInteger sourceIdx = _discoveredDevices.count;
    [_discoveredDevices addObject:peripheral];
    [_discoveredNames addObject:name];
    [_discoveredRSSIs addObject:RSSI ?: @(0)];
    [_discoveredTransports addObject:@(OBD_ADAPTER_TRANSPORT_BLE)];
    [_discoveredSourceIndices addObject:@(sourceIdx)];
}

- (void)centralManager:(CBCentralManager *)central
  didConnectPeripheral:(CBPeripheral *)peripheral {
    (void)central;
    _isConnecting = NO;
    _isConnected = YES;
    obd_replace(&_connectedPeripheral, peripheral);
    peripheral.delegate = self;
    [peripheral discoverServices:@[[CBUUID UUIDWithString:kOBDAdapterServiceUUID]]];
}

- (void)centralManager:(CBCentralManager *)central
didFailToConnectPeripheral:(CBPeripheral *)peripheral
                 error:(NSError *)error {
    (void)central;
    (void)peripheral;
    if (!_quiet) {
        fprintf(stderr, "[ERROR] BLE connect failed: %s\n",
                error.localizedDescription.UTF8String ?: "unknown");
    }
    _isConnecting = NO;
}

- (void)centralManager:(CBCentralManager *)central
didDisconnectPeripheral:(CBPeripheral *)peripheral
                 error:(NSError *)error {
    (void)central;
    (void)peripheral;
    (void)error;
    _isConnected = NO;
    obd_replace(&_connectedPeripheral, nil);
    obd_replace(&_rxCharacteristic, nil);
    obd_replace(&_txCharacteristic, nil);
    _serviceDiscoveryDone = NO;
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error {
    (void)peripheral;
    if (error) {
        if (!_quiet) {
            fprintf(stderr, "[ERROR] BLE service discovery failed: %s\n",
                    error.localizedDescription.UTF8String ?: "unknown");
        }
        return;
    }

    for (CBService *service in peripheral.services) {
        [peripheral discoverCharacteristics:nil forService:service];
    }
}

- (void)peripheral:(CBPeripheral *)peripheral
didDiscoverCharacteristicsForService:(CBService *)service
             error:(NSError *)error {
    if (error) {
        if (!_quiet) {
            fprintf(stderr, "[ERROR] BLE characteristic discovery failed: %s\n",
                    error.localizedDescription.UTF8String ?: "unknown");
        }
        return;
    }

    for (CBCharacteristic *characteristic in service.characteristics) {
        NSString *uuid = [characteristic.UUID.UUIDString uppercaseString];
        if ([uuid containsString:@"FFF1"]) {
            obd_replace(&_rxCharacteristic, characteristic);
        } else if ([uuid containsString:@"FFF2"]) {
            obd_replace(&_txCharacteristic, characteristic);
            [peripheral setNotifyValue:YES forCharacteristic:characteristic];
        }
    }

    if (_rxCharacteristic && _txCharacteristic) {
        _serviceDiscoveryDone = YES;
    } else if ([self _logErrors]) {
        fprintf(stderr, "[WARN] Missing required characteristics (RX=%s, TX=%s)\n",
                _rxCharacteristic ? "OK" : "missing",
                _txCharacteristic ? "OK" : "missing");
    }
}

- (void)_handleReceivedBytes:(const char *)bytes length:(int)len {
    if (_canMonitorActive) {
        for (int i = 0; i < len; i++) {
            if (bytes[i] == '\r' || bytes[i] == '\n') {
                if (_canLineLen > 0) {
                    _canLineBuf[_canLineLen] = '\0';

                    can_frame_t frame;
                    if (elm327_parse_can_frame(_canLineBuf, &frame)) {
                        struct timeval tv;
                        gettimeofday(&tv, NULL);
                        frame.timestamp_ms = (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
                        if (_frameCallback) {
                            _frameCallback(&frame, _frameContext);
                        }
                    }
                    _canLineLen = 0;
                }
            } else if (bytes[i] != '>' && _canLineLen < (int)sizeof(_canLineBuf) - 1) {
                _canLineBuf[_canLineLen++] = bytes[i];
            }
        }
        return;
    }

    for (int i = 0; i < len && _responseLen < (int)sizeof(_responseBuf) - 1; i++) {
        _responseBuf[_responseLen++] = bytes[i];
        if (bytes[i] == '>') {
            _responseBuf[_responseLen] = '\0';
            _responseReady = YES;
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral
didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic
             error:(NSError *)error {
    (void)peripheral;
    (void)characteristic;
    if (error) return;

    NSData *data = characteristic.value;
    if (!data || data.length == 0) return;
    [self _handleReceivedBytes:(const char *)data.bytes length:(int)data.length];
}

- (void)peripheral:(CBPeripheral *)peripheral
didUpdateNotificationStateForCharacteristic:(CBCharacteristic *)characteristic
             error:(NSError *)error {
    (void)peripheral;
    (void)characteristic;
    if (error && !_quiet) {
        fprintf(stderr, "[ERROR] BLE notification failed: %s\n",
                error.localizedDescription.UTF8String ?: "unknown");
    }
}

- (void)deviceInquiryDeviceFound:(IOBluetoothDeviceInquiry *)sender
                          device:(IOBluetoothDevice *)device {
    (void)sender;
    [self _addClassicDeviceIfMatched:device];
}

- (void)deviceInquiryDeviceNameUpdated:(IOBluetoothDeviceInquiry *)sender
                                device:(IOBluetoothDevice *)device
                      devicesRemaining:(uint32_t)devicesRemaining {
    (void)sender;
    (void)devicesRemaining;
    [self _addClassicDeviceIfMatched:device];
}

- (void)deviceInquiryComplete:(IOBluetoothDeviceInquiry *)sender
                        error:(IOReturn)error
                      aborted:(BOOL)aborted {
    (void)sender;
    (void)error;
    (void)aborted;
    _classicInquiryComplete = YES;
}

- (void)rfcommChannelOpenComplete:(IOBluetoothRFCOMMChannel *)rfcommChannel
                           status:(IOReturn)error {
    (void)rfcommChannel;
    _isConnecting = NO;
    if (error != kIOReturnSuccess) {
        if (!_quiet) fprintf(stderr, "[ERROR] RFCOMM open failed: 0x%08x\n", error);
        return;
    }
    _isConnected = YES;
    _serviceDiscoveryDone = YES;
}

- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel *)rfcommChannel
                     data:(void *)dataPointer
                   length:(size_t)dataLength {
    (void)rfcommChannel;
    [self _handleReceivedBytes:(const char *)dataPointer length:(int)dataLength];
}

- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel *)rfcommChannel {
    (void)rfcommChannel;
    _isConnected = NO;
    obd_replace(&_connectedClassicDevice, nil);
    obd_replace(&_rfcommChannel, nil);
    _serviceDiscoveryDone = NO;
}

- (void)startScan {
    [_discoveredDevices removeAllObjects];
    [_discoveredNames removeAllObjects];
    [_discoveredRSSIs removeAllObjects];
    [_classicDevices removeAllObjects];
    [_discoveredTransports removeAllObjects];
    [_discoveredSourceIndices removeAllObjects];
    _classicInquiryComplete = NO;
    _isScanning = YES;

    NSArray *paired = [IOBluetoothDevice pairedDevices];
    for (IOBluetoothDevice *device in paired) {
        [self _addClassicDeviceIfMatched:device];
    }

    if (_bleReady) {
        [_central scanForPeripheralsWithServices:nil
                                         options:@{CBCentralManagerScanOptionAllowDuplicatesKey: @NO}];
    }
    [_classicInquiry start];
}

- (void)stopScan {
    [_central stopScan];
    [_classicInquiry stop];
    _isScanning = NO;
}

- (BOOL)connectToDevice:(int)index {
    if (index < 0 || index >= (int)_discoveredNames.count) {
        if ([self _logErrors]) fprintf(stderr, "[ERROR] Invalid device index\n");
        return NO;
    }
    if (_isConnected) {
        if ([self _logErrors]) fprintf(stderr, "[ERROR] Already connected. Disconnect first\n");
        return NO;
    }

    OBDAdapterTransport transport =
        (OBDAdapterTransport)[_discoveredTransports[index] intValue];
    int sourceIdx = [_discoveredSourceIndices[index] intValue];

    if (transport == OBD_ADAPTER_TRANSPORT_BLE) {
        if (!_bleReady) return NO;
        CBPeripheral *peripheral = _discoveredDevices[sourceIdx];
        _activeTransport = OBD_ADAPTER_TRANSPORT_BLE;
        _isConnecting = YES;
        [_central connectPeripheral:peripheral options:nil];
        return YES;
    }

    IOBluetoothDevice *device = _classicDevices[sourceIdx];
    IOBluetoothRFCOMMChannel *channel = nil;
    _activeTransport = OBD_ADAPTER_TRANSPORT_CLASSIC;
    _isConnecting = YES;

    IOReturn result = [device openRFCOMMChannelSync:&channel
                                      withChannelID:1
                                           delegate:self];
    _isConnecting = NO;
    if (result != kIOReturnSuccess || channel == nil) {
        if (!_quiet) fprintf(stderr, "[ERROR] RFCOMM connect failed: 0x%08x\n", result);
        return NO;
    }

    obd_replace(&_rfcommChannel, channel);
    obd_replace(&_connectedClassicDevice, device);
    _isConnected = YES;
    _serviceDiscoveryDone = YES;
    return YES;
}

- (void)disconnect {
    if (_activeTransport == OBD_ADAPTER_TRANSPORT_CLASSIC) {
        if (_rfcommChannel) {
            [_rfcommChannel closeChannel];
            obd_replace(&_rfcommChannel, nil);
        }
        if (_connectedClassicDevice) {
            [_connectedClassicDevice closeConnection];
            obd_replace(&_connectedClassicDevice, nil);
        }
    } else {
        if (_connectedPeripheral) {
            [_central cancelPeripheralConnection:_connectedPeripheral];
            obd_replace(&_connectedPeripheral, nil);
        }
        obd_replace(&_rxCharacteristic, nil);
        obd_replace(&_txCharacteristic, nil);
    }
    _isConnected = NO;
    _serviceDiscoveryDone = NO;
    _canMonitorActive = NO;
}

- (BOOL)sendCommand:(const char *)cmd response:(char *)resp maxLen:(int)maxLen {
    return [self sendCommand:cmd
                    response:resp
                      maxLen:maxLen
                     timeout:kOBDAdapterDefaultTimeout];
}

- (BOOL)sendCommand:(const char *)cmd
            response:(char *)resp
              maxLen:(int)maxLen
             timeout:(double)timeoutSeconds {
    if (!_isConnected || !_serviceDiscoveryDone) {
        if ([self _logErrors]) fprintf(stderr, "[ERROR] Not connected or service not ready\n");
        return NO;
    }
    if (!cmd || !resp || maxLen <= 0) {
        return NO;
    }
    if (_activeTransport == OBD_ADAPTER_TRANSPORT_BLE && !_rxCharacteristic) {
        if ([self _logErrors]) fprintf(stderr, "[ERROR] BLE characteristic not ready\n");
        return NO;
    }

    memset(_responseBuf, 0, sizeof(_responseBuf));
    _responseLen = 0;
    _responseReady = NO;

    char cmdWithCr[160];
    snprintf(cmdWithCr, sizeof(cmdWithCr), "%s\r", cmd);

    if (_activeTransport == OBD_ADAPTER_TRANSPORT_CLASSIC) {
        [_rfcommChannel writeSync:(void *)cmdWithCr length:strlen(cmdWithCr)];
    } else {
        NSData *data = [NSData dataWithBytes:cmdWithCr length:strlen(cmdWithCr)];
        CBCharacteristicWriteType writeType =
            (_rxCharacteristic.properties & CBCharacteristicPropertyWriteWithoutResponse)
                ? CBCharacteristicWriteWithoutResponse
                : CBCharacteristicWriteWithResponse;
        [_connectedPeripheral writeValue:data
                       forCharacteristic:_rxCharacteristic
                                    type:writeType];
    }

    NSDate *timeout = [NSDate dateWithTimeIntervalSinceNow:timeoutSeconds];
    while (!_responseReady && [[NSDate date] compare:timeout] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }
    if (!_responseReady) {
        if ([self _logErrors]) fprintf(stderr, "[WARN] Command timeout: %s\n", cmd);
        return NO;
    }

    char *src = _responseBuf;
    int dstLen = 0;
    char *echoEnd = strstr(src, cmd);
    if (echoEnd) src = echoEnd + strlen(cmd);

    for (int i = 0; src[i] && dstLen < maxLen - 1; i++) {
        if (src[i] == '>') continue;
        if (src[i] == '\r') {
            if (dstLen > 0 && resp[dstLen - 1] != '\n') resp[dstLen++] = '\n';
            continue;
        }
        resp[dstLen++] = src[i];
    }
    resp[dstLen] = '\0';

    while (dstLen > 0 &&
           (resp[dstLen - 1] == '\n' || resp[dstLen - 1] == ' ' ||
            resp[dstLen - 1] == '\t')) {
        resp[--dstLen] = '\0';
    }
    return YES;
}

- (void)sendRawString:(const char *)str {
    if (!_isConnected || !str) return;

    if (_activeTransport == OBD_ADAPTER_TRANSPORT_CLASSIC) {
        if (_rfcommChannel) {
            [_rfcommChannel writeSync:(void *)str length:strlen(str)];
        }
        return;
    }

    if (!_rxCharacteristic) return;
    NSData *data = [NSData dataWithBytes:str length:strlen(str)];
    CBCharacteristicWriteType writeType =
        (_rxCharacteristic.properties & CBCharacteristicPropertyWriteWithoutResponse)
            ? CBCharacteristicWriteWithoutResponse
            : CBCharacteristicWriteWithResponse;
    [_connectedPeripheral writeValue:data
                   forCharacteristic:_rxCharacteristic
                                type:writeType];
}

- (void)startMonitorWithCallback:(obd_adapter_frame_callback_t)callback context:(void *)context {
    _frameCallback = callback;
    _frameContext = context;
    _canLineLen = 0;
    _canMonitorActive = YES;

    memset(_responseBuf, 0, sizeof(_responseBuf));
    _responseLen = 0;
    _responseReady = NO;
}

- (void)stopMonitor {
    _canMonitorActive = NO;
    _frameCallback = NULL;
    _frameContext = NULL;
}

@end
