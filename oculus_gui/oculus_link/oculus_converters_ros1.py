import oculus_sonar.msg as oculus_msg

def ip_to_string(uint32_ip):
    return '{}.{}.{}.{}'.format( uint32_ip & 0x000000ff,
                                (uint32_ip & 0x0000ff00) >> 8,
                                (uint32_ip & 0x00ff0000) >> 16,
                                (uint32_ip & 0xff000000) >> 24)

def mac_to_string(uint8_mac):
    return '{}{}:{}{}:{}{}'.format(uint8_mac[0], uint8_mac[1], uint8_mac[2],
                                   uint8_mac[3], uint8_mac[4], uint8_mac[5])

def from_OculusHeader(msg):
    return {'type'        : 'OculusHeader',
            'oculusId'    : msg.oculusId,
            'srcDeviceId' : msg.srcDeviceId,
            'dstDeviceId' : msg.dstDeviceId,
            'msgId'       : msg.msgId,
            'msgVersion'  : msg.msgVersion,
            'payloadSize' : msg.payloadSize,
            'spare2'      : msg.spare2}


def from_OculusVersionInfo(msg):
    return {'type'             : 'OculusVersionInfo',
            'firmwareVersion0' : msg.firmwareVersion0,
            'firmwareDate0'    : msg.firmwareDate0,
            'firmwareVersion1' : msg.firmwareVersion1,
            'firmwareDate1'    : msg.firmwareDate1,
            'firmwareVersion2' : msg.firmwareVersion2,
            'firmwareDate2'    : msg.firmwareDate2}


def from_OculusStatus(msg):

    return {'type'       : 'OculusStatus',
            'hdr'        : from_OculusHeader(msg.hdr),
            'deviceId'   : msg.deviceId,
            'deviceType' : msg.deviceType,
            'partNumber' : msg.partNumber,
            'status'     : msg.status,
            'versinInfo' : from_OculusVersionInfo(msg.versinInfo),

            'ipAddr'          : ip_to_string(msg.ipAddr),
            'ipMask'          : ip_to_string(msg.ipMask),
            'connectedIpAddr' : ip_to_string(msg.connectedIpAddr),
            'macAddr'         : mac_to_string([msg.macAddr0, msg.macAddr1,
                                               msg.macAddr2, msg.macAddr3,
                                               msg.macAddr4, msg.macAddr5]),
            'temperature0' : msg.temperature0,
            'temperature1' : msg.temperature1,
            'temperature2' : msg.temperature2,
            'temperature3' : msg.temperature3,
            'temperature4' : msg.temperature4,
            'temperature5' : msg.temperature5,
            'temperature6' : msg.temperature6,
            'temperature7' : msg.temperature7,
            'pressure'     : msg.pressure}

def from_OculusFireConfig(msg):
    return {'type'            : 'OculusFireConfig',
            'head'            : from_OculusHeader(msg.head),
            'masterMode'      : msg.masterMode,
            'pingRate'        : msg.pingRate,
            'networkSpeed'    : msg.networkSpeed,
            'gammaCorrection' : msg.gammaCorrection,
            'flags'           : msg.flags,
            'range'           : msg.range,
            'gainPercent'     : msg.gainPercent,
            'speedOfSound'    : msg.speedOfSound,
            'salinity'        : msg.salinity}

def from_OculusPing(msg):
    return {'type'              : 'OculusPing',
            'fireMessage'       : from_OculusFireConfig(msg.fireMessage),
            'pingId'            : msg.pingId,
            'status'            : msg.status,
            'frequency'         : msg.frequency,
            'temperature'       : msg.temperature,
            'pressure'          : msg.pressure,
            'speeedOfSoundUsed' : msg.speeedOfSoundUsed,
            'pingStartTime'     : msg.pingStartTime,
            'dataSize'          : msg.dataSize,
            'rangeResolution'   : msg.rangeResolution,
            'nRanges'           : msg.nRanges,
            'nBeams'            : msg.nBeams,
            'imageOffset'       : msg.imageOffset,
            'imageSize'         : msg.imageSize,
            'messageSize'       : msg.messageSize}

def from_OculusStampedPing(msg):
    return {'type'              : 'OculusStampedPing',
            'fireMessage'       : from_OculusFireConfig(msg.ping.fireMessage),
            'pingId'            : msg.ping.pingId,
            'status'            : msg.ping.status,
            'frequency'         : msg.ping.frequency,
            'temperature'       : msg.ping.temperature,
            'pressure'          : msg.ping.pressure,
            'speeedOfSoundUsed' : msg.ping.speeedOfSoundUsed,
            'pingStartTime'     : msg.ping.pingStartTime,
            'dataSize'          : msg.ping.dataSize,
            'rangeResolution'   : msg.ping.rangeResolution,
            'nRanges'           : msg.ping.nRanges,
            'nBeams'            : msg.ping.nBeams,
            'imageOffset'       : msg.ping.imageOffset,
            'imageSize'         : msg.ping.imageSize,
            'messageSize'       : msg.ping.messageSize}
    

serializers = {
'OculusHeader'      : (oculus_msg.OculusHeader, 
                       lambda msg: {'scalars' : from_OculusHeader(msg),
                                    'vectors' : {}}),
'OculusVersionInfo' : (oculus_msg.OculusVersionInfo,
                       lambda msg: {'scalars' : from_OculusVersionInfo(msg),
                                    'vectors' : {}}),
'OculusStatus'      : (oculus_msg.OculusStatus,
                       lambda msg: {'scalars' : from_OculusStatus(msg), 
                                    'vectors' : {}}),
'OculusFireConfig'  : (oculus_msg.OculusFireConfig,
                       lambda msg: {'scalars' : from_OculusFireConfig(msg),
                                    'vectors' : {}}),
'OculusPing'        : (oculus_msg.OculusPing,
                       lambda msg: {'scalars' : from_OculusPing(msg),
                                    'vectors' : {'data' : ('data',  msg.data)}}),
'OculusStampedPing' : (oculus_msg.OculusStampedPing,
                       lambda msg: {'scalars' : from_OculusStampedPing(msg),
                                    'vectors' : {'data' : ('data',  msg.ping.data)}})
}


