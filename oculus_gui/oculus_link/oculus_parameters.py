import oculus_python

def from_OculusSimpleFireMessage(fireMessage):
    return {'frequency_mode'   : fireMessage.masterMode,
            'ping_rate'        : fireMessage.pingRate,
            'data_depth'       : (fireMessage.flags & 0x02) > 0,
            'nbeams'           : (fireMessage.flags & 0x40) > 0,
            'send_gain'        : (fireMessage.flags & 0x04) > 0,
            'gain_assist'      : (fireMessage.flags & 0x10) == 0,
            'range'            : fireMessage.range,
            'gamma_correction' : fireMessage.gammaCorrection,
            'gain_percent'     : fireMessage.gainPercent,
            'sound_speed'      : fireMessage.speedOfSound,
            'use_salinity'     : fireMessage.speedOfSound == 0,
            'salinity'         : fireMessage.salinity
            }

def to_OculusSimpleFireMessage(data):
    # data is supposed to be complete
    fireMessage = oculus_python.OculusSimpleFireMessage()

    fireMessage.masterMode      = data['frequency_mode']
    fireMessage.pingRate        = data['ping_rate']
    fireMessage.range           = data['range']
    fireMessage.gammaCorrection = data['gamma_correction']
    fireMessage.gainPercent     = data['gain_percent']
    fireMessage.speedOfSound    = data['sound_speed']
    fireMessage.salinity        = data['salinity']
    
    fireMessage.flags = 0x08
    if data['data_depth']:
        fireMessage.flags = fireMessage.flags | 0x02
    if data['nbeams']:
        fireMessage.flags = fireMessage.flags | 0x40
    if data['send_gain']:
        fireMessage.flags = fireMessage.flags | 0x04
    if not data['gain_assist']:
        fireMessage.flags = fireMessage.flags | 0x10 # setup inverted on the sonar
    if data['use_salinity']:
        fireMessage.speedOfSound = 0

    return fireMessage


def update_OculusSimpleFireMessage(fireMessage, request):
    updatedRequest = from_OculusSimpleFireMessage(fireMessage)
    updatedRequest.update(request)
    return to_OculusSimpleFireMessage(updatedRequest)
    

def parameter_description(currentConfig):
    return [{'name': 'frequency_mode',
             'description': 'Sonar beam frequency mode.',
             'type': 'int',
             'default': 2,
             'edit_method': {'type': 'enum',
                             'entries': [{'name': 'FrequencyLow', 'value': 1, 'description': 'Low frequency (1.2MHz, wide aperture).'},
                                         {'name': 'FrequencyHigh', 'value': 2, 'description': 'High frequency (2.1Mhz, narrow aperture).'}]},
             'current_value': currentConfig.masterMode
            },
            {'name': 'ping_rate',
             'description': 'Frequency of ping fires.',
             'type': 'int',
             'default': 3,
             'edit_method': {'type': 'enum',
                             'entries': [{'name': 'PingRateNormal',  'value': 0, 'description': '10Hz max ping rate.'},
                                         {'name': 'PingRateHigh',    'value': 1, 'description': '15Hz max ping rate.'},
                                         {'name': 'PingRateHighest', 'value': 2, 'description': '40Hz max ping rate.'},
                                         {'name': 'PingRateLow',     'value': 3, 'description': '5Hz max ping rate.'},
                                         {'name': 'PingRateLowest',  'value': 4, 'description': '2Hz max ping rate.'},
                                         {'name': 'NoPing',          'value': 5, 'description': 'Standby mode (no ping fire).'}]},
             'current_value': currentConfig.pingRate
            },
            {'name': 'data_depth',
             'description': 'Encoding of ping data.',
             'type': 'int',
             'default': 0,
             'edit_method': {'type': 'fixed'},
             'current_value': 0
            },
            {'name': 'nbeams',
             'description': 'Number of beams.',
             'type': 'int',
             'default': 0,
             'edit_method': {'type': 'enum',
                             'entries': [{'name': '256beams', 'value': 0, 'description': 'Oculus outputs 256 beams.'},
                                         {'name': '512beams', 'value': 1, 'description': 'Oculus outputs 512 beams.'}]},
             'current_value': 0
            },
            {'name': 'send_gain',
             'description': 'Send range gain with data.',
             'type': 'bool',
             'default': True,
             # 'edit_method': {'type': 'bool'}, 
             'edit_method': {'type': 'fixed'}, 
             'current_value': True
            },
            {'name': 'gain_assist',
             'description': 'Enable auto gain.',
             'type': 'bool',
             'default': False,
             'edit_method': {'type': 'bool'},
             # 'edit_method': {'type': 'fixed'},
             'current_value': True
            },
            {'name': 'range',
             'description': 'Sonar range (in meters)',
             'type': 'double',
             'default': 3.0,
             'edit_method': {'type': 'range', 'min': 0.3, 'max': 40.0},
             'current_value': currentConfig.range
            },
            {'name': 'gamma_correction',
             'description': 'Gamma correction',
             'type': 'int',
             'default': 127,
             'edit_method': {'type': 'range', 'min': 0, 'max': 255},
             'current_value': currentConfig.gammaCorrection
            },
            {'name': 'gain_percent',
             'description': 'Gain percentage (%)',
             'type': 'double',
             'default': 50.0,
             'edit_method': {'type': 'range', 'min': 0.1, 'max': 100.0},
             'current_value': currentConfig.gainPercent
            },
            {'name': 'sound_speed',
             'description': 'Sound speed (in m/s, set to 0 for it to be calculated using salinity).',
             'type': 'double',
             'default': 1400.0,
             'edit_method': {'type': 'range', 'min': 1400.0, 'max': 1600.0},
             'current_value': currentConfig.speedOfSound
            },
            {'name': 'use_salinity',
             'description': 'Use salinity to calculate sound_speed.',
             'type': 'bool',
             'default': True,
             'edit_method': {'type': 'bool'},
             'current_value': True,
            },
            {'name': 'salinity',
             'description': 'Salinity (in parts per thousand (ppt,ppm,g/kg), used to calculate sound speed if needed.',
             'type': 'double',
             'default': 0.0,
             'edit_method': {'type': 'range', 'min': 0.0, 'max': 100.0},
             'current_value': currentConfig.salinity
            }
        ]
