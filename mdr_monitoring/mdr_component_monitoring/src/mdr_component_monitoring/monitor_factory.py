from mdr_component_monitoring.config.config_params import HardwareMonitorNames
from mdr_component_monitoring.monitor_base import MonitorBase

from mdr_component_monitoring.monitors.hardware.laser.laser_device_monitor import LaserDeviceMonitor

'''A factory for creating component monitors

@author Alex Mitrevski, Santosh Thoduka
@contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
'''
class MonitorFactory(object):
    '''Returns a hardware monitor as specified by the given name

    Keyword arguments:
    @param monitor_name monitor description name as specified in 'config_enums/HardwareMonitorNames'

    '''
    @staticmethod
    def get_hardware_monitor(monitor_config_params):
        if monitor_config_params.name == HardwareMonitorNames.LASER_DEVICE_MONITOR:
            monitor = LaserDeviceMonitor(monitor_config_params)
            return monitor
        return MonitorBase(monitor_config_params)

    '''Returns a software monitor as specified by the given name

    Keyword arguments:
    @param monitor_name monitor description name as specified in 'config_enums/HardwareMonitorNames'

    '''
    @staticmethod
    def get_software_monitor(monitor_config_params):
        return MonitorBase(monitor_config_params)
