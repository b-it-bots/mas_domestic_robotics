from __future__ import print_function
from mdr_component_monitoring.monitor_factory import MonitorFactory

class MonitorManager(object):
    def __init__(self, hw_monitor_config_params, sw_monitor_config_params):
        self.hardware_monitors = dict()
        self.software_monitors = dict()
        for monitor_config in hw_monitor_config_params:
            self.hardware_monitors[monitor_config.name] = list()
            for monitor_mode_config in monitor_config.modes:
                monitor = MonitorFactory.get_hardware_monitor(monitor_mode_config)
                self.hardware_monitors[monitor_config.name].append(monitor)

        for monitor_config in sw_monitor_config_params:
            self.software_monitors[monitor_config.name] = list()
            for monitor_mode_config in monitor_config.modes:
                monitor = MonitorFactory.get_software_monitor(monitor_mode_config)
                self.software_monitors[monitor_config.name].append(monitor)

    def monitor_components(self):
        component_status_msg = dict()
        for monitor_name, monitors in self.hardware_monitors.items():
            component_status_msg[monitor_name] = list()
            for monitor in monitors:
                monitor_status = monitor.get_status()
                component_status_msg[monitor_name].append(monitor_status)
        print(component_status_msg)
