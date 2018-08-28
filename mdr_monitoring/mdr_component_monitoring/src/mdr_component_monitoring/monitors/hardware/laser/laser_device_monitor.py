from os import stat

from mdr_component_monitoring.monitor_base import MonitorBase

class LaserDeviceMonitor(MonitorBase):
    def __init__(self, config_params):
        super(LaserDeviceMonitor, self).__init__(config_params)
        self.dev_names = list()
        self.dev_status_names = list()
        for mapping in config_params.mappings:
            self.dev_names.append(mapping.inputs[0])
            self.dev_status_names.append(mapping.outputs[0].name)

    def get_status(self):
        status_msg = self.get_status_message_template()
        status_msg['monitorName'] = self.config_params.name
        status_msg['healthStatus'] = dict()
        for i, dev_name in enumerate(self.dev_names):
            status_msg['healthStatus'][self.dev_status_names[i]] = self.__device_exists(dev_name)
        return status_msg

    def __device_exists(self, dev_name):
        try:
            stat(dev_name)
            return True
        except OSError:
            return False
