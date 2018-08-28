from abc import abstractmethod

class MonitorBase(object):
    def __init__(self, config_params):
        self.config_params = config_params

    @abstractmethod
    def get_status(self):
        pass

    def get_status_message_template(self):
        msg = dict()
        msg['robotId'] = ''
        return msg
