class HeadControllerBase(object):
    def __init__(self):
        self.actions = ['look_up', 'look_down', 'turn_left', 'turn_right']

    def look_up(self):
        raise NotImplementedError('look_up not implemented')

    def look_down(self):
        raise NotImplementedError('look_down not implemented')

    def turn_left(self):
        raise NotImplementedError('turn_left not implemented')

    def turn_right(self):
        raise NotImplementedError('turn_right not implemented')

    def reset(self):
        raise NotImplementedError('reset not implemented')

    def tilt(self, angle):
        """Tilt head by a specific angle in radians"""
        raise NotImplementedError('tilt not implemented')

    def pan(self, angle):
        """Pan head by a specific angle in radians"""
        raise NotImplementedError('pan not implemented')
