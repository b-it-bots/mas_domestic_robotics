class HeadControllerBase(object):
    def look_up(self):
        raise NotImplementedError('look_up not implemented')

    def look_down(self):
        raise NotImplementedError('look_down not implemented')

    def turn_left(self):
        raise NotImplementedError('turn_left not implemented')

    def turn_right(self):
        raise NotImplementedError('turn_right not implemented')
