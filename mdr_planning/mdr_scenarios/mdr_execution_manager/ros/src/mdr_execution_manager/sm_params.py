from collections import OrderedDict

class StateMachineParams(object):
    '''Defines parameters for a state machine

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        self.id = ''
        self.states = list()
        self.outcomes = list()
        self.state_params = OrderedDict()
        self.global_params = dict()

class StateParams(object):
    '''Defines parameters for a single state machine state

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        self.name = ''
        self.state_module_name = ''
        self.state_class_name = ''
        self.transitions = dict()
        self.args = dict()

class SMFileKeys(object):
    '''Defined a set of constants used in a state machine description file

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    ID = 'sm_id'
    STATES = 'states'
    OUTCOMES = 'outcomes'

    STATE_DESCRIPTIONS = 'state_descriptions'
    STATE = 'state'
    STATE_NAME = 'name'
    STATE_MODULE_NAME = 'state_module_name'
    STATE_CLASS_NAME = 'state_class_name'
    REMOVE_STATE = 'remove'

    TRANSITIONS = 'transitions'
    TRANSITION = 'transition'
    TRANSITION_NAME = 'name'
    RESULT_STATE = 'state'

    ARGS = 'arguments'
    ARG = 'argument'
    ARG_NAME = 'name'
    ARG_VALUE = 'value'
