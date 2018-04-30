from __future__ import print_function
import oyaml as yaml

from mdr_execution_manager.sm_params import (StateParams,
                                             StateMachineParams,
                                             SMFileKeys)

class SMLoader(object):
    '''An interface for loading state machine configuration files

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    @staticmethod
    def load_sm(sm_file, parent_sm_file=''):
        '''Returns an 'mdr_execution_manager.sm_params.StateMachineParams'
        object containing state machine description parameters

        Keyword arguments:
        sm_file -- path to a state machine file in yaml format
        parent_sm_file -- path to a parent state machine file in yaml format

        '''
        sm_params = StateMachineParams()

        # we load the state machine file
        sm_data = SMLoader.__load_sm_file(sm_file)

        # we add any new arguments that are defined in the child config
        # to the list of global state machine arguments
        if SMFileKeys.ARGS in sm_data:
            for arg in sm_data[SMFileKeys.ARGS]:
                arg_data = arg[SMFileKeys.ARG]
                sm_params.global_params[arg_data[SMFileKeys.ARG_NAME]] = \
                arg_data[SMFileKeys.ARG_VALUE]

        parent_sm_data = None
        if parent_sm_file != '':
            parent_sm_data = SMLoader.__load_sm_file(parent_sm_file)
            sm_params = SMLoader.__load_parent_config(parent_sm_data, sm_params)

        # we replace the state machine ID if it's redefined in the child config
        if SMFileKeys.ID in sm_data:
            sm_params.id = sm_data[SMFileKeys.ID]

        # we add any new states that are defined in the child config
        # to the list of state machine states
        if SMFileKeys.STATES in sm_data:
            states = sm_data[SMFileKeys.STATES]
            for state in states:
                if state not in sm_params.states:
                    sm_params.states.append(state)

        # we add any new outcomes that are defined in the child config
        # to the list of state machine outcomes
        if SMFileKeys.OUTCOMES in sm_data:
            outcomes = sm_data[SMFileKeys.OUTCOMES]
            for outcome in outcomes:
                if outcome not in sm_params.outcomes:
                    sm_params.outcomes.append(outcome)

        # we add any new states that are defined in the child config
        # to the list of state machine states; we also remove states
        # defined in the parent state machine config if the child config
        # specifies that they should be removed; states that are redefined
        # in the child config are replaced by the child definition
        for state_description in sm_data[SMFileKeys.STATE_DESCRIPTIONS]:
            state_data = state_description[SMFileKeys.STATE]
            if SMFileKeys.REMOVE_STATE in state_data:
                state_name = state_data[SMFileKeys.STATE_NAME]
                sm_params.state_params.pop(state_name, None)
                try:
                    sm_params.states.remove(state_name)
                except ValueError:
                    pass
            else:
                state_params = StateParams()
                state_params.name = state_data[SMFileKeys.STATE_NAME]
                state_params.state_module_name = state_data[SMFileKeys.STATE_MODULE_NAME]
                state_params.state_class_name = state_data[SMFileKeys.STATE_CLASS_NAME]

                for transition in state_data[SMFileKeys.TRANSITIONS]:
                    transition_data = transition[SMFileKeys.TRANSITION]
                    state_params.transitions[transition_data[SMFileKeys.TRANSITION_NAME]] = \
                    transition_data[SMFileKeys.RESULT_STATE]

                if SMFileKeys.ARGS in state_data:
                    for arg in state_data[SMFileKeys.ARGS]:
                        arg_data = arg[SMFileKeys.ARG]
                        state_params.args[arg_data[SMFileKeys.ARG_NAME]] = \
                        arg_data[SMFileKeys.ARG_VALUE]

                for arg_name, arg_value in sm_params.global_params.items():
                    arg_data = arg[SMFileKeys.ARG]
                    state_params.args[arg_name] = arg_value

                # we add the state machine ID and the state name as additional state arguments
                state_params.args['sm_id'] = sm_params.id
                state_params.args['state_name'] = state_params.name

                sm_params.state_params[state_params.name] = state_params
        return sm_params

    @staticmethod
    def __load_sm_file(sm_file):
        '''Loads the file whose path is specified by 'sm_file'

        Keyword arguments:
        sm_file -- path to a state machine file in yaml format

        '''
        file_handle = open(sm_file, 'r')
        sm_data = yaml.load(file_handle)
        file_handle.close()
        return sm_data

    @staticmethod
    def __load_parent_config(parent_sm_data, sm_params):
        '''Returns an 'mdr_execution_manager.sm_params.StateMachineParams'
        object containing description parameters for a generic state machine description

        Keyword arguments:
        parent_sm_data -- an 'mdr_execution_manager.sm_params.StateMachineParams' object

        '''
        sm_params.id = parent_sm_data[SMFileKeys.ID]
        sm_params.states = parent_sm_data[SMFileKeys.STATES]
        sm_params.outcomes = parent_sm_data[SMFileKeys.OUTCOMES]
        if SMFileKeys.ARGS in parent_sm_data:
            for arg in parent_sm_data[SMFileKeys.ARGS]:
                arg_data = arg[SMFileKeys.ARG]
                sm_params.global_params[arg_data[SMFileKeys.ARG_NAME]] = \
                arg_data[SMFileKeys.ARG_VALUE]

        for state_description in parent_sm_data[SMFileKeys.STATE_DESCRIPTIONS]:
            state_data = state_description[SMFileKeys.STATE]
            state_params = StateParams()
            state_params.name = state_data[SMFileKeys.STATE_NAME]
            state_params.state_module_name = state_data[SMFileKeys.STATE_MODULE_NAME]
            state_params.state_class_name = state_data[SMFileKeys.STATE_CLASS_NAME]

            for transition in state_data[SMFileKeys.TRANSITIONS]:
                transition_data = transition[SMFileKeys.TRANSITION]
                state_params.transitions[transition_data[SMFileKeys.TRANSITION_NAME]] = \
                transition_data[SMFileKeys.RESULT_STATE]

            if SMFileKeys.ARGS in state_data:
                for arg in state_data[SMFileKeys.ARGS]:
                    arg_data = arg[SMFileKeys.ARG]
                    state_params.args[arg_data[SMFileKeys.ARG_NAME]] = \
                    arg_data[SMFileKeys.ARG_VALUE]

            for arg_name, arg_value in sm_params.global_params.items():
                arg_data = arg[SMFileKeys.ARG]
                state_params.args[arg_name] = arg_value

            sm_params.state_params[state_params.name] = state_params
        return sm_params

    @staticmethod
    def __print_sm_configuration(sm_params):
        '''Prints the state machine description parameters specified by 'sm_params'

        Keyword arguments:
        sm_params -- an 'mdr_execution_manager.sm_params.StateMachineParams' object

        '''
        print('State machine ID: %s' % (sm_params.id))
        print('States: %s' % (sm_params.states))
        print()
        for _, state in sm_params.state_params.items():
            print('    State: %s' % (state.name))
            print('    State module name: %s' % (state.state_module_name))
            print('    State class name: %s' % (state.state_class_name))
            print('    Transitions:')
            for transition, resulting_state in state.transitions.items():
                print('        Transition name: %s' % (transition))
                print('        Resulting state: %s' % (resulting_state))
                print()
            if state.args:
                print('    Arguments:')
                for arg, value in state.args.items():
                    print('        Name: %s' % (arg))
                    print('        Value: %s' % (value))
                    print()
