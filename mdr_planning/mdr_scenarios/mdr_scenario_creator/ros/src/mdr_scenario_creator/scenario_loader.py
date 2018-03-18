from __future__ import print_function
import yaml

from mdr_scenario_creator.scenario_params import (StateParams,
                                                  StateMachineParams,
                                                  ScenarioFileKeys)

class ScenarioLoader(object):
    '''An interface for loading scenario configuration files

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    @staticmethod
    def load_scenario(scenario_file, parent_scenario_file=''):
        '''Returns an 'mdr_scenario_creator.scenario_params.StateMachineParams'
        object containing scenario description parameters

        Keyword arguments:
        scenario_file -- path to a scenario file in yaml format
        parent_scenario_file -- path to a parent scenario file in yaml format

        '''
        scenario_params = StateMachineParams()
        parent_scenario_data = None
        if parent_scenario_file != '':
            parent_scenario_data = ScenarioLoader.__load_scenario_file(parent_scenario_file)
            scenario_params = ScenarioLoader.__load_parent_config(parent_scenario_data)

        # we load the scenario file
        scenario_data = ScenarioLoader.__load_scenario_file(scenario_file)

        # we replace the scenario ID if it's redefined in the child config
        if ScenarioFileKeys.ID in scenario_data:
            scenario_params.id = scenario_data[ScenarioFileKeys.ID]

        # we add any new states that are defined in the child config
        # to the list of scenario states
        if ScenarioFileKeys.STATES in scenario_data:
            states = scenario_data[ScenarioFileKeys.STATES]
            for state in states:
                if state not in scenario_params.states:
                    scenario_params.states.append(state)

        # we add any new outcomes that are defined in the child config
        # to the list of scenario outcomes
        if ScenarioFileKeys.OUTCOMES in scenario_data:
            outcomes = scenario_data[ScenarioFileKeys.OUTCOMES]
            for outcome in outcomes:
                if outcome not in scenario_params.outcomes:
                    scenario_params.outcomes.append(outcome)

        # we add any new states that are defined in the child config
        # to the list of scenario states; we also remove states
        # defined in the parent scenario config if the child config
        # specifies that they should be removed; states that are redefined
        # in the child config are replaced by the child definition
        for state_description in scenario_data[ScenarioFileKeys.STATE_DESCRIPTIONS]:
            state_data = state_description[ScenarioFileKeys.STATE]
            if ScenarioFileKeys.REMOVE_STATE in state_data:
                state_name = state_data[ScenarioFileKeys.STATE_NAME]
                scenario_params.state_params.pop(state_name, None)
                try:
                    scenario_params.states.remove(state_name)
                except ValueError:
                    pass
            else:
                state_params = StateParams()
                state_params.name = state_data[ScenarioFileKeys.STATE_NAME]
                state_params.state_module_name = state_data[ScenarioFileKeys.STATE_MODULE_NAME]
                state_params.state_class_name = state_data[ScenarioFileKeys.STATE_CLASS_NAME]

                for transition in state_data[ScenarioFileKeys.TRANSITIONS]:
                    transition_data = transition[ScenarioFileKeys.TRANSITION]
                    state_params.transitions[transition_data[ScenarioFileKeys.TRANSITION_NAME]] = \
                    transition_data[ScenarioFileKeys.RESULT_STATE]

                if ScenarioFileKeys.ARGS in state_data:
                    for arg in state_data[ScenarioFileKeys.ARGS]:
                        arg_data = arg[ScenarioFileKeys.ARG]
                        state_params.args[arg_data[ScenarioFileKeys.ARG_NAME]] = \
                        arg_data[ScenarioFileKeys.ARG_VALUE]

                scenario_params.state_params[state_params.name] = state_params
        return scenario_params

    @staticmethod
    def __load_scenario_file(scenario_file):
        '''Loads the file whose path is specified by 'scenario_file'

        Keyword arguments:
        scenario_file -- path to a scenario file in yaml format

        '''
        file_handle = open(scenario_file, 'r')
        scenario_data = yaml.load(file_handle)
        file_handle.close()
        return scenario_data

    @staticmethod
    def __load_parent_config(parent_scenario_data):
        '''Returns an 'mdr_scenario_creator.scenario_params.StateMachineParams'
        object containing description parameters for a generic scenario description

        Keyword arguments:
        parent_scenario_data -- an 'mdr_scenario_creator.scenario_params.StateMachineParams' object

        '''
        scenario_params = StateMachineParams()
        scenario_params.id = parent_scenario_data[ScenarioFileKeys.ID]
        scenario_params.states = parent_scenario_data[ScenarioFileKeys.STATES]
        scenario_params.outcomes = parent_scenario_data[ScenarioFileKeys.OUTCOMES]

        for state_description in parent_scenario_data[ScenarioFileKeys.STATE_DESCRIPTIONS]:
            state_data = state_description[ScenarioFileKeys.STATE]
            state_params = StateParams()
            state_params.name = state_data[ScenarioFileKeys.STATE_NAME]
            state_params.state_module_name = state_data[ScenarioFileKeys.STATE_MODULE_NAME]
            state_params.state_class_name = state_data[ScenarioFileKeys.STATE_CLASS_NAME]

            for transition in state_data[ScenarioFileKeys.TRANSITIONS]:
                transition_data = transition[ScenarioFileKeys.TRANSITION]
                state_params.transitions[transition_data[ScenarioFileKeys.TRANSITION_NAME]] = \
                transition_data[ScenarioFileKeys.RESULT_STATE]

            if ScenarioFileKeys.ARGS in state_data:
                for arg in state_data[ScenarioFileKeys.ARGS]:
                    arg_data = arg[ScenarioFileKeys.ARG]
                    state_params.args[arg_data[ScenarioFileKeys.ARG_NAME]] = \
                    arg_data[ScenarioFileKeys.ARG_VALUE]

            scenario_params.state_params[state_params.name] = state_params
        return scenario_params

    @staticmethod
    def __print_scenario_configuration(scenario_params):
        '''Prints the scenario description parameters specified by 'scenario_params'

        Keyword arguments:
        scenario_params -- an 'mdr_scenario_creator.scenario_params.StateMachineParams' object

        '''
        print('State machine ID: %s' % (scenario_params.id))
        print('States: %s' % (scenario_params.states))
        print()
        for _, state in scenario_params.state_params.items():
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
