#!/usr/bin/env python3
"""
YAML State Machine Loader

Converts YAML configuration files into SMACH state machines.
This allows non-programmers to create and modify state machines
by editing simple YAML files.

Usage:
    from hsr_task_sm.yaml_sm_loader import YAMLStateMachineLoader
    
    loader = YAMLStateMachineLoader()
    sm = loader.load('/path/to/config.yaml')
    outcome = sm.execute()
"""

import os
import yaml
import rospy
import smach
import smach_ros
from typing import Dict, Any, List, Optional

# Import all available states
from hsr_task_sm.states import (
    ClearCostmap,
    PerceiveTable,
    PickObject,
    NavigateTo,
    GoToGoal,
    CheckDoorOpen,
    Speak,
    ListenForCommand,
    ParseCommand,
    DetectPerson,
    GetPersonFeatures,
    RecognizePerson,
    PlaceObject,
    FollowPerson,
    StartFollowing,
    StopFollowing,
    HandoverToHuman,
    ReceiveFromHuman,
    LookAt,
    LookAtPerson,
    LookAtObject,
    ResetGaze,
    OpenDoor,
    CloseDoor,
    OpenDrawer,
    CloseDrawer,
)


# Registry mapping state type names to classes
STATE_REGISTRY = {
    # Navigation
    'NavigateTo': NavigateTo,
    'GoToGoal': GoToGoal,
    'ClearCostmap': ClearCostmap,
    'FollowPerson': FollowPerson,
    'StartFollowing': StartFollowing,
    'StopFollowing': StopFollowing,
    
    # Perception
    'PerceiveTable': PerceiveTable,
    'DetectPerson': DetectPerson,
    'GetPersonFeatures': GetPersonFeatures,
    'RecognizePerson': RecognizePerson,
    
    # Manipulation
    'PickObject': PickObject,
    'PlaceObject': PlaceObject,
    'HandoverToHuman': HandoverToHuman,
    'ReceiveFromHuman': ReceiveFromHuman,
    
    # HRI
    'Speak': Speak,
    'ListenForCommand': ListenForCommand,
    'ParseCommand': ParseCommand,
    
    # Gaze
    'LookAt': LookAt,
    'LookAtPerson': LookAtPerson,
    'LookAtObject': LookAtObject,
    'ResetGaze': ResetGaze,
    
    # Furniture
    'OpenDoor': OpenDoor,
    'CloseDoor': CloseDoor,
    'OpenDrawer': OpenDrawer,
    'CloseDrawer': CloseDrawer,
    'CheckDoorOpen': CheckDoorOpen,
    
    # Utility states (defined below)
    'Wait': None,  # Will be set after class definition
    'SetUserdata': None,
    'CheckCondition': None,
    'CheckRetries': None,
    'IncrementCounter': None,
    'Log': None,
}


# ============================================================================
# Utility States for YAML configs
# ============================================================================

class Wait(smach.State):
    """Wait for specified duration."""
    
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.duration = duration
    
    def execute(self, userdata):
        rospy.sleep(self.duration)
        return 'succeeded'


class SetUserdata(smach.State):
    """Set userdata values."""
    
    def __init__(self, values=None):
        self.values = values or {}
        output_keys = list(self.values.keys())
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=output_keys)
    
    def execute(self, userdata):
        for key, value in self.values.items():
            setattr(userdata, key, value)
        return 'succeeded'


class CheckCondition(smach.State):
    """Check a boolean condition in userdata."""
    
    def __init__(self, condition_key='condition'):
        smach.State.__init__(
            self,
            outcomes=['true', 'false'],
            input_keys=[condition_key]
        )
        self.condition_key = condition_key
    
    def execute(self, userdata):
        value = getattr(userdata, self.condition_key, False)
        return 'true' if value else 'false'


class CheckRetries(smach.State):
    """Check retry count against maximum."""
    
    def __init__(self, max_retries=3, counter_key='retry_count'):
        smach.State.__init__(
            self,
            outcomes=['retry', 'max_reached'],
            input_keys=[counter_key],
            output_keys=[counter_key]
        )
        self.max_retries = max_retries
        self.counter_key = counter_key
    
    def execute(self, userdata):
        count = getattr(userdata, self.counter_key, 0)
        count += 1
        setattr(userdata, self.counter_key, count)
        
        if count >= self.max_retries:
            return 'max_reached'
        return 'retry'


class IncrementCounter(smach.State):
    """Increment a counter in userdata."""
    
    def __init__(self, counter_key='counter', increment=1):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=[counter_key],
            output_keys=[counter_key]
        )
        self.counter_key = counter_key
        self.increment = increment
    
    def execute(self, userdata):
        value = getattr(userdata, self.counter_key, 0)
        setattr(userdata, self.counter_key, value + self.increment)
        return 'succeeded'


class Log(smach.State):
    """Log a message."""
    
    def __init__(self, message='', level='info'):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.message = message
        self.level = level
    
    def execute(self, userdata):
        log_func = {
            'debug': rospy.logdebug,
            'info': rospy.loginfo,
            'warn': rospy.logwarn,
            'error': rospy.logerr,
        }.get(self.level, rospy.loginfo)
        log_func('[YAML_SM] %s', self.message)
        return 'succeeded'


# Register utility states
STATE_REGISTRY['Wait'] = Wait
STATE_REGISTRY['SetUserdata'] = SetUserdata
STATE_REGISTRY['CheckCondition'] = CheckCondition
STATE_REGISTRY['CheckRetries'] = CheckRetries
STATE_REGISTRY['IncrementCounter'] = IncrementCounter
STATE_REGISTRY['Log'] = Log


# ============================================================================
# YAML State Machine Loader
# ============================================================================

class YAMLStateMachineLoader:
    """
    Loads state machine configurations from YAML files.
    
    Example YAML:
        name: my_task
        description: "Task description"
        
        userdata:
            location: "kitchen"
            max_retries: 3
        
        states:
          - name: START
            type: Speak
            params:
                text: "Starting task"
            transitions:
                succeeded: NAVIGATE
                failed: NAVIGATE
        
        outcomes:
          - SUCCEEDED
          - FAILED
    """
    
    def __init__(self, state_registry: Dict = None):
        """
        Initialize loader with optional custom state registry.
        
        Args:
            state_registry: Dict mapping type names to state classes
        """
        self.state_registry = state_registry or STATE_REGISTRY
        self._loaded_configs = {}  # Cache for sub-state machines
    
    def register_state(self, type_name: str, state_class: type):
        """Register a custom state type."""
        self.state_registry[type_name] = state_class
    
    def load(self, config_path: str) -> smach.StateMachine:
        """
        Load a state machine from a YAML file.
        
        Args:
            config_path: Path to YAML configuration file
            
        Returns:
            Configured SMACH StateMachine
        """
        # Resolve path
        if not os.path.isabs(config_path):
            # Look in package config directory
            pkg_path = self._get_package_path()
            config_path = os.path.join(pkg_path, 'config', 'challenges', config_path)
        
        rospy.loginfo('[YAMLLoader] Loading config: %s', config_path)
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        return self._build_sm(config)
    
    def load_from_string(self, yaml_string: str) -> smach.StateMachine:
        """Load state machine from YAML string."""
        config = yaml.safe_load(yaml_string)
        return self._build_sm(config)
    
    def _build_sm(self, config: Dict) -> smach.StateMachine:
        """Build state machine from config dict."""
        name = config.get('name', 'unnamed_sm')
        description = config.get('description', '')
        outcomes = config.get('outcomes', ['SUCCEEDED', 'FAILED'])
        
        rospy.loginfo('[YAMLLoader] Building SM: %s - %s', name, description)
        
        # Create state machine
        sm = smach.StateMachine(outcomes=outcomes)
        
        # Set initial userdata
        userdata = config.get('userdata', {})
        for key, value in userdata.items():
            setattr(sm.userdata, key, value)
        
        # Add states
        states = config.get('states', [])
        with sm:
            for state_config in states:
                self._add_state(sm, state_config)
        
        return sm
    
    def _add_state(self, sm: smach.StateMachine, state_config: Dict):
        """Add a single state to the state machine."""
        name = state_config['name']
        state_type = state_config['type']
        params = state_config.get('params', {})
        transitions = state_config.get('transitions', {})
        remapping = state_config.get('remapping', {})
        
        rospy.logdebug('[YAMLLoader] Adding state: %s (type=%s)', name, state_type)
        
        # Handle special state types
        if state_type == 'SubStateMachine':
            state = self._load_sub_sm(params)
        else:
            state = self._create_state(state_type, params)
        
        if state is None:
            rospy.logerr('[YAMLLoader] Failed to create state: %s', name)
            return
        
        smach.StateMachine.add(
            name,
            state,
            transitions=transitions,
            remapping=remapping
        )
    
    def _create_state(self, state_type: str, params: Dict) -> Optional[smach.State]:
        """Create a state instance from type name and parameters."""
        state_class = self.state_registry.get(state_type)
        
        if state_class is None:
            rospy.logerr('[YAMLLoader] Unknown state type: %s', state_type)
            rospy.logerr('[YAMLLoader] Available types: %s', list(self.state_registry.keys()))
            return None
        
        # Process parameters
        processed_params = self._process_params(params)
        
        try:
            return state_class(**processed_params)
        except Exception as e:
            rospy.logerr('[YAMLLoader] Error creating %s: %s', state_type, e)
            return None
    
    def _process_params(self, params: Dict) -> Dict:
        """
        Process parameters, resolving special suffixes:
        - _key: Get value from userdata at runtime
        - _param: Get value from ROS param server
        """
        processed = {}
        
        for key, value in params.items():
            if key.endswith('_param'):
                # Get from ROS param server
                actual_key = key[:-6]  # Remove '_param' suffix
                processed[actual_key] = rospy.get_param(value, None)
            elif key.endswith('_key'):
                # Keep as-is, state should handle userdata lookup
                actual_key = key[:-4]  # Remove '_key' suffix
                # Pass the key name so state can look it up
                processed[f'{actual_key}_key'] = value
            else:
                processed[key] = value
        
        return processed
    
    def _load_sub_sm(self, params: Dict) -> Optional[smach.StateMachine]:
        """Load a sub state machine from config reference."""
        config_path = params.get('config')
        if not config_path:
            rospy.logerr('[YAMLLoader] SubStateMachine missing config param')
            return None
        
        # Check cache
        if config_path in self._loaded_configs:
            return self._loaded_configs[config_path]
        
        # Load and cache
        sub_sm = self.load(config_path)
        self._loaded_configs[config_path] = sub_sm
        return sub_sm
    
    def _get_package_path(self) -> str:
        """Get the hsr_task_sm package path."""
        try:
            import rospkg
            rospack = rospkg.RosPack()
            return rospack.get_path('hsr_task_sm')
        except Exception:
            # Fallback to relative path
            return os.path.dirname(os.path.dirname(os.path.dirname(__file__)))


def validate_config(config_path: str) -> List[str]:
    """
    Validate a YAML configuration file.
    
    Returns:
        List of validation errors (empty if valid)
    """
    errors = []
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        return [f'Failed to parse YAML: {e}']
    
    # Check required fields
    if 'states' not in config:
        errors.append('Missing required field: states')
    
    if 'outcomes' not in config:
        errors.append('Missing required field: outcomes')
    
    # Validate states
    states = config.get('states', [])
    state_names = set()
    outcomes = set(config.get('outcomes', []))
    
    for i, state in enumerate(states):
        if 'name' not in state:
            errors.append(f'State {i}: missing name')
            continue
        
        name = state['name']
        state_names.add(name)
        
        if 'type' not in state:
            errors.append(f'State {name}: missing type')
        elif state['type'] not in STATE_REGISTRY and state['type'] != 'SubStateMachine':
            errors.append(f'State {name}: unknown type "{state["type"]}"')
        
        if 'transitions' not in state:
            errors.append(f'State {name}: missing transitions')
    
    # Check that all transition targets exist
    for state in states:
        name = state.get('name', '?')
        transitions = state.get('transitions', {})
        for outcome, target in transitions.items():
            if target not in state_names and target not in outcomes:
                errors.append(f'State {name}: transition "{outcome}" -> "{target}" targets unknown state')
    
    return errors


# ============================================================================
# Main entry point for testing
# ============================================================================

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) < 2:
        print('Usage: yaml_sm_loader.py <config.yaml> [--validate]')
        sys.exit(1)
    
    config_path = sys.argv[1]
    
    if '--validate' in sys.argv:
        errors = validate_config(config_path)
        if errors:
            print('Validation errors:')
            for error in errors:
                print(f'  - {error}')
            sys.exit(1)
        else:
            print('Configuration is valid!')
            sys.exit(0)
    
    # Normal execution
    rospy.init_node('yaml_sm_loader_test')
    loader = YAMLStateMachineLoader()
    sm = loader.load(config_path)
    
    # Run with introspection
    sis = smach_ros.IntrospectionServer('yaml_sm', sm, '/YAML_SM')
    sis.start()
    
    outcome = sm.execute()
    rospy.loginfo('Final outcome: %s', outcome)
    
    sis.stop()
