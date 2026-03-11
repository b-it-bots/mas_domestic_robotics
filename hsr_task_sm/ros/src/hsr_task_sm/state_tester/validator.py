#!/usr/bin/env python3
"""
State Validator Module

Validates state configurations before execution to catch errors early.
Checks parameter types, required fields, action server availability, etc.
"""

import inspect
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class ValidationSeverity(Enum):
    """Severity levels for validation issues."""
    ERROR = "error"      # Cannot run
    WARNING = "warning"  # May cause issues
    INFO = "info"        # Informational


@dataclass
class ValidationIssue:
    """A single validation issue."""
    severity: ValidationSeverity
    field: str
    message: str
    suggestion: Optional[str] = None
    
    def __str__(self):
        icon = {"error": "✗", "warning": "⚠", "info": "ℹ"}[self.severity.value]
        result = f"{icon} [{self.field}] {self.message}"
        if self.suggestion:
            result += f"\n   Suggestion: {self.suggestion}"
        return result


@dataclass 
class ValidationResult:
    """Result of state validation."""
    state_name: str
    valid: bool
    issues: List[ValidationIssue]
    
    @property
    def errors(self):
        return [i for i in self.issues if i.severity == ValidationSeverity.ERROR]
    
    @property
    def warnings(self):
        return [i for i in self.issues if i.severity == ValidationSeverity.WARNING]
    
    def __str__(self):
        status = "✓ VALID" if self.valid else "✗ INVALID"
        lines = [f"{status}: {self.state_name}"]
        for issue in self.issues:
            lines.append(f"  {issue}")
        return "\n".join(lines)


class StateValidator:
    """
    Validates state configurations before execution.
    
    Checks:
    - State type exists
    - Required parameters provided
    - Parameter types correct
    - Userdata keys available
    - Action servers/services available (optional live check)
    
    Usage:
        validator = StateValidator()
        result = validator.validate('NavigateTo', 
                                    params={'destination': 'kitchen'},
                                    userdata={})
        if not result.valid:
            print(result)
    """
    
    # Known state parameters and their expected types
    KNOWN_PARAMS = {
        'NavigateTo': {
            'destination': {'type': str, 'required': False},
            'destination_key': {'type': str, 'required': False},
            'server_name': {'type': str, 'required': False, 'default': 'move_base_server'},
            'timeout': {'type': (int, float), 'required': False, 'default': 120.0},
            'retries': {'type': int, 'required': False, 'default': 2},
        },
        'NavigateToPose': {
            'x': {'type': (int, float), 'required': False},
            'y': {'type': (int, float), 'required': False},
            'theta': {'type': (int, float), 'required': False},
            'pose_key': {'type': str, 'required': False},
            'frame_id': {'type': str, 'required': False, 'default': 'map'},
        },
        'Speak': {
            'text': {'type': str, 'required': False},
            'text_key': {'type': str, 'required': False},
        },
        'WaitState': {
            'duration': {'type': (int, float), 'required': True},
        },
        'PickObject': {
            'object_name': {'type': str, 'required': False},
            'object_key': {'type': str, 'required': False},
        },
        'PlaceObject': {
            'location': {'type': str, 'required': False},
            'location_key': {'type': str, 'required': False},
        },
        'ListenWithWhisper': {
            'timeout': {'type': (int, float), 'required': False},
        },
        'GetVoicebotResponse': {
            'prompt_key': {'type': str, 'required': False},
        },
    }
    
    # Required action servers for each state
    REQUIRED_SERVERS = {
        'NavigateTo': ['move_base_server'],
        'NavigateToPose': ['move_base_server'],
        'PickObject': ['pickup_action_server'],
        'PlaceObject': ['place_action_server'],
        'FollowPerson': ['follow_person_server'],
        'Speak': ['sound_play'],
    }
    
    # Required services for each state
    REQUIRED_SERVICES = {
        'GetVoicebotResponse': ['/voicebot_service'],
        'ControlMicrophone': ['/microphone_control'],
    }
    
    def __init__(self, check_live: bool = False):
        """
        Initialize validator.
        
        Args:
            check_live: Whether to check if action servers/services are running
        """
        self.check_live = check_live
    
    def validate(self, state_name: str, 
                 params: Dict[str, Any] = None,
                 userdata: Dict[str, Any] = None) -> ValidationResult:
        """
        Validate a state configuration.
        
        Args:
            state_name: Name of the state type
            params: State constructor parameters
            userdata: Available userdata keys
            
        Returns:
            ValidationResult with any issues found
        """
        params = params or {}
        userdata = userdata or {}
        issues = []
        
        # Import state registry
        try:
            from hsr_task_sm.state_tester.core import STATE_REGISTRY
        except ImportError:
            issues.append(ValidationIssue(
                ValidationSeverity.ERROR,
                'import',
                'Could not import STATE_REGISTRY',
                'Ensure hsr_task_sm is properly installed'
            ))
            return ValidationResult(state_name, False, issues)
        
        # Check state exists
        if state_name not in STATE_REGISTRY:
            issues.append(ValidationIssue(
                ValidationSeverity.ERROR,
                'state_type',
                f"Unknown state type: {state_name}",
                f"Available types: {', '.join(sorted(STATE_REGISTRY.keys())[:10])}..."
            ))
            return ValidationResult(state_name, False, issues)
        
        state_class = STATE_REGISTRY[state_name]
        
        # Validate parameters
        param_issues = self._validate_params(state_name, state_class, params)
        issues.extend(param_issues)
        
        # Validate userdata references
        userdata_issues = self._validate_userdata(state_name, params, userdata)
        issues.extend(userdata_issues)
        
        # Check required servers/services
        if self.check_live:
            server_issues = self._check_servers(state_name)
            issues.extend(server_issues)
        
        # Determine if valid (no errors)
        valid = not any(i.severity == ValidationSeverity.ERROR for i in issues)
        
        return ValidationResult(state_name, valid, issues)
    
    def _validate_params(self, state_name: str, state_class, 
                         params: Dict) -> List[ValidationIssue]:
        """Validate constructor parameters."""
        issues = []
        
        # Get expected params from KNOWN_PARAMS or introspect
        if state_name in self.KNOWN_PARAMS:
            expected = self.KNOWN_PARAMS[state_name]
        else:
            # Introspect constructor signature
            try:
                sig = inspect.signature(state_class.__init__)
                expected = {}
                for name, param in sig.parameters.items():
                    if name == 'self':
                        continue
                    expected[name] = {
                        'type': Any,
                        'required': param.default == inspect.Parameter.empty,
                        'default': param.default if param.default != inspect.Parameter.empty else None
                    }
            except:
                expected = {}
        
        # Check for required params
        for name, spec in expected.items():
            if spec.get('required', False) and name not in params:
                issues.append(ValidationIssue(
                    ValidationSeverity.ERROR,
                    f'params.{name}',
                    f"Required parameter '{name}' is missing"
                ))
        
        # Check param types
        for name, value in params.items():
            if name in expected:
                expected_type = expected[name].get('type')
                if expected_type and expected_type != Any:
                    if not isinstance(value, expected_type):
                        issues.append(ValidationIssue(
                            ValidationSeverity.WARNING,
                            f'params.{name}',
                            f"Expected type {expected_type}, got {type(value).__name__}",
                            f"Convert value to {expected_type}"
                        ))
            else:
                # Unknown parameter
                issues.append(ValidationIssue(
                    ValidationSeverity.INFO,
                    f'params.{name}',
                    f"Unknown parameter '{name}' (may be valid)"
                ))
        
        return issues
    
    def _validate_userdata(self, state_name: str, params: Dict, 
                           userdata: Dict) -> List[ValidationIssue]:
        """Validate userdata key references."""
        issues = []
        
        # Check any param ending with '_key'
        for name, value in params.items():
            if name.endswith('_key') and isinstance(value, str):
                if value not in userdata:
                    issues.append(ValidationIssue(
                        ValidationSeverity.WARNING,
                        f'userdata.{value}',
                        f"Userdata key '{value}' referenced by '{name}' not found",
                        f"Ensure '{value}' is set in userdata before this state"
                    ))
        
        return issues
    
    def _check_servers(self, state_name: str) -> List[ValidationIssue]:
        """Check if required ROS servers are available."""
        issues = []
        
        if not ROS_AVAILABLE:
            issues.append(ValidationIssue(
                ValidationSeverity.WARNING,
                'ros',
                'ROS not available, cannot check live servers'
            ))
            return issues
        
        # Check action servers
        if state_name in self.REQUIRED_SERVERS:
            for server_name in self.REQUIRED_SERVERS[state_name]:
                if not self._check_action_server(server_name):
                    issues.append(ValidationIssue(
                        ValidationSeverity.WARNING,
                        f'server.{server_name}',
                        f"Action server '{server_name}' not responding",
                        f"Start the action server or use mock mode"
                    ))
        
        # Check services
        if state_name in self.REQUIRED_SERVICES:
            for service_name in self.REQUIRED_SERVICES[state_name]:
                if not self._check_service(service_name):
                    issues.append(ValidationIssue(
                        ValidationSeverity.WARNING,
                        f'service.{service_name}',
                        f"Service '{service_name}' not available",
                        f"Start the service or use mock mode"
                    ))
        
        return issues
    
    def _check_action_server(self, name: str, timeout: float = 1.0) -> bool:
        """Check if an action server is available."""
        try:
            import actionlib
            from actionlib_msgs.msg import GoalStatusArray
            
            # Check if status topic exists
            topics = rospy.get_published_topics()
            status_topic = f'{name}/status'
            return any(status_topic in t[0] for t in topics)
        except:
            return False
    
    def _check_service(self, name: str, timeout: float = 1.0) -> bool:
        """Check if a service is available."""
        try:
            rospy.wait_for_service(name, timeout=timeout)
            return True
        except:
            return False
    
    def validate_yaml_config(self, config: Dict) -> List[ValidationResult]:
        """
        Validate a complete YAML state machine configuration.
        
        Args:
            config: Parsed YAML config with 'states' and 'userdata' sections
            
        Returns:
            List of ValidationResults for each state
        """
        results = []
        userdata = config.get('userdata', {})
        states = config.get('states', [])
        
        for state_config in states:
            state_name = state_config.get('name', 'unnamed')
            state_type = state_config.get('type')
            params = state_config.get('params', {})
            
            if not state_type:
                results.append(ValidationResult(
                    state_name,
                    False,
                    [ValidationIssue(
                        ValidationSeverity.ERROR,
                        'type',
                        'State type not specified'
                    )]
                ))
                continue
            
            result = self.validate(state_type, params, userdata)
            result.state_name = f"{state_name} ({state_type})"
            results.append(result)
            
            # Update userdata with output keys for subsequent states
            # (simplified - assumes state might add keys)
            output_keys = state_config.get('output_keys', [])
            for key in output_keys:
                userdata[key] = None
        
        return results
