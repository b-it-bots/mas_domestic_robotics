#!/usr/bin/env python3
"""
Core State Testing Module

Provides StateTester class for running individual states with:
- Mock service support for offline testing
- Userdata injection
- Timeout handling
- Result capturing
"""

import time
import traceback
import threading
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional, Type
from enum import Enum

try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rospy = None

import smach

# Import available states dynamically (handles missing dependencies gracefully)
from hsr_task_sm.states import get_available_states, get_import_errors

# Import utility states from simple_action_base
from hsr_task_sm.simple_action_base import (
    SimpleActionState, SimpleServiceState, SimpleTopicState,
    UserDataTransfer, WaitState, CheckCondition, IncrementCounter, CheckRetries
)


class TestMode(Enum):
    """Test execution modes."""
    LIVE = "live"        # Use real ROS services
    MOCK = "mock"        # Use mock services
    DRY_RUN = "dry_run"  # Don't execute, just validate


@dataclass
class StateTestResult:
    """Result of a state test execution."""
    state_name: str
    outcome: str
    success: bool
    execution_time: float
    input_userdata: Dict[str, Any]
    output_userdata: Dict[str, Any]
    error: Optional[str] = None
    traceback: Optional[str] = None
    logs: List[str] = field(default_factory=list)
    
    def __str__(self):
        status = "✓ PASSED" if self.success else "✗ FAILED"
        result = f"{status} - {self.state_name}: {self.outcome} ({self.execution_time:.2f}s)"
        if self.error:
            result += f"\n  Error: {self.error}"
        return result
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization."""
        return {
            'state_name': self.state_name,
            'outcome': self.outcome,
            'success': self.success,
            'execution_time': self.execution_time,
            'input_userdata': self.input_userdata,
            'output_userdata': self.output_userdata,
            'error': self.error,
            'logs': self.logs
        }


# Build STATE_REGISTRY dynamically from available states
def _build_state_registry() -> Dict[str, Type[smach.State]]:
    """Build registry of all available state classes."""
    registry = {}
    
    # Add states that were successfully imported
    available = get_available_states()
    registry.update(available)
    
    # Add utility states from simple_action_base
    registry.update({
        'UserDataTransfer': UserDataTransfer,
        'WaitState': WaitState,
        'CheckCondition': CheckCondition,
        'IncrementCounter': IncrementCounter,
        'CheckRetries': CheckRetries,
    })
    
    return registry

STATE_REGISTRY: Dict[str, Type[smach.State]] = _build_state_registry()

# Log any import errors for debugging
_import_errors = get_import_errors()
if _import_errors:
    import sys
    print(f"[StateTester] Warning: Some states unavailable due to missing dependencies:", file=sys.stderr)
    for err in _import_errors:
        print(f"  - {err}", file=sys.stderr)


class LogCapture:
    """Captures rospy log messages during test execution."""
    
    def __init__(self):
        self.logs = []
        self._original_loginfo = None
        self._original_logwarn = None
        self._original_logerr = None
    
    def start(self):
        """Start capturing logs."""
        if not ROS_AVAILABLE or rospy is None:
            return
            
        self._original_loginfo = rospy.loginfo
        self._original_logwarn = rospy.logwarn
        self._original_logerr = rospy.logerr
        
        def capture_info(msg, *args):
            self.logs.append(f"[INFO] {msg}")
            if self._original_loginfo:
                self._original_loginfo(msg, *args)
        
        def capture_warn(msg, *args):
            self.logs.append(f"[WARN] {msg}")
            if self._original_logwarn:
                self._original_logwarn(msg, *args)
        
        def capture_err(msg, *args):
            self.logs.append(f"[ERROR] {msg}")
            if self._original_logerr:
                self._original_logerr(msg, *args)
        
        rospy.loginfo = capture_info
        rospy.logwarn = capture_warn
        rospy.logerr = capture_err
    
    def stop(self):
        """Stop capturing and restore original loggers."""
        if not ROS_AVAILABLE or rospy is None:
            return
            
        if self._original_loginfo:
            rospy.loginfo = self._original_loginfo
        if self._original_logwarn:
            rospy.logwarn = self._original_logwarn
        if self._original_logerr:
            rospy.logerr = self._original_logerr
    
    def get_logs(self) -> List[str]:
        return self.logs.copy()


class MockUserData:
    """Mock userdata container for testing."""
    
    def __init__(self, data: Dict[str, Any] = None):
        self._data = data or {}
        for key, value in self._data.items():
            setattr(self, key, value)
    
    def __getattr__(self, name):
        if name.startswith('_'):
            return super().__getattribute__(name)
        return self._data.get(name)
    
    def __setattr__(self, name, value):
        if name.startswith('_'):
            super().__setattr__(name, value)
        else:
            self._data[name] = value
    
    def to_dict(self) -> Dict[str, Any]:
        return self._data.copy()
    
    def __contains__(self, key):
        return key in self._data
    
    def keys(self):
        return self._data.keys()


class StateTester:
    """
    Main class for testing individual SMACH states.
    
    Features:
    - Run states in isolation with mock or live services
    - Inject custom userdata
    - Capture logs and timing
    - Validate state configurations
    
    Example:
        tester = StateTester(mode=TestMode.MOCK)
        result = tester.test_state('NavigateTo', 
                                   params={'destination': 'kitchen'},
                                   timeout=10.0)
        print(result)
    """
    
    def __init__(self, mode: TestMode = TestMode.MOCK, 
                 mock_services: 'MockServices' = None,
                 init_ros: bool = True):
        """
        Initialize the state tester.
        
        Args:
            mode: Test execution mode (LIVE, MOCK, or DRY_RUN)
            mock_services: Optional pre-configured MockServices instance
            init_ros: Whether to initialize ROS node if not running
        """
        self.mode = mode
        self.mock_services = mock_services
        self.results: List[StateTestResult] = []
        
        if init_ros:
            self._ensure_ros_init()
    
    def _ensure_ros_init(self):
        """Ensure ROS node is initialized."""
        if not ROS_AVAILABLE or rospy is None:
            return
            
        try:
            rospy.get_rostime()
        except rospy.ROSInitException:
            try:
                rospy.init_node('state_tester', anonymous=True)
            except rospy.ROSException:
                pass  # Node already initialized
        except Exception:
            pass  # ROS not running
    
    def get_available_states(self) -> List[str]:
        """Get list of all available state types."""
        return sorted(STATE_REGISTRY.keys())
    
    def get_state_info(self, state_name: str) -> Dict[str, Any]:
        """Get detailed information about a state type."""
        if state_name not in STATE_REGISTRY:
            return {'error': f'Unknown state: {state_name}'}
        
        state_class = STATE_REGISTRY[state_name]
        
        # Get constructor parameters
        import inspect
        sig = inspect.signature(state_class.__init__)
        params = {}
        for name, param in sig.parameters.items():
            if name == 'self':
                continue
            params[name] = {
                'default': str(param.default) if param.default != inspect.Parameter.empty else 'required',
                'annotation': str(param.annotation) if param.annotation != inspect.Parameter.empty else 'Any'
            }
        
        # Get outcomes
        try:
            # Try to instantiate with minimal params to get outcomes
            instance = state_class()
            outcomes = list(instance.get_registered_outcomes())
        except:
            outcomes = ['succeeded', 'failed']  # Common defaults
        
        return {
            'name': state_name,
            'class': state_class.__name__,
            'module': state_class.__module__,
            'docstring': state_class.__doc__ or "No documentation",
            'parameters': params,
            'outcomes': outcomes
        }
    
    def create_state(self, state_name: str, params: Dict[str, Any] = None) -> smach.State:
        """
        Create a state instance.
        
        Args:
            state_name: Name of the state type
            params: Constructor parameters
            
        Returns:
            Configured state instance
        """
        if state_name not in STATE_REGISTRY:
            raise ValueError(f"Unknown state type: {state_name}")
        
        state_class = STATE_REGISTRY[state_name]
        params = params or {}
        
        return state_class(**params)
    
    def test_state(self, 
                   state_name: str,
                   params: Dict[str, Any] = None,
                   userdata: Dict[str, Any] = None,
                   expected_outcome: str = None,
                   timeout: float = 30.0) -> StateTestResult:
        """
        Test a single state.
        
        Args:
            state_name: Name of the state type to test
            params: State constructor parameters
            userdata: Input userdata dictionary
            expected_outcome: Expected outcome (for pass/fail determination)
            timeout: Execution timeout in seconds
            
        Returns:
            StateTestResult with execution details
        """
        params = params or {}
        userdata = userdata or {}
        
        # Validate state exists
        if state_name not in STATE_REGISTRY:
            return StateTestResult(
                state_name=state_name,
                outcome='error',
                success=False,
                execution_time=0.0,
                input_userdata=userdata,
                output_userdata={},
                error=f"Unknown state type: {state_name}"
            )
        
        # DRY_RUN mode just validates
        if self.mode == TestMode.DRY_RUN:
            return self._dry_run_test(state_name, params, userdata)
        
        # Create state and userdata
        log_capture = LogCapture()
        mock_ud = MockUserData(userdata)
        
        try:
            state = self.create_state(state_name, params)
        except Exception as e:
            return StateTestResult(
                state_name=state_name,
                outcome='error',
                success=False,
                execution_time=0.0,
                input_userdata=userdata,
                output_userdata={},
                error=f"Failed to create state: {str(e)}",
                traceback=traceback.format_exc()
            )
        
        # Execute with timeout
        outcome = None
        error = None
        tb = None
        start_time = time.time()
        
        log_capture.start()
        try:
            if self.mode == TestMode.MOCK and self.mock_services:
                self.mock_services.activate()
            
            # Run state in thread with timeout
            result_container = {'outcome': None, 'error': None}
            
            def run_state():
                try:
                    result_container['outcome'] = state.execute(mock_ud)
                except Exception as e:
                    result_container['error'] = str(e)
                    result_container['traceback'] = traceback.format_exc()
            
            thread = threading.Thread(target=run_state)
            thread.start()
            thread.join(timeout=timeout)
            
            if thread.is_alive():
                outcome = 'timeout'
                error = f"State execution timed out after {timeout}s"
            else:
                outcome = result_container['outcome'] or 'error'
                error = result_container.get('error')
                tb = result_container.get('traceback')
                
        except Exception as e:
            outcome = 'error'
            error = str(e)
            tb = traceback.format_exc()
        finally:
            log_capture.stop()
            if self.mode == TestMode.MOCK and self.mock_services:
                self.mock_services.deactivate()
        
        execution_time = time.time() - start_time
        
        # Determine success
        if expected_outcome:
            success = (outcome == expected_outcome and error is None)
        else:
            success = (outcome in ['succeeded', 'running'] and error is None)
        
        result = StateTestResult(
            state_name=state_name,
            outcome=outcome,
            success=success,
            execution_time=execution_time,
            input_userdata=userdata,
            output_userdata=mock_ud.to_dict(),
            error=error,
            traceback=tb,
            logs=log_capture.get_logs()
        )
        
        self.results.append(result)
        return result
    
    def _dry_run_test(self, state_name: str, params: Dict, userdata: Dict) -> StateTestResult:
        """Validate state config without execution."""
        try:
            state = self.create_state(state_name, params)
            return StateTestResult(
                state_name=state_name,
                outcome='validated',
                success=True,
                execution_time=0.0,
                input_userdata=userdata,
                output_userdata=userdata,
                logs=['[DRY_RUN] State configuration validated successfully']
            )
        except Exception as e:
            return StateTestResult(
                state_name=state_name,
                outcome='validation_failed',
                success=False,
                execution_time=0.0,
                input_userdata=userdata,
                output_userdata={},
                error=f"Validation failed: {str(e)}",
                traceback=traceback.format_exc()
            )
    
    def run_test_suite(self, tests: List[Dict[str, Any]]) -> List[StateTestResult]:
        """
        Run a suite of tests.
        
        Args:
            tests: List of test configs, each with keys:
                   - state_name (required)
                   - params (optional)
                   - userdata (optional)
                   - expected_outcome (optional)
                   
        Returns:
            List of StateTestResult
        """
        results = []
        for test in tests:
            result = self.test_state(
                state_name=test['state_name'],
                params=test.get('params', {}),
                userdata=test.get('userdata', {}),
                expected_outcome=test.get('expected_outcome')
            )
            results.append(result)
        return results
    
    def get_summary(self) -> Dict[str, Any]:
        """Get summary of all test results."""
        if not self.results:
            return {'total': 0, 'passed': 0, 'failed': 0}
        
        passed = sum(1 for r in self.results if r.success)
        failed = len(self.results) - passed
        total_time = sum(r.execution_time for r in self.results)
        
        return {
            'total': len(self.results),
            'passed': passed,
            'failed': failed,
            'pass_rate': f"{100 * passed / len(self.results):.1f}%",
            'total_time': f"{total_time:.2f}s",
            'results': [r.to_dict() for r in self.results]
        }
    
    def clear_results(self):
        """Clear all stored test results."""
        self.results = []
