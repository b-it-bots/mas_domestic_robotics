#!/usr/bin/env python3
"""
Mock Services for Offline State Testing

Simulates ROS action servers and services to allow state testing
without a running robot or ROS infrastructure.

These mocks return configurable responses and can simulate:
- Success/failure scenarios
- Delays and timeouts
- Various response data
"""

import time
import threading
from typing import Dict, Any, Callable, Optional
from dataclasses import dataclass
from enum import Enum

try:
    import rospy
    from actionlib import SimpleActionServer, GoalStatus
    from actionlib_msgs.msg import GoalStatus as GoalStatusMsg
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class MockResult(Enum):
    """Configurable mock outcomes."""
    SUCCESS = "success"
    FAILURE = "failure"
    PREEMPTED = "preempted"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass
class MockConfig:
    """Configuration for a mock service/action."""
    result: MockResult = MockResult.SUCCESS
    delay: float = 0.1  # Simulated execution time
    response_data: Dict[str, Any] = None
    callback: Callable = None  # Custom response generator
    
    def __post_init__(self):
        if self.response_data is None:
            self.response_data = {}


class MockActionClient:
    """
    Mock action client that simulates action server responses.
    
    Drop-in replacement for SimpleActionClient during testing.
    """
    
    def __init__(self, name: str, action_type, config: MockConfig = None):
        self.name = name
        self.action_type = action_type
        self.config = config or MockConfig()
        self._goal = None
        self._result = None
        self._status = GoalStatusMsg.PENDING if ROS_AVAILABLE else 0
        self._feedback_cb = None
        self._done_cb = None
        
    def wait_for_server(self, timeout=None):
        """Always returns True (server is "available")."""
        time.sleep(0.01)  # Simulate brief connection time
        return True
    
    def send_goal(self, goal, done_cb=None, feedback_cb=None):
        """Store goal and prepare mock result."""
        self._goal = goal
        self._done_cb = done_cb
        self._feedback_cb = feedback_cb
        self._status = GoalStatusMsg.ACTIVE if ROS_AVAILABLE else 1
        
        # Generate result in background
        def process_goal():
            if self.config.delay > 0:
                time.sleep(self.config.delay)
            
            # Create result based on config
            if self.config.result == MockResult.SUCCESS:
                self._status = GoalStatusMsg.SUCCEEDED if ROS_AVAILABLE else 3
                self._result = self._create_success_result()
            elif self.config.result == MockResult.FAILURE:
                self._status = GoalStatusMsg.ABORTED if ROS_AVAILABLE else 4
                self._result = self._create_failure_result()
            elif self.config.result == MockResult.PREEMPTED:
                self._status = GoalStatusMsg.PREEMPTED if ROS_AVAILABLE else 2
                self._result = None
            elif self.config.result == MockResult.TIMEOUT:
                self._status = GoalStatusMsg.LOST if ROS_AVAILABLE else 9
                self._result = None
            
            if self._done_cb:
                self._done_cb(self._status, self._result)
        
        threading.Thread(target=process_goal).start()
    
    def _create_success_result(self):
        """Create a success result object."""
        try:
            result_class = self.action_type().action_result.result.__class__
            result = result_class()
            result.success = True
            if hasattr(result, 'message'):
                result.message = "Mock success"
            return result
        except:
            return type('MockResult', (), {'success': True})()
    
    def _create_failure_result(self):
        """Create a failure result object."""
        try:
            result_class = self.action_type().action_result.result.__class__
            result = result_class()
            result.success = False
            if hasattr(result, 'message'):
                result.message = "Mock failure"
            return result
        except:
            return type('MockResult', (), {'success': False})()
    
    def wait_for_result(self, timeout=None):
        """Wait for goal completion."""
        if timeout is None:
            timeout = 30.0
        
        start = time.time()
        while time.time() - start < timeout:
            if self._status in [3, 4, 2, 5, 9]:  # Terminal states
                return True
            time.sleep(0.05)
        return False
    
    def get_result(self):
        """Get the action result."""
        return self._result
    
    def get_state(self):
        """Get current goal status."""
        return self._status
    
    def cancel_goal(self):
        """Cancel the current goal."""
        self._status = GoalStatusMsg.PREEMPTED if ROS_AVAILABLE else 2


class MockServiceProxy:
    """
    Mock service proxy for simulating ROS services.
    """
    
    def __init__(self, name: str, service_type, config: MockConfig = None):
        self.name = name
        self.service_type = service_type
        self.config = config or MockConfig()
        self._calls = []
    
    def wait_for_service(self, timeout=None):
        """Always succeeds."""
        return True
    
    def __call__(self, *args, **kwargs):
        """Handle service call."""
        self._calls.append({'args': args, 'kwargs': kwargs})
        
        if self.config.delay > 0:
            time.sleep(self.config.delay)
        
        # Use custom callback if provided
        if self.config.callback:
            return self.config.callback(*args, **kwargs)
        
        # Generate default response
        try:
            response = self.service_type._response_class()
            if hasattr(response, 'success'):
                response.success = (self.config.result == MockResult.SUCCESS)
            if hasattr(response, 'message'):
                response.message = f"Mock response for {self.name}"
            return response
        except:
            return type('MockResponse', (), {'success': self.config.result == MockResult.SUCCESS})()
    
    def get_call_history(self):
        """Get history of service calls."""
        return self._calls


class MockPublisher:
    """Mock publisher for testing."""
    
    def __init__(self, topic: str, msg_type, queue_size=10):
        self.topic = topic
        self.msg_type = msg_type
        self.messages = []
    
    def publish(self, msg):
        """Store published message."""
        self.messages.append(msg)
    
    def get_num_connections(self):
        return 1
    
    def get_published_messages(self):
        return self.messages


class MockSubscriber:
    """Mock subscriber that can inject messages."""
    
    def __init__(self, topic: str, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self._active = True
    
    def inject_message(self, msg):
        """Inject a message to trigger callback."""
        if self._active and self.callback:
            self.callback(msg)
    
    def unregister(self):
        self._active = False


class MockServices:
    """
    Central manager for all mock services during testing.
    
    Provides configurable mocks for common action servers and services
    used by states.
    
    Usage:
        mocks = MockServices()
        mocks.configure('move_base_server', MockResult.SUCCESS, delay=2.0)
        mocks.activate()
        
        # Run your state tests...
        
        mocks.deactivate()
    """
    
    # Default configurations for common services
    DEFAULT_CONFIGS = {
        # Navigation
        'move_base_server': MockConfig(MockResult.SUCCESS, delay=1.0),
        '/move_base': MockConfig(MockResult.SUCCESS, delay=1.0),
        
        # Manipulation
        'pickup_action_server': MockConfig(MockResult.SUCCESS, delay=2.0),
        'place_action_server': MockConfig(MockResult.SUCCESS, delay=2.0),
        'arm_action_server': MockConfig(MockResult.SUCCESS, delay=1.5),
        
        # Perception
        'perceive_plane_server': MockConfig(MockResult.SUCCESS, delay=1.0, 
                                            response_data={'objects': ['cup', 'bottle']}),
        
        # Speech
        'sound_play': MockConfig(MockResult.SUCCESS, delay=0.5),
        '/voicebot_service': MockConfig(MockResult.SUCCESS, delay=0.3,
                                        response_data={'response': 'Hello, I am a robot.'}),
    }
    
    def __init__(self):
        self.configs: Dict[str, MockConfig] = dict(self.DEFAULT_CONFIGS)
        self.action_clients: Dict[str, MockActionClient] = {}
        self.service_proxies: Dict[str, MockServiceProxy] = {}
        self.publishers: Dict[str, MockPublisher] = {}
        self.subscribers: Dict[str, MockSubscriber] = {}
        self._active = False
        self._original_actionlib = None
        self._original_rospy = None
    
    def configure(self, name: str, result: MockResult = MockResult.SUCCESS,
                  delay: float = 0.1, response_data: Dict = None,
                  callback: Callable = None):
        """
        Configure a specific mock.
        
        Args:
            name: Service/action server name
            result: Expected outcome
            delay: Simulated execution time
            response_data: Custom response data
            callback: Custom response generator function
        """
        self.configs[name] = MockConfig(
            result=result,
            delay=delay,
            response_data=response_data or {},
            callback=callback
        )
    
    def configure_success(self, name: str, delay: float = 0.1):
        """Shorthand to configure successful mock."""
        self.configure(name, MockResult.SUCCESS, delay)
    
    def configure_failure(self, name: str, delay: float = 0.1):
        """Shorthand to configure failing mock."""
        self.configure(name, MockResult.FAILURE, delay)
    
    def get_action_client(self, name: str, action_type) -> MockActionClient:
        """Get or create a mock action client."""
        if name not in self.action_clients:
            config = self.configs.get(name, MockConfig())
            self.action_clients[name] = MockActionClient(name, action_type, config)
        return self.action_clients[name]
    
    def get_service_proxy(self, name: str, service_type) -> MockServiceProxy:
        """Get or create a mock service proxy."""
        if name not in self.service_proxies:
            config = self.configs.get(name, MockConfig())
            self.service_proxies[name] = MockServiceProxy(name, service_type, config)
        return self.service_proxies[name]
    
    def activate(self):
        """
        Activate mock services by monkey-patching actionlib/rospy.
        
        This redirects action client and service proxy creation to use mocks.
        """
        if self._active:
            return
        
        self._active = True
        
        # Store originals for restoration
        if ROS_AVAILABLE:
            import actionlib
            self._original_actionlib_client = actionlib.SimpleActionClient
            self._original_service_proxy = rospy.ServiceProxy
            self._original_publisher = rospy.Publisher
            
            # Replace with mock factories
            mock_services = self
            
            class MockedActionClient:
                def __new__(cls, name, action_type):
                    return mock_services.get_action_client(name, action_type)
            
            class MockedServiceProxy:
                def __new__(cls, name, service_type, *args, **kwargs):
                    return mock_services.get_service_proxy(name, service_type)
            
            actionlib.SimpleActionClient = MockedActionClient
            rospy.ServiceProxy = MockedServiceProxy
    
    def deactivate(self):
        """Restore original actionlib/rospy functions."""
        if not self._active:
            return
        
        if ROS_AVAILABLE:
            import actionlib
            if self._original_actionlib_client:
                actionlib.SimpleActionClient = self._original_actionlib_client
            if self._original_service_proxy:
                rospy.ServiceProxy = self._original_service_proxy
        
        self._active = False
    
    def reset(self):
        """Reset all mocks to default state."""
        self.action_clients.clear()
        self.service_proxies.clear()
        self.publishers.clear()
        self.subscribers.clear()
        self.configs = dict(self.DEFAULT_CONFIGS)
    
    def get_call_stats(self) -> Dict[str, int]:
        """Get statistics on mock usage."""
        stats = {}
        for name, proxy in self.service_proxies.items():
            stats[name] = len(proxy.get_call_history())
        return stats


# Predefined mock scenarios for common test cases
class MockScenarios:
    """Predefined mock configurations for common test scenarios."""
    
    @staticmethod
    def all_success(delay: float = 0.5) -> MockServices:
        """All services succeed."""
        mocks = MockServices()
        for name in mocks.configs:
            mocks.configure_success(name, delay)
        return mocks
    
    @staticmethod
    def navigation_failure() -> MockServices:
        """Navigation fails, everything else succeeds."""
        mocks = MockServices()
        mocks.configure_failure('move_base_server', delay=0.5)
        mocks.configure_failure('/move_base', delay=0.5)
        return mocks
    
    @staticmethod
    def manipulation_failure() -> MockServices:
        """Manipulation fails, everything else succeeds."""
        mocks = MockServices()
        mocks.configure_failure('pickup_action_server')
        mocks.configure_failure('place_action_server')
        return mocks
    
    @staticmethod
    def perception_empty() -> MockServices:
        """Perception returns no objects."""
        mocks = MockServices()
        mocks.configure('perceive_plane_server', 
                       result=MockResult.SUCCESS,
                       response_data={'objects': []})
        return mocks
    
    @staticmethod
    def slow_robot(delay: float = 5.0) -> MockServices:
        """All operations are slow (for timeout testing)."""
        mocks = MockServices()
        for name in mocks.configs:
            mocks.configs[name].delay = delay
        return mocks
