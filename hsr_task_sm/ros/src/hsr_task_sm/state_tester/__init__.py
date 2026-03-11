#!/usr/bin/env python3
"""
State Testing Framework for hsr_task_sm

Provides tools for testing and verifying individual states:
- StateTester: Core testing class with mock support
- MockServices: Simulated ROS services/action servers
- StateValidator: Validate state configs before running
- TestReporter: Generate test reports

Usage:
    from hsr_task_sm.state_tester import StateTester, MockServices
    
    # Create tester with mocks
    tester = StateTester(use_mocks=True)
    
    # Test a state
    result = tester.test_state('NavigateTo', 
                               params={'destination': 'living_room_table'},
                               userdata={'robot_pose': [0, 0, 0]})
    
    # Check result
    if result.success:
        print(f"State returned: {result.outcome}")
"""

from hsr_task_sm.state_tester.core import StateTester, StateTestResult
from hsr_task_sm.state_tester.mock_services import MockServices, MockActionClient, MockScenarios
from hsr_task_sm.state_tester.validator import StateValidator
from hsr_task_sm.state_tester.reporter import TestReporter

__all__ = [
    'StateTester',
    'StateTestResult', 
    'MockServices',
    'MockActionClient',
    'MockScenarios',
    'StateValidator',
    'TestReporter'
]
