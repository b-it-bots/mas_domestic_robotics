#!/usr/bin/env python3
"""
Simple Action State Base

A lightweight replacement for mas_execution's ActionSMBase that:
- Does NOT use knowledge base (MongoDB, ROSPlan KB)
- Does NOT use ontology (OWL files, concepts)
- Uses only SMACH userdata for data transfer
- Works reliably without external dependencies

Usage:
    class MyAction(SimpleActionState):
        def __init__(self, action_server, goal_type, ...):
            super().__init__(
                action_name='my_action',
                action_server=action_server,
                action_type=MyActionType,
                goal_type=goal_type
            )
        
        def get_goal(self, userdata):
            goal = self.goal_type()
            goal.target = userdata.target
            return goal
        
        def process_result(self, result, userdata):
            userdata.output = result.data
            return 'succeeded'
"""

import rospy
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus


class SimpleActionState(smach.State):
    """
    Simple action client state without knowledge base dependencies.
    
    Uses userdata for all data transfer between states.
    Subclass and implement get_goal() and process_result().
    """
    
    def __init__(self, 
                 action_name,
                 action_server,
                 action_type,
                 goal_type,
                 timeout=60.0,
                 max_retries=3,
                 input_keys=None,
                 output_keys=None):
        """
        Args:
            action_name: Human-readable name for logging
            action_server: ROS action server name (e.g., '/move_base_server')
            action_type: Action type class (e.g., MoveBaseAction)
            goal_type: Goal type class (e.g., MoveBaseGoal)
            timeout: Action timeout in seconds
            max_retries: Number of retries on failure
            input_keys: List of userdata input keys
            output_keys: List of userdata output keys
        """
        outcomes = ['succeeded', 'failed', 'failed_after_retrying', 'preempted']
        
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys or [],
            output_keys=output_keys or []
        )
        
        self.action_name = action_name
        self.action_server = action_server
        self.action_type = action_type
        self.goal_type = goal_type
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_count = 0
        
        self.client = None
    
    def _ensure_client(self):
        """Create action client if not exists."""
        if self.client is None:
            self.client = actionlib.SimpleActionClient(
                self.action_server, 
                self.action_type
            )
    
    def get_goal(self, userdata):
        """
        Build action goal from userdata.
        Override in subclass.
        
        Returns:
            Action goal object
        """
        return self.goal_type()
    
    def process_result(self, result, userdata):
        """
        Process action result and update userdata.
        Override in subclass.
        
        Args:
            result: Action result
            userdata: SMACH userdata
            
        Returns:
            'succeeded' or 'failed'
        """
        return 'succeeded'
    
    def execute(self, userdata):
        """Execute the action with retry logic."""
        self._ensure_client()
        
        # Wait for server
        rospy.loginfo(f'[{self.action_name}] Waiting for action server: {self.action_server}')
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr(f'[{self.action_name}] Action server not available: {self.action_server}')
            return 'failed'
        
        # Build goal from userdata
        try:
            goal = self.get_goal(userdata)
        except Exception as e:
            rospy.logerr(f'[{self.action_name}] Failed to build goal: {e}')
            return 'failed'
        
        rospy.loginfo(f'[{self.action_name}] Sending goal...')
        self.client.send_goal(goal)
        
        # Wait for result
        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        
        if not finished:
            rospy.logwarn(f'[{self.action_name}] Action timed out')
            self.client.cancel_goal()
            return self._handle_failure(userdata)
        
        state = self.client.get_state()
        
        if state == GoalStatus.PREEMPTED:
            rospy.logwarn(f'[{self.action_name}] Action preempted')
            return 'preempted'
        
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn(f'[{self.action_name}] Action failed with state: {state}')
            return self._handle_failure(userdata)
        
        # Process result
        result = self.client.get_result()
        try:
            outcome = self.process_result(result, userdata)
            self.retry_count = 0  # Reset on success
            rospy.loginfo(f'[{self.action_name}] Completed with outcome: {outcome}')
            return outcome
        except Exception as e:
            rospy.logerr(f'[{self.action_name}] Failed to process result: {e}')
            return 'failed'
    
    def _handle_failure(self, userdata):
        """Handle failure with retry logic."""
        self.retry_count += 1
        
        if self.retry_count >= self.max_retries:
            rospy.logerr(f'[{self.action_name}] Failed after {self.retry_count} retries')
            self.retry_count = 0
            return 'failed_after_retrying'
        
        rospy.logwarn(f'[{self.action_name}] Retry {self.retry_count}/{self.max_retries}')
        return 'failed'


class SimpleServiceState(smach.State):
    """
    Simple service client state without knowledge base dependencies.
    """
    
    def __init__(self,
                 service_name,
                 service_type,
                 request_type=None,
                 timeout=30.0,
                 input_keys=None,
                 output_keys=None):
        """
        Args:
            service_name: ROS service name
            service_type: Service type class
            request_type: Request type (optional, uses service type if None)
            timeout: Service call timeout
            input_keys: Userdata input keys
            output_keys: Userdata output keys
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=input_keys or [],
            output_keys=output_keys or []
        )
        
        self.service_name = service_name
        self.service_type = service_type
        self.request_type = request_type
        self.timeout = timeout
    
    def get_request(self, userdata):
        """Build service request from userdata. Override in subclass."""
        if self.request_type:
            return self.request_type()
        return None
    
    def process_response(self, response, userdata):
        """Process service response. Override in subclass."""
        return 'succeeded'
    
    def execute(self, userdata):
        try:
            rospy.wait_for_service(self.service_name, timeout=self.timeout)
            proxy = rospy.ServiceProxy(self.service_name, self.service_type)
            
            request = self.get_request(userdata)
            
            if request:
                response = proxy(request)
            else:
                response = proxy()
            
            return self.process_response(response, userdata)
            
        except rospy.ServiceException as e:
            rospy.logerr(f'[{self.service_name}] Service call failed: {e}')
            return 'failed'
        except rospy.ROSException as e:
            rospy.logerr(f'[{self.service_name}] Service not available: {e}')
            return 'failed'


class SimpleTopicState(smach.State):
    """
    Simple state for publishing/subscribing to topics.
    """
    
    def __init__(self,
                 topic_name,
                 msg_type,
                 mode='publish',  # 'publish' or 'subscribe'
                 timeout=10.0,
                 input_keys=None,
                 output_keys=None):
        
        outcomes = ['succeeded', 'failed'] if mode == 'subscribe' else ['succeeded']
        
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys or [],
            output_keys=output_keys or []
        )
        
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.mode = mode
        self.timeout = timeout
        self.pub = None
    
    def get_message(self, userdata):
        """Build message to publish. Override in subclass."""
        return self.msg_type()
    
    def process_message(self, msg, userdata):
        """Process received message. Override in subclass."""
        return 'succeeded'
    
    def execute(self, userdata):
        if self.mode == 'publish':
            return self._publish(userdata)
        else:
            return self._subscribe(userdata)
    
    def _publish(self, userdata):
        if not self.pub:
            self.pub = rospy.Publisher(self.topic_name, self.msg_type, queue_size=10)
            rospy.sleep(0.2)
        
        msg = self.get_message(userdata)
        self.pub.publish(msg)
        return 'succeeded'
    
    def _subscribe(self, userdata):
        try:
            msg = rospy.wait_for_message(
                self.topic_name, 
                self.msg_type, 
                timeout=self.timeout
            )
            return self.process_message(msg, userdata)
        except rospy.ROSException:
            rospy.logerr(f'[{self.topic_name}] Timeout waiting for message')
            return 'failed'


class UserDataTransfer(smach.State):
    """
    Utility state for transferring/transforming userdata.
    Useful for data manipulation between states without KB.
    """
    
    def __init__(self, transfers=None, transforms=None):
        """
        Args:
            transfers: Dict mapping input_key -> output_key for direct copy
            transforms: Dict mapping output_key -> function(userdata) for computed values
        """
        self.transfers = transfers or {}
        self.transforms = transforms or {}
        
        input_keys = list(self.transfers.keys())
        output_keys = list(self.transfers.values()) + list(self.transforms.keys())
        
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=input_keys,
            output_keys=output_keys
        )
    
    def execute(self, userdata):
        # Direct transfers
        for src, dst in self.transfers.items():
            setattr(userdata, dst, getattr(userdata, src))
        
        # Computed transforms
        for key, func in self.transforms.items():
            setattr(userdata, key, func(userdata))
        
        return 'succeeded'


class WaitState(smach.State):
    """Simple wait state."""
    
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.duration = duration
    
    def execute(self, userdata):
        import time
        try:
            rospy.sleep(self.duration)
        except:
            # Fallback if ROS not running
            time.sleep(self.duration)
        return 'succeeded'


class CheckCondition(smach.State):
    """
    Check a condition based on userdata.
    """
    
    def __init__(self, condition_fn, input_keys=None):
        """
        Args:
            condition_fn: Function(userdata) -> bool
            input_keys: Userdata keys to read
        """
        smach.State.__init__(
            self,
            outcomes=['true', 'false'],
            input_keys=input_keys or []
        )
        self.condition_fn = condition_fn
    
    def execute(self, userdata):
        try:
            result = self.condition_fn(userdata)
            return 'true' if result else 'false'
        except Exception as e:
            rospy.logerr(f'[CheckCondition] Error: {e}')
            return 'false'


class IncrementCounter(smach.State):
    """Increment a counter in userdata."""
    
    def __init__(self, counter_key='counter', max_value=None):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'max_reached'],
            input_keys=[counter_key],
            output_keys=[counter_key]
        )
        self.counter_key = counter_key
        self.max_value = max_value
    
    def execute(self, userdata):
        current = getattr(userdata, self.counter_key, 0)
        new_value = current + 1
        setattr(userdata, self.counter_key, new_value)
        
        if self.max_value and new_value >= self.max_value:
            return 'max_reached'
        return 'succeeded'


class CheckRetries(smach.State):
    """Check if max retries reached."""
    
    def __init__(self, counter_key='retry_count', max_retries=3):
        smach.State.__init__(
            self,
            outcomes=['retry', 'max_reached'],
            input_keys=[counter_key],
            output_keys=[counter_key]
        )
        self.counter_key = counter_key
        self.max_retries = max_retries
    
    def execute(self, userdata):
        current = getattr(userdata, self.counter_key, 0)
        new_value = current + 1
        setattr(userdata, self.counter_key, new_value)
        
        if new_value >= self.max_retries:
            return 'max_reached'
        return 'retry'
