#!/usr/bin/env python3
"""
State Tester CLI - Test individual states from command line

Supports both offline mock testing and live on-robot testing.

Usage:
    # List available states
    rosrun hsr_task_sm state_tester_cli.py --list
    
    # Get info about a state
    rosrun hsr_task_sm state_tester_cli.py --info NavigateTo
    
    # Test a state with mocks (offline)
    rosrun hsr_task_sm state_tester_cli.py NavigateTo --mock --params '{"destination": "kitchen"}'
    
    # Test a state on real robot (LIVE)
    rosrun hsr_task_sm state_tester_cli.py NavigateTo --live --params '{"destination": "living_room_table"}'
    
    # Interactive mode
    rosrun hsr_task_sm state_tester_cli.py --interactive
    
    # Run test suite from file
    rosrun hsr_task_sm state_tester_cli.py --suite test_suite.yaml
"""

import sys
import json
import yaml
import argparse
import readline  # For arrow key support in interactive mode

import rospy

from hsr_task_sm.state_tester.core import StateTester, TestMode, STATE_REGISTRY
from hsr_task_sm.state_tester.mock_services import MockServices, MockScenarios
from hsr_task_sm.state_tester.validator import StateValidator
from hsr_task_sm.state_tester.reporter import TestReporter, Colors


def parse_args():
    parser = argparse.ArgumentParser(
        description='Test individual SMACH states',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List all states
  %(prog)s --list

  # Get state info
  %(prog)s --info NavigateTo

  # Mock test (offline)
  %(prog)s NavigateTo --mock --params '{"destination": "kitchen"}'

  # Live test on robot
  %(prog)s NavigateTo --live --params '{"destination": "living_room_table"}'

  # Validate only (no execution)
  %(prog)s NavigateTo --dry-run --params '{"destination": "kitchen"}'

  # Interactive testing mode
  %(prog)s --interactive

  # Run test suite
  %(prog)s --suite tests/navigation_tests.yaml
        """
    )
    
    # Main arguments
    parser.add_argument('state', nargs='?', help='State type to test')
    
    # Mode selection
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--mock', action='store_true', 
                           help='Use mock services (offline testing)')
    mode_group.add_argument('--live', action='store_true', 
                           help='Use real ROS services (on-robot testing)')
    mode_group.add_argument('--dry-run', action='store_true',
                           help='Validate only, no execution')
    
    # State configuration
    parser.add_argument('--params', '-p', type=str, default='{}',
                       help='State parameters as JSON')
    parser.add_argument('--userdata', '-u', type=str, default='{}',
                       help='Userdata as JSON')
    parser.add_argument('--expected', '-e', type=str,
                       help='Expected outcome')
    parser.add_argument('--timeout', '-t', type=float, default=30.0,
                       help='Execution timeout (seconds)')
    
    # Special modes
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all available states')
    parser.add_argument('--info', '-i', type=str, metavar='STATE',
                       help='Show detailed info about a state')
    parser.add_argument('--interactive', action='store_true',
                       help='Interactive testing mode')
    parser.add_argument('--suite', '-s', type=str, metavar='FILE',
                       help='Run test suite from YAML file')
    
    # Mock configuration
    parser.add_argument('--scenario', type=str,
                       choices=['all_success', 'nav_fail', 'manip_fail', 'slow'],
                       help='Use predefined mock scenario')
    
    # Output options
    parser.add_argument('--json', action='store_true',
                       help='Output results as JSON')
    parser.add_argument('--output', '-o', type=str,
                       help='Save results to file')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Show detailed output')
    parser.add_argument('--no-color', action='store_true',
                       help='Disable colored output')
    parser.add_argument('--no-ros', action='store_true',
                       help='Skip ROS initialization (for pure offline testing)')
    
    return parser.parse_args()


def safe_ros_init(node_name='state_tester', timeout=3.0):
    """Try to init ROS, return True if successful."""
    import os
    import signal
    
    # Check if ROS_MASTER_URI is set
    if 'ROS_MASTER_URI' not in os.environ:
        print("Warning: ROS_MASTER_URI not set, skipping ROS init")
        return False
    
    def timeout_handler(signum, frame):
        raise TimeoutError("ROS master not responding")
    
    try:
        # Set timeout for init
        old_handler = signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(int(timeout))
        
        rospy.init_node(node_name, anonymous=True)
        
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)
        return True
    except TimeoutError:
        print("Warning: ROS master not responding, running without ROS")
        signal.alarm(0)
        return False
    except rospy.ROSInitException as e:
        print(f"Warning: ROS init failed: {e}")
        return False
    except Exception as e:
        print(f"Warning: Could not init ROS: {e}")
        return False


def c(color, text, use_colors=True):
    """Apply color if enabled."""
    if use_colors:
        return f"{color}{text}{Colors.RESET}"
    return text


def list_states(use_colors=True):
    """Print list of available states grouped by category."""
    print(c(Colors.BOLD, "\nAvailable State Types:", use_colors))
    print("=" * 50)
    
    # Group states by category
    categories = {
        'Navigation': ['NavigateTo', 'NavigateToPose', 'GoToGoal', 'ClearCostmap'],
        'Perception': ['PerceiveTable', 'DetectPerson', 'GetPersonFeatures', 
                      'RecognizePerson', 'CheckDoorOpen'],
        'Manipulation': ['PickObject', 'PlaceObject', 'HandoverToHuman', 
                        'ReceiveFromHuman', 'OpenDoor', 'CloseDoor', 
                        'OpenDrawer', 'CloseDrawer'],
        'HRI - Speech': ['Speak', 'ListenForCommand', 'ParseCommand'],
        'HRI - Ollama': ['ListenWithWhisper', 'GetVoicebotResponse', 'SpeakResponse',
                        'ControlMicrophone', 'ConversationLoop', 'SaveGuestInfo'],
        'Gaze': ['LookAt', 'LookAtPerson', 'LookAtObject', 'ResetGaze'],
        'Following': ['FollowPerson', 'StartFollowing', 'StopFollowing'],
        'Utility': ['UserDataTransfer', 'WaitState', 'CheckCondition',
                   'IncrementCounter', 'CheckRetries'],
    }
    
    for category, states in categories.items():
        print(f"\n{c(Colors.CYAN, category, use_colors)}:")
        for state in states:
            if state in STATE_REGISTRY:
                print(f"  - {state}")
    
    # Show uncategorized
    all_categorized = set(s for states in categories.values() for s in states)
    uncategorized = set(STATE_REGISTRY.keys()) - all_categorized
    if uncategorized:
        print(f"\n{c(Colors.CYAN, 'Other', use_colors)}:")
        for state in sorted(uncategorized):
            print(f"  - {state}")


def show_state_info(state_name, use_colors=True):
    """Show detailed info about a state."""
    if state_name not in STATE_REGISTRY:
        print(c(Colors.RED, f"Unknown state: {state_name}", use_colors))
        return
    
    tester = StateTester(mode=TestMode.DRY_RUN, init_ros=False)
    info = tester.get_state_info(state_name)
    
    print(f"\n{c(Colors.BOLD, info['name'], use_colors)}")
    print("=" * 50)
    print(f"Class: {info['class']}")
    print(f"Module: {info['module']}")
    print(f"\n{c(Colors.CYAN, 'Outcomes:', use_colors)} {', '.join(info['outcomes'])}")
    
    print(f"\n{c(Colors.CYAN, 'Parameters:', use_colors)}")
    for name, spec in info['parameters'].items():
        default = spec['default']
        if default == 'required':
            default_str = c(Colors.RED, '(required)', use_colors)
        else:
            default_str = f"= {default}"
        print(f"  {name}: {spec['annotation']} {default_str}")
    
    print(f"\n{c(Colors.CYAN, 'Description:', use_colors)}")
    doc = info['docstring'] or "No documentation available"
    print(f"  {doc[:200]}..." if len(doc) > 200 else f"  {doc}")


def run_interactive_mode(use_colors=True):
    """Interactive state testing mode."""
    print(c(Colors.BOLD, "\n=== Interactive State Tester ===", use_colors))
    print("Type 'help' for commands, 'quit' to exit\n")
    
    # Initialize
    mode = TestMode.MOCK
    mocks = MockScenarios.all_success()
    tester = StateTester(mode=mode, mock_services=mocks)
    reporter = TestReporter(use_colors=use_colors)
    userdata = {}
    
    commands = {
        'help': 'Show this help',
        'list': 'List available states',
        'info <state>': 'Show state info',
        'mode [mock|live]': 'Get/set test mode',
        'userdata': 'Show current userdata',
        'set <key> <value>': 'Set userdata value',
        'clear': 'Clear userdata',
        'test <state> [params_json]': 'Test a state',
        'validate <state> [params_json]': 'Validate state config',
        'summary': 'Show test summary',
        'quit': 'Exit interactive mode'
    }
    
    while True:
        try:
            line = input(c(Colors.GREEN, "tester> ", use_colors)).strip()
        except (EOFError, KeyboardInterrupt):
            print("\nGoodbye!")
            break
        
        if not line:
            continue
        
        parts = line.split(maxsplit=2)
        cmd = parts[0].lower()
        args = parts[1:] if len(parts) > 1 else []
        
        if cmd == 'quit' or cmd == 'q':
            break
        
        elif cmd == 'help' or cmd == 'h':
            print("\nCommands:")
            for k, v in commands.items():
                print(f"  {k:25} - {v}")
            print()
        
        elif cmd == 'list' or cmd == 'ls':
            list_states(use_colors)
        
        elif cmd == 'info':
            if args:
                show_state_info(args[0], use_colors)
            else:
                print("Usage: info <state_name>")
        
        elif cmd == 'mode':
            if args:
                if args[0] == 'mock':
                    mode = TestMode.MOCK
                    tester = StateTester(mode=mode, mock_services=mocks)
                    print("Switched to MOCK mode (offline)")
                elif args[0] == 'live':
                    mode = TestMode.LIVE
                    tester = StateTester(mode=mode)
                    print(c(Colors.YELLOW, "⚠ Switched to LIVE mode (on-robot)", use_colors))
                else:
                    print("Mode must be 'mock' or 'live'")
            else:
                print(f"Current mode: {mode.value}")
        
        elif cmd == 'userdata' or cmd == 'ud':
            print(f"Userdata: {json.dumps(userdata, indent=2)}")
        
        elif cmd == 'set':
            if len(args) >= 2:
                key = args[0]
                try:
                    value = json.loads(args[1])
                except:
                    value = args[1]
                userdata[key] = value
                print(f"Set {key} = {value}")
            else:
                print("Usage: set <key> <value>")
        
        elif cmd == 'clear':
            userdata = {}
            tester.clear_results()
            print("Cleared userdata and results")
        
        elif cmd == 'test' or cmd == 't':
            if not args:
                print("Usage: test <state_name> [params_json]")
                continue
            
            state_name = args[0]
            params = json.loads(args[1]) if len(args) > 1 else {}
            
            print(f"Testing {state_name}...")
            result = tester.test_state(state_name, params, userdata)
            reporter.add_result(result)
            reporter.print_result(result)
            
            # Update userdata from output
            userdata.update(result.output_userdata)
        
        elif cmd == 'validate' or cmd == 'v':
            if not args:
                print("Usage: validate <state_name> [params_json]")
                continue
            
            state_name = args[0]
            params = json.loads(args[1]) if len(args) > 1 else {}
            
            validator = StateValidator()
            result = validator.validate(state_name, params, userdata)
            print(result)
        
        elif cmd == 'summary':
            reporter.print_summary()
        
        else:
            print(f"Unknown command: {cmd}. Type 'help' for available commands.")


def run_test_suite(filepath, mode, use_colors=True):
    """Run a test suite from YAML file."""
    print(f"Loading test suite: {filepath}")
    
    with open(filepath, 'r') as f:
        suite = yaml.safe_load(f)
    
    reporter = TestReporter(use_colors=use_colors)
    mocks = MockScenarios.all_success() if mode == TestMode.MOCK else None
    tester = StateTester(mode=mode, mock_services=mocks)
    
    tests = suite.get('tests', [])
    userdata = suite.get('userdata', {})
    
    print(f"Running {len(tests)} tests...\n")
    
    for i, test in enumerate(tests, 1):
        state_name = test.get('state')
        params = test.get('params', {})
        expected = test.get('expected')
        test_userdata = {**userdata, **test.get('userdata', {})}
        
        print(f"[{i}/{len(tests)}] Testing {state_name}...")
        
        result = tester.test_state(
            state_name, params, test_userdata, expected
        )
        reporter.add_result(result)
        
        # Update running userdata
        userdata.update(result.output_userdata)
    
    reporter.print_summary()
    return reporter


def main():
    args = parse_args()
    use_colors = not args.no_color
    skip_ros = args.no_ros
    
    # Handle special modes first (no ROS needed)
    if args.list:
        list_states(use_colors)
        return 0
    
    if args.info:
        show_state_info(args.info, use_colors)
        return 0
    
    if args.interactive:
        if not skip_ros:
            safe_ros_init('state_tester_interactive')
        run_interactive_mode(use_colors)
        return 0
    
    if args.suite:
        if not skip_ros:
            safe_ros_init('state_tester_suite')
        mode = TestMode.LIVE if args.live else TestMode.MOCK
        reporter = run_test_suite(args.suite, mode, use_colors)
        
        if args.output:
            if args.output.endswith('.html'):
                reporter.save_html(args.output)
            else:
                reporter.save_json(args.output)
        return 0
    
    # Single state test
    if not args.state:
        print("Error: State name required. Use --list to see available states.")
        return 1
    
    # Initialize ROS (skip in no-ros mode or if mock mode without requiring ROS)
    if not skip_ros and not args.dry_run:
        if args.live:
            # Live mode requires ROS
            if not safe_ros_init('state_tester'):
                print("Error: Live mode requires ROS master to be running")
                return 1
        else:
            # Mock mode - try ROS but continue without it
            safe_ros_init('state_tester')
    
    # Determine mode
    if args.live:
        mode = TestMode.LIVE
        print(c(Colors.YELLOW, "⚠ LIVE MODE - Testing on real robot", use_colors))
    elif args.dry_run:
        mode = TestMode.DRY_RUN
    else:
        mode = TestMode.MOCK
    
    # Setup mocks if needed
    mocks = None
    if mode == TestMode.MOCK:
        if args.scenario == 'nav_fail':
            mocks = MockScenarios.navigation_failure()
        elif args.scenario == 'manip_fail':
            mocks = MockScenarios.manipulation_failure()
        elif args.scenario == 'slow':
            mocks = MockScenarios.slow_robot()
        else:
            mocks = MockScenarios.all_success()
    
    # Parse params and userdata
    try:
        params = json.loads(args.params)
    except json.JSONDecodeError as e:
        print(f"Error parsing params JSON: {e}")
        return 1
    
    try:
        userdata = json.loads(args.userdata)
    except json.JSONDecodeError as e:
        print(f"Error parsing userdata JSON: {e}")
        return 1
    
    # Run test
    tester = StateTester(mode=mode, mock_services=mocks, init_ros=False)
    reporter = TestReporter(use_colors=use_colors)
    
    print(f"\nTesting: {args.state}")
    print(f"Mode: {mode.value}")
    print(f"Params: {params}")
    if userdata:
        print(f"Userdata: {userdata}")
    print()
    
    result = tester.test_state(
        args.state, params, userdata, args.expected, args.timeout
    )
    reporter.add_result(result)
    
    # Output
    if args.json:
        print(json.dumps(result.to_dict(), indent=2))
    else:
        reporter.print_result(result)
    
    if args.output:
        if args.output.endswith('.html'):
            reporter.save_html(args.output)
        else:
            reporter.save_json(args.output)
    
    return 0 if result.success else 1


if __name__ == '__main__':
    sys.exit(main())
