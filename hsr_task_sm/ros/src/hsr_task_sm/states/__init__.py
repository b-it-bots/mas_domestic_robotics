# Simple action base classes (NO knowledge base dependencies)
from hsr_task_sm.simple_action_base import (
    SimpleActionState,
    SimpleServiceState,
    SimpleTopicState,
    UserDataTransfer,
    WaitState,
    CheckCondition,
    IncrementCounter,
    CheckRetries
)

# Import states with graceful handling for missing dependencies
import warnings

_IMPORT_ERRORS = []

def _safe_import(module_path, names):
    """Safely import from a module, returning dict of name->class."""
    result = {}
    try:
        module = __import__(module_path, fromlist=names)
        for name in names:
            if hasattr(module, name):
                result[name] = getattr(module, name)
    except ImportError as e:
        _IMPORT_ERRORS.append(f"{module_path}: {e}")
    except Exception as e:
        _IMPORT_ERRORS.append(f"{module_path}: {e}")
    return result

# Original states
_states = {}
_states.update(_safe_import('hsr_task_sm.states.clear_costmap', ['ClearCostmap']))
_states.update(_safe_import('hsr_task_sm.states.perceive_table', ['PerceiveTable']))
_states.update(_safe_import('hsr_task_sm.states.pick_object', ['PickObject']))
_states.update(_safe_import('hsr_task_sm.states.navigate_to', ['NavigateTo', 'NavigateToPose']))
_states.update(_safe_import('hsr_task_sm.states.go_to_goal', ['GoToGoal']))
_states.update(_safe_import('hsr_task_sm.states.check_door_open', ['CheckDoorOpen']))

# New utility states for RoboCup@Home 2026 challenges
_states.update(_safe_import('hsr_task_sm.states.speak', ['Speak']))
_states.update(_safe_import('hsr_task_sm.states.listen_for_command', ['ListenForCommand', 'ParseCommand']))
_states.update(_safe_import('hsr_task_sm.states.detect_person', ['DetectPerson', 'GetPersonFeatures', 'RecognizePerson']))
_states.update(_safe_import('hsr_task_sm.states.place_object', ['PlaceObject']))
_states.update(_safe_import('hsr_task_sm.states.follow_person', ['FollowPerson', 'StartFollowing', 'StopFollowing']))
_states.update(_safe_import('hsr_task_sm.states.handover', ['HandoverToHuman', 'ReceiveFromHuman']))
_states.update(_safe_import('hsr_task_sm.states.gaze_control', ['LookAt', 'LookAtPerson', 'LookAtObject', 'ResetGaze']))
_states.update(_safe_import('hsr_task_sm.states.furniture_manipulation', ['OpenDoor', 'CloseDoor', 'OpenDrawer', 'CloseDrawer']))

# Ollama-based HRI states (for Whisper STT + Ollama LLM on slave laptop)
_states.update(_safe_import('hsr_task_sm.states.ollama_hri_states', [
    'ListenWithWhisper', 'GetVoicebotResponse', 'SpeakResponse',
    'ControlMicrophone', 'ConversationLoop', 'SaveGuestInfo'
]))

# Export all successfully imported states
globals().update(_states)

# For debugging import issues
def get_import_errors():
    """Return list of import errors that occurred."""
    return _IMPORT_ERRORS.copy()

def get_available_states():
    """Return dict of all successfully imported state classes."""
    return _states.copy()
