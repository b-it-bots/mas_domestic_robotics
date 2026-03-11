#!/usr/bin/env python3
"""
State Machine Editor GUI

A simple graphical interface for creating and editing YAML state machine
configurations for RoboCup@Home challenges.

Usage:
    rosrun hsr_task_sm sm_editor_gui.py
    rosrun hsr_task_sm sm_editor_gui.py /path/to/config.yaml

Features:
- Visual state list with add/edit/delete
- Parameter editing with type hints
- Transition editor
- YAML preview and validation
- Save/Load configurations
"""

import os
import sys
import yaml
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
from typing import Dict, List, Any, Optional

# Available state types with their parameters and outcomes
# SORTED ALPHABETICALLY for easier finding in dropdown
STATE_TYPES = {
    # Utility - Common
    'CheckCondition': {
        'params': {'condition_key': 'str'},
        'outcomes': ['true', 'false'],
        'description': 'Check boolean userdata'
    },
    'CheckRetries': {
        'params': {'max_retries': 'int', 'counter_key': 'str'},
        'outcomes': ['retry', 'max_reached'],
        'description': 'Check retry count'
    },
    'ClearCostmap': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Clear navigation costmaps'
    },
    'CloseDoor': {
        'params': {'door_type': 'str'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Close a door'
    },
    'CloseDrawer': {
        'params': {'drawer_name': 'str'},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Close a drawer'
    },
    'ConversationLoop': {
        'params': {'max_iterations': 'int'},
        'outcomes': ['guest_complete', 'max_iterations', 'failed'],
        'description': 'Ollama conversation until guest info complete'
    },
    'ControlMicrophone': {
        'params': {'enable': 'bool'},
        'outcomes': ['succeeded'],
        'description': 'Enable/disable slave laptop mic'
    },
    'DetectPerson': {
        'params': {'timeout': 'float'},
        'outcomes': ['succeeded', 'no_person', 'failed'],
        'description': 'Detect a person in view'
    },
    'FollowPerson': {
        'params': {'timeout': 'float', 'distance': 'float'},
        'outcomes': ['succeeded', 'lost_person', 'failed'],
        'description': 'Follow a detected person'
    },
    'GetPersonFeatures': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Extract person features (clothes, etc)'
    },
    'GetVoicebotResponse': {
        'params': {},
        'outcomes': ['succeeded', 'guest_complete', 'failed'],
        'description': 'Send text to Ollama LLM'
    },
    'GoToGoal': {
        'params': {'x': 'float', 'y': 'float', 'theta': 'float'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Navigate to a specific pose'
    },
    'HandoverToHuman': {
        'params': {'timeout': 'float'},
        'outcomes': ['succeeded', 'failed', 'timeout'],
        'description': 'Hand over object to person'
    },
    'IncrementCounter': {
        'params': {'counter_key': 'str'},
        'outcomes': ['succeeded', 'max_reached'],
        'description': 'Increment a counter'
    },
    'ListenForCommand': {
        'params': {'timeout': 'float'},
        'outcomes': ['succeeded', 'timeout', 'failed'],
        'description': 'Listen for speech command'
    },
    'ListenWithWhisper': {
        'params': {'timeout': 'float'},
        'outcomes': ['succeeded', 'timeout', 'failed'],
        'description': 'Listen via Whisper STT (slave laptop)'
    },
    'Log': {
        'params': {'message': 'str', 'level': 'str'},
        'outcomes': ['succeeded'],
        'description': 'Log a message'
    },
    'LookAt': {
        'params': {'target': 'str'},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Look at a target'
    },
    'LookAtObject': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Look at detected object'
    },
    'LookAtPerson': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Look at detected person'
    },
    'NavigateTo': {
        'params': {'destination': 'str'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Navigate to a named location (e.g., kitchen, entrance, dining_table)'
    },
    'NavigateToPose': {
        'params': {'x': 'float', 'y': 'float', 'theta': 'float'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Navigate to specific coordinates (x, y, theta)'
    },
    'OpenDoor': {
        'params': {'door_type': 'str'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Open a door'
    },
    'OpenDrawer': {
        'params': {'drawer_name': 'str'},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Open a drawer'
    },
    'ParseCommand': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Parse spoken command'
    },
    'PerceiveTable': {
        'params': {'plane_frame_prefix': 'str'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Perceive objects on a surface'
    },
    'PickObject': {
        'params': {'object_name': 'str'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Pick up an object'
    },
    'PlaceObject': {
        'params': {'surface': 'str'},
        'outcomes': ['succeeded', 'failed', 'failed_after_retrying'],
        'description': 'Place the held object'
    },
    'ReceiveFromHuman': {
        'params': {'timeout': 'float'},
        'outcomes': ['succeeded', 'failed', 'timeout'],
        'description': 'Receive object from person'
    },
    'RecognizePerson': {
        'params': {},
        'outcomes': ['recognized', 'unknown', 'failed'],
        'description': 'Recognize a known person'
    },
    'ResetGaze': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Reset head to neutral'
    },
    'SaveGuestInfo': {
        'params': {'output_dir': 'str'},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Save guest info to JSON'
    },
    'Speak': {
        'params': {'text': 'str'},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Speak text via TTS'
    },
    'SpeakResponse': {
        'params': {'text': 'str'},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Speak LLM response'
    },
    'StartFollowing': {
        'params': {},
        'outcomes': ['succeeded', 'failed'],
        'description': 'Start following person'
    },
    'StopFollowing': {
        'params': {},
        'outcomes': ['succeeded'],
        'description': 'Stop following person'
    },
    'Wait': {
        'params': {'duration': 'float'},
        'outcomes': ['succeeded'],
        'description': 'Wait for duration (seconds)'
    },
}
    },
}


class StateEditor(tk.Toplevel):
    """Dialog for editing a single state."""
    
    def __init__(self, parent, state_data: Dict = None, all_states: List[str] = None):
        super().__init__(parent)
        self.title('Edit State')
        self.geometry('500x600')
        self.result = None
        self.all_states = all_states or []
        
        self.state_data = state_data or {
            'name': 'NEW_STATE',
            'type': 'Speak',
            'params': {},
            'transitions': {}
        }
        
        self._create_widgets()
        self._populate_fields()
        
        self.transient(parent)
        self.grab_set()
    
    def _create_widgets(self):
        # Main frame
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)
        
        # State name
        ttk.Label(main, text='State Name:').grid(row=0, column=0, sticky='w')
        self.name_entry = ttk.Entry(main, width=30)
        self.name_entry.grid(row=0, column=1, sticky='ew', pady=5)
        
        # State type
        ttk.Label(main, text='Type:').grid(row=1, column=0, sticky='w')
        self.type_combo = ttk.Combobox(main, values=list(STATE_TYPES.keys()), state='readonly')
        self.type_combo.grid(row=1, column=1, sticky='ew', pady=5)
        self.type_combo.bind('<<ComboboxSelected>>', self._on_type_changed)
        
        # Description
        self.desc_label = ttk.Label(main, text='', foreground='gray')
        self.desc_label.grid(row=2, column=0, columnspan=2, sticky='w')
        
        # Parameters frame
        param_frame = ttk.LabelFrame(main, text='Parameters', padding=5)
        param_frame.grid(row=3, column=0, columnspan=2, sticky='nsew', pady=10)
        
        self.param_frame = ttk.Frame(param_frame)
        self.param_frame.pack(fill=tk.BOTH, expand=True)
        self.param_widgets = {}
        
        # Transitions frame
        trans_frame = ttk.LabelFrame(main, text='Transitions', padding=5)
        trans_frame.grid(row=4, column=0, columnspan=2, sticky='nsew', pady=10)
        
        self.trans_frame = ttk.Frame(trans_frame)
        self.trans_frame.pack(fill=tk.BOTH, expand=True)
        self.trans_widgets = {}
        
        # Buttons
        btn_frame = ttk.Frame(main)
        btn_frame.grid(row=5, column=0, columnspan=2, pady=10)
        
        ttk.Button(btn_frame, text='OK', command=self._on_ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text='Cancel', command=self.destroy).pack(side=tk.LEFT, padx=5)
        
        main.columnconfigure(1, weight=1)
        main.rowconfigure(3, weight=1)
        main.rowconfigure(4, weight=1)
    
    def _populate_fields(self):
        self.name_entry.insert(0, self.state_data.get('name', ''))
        state_type = self.state_data.get('type', 'Speak')
        self.type_combo.set(state_type)
        self._update_params_ui(state_type)
        self._update_transitions_ui(state_type)
    
    def _on_type_changed(self, event):
        state_type = self.type_combo.get()
        self._update_params_ui(state_type)
        self._update_transitions_ui(state_type)
    
    def _update_params_ui(self, state_type: str):
        # Clear existing widgets
        for widget in self.param_frame.winfo_children():
            widget.destroy()
        self.param_widgets = {}
        
        # Get type info
        type_info = STATE_TYPES.get(state_type, {})
        self.desc_label.config(text=type_info.get('description', ''))
        
        params = type_info.get('params', {})
        existing_params = self.state_data.get('params', {})
        
        for i, (param_name, param_type) in enumerate(params.items()):
            ttk.Label(self.param_frame, text=f'{param_name} ({param_type}):').grid(
                row=i, column=0, sticky='w', pady=2)
            
            entry = ttk.Entry(self.param_frame, width=30)
            entry.grid(row=i, column=1, sticky='ew', pady=2)
            
            # Populate existing value
            if param_name in existing_params:
                entry.insert(0, str(existing_params[param_name]))
            elif f'{param_name}_key' in existing_params:
                entry.insert(0, f'${existing_params[f"{param_name}_key"]}')
            
            self.param_widgets[param_name] = entry
        
        self.param_frame.columnconfigure(1, weight=1)
    
    def _update_transitions_ui(self, state_type: str):
        # Clear existing widgets
        for widget in self.trans_frame.winfo_children():
            widget.destroy()
        self.trans_widgets = {}
        
        # Get outcomes for this type
        type_info = STATE_TYPES.get(state_type, {})
        outcomes = type_info.get('outcomes', ['succeeded', 'failed'])
        existing_trans = self.state_data.get('transitions', {})
        
        # Add terminal states to options
        target_options = ['SUCCEEDED', 'FAILED'] + self.all_states
        
        for i, outcome in enumerate(outcomes):
            ttk.Label(self.trans_frame, text=f'{outcome} →').grid(
                row=i, column=0, sticky='w', pady=2)
            
            combo = ttk.Combobox(self.trans_frame, values=target_options, width=25)
            combo.grid(row=i, column=1, sticky='ew', pady=2)
            
            # Populate existing transition
            if outcome in existing_trans:
                combo.set(existing_trans[outcome])
            
            self.trans_widgets[outcome] = combo
        
        self.trans_frame.columnconfigure(1, weight=1)
    
    def _on_ok(self):
        # Collect data
        name = self.name_entry.get().strip().upper().replace(' ', '_')
        if not name:
            messagebox.showerror('Error', 'State name is required')
            return
        
        state_type = self.type_combo.get()
        
        # Collect parameters
        params = {}
        for param_name, entry in self.param_widgets.items():
            value = entry.get().strip()
            if value:
                if value.startswith('$'):
                    # It's a userdata key reference
                    params[f'{param_name}_key'] = value[1:]
                else:
                    # Try to convert to appropriate type
                    try:
                        if '.' in value:
                            params[param_name] = float(value)
                        elif value.isdigit():
                            params[param_name] = int(value)
                        else:
                            params[param_name] = value
                    except ValueError:
                        params[param_name] = value
        
        # Collect transitions
        transitions = {}
        for outcome, combo in self.trans_widgets.items():
            target = combo.get().strip()
            if target:
                transitions[outcome] = target
        
        self.result = {
            'name': name,
            'type': state_type,
            'params': params,
            'transitions': transitions
        }
        
        self.destroy()


class StateMachineEditorApp:
    """Main application for editing state machine YAML configs."""
    
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title('State Machine Editor - RoboCup@Home 2026')
        self.root.geometry('1200x800')
        
        self.current_file = None
        self.config = {
            'name': 'new_challenge',
            'description': '',
            'userdata': {},
            'states': [],
            'outcomes': ['SUCCEEDED', 'FAILED']
        }
        
        self._create_menu()
        self._create_widgets()
        self._update_ui()
    
    def _create_menu(self):
        menubar = tk.Menu(self.root)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label='New', command=self._new_config, accelerator='Ctrl+N')
        file_menu.add_command(label='Open...', command=self._open_config, accelerator='Ctrl+O')
        file_menu.add_command(label='Save', command=self._save_config, accelerator='Ctrl+S')
        file_menu.add_command(label='Save As...', command=self._save_as_config)
        file_menu.add_separator()
        file_menu.add_command(label='Exit', command=self.root.quit)
        menubar.add_cascade(label='File', menu=file_menu)
        
        # Edit menu
        edit_menu = tk.Menu(menubar, tearoff=0)
        edit_menu.add_command(label='Add State', command=self._add_state, accelerator='Ctrl+A')
        edit_menu.add_command(label='Edit State', command=self._edit_state)
        edit_menu.add_command(label='Delete State', command=self._delete_state)
        edit_menu.add_separator()
        edit_menu.add_command(label='Move Up', command=self._move_state_up)
        edit_menu.add_command(label='Move Down', command=self._move_state_down)
        menubar.add_cascade(label='Edit', menu=edit_menu)
        
        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        tools_menu.add_command(label='Validate', command=self._validate_config, accelerator='F5')
        tools_menu.add_command(label='Preview YAML', command=self._preview_yaml)
        menubar.add_cascade(label='Tools', menu=tools_menu)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label='State Types Reference', command=self._show_state_types)
        help_menu.add_command(label='About', command=self._show_about)
        menubar.add_cascade(label='Help', menu=help_menu)
        
        self.root.config(menu=menubar)
        
        # Keyboard shortcuts
        self.root.bind('<Control-n>', lambda e: self._new_config())
        self.root.bind('<Control-o>', lambda e: self._open_config())
        self.root.bind('<Control-s>', lambda e: self._save_config())
        self.root.bind('<Control-a>', lambda e: self._add_state())
        self.root.bind('<F5>', lambda e: self._validate_config())
    
    def _create_widgets(self):
        # Main paned window
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Config & States
        left_frame = ttk.Frame(paned)
        paned.add(left_frame, weight=1)
        
        # Config section
        config_frame = ttk.LabelFrame(left_frame, text='Configuration', padding=5)
        config_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(config_frame, text='Name:').grid(row=0, column=0, sticky='w')
        self.name_entry = ttk.Entry(config_frame, width=30)
        self.name_entry.grid(row=0, column=1, sticky='ew', pady=2)
        self.name_entry.bind('<KeyRelease>', self._on_config_changed)
        
        ttk.Label(config_frame, text='Description:').grid(row=1, column=0, sticky='w')
        self.desc_entry = ttk.Entry(config_frame, width=30)
        self.desc_entry.grid(row=1, column=1, sticky='ew', pady=2)
        self.desc_entry.bind('<KeyRelease>', self._on_config_changed)
        
        config_frame.columnconfigure(1, weight=1)
        
        # Userdata section
        userdata_frame = ttk.LabelFrame(left_frame, text='Userdata (Initial Variables)', padding=5)
        userdata_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.userdata_text = scrolledtext.ScrolledText(userdata_frame, height=6, width=40)
        self.userdata_text.pack(fill=tk.X, expand=True)
        self.userdata_text.bind('<KeyRelease>', self._on_userdata_changed)
        
        ttk.Label(userdata_frame, text='Format: key: value (YAML)', foreground='gray').pack()
        
        # States section
        states_frame = ttk.LabelFrame(left_frame, text='States', padding=5)
        states_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # States listbox with scrollbar
        list_frame = ttk.Frame(states_frame)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.states_listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, 
                                         selectmode=tk.SINGLE, font=('Consolas', 10))
        self.states_listbox.pack(fill=tk.BOTH, expand=True)
        self.states_listbox.bind('<Double-1>', lambda e: self._edit_state())
        scrollbar.config(command=self.states_listbox.yview)
        
        # State buttons
        btn_frame = ttk.Frame(states_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text='Add', command=self._add_state).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text='Edit', command=self._edit_state).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text='Delete', command=self._delete_state).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text='↑', command=self._move_state_up, width=3).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text='↓', command=self._move_state_down, width=3).pack(side=tk.LEFT, padx=2)
        
        # Right panel - YAML Preview
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=1)
        
        preview_frame = ttk.LabelFrame(right_frame, text='YAML Preview', padding=5)
        preview_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.yaml_preview = scrolledtext.ScrolledText(preview_frame, font=('Consolas', 10))
        self.yaml_preview.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        self.status_var = tk.StringVar(value='Ready')
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)
    
    def _update_ui(self):
        # Update name and description
        self.name_entry.delete(0, tk.END)
        self.name_entry.insert(0, self.config.get('name', ''))
        
        self.desc_entry.delete(0, tk.END)
        self.desc_entry.insert(0, self.config.get('description', ''))
        
        # Update userdata
        self.userdata_text.delete('1.0', tk.END)
        userdata = self.config.get('userdata', {})
        if userdata:
            for key, value in userdata.items():
                self.userdata_text.insert(tk.END, f'{key}: {value}\n')
        
        # Update states list
        self._update_states_list()
        
        # Update YAML preview
        self._update_yaml_preview()
    
    def _update_states_list(self):
        self.states_listbox.delete(0, tk.END)
        for state in self.config.get('states', []):
            name = state.get('name', '?')
            state_type = state.get('type', '?')
            self.states_listbox.insert(tk.END, f'{name} [{state_type}]')
    
    def _update_yaml_preview(self):
        self.yaml_preview.delete('1.0', tk.END)
        yaml_str = yaml.dump(self.config, default_flow_style=False, sort_keys=False, 
                            allow_unicode=True, indent=2)
        self.yaml_preview.insert('1.0', yaml_str)
    
    def _on_config_changed(self, event=None):
        self.config['name'] = self.name_entry.get().strip()
        self.config['description'] = self.desc_entry.get().strip()
        self._update_yaml_preview()
    
    def _on_userdata_changed(self, event=None):
        try:
            text = self.userdata_text.get('1.0', tk.END).strip()
            if text:
                self.config['userdata'] = yaml.safe_load(text) or {}
            else:
                self.config['userdata'] = {}
            self._update_yaml_preview()
        except yaml.YAMLError:
            pass  # Ignore parse errors while typing
    
    def _get_all_state_names(self) -> List[str]:
        return [s.get('name', '') for s in self.config.get('states', [])]
    
    def _add_state(self):
        dialog = StateEditor(self.root, all_states=self._get_all_state_names())
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.config.setdefault('states', []).append(dialog.result)
            self._update_states_list()
            self._update_yaml_preview()
            self.status_var.set(f'Added state: {dialog.result["name"]}')
    
    def _edit_state(self):
        selection = self.states_listbox.curselection()
        if not selection:
            messagebox.showinfo('Info', 'Please select a state to edit')
            return
        
        index = selection[0]
        state_data = self.config['states'][index].copy()
        
        dialog = StateEditor(self.root, state_data=state_data, 
                            all_states=self._get_all_state_names())
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.config['states'][index] = dialog.result
            self._update_states_list()
            self._update_yaml_preview()
            self.status_var.set(f'Updated state: {dialog.result["name"]}')
    
    def _delete_state(self):
        selection = self.states_listbox.curselection()
        if not selection:
            messagebox.showinfo('Info', 'Please select a state to delete')
            return
        
        index = selection[0]
        state_name = self.config['states'][index].get('name', '?')
        
        if messagebox.askyesno('Confirm Delete', f'Delete state "{state_name}"?'):
            del self.config['states'][index]
            self._update_states_list()
            self._update_yaml_preview()
            self.status_var.set(f'Deleted state: {state_name}')
    
    def _move_state_up(self):
        selection = self.states_listbox.curselection()
        if not selection or selection[0] == 0:
            return
        
        index = selection[0]
        states = self.config['states']
        states[index], states[index-1] = states[index-1], states[index]
        
        self._update_states_list()
        self._update_yaml_preview()
        self.states_listbox.selection_set(index - 1)
    
    def _move_state_down(self):
        selection = self.states_listbox.curselection()
        states = self.config.get('states', [])
        if not selection or selection[0] >= len(states) - 1:
            return
        
        index = selection[0]
        states[index], states[index+1] = states[index+1], states[index]
        
        self._update_states_list()
        self._update_yaml_preview()
        self.states_listbox.selection_set(index + 1)
    
    def _new_config(self):
        if messagebox.askyesno('New Configuration', 'Create new configuration? Unsaved changes will be lost.'):
            self.current_file = None
            self.config = {
                'name': 'new_challenge',
                'description': 'New challenge description',
                'userdata': {},
                'states': [],
                'outcomes': ['SUCCEEDED', 'FAILED']
            }
            self._update_ui()
            self.status_var.set('New configuration created')
    
    def _open_config(self):
        filepath = filedialog.askopenfilename(
            title='Open Configuration',
            filetypes=[('YAML files', '*.yaml *.yml'), ('All files', '*.*')],
            initialdir=os.path.expanduser('~/catkin_ws/src/hsr_task_sm/ros/config/challenges')
        )
        
        if filepath:
            try:
                with open(filepath, 'r') as f:
                    self.config = yaml.safe_load(f)
                self.current_file = filepath
                self._update_ui()
                self.status_var.set(f'Opened: {filepath}')
            except Exception as e:
                messagebox.showerror('Error', f'Failed to open file: {e}')
    
    def _save_config(self):
        if self.current_file:
            self._save_to_file(self.current_file)
        else:
            self._save_as_config()
    
    def _save_as_config(self):
        filepath = filedialog.asksaveasfilename(
            title='Save Configuration',
            filetypes=[('YAML files', '*.yaml'), ('All files', '*.*')],
            defaultextension='.yaml',
            initialdir=os.path.expanduser('~/catkin_ws/src/hsr_task_sm/ros/config/challenges')
        )
        
        if filepath:
            self._save_to_file(filepath)
    
    def _save_to_file(self, filepath: str):
        try:
            with open(filepath, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False, sort_keys=False,
                         allow_unicode=True, indent=2)
            self.current_file = filepath
            self.status_var.set(f'Saved: {filepath}')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to save file: {e}')
    
    def _validate_config(self):
        errors = []
        
        # Check required fields
        if not self.config.get('name'):
            errors.append('Missing name')
        
        if not self.config.get('states'):
            errors.append('No states defined')
        
        # Check states
        state_names = set()
        outcomes = set(self.config.get('outcomes', []))
        
        for i, state in enumerate(self.config.get('states', [])):
            name = state.get('name')
            if not name:
                errors.append(f'State {i}: missing name')
                continue
            
            if name in state_names:
                errors.append(f'Duplicate state name: {name}')
            state_names.add(name)
            
            if not state.get('type'):
                errors.append(f'State {name}: missing type')
            
            # Check transitions
            for outcome, target in state.get('transitions', {}).items():
                if target not in state_names and target not in outcomes:
                    errors.append(f'State {name}: transition "{outcome}" -> "{target}" targets unknown state')
        
        if errors:
            messagebox.showerror('Validation Failed', '\n'.join(errors))
        else:
            messagebox.showinfo('Validation Passed', 'Configuration is valid!')
        
        return len(errors) == 0
    
    def _preview_yaml(self):
        preview_window = tk.Toplevel(self.root)
        preview_window.title('YAML Preview')
        preview_window.geometry('600x800')
        
        text = scrolledtext.ScrolledText(preview_window, font=('Consolas', 11))
        text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        yaml_str = yaml.dump(self.config, default_flow_style=False, sort_keys=False,
                            allow_unicode=True, indent=2)
        text.insert('1.0', yaml_str)
        
        ttk.Button(preview_window, text='Copy to Clipboard', 
                  command=lambda: self._copy_to_clipboard(yaml_str)).pack(pady=5)
    
    def _copy_to_clipboard(self, text: str):
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.status_var.set('Copied to clipboard')
    
    def _show_state_types(self):
        ref_window = tk.Toplevel(self.root)
        ref_window.title('State Types Reference')
        ref_window.geometry('700x600')
        
        text = scrolledtext.ScrolledText(ref_window, font=('Consolas', 10))
        text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        for type_name, info in STATE_TYPES.items():
            text.insert(tk.END, f'━━━ {type_name} ━━━\n')
            text.insert(tk.END, f'  {info["description"]}\n')
            text.insert(tk.END, f'  Parameters: {info["params"]}\n')
            text.insert(tk.END, f'  Outcomes: {info["outcomes"]}\n\n')
    
    def _show_about(self):
        messagebox.showinfo('About', 
            'State Machine Editor\n'
            'RoboCup@Home 2026\n\n'
            'A visual editor for creating and modifying\n'
            'YAML state machine configurations.')


def main():
    root = tk.Tk()
    app = StateMachineEditorApp(root)
    
    # Open file from command line argument
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    app.config = yaml.safe_load(f)
                app.current_file = filepath
                app._update_ui()
            except Exception as e:
                messagebox.showerror('Error', f'Failed to open file: {e}')
    
    root.mainloop()


if __name__ == '__main__':
    main()
