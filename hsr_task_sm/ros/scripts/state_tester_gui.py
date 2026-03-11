#!/usr/bin/env python3
"""
State Tester GUI - Visual interface for testing individual SMACH states

Supports both offline mock testing and live on-robot testing with a clean UI.

Usage:
    rosrun hsr_task_sm state_tester_gui.py
"""

import sys
import json
import yaml
import threading
from datetime import datetime

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QPushButton, QComboBox, QTextEdit, QLineEdit, QTabWidget,
        QTableWidget, QTableWidgetItem, QHeaderView, QGroupBox, QSpinBox,
        QDoubleSpinBox, QCheckBox, QSplitter, QFileDialog, QMessageBox,
        QListWidget, QListWidgetItem, QFormLayout, QScrollArea, QFrame,
        QProgressBar, QStatusBar
    )
    from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
    from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False
    print("PyQt5 not available. Install with: pip install PyQt5")

import rospy

from hsr_task_sm.state_tester.core import StateTester, TestMode, STATE_REGISTRY, StateTestResult
from hsr_task_sm.state_tester.mock_services import MockServices, MockScenarios
from hsr_task_sm.state_tester.validator import StateValidator
from hsr_task_sm.state_tester.reporter import TestReporter


class TestWorker(QThread):
    """Background worker for running state tests."""
    
    finished = pyqtSignal(object)  # StateTestResult
    progress = pyqtSignal(str)
    
    def __init__(self, tester, state_name, params, userdata, timeout):
        super().__init__()
        self.tester = tester
        self.state_name = state_name
        self.params = params
        self.userdata = userdata
        self.timeout = timeout
    
    def run(self):
        self.progress.emit(f"Executing {self.state_name}...")
        result = self.tester.test_state(
            self.state_name, self.params, self.userdata, timeout=self.timeout
        )
        self.finished.emit(result)


class StateTesterGUI(QMainWindow):
    """Main GUI window for state testing."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HSR State Tester")
        self.setMinimumSize(1200, 800)
        
        # Initialize ROS
        try:
            rospy.init_node('state_tester_gui', anonymous=True)
        except:
            pass
        
        # State
        self.mode = TestMode.MOCK
        self.mocks = MockScenarios.all_success()
        self.tester = StateTester(mode=self.mode, mock_services=self.mocks)
        self.validator = StateValidator()
        self.userdata = {}
        self.results = []
        self.current_worker = None
        
        self.init_ui()
        self.apply_style()
    
    def init_ui(self):
        """Initialize the user interface."""
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Top toolbar
        toolbar = self.create_toolbar()
        layout.addWidget(toolbar)
        
        # Main content with splitter
        splitter = QSplitter(Qt.Horizontal)
        
        # Left panel - State selection and config
        left_panel = self.create_left_panel()
        splitter.addWidget(left_panel)
        
        # Right panel - Results and logs
        right_panel = self.create_right_panel()
        splitter.addWidget(right_panel)
        
        splitter.setSizes([500, 700])
        layout.addWidget(splitter)
        
        # Status bar
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.statusbar.showMessage("Ready - Select a state to test")
    
    def create_toolbar(self):
        """Create top toolbar."""
        toolbar = QWidget()
        layout = QHBoxLayout(toolbar)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Mode selection
        layout.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Mock (Offline)", "Live (On Robot)", "Dry Run (Validate)"])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        self.mode_combo.setMinimumWidth(150)
        layout.addWidget(self.mode_combo)
        
        # Mock scenario
        layout.addWidget(QLabel("Scenario:"))
        self.scenario_combo = QComboBox()
        self.scenario_combo.addItems(["All Success", "Navigation Fail", "Manipulation Fail", "Slow"])
        self.scenario_combo.currentIndexChanged.connect(self.on_scenario_changed)
        layout.addWidget(self.scenario_combo)
        
        layout.addStretch()
        
        # Timeout
        layout.addWidget(QLabel("Timeout:"))
        self.timeout_spin = QDoubleSpinBox()
        self.timeout_spin.setRange(1, 300)
        self.timeout_spin.setValue(30)
        self.timeout_spin.setSuffix("s")
        layout.addWidget(self.timeout_spin)
        
        # Actions
        self.run_btn = QPushButton("▶ Run Test")
        self.run_btn.clicked.connect(self.run_test)
        self.run_btn.setMinimumWidth(120)
        self.run_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
        layout.addWidget(self.run_btn)
        
        validate_btn = QPushButton("✓ Validate")
        validate_btn.clicked.connect(self.validate_state)
        layout.addWidget(validate_btn)
        
        clear_btn = QPushButton("Clear Results")
        clear_btn.clicked.connect(self.clear_results)
        layout.addWidget(clear_btn)
        
        return toolbar
    
    def create_left_panel(self):
        """Create left panel with state selection and configuration."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # State selection
        state_group = QGroupBox("Select State")
        state_layout = QVBoxLayout(state_group)
        
        # Category filter
        cat_layout = QHBoxLayout()
        cat_layout.addWidget(QLabel("Category:"))
        self.category_combo = QComboBox()
        self.category_combo.addItems([
            "All", "Navigation", "Perception", "Manipulation", 
            "HRI - Speech", "HRI - Ollama", "Gaze", "Following", "Utility"
        ])
        self.category_combo.currentIndexChanged.connect(self.filter_states)
        cat_layout.addWidget(self.category_combo)
        state_layout.addLayout(cat_layout)
        
        # State list
        self.state_list = QListWidget()
        self.state_list.itemClicked.connect(self.on_state_selected)
        self.populate_state_list()
        state_layout.addWidget(self.state_list)
        
        layout.addWidget(state_group)
        
        # Parameters
        params_group = QGroupBox("Parameters")
        params_layout = QVBoxLayout(params_group)
        
        self.params_edit = QTextEdit()
        self.params_edit.setPlaceholderText('{\n  "destination": "living_room_table"\n}')
        self.params_edit.setMaximumHeight(150)
        params_layout.addWidget(self.params_edit)
        
        # Quick param buttons
        quick_layout = QHBoxLayout()
        for name, params in [
            ("Nav: Home", '{"destination": "home"}'),
            ("Nav: Living", '{"destination": "living_room_table"}'),
            ("Speak", '{"text": "Hello world"}'),
            ("Wait 2s", '{"duration": 2.0}'),
        ]:
            btn = QPushButton(name)
            btn.clicked.connect(lambda _, p=params: self.params_edit.setPlainText(p))
            quick_layout.addWidget(btn)
        params_layout.addLayout(quick_layout)
        
        layout.addWidget(params_group)
        
        # Userdata
        userdata_group = QGroupBox("Userdata")
        userdata_layout = QVBoxLayout(userdata_group)
        
        self.userdata_edit = QTextEdit()
        self.userdata_edit.setPlaceholderText('{\n  "target_object": "cup"\n}')
        self.userdata_edit.setMaximumHeight(120)
        userdata_layout.addWidget(self.userdata_edit)
        
        layout.addWidget(userdata_group)
        
        # State info
        info_group = QGroupBox("State Info")
        info_layout = QVBoxLayout(info_group)
        
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(150)
        info_layout.addWidget(self.info_text)
        
        layout.addWidget(info_group)
        
        return panel
    
    def create_right_panel(self):
        """Create right panel with results and logs."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Tabs for results
        tabs = QTabWidget()
        
        # Results table
        results_tab = QWidget()
        results_layout = QVBoxLayout(results_tab)
        
        self.results_table = QTableWidget()
        self.results_table.setColumnCount(5)
        self.results_table.setHorizontalHeaderLabels([
            "State", "Outcome", "Time", "Status", "Error"
        ])
        self.results_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.results_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.results_table.itemClicked.connect(self.on_result_selected)
        results_layout.addWidget(self.results_table)
        
        tabs.addTab(results_tab, "Results")
        
        # Log viewer
        log_tab = QWidget()
        log_layout = QVBoxLayout(log_tab)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 10))
        log_layout.addWidget(self.log_text)
        
        tabs.addTab(log_tab, "Logs")
        
        # Output userdata
        output_tab = QWidget()
        output_layout = QVBoxLayout(output_tab)
        
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        output_layout.addWidget(self.output_text)
        
        # Copy output to userdata button
        copy_btn = QPushButton("Copy to Userdata →")
        copy_btn.clicked.connect(self.copy_output_to_userdata)
        output_layout.addWidget(copy_btn)
        
        tabs.addTab(output_tab, "Output Userdata")
        
        layout.addWidget(tabs)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Export buttons
        export_layout = QHBoxLayout()
        
        json_btn = QPushButton("Export JSON")
        json_btn.clicked.connect(lambda: self.export_results('json'))
        export_layout.addWidget(json_btn)
        
        html_btn = QPushButton("Export HTML Report")
        html_btn.clicked.connect(lambda: self.export_results('html'))
        export_layout.addWidget(html_btn)
        
        layout.addLayout(export_layout)
        
        return panel
    
    def populate_state_list(self, category=None):
        """Populate state list, optionally filtered by category."""
        self.state_list.clear()
        
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
        
        if category and category != "All":
            states = categories.get(category, [])
        else:
            states = sorted(STATE_REGISTRY.keys())
        
        for state in states:
            if state in STATE_REGISTRY:
                item = QListWidgetItem(state)
                self.state_list.addItem(item)
    
    def filter_states(self):
        """Filter state list by category."""
        category = self.category_combo.currentText()
        self.populate_state_list(category)
    
    def on_state_selected(self, item):
        """Handle state selection."""
        state_name = item.text()
        
        # Show state info
        info = self.tester.get_state_info(state_name)
        
        info_text = f"State: {info['name']}\n"
        info_text += f"Class: {info['class']}\n"
        info_text += f"Outcomes: {', '.join(info['outcomes'])}\n\n"
        info_text += "Parameters:\n"
        for name, spec in info['parameters'].items():
            info_text += f"  {name}: {spec['default']}\n"
        
        self.info_text.setPlainText(info_text)
    
    def on_mode_changed(self, index):
        """Handle mode change."""
        modes = [TestMode.MOCK, TestMode.LIVE, TestMode.DRY_RUN]
        self.mode = modes[index]
        
        # Enable/disable scenario combo
        self.scenario_combo.setEnabled(self.mode == TestMode.MOCK)
        
        # Update tester
        if self.mode == TestMode.LIVE:
            self.tester = StateTester(mode=self.mode)
            self.statusbar.showMessage("⚠ LIVE MODE - Commands will run on robot!")
            self.run_btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold;")
        else:
            self.tester = StateTester(mode=self.mode, mock_services=self.mocks)
            self.statusbar.showMessage("Ready - Mock mode active")
            self.run_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold;")
    
    def on_scenario_changed(self, index):
        """Handle mock scenario change."""
        scenarios = [
            MockScenarios.all_success(),
            MockScenarios.navigation_failure(),
            MockScenarios.manipulation_failure(),
            MockScenarios.slow_robot()
        ]
        self.mocks = scenarios[index]
        self.tester = StateTester(mode=self.mode, mock_services=self.mocks)
    
    def validate_state(self):
        """Validate current state configuration."""
        item = self.state_list.currentItem()
        if not item:
            QMessageBox.warning(self, "Warning", "Select a state first")
            return
        
        state_name = item.text()
        
        try:
            params = json.loads(self.params_edit.toPlainText() or '{}')
            userdata = json.loads(self.userdata_edit.toPlainText() or '{}')
        except json.JSONDecodeError as e:
            QMessageBox.critical(self, "JSON Error", f"Invalid JSON: {e}")
            return
        
        result = self.validator.validate(state_name, params, userdata)
        
        self.log_text.append(f"\n=== Validation: {state_name} ===")
        self.log_text.append(str(result))
        
        if result.valid:
            QMessageBox.information(self, "Validation", f"✓ {state_name} configuration is valid")
        else:
            errors = "\n".join(str(e) for e in result.errors)
            QMessageBox.warning(self, "Validation Errors", errors)
    
    def run_test(self):
        """Run the selected state test."""
        item = self.state_list.currentItem()
        if not item:
            QMessageBox.warning(self, "Warning", "Select a state first")
            return
        
        state_name = item.text()
        
        try:
            params = json.loads(self.params_edit.toPlainText() or '{}')
            userdata = json.loads(self.userdata_edit.toPlainText() or '{}')
        except json.JSONDecodeError as e:
            QMessageBox.critical(self, "JSON Error", f"Invalid JSON: {e}")
            return
        
        # Confirm live mode
        if self.mode == TestMode.LIVE:
            reply = QMessageBox.question(
                self, "Confirm Live Test",
                f"This will execute {state_name} on the REAL ROBOT.\n\nContinue?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return
        
        # Disable UI during test
        self.run_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)  # Indeterminate
        
        # Run in background thread
        timeout = self.timeout_spin.value()
        self.current_worker = TestWorker(
            self.tester, state_name, params, userdata, timeout
        )
        self.current_worker.finished.connect(self.on_test_finished)
        self.current_worker.progress.connect(self.on_test_progress)
        self.current_worker.start()
        
        self.log_text.append(f"\n=== Testing: {state_name} ({self.mode.value}) ===")
        self.log_text.append(f"Params: {params}")
        self.statusbar.showMessage(f"Running {state_name}...")
    
    def on_test_progress(self, message):
        """Handle test progress update."""
        self.log_text.append(message)
    
    def on_test_finished(self, result: StateTestResult):
        """Handle test completion."""
        self.run_btn.setEnabled(True)
        self.progress_bar.setVisible(False)
        
        # Store result
        self.results.append(result)
        
        # Add to table
        row = self.results_table.rowCount()
        self.results_table.insertRow(row)
        
        self.results_table.setItem(row, 0, QTableWidgetItem(result.state_name))
        self.results_table.setItem(row, 1, QTableWidgetItem(result.outcome))
        self.results_table.setItem(row, 2, QTableWidgetItem(f"{result.execution_time:.3f}s"))
        
        status_item = QTableWidgetItem("✓ PASS" if result.success else "✗ FAIL")
        status_item.setForeground(QColor("#28a745" if result.success else "#dc3545"))
        self.results_table.setItem(row, 3, status_item)
        
        self.results_table.setItem(row, 4, QTableWidgetItem(result.error or ""))
        
        # Update logs
        self.log_text.append(f"Outcome: {result.outcome}")
        self.log_text.append(f"Time: {result.execution_time:.3f}s")
        if result.error:
            self.log_text.append(f"Error: {result.error}")
        for log in result.logs:
            self.log_text.append(f"  {log}")
        
        # Update output userdata
        self.output_text.setPlainText(json.dumps(result.output_userdata, indent=2))
        
        # Status
        status = "PASSED" if result.success else "FAILED"
        self.statusbar.showMessage(f"Test {status}: {result.state_name} → {result.outcome}")
    
    def on_result_selected(self, item):
        """Handle result row selection to show details."""
        row = item.row()
        if row < len(self.results):
            result = self.results[row]
            
            self.log_text.clear()
            self.log_text.append(f"=== {result.state_name} ===")
            self.log_text.append(f"Outcome: {result.outcome}")
            self.log_text.append(f"Success: {result.success}")
            self.log_text.append(f"Time: {result.execution_time:.3f}s")
            self.log_text.append(f"\nInput Userdata:\n{json.dumps(result.input_userdata, indent=2)}")
            if result.error:
                self.log_text.append(f"\nError: {result.error}")
            if result.logs:
                self.log_text.append("\nLogs:")
                for log in result.logs:
                    self.log_text.append(f"  {log}")
            
            self.output_text.setPlainText(json.dumps(result.output_userdata, indent=2))
    
    def copy_output_to_userdata(self):
        """Copy output userdata to input."""
        output = self.output_text.toPlainText()
        if output:
            self.userdata_edit.setPlainText(output)
            self.statusbar.showMessage("Copied output to userdata")
    
    def clear_results(self):
        """Clear all results."""
        self.results.clear()
        self.results_table.setRowCount(0)
        self.log_text.clear()
        self.output_text.clear()
        self.tester.clear_results()
        self.statusbar.showMessage("Results cleared")
    
    def export_results(self, format_type):
        """Export results to file."""
        if not self.results:
            QMessageBox.warning(self, "Warning", "No results to export")
            return
        
        if format_type == 'json':
            filepath, _ = QFileDialog.getSaveFileName(
                self, "Save JSON Report", "test_results.json", "JSON Files (*.json)"
            )
        else:
            filepath, _ = QFileDialog.getSaveFileName(
                self, "Save HTML Report", "test_results.html", "HTML Files (*.html)"
            )
        
        if filepath:
            reporter = TestReporter()
            reporter.add_results(self.results)
            
            if format_type == 'json':
                reporter.save_json(filepath)
            else:
                reporter.save_html(filepath)
            
            self.statusbar.showMessage(f"Report saved: {filepath}")
    
    def apply_style(self):
        """Apply custom styling."""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #ccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                padding: 0 5px;
            }
            QPushButton {
                padding: 8px 15px;
                border-radius: 4px;
                background-color: #4a90d9;
                color: white;
                border: none;
            }
            QPushButton:hover {
                background-color: #357abd;
            }
            QPushButton:disabled {
                background-color: #ccc;
            }
            QTableWidget {
                gridline-color: #ddd;
                selection-background-color: #4a90d9;
            }
            QTextEdit {
                border: 1px solid #ccc;
                border-radius: 4px;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 4px;
            }
            QListWidget {
                border: 1px solid #ccc;
                border-radius: 4px;
            }
            QListWidget::item:selected {
                background-color: #4a90d9;
            }
        """)


def main():
    if not PYQT_AVAILABLE:
        print("PyQt5 is required. Install with: pip install PyQt5")
        return 1
    
    app = QApplication(sys.argv)
    app.setApplicationName("HSR State Tester")
    
    window = StateTesterGUI()
    window.show()
    
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
