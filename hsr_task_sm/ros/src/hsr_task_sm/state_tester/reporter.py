#!/usr/bin/env python3
"""
Test Reporter Module

Generates test reports in various formats:
- Console output (colored)
- JSON reports
- HTML reports
"""

import json
import datetime
from typing import List, Dict, Any
from dataclasses import dataclass

from hsr_task_sm.state_tester.core import StateTestResult


class Colors:
    """ANSI color codes for terminal output."""
    RESET = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    DIM = '\033[2m'


class TestReporter:
    """
    Generates formatted test reports.
    
    Usage:
        reporter = TestReporter()
        reporter.add_results(test_results)
        
        # Print to console
        reporter.print_summary()
        
        # Save reports
        reporter.save_json('test_results.json')
        reporter.save_html('test_results.html')
    """
    
    def __init__(self, use_colors: bool = True):
        """
        Initialize reporter.
        
        Args:
            use_colors: Use ANSI colors in console output
        """
        self.use_colors = use_colors
        self.results: List[StateTestResult] = []
        self.metadata = {
            'start_time': None,
            'end_time': None,
            'test_mode': None,
            'robot_name': None
        }
    
    def _c(self, color: str, text: str) -> str:
        """Apply color to text if colors enabled."""
        if self.use_colors:
            return f"{color}{text}{Colors.RESET}"
        return text
    
    def add_result(self, result: StateTestResult):
        """Add a single test result."""
        self.results.append(result)
    
    def add_results(self, results: List[StateTestResult]):
        """Add multiple test results."""
        self.results.extend(results)
    
    def set_metadata(self, **kwargs):
        """Set report metadata."""
        self.metadata.update(kwargs)
    
    def get_stats(self) -> Dict[str, Any]:
        """Calculate test statistics."""
        if not self.results:
            return {'total': 0, 'passed': 0, 'failed': 0, 'pass_rate': 0}
        
        passed = sum(1 for r in self.results if r.success)
        failed = len(self.results) - passed
        total_time = sum(r.execution_time for r in self.results)
        
        return {
            'total': len(self.results),
            'passed': passed,
            'failed': failed,
            'pass_rate': 100 * passed / len(self.results),
            'total_time': total_time,
            'avg_time': total_time / len(self.results)
        }
    
    def print_summary(self):
        """Print colorful summary to console."""
        stats = self.get_stats()
        
        print("\n" + "=" * 60)
        print(self._c(Colors.BOLD, "STATE TEST RESULTS"))
        print("=" * 60 + "\n")
        
        # Individual results
        for result in self.results:
            if result.success:
                status = self._c(Colors.GREEN, "✓ PASS")
            else:
                status = self._c(Colors.RED, "✗ FAIL")
            
            print(f"{status}  {result.state_name}")
            print(f"       Outcome: {result.outcome}")
            print(f"       Time: {result.execution_time:.3f}s")
            
            if result.error:
                print(f"       {self._c(Colors.RED, 'Error:')} {result.error}")
            
            if result.logs:
                print(f"       Logs: {len(result.logs)} messages")
            print()
        
        # Summary
        print("-" * 60)
        
        if stats['pass_rate'] == 100:
            rate_color = Colors.GREEN
        elif stats['pass_rate'] >= 80:
            rate_color = Colors.YELLOW
        else:
            rate_color = Colors.RED
        
        print(f"Total:  {stats['total']} tests")
        print(f"Passed: {self._c(Colors.GREEN, str(stats['passed']))}")
        print(f"Failed: {self._c(Colors.RED, str(stats['failed']))}")
        rate_str = f"{stats['pass_rate']:.1f}%"
        print(f"Rate:   {self._c(rate_color, rate_str)}")
        print(f"Time:   {stats['total_time']:.2f}s")
        print("=" * 60 + "\n")
    
    def print_result(self, result: StateTestResult):
        """Print a single result with details."""
        if result.success:
            status = self._c(Colors.GREEN, "✓ PASSED")
        else:
            status = self._c(Colors.RED, "✗ FAILED")
        
        print(f"\n{self._c(Colors.BOLD, result.state_name)}: {status}")
        print(f"  Outcome: {result.outcome}")
        print(f"  Time: {result.execution_time:.3f}s")
        
        if result.input_userdata:
            print(f"  Input: {result.input_userdata}")
        
        if result.output_userdata and result.output_userdata != result.input_userdata:
            print(f"  Output: {result.output_userdata}")
        
        if result.error:
            print(f"  {self._c(Colors.RED, 'Error:')} {result.error}")
        
        if result.logs:
            print(f"  {self._c(Colors.CYAN, 'Logs:')}")
            for log in result.logs[-5:]:  # Last 5 logs
                print(f"    {log}")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert all results to dictionary."""
        stats = self.get_stats()
        return {
            'metadata': {
                **self.metadata,
                'generated_at': datetime.datetime.now().isoformat()
            },
            'summary': stats,
            'results': [r.to_dict() for r in self.results]
        }
    
    def save_json(self, filepath: str):
        """Save results to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2, default=str)
        print(f"Report saved to: {filepath}")
    
    def save_html(self, filepath: str):
        """Save results to HTML file."""
        stats = self.get_stats()
        
        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>State Test Report</title>
    <style>
        body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; 
               max-width: 900px; margin: 50px auto; padding: 20px; }}
        h1 {{ color: #333; border-bottom: 2px solid #4a90d9; padding-bottom: 10px; }}
        .summary {{ background: #f5f5f5; padding: 20px; border-radius: 8px; margin: 20px 0; }}
        .summary-stats {{ display: flex; gap: 30px; }}
        .stat {{ text-align: center; }}
        .stat-value {{ font-size: 2em; font-weight: bold; }}
        .stat-label {{ color: #666; }}
        .passed {{ color: #28a745; }}
        .failed {{ color: #dc3545; }}
        .result {{ border: 1px solid #ddd; margin: 10px 0; padding: 15px; border-radius: 5px; }}
        .result.pass {{ border-left: 4px solid #28a745; }}
        .result.fail {{ border-left: 4px solid #dc3545; }}
        .result-header {{ display: flex; justify-content: space-between; align-items: center; }}
        .result-name {{ font-weight: bold; font-size: 1.1em; }}
        .result-outcome {{ padding: 3px 10px; border-radius: 3px; font-size: 0.9em; }}
        .result-details {{ margin-top: 10px; color: #666; font-size: 0.9em; }}
        .error {{ color: #dc3545; background: #ffeaea; padding: 10px; border-radius: 4px; margin-top: 10px; }}
        .logs {{ background: #f8f9fa; padding: 10px; border-radius: 4px; margin-top: 10px; 
                 font-family: monospace; font-size: 0.85em; max-height: 200px; overflow-y: auto; }}
    </style>
</head>
<body>
    <h1>🤖 State Test Report</h1>
    
    <div class="summary">
        <div class="summary-stats">
            <div class="stat">
                <div class="stat-value">{stats['total']}</div>
                <div class="stat-label">Total Tests</div>
            </div>
            <div class="stat">
                <div class="stat-value passed">{stats['passed']}</div>
                <div class="stat-label">Passed</div>
            </div>
            <div class="stat">
                <div class="stat-value failed">{stats['failed']}</div>
                <div class="stat-label">Failed</div>
            </div>
            <div class="stat">
                <div class="stat-value">{stats['pass_rate']:.1f}%</div>
                <div class="stat-label">Pass Rate</div>
            </div>
            <div class="stat">
                <div class="stat-value">{stats['total_time']:.2f}s</div>
                <div class="stat-label">Total Time</div>
            </div>
        </div>
    </div>
    
    <h2>Test Results</h2>
"""
        
        for result in self.results:
            status_class = "pass" if result.success else "fail"
            status_text = "PASSED" if result.success else "FAILED"
            status_color = "#28a745" if result.success else "#dc3545"
            
            html += f"""
    <div class="result {status_class}">
        <div class="result-header">
            <span class="result-name">{result.state_name}</span>
            <span class="result-outcome" style="background: {status_color}; color: white;">
                {status_text}
            </span>
        </div>
        <div class="result-details">
            <strong>Outcome:</strong> {result.outcome} | 
            <strong>Time:</strong> {result.execution_time:.3f}s
        </div>
"""
            if result.error:
                html += f"""
        <div class="error">
            <strong>Error:</strong> {result.error}
        </div>
"""
            if result.logs:
                logs_html = "<br>".join(result.logs[-10:])
                html += f"""
        <div class="logs">{logs_html}</div>
"""
            html += "    </div>\n"
        
        html += f"""
    <footer style="margin-top: 40px; color: #999; font-size: 0.9em;">
        Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
    </footer>
</body>
</html>"""
        
        with open(filepath, 'w') as f:
            f.write(html)
        print(f"HTML report saved to: {filepath}")
    
    def clear(self):
        """Clear all results."""
        self.results = []
