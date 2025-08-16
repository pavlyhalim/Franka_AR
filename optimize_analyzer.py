#!/usr/bin/env python3
"""
Code optimization analyzer for Franka AR application.
Provides suggestions for performance improvements.
"""

import ast
import os
import re
from typing import List, Dict, Any

class OptimizationAnalyzer(ast.NodeVisitor):
    def __init__(self):
        self.issues = []
        self.suggestions = []
        self.current_file = ""
        self.current_function = ""
        
    def analyze_file(self, filepath: str) -> Dict[str, Any]:
        """Analyze a Python file for optimization opportunities."""
        self.current_file = filepath
        self.issues = []
        self.suggestions = []
        
        try:
            with open(filepath, 'r') as f:
                content = f.read()
                
            # Parse the AST
            tree = ast.parse(content)
            self.visit(tree)
            
            # Additional regex-based checks
            self._check_print_statements(content)
            self._check_string_formatting(content)
            
        except Exception as e:
            self.issues.append(f"Error parsing {filepath}: {e}")
        
        return {
            'file': filepath,
            'issues': self.issues,
            'suggestions': self.suggestions
        }
    
    def visit_FunctionDef(self, node):
        self.current_function = node.name
        self.generic_visit(node)
        self.current_function = ""
    
    def visit_For(self, node):
        """Check for inefficient loops."""
        # Look for list concatenation in loops
        for child in ast.walk(node):
            if isinstance(child, ast.AugAssign) and isinstance(child.op, ast.Add):
                if hasattr(child.target, 'id'):
                    self.issues.append(f"Potential inefficient list concatenation in loop in {self.current_function}")
                    self.suggestions.append("Consider using list comprehension or pre-allocating list size")
        
        self.generic_visit(node)
    
    def visit_Call(self, node):
        """Check for inefficient function calls."""
        if hasattr(node.func, 'attr'):
            # Check for repeated numpy array creation
            if hasattr(node.func.value, 'id') and node.func.value.id == 'np':
                if node.func.attr in ['array', 'zeros', 'ones', 'identity']:
                    self.issues.append(f"Numpy array creation in potentially hot path: {self.current_function}")
                    self.suggestions.append("Consider pre-allocating arrays outside loops")
            
            # Check for inefficient string operations
            if node.func.attr in ['format', 'join', 'replace']:
                self.issues.append(f"String operation that could be optimized: {self.current_function}")
        
        # Check for print statements in likely hot paths
        if hasattr(node.func, 'id') and node.func.id == 'print':
            if self.current_function in ['__main__', 'control_loop', 'update_pose']:
                self.issues.append(f"Print statement in performance-critical section: {self.current_function}")
                self.suggestions.append("Remove or reduce print statements in control loops")
        
        self.generic_visit(node)
    
    def visit_Import(self, node):
        """Check imports."""
        for alias in node.names:
            if alias.name in ['matplotlib.pyplot', 'seaborn']:
                self.suggestions.append("Heavy plotting library imported - consider lazy loading")
    
    def visit_While(self, node):
        """Check while loops for potential optimizations."""
        # Look for sleep calls in while loops
        for child in ast.walk(node):
            if isinstance(child, ast.Call) and hasattr(child.func, 'attr'):
                if child.func.attr == 'sleep':
                    self.suggestions.append("Consider using rate limiters instead of sleep in control loops")
        
        self.generic_visit(node)
    
    def _check_print_statements(self, content: str):
        """Check for print statements using regex."""
        print_count = len(re.findall(r'\bprint\s*\(', content))
        if print_count > 5:
            self.issues.append(f"High number of print statements ({print_count}) - impacts performance")
            self.suggestions.append("Consider using logging with appropriate levels")
    
    def _check_string_formatting(self, content: str):
        """Check for inefficient string formatting."""
        if '%' in content and 'format' not in content:
            self.issues.append("Old-style string formatting found")
            self.suggestions.append("Use f-strings or .format() for better performance")

def analyze_codebase(directory: str = '.') -> List[Dict[str, Any]]:
    """Analyze the entire codebase for optimization opportunities."""
    analyzer = OptimizationAnalyzer()
    results = []
    
    python_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.py'):
                python_files.append(os.path.join(root, file))
    
    for filepath in python_files:
        result = analyzer.analyze_file(filepath)
        if result['issues'] or result['suggestions']:
            results.append(result)
    
    return results

def generate_optimization_report(results: List[Dict[str, Any]]):
    """Generate a comprehensive optimization report."""
    print("\n" + "="*60)
    print("FRANKA AR CODE OPTIMIZATION REPORT")
    print("="*60)
    
    total_issues = sum(len(result['issues']) for result in results)
    total_suggestions = sum(len(result['suggestions']) for result in results)
    
    print(f"Files analyzed: {len(results)}")
    print(f"Total issues found: {total_issues}")
    print(f"Total suggestions: {total_suggestions}")
    print()
    
    for result in results:
        if result['issues'] or result['suggestions']:
            print(f"File: {result['file']}")
            print("-" * 40)
            
            if result['issues']:
                print("Issues:")
                for issue in result['issues']:
                    print(f"  • {issue}")
                print()
            
            if result['suggestions']:
                print("Suggestions:")
                for suggestion in result['suggestions']:
                    print(f"  → {suggestion}")
                print()
    
    # General recommendations
    print("GENERAL OPTIMIZATION RECOMMENDATIONS:")
    print("-" * 40)
    print("1. Profile code with tools like cProfile or line_profiler")
    print("2. Use numpy operations instead of Python loops where possible")
    print("3. Pre-allocate arrays and matrices outside control loops")
    print("4. Cache expensive calculations (transformations, rotations)")
    print("5. Use connection pooling for network operations")
    print("6. Consider using numba for JIT compilation of hot functions")
    print("7. Minimize memory allocations in high-frequency loops")
    print("8. Use asynchronous I/O for network operations when possible")
    print("=" * 60)

def check_c_cpp_files(directory: str = '.') -> List[str]:
    """Check C/C++ files for common optimization opportunities."""
    suggestions = []
    
    cpp_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(('.cpp', '.c', '.hpp', '.h')):
                cpp_files.append(os.path.join(root, file))
    
    for filepath in cpp_files:
        try:
            with open(filepath, 'r') as f:
                content = f.read()
            
            # Check for common C++ performance issues
            if 'std::endl' in content:
                suggestions.append(f"{filepath}: Use '\\n' instead of std::endl for better performance")
            
            if 'std::vector' in content and 'reserve(' not in content:
                suggestions.append(f"{filepath}: Consider using vector.reserve() to avoid reallocations")
            
            if 'new ' in content and 'delete' in content:
                suggestions.append(f"{filepath}: Consider using smart pointers instead of raw pointers")
            
            if 'sleep_for' in content:
                sleep_matches = re.findall(r'sleep_for\(.*?(\d+)', content)
                for match in sleep_matches:
                    if int(match) > 100:
                        suggestions.append(f"{filepath}: Large sleep duration ({match}ms) may impact performance")
        
        except Exception as e:
            suggestions.append(f"Error analyzing {filepath}: {e}")
    
    return suggestions

def main():
    print("Analyzing Franka AR codebase for optimization opportunities...")
    
    # Analyze Python files
    results = analyze_codebase('.')
    
    # Analyze C++ files
    cpp_suggestions = check_c_cpp_files('.')
    
    # Generate report
    generate_optimization_report(results)
    
    if cpp_suggestions:
        print("\nC/C++ OPTIMIZATION SUGGESTIONS:")
        print("-" * 40)
        for suggestion in cpp_suggestions:
            print(f"  → {suggestion}")
        print()

if __name__ == "__main__":
    main()