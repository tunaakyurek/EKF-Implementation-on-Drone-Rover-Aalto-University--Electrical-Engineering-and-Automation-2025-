"""
Memory Profiler for RoboMaster S1

This utility monitors memory usage during direct execution on the S1,
helping to understand the system's memory constraints and optimize
code for the limited RAM environment.

USAGE:
- Can be run standalone or imported as a module
- Provides lightweight memory monitoring without external dependencies
- Designed for the S1's 272MB RAM constraint
"""

import time
import gc
import sys
import os

class S1MemoryProfiler:
    """
    Lightweight memory profiler for RoboMaster S1
    
    This profiler is designed to work within the S1's constrained
    environment without requiring external libraries like psutil.
    """
    
    def __init__(self, sample_interval=1.0):
        """
        Initialize memory profiler
        
        Args:
            sample_interval: How often to sample memory (seconds)
        """
        self.sample_interval = sample_interval
        self.samples = []
        self.start_time = None
        self.is_running = False
        
        # Memory conversion factors
        self.KB = 1024
        self.MB = 1024 * 1024
        
        print("S1MemoryProfiler initialized")
    
    def get_system_memory(self):
        """
        Get system memory information from /proc/meminfo
        
        Returns:
            dict: Memory information in KB
        """
        try:
            with open('/proc/meminfo', 'r') as f:
                meminfo = f.read()
            
            memory_data = {}
            for line in meminfo.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    # Extract numeric value (remove 'kB' suffix)
                    value_str = value.strip().split()[0]
                    try:
                        memory_data[key.strip()] = int(value_str)
                    except ValueError:
                        continue
            
            return memory_data
        
        except Exception as e:
            print(f"Error reading /proc/meminfo: {e}")
            return {}
    
    def get_process_memory(self):
        """
        Get current process memory information from /proc/self/status
        
        Returns:
            dict: Process memory information in KB
        """
        try:
            with open('/proc/self/status', 'r') as f:
                status = f.read()
            
            process_data = {}
            for line in status.split('\n'):
                if line.startswith(('VmPeak', 'VmSize', 'VmLck', 'VmPin', 
                                   'VmHWM', 'VmRSS', 'VmData', 'VmStk')):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        # Extract numeric value (remove 'kB' suffix)
                        value_str = value.strip().split()[0]
                        try:
                            process_data[key.strip()] = int(value_str)
                        except ValueError:
                            continue
            
            return process_data
        
        except Exception as e:
            print(f"Error reading /proc/self/status: {e}")
            return {}
    
    def get_python_memory(self):
        """
        Get Python-specific memory information
        
        Returns:
            dict: Python memory statistics
        """
        # Force garbage collection to get accurate count
        gc.collect()
        
        python_data = {
            'objects': len(gc.get_objects()),
            'garbage': len(gc.garbage),
            'ref_count': sys.gettotalrefcount() if hasattr(sys, 'gettotalrefcount') else 0
        }
        
        # Try to get memory usage from sys.getsizeof for major objects
        try:
            # Estimate memory usage of all objects (expensive!)
            objects = gc.get_objects()
            total_size = 0
            
            # Sample first 1000 objects to avoid taking too long
            sample_size = min(1000, len(objects))
            for obj in objects[:sample_size]:
                try:
                    total_size += sys.getsizeof(obj)
                except:
                    continue
            
            # Estimate total based on sample
            if sample_size > 0:
                estimated_total = (total_size * len(objects)) // sample_size
                python_data['estimated_python_memory_bytes'] = estimated_total
        
        except Exception as e:
            print(f"Error estimating Python memory: {e}")
        
        return python_data
    
    def sample_memory(self):
        """
        Take a single memory sample
        
        Returns:
            dict: Complete memory sample
        """
        current_time = time.time()
        
        sample = {
            'timestamp': current_time,
            'relative_time': current_time - self.start_time if self.start_time else 0,
            'system': self.get_system_memory(),
            'process': self.get_process_memory(),
            'python': self.get_python_memory()
        }
        
        return sample
    
    def start_monitoring(self):
        """Start continuous memory monitoring"""
        if self.is_running:
            print("Memory monitoring already running")
            return
        
        self.start_time = time.time()
        self.is_running = True
        self.samples = []
        
        print(f"Starting memory monitoring (interval: {self.sample_interval}s)")
    
    def stop_monitoring(self):
        """Stop memory monitoring"""
        self.is_running = False
        print("Memory monitoring stopped")
    
    def monitor_loop(self, duration=None):
        """
        Run monitoring loop for specified duration
        
        Args:
            duration: How long to monitor (seconds), None for manual stop
        """
        if not self.is_running:
            self.start_monitoring()
        
        end_time = time.time() + duration if duration else None
        
        try:
            while self.is_running:
                # Take sample
                sample = self.sample_memory()
                self.samples.append(sample)
                
                # Print periodic status
                if len(self.samples) % 10 == 1:  # Every 10 samples
                    self._print_sample_summary(sample)
                
                # Check if duration exceeded
                if end_time and time.time() >= end_time:
                    break
                
                # Wait for next sample
                time.sleep(self.sample_interval)
        
        except KeyboardInterrupt:
            print("\\nMonitoring interrupted by user")
        
        self.stop_monitoring()
    
    def _print_sample_summary(self, sample):
        """Print a summary of a memory sample"""
        t = sample['relative_time']
        system = sample['system']
        process = sample['process']
        python = sample['python']
        
        print(f"\\n--- Memory Sample at t={t:.1f}s ---")
        
        # System memory
        if 'MemTotal' in system and 'MemFree' in system:
            total_mb = system['MemTotal'] / 1024
            free_mb = system['MemFree'] / 1024
            used_mb = total_mb - free_mb
            usage_pct = (used_mb / total_mb) * 100
            
            print(f"System: {used_mb:.1f}/{total_mb:.1f} MB ({usage_pct:.1f}% used)")
        
        # Process memory
        if 'VmRSS' in process:
            process_mb = process['VmRSS'] / 1024
            print(f"Process RSS: {process_mb:.1f} MB")
        
        if 'VmSize' in process:
            vsize_mb = process['VmSize'] / 1024
            print(f"Process Virtual: {vsize_mb:.1f} MB")
        
        # Python objects
        print(f"Python objects: {python['objects']}")
        if 'estimated_python_memory_bytes' in python:
            py_mb = python['estimated_python_memory_bytes'] / (1024 * 1024)
            print(f"Est. Python memory: {py_mb:.1f} MB")
    
    def get_peak_usage(self):
        """Get peak memory usage from samples"""
        if not self.samples:
            return None
        
        peak_system = 0
        peak_process = 0
        peak_python = 0
        
        for sample in self.samples:
            # System memory usage
            system = sample['system']
            if 'MemTotal' in system and 'MemFree' in system:
                used = system['MemTotal'] - system['MemFree']
                peak_system = max(peak_system, used)
            
            # Process memory usage
            process = sample['process']
            if 'VmRSS' in process:
                peak_process = max(peak_process, process['VmRSS'])
            
            # Python objects
            python = sample['python']
            peak_python = max(peak_python, python['objects'])
        
        return {
            'peak_system_kb': peak_system,
            'peak_process_kb': peak_process,
            'peak_python_objects': peak_python
        }
    
    def save_profile(self, filename=None):
        """Save memory profile to file"""
        if not self.samples:
            print("No samples to save")
            return False
        
        if filename is None:
            filename = f"memory_profile_{int(time.time())}.txt"
        
        try:
            with open(filename, 'w') as f:
                f.write("# RoboMaster S1 Memory Profile\n")
                f.write(f"# Samples: {len(self.samples)}\n")
                f.write(f"# Interval: {self.sample_interval}s\n")
                f.write("# time,sys_total_kb,sys_free_kb,proc_rss_kb,proc_vsize_kb,py_objects\n")
                
                for sample in self.samples:
                    t = sample['relative_time']
                    sys_data = sample['system']
                    proc_data = sample['process']
                    py_data = sample['python']
                    
                    sys_total = sys_data.get('MemTotal', 0)
                    sys_free = sys_data.get('MemFree', 0)
                    proc_rss = proc_data.get('VmRSS', 0)
                    proc_vsize = proc_data.get('VmSize', 0)
                    py_objects = py_data.get('objects', 0)
                    
                    f.write(f"{t:.1f},{sys_total},{sys_free},{proc_rss},{proc_vsize},{py_objects}\n")
            
            print(f"Memory profile saved to {filename}")
            return True
        
        except Exception as e:
            print(f"Error saving profile: {e}")
            return False
    
    def print_summary(self):
        """Print summary of memory monitoring session"""
        if not self.samples:
            print("No samples collected")
            return
        
        print(f"\\n=== Memory Profile Summary ===")
        print(f"Samples collected: {len(self.samples)}")
        print(f"Duration: {self.samples[-1]['relative_time']:.1f} seconds")
        print(f"Sample interval: {self.sample_interval}s")
        
        # Peak usage
        peak = self.get_peak_usage()
        if peak:
            print(f"\\nPeak Usage:")
            print(f"  System: {peak['peak_system_kb']/1024:.1f} MB")
            print(f"  Process RSS: {peak['peak_process_kb']/1024:.1f} MB")
            print(f"  Python objects: {peak['peak_python_objects']}")
        
        # Current status
        if self.samples:
            current = self.samples[-1]
            sys_data = current['system']
            if 'MemTotal' in sys_data and 'MemFree' in sys_data:
                total_mb = sys_data['MemTotal'] / 1024
                free_mb = sys_data['MemFree'] / 1024
                print(f"\\nCurrent System Memory: {free_mb:.1f} MB free of {total_mb:.1f} MB")


def memory_stress_test():
    """Run a memory stress test to see S1's limits"""
    print("=== Memory Stress Test ===")
    print("This test gradually increases memory usage to find limits")
    
    profiler = S1MemoryProfiler(sample_interval=0.5)
    profiler.start_monitoring()
    
    # Allocate memory in chunks
    memory_chunks = []
    chunk_size = 1024 * 1024  # 1 MB chunks
    
    try:
        for i in range(100):  # Try to allocate up to 100 MB
            # Create a chunk of memory
            chunk = bytearray(chunk_size)
            memory_chunks.append(chunk)
            
            # Take a memory sample
            sample = profiler.sample_memory()
            profiler.samples.append(sample)
            
            # Print progress
            allocated_mb = (i + 1)
            sys_data = sample['system']
            if 'MemFree' in sys_data:
                free_mb = sys_data['MemFree'] / 1024
                print(f"Allocated: {allocated_mb} MB, System free: {free_mb:.1f} MB")
                
                # Stop if we're getting low on memory
                if free_mb < 50:  # Stop when less than 50MB free
                    print(f"Stopping allocation - low memory warning")
                    break
            
            time.sleep(0.1)
    
    except MemoryError:
        print("MemoryError encountered!")
    except Exception as e:
        print(f"Error during stress test: {e}")
    
    finally:
        # Clean up
        del memory_chunks
        gc.collect()
        
        profiler.stop_monitoring()
        profiler.print_summary()
        profiler.save_profile("stress_test_profile.txt")


def monitor_ekf_execution():
    """Monitor memory during EKF execution"""
    print("=== EKF Memory Monitoring ===")
    
    profiler = S1MemoryProfiler(sample_interval=1.0)
    
    # Try to import and run minimal EKF
    try:
        from minimal_ekf import MinimalEKF, SensorReading
        print("Imported minimal EKF successfully")
        
        profiler.start_monitoring()
        
        # Create EKF and run simulation
        ekf = MinimalEKF()
        
        print("Running EKF simulation with memory monitoring...")
        
        for i in range(60):  # 1 minute simulation
            # Create sensor data
            sensor_data = SensorReading()
            sensor_data.timestamp = time.time()
            sensor_data.accel_x = 0.1
            sensor_data.accel_y = 9.8
            sensor_data.accel_z = 0.1
            sensor_data.gyro_x = 0.01
            sensor_data.gyro_y = 0.01
            sensor_data.gyro_z = 0.05
            
            # Process with EKF
            ekf.process_sensor_data(sensor_data)
            
            # Take memory sample
            sample = profiler.sample_memory()
            profiler.samples.append(sample)
            
            time.sleep(1.0)
        
        profiler.stop_monitoring()
        profiler.print_summary()
        profiler.save_profile("ekf_memory_profile.txt")
    
    except ImportError:
        print("Could not import minimal EKF - running basic monitoring instead")
        profiler.monitor_loop(duration=30)
        profiler.print_summary()


if __name__ == "__main__":
    print("=== RoboMaster S1 Memory Profiler ===")
    print("Choose test mode:")
    print("1. Basic monitoring (30 seconds)")
    print("2. Memory stress test")
    print("3. EKF execution monitoring")
    
    # For automated execution, run basic monitoring
    print("\\nRunning basic monitoring for 30 seconds...")
    profiler = S1MemoryProfiler(sample_interval=2.0)
    profiler.monitor_loop(duration=30)
    profiler.print_summary()
    profiler.save_profile()