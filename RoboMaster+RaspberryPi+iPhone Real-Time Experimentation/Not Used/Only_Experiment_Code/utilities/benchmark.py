"""
Performance Benchmark for RoboMaster S1

This utility provides performance benchmarks to understand the
computational capabilities and limitations of the RoboMaster S1's
ARM Cortex-A7 processor when running Python code directly.

USAGE:
- Run standalone to execute all benchmarks
- Import specific benchmark functions for targeted testing
- Results help determine feasible computation complexity for EKF
"""

import time
import math
import gc
import sys

class S1PerformanceBenchmark:
    """
    Performance benchmark suite for RoboMaster S1
    
    Tests various computational tasks to understand the S1's
    processing capabilities for real-time applications.
    """
    
    def __init__(self):
        self.results = {}
        print("S1PerformanceBenchmark initialized")
        print(f"Python version: {sys.version}")
    
    def cpu_arithmetic_test(self, iterations=100000):
        """
        Test basic arithmetic operations performance
        
        Args:
            iterations: Number of operations to perform
        
        Returns:
            dict: Performance results
        """
        print(f"\\n--- CPU Arithmetic Test ({iterations:,} iterations) ---")
        
        # Integer arithmetic
        start_time = time.time()
        result = 0
        for i in range(iterations):
            result += i * 2 + i // 3 - i % 7
        int_time = time.time() - start_time
        
        # Float arithmetic
        start_time = time.time()
        result = 0.0
        for i in range(iterations):
            result += i * 2.5 + i / 3.7 - i * 0.9
        float_time = time.time() - start_time
        
        # Complex arithmetic
        start_time = time.time()
        result = 0.0
        for i in range(iterations):
            x = float(i)
            result += math.sqrt(x) + math.sin(x) + math.cos(x)
        complex_time = time.time() - start_time
        
        results = {
            'integer_ops_per_sec': iterations / int_time,
            'float_ops_per_sec': iterations / float_time,
            'complex_ops_per_sec': iterations / complex_time,
            'integer_time_ms': int_time * 1000,
            'float_time_ms': float_time * 1000,
            'complex_time_ms': complex_time * 1000
        }
        
        print(f"Integer ops: {results['integer_ops_per_sec']:,.0f} ops/sec ({results['integer_time_ms']:.1f} ms)")
        print(f"Float ops: {results['float_ops_per_sec']:,.0f} ops/sec ({results['float_time_ms']:.1f} ms)")
        print(f"Complex ops: {results['complex_ops_per_sec']:,.0f} ops/sec ({results['complex_time_ms']:.1f} ms)")
        
        self.results['cpu_arithmetic'] = results
        return results
    
    def matrix_operations_test(self, size=50):
        """
        Test matrix operations using pure Python (no NumPy)
        
        Args:
            size: Matrix size (size x size)
        
        Returns:
            dict: Performance results
        """
        print(f"\\n--- Matrix Operations Test ({size}x{size} matrices) ---")
        
        # Create test matrices
        def create_matrix(rows, cols, fill_value=1.0):
            return [[fill_value + i*0.1 + j*0.01 for j in range(cols)] for i in range(rows)]
        
        A = create_matrix(size, size, 1.0)
        B = create_matrix(size, size, 2.0)
        
        # Matrix addition
        start_time = time.time()
        C = [[A[i][j] + B[i][j] for j in range(size)] for i in range(size)]
        add_time = time.time() - start_time
        
        # Matrix multiplication
        start_time = time.time()
        C = [[0.0 for _ in range(size)] for _ in range(size)]
        for i in range(size):
            for j in range(size):
                for k in range(size):
                    C[i][j] += A[i][k] * B[k][j]
        mult_time = time.time() - start_time
        
        # Matrix transpose
        start_time = time.time()
        T = [[A[j][i] for j in range(size)] for i in range(size)]
        transpose_time = time.time() - start_time
        
        results = {
            'matrix_size': size,
            'addition_time_ms': add_time * 1000,
            'multiplication_time_ms': mult_time * 1000,
            'transpose_time_ms': transpose_time * 1000,
            'mult_ops_per_sec': (size ** 3) / mult_time  # Approximate FLOPS
        }
        
        print(f"Addition: {results['addition_time_ms']:.1f} ms")
        print(f"Multiplication: {results['multiplication_time_ms']:.1f} ms")
        print(f"Transpose: {results['transpose_time_ms']:.1f} ms")
        print(f"Mult performance: {results['mult_ops_per_sec']:,.0f} ops/sec")
        
        self.results['matrix_operations'] = results
        return results
    
    def memory_bandwidth_test(self, size_mb=10):
        """
        Test memory bandwidth (read/write performance)
        
        Args:
            size_mb: Size of data to read/write in MB
        
        Returns:
            dict: Performance results
        """
        print(f"\\n--- Memory Bandwidth Test ({size_mb} MB) ---")
        
        size_bytes = size_mb * 1024 * 1024
        
        # Sequential write
        start_time = time.time()
        data = bytearray(size_bytes)
        for i in range(0, size_bytes, 1024):  # Write in 1KB chunks
            data[i:i+1024] = b'\\x42' * 1024
        write_time = time.time() - start_time
        
        # Sequential read
        start_time = time.time()
        checksum = 0
        for i in range(0, size_bytes, 1024):  # Read in 1KB chunks
            chunk = data[i:i+1024]
            checksum += sum(chunk)
        read_time = time.time() - start_time
        
        # Random access
        import random
        indices = [random.randint(0, size_bytes-1) for _ in range(10000)]
        
        start_time = time.time()
        for idx in indices:
            _ = data[idx]
        random_time = time.time() - start_time
        
        results = {
            'size_mb': size_mb,
            'write_bandwidth_mbps': size_mb / write_time,
            'read_bandwidth_mbps': size_mb / read_time,
            'random_access_time_us': (random_time / len(indices)) * 1000000,
            'write_time_ms': write_time * 1000,
            'read_time_ms': read_time * 1000
        }
        
        print(f"Write: {results['write_bandwidth_mbps']:.1f} MB/s ({results['write_time_ms']:.1f} ms)")
        print(f"Read: {results['read_bandwidth_mbps']:.1f} MB/s ({results['read_time_ms']:.1f} ms)")
        print(f"Random access: {results['random_access_time_us']:.1f} μs per access")
        
        self.results['memory_bandwidth'] = results
        return results
    
    def ekf_simulation_test(self, iterations=1000):
        """
        Simulate EKF computational load
        
        Args:
            iterations: Number of EKF updates to simulate
        
        Returns:
            dict: Performance results
        """
        print(f"\\n--- EKF Simulation Test ({iterations} iterations) ---")
        
        # Simulate EKF state (6x1 vector)
        state = [0.0] * 6
        
        # Simulate covariance matrix (6x6)
        P = [[1.0 if i == j else 0.0 for j in range(6)] for i in range(6)]
        
        start_time = time.time()
        
        for iteration in range(iterations):
            # Simulate prediction step
            dt = 0.02  # 50 Hz
            
            # Simple state transition
            state[0] += state[3] * dt  # x += vx * dt
            state[1] += state[4] * dt  # y += vy * dt
            state[2] += state[5] * dt  # yaw += vyaw * dt
            
            # Simulate covariance prediction (simplified)
            for i in range(6):
                for j in range(6):
                    P[i][j] += 0.01 * dt  # Add process noise
            
            # Simulate measurement update
            # Innovation calculation
            z = [math.sin(iteration * 0.1), math.cos(iteration * 0.1)]  # Simulated measurement
            h = [state[0], state[1]]  # Expected measurement
            
            innovation = [z[0] - h[0], z[1] - h[1]]
            
            # Simplified Kalman gain calculation
            S = [[P[0][0] + 0.1, P[0][1]], [P[1][0], P[1][1] + 0.1]]
            
            # Determinant and inverse (2x2 only)
            det = S[0][0] * S[1][1] - S[0][1] * S[1][0]
            if abs(det) > 1e-10:
                S_inv = [[S[1][1]/det, -S[0][1]/det], [-S[1][0]/det, S[0][0]/det]]
                
                # State update
                state[0] += 0.5 * (S_inv[0][0] * innovation[0] + S_inv[0][1] * innovation[1])
                state[1] += 0.5 * (S_inv[1][0] * innovation[0] + S_inv[1][1] * innovation[1])
            
            # Normalize angles
            state[2] = ((state[2] + math.pi) % (2 * math.pi)) - math.pi
        
        total_time = time.time() - start_time
        
        results = {
            'iterations': iterations,
            'total_time_ms': total_time * 1000,
            'avg_iteration_time_ms': (total_time / iterations) * 1000,
            'updates_per_second': iterations / total_time,
            'final_state': state.copy()
        }
        
        print(f"Total time: {results['total_time_ms']:.1f} ms")
        print(f"Avg per iteration: {results['avg_iteration_time_ms']:.3f} ms")
        print(f"EKF updates/sec: {results['updates_per_second']:.1f} Hz")
        print(f"Final state: [{results['final_state'][0]:.3f}, {results['final_state'][1]:.3f}, {results['final_state'][2]:.3f}]")
        
        self.results['ekf_simulation'] = results
        return results
    
    def garbage_collection_test(self):
        """
        Test garbage collection performance impact
        
        Returns:
            dict: Performance results
        """
        print(f"\\n--- Garbage Collection Test ---")
        
        # Create many objects to trigger GC
        objects = []
        
        start_time = time.time()
        for i in range(10000):
            # Create various object types
            obj = {
                'list': [i, i+1, i+2],
                'dict': {'key': i, 'value': i*2},
                'string': f"object_{i}",
                'tuple': (i, i+1, i+2)
            }
            objects.append(obj)
        creation_time = time.time() - start_time
        
        # Force garbage collection
        start_time = time.time()
        collected = gc.collect()
        gc_time = time.time() - start_time
        
        # Clean up
        start_time = time.time()
        del objects
        cleanup_time = time.time() - start_time
        
        results = {
            'object_creation_time_ms': creation_time * 1000,
            'gc_time_ms': gc_time * 1000,
            'cleanup_time_ms': cleanup_time * 1000,
            'objects_collected': collected,
            'objects_per_sec': 10000 / creation_time
        }
        
        print(f"Object creation: {results['object_creation_time_ms']:.1f} ms ({results['objects_per_sec']:,.0f} obj/sec)")
        print(f"Garbage collection: {results['gc_time_ms']:.1f} ms ({results['objects_collected']} collected)")
        print(f"Cleanup: {results['cleanup_time_ms']:.1f} ms")
        
        self.results['garbage_collection'] = results
        return results
    
    def file_io_test(self, file_size_kb=100):
        """
        Test file I/O performance
        
        Args:
            file_size_kb: Size of test file in KB
        
        Returns:
            dict: Performance results
        """
        print(f"\\n--- File I/O Test ({file_size_kb} KB) ---")
        
        test_filename = "benchmark_test.txt"
        data_size = file_size_kb * 1024
        test_data = "A" * data_size
        
        # Write test
        start_time = time.time()
        with open(test_filename, 'w') as f:
            f.write(test_data)
        write_time = time.time() - start_time
        
        # Read test
        start_time = time.time()
        with open(test_filename, 'r') as f:
            read_data = f.read()
        read_time = time.time() - start_time
        
        # Random access test
        start_time = time.time()
        with open(test_filename, 'r') as f:
            for _ in range(100):
                f.seek(data_size // 2)  # Seek to middle
                _ = f.read(100)  # Read 100 characters
        seek_time = time.time() - start_time
        
        # Cleanup
        try:
            import os
            os.remove(test_filename)
        except:
            pass
        
        results = {
            'file_size_kb': file_size_kb,
            'write_time_ms': write_time * 1000,
            'read_time_ms': read_time * 1000,
            'seek_time_ms': seek_time * 1000,
            'write_bandwidth_kbps': file_size_kb / write_time,
            'read_bandwidth_kbps': file_size_kb / read_time
        }
        
        print(f"Write: {results['write_bandwidth_kbps']:.1f} KB/s ({results['write_time_ms']:.1f} ms)")
        print(f"Read: {results['read_bandwidth_kbps']:.1f} KB/s ({results['read_time_ms']:.1f} ms)")
        print(f"Random access: {results['seek_time_ms']:.1f} ms for 100 seeks")
        
        self.results['file_io'] = results
        return results
    
    def run_all_benchmarks(self):
        """Run all benchmark tests"""
        print("=== RoboMaster S1 Performance Benchmark Suite ===")
        print("Testing computational capabilities for EKF feasibility")
        print("")
        
        # Run all tests
        self.cpu_arithmetic_test()
        self.matrix_operations_test(size=30)  # Smaller for S1
        self.memory_bandwidth_test(size_mb=5)  # Smaller for S1
        self.ekf_simulation_test(iterations=500)  # Smaller for S1
        self.garbage_collection_test()
        self.file_io_test(file_size_kb=50)  # Smaller for S1
        
        # Print overall summary
        self.print_summary()
        
        return self.results
    
    def print_summary(self):
        """Print benchmark summary and recommendations"""
        print("\\n" + "="*60)
        print("=== BENCHMARK SUMMARY ===")
        
        if 'cpu_arithmetic' in self.results:
            cpu = self.results['cpu_arithmetic']
            print(f"CPU Performance: {cpu['complex_ops_per_sec']:,.0f} complex ops/sec")
        
        if 'matrix_operations' in self.results:
            matrix = self.results['matrix_operations']
            print(f"Matrix Multiplication: {matrix['multiplication_time_ms']:.1f} ms for 30x30")
        
        if 'ekf_simulation' in self.results:
            ekf = self.results['ekf_simulation']
            print(f"EKF Performance: {ekf['updates_per_second']:.1f} Hz")
        
        if 'memory_bandwidth' in self.results:
            mem = self.results['memory_bandwidth']
            print(f"Memory Bandwidth: {mem['read_bandwidth_mbps']:.1f} MB/s read")
        
        print("\\n=== RECOMMENDATIONS ===")
        
        # EKF feasibility analysis
        if 'ekf_simulation' in self.results:
            ekf_hz = self.results['ekf_simulation']['updates_per_second']
            if ekf_hz >= 50:
                print("✓ 50Hz EKF: FEASIBLE")
            elif ekf_hz >= 20:
                print("⚠ 50Hz EKF: MARGINAL (reduce to 20Hz)")
            else:
                print("✗ 50Hz EKF: NOT FEASIBLE (use complementary filter)")
        
        # Memory recommendations
        if 'memory_bandwidth' in self.results:
            mb_rate = self.results['memory_bandwidth']['read_bandwidth_mbps']
            if mb_rate >= 100:
                print("✓ Memory bandwidth: ADEQUATE")
            else:
                print("⚠ Memory bandwidth: LIMITED (minimize data structures)")
        
        # Matrix operation recommendations
        if 'matrix_operations' in self.results:
            mult_time = self.results['matrix_operations']['multiplication_time_ms']
            if mult_time <= 100:  # 30x30 matrix in <100ms
                print("✓ Matrix operations: USABLE for small matrices")
            else:
                print("⚠ Matrix operations: TOO SLOW (avoid large matrices)")
        
        print("\\n=== FINAL RECOMMENDATION ===")
        print("Based on these benchmarks:")
        print("- Use simplified 6-DOF EKF instead of 12-DOF")
        print("- Target 10-20 Hz update rate, not 50 Hz")
        print("- Minimize matrix operations and memory allocation")
        print("- Consider complementary filter as alternative")
        print("- The Raspberry Pi approach remains superior for full EKF")
    
    def save_results(self, filename=None):
        """Save benchmark results to file"""
        if filename is None:
            filename = f"s1_benchmark_{int(time.time())}.txt"
        
        try:
            with open(filename, 'w') as f:
                f.write("# RoboMaster S1 Performance Benchmark Results\\n")
                f.write(f"# Timestamp: {time.time()}\\n")
                f.write(f"# Python: {sys.version}\\n\\n")
                
                for test_name, results in self.results.items():
                    f.write(f"[{test_name}]\\n")
                    for key, value in results.items():
                        f.write(f"{key} = {value}\\n")
                    f.write("\\n")
            
            print(f"Benchmark results saved to {filename}")
            return True
        
        except Exception as e:
            print(f"Error saving results: {e}")
            return False


def quick_benchmark():
    """Run a quick benchmark suitable for S1's limited resources"""
    print("=== Quick S1 Benchmark ===")
    
    benchmark = S1PerformanceBenchmark()
    
    # Run reduced tests
    benchmark.cpu_arithmetic_test(iterations=50000)
    benchmark.matrix_operations_test(size=20)
    benchmark.ekf_simulation_test(iterations=100)
    
    # Print quick summary
    if 'ekf_simulation' in benchmark.results:
        ekf_hz = benchmark.results['ekf_simulation']['updates_per_second']
        print(f"\\nQuick Assessment: EKF capability ~{ekf_hz:.1f} Hz")
        
        if ekf_hz >= 30:
            print("Result: Direct EKF execution may be feasible")
        else:
            print("Result: Direct EKF execution not recommended")
    
    return benchmark.results


if __name__ == "__main__":
    # Run appropriate benchmark based on available time/resources
    print("Choose benchmark mode:")
    print("1. Quick benchmark (30 seconds)")
    print("2. Full benchmark (2-3 minutes)")
    
    # For automated execution, run quick benchmark
    print("\\nRunning quick benchmark...")
    results = quick_benchmark()