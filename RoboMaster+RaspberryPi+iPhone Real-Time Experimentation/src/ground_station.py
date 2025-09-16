"""
Ground station application for receiving and visualizing EKF data
Run this on the Ground PC
"""

import sys
import time
import threading
import argparse
from typing import Dict, Any

from communication.network_client import GroundStationServer, NetworkConfig
from visualization.real_time_plotter import RealTimePlotter
from data_collection.data_logger import DataAnalyzer

class GroundStation:
    """Ground station for EKF data visualization and analysis"""
    
    def __init__(self, port: int = 5555):
        self.port = port
        
        # Initialize components
        self.server = GroundStationServer(port=port, callback=self._process_received_data)
        self.plotter = RealTimePlotter(max_points=2000)
        
        # Data tracking
        self.last_data_time = time.time()
        self.data_count = {"sensor": 0, "ekf": 0, "status": 0}
        
        print(f"Ground Station initialized on port {port}")
    
    def _process_received_data(self, data: Dict[str, Any]):
        """Process data received from Raspberry Pi"""
        data_type = data.get('type', 'unknown')
        timestamp = data.get('timestamp', time.time())
        
        try:
            if data_type == 'sensor_data':
                self.plotter.add_sensor_data(timestamp, data['data'])
                self.data_count['sensor'] += 1
                
            elif data_type == 'ekf_state':
                covariance_trace = data['data'].get('covariance_trace', 0.0)
                self.plotter.add_ekf_data(timestamp, data['data'], covariance_trace)
                self.data_count['ekf'] += 1
                
            elif data_type == 'status_update':
                self._handle_status_update(data['data'])
                self.data_count['status'] += 1
                
            elif data_type == 'connection_test':
                print(f"Connection test received: {data.get('message', 'No message')}")
            
            self.last_data_time = timestamp
            
            # Periodic status print
            if hasattr(self, 'last_print_time'):
                if time.time() - self.last_print_time > 10.0:  # Every 10 seconds
                    self._print_status()
                    self.last_print_time = time.time()
            else:
                self.last_print_time = time.time()
        
        except Exception as e:
            print(f"Error processing received data: {e}")
    
    def _handle_status_update(self, status_data: Dict[str, Any]):
        """Handle status updates from Raspberry Pi"""
        uptime = status_data.get('system_uptime', 0)
        robot_status = status_data.get('robot_status', {})
        data_logger = status_data.get('data_logger', {})
        
        print(f"\\n=== System Status Update ===")
        print(f"Uptime: {uptime:.1f} seconds")
        print(f"Robot connected: {robot_status.get('connected', False)}")
        print(f"Battery level: {robot_status.get('battery_level', 'Unknown')}")
        print(f"Sensor samples: {data_logger.get('sensor_count', 0)}")
        print(f"EKF states: {data_logger.get('ekf_count', 0)}")
        print("=" * 30)
    
    def _print_status(self):
        """Print ground station status"""
        data_age = time.time() - self.last_data_time
        
        print(f"\\n--- Ground Station Status ---")
        print(f"Data received - Sensor: {self.data_count['sensor']}, "
              f"EKF: {self.data_count['ekf']}, Status: {self.data_count['status']}")
        print(f"Last data received: {data_age:.1f} seconds ago")
        print("-" * 30)
    
    def start_server(self) -> bool:
        """Start the ground station server"""
        print("Starting Ground Station server...")
        
        if not self.server.start_server():
            print("Failed to start server")
            return False
        
        print(f"Ground Station server running on port {self.port}")
        print("Waiting for data from Raspberry Pi...")
        return True
    
    def start_visualization(self):
        """Start real-time visualization"""
        print("Starting real-time visualization...")
        print("Close the plot window to stop the ground station")
        
        # Start the animation in a separate thread
        plot_thread = threading.Thread(target=self.plotter.start_animation, args=(100,))
        plot_thread.daemon = True
        plot_thread.start()
        
        return plot_thread
    
    def run(self, enable_visualization: bool = True):
        """Run the ground station"""
        if not self.start_server():
            return False
        
        try:
            if enable_visualization:
                plot_thread = self.start_visualization()
                
                # Keep the main thread alive while visualization is running
                plot_thread.join()
            else:
                # Just run the server without visualization
                print("Running without visualization. Press Ctrl+C to stop.")
                while True:
                    time.sleep(1.0)
        
        except KeyboardInterrupt:
            print("\\nReceived keyboard interrupt")
        
        except Exception as e:
            print(f"Error in ground station: {e}")
        
        finally:
            self.shutdown()
        
        return True
    
    def shutdown(self):
        """Shutdown the ground station"""
        print("Shutting down Ground Station...")
        
        if hasattr(self, 'server'):
            self.server.stop_server()
        
        # Save received data
        if hasattr(self, 'server') and len(self.server.received_data) > 0:
            timestamp = int(time.time())
            filename = f"data/ground_station_data_{timestamp}.json"
            self.server.save_received_data(filename)
            print(f"Saved received data to {filename}")
        
        print("Ground Station shutdown complete")

def run_analysis_mode(log_directory: str = "logs"):
    """Run analysis mode for offline data analysis"""
    print("=== EKF Data Analysis Mode ===")
    
    analyzer = DataAnalyzer(log_directory)
    sessions = analyzer.get_available_sessions()
    
    if not sessions:
        print(f"No session data found in {log_directory}")
        return
    
    print(f"Found {len(sessions)} sessions:")
    for i, session in enumerate(sessions):
        print(f"{i+1}. {session}")
    
    try:
        choice = input("\\nEnter session number for analysis (or 'all' for summary): ").strip()
        
        if choice.lower() == 'all':
            # Generate summary for all sessions
            for session in sessions:
                report = analyzer.generate_summary_report(session)
                print(f"\\n{report}")
        else:
            session_idx = int(choice) - 1
            if 0 <= session_idx < len(sessions):
                session_id = sessions[session_idx]
                
                # Generate detailed report
                report = analyzer.generate_summary_report(session_id)
                print(f"\\n{report}")
                
                # Offer visualization
                vis_choice = input("\\nGenerate trajectory plot? (y/n): ").strip().lower()
                if vis_choice == 'y':
                    from visualization.real_time_plotter import AnalysisPlotter
                    
                    data = analyzer.load_session_data(session_id)
                    ekf_data = [d for d in data.get('ekf', []) if isinstance(d, dict)]
                    sensor_data = [d for d in data.get('sensors', []) if isinstance(d, dict)]
                    
                    if ekf_data and sensor_data:
                        # Convert to expected format
                        ekf_formatted = [{"data": {"position": [d.get('pos_x', 0), d.get('pos_y', 0), d.get('pos_z', 0)]}} for d in ekf_data]
                        sensor_formatted = [{"data": {"chassis_x": d.get('chassis_x'), "chassis_y": d.get('chassis_y')}} for d in sensor_data]
                        
                        AnalysisPlotter.plot_trajectory_comparison(ekf_formatted, sensor_formatted)
            else:
                print("Invalid session number")
    
    except (ValueError, KeyboardInterrupt):
        print("Analysis cancelled")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='RoboMaster S1 EKF Ground Station')
    parser.add_argument('--port', type=int, default=5555,
                       help='Server port number')
    parser.add_argument('--no-viz', action='store_true',
                       help='Disable real-time visualization')
    parser.add_argument('--analysis', action='store_true',
                       help='Run in analysis mode for offline data')
    parser.add_argument('--log-dir', type=str, default='logs',
                       help='Log directory for analysis mode')
    
    args = parser.parse_args()
    
    if args.analysis:
        run_analysis_mode(args.log_dir)
        return 0
    
    # Create ground station
    ground_station = GroundStation(port=args.port)
    
    # Run ground station
    success = ground_station.run(enable_visualization=not args.no_viz)
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())