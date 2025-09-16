#!/usr/bin/env python3
"""
Simple iPhone-EKF Trajectory Visualizer (No Extra Dependencies)
==============================================================
Creates basic trajectory visualizations using only matplotlib
Perfect for Raspberry Pi or systems without plotly
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

class SimpleTrajectoryVisualizer:
    """Simple trajectory visualization with minimal dependencies"""
    
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = None
        self.load_data()
        
    def load_data(self):
        """Load and preprocess the CSV data"""
        self.data = pd.read_csv(self.csv_file)
        
        # Convert time to relative seconds
        self.data['time_rel'] = self.data['timestamp'] - self.data['timestamp'].iloc[0]
        
        # Convert angles to degrees
        self.data['roll_deg'] = np.degrees(self.data['roll'])
        self.data['pitch_deg'] = np.degrees(self.data['pitch'])
        self.data['yaw_deg'] = np.degrees(self.data['yaw'])
        
        # Calculate cumulative distance
        dx = np.diff(self.data['x'], prepend=self.data['x'].iloc[0])
        dy = np.diff(self.data['y'], prepend=self.data['y'].iloc[0])
        dz = np.diff(self.data['z'], prepend=self.data['z'].iloc[0])
        self.data['distance'] = np.cumsum(np.sqrt(dx**2 + dy**2 + dz**2))
        
        print(f"📊 Loaded {len(self.data)} data points")
        print(f"⏱️  Duration: {self.data['time_rel'].iloc[-1]:.1f} seconds")
        print(f"📏 Total distance: {self.data['distance'].iloc[-1]:.2f} meters")
        print(f"📍 Position range: X=[{self.data['x'].min():.1f}, {self.data['x'].max():.1f}], Y=[{self.data['y'].min():.1f}, {self.data['y'].max():.1f}], Z=[{self.data['z'].min():.1f}, {self.data['z'].max():.1f}]")
    
    def plot_complete_analysis(self, save_path=None):
        """Create comprehensive trajectory analysis in one figure"""
        fig = plt.figure(figsize=(20, 15))
        
        # 3D trajectory (main plot)
        ax1 = fig.add_subplot(231, projection='3d')
        
        # Color by time
        colors = plt.cm.viridis(self.data['time_rel'] / self.data['time_rel'].max())
        scatter = ax1.scatter(self.data['x'], self.data['y'], -self.data['z'], 
                             c=self.data['time_rel'], cmap='viridis', s=15, alpha=0.8)
        
        # Plot trajectory line
        ax1.plot(self.data['x'], self.data['y'], -self.data['z'], 'b-', alpha=0.6, linewidth=2)
        
        # Mark start and end
        ax1.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], -self.data['z'].iloc[0], 
                   c='green', s=150, marker='o', label='Start', edgecolors='black', linewidth=2)
        ax1.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], -self.data['z'].iloc[-1], 
                   c='red', s=150, marker='s', label='End', edgecolors='black', linewidth=2)
        
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_zlabel('Altitude (m)', fontsize=12)
        ax1.set_title('3D iPhone Trajectory\n(EKF State Estimates)', fontsize=14, fontweight='bold')
        ax1.legend(fontsize=10)
        
        # Add colorbar
        cbar1 = plt.colorbar(scatter, ax=ax1, shrink=0.6, pad=0.1)
        cbar1.set_label('Time (s)', fontsize=11)
        
        # Top view (X-Y) with direction arrows
        ax2 = fig.add_subplot(232)
        ax2.plot(self.data['x'], self.data['y'], 'b-', linewidth=3, alpha=0.7)
        
        # Add direction arrows
        skip = max(1, len(self.data) // 15)
        for i in range(0, len(self.data)-skip, skip):
            dx = self.data['x'].iloc[i+skip] - self.data['x'].iloc[i]
            dy = self.data['y'].iloc[i+skip] - self.data['y'].iloc[i]
            if abs(dx) > 0.05 or abs(dy) > 0.05:  # Only if significant movement
                ax2.arrow(self.data['x'].iloc[i], self.data['y'].iloc[i], dx*0.7, dy*0.7,
                         head_width=0.15, head_length=0.15, fc='red', ec='red', alpha=0.7)
        
        ax2.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], c='green', s=100, marker='o', label='Start', zorder=5)
        ax2.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], c='red', s=100, marker='s', label='End', zorder=5)
        
        ax2.set_xlabel('X Position (m)', fontsize=12)
        ax2.set_ylabel('Y Position (m)', fontsize=12)
        ax2.set_title('Top View (X-Y Plane)\nwith Movement Direction', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        ax2.legend(fontsize=10)
        
        # Side view (X-Z)
        ax3 = fig.add_subplot(233)
        ax3.plot(self.data['x'], self.data['z'], 'b-', linewidth=3)
        ax3.scatter(self.data['x'].iloc[0], self.data['z'].iloc[0], c='green', s=100, marker='o', zorder=5)
        ax3.scatter(self.data['x'].iloc[-1], self.data['z'].iloc[-1], c='red', s=100, marker='s', zorder=5)
        ax3.set_xlabel('X Position (m)', fontsize=12)
        ax3.set_ylabel('Z Position (m)', fontsize=12)
        ax3.set_title('Side View (X-Z Plane)', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.invert_yaxis()  # NED convention
        
        # Position time series
        ax4 = fig.add_subplot(234)
        ax4.plot(self.data['time_rel'], self.data['x'], 'r-', linewidth=2, label='X')
        ax4.plot(self.data['time_rel'], self.data['y'], 'g-', linewidth=2, label='Y')
        ax4.plot(self.data['time_rel'], self.data['z'], 'b-', linewidth=2, label='Z')
        ax4.set_xlabel('Time (s)', fontsize=12)
        ax4.set_ylabel('Position (m)', fontsize=12)
        ax4.set_title('Position vs Time', fontsize=14, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=10)
        
        # Orientation time series
        ax5 = fig.add_subplot(235)
        ax5.plot(self.data['time_rel'], self.data['roll_deg'], 'r-', linewidth=2, label='Roll')
        ax5.plot(self.data['time_rel'], self.data['pitch_deg'], 'g-', linewidth=2, label='Pitch')
        ax5.plot(self.data['time_rel'], self.data['yaw_deg'], 'b-', linewidth=2, label='Yaw')
        ax5.set_xlabel('Time (s)', fontsize=12)
        ax5.set_ylabel('Angle (degrees)', fontsize=12)
        ax5.set_title('Orientation vs Time', fontsize=14, fontweight='bold')
        ax5.grid(True, alpha=0.3)
        ax5.legend(fontsize=10)
        
        # Distance and covariance
        ax6 = fig.add_subplot(236)
        ax6_twin = ax6.twinx()
        
        line1 = ax6.plot(self.data['time_rel'], self.data['distance'], 'purple', linewidth=3, label='Cumulative Distance')
        line2 = ax6_twin.plot(self.data['time_rel'], self.data['cov_trace'], 'orange', linewidth=2, label='EKF Uncertainty')
        
        ax6.set_xlabel('Time (s)', fontsize=12)
        ax6.set_ylabel('Distance (m)', color='purple', fontsize=12)
        ax6_twin.set_ylabel('Covariance Trace', color='orange', fontsize=12)
        ax6.set_title('Distance & EKF Convergence', fontsize=14, fontweight='bold')
        ax6.grid(True, alpha=0.3)
        
        # Combine legends
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax6.legend(lines, labels, loc='center right', fontsize=10)
        
        plt.tight_layout()
        
        # Add overall title
        fig.suptitle(f'iPhone-EKF Trajectory Analysis\n{os.path.basename(self.csv_file)}', 
                    fontsize=16, fontweight='bold', y=0.98)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"📁 Complete analysis saved to {save_path}")
        
        return fig
    
    def plot_3d_only(self, save_path=None):
        """Create focused 3D trajectory plot"""
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Color by time with more points
        colors = plt.cm.plasma(self.data['time_rel'] / self.data['time_rel'].max())
        scatter = ax.scatter(self.data['x'], self.data['y'], -self.data['z'], 
                           c=self.data['time_rel'], cmap='plasma', s=25, alpha=0.8)
        
        # Plot trajectory line
        ax.plot(self.data['x'], self.data['y'], -self.data['z'], 'navy', alpha=0.7, linewidth=3)
        
        # Mark start and end with larger markers
        ax.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], -self.data['z'].iloc[0], 
                  c='lime', s=200, marker='o', label='Start', edgecolors='black', linewidth=3)
        ax.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], -self.data['z'].iloc[-1], 
                  c='red', s=200, marker='s', label='End', edgecolors='black', linewidth=3)
        
        # Enhance labels
        ax.set_xlabel('X Position (m)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Y Position (m)', fontsize=14, fontweight='bold')
        ax.set_zlabel('Altitude (m)', fontsize=14, fontweight='bold')
        ax.set_title(f'3D iPhone Trajectory\nTotal Distance: {self.data["distance"].iloc[-1]:.2f}m over {self.data["time_rel"].iloc[-1]:.1f}s', 
                    fontsize=16, fontweight='bold')
        ax.legend(fontsize=12)
        
        # Add colorbar with better formatting
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.8, pad=0.1)
        cbar.set_label('Time (seconds)', fontsize=12, fontweight='bold')
        
        # Better viewing angle
        ax.view_init(elev=20, azim=45)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"📁 3D trajectory saved to {save_path}")
        
        return fig

def main():
    parser = argparse.ArgumentParser(description='Simple iPhone EKF Trajectory Visualizer')
    parser.add_argument('csv_file', help='Path to EKF log CSV file')
    parser.add_argument('--output-dir', default='simple_plots', help='Output directory')
    parser.add_argument('--show', action='store_true', help='Show plots interactively')
    parser.add_argument('--3d-only', action='store_true', help='Generate only 3D plot')
    
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Initialize visualizer
    viz = SimpleTrajectoryVisualizer(args.csv_file)
    
    # Generate base filename
    base_name = os.path.splitext(os.path.basename(args.csv_file))[0]
    
    if getattr(args, '3d_only'):
        print("🎯 Generating focused 3D trajectory plot...")
        viz.plot_3d_only(save_path=os.path.join(args.output_dir, f'{base_name}_3d.png'))
    else:
        print("📊 Generating complete trajectory analysis...")
        viz.plot_complete_analysis(save_path=os.path.join(args.output_dir, f'{base_name}_complete.png'))
    
    if args.show:
        plt.show()
    
    print(f"✅ Visualization complete! Files saved in {args.output_dir}/")

if __name__ == "__main__":
    main()
