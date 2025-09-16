#!/usr/bin/env python3
"""
iPhone-EKF Trajectory Visualization Tool
========================================
Creates comprehensive 3D visualizations of iPhone motion tracking data
Supports both static plots and interactive 3D visualization

Run this script to visualize your EKF trajectory data with:
- 3D trajectory plots
- Orientation visualization  
- Time-series analysis
- Interactive plots
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import argparse
import os

class TrajectoryVisualizer:
    """Advanced trajectory visualization for iPhone-EKF data"""
    
    def __init__(self, csv_file):
        """Initialize with CSV data file"""
        self.csv_file = csv_file
        self.data = None
        self.load_data()
        
    def load_data(self):
        """Load and preprocess the CSV data"""
        try:
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
            
            # Calculate speeds
            dt = np.diff(self.data['time_rel'], prepend=0.1)
            dt[dt == 0] = 0.1  # Avoid division by zero
            self.data['speed_3d'] = np.sqrt(dx**2 + dy**2 + dz**2) / dt
            
            print(f"Loaded {len(self.data)} data points from {os.path.basename(self.csv_file)}")
            print(f"Duration: {self.data['time_rel'].iloc[-1]:.1f} seconds")
            print(f"Total distance: {self.data['distance'].iloc[-1]:.2f} meters")
            
        except Exception as e:
            print(f"Error loading data: {e}")
            raise
    
    def plot_3d_trajectory_matplotlib(self, save_path=None, interactive=False):
        """Create 3D trajectory plot using matplotlib"""
        fig = plt.figure(figsize=(15, 12))
        
        # Main 3D trajectory
        ax1 = fig.add_subplot(221, projection='3d')
        
        # Color by time
        colors = plt.cm.viridis(self.data['time_rel'] / self.data['time_rel'].max())
        scatter = ax1.scatter(self.data['x'], self.data['y'], -self.data['z'], 
                             c=self.data['time_rel'], cmap='viridis', s=20, alpha=0.7)
        
        # Plot trajectory line
        ax1.plot(self.data['x'], self.data['y'], -self.data['z'], 'b-', alpha=0.5, linewidth=1)
        
        # Mark start and end
        ax1.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], -self.data['z'].iloc[0], 
                   c='green', s=100, marker='o', label='Start')
        ax1.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], -self.data['z'].iloc[-1], 
                   c='red', s=100, marker='s', label='End')
        
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_zlabel('Altitude (m)')
        ax1.set_title('3D iPhone Trajectory (EKF Estimated)')
        ax1.legend()
        
        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax1, shrink=0.6, pad=0.1)
        cbar.set_label('Time (s)')
        
        # Top view (X-Y)
        ax2 = fig.add_subplot(222)
        ax2.plot(self.data['x'], self.data['y'], 'b-', linewidth=2, alpha=0.7)
        ax2.scatter(self.data['x'].iloc[0], self.data['y'].iloc[0], c='green', s=100, marker='o', label='Start')
        ax2.scatter(self.data['x'].iloc[-1], self.data['y'].iloc[-1], c='red', s=100, marker='s', label='End')
        
        # Add arrows to show direction
        skip = max(1, len(self.data) // 20)
        for i in range(0, len(self.data)-skip, skip):
            dx = self.data['x'].iloc[i+skip] - self.data['x'].iloc[i]
            dy = self.data['y'].iloc[i+skip] - self.data['y'].iloc[i]
            if abs(dx) > 0.01 or abs(dy) > 0.01:  # Only if significant movement
                ax2.arrow(self.data['x'].iloc[i], self.data['y'].iloc[i], dx, dy,
                         head_width=0.1, head_length=0.1, fc='red', ec='red', alpha=0.5)
        
        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Top View (X-Y Plane)')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        ax2.legend()
        
        # Side view (X-Z)  
        ax3 = fig.add_subplot(223)
        ax3.plot(self.data['x'], self.data['z'], 'b-', linewidth=2)
        ax3.set_xlabel('X Position (m)')
        ax3.set_ylabel('Z Position (m)')
        ax3.set_title('Side View (X-Z Plane)')
        ax3.grid(True, alpha=0.3)
        ax3.invert_yaxis()  # NED convention
        
        # Speed profile
        ax4 = fig.add_subplot(224)
        ax4.plot(self.data['time_rel'], self.data['speed_3d'], 'r-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('3D Speed (m/s)')
        ax4.set_title('Speed Profile')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"3D trajectory plot saved to {save_path}")
        
        if interactive:
            plt.show()
        
        return fig
    
    def plot_orientation_analysis(self, save_path=None):
        """Create detailed orientation analysis plots"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Roll, Pitch, Yaw over time
        ax = axes[0, 0]
        ax.plot(self.data['time_rel'], self.data['roll_deg'], 'r-', label='Roll', linewidth=2)
        ax.plot(self.data['time_rel'], self.data['pitch_deg'], 'g-', label='Pitch', linewidth=2)
        ax.plot(self.data['time_rel'], self.data['yaw_deg'], 'b-', label='Yaw', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.set_title('Orientation vs Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 3D orientation sphere
        ax = axes[0, 1]
        # Create unit sphere points from roll/pitch
        phi = np.radians(self.data['roll_deg'])
        theta = np.radians(self.data['pitch_deg'])
        
        # Project to unit sphere
        x_sphere = np.cos(theta) * np.cos(phi)
        y_sphere = np.cos(theta) * np.sin(phi)
        
        scatter = ax.scatter(x_sphere, y_sphere, c=self.data['time_rel'], 
                           cmap='viridis', s=20, alpha=0.7)
        ax.set_xlabel('Roll Component')
        ax.set_ylabel('Pitch Component')
        ax.set_title('Orientation Sphere Projection')
        ax.axis('equal')
        plt.colorbar(scatter, ax=ax)
        
        # Yaw distribution
        ax = axes[1, 0]
        ax.hist(self.data['yaw_deg'], bins=30, alpha=0.7, color='blue', edgecolor='black')
        ax.set_xlabel('Yaw (degrees)')
        ax.set_ylabel('Frequency')
        ax.set_title('Yaw Distribution')
        ax.grid(True, alpha=0.3)
        
        # Angular rates
        ax = axes[1, 1]
        ax.plot(self.data['time_rel'], np.degrees(self.data['yaw_rate']), 'purple', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw Rate (deg/s)')
        ax.set_title('Angular Velocity')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Orientation analysis saved to {save_path}")
        
        return fig
    
    def create_interactive_plotly(self, save_path=None):
        """Create interactive 3D visualization using Plotly"""
        
        # Create subplots
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=('3D Trajectory', 'Top View (X-Y)', 'Position vs Time', 'Orientation vs Time'),
            specs=[[{"type": "scatter3d"}, {"type": "scatter"}],
                   [{"type": "scatter"}, {"type": "scatter"}]]
        )
        
        # 3D trajectory
        fig.add_trace(
            go.Scatter3d(
                x=self.data['x'], y=self.data['y'], z=-self.data['z'],
                mode='lines+markers',
                marker=dict(
                    size=4,
                    color=self.data['time_rel'],
                    colorscale='Viridis',
                    colorbar=dict(title="Time (s)"),
                    showscale=True
                ),
                line=dict(color='blue', width=3),
                name='Trajectory',
                text=[f'Time: {t:.1f}s<br>Pos: ({x:.2f}, {y:.2f}, {z:.2f})' 
                      for t, x, y, z in zip(self.data['time_rel'], self.data['x'], self.data['y'], self.data['z'])],
                hovertemplate='%{text}<extra></extra>'
            ),
            row=1, col=1
        )
        
        # Start and end markers
        fig.add_trace(
            go.Scatter3d(
                x=[self.data['x'].iloc[0]], y=[self.data['y'].iloc[0]], z=[-self.data['z'].iloc[0]],
                mode='markers',
                marker=dict(size=10, color='green'),
                name='Start',
                showlegend=False
            ),
            row=1, col=1
        )
        
        fig.add_trace(
            go.Scatter3d(
                x=[self.data['x'].iloc[-1]], y=[self.data['y'].iloc[-1]], z=[-self.data['z'].iloc[-1]],
                mode='markers',
                marker=dict(size=10, color='red'),
                name='End',
                showlegend=False
            ),
            row=1, col=1
        )
        
        # Top view
        fig.add_trace(
            go.Scatter(
                x=self.data['x'], y=self.data['y'],
                mode='lines+markers',
                marker=dict(color=self.data['time_rel'], colorscale='Viridis', size=4),
                line=dict(color='blue', width=2),
                name='XY Path',
                showlegend=False
            ),
            row=1, col=2
        )
        
        # Position vs time
        fig.add_trace(
            go.Scatter(
                x=self.data['time_rel'], y=self.data['x'],
                mode='lines', name='X', line=dict(color='red')
            ),
            row=2, col=1
        )
        fig.add_trace(
            go.Scatter(
                x=self.data['time_rel'], y=self.data['y'],
                mode='lines', name='Y', line=dict(color='green')
            ),
            row=2, col=1
        )
        fig.add_trace(
            go.Scatter(
                x=self.data['time_rel'], y=self.data['z'],
                mode='lines', name='Z', line=dict(color='blue')
            ),
            row=2, col=1
        )
        
        # Orientation vs time
        fig.add_trace(
            go.Scatter(
                x=self.data['time_rel'], y=self.data['roll_deg'],
                mode='lines', name='Roll', line=dict(color='red')
            ),
            row=2, col=2
        )
        fig.add_trace(
            go.Scatter(
                x=self.data['time_rel'], y=self.data['pitch_deg'],
                mode='lines', name='Pitch', line=dict(color='green')
            ),
            row=2, col=2
        )
        fig.add_trace(
            go.Scatter(
                x=self.data['time_rel'], y=self.data['yaw_deg'],
                mode='lines', name='Yaw', line=dict(color='blue')
            ),
            row=2, col=2
        )
        
        # Update layout
        fig.update_layout(
            title_text=f"iPhone EKF Trajectory Analysis - {os.path.basename(self.csv_file)}",
            height=800,
            showlegend=True
        )
        
        # Update 3D scene
        fig.update_scenes(
            xaxis_title="X Position (m)",
            yaxis_title="Y Position (m)", 
            zaxis_title="Altitude (m)",
            row=1, col=1
        )
        
        # Update other axes
        fig.update_xaxes(title_text="X Position (m)", row=1, col=2)
        fig.update_yaxes(title_text="Y Position (m)", row=1, col=2)
        fig.update_xaxes(title_text="Time (s)", row=2, col=1)
        fig.update_yaxes(title_text="Position (m)", row=2, col=1)
        fig.update_xaxes(title_text="Time (s)", row=2, col=2)
        fig.update_yaxes(title_text="Angle (degrees)", row=2, col=2)
        
        if save_path:
            fig.write_html(save_path)
            print(f"Interactive plot saved to {save_path}")
        
        return fig
    
    def create_animation(self, save_path=None, fps=10):
        """Create animated trajectory visualization"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Set up the plot
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Altitude (m)')
        ax.set_title('iPhone Trajectory Animation')
        
        # Set consistent axes limits
        margin = 0.5
        ax.set_xlim([self.data['x'].min() - margin, self.data['x'].max() + margin])
        ax.set_ylim([self.data['y'].min() - margin, self.data['y'].max() + margin])
        ax.set_zlim([-self.data['z'].max() - margin, -self.data['z'].min() + margin])
        
        # Initialize empty plots
        line, = ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7)
        point, = ax.plot([], [], [], 'ro', markersize=8)
        trail, = ax.plot([], [], [], 'b-', alpha=0.3, linewidth=1)
        
        def animate(frame):
            # Update trail (path so far)
            if frame > 0:
                trail.set_data_3d(self.data['x'][:frame], self.data['y'][:frame], -self.data['z'][:frame])
            
            # Update current position
            if frame < len(self.data):
                point.set_data_3d([self.data['x'].iloc[frame]], 
                                 [self.data['y'].iloc[frame]], 
                                 [-self.data['z'].iloc[frame]])
                
                # Update title with time
                ax.set_title(f'iPhone Trajectory - Time: {self.data["time_rel"].iloc[frame]:.1f}s')
            
            return line, point, trail
        
        # Create animation
        frames = len(self.data)
        anim = animation.FuncAnimation(fig, animate, frames=frames, 
                                     interval=1000//fps, blit=False, repeat=True)
        
        if save_path:
            anim.save(save_path, writer='pillow', fps=fps)
            print(f"Animation saved to {save_path}")
        
        return anim, fig

def main():
    parser = argparse.ArgumentParser(description='iPhone EKF Trajectory Visualizer')
    parser.add_argument('csv_file', help='Path to EKF log CSV file')
    parser.add_argument('--output-dir', default='trajectory_plots', help='Output directory for plots')
    parser.add_argument('--interactive', action='store_true', help='Show interactive plots')
    parser.add_argument('--animate', action='store_true', help='Create animation')
    parser.add_argument('--all', action='store_true', help='Generate all visualizations')
    
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Initialize visualizer
    viz = TrajectoryVisualizer(args.csv_file)
    
    # Generate base filename
    base_name = os.path.splitext(os.path.basename(args.csv_file))[0]
    
    if args.all or not (args.interactive or args.animate):
        # Generate all static plots
        print("Generating 3D trajectory plot...")
        viz.plot_3d_trajectory_matplotlib(
            save_path=os.path.join(args.output_dir, f'{base_name}_3d_trajectory.png'),
            interactive=args.interactive
        )
        
        print("Generating orientation analysis...")
        viz.plot_orientation_analysis(
            save_path=os.path.join(args.output_dir, f'{base_name}_orientation.png')
        )
        
        print("Generating interactive Plotly visualization...")
        viz.create_interactive_plotly(
            save_path=os.path.join(args.output_dir, f'{base_name}_interactive.html')
        )
    
    if args.animate or args.all:
        print("Creating trajectory animation...")
        anim, fig = viz.create_animation(
            save_path=os.path.join(args.output_dir, f'{base_name}_animation.gif')
        )
    
    if args.interactive:
        plt.show()
    
    print(f"\nVisualization complete! Check {args.output_dir}/ for output files.")

if __name__ == "__main__":
    main()
