"""Real-time 3D trajectory visualizer for the DeepVO pipeline.

This script monitors a CSV file containing camera poses and updates a 3D plot
in real-time. Upon closing the viewer, it automatically saves a high-resolution 
static image of the final trajectory to the same directory.
"""

import os
import sys
from typing import Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import pandas as pd


class TrajectoryVisualizer3D:
    """Visualizes camera trajectory in 3D real-time and saves the final plot.

    Attributes:
        csv_file: Path to the CSV file containing trajectory data (x, y, z).
        output_image: Path where the final static plot will be saved.
        fig: Matplotlib figure instance.
        ax: Matplotlib 3D axes instance.
        line: Matplotlib line instance for the trajectory.
    """

    def __init__(self, csv_file: str) -> None:
        """Initializes the visualizer with the target CSV file.

        Args:
            csv_file: The path to the trajectory CSV file.
        """
        self.csv_file = csv_file
        
        # Construct output image path based on the CSV path
        base_dir = os.path.dirname(csv_file)
        base_name = os.path.splitext(os.path.basename(csv_file))[0]
        self.output_image = os.path.join(base_dir, f"{base_name}_plot.png")

        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize an empty line object
        self.line, = self.ax.plot([], [], [], 'b-', linewidth=2, label='Camera Path')
        self.ax.set_title('DeepVO Real-Time 3D Trajectory')
        self.ax.set_xlabel('X Axis')
        self.ax.set_ylabel('Y Axis')
        self.ax.set_zlabel('Z Axis')
        self.ax.legend()

    def update_plot(self, frame: int) -> Tuple[Line2D]:
        """Reads the latest CSV data and dynamically updates the 3D plot.

        Args:
            frame: The current animation frame index provided by FuncAnimation.

        Returns:
            A tuple containing the updated Matplotlib line object.
        """
        if not os.path.exists(self.csv_file):
            return self.line,
            
        try:
            # Read the CSV dynamically
            df = pd.read_csv(self.csv_file)
            if len(df) < 2:
                return self.line,
                
            x = df['x'].values
            y = df['y'].values
            z = df['z'].values
            
            # Update the line data
            self.line.set_data(x, y)
            self.line.set_3d_properties(z)
            
            # Dynamically adjust the axes limits to follow the camera
            self.ax.set_xlim(x.min() - 1.0, x.max() + 1.0)
            self.ax.set_ylim(y.min() - 1.0, y.max() + 1.0)
            self.ax.set_zlim(z.min() - 1.0, z.max() + 1.0)
            
        except Exception:
            # Handle potential file-lock collisions gracefully during active writing
            pass
            
        return self.line,

    def save_final_plot(self) -> None:
        """Saves a high-resolution static plot of the final trajectory."""
        if not os.path.exists(self.csv_file):
            return

        print("[Python Viewer] Generating final static plot...")
        try:
            df = pd.read_csv(self.csv_file)
            if len(df) < 2:
                print("[Python Viewer] Not enough data to save a plot.")
                return

            # Create a clean, new figure for the static export
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')

            x = df['x'].values
            y = df['y'].values
            z = df['z'].values

            # Plot the path and highlight start/end positions
            ax.plot(x, y, z, 'b-', linewidth=2, label='Camera Path')
            ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')
            ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')

            ax.set_title('DeepVO Final 3D Trajectory')
            ax.set_xlabel('X Axis')
            ax.set_ylabel('Y Axis')
            ax.set_zlabel('Z Axis')
            ax.legend()
            ax.grid(True)
            
            # Set an optimal viewing angle
            ax.view_init(elev=30, azim=-45)

            # Export to the targeted directory
            fig.savefig(self.output_image, dpi=300, bbox_inches='tight')
            print(f"[Python Viewer] Successfully saved trajectory plot to: {self.output_image}")
            plt.close(fig)

        except Exception as e:
            print(f"[Python Viewer] Failed to save final plot: {e}")

    def run(self) -> None:
        """Starts the real-time animation loop and saves the plot upon exit."""
        # Keep a reference to the animation object to prevent garbage collection
        _ani = FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False
        )
        # plt.show() blocks execution until the window is closed
        plt.show() 
        
        # Trigger the export immediately after the user closes the real-time window
        self.save_final_plot()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python viewer_3d.py <path_to_trajectory_csv>")
        sys.exit(1)
        
    target_csv_path = sys.argv[1]
    
    print(f"[Python Viewer] Monitoring trajectory file: {target_csv_path}")
    
    visualizer = TrajectoryVisualizer3D(target_csv_path)
    visualizer.run()