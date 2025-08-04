#!/usr/bin/env python3
"""
Interactive Frame Viewer GUI for Hand-Eye Calibration
======================================================

This script provides an interactive matplotlib GUI for viewing and selecting
frames for hand-eye calibration. It allows users to navigate through frames,
view AprilTag detections, and select frames for calibration.

Features:
- Range slider for frame navigation
- Previous/Next buttons for step-by-step navigation
- Frame selection/deselection with visual feedback
- Real-time AprilTag detection display
- Run calibration

Usage:
    ./offline_hand_eye_calibration_gui.py --data_path <path> --tagsize <size> --apriltag_family <family> --config <config_file> --output <output_file>
    
e.g.:
    ./offline_hand_eye_calibration_gui.py --data_path ~/data/cobot/calibration/calibdata_2025_08_03-17_58_51 --tagsize 0.074 --config ../../../handeye_calibration_params.json --output ../../../handeye_calibration.json
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider, Button
import os
import glob
import sys
import json
import argparse
from apriltag import apriltag

# Add the offline_hand_eye directory to the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../offline_hand_eye")))
from calib_io_utils import save_calibration, load_calibration
from calibration_utils import load_and_detect, show_detections, compute_hand_eye_calibration


class FrameViewer:
    """
    Interactive Frame Viewer for Hand-Eye Calibration
    
    This class creates a matplotlib-based GUI for viewing and selecting frames
    for hand-eye calibration.
    """
    
    def __init__(self, data_path, detector, apriltag_family, max_frames, calibration_config, calibration_output, tagsize=0.074):
        """
        Initialize the Frame Viewer
        
        Args:
            data_path: Path to the calibration data directory
            detector: AprilTag detector object
            apriltag_family: AprilTag family name
            max_frames: Maximum number of frames available
            calibration_config: Path to the calibration config file
            calibration_output: Path to the calibration output file
            tagsize: Size of the AprilTag in meters (default: 0.074)
        """
        self.data_path = data_path
        self.detector = detector
        self.apriltag_family = apriltag_family
        self.max_frames = max_frames
        self.current_frame = 0
        self.selected_frames = set()
        self.calibration_config_file = calibration_config
        self.calibration_output_file = calibration_output
        self.tagsize = tagsize
        self.is_calibrated = False

        # Create the main figure and layout
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('Interactive Frame Viewer for Hand-Eye Calibration', fontsize=16)
        
        # Main image axis
        self.ax = plt.subplot2grid((4, 4), (0, 0), colspan=4, rowspan=3)
        
        # Control panel area
        self.setup_controls()
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Initial display
        self.update_display()
        
        print("Frame Viewer Controls:")
        print("- Use slider or arrow keys to navigate")
        print("- Space: Select/Unselect current frame")
        print("- 'q': Quit")
        print("- 's': Show selected frames")
    
    def setup_controls(self):
        """Setup all GUI controls"""
        # Frame slider
        slider_ax = plt.axes([0.1, 0.15, 0.65, 0.03])
        self.frame_slider = Slider(
            slider_ax, 'Frame', 0, self.max_frames-1, 
            valinit=0, valfmt='%d', valstep=1
        )
        self.frame_slider.on_changed(self.update_frame_from_slider)
        
        # Navigation buttons
        prev_ax = plt.axes([0.1, 0.08, 0.08, 0.04])
        self.prev_button = Button(prev_ax, '◀ Prev')
        self.prev_button.on_clicked(self.prev_frame)
        
        next_ax = plt.axes([0.2, 0.08, 0.08, 0.04])
        self.next_button = Button(next_ax, 'Next ▶')
        self.next_button.on_clicked(self.next_frame)
        
        # Selection button
        select_ax = plt.axes([0.3, 0.08, 0.12, 0.04])
        self.select_button = Button(select_ax, 'Select/Unselect')
        self.select_button.on_clicked(self.toggle_frame_selection)
        
        # Show selected button
        show_selected_ax = plt.axes([0.44, 0.08, 0.12, 0.04])
        self.show_selected_button = Button(show_selected_ax, 'Show Selected')
        self.show_selected_button.on_clicked(self.show_selected_frames)
        
        # Clear selection button
        clear_ax = plt.axes([0.58, 0.08, 0.12, 0.04])
        self.clear_button = Button(clear_ax, 'Clear All')
        self.clear_button.on_clicked(self.clear_selection)
        
        # Run button
        run_ax = plt.axes([0.72, 0.08, 0.12, 0.04])
        self.run_button = Button(run_ax, 'Run Calibration')
        self.run_button.on_clicked(self.run_calibration)

    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == 'left' or event.key == 'a':
            self.prev_frame(None)
        elif event.key == 'right' or event.key == 'd':
            self.next_frame(None)
        elif event.key == ' ':  # Space bar
            self.toggle_frame_selection(None)
        elif event.key == 's':
            self.show_selected_frames(None)
        elif event.key == 'q':
            plt.close(self.fig)
    
    def update_frame_from_slider(self, val):
        """Update frame based on slider value"""
        self.current_frame = int(self.frame_slider.val)
        self.update_display()
    
    def prev_frame(self, event):
        """Navigate to previous frame"""
        if self.current_frame > 0:
            self.current_frame -= 1
            self.frame_slider.set_val(self.current_frame)
            self.update_display()
    
    def next_frame(self, event):
        """Navigate to next frame"""
        if self.current_frame < self.max_frames - 1:
            self.current_frame += 1
            self.frame_slider.set_val(self.current_frame)
            self.update_display()
    
    def toggle_frame_selection(self, event):
        """Toggle selection of current frame"""
        if self.current_frame in self.selected_frames:
            self.selected_frames.remove(self.current_frame)
            print(f"Frame {self.current_frame} deselected")
        else:
            self.selected_frames.add(self.current_frame)
            print(f"Frame {self.current_frame} selected")
        self.update_display()
    
    def show_selected_frames(self, event):
        """Display selected frames"""
        selected_list = sorted(list(self.selected_frames))
        print(f"\nSelected frames ({len(selected_list)}): {selected_list}")
        return selected_list
    
    def clear_selection(self, event):
        """Clear all selected frames"""
        self.selected_frames.clear()
        print("All selections cleared")
        self.update_display()
    
    def run_calibration(self, event):
        """Run calibration on selected frames"""
        selected_list = sorted(list(self.selected_frames))
        if not selected_list:
            print("Error: No frames selected for calibration")
            return
        if len(selected_list) < 2:
            print("Error: At least 2 frames are required for calibration")
            return

        hand_camera_rot, hand_camera_tr, hand_camera_qwxyz = \
            compute_hand_eye_calibration(self.data_path, selected_list, self.detector, self.tagsize)

        print("Hand-Eye Calibration Results:")
        print("Rotation Matrix:")
        print(hand_camera_rot)
        print("Quaternion (wxyz):")
        print(hand_camera_qwxyz)
        print("Translation Vector:")
        print(hand_camera_tr)
        
        # Save all relevant calibration data
        with open(self.calibration_config_file, 'r') as f:
            calibration_config = json.load(f)

        save_calibration(self.calibration_output_file, calibration_config, hand_camera_qwxyz.tolist(), hand_camera_tr.tolist(),
                         selected_list, self.data_path)
        print(f"Written calibration results to: {self.calibration_output_file}")
        self.is_calibrated = True
        self.update_display()

    def update_display(self):
        """Update the main display with current frame"""
        self.ax.clear()
        
        try:
            # Load and detect frame
            frame, gray, detections = load_and_detect(self.current_frame, self.data_path, self.detector)
            
            # Display the image with detections
            plt.sca(self.ax)
            self.ax.clear()
            show_detections(gray, detections, self.apriltag_family, show_legend=False, show_family=False)

            # Update title with selection status
            selection_status = " ✓ SELECTED" if self.current_frame in self.selected_frames else ""
            detection_info = f" | {len(detections)} tags detected" if detections else " | No tags detected"
            
            self.ax.set_title(
                f'Frame {self.current_frame}{selection_status}{detection_info}', 
                fontsize=14, 
                color='green' if self.current_frame in self.selected_frames else 'black'
            )
            
            # Add colored border if frame is selected
            if self.current_frame in self.selected_frames:
                rect = patches.Rectangle((0, 0), 1, 1, linewidth=4, 
                                       edgecolor='green', facecolor='none', 
                                       transform=self.ax.transAxes)
                self.ax.add_patch(rect)
            
        except Exception as e:
            self.ax.text(0.5, 0.5, f'Error loading frame {self.current_frame}:\n{str(e)}', 
                        ha='center', va='center', transform=self.ax.transAxes,
                        bbox=dict(boxstyle="round,pad=0.5", facecolor="red", alpha=0.7))
            self.ax.set_title(f'Frame {self.current_frame} (ERROR)', color='red')
        
        # Update info text
        if not self.is_calibrated:
            info_text = f"Selected: {len(self.selected_frames)} frames | Current: {self.current_frame}/{self.max_frames-1}"
            self.fig.suptitle(f'Interactive Frame Viewer | {info_text}', fontsize=14)
        else:
            info_text = f"Selected: {len(self.selected_frames)} frames | Current: {self.current_frame}/{self.max_frames-1}"
            self.fig.suptitle(f'Interactive Frame Viewer | {info_text} | CALIBRATED', fontsize=14, color='green')

        plt.draw()
    
    def get_selected_frames(self):
        """Return the list of selected frames"""
        return sorted(list(self.selected_frames))


def main():
    """Main function to run the Frame Viewer GUI"""
    parser = argparse.ArgumentParser(description='Interactive Frame Viewer for Hand-Eye Calibration')
    parser.add_argument('--data_path', '-d', 
                       help='Path to calibration data directory')
    parser.add_argument('--tagsize', '-t', type=float, default=0.074,
                       help='Size of the AprilTag in meters (default: 0.074)')
    parser.add_argument('--apriltag_family', '-f', default='tagStandard41h12',
                       help='AprilTag family (default: tagStandard41h12)')
    parser.add_argument('--config', '-c', default='handeye_calibration_params.json',
                       help='Path to the calibration config file (default: handeye_calibration_params.json)')
    parser.add_argument('--output', '-o', default='handeye_calibration.json',
                       help='Path to save the calibration results (default: handeye_calibration.json)')
    
    # calibration_config_file = '../../../handeye_calibration_params.json'
    # calibration_output_file = '../../../handeye_calibration.json'
    
    args = parser.parse_args()
    
    # Use default data path if not provided
    if args.data_path is None:
        print(f"\nError: No data path provided.")
        print("Please provide a valid data directory path using --data_path")
        return
    
    # Check if data directory exists
    if not os.path.exists(args.data_path):
        print(f"Error: Data directory not found: {args.data_path}")
        print("Please provide a valid data directory path using --data_path")
        return
    
    num_frames = len(glob.glob1(args.data_path, "frame_*.pkl"))
    if num_frames == 0:
        print(f"No frames found in directory: {args.data_path}")
        print("Please ensure the directory contains frame_*.pkl files")
        return

    # Check calibration config file
    if not os.path.isfile(args.config):
        print(f"Error: Calibration config file not found: {args.config}")
        print("Please provide a valid calibration config file path using --config")
        return
    
    print(f"Loading data from: {args.data_path}")
    print(f"Using calibration config: {args.config}")
    print(f"Saving calibration results to: {args.output}")
    print(f"AprilTag size: {args.tagsize} m")
    print(f"AprilTag family: {args.apriltag_family}")
    print(f"Num. frames: {num_frames}")

    # Initialize AprilTag detector
    detector = apriltag(args.apriltag_family)
    
    # Create and show the GUI
    viewer = FrameViewer(data_path=args.data_path, 
                         detector=detector, 
                         apriltag_family=args.apriltag_family, 
                         max_frames=num_frames,
                         calibration_config=args.config,
                         calibration_output=args.output, 
                         tagsize=args.tagsize)

    try:
        plt.show()
        
        # After GUI is closed, show final selection
        selected = viewer.get_selected_frames()
        if selected:
            print(f"\nFinal selection: {selected}")
        else:
            print("\nNo frames were selected.")
            
    except KeyboardInterrupt:
        print("\nGUI closed by user")


if __name__ == "__main__":
    main()
