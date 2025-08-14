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
    ./offline_hand_eye_calibration_gui.py --data_path <path> --config <config_file> --output <output_file>
    
e.g.:
    ./offline_hand_eye_calibration_gui.py --data_path ~/data/cobot/calibration/calibdata_2025_08_03-17_58_51 --config ../../../handeye_calibration_params.json --output ../../../handeye_calibration.json

Copyright (c) 2025, Thao Dang, Esslingen University.
This file is part of the offline_hand_eye package (see https://github.com/robgineer/cobot).
License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider, Button, TextBox
import os
import glob
import sys
import json
import argparse
from apriltag import apriltag

# Add the offline_hand_eye directory to the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../offline_hand_eye")))
from calib_io_utils import save_calibration, load_calibration
from calibration_utils import load_and_detect, show_detections, compute_hand_eye_calibration, frame_is_valid, compute_TCP_image_position, compute_target_to_gripper_transform, compute_target_image_position, compute_reprojection_error_mean_max


class FrameViewer:
    """
    Interactive Frame Viewer for Hand-Eye Calibration
    
    This class creates a matplotlib-based GUI for viewing and selecting frames
    for hand-eye calibration.
    """
    
    def __init__(self, data_path, max_frames, calibration_config, calibration_output):
        """
        Initialize the Frame Viewer
        
        Args:
            data_path: Path to the calibration data directory
            max_frames: Maximum number of frames available
            calibration_config: Path to the calibration config file
            calibration_output: Path to the calibration output file
        """
        self.data_path = data_path
        self.max_frames = max_frames
        self.current_frame = 0
        self.selected_frames = set()
        self.calibration_config_file = calibration_config
        self.calibration_output_file = calibration_output
        self.tagsize = 0.14
        self.is_calibrated = False
        self.hand_camera_rot, self.hand_camera_tr, self.hand_camera_qwxyz = None, None, None
        self.rvec_target_to_gripper_rot, self.tvec_target_to_gripper_tr = None, None

        # Available AprilTag families
        self.apriltag_families = ['tag16h5', 'tag25h9', 'tag36h11', 'tagCircle21h7', 'tagCircle49h12', 
                                  'tagCustom48h12', 'tagStandard41h12', 'tagStandard52h13']

        # Initialize AprilTag detector
        self.current_family_index = 2
        self.apriltag_family = self.apriltag_families[self.current_family_index]
        self.detector = apriltag(self.apriltag_family)

        # Available calibration methods
        self.calibration_methods = ['TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS']
        self.current_method_index = 1

        # Create the main figure and layout
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('Interactive Frame Viewer for Hand-Eye Calibration', fontsize=16)
        
        # Main image axis
        self.ax = plt.subplot2grid((4, 6), (0, 0), colspan=4, rowspan=3)
        
        # Text info axis for robot transform data
        self.info_ax = plt.subplot2grid((4, 6), (0, 4), colspan=2, rowspan=3)
        self.info_ax.axis('off')  # Hide axis for text display
        
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
        print("- Edit tag size in the text box and press Enter")
        print("- Click 'Family' button to cycle through AprilTag families")
        print("- Click 'Update Detector' to apply new family setting")
    
    def setup_controls(self):
        """Setup all GUI controls"""
        # Frame slider
        slider_ax = plt.axes([0.1, 0.22, 0.65, 0.03])
        self.frame_slider = Slider(
            slider_ax, 'Frame', 0, self.max_frames-1, 
            valinit=0, valfmt='%d', valstep=1
        )
        self.frame_slider.on_changed(self.update_frame_from_slider)
        
        # AprilTag size text box
        tagsize_ax = plt.axes([0.1, 0.15, 0.08, 0.03])
        self.tagsize_textbox = TextBox(tagsize_ax, 'Tag Size (m): ', initial=str(self.tagsize))
        self.tagsize_textbox.on_submit(self.update_tagsize)
        
        # AprilTag family dropdown (implemented as cycling button)
        family_ax = plt.axes([0.2, 0.15, 0.22, 0.03])
        self.family_button = Button(family_ax, f'Family: {self.apriltag_families[self.current_family_index]}')
        self.family_button.on_clicked(self.cycle_apriltag_family)
        
        # Update detector button
        update_detector_ax = plt.axes([0.44, 0.15, 0.12, 0.03])
        self.update_detector_button = Button(update_detector_ax, 'Update Detector')
        self.update_detector_button.on_clicked(self.update_detector)

        # Calibration method dropdown (implemented as cycling button)
        method_ax = plt.axes([0.58, 0.15, 0.26, 0.03])
        self.method_button = Button(method_ax, f'Calib Method: {self.calibration_methods[self.current_method_index]}')
        self.method_button.on_clicked(self.cycle_calibration_method)

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
        self.clear_button = Button(clear_ax, 'Clear All/Select All')
        self.clear_button.on_clicked(self.clear_select_all)
        
        # Run button
        run_ax = plt.axes([0.72, 0.08, 0.12, 0.04])
        self.run_button = Button(run_ax, 'Run Calibration')
        self.run_button.on_clicked(self.run_calibration)

    def update_tagsize(self, text):
        """Update AprilTag size from text input"""
        try:
            new_tagsize = float(text)
            if new_tagsize > 0:
                self.tagsize = new_tagsize
                print(f"AprilTag size updated to: {self.tagsize} m")
            else:
                print("Error: Tag size must be positive")
                self.tagsize_textbox.set_val(str(self.tagsize))
        except ValueError:
            print("Error: Invalid tag size format")
            self.tagsize_textbox.set_val(str(self.tagsize))
    
    def cycle_apriltag_family(self, event):
        """Cycle through available AprilTag families"""
        self.current_family_index = (self.current_family_index + 1) % len(self.apriltag_families)
        new_family = self.apriltag_families[self.current_family_index]
        self.family_button.label.set_text(f'Family: {new_family}')
        print(f"AprilTag family changed to: {new_family}")
        plt.draw()

    def cycle_calibration_method(self, event):
        """Cycle through available calibration methods"""
        self.current_method_index = (self.current_method_index + 1) % len(self.calibration_methods)
        new_method = self.calibration_methods[self.current_method_index]
        self.method_button.label.set_text(f'Calib Method: {new_method}')
        print(f"Calibration method changed to: {new_method}")
        plt.draw()    
    
    def update_detector(self, event):
        """Update the AprilTag detector with new family"""
        new_family = self.apriltag_families[self.current_family_index]
        if new_family != self.apriltag_family:
            try:
                self.detector = apriltag(new_family)
                self.apriltag_family = new_family
                self.update_display()  # Refresh display with new detector
                print(f"Detector updated to family: {new_family}")
                
            except Exception as e:
                print(f"Error updating detector: {e}")
                # Reset to previous family on error
                if self.apriltag_family in self.apriltag_families:
                    self.current_family_index = self.apriltag_families.index(self.apriltag_family)
                    self.family_button.label.set_text(f'Family: {self.apriltag_family}')
                    plt.draw()
    
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
    
    def clear_select_all(self, event):
        """Clear all selected frames"""
        if not self.selected_frames:
            self.selected_frames = sorted(list(range(self.max_frames)))
            print("All frames selected")
        else:
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

        print(f"Running calibration with:")
        print(f"  - Tag size: {self.tagsize} m")
        print(f"  - AprilTag family: {self.apriltag_family}")
        print(f"  - Selected frames: {selected_list}")
        print(f"  - Method: {self.calibration_methods[self.current_method_index]}")

        self.hand_camera_rot, self.hand_camera_tr, self.hand_camera_qwxyz = \
            compute_hand_eye_calibration(self.data_path, selected_list, self.detector, self.tagsize, 
                                         method_str=self.calibration_methods[self.current_method_index])

        print("Hand-Eye Calibration Results:")
        print("Rotation Matrix:")
        print(self.hand_camera_rot)
        print("Quaternion (wxyz):")
        print(self.hand_camera_qwxyz)
        print("Translation Vector:")
        print(self.hand_camera_tr)
        
        # Compute target to gripper transformation (needed for reprojection error analysis)
        self.rvec_target_to_gripper_rot, self.tvec_target_to_gripper_tr = \
            compute_target_to_gripper_transform(self.hand_camera_rot, self.hand_camera_tr, self.data_path, selected_list, self.detector, self.tagsize)

        # Compute overall reprojection error
        mean_error, max_error = compute_reprojection_error_mean_max(self.hand_camera_rot, self.hand_camera_tr, self.data_path, selected_list, self.detector, 
                                                                    self.rvec_target_to_gripper_rot, self.tvec_target_to_gripper_tr, self.tagsize)
        print(f'Overall reprojection error: mean={mean_error:.2f}px, max={max_error:.2f}px')

        # Save all relevant calibration data
        with open(self.calibration_config_file, 'r') as f:
            calibration_config = json.load(f)

        save_calibration(self.calibration_output_file, calibration_config, self.hand_camera_qwxyz.tolist(), self.hand_camera_tr.tolist(),
                         selected_list, self.data_path)
        print(f"Written calibration results to: {self.calibration_output_file}")
        self.is_calibrated = True
        self.update_display()

    def update_display(self):
        """Update the main display with current frame"""
        self.ax.clear()
        self.info_ax.clear()
        self.info_ax.axis('off')
        
        try:
            # Load and detect frame
            frame, gray, detections = load_and_detect(self.current_frame, self.data_path, self.detector)
            valid_frame = frame_is_valid(frame, detections)
            if not valid_frame:
                self.select_button.active = False  # Disable select button if frame is invalid
            else:
                self.select_button.active = True

            # Display the image with detections
            plt.sca(self.ax)
            self.ax.clear()
            show_detections(gray, detections, self.apriltag_family, show_legend=False, show_family=False)
            
            if self.is_calibrated:
                img_points, _ = compute_TCP_image_position(frame, self.hand_camera_rot, self.hand_camera_tr)
                plt.plot(img_points[0], img_points[1], 'g*', ms=12, label='TCP Projection')
                
                xc_proj = compute_target_image_position(frame, self.hand_camera_rot, self.hand_camera_tr, self.rvec_target_to_gripper_rot, self.tvec_target_to_gripper_tr, self.tagsize)
                plt.plot(xc_proj[:,0], xc_proj[:,1], 'c.', ms=12, label='Target Projection')
                plt.legend()                

            # Display robot transform information in the info panel
            self.display_robot_transform_info(frame, detections)

            # Update title with selection status
            selection_status = " ✓ SELECTED" if self.current_frame in self.selected_frames else ""
            detection_info = f" | {len(detections)} tags detected" if detections else " | No tags detected"
            
            if not valid_frame:
                color = 'red'
            elif self.current_frame in self.selected_frames:
                color = 'green'
            else:
                color = 'black'
            self.ax.set_title(
                f'Frame {self.current_frame}{selection_status}{detection_info}', 
                fontsize=14, 
                color=color
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
            
            # Show error in info panel as well
            self.info_ax.text(0.5, 0.5, 'No transform data\navailable', 
                             ha='center', va='center', transform=self.info_ax.transAxes,
                             fontsize=12, color='red')
        
        # Update info text
        if not self.is_calibrated:
            info_text = f"Selected: {len(self.selected_frames)} frames | Current: {self.current_frame}/{self.max_frames-1}"
            self.fig.suptitle(f'Interactive Frame Viewer | {info_text}', fontsize=14)
        else:
            info_text = f"Selected: {len(self.selected_frames)} frames | Current: {self.current_frame}/{self.max_frames-1}"
            self.fig.suptitle(f'Interactive Frame Viewer | {info_text} | CALIBRATED', fontsize=14, color='green')

        plt.draw()
    
    def display_robot_transform_info(self, frame, detections):
        """Display robot transform information in the info panel"""
        try:
            # Extract robot transform data
            robot_transform = frame.get('robot_transform', {})
            rotation = robot_transform.get('rotation', None)
            translation = robot_transform.get('translation', None)
            
            # Format and display the information
            info_text = f"Frame {self.current_frame}\n"
            info_text += "=" * 35 + "\n\n"
            
            if rotation is not None:
                info_text += "Rotation wxyz:\n"
                info_text += f"  {rotation.get('w', None):.6f}\n"
                info_text += f"  {rotation.get('x', None):.6f}\n"
                info_text += f"  {rotation.get('y', None):.6f}\n"
                info_text += f"  {rotation.get('z', None):.6f}\n"
            else:
                info_text += "Rotation wxyz: None\n"
            
            info_text += "\n"
            
            if translation is not None:
                info_text += "Translation xyz:\n"
                info_text += f"  {translation.get('x', None):.6f}\n"
                info_text += f"  {translation.get('y', None):.6f}\n"
                info_text += f"  {translation.get('z', None):.6f}\n"
            else:
                info_text += "Translation xyz: None\n"
                
            if detections is not None and len(detections) > 0:
                info_text += "\nAprilTag Detections:\n"
                for det in detections:
                    info_text += f"  ID: {det['id']},\n  Center: {det['center'][0]:.1f}, {det['center'][1]:.1f}\n"
            else:
                info_text += "\nNo AprilTag detections found\n"
                
            # Add timestamp if available
            timestamp = frame.get('timestamp', None)
            if timestamp is not None:
                info_text += f"\nTimestamp: {timestamp}"
            
            # Display the text
            self.info_ax.text(0.05, 1.0, info_text, 
                             ha='left', va='top', transform=self.info_ax.transAxes,
                             fontsize=10, fontfamily='monospace',
                             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
                        
        except Exception as e:
            # If there's an error extracting transform data, show a message
            error_text = f"Error reading transform data:\n{str(e)}"
            self.info_ax.text(0.5, 0.5, error_text, 
                             ha='center', va='center', transform=self.info_ax.transAxes,
                             fontsize=10, color='red',
                             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.8))
    
    def get_selected_frames(self):
        """Return the list of selected frames"""
        return sorted(list(self.selected_frames))


def main():
    """Main function to run the Frame Viewer GUI"""
    parser = argparse.ArgumentParser(description='Interactive Frame Viewer for Hand-Eye Calibration')
    parser.add_argument('--data_path', '-d', 
                       help='Path to calibration data directory')
    parser.add_argument('--config', '-c', default='handeye_calibration_params.json',
                       help='Path to the calibration config file (default: handeye_calibration_params.json)')
    parser.add_argument('--output', '-o', default='handeye_calibration.json',
                       help='Path to save the calibration results (default: handeye_calibration.json)')
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
    print(f"Num. frames: {num_frames}")
    
    # Create and show the GUI
    viewer = FrameViewer(data_path=args.data_path, 
                         max_frames=num_frames,
                         calibration_config=args.config,
                         calibration_output=args.output)

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
