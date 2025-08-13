"""
Copyright (c) 2025, Thao Dang, Esslingen University.
This file is part of the offline_hand_eye package (see https://github.com/robgineer/cobot).
License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import rclpy
from rclpy.node import Node
from typing import List, Optional
from datetime import datetime
import json
import os
import time

NO_TRACKER_MARKER_STR = "none - compute offline"

class CalibrationGUI:
    def __init__(self, node: Optional[Node] = None):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Hand-Eye Calibration Configuration")
        self.root.geometry("500x550")
        
        self.filepath = None
        
        # Initialize variables for parameters
        self.calibration_type = tk.StringVar(value="eye_in_hand")
        self.camera_image_topic = tk.StringVar()
        self.camera_info_topic = tk.StringVar()
        self.robot_base_frame = tk.StringVar()
        self.robot_effector_frame = tk.StringVar()
        self.tracking_base_frame = tk.StringVar()
        self.tracking_marker_frame = tk.StringVar()
        self.storage_root = tk.StringVar(value="/workspace/data/cobot/calibration")
        
        # Wait a bit for TF data to be available
        if self.node is not None:
            for _ in range(10):
                time.sleep(0.1)
                rclpy.spin_once(node, timeout_sec=1.0)

        # Get available topics and frames
        if self.node is not None:
            self.image_topics_list, self.camera_info_topics_list = node.get_available_topics()
            self.tf_frame_ids = node.get_tf_frames()
        else:
            self.image_topics_list = ['invalid']
            self.camera_info_topics_list = ['invalid']
            self.tf_frame_ids = ['invalid']
        
        self._create_widgets()
        
        # Log initial message
        self.log_message("GUI initialized", "INFO")
        
        # read default parameters from JSON if available
        default_params_path = os.path.join('/workspace', 'handeye_calibration_params.json')
        if os.path.exists(default_params_path):
            self._read_json(default_params_path)
        
    def _get_tf_frames(self) -> List[str]:
        """Get available TF frames from the TF tree"""
        if self.node is not None:
            try:
                # Get all frame IDs 
                tf_frame_ids = self.node.get_tf_frames()
                
            except Exception as e:
                self.log_message(f"Could not get TF frames: {e}", "WARNING")
                # Fallback to common frame names
                tf_frame_ids = ["invalid"]
        else:
            # Default frame names when no node is provided
            tf_frame_ids = ["invalid"]
            
        if (tf_frame_ids == ["invalid"]) or not tf_frame_ids:
            self.log_message(f"Invalid TF frames", "ERROR")
        else:
            self.log_message(f"Found {len(tf_frame_ids)} TF frames", "INFO")
            
        return sorted(tf_frame_ids)
    
    def _create_widgets(self):
        """Create and layout GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        row = 0
        
        # Calibration Type
        ttk.Label(main_frame, text="Calibration Type:").grid(
            row=row, column=0, sticky=tk.W, pady=5)
        
        calibration_combo = ttk.Combobox(
            main_frame, 
            textvariable=self.calibration_type,
            values=["eye_in_hand", "eye_on_base"],
            state="readonly",
            width=30
        )
        calibration_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        row += 1
        
        # Camera Image Topic
        ttk.Label(main_frame, text="Camera Image Topic:").grid(
            row=row, column=0, sticky=tk.W, pady=5)

        self.camera_image_combo = ttk.Combobox(
            main_frame,
            textvariable=self.camera_image_topic,
            values=self.image_topics_list,
            state="readonly",
            width=30
        )
        self.camera_image_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        if self.image_topics_list and "/camera/camera/color/image_raw" in self.image_topics_list:
            self.camera_image_combo.set("/camera/camera/color/image_raw")
        elif self.image_topics_list:
            self.camera_image_combo.set(self.image_topics_list[0])
        row += 1

        # Camera Info Topic
        ttk.Label(main_frame, text="Camera Info Topic:").grid(
            row=row, column=0, sticky=tk.W, pady=5)

        self.camera_info_combo = ttk.Combobox(
            main_frame,
            textvariable=self.camera_info_topic,
            values=self.camera_info_topics_list,
            state="readonly",
            width=30
        )
        self.camera_info_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        if self.camera_info_topics_list and "/camera/camera/color/camera_info" in self.camera_info_topics_list:
            self.camera_info_combo.set("/camera/camera/color/camera_info")
        elif self.camera_info_topics_list:
            self.camera_info_combo.set(self.camera_info_topics_list[0])
        row += 1
        
        # Robot Base Frame
        ttk.Label(main_frame, text="Robot Base Frame:").grid(
            row=row, column=0, sticky=tk.W, pady=5)
        
        robot_base_combo = ttk.Combobox(
            main_frame,
            textvariable=self.robot_base_frame,
            values=self.tf_frame_ids,
            state="readonly",
            width=30
        )
        robot_base_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        if self.tf_frame_ids and "base_link" in self.tf_frame_ids:
            robot_base_combo.set("base_link")
        elif self.tf_frame_ids:
            robot_base_combo.set(self.tf_frame_ids[0])
        row += 1
        
        # Robot Effector Frame
        ttk.Label(main_frame, text="Robot Effector Frame:").grid(
            row=row, column=0, sticky=tk.W, pady=5)
        
        robot_effector_combo = ttk.Combobox(
            main_frame,
            textvariable=self.robot_effector_frame,
            values=self.tf_frame_ids,
            state="readonly",
            width=30
        )
        robot_effector_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        if self.tf_frame_ids and "tool0" in self.tf_frame_ids:
            robot_effector_combo.set("tool0")
        elif self.tf_frame_ids:
            robot_effector_combo.set(self.tf_frame_ids[-1])
        row += 1
        
        # Tracking Base Frame
        ttk.Label(main_frame, text="Tracking Base Frame:").grid(
            row=row, column=0, sticky=tk.W, pady=5)
        
        tracking_base_combo = ttk.Combobox(
            main_frame,
            textvariable=self.tracking_base_frame,
            values=self.tf_frame_ids,
            state="readonly",
            width=30
        )
        tracking_base_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        if self.tf_frame_ids and "camera_link" in self.tf_frame_ids:
            tracking_base_combo.set("camera_link")
        elif self.tf_frame_ids:
            tracking_base_combo.set(self.tf_frame_ids[0])
        row += 1
        
        # Tracking Marker Frame
        ttk.Label(main_frame, text="Tracking Marker Frame:").grid(
            row=row, column=0, sticky=tk.W, pady=5)
        
        marker_frame_values = self.tf_frame_ids + [NO_TRACKER_MARKER_STR]
        tracking_marker_combo = ttk.Combobox(
            main_frame,
            textvariable=self.tracking_marker_frame,
            values=marker_frame_values,
            state="readonly",
            width=30
        )
        tracking_marker_combo.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        tracking_marker_combo.set(NO_TRACKER_MARKER_STR)
        row += 1
        
        # Storage Directory
        ttk.Label(main_frame, text="Recording Directory:").grid(
            row=row, column=0, sticky=tk.W, pady=5)
        
        storage_frame = ttk.Frame(main_frame)
        storage_frame.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        storage_frame.columnconfigure(0, weight=1)
        
        self.storage_entry = ttk.Entry(
            storage_frame,
            textvariable=self.storage_root,
            width=25
        )
        self.storage_entry.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 5))
        
        ttk.Button(
            storage_frame,
            text="Browse",
            command=self._select_storage_directory,
            width=8
        ).grid(row=0, column=1)
        
        row += 1
        
        # Buttons frame - First row
        button_frame1 = ttk.Frame(main_frame)
        button_frame1.grid(row=row, column=0, columnspan=2, pady=(10, 5))
        
        # First row buttons
        ttk.Button(
            button_frame1,
            text="Refresh TF Frames",
            command=self._refresh_tf_frames
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(
            button_frame1,
            text="Start Recording",
            command=self._start_recording
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(
            button_frame1,
            text="Stop Recording",
            command=self._stop_recording
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(
            button_frame1,
            text="Record Single",
            command=self._record_single
        ).pack(side=tk.LEFT, padx=5)        
        
        row += 1
        
        # Buttons frame - Second row
        button_frame2 = ttk.Frame(main_frame)
        button_frame2.grid(row=row, column=0, columnspan=2, pady=(5, 10))
        
        # Second row buttons
        ttk.Button(
            button_frame2,
            text="Save Parameters",
            command=self._save_parameters
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(
            button_frame2,
            text="Load Parameters",
            command=self._load_parameters
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(
            button_frame2,
            text="Close",
            command=self.root.quit
        ).pack(side=tk.LEFT, padx=5)
        
        # Logger frame
        row += 1
        logger_frame = ttk.LabelFrame(main_frame, text="Log Messages", padding="10")
        logger_frame.grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        logger_frame.columnconfigure(0, weight=1)
        logger_frame.rowconfigure(0, weight=1)
        
        self.log_text = tk.Text(logger_frame, height=8, wrap=tk.WORD, state=tk.DISABLED,
                               font=("Courier", 9))
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Scrollbar for log text
        scrollbar = ttk.Scrollbar(logger_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        # Configure text tags for different log levels
        self.log_text.tag_configure("INFO", foreground="black")
        self.log_text.tag_configure("WARNING", foreground="orange")
        self.log_text.tag_configure("ERROR", foreground="red")
        self.log_text.tag_configure("SUCCESS", foreground="green")
        self.log_text.tag_configure("DEBUG", foreground="gray")
    
    def log_message(self, message: str, level: str = "INFO"):
        """Add a message to the log text widget"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, log_entry, level)
        self.log_text.config(state=tk.DISABLED)
        self.log_text.see(tk.END)  # Auto-scroll to bottom
        
        # Also print to console
        print(log_entry.strip())
    
    def _clear_log(self):
        """Clear the log text widget"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log_message("Log cleared", "INFO")
    
    def _select_storage_directory(self):
        """Open directory selection dialog"""
        directory = filedialog.askdirectory(
            title="Select Recording Directory",
            initialdir=self.storage_root.get()
        )
        
        if directory:
            self.storage_root.set(directory)
            self.log_message(f"Storage directory set to: {directory}", "INFO")
        else:
            self.log_message("Directory selection cancelled", "INFO")
    
    def _refresh_tf_frames(self):
        """Refresh the list of available TF frames"""
        self.log_message("Refreshing TF frames...", "INFO")
        self.tf_frame_ids = self._get_tf_frames()
        
        # Update all comboboxes with new frame list
        for widget in self.root.winfo_children():
            for child in widget.winfo_children():
                if isinstance(child, ttk.Combobox) and child['textvariable'] != str(self.calibration_type):
                    if child['textvariable'] == str(self.tracking_marker_frame):
                        child['values'] = self.tf_frame_ids + [NO_TRACKER_MARKER_STR]
                    else:
                        child['values'] = self.tf_frame_ids
        
        self.log_message(f"Found {len(self.tf_frame_ids)} TF frames", "SUCCESS")
    
    def _start_recording(self, single_frame: bool = False):
        """Start recording calibration data"""
        if not single_frame:
            self.log_message("Start recording...", "INFO")
        else:
            self.log_message("Record single frame...", "INFO")
        
        if not self.filepath: 
            # prepare recording and create directory for storing frames 
            self.camera_image_combo.config(state=tk.DISABLED)
            self.camera_info_combo.config(state=tk.DISABLED)
            
            config = self.get_configuration()
            self.node.set_new_subscriptions(config)

            # Use the configured storage directory
            storage_root = self.storage_root.get()
            if not storage_root:
                storage_root = "/workspace/data/cobot/calibration"
                self.log_message("No storage directory set, using default", "WARNING")
            
            self.filepath = datetime.now().strftime(os.path.join(storage_root, "calibdata_%Y_%m_%d-%H_%M_%S"))
            
            # Create directory if it doesn't exist
            try:
                if not os.path.exists(self.filepath):
                    os.makedirs(self.filepath)
            except Exception as e:
                self.log_message(f"Failed to create directory {self.filepath}: {e}", "ERROR")
                return

            self.log_message(f"Recording calibration data to: {self.filepath}", "SUCCESS")

        if not single_frame:
            self.node.start_recording(self.filepath)
        else:
            self.node.single_frame_recording(self.filepath)

    def _stop_recording(self):
        """Stop recording calibration data"""
        self.log_message("Stopped recording...", "INFO")
        self.node.stop_recording()

    def _record_single(self):
        """Record a single frame of calibration data"""
        self._start_recording(single_frame=True)

    def _save_parameters(self):
        """Save current parameters to a JSON file"""
        try:
            # Get current configuration
            config = self.get_configuration()
            
            # Open file dialog
            filename = filedialog.asksaveasfilename(
                title="Save Parameters",
                defaultextension=".json",
                filetypes=[
                    ("JSON files", "*.json"),
                    ("All files", "*.*")
                ],
                initialdir=os.path.expanduser("~"),
                initialfile="handeye_calibration_params.json"
            )
            
            if filename:
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                
                self.log_message(f"Parameters saved to: {filename}", "SUCCESS")
                messagebox.showinfo("Success", f"Parameters saved successfully to:\n{filename}")
            else:
                self.log_message("Save operation cancelled", "INFO")
                
        except Exception as e:
            error_msg = f"Failed to save parameters: {str(e)}"
            self.log_message(error_msg, "ERROR")
            messagebox.showerror("Error", error_msg)
    
    def _read_json(self, filename: str):
        """Read a JSON file and store its contents in the configuration"""
        with open(filename, 'r') as f:
            config = json.load(f)
                
            # Remove metadata if present
            if "_metadata" in config:
                metadata = config.pop("_metadata")
                self.log_message(f"Loading parameters saved at: {metadata.get('saved_at', 'unknown')}", "INFO")
            
            # Validate and load parameters
            self._validate_and_load_config(config)            
            self.log_message(f"Parameters loaded from: {filename}", "SUCCESS")
    
    def _load_parameters(self):
        """Load parameters from a user-selected JSON file"""
        try:
            # Open file dialog
            filename = filedialog.askopenfilename(
                title="Load Parameters",
                filetypes=[
                    ("JSON files", "*.json"),
                    ("All files", "*.*")
                ],
                initialdir=os.path.expanduser("~")
            )
            
            if filename:
                self._read_json(filename)
            else:
                self.log_message("Load operation cancelled", "INFO")
                
        except Exception as e:
            error_msg = f"Failed to load parameters: {str(e)}"
            self.log_message(error_msg, "ERROR")
            messagebox.showerror("Error", error_msg)
    
    def _validate_and_load_config(self, config: dict):
        """Validate and load configuration parameters"""
        # Define expected parameters
        expected_params = {
            "calibration_type": self.calibration_type,
            "camera_image_topic": self.camera_image_topic,
            "camera_info_topic": self.camera_info_topic,
            "robot_base_frame": self.robot_base_frame,
            "robot_effector_frame": self.robot_effector_frame,
            "tracking_base_frame": self.tracking_base_frame,
            "tracking_marker_frame": self.tracking_marker_frame,
            "storage_root": self.storage_root
        }
        
        # Load each parameter if it exists and is valid
        for param_name, param_var in expected_params.items():
            if param_name in config:
                value = config[param_name]
                
                # Validate calibration_type
                if param_name == "calibration_type":
                    if value in ["eye_in_hand", "eye_on_base"]:
                        param_var.set(value)
                        self.log_message(f"Loaded {param_name}: {value}", "INFO")
                    else:
                        self.log_message(f"Invalid calibration_type: {value}, using default", "WARNING")
                
                # Validate storage_root (directory path)
                elif param_name == "storage_root":
                    if os.path.isdir(value) or not os.path.exists(value):  # Allow non-existing dirs (can be created)
                        param_var.set(value)
                        self.log_message(f"Loaded {param_name}: {value}", "INFO")
                    else:
                        self.log_message(f"Invalid storage directory: {value}, using default", "WARNING")
                
                # Validate topic names
                elif param_name in ["camera_image_topic"]:
                    if value in self.image_topics_list:
                        param_var.set(value)
                        self.log_message(f"Loaded {param_name}: {value}", "INFO")
                    else:
                        self.log_message(f"Topic '{value}' not available, parameter not loaded", "WARNING")    

                elif param_name in ["camera_info_topic"]:
                    if value in self.camera_info_topics_list:
                        param_var.set(value)
                        self.log_message(f"Loaded {param_name}: {value}", "INFO")
                    else:
                        self.log_message(f"Topic '{value}' not available, parameter not loaded", "WARNING") 
                                        
                # Validate frame parameters
                elif param_name.endswith("_frame"):
                    if param_name == "tracking_marker_frame":
                        # Special case for tracking_marker_frame (can include NO_TRACKER_MARKER_STR)
                        valid_values = self.tf_frame_ids + [NO_TRACKER_MARKER_STR]
                    else:
                        valid_values = self.tf_frame_ids
                    
                    if value in valid_values or not self.tf_frame_ids:  # Allow any value if TF frames not available
                        param_var.set(value)
                        self.log_message(f"Loaded {param_name}: {value}", "INFO")
                    else:
                        self.log_message(f"Frame '{value}' not available, parameter not loaded", "WARNING")
                
                else:
                    param_var.set(value)
                    self.log_message(f"Loaded {param_name}: {value}", "INFO")
            else:
                self.log_message(f"Parameter '{param_name}' not found in file", "WARNING")
    
    def get_configuration(self) -> dict:
        """Get the current configuration as a dictionary"""
        return {
            "calibration_type": self.calibration_type.get(),
            "camera_image_topic": self.camera_image_topic.get(),
            "camera_info_topic": self.camera_info_topic.get(),
            "robot_base_frame": self.robot_base_frame.get(),
            "robot_effector_frame": self.robot_effector_frame.get(),
            "tracking_base_frame": self.tracking_base_frame.get(),
            "tracking_marker_frame": self.tracking_marker_frame.get(),
            "storage_root": self.storage_root.get()
        }
    
    def spin_once(self, timeout_sec: float = 0.1, period: float = 0.2):
        """Spin the ROS node once to process callbacks"""
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
            self.root.after(int(period * 1000), self.spin_once, timeout_sec, period)
            
    def run(self):
        """Start the GUI main loop"""
        self.spin_once()
        self.root.mainloop()
