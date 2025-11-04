import customtkinter as ctk
import tkinter as tk
import math
import random
from PIL import Image, ImageTk # For more advanced image handling if needed
from datetime import datetime

# --- IMPORT: Add imports for networking and threading ---
import socket
import threading
import queue
import time

# --- Configuration ---
# *** UDP Configuration is now active ***
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
PACKET_FORMAT = "roll,pitch,z_accel,throttle,b1,b2,b3,b4,b5,b6"

# --- Appearance Settings ---
BG_COLOR = "#222222" # Darker background
PANEL_COLOR = "#2c2c2c" # Slightly lighter for panels
ACCENT_COLOR = "#4a4a4a" # For borders, inactive elements
ACTIVE_COLOR = "#00ff00" # Bright green for active
WARNING_COLOR = "#ffcc00" # Amber for warnings
CRITICAL_COLOR = "#ff0000" # Red for critical
TEXT_COLOR_PRIMARY = "#e0e0e0" # Light grey
TEXT_COLOR_HIGHLIGHT = "#00ffff" # Cyan for primary readouts

# *** FIX: Added explicit hover colors to avoid alpha channel errors ***
ACCENT_COLOR_HOVER = "#5a5a5a"
ACTIVE_COLOR_HOVER = "#00cc00"

FONT_FAMILY = "Segoe UI" # A clean, modern font
LARGE_FONT = (FONT_FAMILY, 24, "bold")
MEDIUM_FONT = (FONT_FAMILY, 16, "bold")
SMALL_FONT = (FONT_FAMILY, 12)
BUTTON_FONT = (FONT_FAMILY, 12, "bold") # Bolder font for button names
SWITCH_FONT = (FONT_FAMILY, 10, "bold") # Font for the "ON"/"OFF" part

CONSOLE_FONT = ("Courier New", 12, "bold")
CONSOLE_TEXT_COLOR = "#00ff00" # Bright green
CONSOLE_BG_COLOR = "#000000" # Pure black
CONSOLE_PROMPT = "RECV> "

BUTTON_NAMES = [
    "LANDING GEAR", "AUTOPILOT", "FLAPS UP/DN",
    "NAV LIGHTS", "STALL WARN", "ENGINE START"
]

# --- Custom ToggleSwitch Widget ---
class ToggleSwitch(ctk.CTkFrame):
    """
    A custom widget with a clear label and a separate ON/OFF switch.
    """
    def __init__(self, parent, text, command, **kwargs):
        super().__init__(parent, fg_color="transparent", **kwargs)
        
        self.command = command
        self.state = False # Start as OFF

        self.grid_columnconfigure(0, weight=1)
        
        self.label = ctk.CTkLabel(self, text=text, font=BUTTON_FONT, text_color=TEXT_COLOR_PRIMARY)
        self.label.grid(row=0, column=0, pady=(0, 5), sticky="ew")
        
        self.switch = ctk.CTkButton(self, text="OFF",
                                    font=SWITCH_FONT,
                                    corner_radius=5, 
                                    height=25,
                                    fg_color=ACCENT_COLOR, 
                                    # *** FIX: Use explicit hover color constant ***
                                    hover_color=ACCENT_COLOR_HOVER,
                                    text_color=TEXT_COLOR_PRIMARY,
                                    command=self._toggle)
        self.switch.grid(row=1, column=0, sticky="ew")

    def _toggle(self):
        # This function is called by the button press
        # It then calls the external command (which will update the app's BooleanVar)
        self.command() 
        
    def set_state(self, is_on: bool):
        """
        Updates the visual state of the switch (ON or OFF)
        """
        self.state = is_on
        if is_on:
            self.switch.configure(text="ON", 
                                  fg_color=ACTIVE_COLOR, 
                                  # *** FIX: Use explicit hover color constant ***
                                  hover_color=ACTIVE_COLOR_HOVER,
                                  text_color=BG_COLOR)
        else:
            self.switch.configure(text="OFF", 
                                  fg_color=ACCENT_COLOR, 
                                  # *** FIX: Use explicit hover color constant ***
                                  hover_color=ACCENT_COLOR_HOVER,
                                  text_color=TEXT_COLOR_PRIMARY)

# --- Main Application ---
class FlightCockpitApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        # --- Window Setup ---
        self.title("Advanced Flight Control Cockpit (UDP LOOPBACK)")
        self.geometry("1540x920") # Fixed window size
        self.resizable(False, False) 
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue") 
        self.configure(fg_color=BG_COLOR) 

        self.grid_columnconfigure(0, weight=0)
        self.grid_rowconfigure(0, weight=0) # Row 0 for top_frame
        self.grid_rowconfigure(1, weight=0) # Row 1 for bottom_frame
        self.grid_rowconfigure(2, weight=0) # Row 2 for console_frame

        # --- Data Variables ---
        self.pitch = ctk.DoubleVar(value=0.0)
        self.roll = ctk.DoubleVar(value=0.0)
        self.z_accel = ctk.DoubleVar(value=0.0)
        self.throttle = ctk.DoubleVar(value=0.0) # 0.0 to 1.0 (internal logic is 0-100)
        self.buttons = [ctk.BooleanVar(value=False) for _ in range(6)]
        self.airspeed = ctk.DoubleVar(value=0.0) # knots
        self.altitude = ctk.DoubleVar(value=0.0) # feet
        self.heading = ctk.DoubleVar(value=0.0) # degrees 0-360
        self.vertical_speed = ctk.DoubleVar(value=0.0) # ft/min

        # --- Simulation State Variables (for sender thread) ---
        self.sim_roll = 0.0
        self.sim_pitch = 0.0
        self.sim_throttle = 50.0
        self.sim_airspeed = 100.0
        self.sim_altitude = 5000.0
        self.sim_heading = 0.0
        self.sim_vertical_speed = 0.0 # ft/min
        self.sim_buttons = [False] * 6

        # --- Threading & UDP Setup ---
        self.running = True # Flag to control threads
        self.udp_queue = queue.Queue() # Queue for thread-safe GUI updates
        self.last_packet_time = time.time()

        # Create UI Elements
        self.create_main_layout()

        # --- Start Threads and Polling Loops ---
        
        # Start the UDP receiver thread
        self.receiver_thread = threading.Thread(target=self.udp_receiver_thread, daemon=True)
        self.receiver_thread.start()
        
        # Start the UDP simulator (sender) thread
        self.simulator_thread = threading.Thread(target=self.udp_simulator_thread, daemon=True)
        self.simulator_thread.start()

        # Start the main GUI poller for the queue
        self.after(50, self.check_udp_queue)
        
        # Start the 1-second-timer for the flight clock
        self.after(1000, self.update_flight_time)

        # Handle window closing
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def on_closing(self):
        """Handle window close event to shut down threads."""
        self.running = False
        time.sleep(0.1) # Give threads a moment to stop
        self.destroy()

    # --- UI Creation Methods ---
    def create_main_layout(self):
        # Top-level frame for PFD and side panels
        top_frame = ctk.CTkFrame(self, fg_color=BG_COLOR)
        top_frame.grid(row=0, column=0, padx=10, pady=10) 
        top_frame.grid_columnconfigure(0, weight=0) # PFD column
        top_frame.grid_columnconfigure(1, weight=0) # Comm panel column
        top_frame.grid_columnconfigure(2, weight=0) # Throttle column
        top_frame.grid_rowconfigure(0, weight=0) # Main content row

        self.create_pfd_frame(top_frame)
        self.create_comm_panel(top_frame) 
        self.create_vertical_throttle_panel(top_frame) # Vertical throttle on the right

        # Bottom panel for buttons and other info
        # *** FIX: Adjust height for layout ***
        bottom_frame = ctk.CTkFrame(self, fg_color=BG_COLOR, height=140)
        bottom_frame.grid(row=1, column=0, padx=10, pady=(0,10), sticky="ew") 
        bottom_frame.grid_columnconfigure(0, weight=0) 
        bottom_frame.grid_columnconfigure(1, weight=0)

        self.create_button_panel(bottom_frame)
        self.create_bottom_info_panel(bottom_frame)
        
        # Create and grid the new console frame
        self.create_console_frame()

    def create_pfd_frame(self, parent_frame):
        """Main frame for the Primary Flight Display."""
        # *** FIX: Adjust height for layout ***
        pfd_frame = ctk.CTkFrame(parent_frame, fg_color=PANEL_COLOR, 
                                 border_width=2, border_color=ACCENT_COLOR, corner_radius=10,
                                 width=1130, height=580)
        pfd_frame.grid(row=0, column=0, padx=(10, 5), pady=10)
        pfd_frame.grid_propagate(False) 
        
        pfd_frame.grid_columnconfigure(0, weight=0) # Airspeed tape (fixed width)
        pfd_frame.grid_columnconfigure(1, weight=1) # Horizon (expand)
        pfd_frame.grid_columnconfigure(2, weight=0) # Altitude tape (fixed width)
        pfd_frame.grid_columnconfigure(3, weight=0) # VSI (fixed width)
        pfd_frame.grid_rowconfigure(0, weight=1) # Main content row (expand)
        pfd_frame.grid_rowconfigure(1, weight=0) # Heading indicator (fixed height)

        self.create_airspeed_tape(pfd_frame)
        self.create_attitude_indicator(pfd_frame)
        self.create_altitude_tape(pfd_frame)
        self.create_heading_indicator(pfd_frame)
        self.create_vertical_speed_indicator(pfd_frame) 

    def create_vertical_throttle_panel(self, parent_frame):
        # *** FIX: Adjust height for layout ***
        frame = ctk.CTkFrame(parent_frame, fg_color=PANEL_COLOR, 
                             border_width=2, border_color=ACCENT_COLOR, corner_radius=10,
                             width=150, height=580)
        frame.grid(row=0, column=2, padx=(5,10), pady=10)
        frame.grid_propagate(False) 
        
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=0) # Label
        frame.grid_rowconfigure(1, weight=1) # Throttle bar
        frame.grid_rowconfigure(2, weight=0) # Label

        label = ctk.CTkLabel(frame, text="THROTTLE", font=MEDIUM_FONT, text_color=TEXT_COLOR_PRIMARY)
        label.grid(row=0, column=0, padx=10, pady=10)

        self.throttle_bar = ctk.CTkProgressBar(frame, orientation="vertical", width=40,
                                               fg_color=ACCENT_COLOR, progress_color=TEXT_COLOR_HIGHLIGHT)
        self.throttle_bar.grid(row=1, column=0, padx=20, pady=10, sticky="ns")
        self.throttle_bar.set(0) 

        self.throttle_label = ctk.CTkLabel(frame, text="0%", font=MEDIUM_FONT, text_color=TEXT_COLOR_HIGHLIGHT)
        self.throttle_label.grid(row=2, column=0, padx=10, pady=10)

    def create_comm_panel(self, parent_frame):
        """Creates a new panel for communication/debug status."""
        # *** FIX: Adjust height for layout ***
        frame = ctk.CTkFrame(parent_frame, fg_color=PANEL_COLOR, 
                             border_width=2, border_color=ACCENT_COLOR, corner_radius=10,
                             width=200, height=580)
        frame.grid(row=0, column=1, padx=(5, 5), pady=10)
        frame.grid_propagate(False) 
        
        frame.grid_columnconfigure(0, weight=1)
        
        title_label = ctk.CTkLabel(frame, text="COMM STATUS", font=MEDIUM_FONT, text_color=TEXT_COLOR_PRIMARY)
        title_label.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        udp_title = ctk.CTkLabel(frame, text="UDP Status:", font=SMALL_FONT, text_color=TEXT_COLOR_PRIMARY)
        udp_title.grid(row=1, column=0, padx=10, pady=(10, 0), sticky="w")
        
        self.udp_status_label = ctk.CTkLabel(frame, text="LISTENING...", font=MEDIUM_FONT, text_color=WARNING_COLOR)
        self.udp_status_label.grid(row=2, column=0, padx=10, pady=5, sticky="w")
        
        last_packet_title = ctk.CTkLabel(frame, text="Last Packet:", font=SMALL_FONT, text_color=TEXT_COLOR_PRIMARY)
        last_packet_title.grid(row=3, column=0, padx=10, pady=(10, 0), sticky="w")
        
        self.last_packet_label = ctk.CTkLabel(frame, text="N/A", font=MEDIUM_FONT, text_color=WARNING_COLOR)
        self.last_packet_label.grid(row=4, column=0, padx=10, pady=5, sticky="w")

    def create_airspeed_tape(self, parent_frame):
        frame = ctk.CTkFrame(parent_frame, width=120, fg_color="transparent") 
        frame.grid(row=0, column=0, sticky="ns", padx=(10, 0))
        frame.grid_propagate(False)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self.airspeed_canvas = tk.Canvas(frame, bg="#1a1a1a", highlightthickness=0, width=100)
        self.airspeed_canvas.grid(row=0, column=0, sticky="ns")

        self.airspeed_tape_elements = [] # Will store [line_id, text_id, speed_value]
        self.airspeed_readout_text = None
        self.pixels_per_knot = 4 
        
        self.airspeed_canvas.bind("<Configure>", self.on_airspeed_tape_resize)

    def create_attitude_indicator(self, parent_frame):
        """ Creates the Artificial Horizon (Attitude Indicator) """
        frame = ctk.CTkFrame(parent_frame, fg_color="transparent")
        frame.grid(row=0, column=1, padx=0, pady=0, sticky="nsew")

        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self.horizon_canvas = tk.Canvas(frame, bg="#3477eb", highlightthickness=0) # Sky color
        self.horizon_canvas.grid(row=0, column=0, sticky="nsew")

        self.ground_poly = self.horizon_canvas.create_polygon(
            -1000, 300, 1000, 300, 1000, 1000, -1000, 1000,
            fill="#8b4513", outline="white", width=2
        )

        self.pitch_ladder_lines = []
        self.pitch_ladder_texts = []
        # self.pitch_ladder_bgs = [] # No longer needed
        self.roll_indicator_lines = []
        self.roll_indicator_texts = []
        self.roll_indicator_static_arc = None
        self.roll_indicator_static_pointer = None
        self.static_plane_symbol = None
        
        # *** FIX: Initialize roll_pointer to None to prevent race condition ***
        self.roll_pointer = None
        
        self.horizon_canvas.bind("<Configure>", self.on_horizon_canvas_resize)

    def create_roll_indicator(self):
        """ Draws the fixed roll indicator at the top of the horizon canvas """
        c = self.horizon_canvas
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10 or h < 10: w, h = 300, 300 # Default size

        if self.roll_indicator_static_arc:
            c.delete(self.roll_indicator_static_arc)
        for item in [item[0] for item in self.roll_indicator_lines]: c.delete(item)
        for item in [item[0] for item in self.roll_indicator_texts]: c.delete(item)
        if self.roll_indicator_static_pointer: c.delete(self.roll_indicator_static_pointer)
        self.roll_indicator_lines = []
        self.roll_indicator_texts = []

        self.roll_indicator_static_arc = c.create_arc(
            w/2 - 100, 10, w/2 + 100, 10 + 200, 
            start=30, extent=120, style=tk.ARC, outline="white", width=2
        )

        roll_marks = [-45, -30, -20, -10, 0, 10, 20, 30, 45]
        for angle in roll_marks:
            length = 15 if angle % 30 == 0 else 10 
            theta = math.radians(90 - angle) 
            x1 = w/2 + 70 * math.cos(theta)
            y1 = 85 - 70 * math.sin(theta)
            x2 = w/2 + (70+length) * math.cos(theta)
            y2 = 85 - (70+length) * math.sin(theta)
            line_id = c.create_line(x1, y1, x2, y2, fill="white", width=2)
            self.roll_indicator_lines.append((line_id, angle))
            
            if self.roll_indicator_static_pointer:
                c.delete(self.roll_indicator_static_pointer)
            
            if angle == 0:
                 self.roll_indicator_static_pointer = c.create_polygon(
                     w/2, 20, w/2-10, 30, w/2+10, 30, fill="yellow", outline=""
                 ) 
            
            if angle in [-30, 30]:
                text_id = c.create_text(x2, y2 + 15, text=str(abs(angle)), fill="white", font=(FONT_FAMILY, 10), anchor="center")
                self.roll_indicator_texts.append((text_id, angle))

        self.roll_pointer = c.create_polygon(0,0,0,0,0,0, fill="yellow", outline="") # Placeholder

    def create_static_plane_symbol(self):
        """ Draws the fixed aircraft symbol in the middle of the canvas """
        c = self.horizon_canvas
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10 or h < 10: w, h = 600, 400 
        
        if self.static_plane_symbol:
            c.delete(self.static_plane_symbol)

        center_x, center_y = w / 2, h / 2
        
        wing_w = 120
        wing_h = 10
        dot_r = 4
        
        self.static_plane_symbol = c.create_polygon(
            center_x - wing_w, center_y,
            center_x - 80, center_y,
            center_x - dot_r, center_y,
            center_x - dot_r, center_y - dot_r,
            center_x + dot_r, center_y - dot_r,
            center_x + dot_r, center_y,
            center_x + 80, center_y,
            center_x + wing_w, center_y,
            center_x + wing_w, center_y + wing_h,
            center_x + 80, center_y + wing_h/2,
            center_x + 30, center_y + wing_h/2,
            center_x + 20, center_y + 10,
            center_x, center_y + 5,
            center_x - 20, center_y + 10,
            center_x - 30, center_y + wing_h/2,
            center_x - 80, center_y + wing_h/2,
            center_x - wing_w, center_y + wing_h,
            center_x - wing_w, center_y,
            fill="yellow", outline=""
        )

    def create_altitude_tape(self, parent_frame):
        frame = ctk.CTkFrame(parent_frame, width=120, fg_color="transparent")
        frame.grid(row=0, column=2, sticky="ns", padx=(0, 10))
        frame.grid_propagate(False)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self.altitude_canvas = tk.Canvas(frame, bg="#1a1a1a", highlightthickness=0, width=100)
        self.altitude_canvas.grid(row=0, column=0, sticky="ns")
        
        self.altitude_tape_elements = [] # Will store [line_id, text_id, alt_value]
        self.altitude_readout_text = None
        self.pixels_per_foot = 0.2 
        
        self.altitude_canvas.bind("<Configure>", self.on_altitude_tape_resize)

    def create_vertical_speed_indicator(self, parent_frame):
        """Creates a simple vertical speed indicator (VSI) to the right of altitude tape."""
        frame = ctk.CTkFrame(parent_frame, width=60, fg_color="transparent")
        frame.grid(row=0, column=3, sticky="ns", padx=(0,10))
        frame.grid_propagate(False)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self.vsi_canvas = tk.Canvas(frame, bg="#1a1a1a", highlightthickness=0, width=50)
        self.vsi_canvas.grid(row=0, column=0, sticky="ns")
        self.vsi_readout_text = None 
        
        self.vsi_canvas.bind("<Configure>", self.on_vsi_resize)

    def on_vsi_resize(self, event=None):
        c = self.vsi_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10: return

        c.delete("all")
        
        center_x = w/2
        center_y = h/2
        
        c.create_line(center_x, 0, center_x, h, fill="white", width=1)
        
        range_vsi = 6000 # +/- 6000 ft/min
        pixels_per_1000 = h / (range_vsi / 500) 

        for i in range(-6000, 6001, 1000):
            if i == 0: continue
            
            y_pos = center_y - (i / 1000) * pixels_per_1000
            
            if -10 < y_pos < h + 10:
                c.create_line(center_x - 5, y_pos, center_x + 5, y_pos, fill="white", width=1)
                if i % 2000 == 0:
                    c.create_text(center_x + 15, y_pos, text=str(abs(i // 1000)), fill="white", font=(FONT_FAMILY, 8), anchor="w")
        
        c.create_polygon(center_x, center_y, center_x + 10, center_y - 5, center_x + 10, center_y + 5,
                         fill="yellow", outline="")
        
        box_width = 40
        box_height = 20
        c.create_rectangle(center_x - box_width/2, h - box_height, center_x + box_width/2, h,
                           fill="#333333", outline="white", width=1)
        self.vsi_readout_text = c.create_text(center_x, h - box_height/2, text="+000",
                                               fill="lime", font=(FONT_FAMILY, 10, "bold"))


    def create_heading_indicator(self, parent_frame):
        frame = ctk.CTkFrame(parent_frame, height=100, fg_color="transparent")
        frame.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(10,0))
        frame.grid_propagate(False)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)

        self.heading_canvas = tk.Canvas(frame, bg="#1a1a1a", highlightthickness=0, height=80)
        self.heading_canvas.grid(row=0, column=0, sticky="ew", padx=10)
        
        self.heading_tape_elements = [] # Will store [line_id, text_id, deg_value]
        self.heading_readout_text = None
        self.heading_static_pointer = None
        self.heading_static_box = None
        self.pixels_per_degree = 2.0 

        self.heading_canvas.bind("<Configure>", self.on_heading_indicator_resize)

    def create_button_panel(self, parent_frame):
        # *** FIX: Adjust height for layout ***
        frame = ctk.CTkFrame(parent_frame, fg_color=PANEL_COLOR, 
                             border_width=2, border_color=ACCENT_COLOR, corner_radius=10,
                             width=900, height=140)
        frame.grid(row=0, column=0, padx=(10, 5), pady=10)
        frame.pack_propagate(False) # Stop it from shrinking

        ctk.CTkLabel(frame, text="SYSTEM CONTROLS", font=MEDIUM_FONT, text_color=TEXT_COLOR_PRIMARY).pack(pady=5)

        button_grid = ctk.CTkFrame(frame, fg_color="transparent")
        button_grid.pack(pady=10, fill="x", expand=True)
        button_grid.grid_columnconfigure((0,1,2,3,4,5), weight=1)

        self.button_widgets = []
        for i in range(6):
            switch = ToggleSwitch(button_grid, 
                                  text=BUTTON_NAMES[i],
                                  command=lambda idx=i: self.toggle_button_state(idx))
            switch.grid(row=0, column=i, padx=10, pady=5, sticky="ew")
            self.button_widgets.append(switch)

    def toggle_button_state(self, index):
        """
        Called by the ToggleSwitch. This updates the *data* (BooleanVar).
        The update_button_appearance function will update the *visuals*.
        Note: This is for MANUAL clicks, not for UDP updates.
        """
        current_state = self.buttons[index].get()
        self.buttons[index].set(not current_state)
        self.update_button_appearance(index) # Update GUI immediately on click

    def update_button_appearance(self, index):
        """
        Updates the custom ToggleSwitch widget based on the BooleanVar state.
        """
        is_on = self.buttons[index].get()
        self.button_widgets[index].set_state(is_on)


    def create_bottom_info_panel(self, parent_frame):
        # *** FIX: Adjust height for layout ***
        frame = ctk.CTkFrame(parent_frame, fg_color=PANEL_COLOR, 
                             border_width=2, border_color=ACCENT_COLOR, corner_radius=10,
                             width=580, height=140)
        frame.grid(row=0, column=1, padx=(5, 10), pady=10)
        frame.grid_propagate(False) # Stop it from shrinking
        
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_columnconfigure(2, weight=1)
        frame.grid_rowconfigure((0,1), weight=1)
        
        # Z-Accel
        z_accel_label = ctk.CTkLabel(frame, text="Z-ACCEL", font=MEDIUM_FONT, text_color=TEXT_COLOR_PRIMARY)
        z_accel_label.grid(row=0, column=0, pady=(10,0), padx=20, sticky="s")
        self.z_label = ctk.CTkLabel(frame, text="0.00 g", font=LARGE_FONT, text_color=TEXT_COLOR_HIGHLIGHT)
        self.z_label.grid(row=1, column=0, pady=(0,10), padx=20, sticky="n")
        
        # Flight Time
        time_label = ctk.CTkLabel(frame, text="FLIGHT TIME", font=MEDIUM_FONT, text_color=TEXT_COLOR_PRIMARY)
        time_label.grid(row=0, column=1, pady=(10,0), padx=10, sticky="s")
        self.flight_time_label = ctk.CTkLabel(frame, text="00:00:00", font=LARGE_FONT, text_color=TEXT_COLOR_HIGHLIGHT)
        self.flight_time_label.grid(row=1, column=1, pady=(0,10), padx=10, sticky="n")
        
        # System Status
        status_label = ctk.CTkLabel(frame, text="SYSTEM STATUS", font=MEDIUM_FONT, text_color=TEXT_COLOR_PRIMARY)
        status_label.grid(row=0, column=2, pady=(10,0), padx=10, sticky="s")
        self.status_display = ctk.CTkLabel(frame, text="NORMAL", font=LARGE_FONT, text_color=ACTIVE_COLOR)
        self.status_display.grid(row=1, column=2, pady=(0,10), padx=10, sticky="n")

    def create_console_frame(self):
        """Creates the debug console frame at the bottom of the window."""
        # *** STYLE: Apply retro styling ***
        console_frame = ctk.CTkFrame(self, fg_color="#1a1a1a", 
                                     border_width=2, border_color="#666666", corner_radius=0, # Square corners
                                     height=115)
        console_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=(0,10))
        console_frame.grid_propagate(False)
        
        console_frame.grid_columnconfigure(0, weight=1)
        console_frame.grid_rowconfigure(0, weight=1)
        
        self.console_textbox = ctk.CTkTextbox(console_frame, 
                                              # *** STYLE: Apply geeky font and colors ***
                                              font=CONSOLE_FONT, 
                                              text_color=CONSOLE_TEXT_COLOR,
                                              fg_color=CONSOLE_BG_COLOR,
                                              border_width=0)
        self.console_textbox.grid(row=0, column=0, padx=10, pady=10, sticky="nsew") # Added padding
        self.console_textbox.insert("end", "Debug Console Initialized...\n")
        self.console_textbox.configure(state="disabled")

    # --- Threading, UDP and GUI Update Loop ---

    def udp_receiver_thread(self):
        """
        Runs in a separate thread, listens for UDP packets,
        and puts them in the queue.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0) # 1 second timeout
        try:
            sock.bind((UDP_IP, UDP_PORT))
            print(f"UDP Receiver listening on {UDP_IP}:{UDP_PORT}")
        except OSError as e:
            print(f"!!! UDP BIND FAILED: {e}")
            print("!!! Is another instance running? Exiting thread.")
            return

        while self.running:
            try:
                data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
                raw_packet = data.decode()
                self.udp_queue.put(raw_packet)
            except socket.timeout:
                continue # Just loop again if no packet in 1s
            except Exception as e:
                if self.running:
                    print(f"Receiver error: {e}")
        sock.close()
        print("UDP Receiver thread stopped.")

    def udp_simulator_thread(self):
        """
        Runs in a separate thread, generates fake data,
        and sends it to the receiver.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"UDP Simulator sending to {UDP_IP}:{UDP_PORT}")
        
        while self.running:
            # --- Generate Simulated Data ---
            self.sim_roll += random.uniform(-1.5, 1.5)
            self.sim_pitch += random.uniform(-1.0, 1.0)
            self.sim_roll = max(-45.0, min(45.0, self.sim_roll))
            self.sim_pitch = max(-25.0, min(25.0, self.sim_pitch))
            
            sim_z_accel = 1.0 + random.uniform(-0.05, 0.05)
            
            self.sim_throttle += random.uniform(-2.0, 2.0)
            self.sim_throttle = max(0.0, min(100.0, self.sim_throttle))
            
            self.sim_airspeed += (self.sim_throttle / 20 - 5) + random.uniform(-1.0, 1.0)
            self.sim_airspeed = max(0.0, min(300.0, self.sim_airspeed))
            
            self.sim_vertical_speed = (self.sim_pitch * 100) + (self.sim_throttle * 5) - 250 
            self.sim_vertical_speed = max(-6000, min(6000, self.sim_vertical_speed)) 
            
            self.sim_altitude += self.sim_vertical_speed * (0.05 / 60) 
            self.sim_altitude = max(0.0, min(20000.0, self.sim_altitude)) 
            
            self.sim_heading += random.uniform(-0.5, 0.5)
            self.sim_heading = (self.sim_heading + 360) % 360 
            
            for i in range(6):
                if random.random() < 0.01:
                    self.sim_buttons[i] = not self.sim_buttons[i]

            # --- Format and Send Packet ---
            # Format: "roll,pitch,z_accel,throttle,b1,b2,b3,b4,b5,b6"
            packet_data = (
                f"{self.sim_roll:.2f},"
                f"{self.sim_pitch:.2f},"
                f"{sim_z_accel:.2f},"
                f"{self.sim_throttle:.1f}," # Send as 0-100
                f"{self.sim_buttons[0]},"
                f"{self.sim_buttons[1]},"
                f"{self.sim_buttons[2]},"
                f"{self.sim_buttons[3]},"
                f"{self.sim_buttons[4]},"
                f"{self.sim_buttons[5]}"
            )
            
            # Add sim data for PFD tapes (which are not in the packet)
            # This is a bit of a cheat for the loopback, but makes it work
            # In a real system, the receiver would not have this data.
            packet_data += (
                f",{self.sim_airspeed:.2f}"
                f",{self.sim_altitude:.2f}"
                f",{self.sim_heading:.2f}"
                f",{self.sim_vertical_speed:.2f}"
            )
            
            sock.sendto(packet_data.encode(), (UDP_IP, UDP_PORT))
            
            time.sleep(0.05) # ~20 Hz update rate
        
        sock.close()
        print("UDP Simulator thread stopped.")

    def check_udp_queue(self):
        """
        Polls the queue from the main thread and updates the GUI.
        This is the only thread-safe way to update tkinter.
        """
        try:
            while not self.udp_queue.empty():
                raw_packet = self.udp_queue.get_nowait()
                
                # --- Parse Packet ---
                parts = raw_packet.split(',')
                if len(parts) >= 10: # Check for minimum length
                    # This is where you parse the *real* packet
                    self.roll.set(float(parts[0]))
                    self.pitch.set(float(parts[1]))
                    self.z_accel.set(float(parts[2]))
                    self.throttle.set(float(parts[3]) / 100.0) # Convert 0-100 to 0.0-1.0
                    
                    # Parse boolean strings
                    self.buttons[0].set(parts[4] == "True")
                    self.buttons[1].set(parts[5] == "True")
                    self.buttons[2].set(parts[6] == "True")
                    self.buttons[3].set(parts[7] == "True")
                    self.buttons[4].set(parts[8] == "True")
                    self.buttons[5].set(parts[9] == "True")
                    
                    # Update PFD variables from our "cheated" extra data
                    if len(parts) >= 14:
                        self.airspeed.set(float(parts[10]))
                        self.altitude.set(float(parts[11]))
                        self.heading.set(float(parts[12]))
                        self.vertical_speed.set(float(parts[13]))

                    # --- Update Comm Status Panel ---
                    self.udp_status_label.configure(text="CONNECTED", text_color=ACTIVE_COLOR)
                    self.last_packet_label.configure(text="< 1s ago", text_color=TEXT_COLOR_PRIMARY)
                    self.last_packet_time = time.time()
                    
                    now=datetime.now()
                    time_stamp = now.strftime("%H:%M:%S")
                    # --- Update Console ---
                    self.console_textbox.configure(state="normal")
                    # *** STYLE: Apply retro prompt ***
                    self.console_textbox.insert("end", f"{CONSOLE_PROMPT}[{time_stamp}] {raw_packet}\n")
                    self.console_textbox.see("end")
                    self.console_textbox.configure(state="disabled")

                    # --- Update All GUI Elements ---
                    self.update_gui_elements()
                
                else:
                    print(f"Malformed packet received: {raw_packet}")

        except queue.Empty:
            pass # No new data
        
        # Check for timeout
        if (time.time() - self.last_packet_time) > 2.0:
            self.udp_status_label.configure(text="DISCONNECTED", text_color=CRITICAL_COLOR)
            self.last_packet_label.configure(text="> 2s ago", text_color=WARNING_COLOR)

        # Reschedule the checker
        if self.running:
            self.after(50, self.check_udp_queue) # Check again in 50ms

    def update_flight_time(self):
        """Updates the flight time label every second."""
        try:
            parts = self.flight_time_label.cget("text").split(":")
            hours, minutes, seconds = int(parts[0]), int(parts[1]), int(parts[2])
            
            seconds += 1
            if seconds >= 60:
                seconds = 0
                minutes += 1
                if minutes >= 60:
                    minutes = 0
                    hours += 1
            
            self.flight_time_label.configure(text=f"{hours:02}:{minutes:02}:{seconds:02}")
        except ValueError:
            self.flight_time_label.configure(text="00:00:00")
        
        if self.running:
            self.after(1000, self.update_flight_time) # Schedule next run

    def update_gui_elements(self):
        """ 
        Update all widgets and canvas elements based on the current
        state of the ctk.DoubleVar/BooleanVar variables.
        """
        # Throttle
        throttle_val = self.throttle.get()
        self.throttle_bar.set(throttle_val)
        self.throttle_label.configure(text=f"{throttle_val*100:.0f}%")

        # Z-Accel
        self.z_label.configure(text=f"{self.z_accel.get():.2f} g")

        # Buttons (driven by UDP data)
        for i in range(6):
            self.update_button_appearance(i) 

        # Update PFD elements
        self.update_horizon()
        self.update_airspeed_tape()
        self.update_altitude_tape()
        self.update_heading_indicator()
        self.update_vertical_speed_indicator()

        # Update System Status (Example logic)
        if self.z_accel.get() > 1.5 or self.z_accel.get() < 0.5:
            self.status_display.configure(text="WARNING: G-LOAD", text_color=WARNING_COLOR)
        elif self.buttons[4].get(): # Stall warning button active
            self.status_display.configure(text="STALL WARNING", text_color=CRITICAL_COLOR)
        elif self.altitude.get() < 100 and not self.buttons[0].get(): # Landing gear
            self.status_display.configure(text="GEAR UP LOW ALT", text_color=WARNING_COLOR)
        else:
            self.status_display.configure(text="NORMAL", text_color=ACTIVE_COLOR)


    # --- PFD Drawing Logic (No changes below this line) ---

    def on_horizon_canvas_resize(self, event):
        """ Redraw static elements on resize and update horizon """
        self.horizon_canvas.delete("all") 
        
        self.ground_poly = self.horizon_canvas.create_polygon(
            0,0,0,0,0,0,0,0, # Dummy coords
            fill="#8b4513", outline="white", width=2
        )
        
        self.pitch_ladder_lines = []
        self.pitch_ladder_texts = []
        # self.pitch_ladder_bgs = [] # REMOVED

        c = self.horizon_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10: return
        
        pixels_per_degree_pitch = h / 90.0
        
        for p_deg in range(-60, 61, 5): 
            if p_deg == 0: continue
            
            line_length = 50 if p_deg % 10 == 0 else 25
            line_id = c.create_line(0,0,0,0, fill="white", width=2)
            self.pitch_ladder_lines.append((line_id, p_deg, line_length))

            if p_deg % 10 == 0: 
                txt_l = c.create_text(0,0, text=str(abs(p_deg)), fill="yellow", font=(FONT_FAMILY, 10, "bold")) 
                txt_r = c.create_text(0,0, text=str(abs(p_deg)), fill="yellow", font=(FONT_FAMILY, 10, "bold")) 
                self.pitch_ladder_texts.append((txt_l, txt_r, p_deg))
        
        self.create_static_plane_symbol()
        self.create_roll_indicator() 
        
        self.update_horizon()

    def update_horizon(self):
        """ The core logic to redraw the attitude indicator """
        # *** FIX: Add check to prevent error before canvas is ready ***
        if self.roll_pointer is None:
            return 
            
        c = self.horizon_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10: return 

        pitch = self.pitch.get()
        roll = self.roll.get() 

        pixels_per_degree_pitch = h / 90.0 
        pitch_offset = pitch * pixels_per_degree_pitch 

        roll_rad = math.radians(roll)
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)

        center_x, center_y = w / 2, (h / 2) - pitch_offset
        diag = max(w,h) * 2 

        horizon_line_left_x = center_x + (-diag * cos_roll) - (0 * sin_roll)
        horizon_line_left_y = center_y + (-diag * sin_roll) + (0 * cos_roll)
        horizon_line_right_x = center_x + (diag * cos_roll) - (0 * sin_roll)
        horizon_line_right_y = center_y + (diag * sin_roll) + (0 * cos_roll)

        ground_bottom_left_x = center_x + (-diag * cos_roll) - (diag * sin_roll)
        ground_bottom_left_y = center_y + (-diag * sin_roll) + (diag * cos_roll)
        ground_bottom_right_x = center_x + (diag * cos_roll) - (diag * sin_roll)
        ground_bottom_right_y = center_y + (diag * sin_roll) + (diag * cos_roll)

        self.horizon_canvas.coords(self.ground_poly,
            ground_bottom_left_x, ground_bottom_left_y,
            ground_bottom_right_x, ground_bottom_right_y,
            horizon_line_right_x, horizon_line_right_y,
            horizon_line_left_x, horizon_line_left_y
        )

        cx, cy = w/2, h/2
        
        for (line_id, p_deg, line_length) in self.pitch_ladder_lines:
            p_center_y = (h / 2) - (p_deg * pixels_per_degree_pitch)

            lx1_pre_rot = cx - line_length
            ly1_pre_rot = p_center_y
            lx2_pre_rot = cx + line_length
            ly2_pre_rot = p_center_y

            x1_rot = cx + (lx1_pre_rot - cx) * cos_roll - (ly1_pre_rot - cy) * sin_roll
            y1_rot = cy + (lx1_pre_rot - cx) * sin_roll + (ly1_pre_rot - cy) * cos_roll
            x2_rot = cx + (lx2_pre_rot - cx) * cos_roll - (ly2_pre_rot - cy) * sin_roll
            y2_rot = cy + (lx2_pre_rot - cx) * sin_roll + (ly2_pre_rot - cy) * cos_roll
            
            c.coords(line_id, x1_rot, y1_rot, x2_rot, y2_rot)
        
        for (txt_l, txt_r, p_deg) in self.pitch_ladder_texts:
            p_center_y = (h / 2) - (p_deg * pixels_per_degree_pitch)
            line_length = 50 
            
            lx1_pre_rot = cx - line_length
            ly1_pre_rot = p_center_y
            lx2_pre_rot = cx + line_length
            
            text_x_left = cx + (lx1_pre_rot - cx - 20) * cos_roll - (ly1_pre_rot - cy) * sin_roll 
            text_y_left = cy + (lx1_pre_rot - cx - 20) * sin_roll + (ly1_pre_rot - cy) * cos_roll
            text_x_right = cx + (lx2_pre_rot - cx + 20) * cos_roll - (ly1_pre_rot - cy) * sin_roll 
            text_y_right = cy + (lx2_pre_rot - cx + 20) * sin_roll + (ly1_pre_rot - cy) * cos_roll
            
            c.coords(txt_l, text_x_left, text_y_left)
            c.itemconfigure(txt_l, angle=-roll)
            
            c.coords(txt_r, text_x_right, text_y_right)
            c.itemconfigure(txt_r, angle=-roll)
            
            c.tag_lower(txt_l, self.static_plane_symbol)
            c.tag_lower(txt_r, self.static_plane_symbol)

        for (line_id, _, _) in self.pitch_ladder_lines:
            c.tag_lower(line_id, self.static_plane_symbol)
        c.tag_lower(self.ground_poly, self.static_plane_symbol)

        pointer_base_y = 40 
        pointer_tip_y = 20
        pointer_width_half = 10
        
        p1 = (w/2 - pointer_width_half, pointer_base_y)
        p2 = (w/2 + pointer_width_half, pointer_base_y)
        p3 = (w/2, pointer_tip_y)
        
        rotated_points = []
        for px, py in [p1, p2, p3]:
            rot_x = cx + (px - cx) * cos_roll - (py - cy) * sin_roll
            rot_y = cy + (px - cx) * sin_roll + (py - cy) * cos_roll
            rotated_points.extend([rot_x, rot_y])
        
        c.coords(self.roll_pointer, *rotated_points)

    def _rotate_rectangle_coords(self, x1, y1, x2, y2, pivot_x, pivot_y, angle_rad):
        """Helper to rotate rectangle corners around a pivot point."""
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        rotated = []
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        for x, y in corners:
            temp_x = x - pivot_x
            temp_y = y - pivot_y
            rotated_x = temp_x * cos_a - temp_y * sin_a + pivot_x
            rotated_y = temp_x * sin_a + temp_y * cos_a + pivot_y
            rotated.extend([rotated_x, rotated_y])
        return rotated
        
    def on_airspeed_tape_resize(self, event=None):
        c = self.airspeed_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10: return

        c.delete("all")
        self.airspeed_tape_elements = []
        
        self.pixels_per_knot = h / 100 # 100 knots visible range
        
        tape_center_y = h / 2
        
        box_width = w - 20 
        box_height = 40
        c.create_rectangle(10, tape_center_y - box_height/2, 10 + box_width, tape_center_y + box_height/2,
                           fill="#333333", outline="white", width=1)
        self.airspeed_readout_text = c.create_text(10 + box_width/2, tape_center_y, text="0",
                                            fill="lime", font=(FONT_FAMILY, 20, "bold"))
        
        c.create_polygon(w, tape_center_y, w - 15, tape_center_y - 8, w - 15, tape_center_y + 8,
                         fill="yellow", outline="")
                         
        for i in range(0, 301, 10): 
            line_id = c.create_line(0,0,0,0, fill="white")
            text_id = None
            if i % 50 == 0: 
                text_id = c.create_text(0,0, text=str(i), fill="white", font=(FONT_FAMILY, 12))
            
            self.airspeed_tape_elements.append([line_id, text_id, i])
        
        self.update_airspeed_tape() 

    def update_airspeed_tape(self):
        c = self.airspeed_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10 or not self.airspeed_readout_text: return

        current_speed = self.airspeed.get()
        c.itemconfigure(self.airspeed_readout_text, text=f"{current_speed:.0f}")
        
        tape_center_y = h / 2
        box_width = w - 20
        
        for [line_id, text_id, speed_val] in self.airspeed_tape_elements:
            y_on_canvas = tape_center_y - (speed_val - current_speed) * self.pixels_per_knot
            
            if tape_center_y - h/2 - 20 < y_on_canvas < tape_center_y + h/2 + 20:
                if text_id: 
                    c.coords(line_id, 10 + box_width, y_on_canvas, w - 5, y_on_canvas)
                    c.itemconfigure(line_id, width=2)
                    c.coords(text_id, w - 25, y_on_canvas)
                    c.itemconfigure(text_id, state='normal')
                else: 
                    c.coords(line_id, 10 + box_width, y_on_canvas, w - 15, y_on_canvas)
                    c.itemconfigure(line_id, width=1)
                c.itemconfigure(line_id, state='normal')
            else:
                c.itemconfigure(line_id, state='hidden')
                if text_id:
                    c.itemconfigure(text_id, state='hidden')

    def on_altitude_tape_resize(self, event=None):
        c = self.altitude_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10: return

        c.delete("all")
        self.altitude_tape_elements = []

        self.pixels_per_foot = h / 2000 # 2000 feet visible range
        
        tape_center_y = h / 2
        
        box_width = w - 20
        box_height = 40
        c.create_rectangle(10, tape_center_y - box_height/2, 10 + box_width, tape_center_y + box_height/2,
                           fill="#333333", outline="white", width=1)
        self.altitude_readout_text = c.create_text(10 + box_width/2, tape_center_y, text="0",
                                            fill="lime", font=(FONT_FAMILY, 20, "bold"))
        
        c.create_polygon(0, tape_center_y, 15, tape_center_y - 8, 15, tape_center_y + 8,
                         fill="yellow", outline="")
                         
        for i in range(0, 20001, 100): 
            line_id = c.create_line(0,0,0,0, fill="white")
            text_id = None
            if i % 1000 == 0: 
                text_id = c.create_text(0,0, text=f"{i // 1000}k", fill="white", font=(FONT_FAMILY, 12), anchor="w")
            
            self.altitude_tape_elements.append([line_id, text_id, i])
            
        self.update_altitude_tape() 

    def update_altitude_tape(self):
        c = self.altitude_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10 or not self.altitude_readout_text: return
        
        current_altitude = self.altitude.get()
        c.itemconfigure(self.altitude_readout_text, text=f"{current_altitude:.0f}")
        
        tape_center_y = h / 2
        
        for [line_id, text_id, alt_val] in self.altitude_tape_elements:
            y_on_canvas = tape_center_y - (alt_val - current_altitude) * self.pixels_per_foot
            
            if tape_center_y - h/2 - 20 < y_on_canvas < tape_center_y + h/2 + 20:
                if text_id: 
                    c.coords(line_id, 10, y_on_canvas, 10 + 20, y_on_canvas)
                    c.itemconfigure(line_id, width=2)
                    c.coords(text_id, 10 + 25, y_on_canvas)
                    c.itemconfigure(text_id, state='normal')
                elif alt_val % 500 == 0: 
                     c.coords(line_id, 10, y_on_canvas, 10 + 15, y_on_canvas)
                     c.itemconfigure(line_id, width=1)
                else: 
                    c.coords(line_id, 10, y_on_canvas, 10 + 10, y_on_canvas)
                    c.itemconfigure(line_id, width=1)
                c.itemconfigure(line_id, state='normal')
            else:
                c.itemconfigure(line_id, state='hidden')
                if text_id:
                    c.itemconfigure(text_id, state='hidden')

    def update_vertical_speed_indicator(self):
        c = self.vsi_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10 or not self.vsi_readout_text: return
        
        current_vs = self.vertical_speed.get()
        text_val = f"{current_vs:+.0f}"
        c.itemconfigure(self.vsi_readout_text, text=text_val)

        if current_vs > 100:
            c.itemconfigure(self.vsi_readout_text, fill=ACTIVE_COLOR)
        elif current_vs < -100:
            c.itemconfigure(self.vsi_readout_text, fill=CRITICAL_COLOR)
        else:
            c.itemconfigure(self.vsi_readout_text, fill="white")

    def on_heading_indicator_resize(self, event=None):
        c = self.heading_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10: return

        c.delete("all")
        self.heading_tape_elements = []
        
        self.pixels_per_degree = (w / 90.0) # Show 90 degrees 
        center_x, center_y = w / 2, h / 2 + 10 
        
        box_width = 80
        box_height = 30
        self.heading_static_box = c.create_rectangle(center_x - box_width/2, 0, center_x + box_width/2, box_height,
                           fill="#333333", outline="white", width=1)
        self.heading_readout_text = c.create_text(center_x, box_height/2, text="0°",
                      fill="lime", font=(FONT_FAMILY, 16, "bold"))
        
        self.heading_static_pointer = c.create_polygon(center_x, box_height, center_x - 8, box_height + 10, center_x + 8, box_height + 10,
                         fill="yellow", outline="")
                         
        for deg in range(-360, 360*2, 5): 
            line_id = c.create_line(0,0,0,0, fill="white")
            text_id = None
            
            if deg % 10 == 0: 
                c.itemconfigure(line_id, width=2)
            
            if deg % 30 == 0: 
                deg_norm = deg % 360
                text = str(deg_norm)
                if deg_norm == 0: text = "N"
                elif deg_norm == 90: text = "E"
                elif deg_norm == 180: text = "S"
                elif deg_norm == 270: text = "W"
                
                text_id = c.create_text(0,0, text=text, fill="white", font=(FONT_FAMILY, 12, "bold"))

            self.heading_tape_elements.append([line_id, text_id, deg])
            
        self.update_heading_indicator() 

    def update_heading_indicator(self):
        c = self.heading_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10 or not self.heading_readout_text: return

        current_heading = self.heading.get()
        c.itemconfigure(self.heading_readout_text, text=f"{current_heading:.0f}°")
        
        center_x, center_y = w / 2, h / 2 + 10
        
        for [line_id, text_id, deg_val] in self.heading_tape_elements:
            delta_deg = deg_val - current_heading
            
            if delta_deg > 180: delta_deg -= 360
            if delta_deg < -180: delta_deg += 360
            
            x_on_canvas = center_x - (delta_deg * self.pixels_per_degree)
            
            if 0 < x_on_canvas < w:
                if text_id: 
                    c.coords(line_id, x_on_canvas, center_y - 20, x_on_canvas, center_y)
                    c.coords(text_id, x_on_canvas, center_y - 30)
                    c.itemconfigure(text_id, state='normal')
                elif deg_val % 10 == 0: 
                    c.coords(line_id, x_on_canvas, center_y - 15, x_on_canvas, center_y)
                else: 
                    c.coords(line_id, x_on_canvas, center_y - 10, x_on_canvas, center_y)
                c.itemconfigure(line_id, state='normal')
            else:
                c.itemconfigure(line_id, state='hidden')
                if text_id:
                    c.itemconfigure(text_id, state='hidden')


if __name__ == "__main__":
    app = FlightCockpitApp()
    app.mainloop()