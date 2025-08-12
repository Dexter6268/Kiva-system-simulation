import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time
from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

# 设置默认字体大小
DEFAULT_FONT = ("Arial", 10)
LARGE_FONT = ("Arial", 12)
TITLE_FONT = ("Arial", 14, "bold")


class InteractiveSimulation:
    def __init__(self, master: tk.Tk):
        self.master = master
        self.master.title("Kiva System Interactive Simulation")

        # 设置窗口默认字体
        self.master.option_add("*TLabel*Font", DEFAULT_FONT)
        self.master.option_add("*TButton*Font", DEFAULT_FONT)
        self.master.option_add("*TEntry*Font", DEFAULT_FONT)
        self.master.option_add("*TLabelFrame*Font", LARGE_FONT)

        # Simulation control states
        self.is_running = False
        self.is_paused = False
        self.simulation_speed = 1.0  # Simulation speed multiplier

        # Thread communication
        self.data_queue = queue.Queue()
        self.control_queue = queue.Queue()

        # Simulation parameters
        self.agv_num = 5
        self.order_num = 20
        self.current_step = 0

        # Simulation data
        self.animation_frames = []
        self.simulation_thread: Optional[threading.Thread] = None

        self.setup_ui()
        self.setup_plot()

    def setup_ui(self):
        """Setup user interface"""
        # Main frame
        main_frame = ttk.Frame(self.master)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Control panel
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        # Parameter settings
        params_frame = ttk.LabelFrame(control_frame, text="Simulation Parameters")
        params_frame.pack(side=tk.LEFT, padx=10, pady=5)

        ttk.Label(params_frame, text="AGV Count:", font=LARGE_FONT).grid(row=0, column=0, padx=8, pady=5, sticky="w")
        self.agv_var = tk.StringVar(value=str(self.agv_num))
        ttk.Entry(params_frame, textvariable=self.agv_var, width=10, font=LARGE_FONT).grid(
            row=0, column=1, padx=8, pady=5
        )

        ttk.Label(params_frame, text="Order Count:", font=LARGE_FONT).grid(row=1, column=0, padx=8, pady=5, sticky="w")
        self.order_var = tk.StringVar(value=str(self.order_num))
        ttk.Entry(params_frame, textvariable=self.order_var, width=10, font=LARGE_FONT).grid(
            row=1, column=1, padx=8, pady=5
        )

        # Control buttons
        control_buttons_frame = ttk.LabelFrame(control_frame, text="Simulation Control")
        control_buttons_frame.pack(side=tk.LEFT, padx=10, pady=5)

        self.start_btn = ttk.Button(control_buttons_frame, text="Start", command=self.start_simulation, width=10)
        self.start_btn.grid(row=0, column=0, padx=5, pady=5)

        self.pause_btn = ttk.Button(
            control_buttons_frame, text="Pause", command=self.pause_simulation, state=tk.DISABLED, width=10
        )
        self.pause_btn.grid(row=0, column=1, padx=5, pady=5)

        self.stop_btn = ttk.Button(
            control_buttons_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED, width=10
        )
        self.stop_btn.grid(row=0, column=2, padx=5, pady=5)

        self.reset_btn = ttk.Button(control_buttons_frame, text="Reset", command=self.reset_simulation, width=32)
        self.reset_btn.grid(row=1, column=0, columnspan=3, padx=5, pady=5)

        # Speed control
        speed_frame = ttk.LabelFrame(control_frame, text="Simulation Speed")
        speed_frame.pack(side=tk.LEFT, padx=10, pady=5)

        ttk.Label(speed_frame, text="Speed Multiplier:", font=LARGE_FONT).grid(
            row=0, column=0, padx=8, pady=5, sticky="w"
        )
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_scale = ttk.Scale(
            speed_frame,
            from_=0.1,
            to=5.0,
            variable=self.speed_var,
            orient=tk.HORIZONTAL,
            length=180,
            command=self.update_speed,
        )
        speed_scale.grid(row=0, column=1, padx=8, pady=5)

        self.speed_label = ttk.Label(speed_frame, text="1.0x", font=LARGE_FONT)
        self.speed_label.grid(row=0, column=2, padx=8, pady=5)

        # Status information
        status_frame = ttk.LabelFrame(control_frame, text="Simulation Status")
        status_frame.pack(side=tk.RIGHT, padx=10, pady=5)

        self.status_label = ttk.Label(status_frame, text="Status: Ready", font=LARGE_FONT)
        self.status_label.pack(padx=15, pady=8)

        self.step_label = ttk.Label(status_frame, text="Step: 0", font=LARGE_FONT)
        self.step_label.pack(padx=15, pady=8)

        # Plot frame
        self.plot_frame = ttk.Frame(main_frame)
        self.plot_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def setup_plot(self):
        """Setup matplotlib plot"""
        # 设置matplotlib字体大小
        plt.rcParams.update(
            {
                "font.size": 12,
                "axes.titlesize": 16,
                "axes.labelsize": 14,
                "xtick.labelsize": 12,
                "ytick.labelsize": 12,
                "legend.fontsize": 12,
            }
        )

        self.fig = Figure(figsize=(14, 10), dpi=100)
        self.ax = self.fig.add_subplot(111)

        # Embed into tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Initialize empty plot
        self.init_empty_plot()

    def init_empty_plot(self):
        """Initialize empty plot"""
        self.ax.clear()
        self.ax.set_title("Kiva System Simulation - Waiting to Start", fontsize=18, fontweight="bold")
        self.ax.text(
            0.5,
            0.5,
            "Click 'Start' button to begin simulation",
            horizontalalignment="center",
            verticalalignment="center",
            transform=self.ax.transAxes,
            fontsize=16,
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8),
        )
        self.canvas.draw()

    def start_simulation(self):
        """Start simulation"""
        if self.is_running:
            return

        try:
            self.agv_num = int(self.agv_var.get())
            self.order_num = int(self.order_var.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers", parent=self.master)
            return

        self.is_running = True
        self.is_paused = False
        self.current_step = 0

        # Update UI state
        self.start_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Status: Running", foreground="green")

        # Start simulation thread
        self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.simulation_thread.start()

        # Start UI updates
        self.update_display()

    def pause_simulation(self):
        """Pause/resume simulation"""
        if not self.is_running:
            return

        self.is_paused = not self.is_paused

        if self.is_paused:
            self.pause_btn.config(text="Resume")
            self.status_label.config(text="Status: Paused", foreground="orange")
            self.control_queue.put("pause")
        else:
            self.pause_btn.config(text="Pause")
            self.status_label.config(text="Status: Running", foreground="green")
            self.control_queue.put("resume")

    def stop_simulation(self):
        """Stop simulation"""
        if not self.is_running:
            return

        self.is_running = False
        self.is_paused = False

        # Update UI state
        self.start_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED, text="Pause")
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Stopped", foreground="red")

        self.control_queue.put("stop")

    def reset_simulation(self):
        """Reset simulation"""
        if self.is_running:
            self.stop_simulation()

        self.current_step = 0
        self.animation_frames.clear()
        self.step_label.config(text="Step: 0")
        self.init_empty_plot()
        self.status_label.config(text="Status: Ready", foreground="black")

    def update_speed(self, value):
        """Update simulation speed"""
        self.simulation_speed = float(value)
        self.speed_label.config(text=f"{self.simulation_speed:.1f}x")

    def update_plot(self, frame_data):
        """Update matplotlib plot"""
        self.ax.clear()

        # This should contain complete plotting logic
        agv_states = frame_data["agv_states"]

        if agv_states:
            x_coords = [state["x"] for state in agv_states]
            y_coords = [state["y"] for state in agv_states]

            self.ax.scatter(y_coords, x_coords, c="red", s=150, label="AGV", alpha=0.8)

        self.ax.set_title(f"Kiva System Simulation - Step {frame_data['t']}", fontsize=18, fontweight="bold")
        self.ax.set_xlabel("X Coordinate", fontsize=14)
        self.ax.set_ylabel("Y Coordinate", fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(fontsize=12)

        # 设置刻度标签大小
        self.ax.tick_params(labelsize=12)

        self.canvas.draw()

    def run_simulation(self):
        """Run simulation in background thread"""
        try:
            # Mock simulation for demonstration
            t = 1
            while self.is_running and t <= 2000:
                # Check control commands
                while not self.control_queue.empty():
                    command = self.control_queue.get()
                    if command == "stop":
                        return
                    elif command == "pause":
                        while self.is_paused and self.is_running:
                            time.sleep(0.1)

                if not self.is_running:
                    break

                # Mock AGV data
                agv_states = []
                for i in range(self.agv_num):
                    agv_states.append(
                        {
                            "id": i,
                            "x": 10 + i * 2,
                            "y": 10 + (t % 20),
                            "direction": "North",
                            "color": "red",
                            "status": "Moving",
                            "battery": "85%",
                            "target": "Shelf_1",
                        }
                    )

                frame_data = {
                    "agv_states": agv_states,
                    "t": t,
                }

                self.data_queue.put(frame_data)

                sleep_time = 0.2 / self.simulation_speed
                time.sleep(sleep_time)
                t += 1

        except Exception as e:
            print(f"Simulation thread error: {e}")
        finally:
            if self.is_running:
                self.master.after(0, self.stop_simulation)

    def update_display(self):
        """Update display (runs in main thread)"""
        if not self.is_running:
            return

        while not self.data_queue.empty():
            try:
                frame_data = self.data_queue.get_nowait()
                self.animation_frames.append(frame_data)
                self.current_step = frame_data["t"]

                self.update_plot(frame_data)
                self.step_label.config(text=f"Step: {self.current_step}")

            except queue.Empty:
                break

        if self.is_running:
            self.master.after(50, self.update_display)


def main():
    """Main function"""
    root = tk.Tk()

    # 设置窗口大小和位置
    root.geometry("1400x900+100+50")
    root.minsize(1200, 800)

    app = InteractiveSimulation(root)

    def on_closing():
        if app.is_running:
            app.stop_simulation()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
