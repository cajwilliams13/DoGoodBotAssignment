import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread

class RobotControlGUI(tk.Tk):
    def __init__(self, r1=None, r2=None, plates=None):
        super().__init__()

        self.robot_1 = r1
        self.robot_2 = r2
        self.plates = plates

        self.title("Robot Control Interface")
        self.estop_active = True

        # Status message
        self.status_var = tk.StringVar(value="NONE")
        ttk.Label(self, textvariable=self.status_var).grid(row=3, column=0, columnspan=2, pady=10)

        # Create frame for Production cell control
        prod_cell_frame = ttk.LabelFrame(self, text="Production Cell Control")
        prod_cell_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # E-STOP Button
        self.estop_btn = ttk.Button(prod_cell_frame, text="E-STOP", command=self.toggle_estop)
        self.estop_btn.grid(row=0, column=1, padx=5, pady=5)

        # Start Button
        self.start_btn = ttk.Button(prod_cell_frame, text="Start", command=self.start, state=tk.DISABLED)
        self.start_btn.grid(row=0, column=2, padx=5, pady=5)

        # Toggle Mode Button
        self.mode = tk.StringVar(value="Mode: Joint")
        self.toggle_mode_btn = ttk.Button(prod_cell_frame, textvariable=self.mode, command=self.toggle_mode)
        self.toggle_mode_btn.grid(row=0, column=0, padx=5, pady=5)

        self.create_robot_interface()

        # Create frame for Printer Plate control
        self.printer_plate_frame = ttk.LabelFrame(self, text="Printer Plate Control")
        self.printer_plate_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.plate_status_vars = [tk.StringVar(value=self.query_plate_status(i)) for i in range(8)]
        self.create_printer_plate_controls()

        self.axis_vals = {}
        self.rot_vals = {}

    def update_status(self, new_status):
        """Updates the status message with the given text."""
        self.status_var.set(new_status)

    def create_robot_interface(self):
        if "Joint" in self.mode.get():
            self.create_joint_controls("Robot Joint Control 1", 1, 6, 1, 0)
            self.create_joint_controls("Robot Joint Control 2", 2, 3, 2, 0)
        else:
            self.create_xyz_controls("Robot XYZ Control 1", 1, 0, 1)
            self.create_xyz_controls("Robot XYZ Control 2", 2, 0, 2)

        # Disable robot control panels if E-STOP is pressed
        if self.estop_active:
            self.disable_controls()

    def create_joint_controls(self, title, robot_num, joints, col, row):
        joint_frame = ttk.LabelFrame(self, text=title)
        joint_frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        for i in range(joints):
            ttk.Label(joint_frame, text=f"Joint {i+1}:").grid(row=i, column=0, padx=5, pady=5)
            #ttk.Label(joint_frame, text=str(self.get_joint_value(robot_num, i))).grid(row=i, column=1, padx=5, pady=5)
            ttk.Button(joint_frame, text="+", command=lambda i=i: self.move_joint(robot_num, i, "+")).grid(row=i, column=2, padx=5, pady=5)
            ttk.Button(joint_frame, text="-", command=lambda i=i: self.move_joint(robot_num, i, "-")).grid(row=i, column=3, padx=5, pady=5)

        ttk.Button(joint_frame, text="Reset", command=self.reset).grid(row=joints, column=0, padx=5, pady=5)
        ttk.Button(joint_frame, text="Toggle gripper", command=self.toggle_gripper).grid(row=joints, column=1, columnspan=3, padx=5, pady=5)

    def create_xyz_controls(self, label, robot_num, row, col):
        frame = ttk.LabelFrame(self, text=label)
        frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        # XYZ labels, readouts, and control buttons
        for index, axis in enumerate(['X', 'Y', 'Z']):
            ttk.Label(frame, text=axis).grid(row=index, column=0, padx=5, pady=5)

            # Update the StringVar dictionary structure
            if robot_num not in self.axis_vals:
                self.axis_vals[robot_num] = {}
            self.axis_vals[robot_num][axis] = tk.StringVar(value=str(self.get_pos_value(robot_num, axis)))
            ttk.Label(frame, textvariable=self.axis_vals[robot_num][axis]).grid(row=index, column=1, padx=5, pady=5)
            ttk.Button(frame, text=f"+ {axis}").grid(row=index, column=2, padx=5, pady=5)
            ttk.Button(frame, text=f"- {axis}").grid(row=index, column=3, padx=5, pady=5)

        # Rotation labels, readouts, and control buttons
        for index, axis in enumerate(['RX', 'RY', 'RZ']):
            ttk.Label(frame, text=axis).grid(row=index + 3, column=0, padx=5, pady=5)

            # Update the StringVar dictionary structure
            if robot_num not in self.rot_vals:
                self.rot_vals[robot_num] = {}
            self.rot_vals[robot_num][axis] = tk.StringVar(value=str(self.get_rot_value(robot_num, axis)))
            ttk.Label(frame, textvariable=self.rot_vals[robot_num][axis]).grid(row=index + 3, column=1, padx=5, pady=5)
            ttk.Button(frame, text=f"+ {axis}").grid(row=index + 3, column=2, padx=5, pady=5)
            ttk.Button(frame, text=f"- {axis}").grid(row=index + 3, column=3, padx=5, pady=5)

        # Reset and Toggle Gripper buttons
        ttk.Button(frame, text="Reset", command=lambda: self.reset(robot_num)).grid(row=6, column=2, padx=5, pady=5)
        ttk.Button(frame, text="Toggle Gripper", command=lambda: self.toggle_gripper(robot_num)).grid(row=6, column=3,
                                                                                                      padx=5, pady=5)

    def create_robot_frame(self, title, joints, col, row, joint_values_list):
        robot_frame = ttk.LabelFrame(self, text=title)
        robot_frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        for i in range(joints):
            ttk.Label(robot_frame, text=f"Joint {i+1}").grid(row=i, column=0, padx=5, pady=5)
            ttk.Label(robot_frame, textvariable=joint_values_list[i]).grid(row=i, column=1, padx=5, pady=5)
            ttk.Button(robot_frame, text="+", command=lambda i=i: self.move_joint(i, "+", joint_values_list)).grid(row=i, column=2, padx=5, pady=5)
            ttk.Button(robot_frame, text="-", command=lambda i=i: self.move_joint(i, "-", joint_values_list)).grid(row=i, column=3, padx=5, pady=5)

        ttk.Button(robot_frame, text="Reset", command=lambda: self.reset(joint_values_list)).grid(row=joints, column=0, padx=5, pady=5)
        ttk.Button(robot_frame, text="Toggle gripper", command=self.toggle_gripper).grid(row=joints, column=1, columnspan=3, padx=5, pady=5)

    def move_joint(self, id, joint, direction):
        r = self.robot_1 if id == 0 else self.robot_2

        if direction == "+":
            r.tweak(joint, 5)
        else:
            r.tweak(joint, -5)

    def reset(self, joint_values_list):
        # Dummy code to reset the robot here
        for value in joint_values_list:
            value.set("0")

    def toggle_gripper(self, robot_num):
        # Dummy code to toggle gripper here
        print("Toggling gripper")

    def start(self):
        if self.estop_active:
            messagebox.showerror("Error", "Please release the E-STOP first!")
        else:
            print("Starting...")

    def toggle_mode(self):
        # Remove previous interface
        for child in self.winfo_children():
            if isinstance(child, ttk.LabelFrame) and "Robot" in child.cget("text"):
                child.destroy()

        # Switch mode
        if "Joint" in self.mode.get():
            self.mode.set("Mode: XYZ")
        else:
            self.mode.set("Mode: Joint")

        self.create_robot_interface()

    def get_joint_value(self, robot_num, joint):
        if robot_num == 0:
            return round(self.robot_1.arm.q[joint], 4)
        if robot_num == 1:
            return round(self.robot_2.arm.q[joint], 4)
        return 0

    def get_pos_value(self, robot_num, axes):
        return 0

    def get_rot_value(self, robot_num, axes):
        return 0

    def disable_controls(self):
        for child in self.winfo_children():
            if isinstance(child, ttk.LabelFrame) and "Production Cell Control" not in child.cget("text"):
                for widget in child.winfo_children():
                    widget.configure(state=tk.DISABLED)

    def toggle_estop(self):
        if self.estop_active:
            self.estop_active = False
            self.estop_btn.config(text="E-STOP")
            self.start_btn.config(state=tk.NORMAL)
            for child in self.winfo_children():
                if isinstance(child, ttk.LabelFrame):
                    for widget in child.winfo_children():
                        widget.configure(state=tk.NORMAL)
        else:
            self.estop_active = True
            self.estop_btn.config(text="Release E-STOP")
            self.disable_controls()

    def create_printer_plate_controls(self):
        printer_frame = ttk.LabelFrame(self, text="Printer Plates Control")
        printer_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        for i in range(8):
            ttk.Label(printer_frame, text=f"Plate {i + 1}:").grid(row=i, column=0, padx=5, pady=5)
            ttk.Button(printer_frame, text="Spawn", command=lambda i=i: self.spawn_plate(i)).grid(row=i, column=1,
                                                                                                  padx=5, pady=5)
            ttk.Button(printer_frame, text="Remove", command=lambda i=i: self.remove_plate(i)).grid(row=i, column=2,
                                                                                                    padx=5, pady=5)
            ttk.Label(printer_frame, textvariable=self.plate_status_vars[i]).grid(row=i, column=3, padx=5, pady=5)

    def spawn_plate(self, plate_id):
        print(f"Spawn plate {plate_id + 1}")  # replace with the actual function
        if not any([p == "Waiting" for p in self.plates[plate_id]]):
            self.plates[plate_id] = "Waiting"
        else:
            print("Plate already waiting")

    def remove_plate(self, plate_id):
        print(f"Remove plate {plate_id + 1}")  # replace with the actual function
        self.plates[plate_id] = "Absent"

    def query_plate_status(self, plate_id):
        # Dummy function to return status. Replace with the actual function.
        return "Absent"




def run_gui_in_thread(**kargs):
    app = RobotControlGUI(**kargs)
    app.mainloop()

if __name__ == "__main__":
    gui_thread = Thread(target=run_gui_in_thread)
    gui_thread.start()

    # Add your simulation or other logic here
