import tkinter as tk
from threading import Thread
from tkinter import messagebox, ttk


class RobotControlGUI(tk.Tk):
    def __init__(self, r1=None, r2=None, plates=None, robot_can_move=None, obstructions=None, estop=None, simulation_running=None):
        super().__init__()

        # Capture shared variables
        self.robot_1 = r1
        self.robot_2 = r2
        self.plates = plates
        self.estop = estop
        self.simulation_running = simulation_running
        self.obstructions = obstructions

        self.title("Robot Control Interface")

        # Create frame for Production cell control
        prod_cell_frame = ttk.LabelFrame(self, text="Production Cell Control")
        prod_cell_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # E-STOP Button
        self.estop_btn = ttk.Button(prod_cell_frame, text="E-STOP", command=self.toggle_estop)
        self.estop_btn.grid(row=0, column=1, padx=5, pady=5)

        # Start Button
        self.start_btn = ttk.Button(prod_cell_frame, text="Start", command=self.start, state=tk.DISABLED)
        self.start_btn.grid(row=0, column=2, padx=5, pady=5)

        self.create_robot_interface()

        # Create frame for Printer Plate control
        self.printer_plate_frame = ttk.LabelFrame(self, text="Printer Plate Control")
        self.printer_plate_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.create_printer_plate_controls()

    def create_robot_interface(self):
        self.create_joint_controls("Robot Joint Control 1", 1, 6, 1, 0)
        self.create_joint_controls("Robot Joint Control 2", 2, 3, 2, 0)

        # Disable robot control panels if E-STOP is pressed
        if self.estop.get_state():
            self.disable_controls()

    def create_joint_controls(self, title, robot_num, joints, col, row):
        joint_frame = ttk.LabelFrame(self, text=title)
        joint_frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        for i in range(joints):
            ttk.Label(joint_frame, text=f"Joint {i + 1}:").grid(row=i, column=0, padx=5, pady=5)
            ttk.Button(joint_frame, text="+", command=lambda q=i:
                                    self.move_joint(robot_num, q, "+")).grid(row=i, column=2, padx=5, pady=5)

            ttk.Button(joint_frame, text="-", command=lambda q=i:
                                    self.move_joint(robot_num, q, "-")).grid(row=i, column=3, padx=5, pady=5)

    def create_robot_frame(self, title, joints, col, row, joint_values_list):
        robot_frame = ttk.LabelFrame(self, text=title)
        robot_frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        for i in range(joints):
            ttk.Label(robot_frame, text=f"Joint {i + 1}").grid(row=i, column=0, padx=5, pady=5)
            ttk.Label(robot_frame, textvariable=joint_values_list[i]).grid(row=i, column=1, padx=5, pady=5)
            ttk.Button(robot_frame, text="+", command=lambda q=i: self.move_joint(q, "+", joint_values_list)).grid(
                row=i, column=2, padx=5, pady=5)
            ttk.Button(robot_frame, text="-", command=lambda q=i: self.move_joint(q, "-", joint_values_list)).grid(
                row=i, column=3, padx=5, pady=5)

    def move_joint(self, joint_id, joint, direction):
        r = self.robot_1 if joint_id == 1 else self.robot_2

        if direction == "+":
            r.tweak(joint, 5)
        else:
            r.tweak(joint, -5)

    def start(self):
        if self.estop.get_state():
            messagebox.showerror("Error", "Please release the E-STOP first!")
        self.simulation_running.set()
        

    def disable_controls(self):
        for child in self.winfo_children():
            # Do not disable the estop button :P
            if isinstance(child, ttk.LabelFrame) and "Production Cell Control" not in child.cget("text"):
                for widget in child.winfo_children():
                    widget.configure(state=tk.DISABLED)

    def toggle_estop(self):
        estop_active = self.estop.get_state()

        if estop_active:
            self.estop.release()
            self.estop_btn.config(text="E-STOP")
            self.start_btn.config(state=tk.NORMAL)

            # Enable all children nodes
            for child in self.winfo_children():
                if isinstance(child, ttk.LabelFrame):
                    for widget in child.winfo_children():
                        widget.configure(state=tk.NORMAL)
        else:
            self.estop.press()
            self.estop_btn.config(text="Release E-STOP")
            self.disable_controls()

    def create_printer_plate_controls(self):
        printer_frame = ttk.LabelFrame(self, text="Printer Plates Control")
        printer_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        for i in range(8):
            ttk.Label(printer_frame, text=f"Plate {i + 1}:").grid(row=i, column=0, padx=5, pady=5)
            ttk.Button(printer_frame, text="Spawn", command=lambda q=i: self.spawn_plate(q)).grid(row=i, column=1,
                                                                                                  padx=5, pady=5)

            ttk.Button(printer_frame, text="Remove", command=lambda q=i: self.remove_plate(q)).grid(row=i, column=2,
                                                                                                    padx=5, pady=5)

            ttk.Button(printer_frame, text="Obstruct", command=lambda q=i: self.obstruct_cell(q)).grid(row=i, column=3,
                                                                                                       padx=5, pady=5)

    def spawn_plate(self, plate_id):
        print(f"Spawn plate {plate_id + 1}")
        if not any([p == "Waiting" for p in self.plates[plate_id]]):
            self.plates[plate_id] = "Waiting"
        else:
            print("Plate already waiting")

    def remove_plate(self, plate_id):
        print(f"Remove plate {plate_id + 1}")
        self.plates[plate_id] = "Absent"

    def obstruct_cell(self, cell_id):
        self.obstructions[cell_id] = not self.obstructions[cell_id]
        print(self.obstructions)


def run_gui_in_thread(exit_event, **kwargs):
    app = RobotControlGUI(**kwargs)
    while True:
        app.update_idletasks()
        app.update()
        if exit_event.is_set():
            break

if __name__ == "__main__":
    gui_thread = Thread(target=run_gui_in_thread)
    gui_thread.start()
