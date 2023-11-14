import tkinter as tk
from threading import Thread
from tkinter import StringVar, messagebox, ttk


class RobotControlGUIV2():
    def __init__(self, r1=None, r2=None, plates=None, robot_can_move=None, obstructions=None, estop=None, simulation_running=None):
        self.tk = tk.Tk()

        # Capture shared variables
        self.robot_1 = r1
        self.robot_2 = r2
        self.plates = plates
        self.estop = estop
        self.simulation_running = simulation_running
        self.obstructions = obstructions

        self.tk.title("Robot Control Interface")

        # Create frame for Production cell control
        prod_cell_frame = ttk.LabelFrame(master=self.tk, text="Production Cell Control")
        prod_cell_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # E-STOP Button
        self.estop_btn_text = StringVar(value="Press E-STOP")
        self.estop_btn = ttk.Button(master=prod_cell_frame, textvariable=self.estop_btn_text, command=self.toggle_estop)
        self.estop_btn.grid(row=0, column=1, padx=5, pady=5)

        # Start Button
        self.start_btn = ttk.Button(master=prod_cell_frame, text="Start", command=self.toggle_start, state=tk.DISABLED)
        self.start_btn.grid(row=0, column=2, padx=5, pady=5)

        self.estop_status_string = StringVar(value="UNKNOWN")

        self.estop_status_label = ttk.Label(master=prod_cell_frame, textvariable=self.estop_status_string)
        self.estop_status_label.grid(row=1, column=1, padx=5, pady=5)


        self.simulation_status_string = StringVar(value="UNKNOWN")

        self.simulation_status_label = ttk.Label(master=prod_cell_frame, textvariable=self.simulation_status_string)
        self.simulation_status_label.grid(row=1, column=2, padx=5, pady=5)


        self.create_robot_interface()

        # Create frame for Printer Plate control
        self.printer_plate_frame = ttk.LabelFrame(master=self.tk, text="Printer Plate Control")
        self.printer_plate_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.create_printer_plate_controls()
        self.refresh_data() # Call only once

    def create_robot_interface(self):
        self.create_joint_controls("Robot Joint Control 1", 1, 6, 1, 0)
        self.create_joint_controls("Robot Joint Control 2", 2, 3, 2, 0)

        # Disable robot control panels if E-STOP is pressed
        if self.estop.get_state():
            self.disable_controls()

    def create_joint_controls(self, title, robot_num, joints, col, row):
        joint_frame = ttk.LabelFrame(master=self.tk, text=title)
        joint_frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        for i in range(joints):
            ttk.Label(master=joint_frame, text=f"Joint {i + 1}:").grid(row=i, column=0, padx=5, pady=5)
            ttk.Button(master=joint_frame, text="+", command=lambda q=i:
                                    self.move_joint(robot_num, q, "+")).grid(row=i, column=2, padx=5, pady=5)

            ttk.Button(master=joint_frame, text="-", command=lambda q=i:
                                    self.move_joint(robot_num, q, "-")).grid(row=i, column=3, padx=5, pady=5)

    def create_robot_frame(self, title, joints, col, row, joint_values_list):
        robot_frame = ttk.LabelFrame(master=self.tk, text=title)
        robot_frame.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        for i in range(joints):
            ttk.Label(master=robot_frame, text=f"Joint {i + 1}").grid(row=i, column=0, padx=5, pady=5)
            ttk.Label(master=robot_frame, textvariable=joint_values_list[i]).grid(row=i, column=1, padx=5, pady=5)
            ttk.Button(master=robot_frame, text="+", command=lambda q=i: self.move_joint(q, "+", joint_values_list)).grid(
                row=i, column=2, padx=5, pady=5)
            ttk.Button(master=robot_frame, text="-", command=lambda q=i: self.move_joint(q, "-", joint_values_list)).grid(
                row=i, column=3, padx=5, pady=5)

    def move_joint(self, joint_id, joint, direction):
        r = self.robot_1 if joint_id == 1 else self.robot_2

        if direction == "+":
            r.tweak(joint, 5)
        else:
            r.tweak(joint, -5)

    def toggle_start(self):
        if self.estop.get_state():
            messagebox.showerror("Error", "Please release the E-STOP first!")
            return
        if self.simulation_running.is_set():
            self.simulation_running.clear()
        else:
            self.simulation_running.set()
        

    def toggle_estop(self):
        estop_active = self.estop.get_state()

        if estop_active:
            self.estop.release()
        else:
            self.estop.press()

    def create_printer_plate_controls(self):
        printer_frame = ttk.LabelFrame(master=self.tk, text="Printer Plates Control")
        printer_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        for i in range(8):
            ttk.Label(master=printer_frame, text=f"Plate {i + 1}:").grid(row=i, column=0, padx=5, pady=5)
            ttk.Button(master=printer_frame, text="Spawn", command=lambda q=i: self.spawn_plate(q)).grid(row=i, column=1,
                                                                                                  padx=5, pady=5)

            ttk.Button(master=printer_frame, text="Remove", command=lambda q=i: self.remove_plate(q)).grid(row=i, column=2,
                                                                                                    padx=5, pady=5)

            ttk.Button(master=printer_frame, text="Obstruct", command=lambda q=i: self.obstruct_cell(q)).grid(row=i, column=3,
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

    def refresh_data(self):
        # Note that ttk is meant to be event driven (I think?)
        # This is just polling, not ideal but it works
        if self.estop.get_state():
            self.estop_status_string.set("PRESSED")
            self.estop_btn_text.set("Release E-STOP")
            self.start_btn.config(state=tk.DISABLED)
            for child in self.tk.winfo_children():
                # Do not disable the estop button :P
                if isinstance(child, ttk.LabelFrame) and "Production Cell Control" not in child.cget("text"):
                    for widget in child.winfo_children():
                        widget.configure(state=tk.DISABLED)
        else:
            self.estop_status_string.set("RELEASED")
            self.estop_btn_text.set("Press E-STOP")
            self.start_btn.config(state=tk.NORMAL)
            # Enable all children nodes
            for child in self.tk.winfo_children():
                if isinstance(child, ttk.LabelFrame):
                    for widget in child.winfo_children():
                        widget.configure(state=tk.NORMAL)

        if self.simulation_running.is_set():
            self.simulation_status_string.set("RUNNING")
            self.start_btn.config(text="Stop")
        else:
            self.simulation_status_string.set("STOPPED")
            self.start_btn.config(text="Start")

        self.tk.after(500, self.refresh_data) # Continue refreshing every 500 ms


def run_gui_in_thread_v2(exit_event, **kwargs):
    robot_control_gui = RobotControlGUIV2(**kwargs)
    while True:
        robot_control_gui.tk.update_idletasks()
        robot_control_gui.tk.update()
        if exit_event.is_set():
            break
