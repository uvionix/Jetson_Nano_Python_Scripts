#!/usr/bin/env python

# The script creates a simple GUI prompt for running patch scripts. The command line arguments to be provided are the
# path to the patch script and the patch name. To run the script use the following command
#
#       python3 <script_name> <path_to_patch> <patch_name>
#
# where
#
#       <script_name> is the name of this script (exapmle: <script_name> = /usr/local/bin/uvx-update-gui.py)
#       <path_to_patch> is the path to the patch script to be installed (exapmle: <path_to_patch> = /home/$usr/Repos/patch_script.sh)
#       <patch_name> is the name of the patch (example: <patch_name> = patch_script)

import tkinter as tk
import subprocess
import sys

# Check the number of command line arguments
num_args = len(sys.argv)

if num_args != 3:
	print("Invalid number of arguments! 3 arguments expected!")
	sys.exit(1)

# Get inputs
path_to_patch = sys.argv[1]
patch_name = sys.argv[2]

# Closes the updater window after the update has been performed
def handle_ok_click():
    window.destroy()

# Closes the updater window when updating is rejected by user or the update has failed
def handle_no_click():
    window.destroy()
    sys.exit(1)

# Initiates the update
def handle_yes_click():
    button_no.pack_forget()
    button_yes.destroy()
    label["text"] = "Installing patch..."
    frame_label.update()
    rcode = subprocess.call(['sh', path_to_patch])

    if rcode != 0:
        label["text"] = "Installation failed!"
        frame_label.update()
        button_no["text"] = "OK"
        button_no.pack()
    else:
        label["text"] = "Installation successful!"
        frame_label.update()
        button_ok.pack()

# Create the updater window and get screen resolution
window = tk.Tk()
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()
window.title("UVX updater")

# Create the label frame and the progress label
frame_label = tk.Frame()
label = tk.Label(master=frame_label, text="   New patch %s found. Install?   " % (patch_name), fg="black", bg="white", font=18)
label.pack()

# Create the buttons frame and the buttons
frame_buttons = tk.Frame()
button_yes = tk.Button(master=frame_buttons, text="Yes!", width=25, height=5, bg="purple", fg="yellow", font=14, command=handle_yes_click)
button_no = tk.Button(master=frame_buttons, text="No!", width=25, height=5, bg="purple", fg="yellow", font=14, command=handle_no_click)
button_ok = tk.Button(master=frame_buttons, text="OK", width=25, height=5, bg="purple", fg="yellow", font=14, command=handle_ok_click)
button_yes.pack(side=tk.LEFT)
button_no.pack(side=tk.LEFT)
frame_label.pack()
frame_buttons.pack()

# Configure window geometry
window_width = button_yes.winfo_reqwidth() * 2
window_height = button_yes.winfo_reqheight() + label.winfo_reqheight()
wxoffset = ( screen_width - window_width ) / 2
wyoffset = ( screen_height - window_height ) / 2
window.geometry('%dx%d+%d+%d' %(window_width, window_height, wxoffset, wyoffset))

# Start main loop
window.mainloop()
