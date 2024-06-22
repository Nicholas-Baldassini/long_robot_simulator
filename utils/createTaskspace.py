import tkinter as tk


CURR_SHAPE = "Circle"
STATIC = True
def start_taskspace_creator():
    # Robot centering stuff
    muj_robot_center = (0, 0, 0.04)
    width = 800
    height = 800
    muj_area_space = 6 # This means the end of the screen on the tkinter window will have a coordinate of 1
    xscale = muj_area_space / height
    yscale = muj_area_space / width


    coordinates = []
    def draw_shape(event, rad:tk.Scale):
        x, y = event.x, event.y
        radius = rad.get()
        colour = ["black", "blue"][int(STATIC)]
        

        if CURR_SHAPE == "Circle":
            canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=colour)
        else:
            canvas.create_rectangle(x - radius, y-radius, x+radius, y+radius, fill=colour)
            #print("Hello")
            
        coordinates.append((CURR_SHAPE, radius * ((xscale + yscale) / 2), round((x - width // 2 + 10)  * xscale, 4), round((y -  height // 2 + 70) * yscale, 4), STATIC))
        #print(coordinates)


    def toggle_Circle():
        global CURR_SHAPE
        CURR_SHAPE = "Circle"
    def toggle_Square():
        global CURR_SHAPE
        CURR_SHAPE = "Square"
    def toggle_Static():
        global STATIC
        STATIC = not STATIC

    def save_to_file():
        filename = "utils/taskspace.conf"
        with open(filename, "w+") as f:
            for i in coordinates:
                line = f"{i[0]} {i[1]} {i[2]} {i[3]} {i[4]}\n"
                f.write(line)
        print(f"saved to {filename}")



    root = tk.Tk()
    root.title("Taskspace Creater")

    canvas = tk.Canvas(root, bg="white", width=width, height=height)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # Create a right-hand panel frame
    right_panel = tk.Frame(root)
    right_panel.pack(side=tk.RIGHT, fill=tk.Y)

    square_button = tk.Button(right_panel, text="Draw Square", command=toggle_Square)
    square_button.pack(pady=10)
    circle_button = tk.Button(right_panel, text="Draw Circle", command=toggle_Circle)
    circle_button.pack(pady=10)
    static_button = tk.Button(right_panel, text="Toggle static(locked in place)", command=toggle_Static)
    static_button.pack(pady=10)
    save = tk.Button(right_panel, text="Save to file", command=save_to_file)
    save.pack(pady=10)

    radius_bar = tk.Scale(right_panel, from_=5, to=60, orient=tk.HORIZONTAL)
    radius_bar.pack()
    canvas.bind("<Button-1>", lambda event: draw_shape(event, radius_bar))



    canvas.create_rectangle(width // 2, height // 2, width // 2 + 20, height // 2 - 100, fill="red")
    canvas.create_text(360, 15, text="BLACK = CAN MOVE AROUND(non-static), BLUE = LOCKED IN PLACE(static), RED = TDCR ROBOT at (0, 0, 0)", fill="black", font=('Helvetica 10 bold'))


    # Run the Tkinter event loop
    root.mainloop()
