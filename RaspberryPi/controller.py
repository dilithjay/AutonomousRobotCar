from tkinter import *
from MovementModule.movement import Movement


class Controller:
    def __init__(self):
        self.mv = Movement(0, 0)

        window = Tk()
        window.title("Controller")
        window.geometry("480x320")

        self.turn_label = Label(window, text="Turn amount: 0")
        self.turn_label.place(x=40, y=60)

        self.sensitivity_label = Label(window, text="Sensitivity:")
        self.sensitivity_label.place(x=40, y=100)

        self.sensitivity = Entry(window, width=30)
        self.sensitivity.place(x=150, y=100)
        self.sensitivity.insert(END, "5")

        self.speed_label = Label(window, text="Status: Stopped")
        self.speed_label.place(x=40, y=140)

        self.speed_val = Entry(window, width=30)
        self.speed_val.place(x=150, y=140)
        self.speed_val.insert(END, "180")

        self.running = False

        # Bind canvas with key events
        window.bind("<Up>", self.up)
        window.bind("<Down>", self.down)
        window.bind("<Left>", self.left)
        window.bind("<Right>", self.right)

        window.mainloop()  # Create an event loop

    def up(self, event):
        if not self.running:
            self.mv.set_speed(int(self.speed_val.get()))
            self.speed_label["text"] = "Status: Running"
            self.mv.apply_speeds()
            self.running = True

    def down(self, event):
        if self.running:
            self.mv.reset_speeds()
            self.speed_label["text"] = "Status: Stopped"
            self.running = False

    def left(self, event):
        self.turn_label["text"] = "Turn amount: " + str(self.mv.change_turn_amount(-int(self.sensitivity.get())))

    def right(self, event):
        self.turn_label["text"] = "Turn amount: " + str(self.mv.change_turn_amount(int(self.sensitivity.get())))


if __name__ == "__main__":
    Controller()
