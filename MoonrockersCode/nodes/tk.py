from Tkinter import *

master = Tk()
master.geometry("400x400")
w = Canvas(master, width=300, height=300)
w.pack()


#       StartX, StartY, EndX, EndY                   
w.create_rectangle(100, 25, 250, 150)
w.create_rectangle(100, 25, 250, 75)
w.create_rectangle(100, 25, 250, 200)
w.create_rectangle(160, 200, 190, 210)
#                        
crowLine = w.create_line(100, 25, 175, 200, fill="red", dash=(4,4))



mainloop()
