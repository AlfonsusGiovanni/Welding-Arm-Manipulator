from library import trajectory
from library import pattern_generator

import turtle
import numpy as np

trajectory1 = trajectory.Trajectory()
pg = pattern_generator.Pattern_Generator()

screen = turtle.Screen()
screen.setup(800, 400)
screen.title("Welding Pattern Simulation")

draw = turtle.Turtle()
draw.speed(10)

trajectory1.set_iteration_point(-250, 250, 0, 0)
pg.set_pattern(pg.pattern_type.WAVE_TYPE, 100)

draw.teleport(-300, 150)
draw.forward(600)
draw.right(90)
draw.forward(300)
draw.right(90)
draw.forward(600)
draw.right(90)
draw.forward(300)

draw.penup()
iteration_counter = 0

for t in np.arange(0, 1+pg.iteration, pg.iteration):
    X_pos = trajectory1.calculate(0, t)
    Y_pos = pg.get_pattern_point(20, iteration_counter)

    draw.goto(X_pos, Y_pos)
    draw.pendown()
    draw.teleport(X_pos, Y_pos)
    draw.dot(5, "red")

    iteration_counter += 1

draw.hideturtle()
screen.mainloop()