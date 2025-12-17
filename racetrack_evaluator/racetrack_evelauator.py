import glob
import os.path

import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing, LineString
import textwrap

from deepracer.logs import \
    PlottingUtils as pu


def parametric_circle(t, xc, yc, R):
    x = xc + R * np.cos(t)
    y = yc + R * np.sin(t)
    return x, y


def inv_parametric_circle(x, xc, R):
    t = np.arccos(min((x - xc) / R,1.0))
    return t


def create_circle(c, start, end, R, N):
    start_t = inv_parametric_circle(start[0], c[0], R)
    if start[1] < c[1]:
        start_t = start_t + np.pi
    end_t = inv_parametric_circle(end[0], c[0], R)
    if end[1] < c[1]:
        end_t += np.pi

    if start_t > end_t:
        start_t -= 2 * np.pi

    arc_T = np.linspace(start_t, end_t, N)
    return np.vstack(parametric_circle(arc_T, c[0], c[1], R)).transpose()

def create_circle_angle(c, start, angle, R, N):
    start_t = inv_parametric_circle(start[0], c[0], R)
    if start[1] < c[1]:
        start_t = start_t + np.pi

    end_t = start_t + angle

    arc_T = np.linspace(start_t, end_t, N)
    return np.vstack(parametric_circle(arc_T, c[0], c[1], R)).transpose()

def print_border(ax, waypoints, inner_border_waypoints, outer_border_waypoints):
    
    fig = plt.figure(1, figsize=(16, 10))
    ax = fig.add_subplot(111, facecolor='black')
    plt.axis('equal')

    line = LineString(waypoints)                                                
    pu._plot_coords(ax, line)                                                       
    pu._plot_line(ax, line)                                                         
                                                                                
    line = LineString(inner_border_waypoints)                                   
    pu._plot_coords(ax, line)                                                       
    pu._plot_line(ax, line)                                                         
                                                                                
    line = LineString(outer_border_waypoints)                                   
    pu._plot_coords(ax, line)                                                       
    pu._plot_line(ax, line)  

    plt.show()

       

TARGET_TRACK_NAME = 'Trapezoid'
CENTER_LINE_WIDTH = 25
OUTER_LINE_WIDTH = 75
TRACK_WIDTH = 760
SIZE_ADJUST = -50
START_OFFSET = 0.1

def main():
    # Convert to Shapely objects
  points = np.array([[0.7, 0.0],
        [3.8, 0.0],
        [4.5, 0.7],
        [4.5, 1.0],
        [3.8, 2.2],
        [0.7, 3.0],
        [0.0, 2.7],
        [0.0, 0.7]])

  R_1 = 0.7
  N_1 = 15
  curve_1 = create_circle((points[0,0],points[7,1]), points[7], points[0], R_1, N_1)
  curve_2 = create_circle_angle((points[1,0],points[2,1]), points[1], np.pi/2, R_1, N_1)
  curve_3 = create_circle_angle((3.8, 1.0), points[3], np.pi/2-0.55, R_1, N_1)
  curve_4 = np.flipud(create_circle_angle((points[5,0],points[6,1]), points[6], -np.pi/2-0.55, R_1, N_1))

  center_line = np.vstack([[0,2],curve_1,curve_2,curve_3,curve_4,[0,2]])
  center_line = [line*6 + [-8.0, -8.0] for line in center_line]
  print(center_line)
  l_center_line = LineString(center_line)

  # road_poly = Polygon(np.vstack((outer_border, np.flipud(inner_border))))
  print("Is loop/ring? ", l_center_line.is_ring)
  print("Length: {:0.2f}".format(l_center_line.length))

  fig = plt.figure(1, figsize=(10, 10))
  ax = fig.add_subplot(111, facecolor='black')
  plt.axis('equal')

  # center_line_loop = np.vstack([center_line, outer_border[-7], inner_border[-6], inner_border[68], center_line[0]])
  print_border(ax, center_line, None, None)

if __name__ == "__main__":
    main()

    