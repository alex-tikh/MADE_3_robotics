from tkinter import *
import math
import heapq, random
import time
from sympy import Point, Polygon
import numpy as np

'''================= Your classes and methods ================='''

def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new+cx, y_new+cy))

    return new_points

def get_polygon_from_position(position) :
    x,y,yaw = position
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100)] 
    new_points = rotate(points, yaw * 180 / math.pi, (x,y))
    return Polygon(*list(map(Point, new_points)))

def get_polygon_from_obstacle(obstacle) :
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])] 
    return Polygon(*list(map(Point, points)))


def poly_collides(poly_lhs, poly_rhs) :
    return poly_lhs.intersection(poly_rhs) or \
        True in [poly_lhs.encloses_point(p) for p in poly_rhs.vertices]

def collides(position, obstacle) :
    return poly_collides(get_polygon_from_position(position),
                         get_polygon_from_obstacle(obstacle))
        

class Window():
        
    '''================= Your Main Function ================='''
    
    
    def go(self, event):
        
        self.delete_path()
        start_pos = self.get_start_position()
        path = self.get_a_star_path([start_pos], self.cost_function, 60, 0., 700, 600)
        path2 = self.get_a_star_path(path, self.cost_function, 20, 350., 0, 50)
        path = path + path2
        self.path_object_ids = self.draw_result(path)


    def get_a_star_path(self, path, cost_function, step_size, angle_weight, angle_weight_2, thr):
        
        heap_list = []
        history = set()
        path = path
        
        # start_pos = self.get_start_position()
        # print(start_pos)
        # path.append(start_pos)
        
        cost = cost_function(path[-1], angle_weight, angle_weight_2)
        # use prior distance as tie-breaker
        prior = (len(path) - 1) * step_size
        entry = (cost, prior, path)
        heapq.heappush(heap_list, entry)

        while len(heap_list) != 0:
            _, _, path = heapq.heappop(heap_list)
            
            if cost_function(path[-1], angle_weight, angle_weight_2) < thr:
                print('target is found (according to cost function)')
                break

            for next_pos in self.get_next_positions(path[-1], step_size):
                
                if (round(next_pos[0], 1), round(next_pos[1], 1), round(next_pos[2], 1)) in history:
                    continue
                    
                path_next = path.copy()
                path_next.append(next_pos)
                history.add((round(next_pos[0], 1), round(next_pos[1], 1), round(next_pos[2], 1)))
                prior = (len(path_next) - 1) * step_size

                cost = cost_function(next_pos, angle_weight, angle_weight_2)
                entry = (prior + cost, prior, path_next)
                heapq.heappush(heap_list, entry)
                

            self.draw_path(path)
        return path

    def cost_function(self, position, angle_weight, angle_weight_2):
        
        target_x, target_y, target_yaw = self.get_target_position()
        x, y, yaw = position
        x_diff = target_x - x
        y_diff = target_y - y
        distance = math.sqrt(x_diff**2 + y_diff**2)
            
        rotation = abs(target_yaw - yaw)
        
        if rotation > math.pi:
            rotation = abs(rotation - 2 * math.pi)
            
        path_yaw = math.atan(abs(x_diff / y_diff))
        if (x_diff >= 0) and (y_diff >= 0):
            path_yaw = math.pi - path_yaw
        elif (x_diff <= 0) and (y_diff >= 0):
            path_yaw = path_yaw - math.pi
        elif (x_diff >= 0) and (y_diff <= 0):
            path_yaw = path_yaw
        elif (x_diff <= 0) and (y_diff <= 0):
            path_yaw = -path_yaw
            
        rotation_2 = abs(path_yaw - target_yaw)
        
        if rotation_2 > math.pi:
            rotation_2 = abs(rotation_2 - 2 * math.pi)
        
        return distance  + angle_weight * rotation + angle_weight_2 * rotation_2


    def get_next_positions(self, position, step_size):
        
        x, y, yaw = position
        angle = math.pi * step_size / 180 / 3
        
        add_angle = [-angle, 0., angle]
        
        next_points = []
        for angle in add_angle:
            yaw_new = yaw + angle
            x_new = x + step_size * math.sin(yaw_new)
            y_new = y - step_size * math.cos(yaw_new)

            collision = False
            for obstacle in self.get_obstacles():
                if collides([x_new, y_new, yaw_new], obstacle) :
                    collision = True
                    break
            if collision:
                continue
            
            next_points.append([x_new, y_new, yaw_new])
        return next_points

    def draw_path(self, path):
        
        ids = []
        for point in path:
            id = self.canvas.create_oval(point[0] - 5, point[1] - 5, point[0] + 5, point[1] + 5, fill='red')
            ids.append(id)
        self.root.update()
        for id in ids:
            self.canvas.delete(id)

    def draw_result(self, path):
        
        ids = []
        for point in path:
            id = self.canvas.create_oval(point[0] - 5, point[1] - 5, point[0] + 5, point[1] + 5, fill='green')
            ids.append(id)
        self.root.update()
        return ids
    
    def delete_path(self):
        
        for id in self.path_object_ids:
            self.canvas.delete(id)
        self.path_object_ids = []

        
    '''================= Interface Methods ================='''
    
    def get_obstacles(self) :
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2) :
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles
            
            
    def get_start_position(self) :
        x,y = self.get_center(2) # Purple block has id 2
        yaw = self.get_yaw(2)
        return x,y,yaw
    
    def get_target_position(self) :
        x,y = self.get_center(1) # Green block has id 1 
        yaw = self.get_yaw(1)
        return x,y,yaw 
 

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2])/2
        end_y = (points[1] + points[3])/2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y 
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0 ) :
            return math.acos(cos_yaw)
        else :
            return -math.acos(cos_yaw)
       
    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new+cx)
            new_points.append(y_new+cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0: angle = math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))
        elif wx - x < 0: angle = -math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0,0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width/2)
        self.create_purple_block(self.width/2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()
        
    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width  = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry('{}x{}'.format(self.width, self.height))
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()

        self.path_object_ids = []
    
if __name__ == "__main__":
    run = Window()
    run.run()
