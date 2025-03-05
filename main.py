from kivy.app import App
from kivy.config import Config
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.image import Image
from kivy.uix.button import Button
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy_garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
import osmnx as ox
import matplotlib.pyplot as plt
from pyrosm import OSM
import datetime
import os
import copy
import pickle
from kivy.uix.scrollview import ScrollView
from kivy.uix.gridlayout import GridLayout
import numpy as np
import cv2
import serial
import time
import string
import pynmea2
import smbus
from time import sleep 

class MapViewerApp(App):
    def build(self):
        self.src = None
        self.dest = None
        self.route = None  # Store the route
        self.title = "PiCycle"
        self.geometry = (720, 480)

        # Main layout
        self.layout = BoxLayout(orientation='horizontal')
        # Left layout for map and title
        self.left_layout = BoxLayout(orientation='vertical', padding=0, spacing=0)
        self.layout.add_widget(self.left_layout)

        # Title Label
        self.title_label = Label(text="PiCycle", font_size=70, size_hint=(1, 0.1))
        self.left_layout.add_widget(self.title_label)

        # Vertical layout for map and directions
        self.map_directions_layout = BoxLayout(orientation='vertical', size_hint=(1, 0.9))
        self.left_layout.add_widget(self.map_directions_layout)

        # Matplotlib Figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0)  # Adjust margins
        self.canvas = FigureCanvasKivyAgg(self.fig, size_hint=(1, 0.7))
        self.map_directions_layout.add_widget(self.canvas)

        # Directions ScrollView
        self.directions_scroll = ScrollView(size_hint=(1, 0.3))
        self.directions_layout = GridLayout(cols=1, size_hint_y=None)
        self.directions_layout.bind(minimum_height=self.directions_layout.setter('height'))
        self.directions_scroll.add_widget(self.directions_layout)
        self.map_directions_layout.add_widget(self.directions_scroll)

        # Right layout for datetime
        self.right_layout = BoxLayout(orientation='vertical', size_hint=(0.2, 1))
        self.layout.add_widget(self.right_layout)


        #Reset map Button
        self.reset_map = Button(text='Reset Map', font_size=30)
        self.reset_map.bind(on_press=self.resetMap)
        # Date and Time Label
        self.datetime_label = Label(font_size=30)  # Adjust font size
        self.speed_label = Label(font_size=20)
        self.right_layout.add_widget(self.speed_label)
        self.right_layout.add_widget(self.datetime_label)
        self.speed_offset = 0.9819782714843749 - 1
        self.current_speed = 0
        #some MPU6050 Registers and their Address
        
        Clock.schedule_interval(self.update_speed, 1)
        self.update_datetime()

        #Camera
        self.cam_widget = Image()
        self.cap = cv2.VideoCapture(0)
        Clock.schedule_interval(self.update_camera, 1/30)
        self.right_layout.add_widget(self.cam_widget)

        #GPS
        #self.update_location()
        Clock.schedule_interval(self.update_location, 1)


        # Check if map.graphml exists
        if os.path.exists('laJolla.pkl'):
            with open("laJolla.pkl", "rb") as f:
                self.originalMap = pickle.load(f)
                self.G = copy.deepcopy(self.originalMap)
                print("[DEBUG] pickle file loaded")
        else:
            self.pbf_file = "./map.pbf"
            self.osm = OSM(self.pbf_file)
            self.originalMap = self.load_graph()
            self.G = copy.deepcopy(self.originalMap)

        # self.G = ox.truncate.truncate_graph_bbox(self.G, (-117.8, 32.8, -118, 33))
        # Define the current location (longitude, latitude)
        current_location = (-117.1332, 32.5226)  #  UCSD coordinates

        # Calculate the bounding box for a 5-mile radius around the current location
        #bbox = ox.utils_geo.bbox_from_point(point=current_location, dist=4828.03)  # 5 miles in meters
        ##print('bounding box', (bbox[3], bbox[2], bbox[1], bbox[0])) # (32.681275796937804, -117.20556578930018, 32.36392420306219, -117.06083421069982)
        # Truncate the graph to the bounding box
        #La Jolla box (-117.28626, 32.77712, -117.04971, 32.89929)
        # self.G = ox.truncate.truncate_graph_bbox(self.G, (bbox[3], bbox[2], bbox[1], bbox[0]))
        #self.G = ox.truncate.truncate_graph_bbox(self.G, (-117.28626, 32.77712, -117.04971, 32.89929))
        # Bind click event
        print("Graph Truncated")
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        # Render Map
        self.render_map()

        return self.layout

    def resetMap(self):
        self.G = copy.deepcopy(self.originalMap)
    
    
    
    def update_datetime(self, *args):
        """Updates the datetime label with the current date and time."""
        now = datetime.datetime.now().strftime("%-I:%M %p\n%-m/%d/%Y")
        self.datetime_label.text = now
        Clock.schedule_once(self.update_datetime, 1)

    def update_camera(self, dt):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            buf = frame.tobytes()
            texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='rgb')
            texture.blit_buffer(buf, colorfmt='rgb', bufferfmt='ubyte')

            self.cam_widget.texture = texture
    
    def MPU_Init(self):
        PWR_MGMT_1   = 0x6B
        SMPLRT_DIV   = 0x19
        CONFIG       = 0x1A
        GYRO_CONFIG  = 0x1B
        INT_ENABLE   = 0x38
        
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)
        
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
    
    def update_speed(self, dt):
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47
        #Read Accelerometer raw value
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)
        
        #Read Gyroscope raw value
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        self.current_speed += Ay * 9.81
        self.speed_label.text = str(int(self.current_speed))
        #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
    
    def update_location(self, dt):
        #ser=serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
        # dataout = pynmea2.NMEAStreamReader()
        #newdata=ser.readline()
        # print(newdata)
        self.current_location = (-117.1332, 32.5226)
        """
        if '$GPRMC' in str(newdata):
            # print(newdata.decode('utf-8'))
            newmsg=pynmea2.parse(newdata.decode('utf-8'))
            lat=newmsg.latitude
            lng=newmsg.longitude
            # gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
            # return (lat, lng)
            self.current_location = (lat, lng)
        else:
            self.current_location = (-117.1332, 32.5226)
        """
        
    def load_graph(self):
        """Loads the road network from the .pbf file and converts it to a directed graph."""
        print("Load Graph")
        nodes, edges = self.osm.get_network(network_type="cycling", nodes=True)  # Get road network
        # Filter nodes based on longitude
        nodes["x"] = nodes["lon"]
        nodes["y"] = nodes["lat"]
        edges["key"] = edges.groupby(["u", "v"]).cumcount()
        nodes.set_index('id', inplace=True)
        edges.set_index(['u', 'v', 'key'], inplace=True)
        # Include street names in the edges
        edges['name'] = edges['name'].fillna('Unnamed Road')
        G = ox.graph_from_gdfs(nodes, edges)  # Create directed graph
        with open("map.pkl", "wb") as f:
            pickle.dump(G, f)
        print("file saved to map.pkl")
        return G

    def render_map(self):
        """Displays the entire map inside the Kivy UI."""
        if self.G is None:
            print("Graph not loaded properly!")
            return

        self.ax.clear()
        print("[DEBUG] render_map")
        # Highlight the route edges in blue
        nearest_node = []
        if hasattr(self, 'current_location'):
            nearest_node = [ox.distance.nearest_nodes(self.G, self.current_location[0], self.current_location[1])]
        print('DEBUG: nearest_node', nearest_node)
        node_sizes = [1.0 if node in nearest_node else 0 for node in self.G.nodes()]
        if self.route:
            edge_colors = ['blue' if (u, v) in zip(self.route[:-1], self.route[1:]) or (v, u) in zip(self.route[:-1], self.route[1:]) else '#999999' for u, v in self.G.edges()]
            ox.plot_graph(self.G, ax=self.ax, show=False, close=False, edge_color=edge_colors, node_color='green', node_size=node_sizes, edge_linewidth=.5)
        else:
            ox.plot_graph(self.G, ax=self.ax, show=False, close=False, node_color='green', node_size=node_sizes, edge_linewidth=0.5)
        

        # Refresh Kivy Canvas
        self.canvas.draw()

    def update_directions(self):
        """Updates the directions layout with turn-by-turn directions."""
        if self.route:
            directions = self.get_turn_by_turn_directions()
            self.directions_layout.clear_widgets()
            for step in directions:
                direction_label = Label(text=step, font_size=20, size_hint_y=None, height=40)
                self.directions_layout.add_widget(direction_label)

    def get_turn_by_turn_directions(self):
        """Generates turn-by-turn directions for the current route."""
        directions = []
        total_straight_distance_miles = 0
        total_straight_distance_yards = 0
        for i in range(len(self.route) - 2):
            u, v, w = self.route[i], self.route[i + 1], self.route[i + 2]
            edge_data_uv = self.G.get_edge_data(u, v)
            print('uv', edge_data_uv)
            edge_data_vw = self.G.get_edge_data(v, w)
            print('vw', edge_data_vw)
            if edge_data_uv and edge_data_vw:
                edge_uv = edge_data_uv[0]
                edge_vw = edge_data_vw[0]
                length_uv_meters = edge_uv.get('length', 0)
                length_uv_miles = length_uv_meters * 0.000621371  # Convert meters to miles
                length_uv_yards = length_uv_meters * 1.09361  # Convert meters to yards
                street_name = edge_uv.get('name', 'Unnamed Road')
                x1, y1 = self.G.nodes[u]['x'], self.G.nodes[u]['y']
                x2, y2 = self.G.nodes[v]['x'], self.G.nodes[v]['y']
                x3, y3 = self.G.nodes[w]['x'], self.G.nodes[w]['y']
                angle = np.arctan2(y3 - y2, x3 - x2) - np.arctan2(y2 - y1, x2 - x1)
                angle = np.degrees(angle)
                if angle < -180:
                    angle += 360
                if angle > 180:
                    angle -= 360
                if angle > 45 and angle <= 135:
                    turn = "TURN right"
                elif angle < -45 and angle >= -135:
                    turn = "TURN left"
                elif angle > 135 or angle < -135:
                    turn = "U-TURN"
                elif angle > 15 and angle <= 45:
                    turn = "SLIGHT RIGHT"
                elif angle < -15 and angle >= -45:
                    turn = "SLIGHT LEFT"
                else:
                    turn = "CONTINUE straight"
                
                if turn == "CONTINUE straight":
                    total_straight_distance_miles += length_uv_miles
                    total_straight_distance_yards += length_uv_yards
                else:
                    if total_straight_distance_miles > 0 or total_straight_distance_yards > 0:
                        if total_straight_distance_miles < 0.25:
                            directions.append(f"CONTINUE straight for {total_straight_distance_yards:.1f} yards")
                        else:
                            directions.append(f"CONTINUE straight for {total_straight_distance_miles:.2f} miles")
                        total_straight_distance_miles = 0
                        total_straight_distance_yards = 0
                    if length_uv_miles < 0.25:
                        directions.append(f"In {length_uv_yards:.1f} yards, {turn} onto {street_name}")
                    else:
                        directions.append(f"In {length_uv_miles:.2f} miles, {turn} onto {street_name}")
        
        if total_straight_distance_miles > 0 or total_straight_distance_yards > 0:
            if total_straight_distance_miles < 0.25:
                directions.append(f"CONTINUE straight for {total_straight_distance_yards:.1f} yards")
            else:
                directions.append(f"CONTINUE straight for {total_straight_distance_miles:.2f} miles")
        
        return directions

    def on_click(self, event):
        """Handles click events on the graph."""
        if event.inaxes is not None:
            x, y = event.xdata, event.ydata
            print(f"Clicked at coordinates: ({x}, {y})")
            nearest_node = ox.distance.nearest_nodes(self.G, x, y)
            if not self.src or self.dest:
                self.src = nearest_node
                self.dest = None
                self.route = None  # Reset the route
            else:
                self.dest = nearest_node
                self.route = ox.shortest_path(self.G, self.src, self.dest)
                print('Route: ', self.route)
                if self.route:
                    route_nodes = [self.G.nodes[node] for node in self.route]
                    lons = [node['x'] for node in route_nodes]
                    lats = [node['y'] for node in route_nodes]
                    self.G = ox.truncate.truncate_graph_bbox(self.G, (min(lons), min(lats), max(lons), max(lats)), truncate_by_edge=True)
                    self.right_layout.add_widget(self.reset_map)
                    self.render_map()  # Re-render the map to show the route
                    self.update_directions()  # Update the directions

if __name__ == "__main__":
    Config.set('graphics', 'width', '800')
    Config.set('graphics', 'height', '480')
    bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
    Device_Address = 0x68   # MPU6050 device address
    MapViewerApp().MPU_Init()
    MapViewerApp().run()
