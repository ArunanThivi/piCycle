from kivy.app import App
from kivy.config import Config
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.clock import Clock
from kivy_garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
import osmnx as ox
import matplotlib.pyplot as plt
from pyrosm import OSM
import datetime
import os
import pickle
from kivy.uix.scrollview import ScrollView
from kivy.uix.gridlayout import GridLayout
import numpy as np

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

        # Date and Time Label
        self.datetime_label = Label(font_size=30)  # Adjust font size
        self.right_layout.add_widget(self.datetime_label)
        self.update_datetime()

        # Check if map.graphml exists
        if os.path.exists('map.pkl'):
            with open("map.pkl", "rb") as f:
                self.G = pickle.load(f)
                print("[DEBUG] pickle file loaded")
        else:
            self.pbf_file = "./map.pbf"
            self.osm = OSM(self.pbf_file)
            self.G = self.load_graph()

        # self.G = ox.truncate.truncate_graph_bbox(self.G, (-117.8, 32.8, -118, 33))
        # Define the current location (longitude, latitude)
        current_location = (-117.1332, 32.5226)  #  UCSD coordinates

        # Calculate the bounding box for a 5-mile radius around the current location
        bbox = ox.utils_geo.bbox_from_point(point=current_location, dist=4828.03)  # 5 miles in meters
        print('bounding box', (bbox[3], bbox[2], bbox[1], bbox[0])) # (32.681275796937804, -117.20556578930018, 32.36392420306219, -117.06083421069982)
        # Truncate the graph to the bounding box
        #La Jolla box (-117.28626, 32.77712, -117.04971, 32.89929)
        # self.G = ox.truncate.truncate_graph_bbox(self.G, (bbox[3], bbox[2], bbox[1], bbox[0]))
        self.G = ox.truncate.truncate_graph_bbox(self.G, (-117.28626, 32.77712, -117.04971, 32.89929))
        # Bind click event
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        # Render Map
        self.render_map()

        return self.layout

    def update_datetime(self, *args):
        """Updates the datetime label with the current date and time."""
        now = datetime.datetime.now().strftime("%-I:%M %p\n%-m/%d/%Y")
        self.datetime_label.text = now
        Clock.schedule_once(self.update_datetime, 1)

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
        if self.route:
            edge_colors = ['blue' if (u, v) in zip(self.route[:-1], self.route[1:]) or (v, u) in zip(self.route[:-1], self.route[1:]) else '#999999' for u, v in self.G.edges()]
            ox.plot_graph(self.G, ax=self.ax, show=False, close=False, edge_color=edge_colors, node_size=0, edge_linewidth=.5)
        else:
            ox.plot_graph(self.G, ax=self.ax, show=False, close=False, node_size=0, edge_linewidth=0.5)
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
                    self.render_map()  # Re-render the map to show the route
                    self.update_directions()  # Update the directions

if __name__ == "__main__":
    Config.set('graphics', 'width', '800')
    Config.set('graphics', 'height', '480')
    MapViewerApp().run()
