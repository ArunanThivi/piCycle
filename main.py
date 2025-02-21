import tkinter as tk
from tkinter import Label
import osmnx as ox
import matplotlib.pyplot as plt
from pyrosm import OSM
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import datetime, os
from play_video import VideoPlayer  # Import VideoPlayer class
import pickle

class MapViewerApp:
    def __init__(self, root):
        self.src = None
        self.dest = None
        self.route = None  # Store the route
        self.root = root
        self.root.title("Map Viewer")
        self.root.geometry("900x600")

        # Title Label
        Label(root, text="OpenStreetMap Viewer", font=("Arial", 14)).pack(pady=10)

        # Date and Time Label
        self.datetime_label = Label(root, font=("Arial", 12))
        self.datetime_label.pack(pady=5, side="right")
        self.update_datetime()
        # Check if map.graphml exists
        if os.path.exists('map.pkl'):
            with open("map.pkl", "rb") as f:
                self.G = pickle.load(f)
        else:
            self.pbf_file = "./map.pbf"
            self.osm = OSM(self.pbf_file)
            self.G = self.load_graph()
        
        self.G = ox.truncate.truncate_graph_bbox(self.G, (-117, 32, -118, 33))

        # Matplotlib Figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack()

        # Bind click event
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        # Render Map
        self.render_map()

        # Video Player Frame
        # self.video_frame = tk.Frame(root)
        # self.video_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        # self.video_player = VideoPlayer(self.video_frame, "dashcam.mp4")

    def update_datetime(self):
        """Updates the datetime label with the current date and time."""
        now = datetime.datetime.now().strftime("%-I:%M %p\n%-m/%d/%Y")
        self.datetime_label.config(text=now)
        self.root.after(1000, self.update_datetime)

    def load_graph(self):
        """Loads the road network from the .pbf file and converts it to a graph."""
        nodes, edges = self.osm.get_network(network_type="cycling", nodes=True)  # Get road network
        # Filter nodes based on longitude
        nodes["x"] = nodes["lon"]
        nodes["y"] = nodes["lat"]
        edges["key"] = edges.groupby(["u", "v"]).cumcount()
        nodes.set_index('id', inplace=True)
        edges.set_index(['u', 'v', 'key'], inplace=True)
        G =  ox.graph_from_gdfs(nodes, edges)  # Create graph
        with open("map.pkl", "wb") as f:
            pickle.dump(G, f)
        return ox.truncate.truncate_graph_bbox(G, (-117.233812, 32.878989, -117.230116, 32.880896))

    def render_map(self):
        """Displays the entire map inside the Tkinter UI."""
        if self.G is None:
            print("Graph not loaded properly!")
            return

        self.ax.clear()
        
        # Highlight the route edges in blue
        if self.route:
            edge_colors = ['blue' if (u, v) in zip(self.route[:-1], self.route[1:]) or (v, u) in zip(self.route[:-1], self.route[1:]) else '#999999' for u, v in self.G.edges()]
            ox.plot_graph(self.G, ax=self.ax, show=False, close=False, edge_color=edge_colors, node_size=0, edge_linewidth=.5)
        else:
            ox.plot_graph(self.G, ax=self.ax, show=False, close=False, node_size=0, edge_linewidth=0.5)
        # Refresh Tkinter Canvas
        self.canvas.draw()

    def on_click(self, event):
        """Handles click events on the graph."""
        if event.inaxes is not None:
            x, y = event.xdata, event.ydata
            print(f"Clicked at coordinates: ({x}, {y})")
            nearest_node = ox.distance.nearest_nodes(self.G, x, y)
            if (not self.src or self.dest):
                self.src = nearest_node
                self.dest = None
                self.route = None  # Reset the route
            else:
                self.dest = nearest_node
                self.route = ox.shortest_path(self.G, self.src, self.dest)
                print('Route: ', self.route)
                if self.route:
                    self.render_map()  # Re-render the map to show the route
if __name__ == "__main__":
    root = tk.Tk()
    app = MapViewerApp(root)
    root.mainloop()
