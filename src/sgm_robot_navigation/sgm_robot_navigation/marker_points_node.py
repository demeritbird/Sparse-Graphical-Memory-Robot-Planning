#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from sgm_robot_interfaces.msg import MarkerNode, MapInformation
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np
import cv2

class MarkerPointsNode(Node):
    def __init__(self):
        super().__init__('marker_points_node')
        
        self.get_logger().info("Marker Points Node has started.")
        
        self.map_height_ = 0
        self.map_width_ = 0
        self.map_resolution_ = 0
        self.map_data_ = [] # int8[]
        
        self.original_nodes_spawned = 50
        self.similarity_threshold = 0.15
        self.distance_threshold = 0.4
                
        self.nodes_information_ = {}
        self.marker_visualisation_publisher_ = self.create_publisher(Marker, 'visualization_marker', 5)
        self.marker_information_publisher_ = self.create_publisher(MapInformation, 'sgm_map', 5)
        self.marker_timer_ = self.create_timer(1.0, self.init_marker_nodes) 
        
        self.map_qos_profile_ = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_subscription_ = self.create_subscription(
            OccupancyGrid,
            'map',
            self.get_map_info_callback,
            qos_profile=self.map_qos_profile_
        )
        
    def get_map_info_callback(self, msg):    
        # map dimensions
        self.map_height_ = msg.info.height
        self.map_width_ = msg.info.width
        self.map_resolution_ = msg.info.resolution

        self.get_logger().info(f"Map Width: {self.map_height_}, Map Height: {self.map_width_}, Map Resolution: {self.map_resolution_}")
        
        # map wall information (walls -> 100, empty -> 0)
        self.map_data_ = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        ## NOTE: for testing purposes, save image
        map_image = (self.map_data_ * 255 / 100).astype(np.uint8)  # Scale occupancy values to 0-255
        cv2.imwrite('output/occupancy_grid_map.png', map_image)
        np.save('simplified_sgm/map_data.npy', self.map_data_)          

    def publish_markers_visualisation(self):
        marker_msgs = []

        for idx, info in self.nodes_information_.items():
            marker_msg = Marker()
            marker_msg.header.frame_id = 'map'
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.ns = 'markers'
            marker_msg.id = idx
            marker_msg.type = Marker.CYLINDER
            marker_msg.action = Marker.ADD
            
            # pose
            marker_msg.pose.position.x = info['position'][0] * self.map_width_ * self.map_resolution_
            marker_msg.pose.position.y = info['position'][1] * self.map_height_ * self.map_resolution_
            marker_msg.pose.position.z = 0.5 
            
            # orientation
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            
            # scale
            marker_msg.scale.x = 0.5 
            marker_msg.scale.y = 0.5
            marker_msg.scale.z = 0.5 
            
            #color
            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            
            #lifetime
            marker_msg.lifetime.sec = 10

            marker_msgs.append(marker_msg)

        for marker_msg in marker_msgs:
            self.marker_visualisation_publisher_.publish(marker_msg)

    def publish_markers_information(self):
        if len(self.nodes_information_) == 0:
            self.get_logger().warn("No node information to publish!")
            return

        # there is node information, i want to publish it out there
        map_information = MapInformation()
        map_information.map_height = self.map_height_
        map_information.map_width = self.map_width_
        map_information.map_resolution = self.map_resolution_        
        for idx, info in self.nodes_information_.items():
            node_info = MarkerNode(index=idx, 
                                   position_x = info['position'][0] * self.map_width_ * self.map_resolution_,
                                   position_y = info['position'][1] * self.map_height_ * self.map_resolution_,
                                   connected_nodes = (info.get('connected_nodes_green', []) + info.get('connected_nodes_orange', [])))
        
            map_information.marker_nodes.append(node_info)

        self.get_logger().info("Publishing node information to be captured...")
        self.marker_information_publisher_.publish(map_information)

    def init_marker_nodes(self):
        def calculate_euclidean_distance(node1, node2):
            return np.linalg.norm(node1 - node2)

        def twc_similarity(node1, node2, nodes, threshold):
            '''
            Compute two-way consistency (TWC) similarity between two nodes
            TWC ensures that both nodes can be reached with similar effort from their neighbors
            and all their neighbors can be reached from both nodes with similar effort
            '''

            c_out = max(calculate_euclidean_distance(node1, w) - calculate_euclidean_distance(node2, w) for w in nodes)
            c_in = max(calculate_euclidean_distance(s0, node1) - calculate_euclidean_distance(s0, node2) for s0 in nodes)

            return c_out <= threshold and c_in <= threshold
        
        def sparsify_graph(nodes, threshold):
            '''
            For now, this merges nodes that are close to each other 
            based off TWC simplicity
            '''
            
            merged_nodes = []

            for node in nodes:
                #check if the node has already been merged
                already_merged = False
                for merged_node in merged_nodes:
                    if calculate_euclidean_distance(node, merged_node) <= threshold:
                        already_merged = True
                        break
                
                if not already_merged:
                    neighbors = [n for n in nodes if not np.array_equal(n, node) and calculate_euclidean_distance(node, n) <= threshold]

                    similar_nodes = [n for n in neighbors if twc_similarity(node, n, nodes, threshold)]

                    # perform merging if node is similar enough
                    if similar_nodes:
                        merged_node = np.mean(np.array(similar_nodes + [node]), axis=0)  # Centroid as the representative node
                        merged_nodes.append(merged_node)
                    else:
                        merged_nodes.append(node)

            return merged_nodes

        def is_line_intersecting_obstacle(node1, node2, map_data):
            ''' 
            Check if line between two Sparse Nodes is colliding w a wall, using map pixel value.
            If so, then we want to reject this line, since robot will crash into the wall in simulation.
            '''
            
            x1, y1 = node1
            x2, y2 = node2
            
            x1 = x1 * self.map_width_
            x2 = x2 * self.map_width_
            y1 = y1 * self.map_height_
            y2 = y2 * self.map_height_
            
            num_samples = int(max(abs(x2 - x1), abs(y2 - y1))) + 1
            
            x_coords = np.linspace(x1 , x2 , num_samples)
            y_coords = np.linspace(y1, y2, num_samples)
            
            for x, y in zip(x_coords, y_coords):
                x_pixel = int(round(x))
                y_pixel = int(round(y))

                if 0 <= y_pixel < map_data.shape[0] and 0 <= x_pixel < map_data.shape[1]:
                    if map_data[y_pixel, x_pixel] != 0:
                        return True
            
            # If no intersections found
            return False

        def draw_lines(sparse_nodes, map_data, threshold, image_width, image_height):
            '''
            Draw lines connecting sparse nodes within threshold distance,
            as well as do its best to connect isolated nodes.
            Cuts lines that are clipping through walls as cleanup.
            Also store information about each node as a dictionary.
            '''
            
            # Store connected nodes information for each sparse node
            sparse_info = {} 
            connected_pairs = set() 

            # Loop though nodes within threshold distance
            for i in range(len(sparse_nodes)):
                connected_nodes = []
                for j in range(len(sparse_nodes)):
                    if i != j:
                        dist = calculate_euclidean_distance(sparse_nodes[i], sparse_nodes[j])
                        if dist <= threshold * min(image_width, image_height) / max(image_width, image_height):
                            # check if the pair of nodes is not already connected
                            if (i, j) not in connected_pairs and (j, i) not in connected_pairs:
                                # check if the line intersects with any obstacle and add if no interference 
                                if not is_line_intersecting_obstacle(sparse_nodes[i], sparse_nodes[j], map_data):
                            
                                    connected_nodes.append(j)
                                    # add the connected pair to the set
                                    connected_pairs.add((i, j))

                # add connected node info to our collection
                sparse_info[i] = {'position': sparse_nodes[i], 'connected_nodes_green': connected_nodes}

            # draw lines from each unconnected sparse node to its nearest sparse node
            for i in range(len(sparse_nodes)):
                if not sparse_info[i]['connected_nodes_green']:
                    min_dist = float('inf')
                    nearest_idx = None
                    for j in range(len(sparse_nodes)):
                        if i != j:
                            dist = calculate_euclidean_distance(sparse_nodes[i], sparse_nodes[j])
                            if dist < min_dist:
                                min_dist = dist
                                nearest_idx = j
                    if nearest_idx is not None:
                        # check if the pair of nodes is not already connected
                        if (i, nearest_idx) not in connected_pairs and (nearest_idx, i) not in connected_pairs:
                            # check if the line intersects with any obstacle and add if no interference
                            if not is_line_intersecting_obstacle(sparse_nodes[i], sparse_nodes[nearest_idx], map_data):
                                sparse_info[i]['connected_node_orange'] = nearest_idx
        
                                connected_pairs.add((i, nearest_idx))

            # So we can have two way connection in our sparse_info collection     
            for idx, info in sparse_info.items():
                for connected_node_idx in info.get('connected_nodes_green', []):
                    if idx not in sparse_info[connected_node_idx].get('connected_nodes_green', []):
                        sparse_info[connected_node_idx].get('connected_nodes_green', []).append(idx)
                    
                for connected_node_idx in info.get('connected_nodes_orange', []): 
                    if idx not in sparse_info[connected_node_idx].get('connected_nodes_orange', []):
                        sparse_info[connected_node_idx].get('connected_nodes_orange', []).append(idx)


            return sparse_info

        # first, we want to check if the map data exists
        if len(self.map_data_) == 0 or self.map_height_ == 0 or self.map_width_ == 0:
            self.get_logger().warn("No map_data received yet!")
            self.marker_timer_.reset()
            return

        # we have gotten map information, now we want to spawn nodes
        nodes = np.random.rand(self.original_nodes_spawned, 2) 

        sparse_nodes = sparsify_graph(nodes, self.similarity_threshold)
        sparse_nodes = np.array(sparse_nodes)
        self.nodes_information_ = draw_lines(sparse_nodes, self.map_data_, self.distance_threshold, self.map_width_, self.map_height_)

        # now we have the sparse information, we want to publish them onto the map topic.
        self.publish_markers_visualisation()
        
        self.publish_markers_information()
        # DEBUG: just to see information
        # for idx, info in self.nodes_information_.items():
        #     self.get_logger().info(f"Sparse Node {idx}: Position: ({round(info['position'][0] * self.map_width_, 2)}, {round(info['position'][1] * self.map_height_, 2)}), Connected Nodes (Green): {info.get('connected_nodes_green', [])}, Connected Node (Orange): {info.get('connected_node_orange', [])}")
        self.marker_timer_.cancel() 
        
         

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPointsNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()