    # This is an adapted version of the RRT implementation done by Atsushi Sakai (@Atsushi_twi)
    class RRTC:
        """
        Class for RRT planning
        """
        class Node:
            """
            RRT Node
            """
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.path_x = []
                self.path_y = []
                self.parent = None

        def __init__(self, start=np.zeros(2),
                    goal=np.array([120,90]),
                    obstacle_list=None,
                    width = 160,
                    height=100,
                    expand_dis=3.0,
                    path_resolution=0.5,
                    max_points=200):
            """
            Setting Parameter
            start:Start Position [x,y]
            goal:Goal Position [x,y]
            obstacle_list: list of obstacle objects
            width, height: search area
            expand_dis: min distance between random node and closest node in rrt to it
            path_resolion: step size to considered when looking for node to expand
            """
            self.start = self.Node(start[0], start[1])
            self.end = self.Node(goal[0], goal[1])
            self.width = width
            self.height = height
            self.expand_dis = expand_dis
            self.path_resolution = path_resolution
            self.max_nodes = max_points
            self.obstacle_list = obstacle_list
            self.start_node_list = [] # Tree from start
            self.end_node_list = [] # Tree from end

        def planning(self):
            """
            rrt path planning
            """
            self.start_node_list = [self.start]
            self.end_node_list = [self.end]
            while len(self.start_node_list) + len(self.end_node_list) <= self.max_nodes:

                # 1. Generate a random node
                #  Sample and add a node in the start tree
                rnd_node = self.get_random_node()
                expansion_ind = self.get_nearest_node_index(self.start_node_list, rnd_node)
                expansion_node = self.start_node_list[expansion_ind]

                #TODO: Complete RRT ----------------------------------------------------------------

                nearby_node = self.steer(expansion_node, rnd_node, self.expand_dis)
                if self.is_collision_free(nearby_node):
                    self.start_node_list.append(nearby_node)

                #ENDTODO ----------------------------------------------------------------------------------------------

                # 2. Check whether trees can be connected
                node_index = self.get_nearest_node_index(self.end_node_list, nearby_node)
                node = self.end_node_list[node_index]

                distance, _ = self.calc_distance_and_angle(node, nearby_node)
                if distance <= self.expand_dis:


                # 3. Add the node that connects the trees and generate the path
                    # Note: It is important that you return path found as:
                    # return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
                    if self.is_collision_free(nearby_node):
                        self.end_node_list.append(nearby_node)
                        self.start_node_list.append(node)
                        return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
                # 4. Sample and add a node in the end tree

                else:
                    rnd_node = self.get_random_node()
                    expansion_ind = self.get_nearest_node_index(self.end_node_list, rnd_node)
                    expansion_node = self.end_node_list[expansion_ind]
                    nearby_node = self.steer(expansion_node, rnd_node, self.expand_dis)
                    if self.is_collision_free(nearby_node):
                        self.end_node_list.append(nearby_node)

                # 5. Swap start and end trees
                self.start_node_list, self.end_node_list = self.end_node_list, self.start_node_list

            return None  # cannot find path

        # ------------------------------DO NOT change helper methods below ----------------------------
        def steer(self, from_node, to_node, extend_length=float("inf")):
            """
            Given two nodes from_node, to_node, this method returns a node new_node such that new_node
            is “closer” to to_node than from_node is.
            """

            new_node = self.Node(from_node.x, from_node.y)
            d, theta = self.calc_distance_and_angle(new_node, to_node)
            cos_theta, sin_theta = np.cos(theta), np.sin(theta)

            new_node.path_x = [new_node.x]
            new_node.path_y = [new_node.y]

            if extend_length > d:
                extend_length = d

            # How many intermediate positions are considered between from_node and to_node
            n_expand = math.floor(extend_length / self.path_resolution)

            # Compute all intermediate positions
            for _ in range(n_expand):
                new_node.x += self.path_resolution * cos_theta
                new_node.y += self.path_resolution * sin_theta
                new_node.path_x.append(new_node.x)
                new_node.path_y.append(new_node.y)

            d, _ = self.calc_distance_and_angle(new_node, to_node)
            if d <= self.path_resolution:
                new_node.path_x.append(to_node.x)
                new_node.path_y.append(to_node.y)

            new_node.parent = from_node

            return new_node

        def is_collision_free(self, new_node):
            """
            Determine if nearby_node (new_node) is in the collision-free space.
            """
            if new_node is None:
                return True

            points = np.vstack((new_node.path_x, new_node.path_y)).T
            for obs in self.obstacle_list:
                in_collision = obs.is_in_collision_with_points(points)
                if in_collision:
                    return False

            return True  # safe

        def generate_final_course(self, start_mid_point, end_mid_point):
            """
            Reconstruct path from start to end node
            """
            # First half
            node = self.start_node_list[start_mid_point]
            path = []
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([node.x, node.y])

            # Other half
            node = self.end_node_list[end_mid_point]
            path = path[::-1]
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([node.x, node.y])

            return path

        def calc_dist_to_goal(self, x, y):
            dx = x - self.end.x
            dy = y - self.end.y
            return math.hypot(dx, dy)

        def get_random_node(self):
            x = self.width * np.random.random_sample()
            y = self.height * np.random.random_sample()
            rnd = self.Node(x, y)
            return rnd

        @staticmethod
        def get_nearest_node_index(node_list, rnd_node):
            # Compute Euclidean disteance between rnd_node and all nodes in tree
            # Return index of closest element
            dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                    ** 2 for node in node_list]
            minind = dlist.index(min(dlist))
            return minind

        @staticmethod
        def calc_distance_and_angle(from_node, to_node):
            dx = to_node.x - from_node.x
            dy = to_node.y - from_node.y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            return d, theta