class Manipulator:
    def __init__(self):
        self.links = [
            {"r": 0, "alpha": -PI/2, "d": 0, "theta": 0, "type": 0},
            {"r": 10, "alpha": 0, "d": 0, "theta": -PI/4, "type": 0},
            {"r": 10, "alpha": 0, "d": 0, "theta": PI/2, "type": 0},
        ]
        self.goal = [15, 0, -2]

    def draw_goal(self):
        # Draw a translucent sphere at the goal position
        pushMatrix()
        strokeWeight(0)
        fill(255, 0, 0, 100)
        translate(self.goal[0], self.goal[1], self.goal[2])
        sphere(1.25 * shape_scale)
        popMatrix()

    def draw(self):
        pushMatrix()
        # Draw base node at origin
        Manipulator.draw_node(0)
        
        for i in range(len(self.links)):
            link = self.links[i]
            # Draw the connector
            Manipulator.draw_connector(link)
            
            # Draw the node
            Manipulator.apply_transformations(link)
            Manipulator.draw_node(self.links[i+1]["type"] if i+1 < len(self.links) else 2)
        popMatrix()

    def move_to_goal(self):
        heading = Manipulator.goal_heading(self.goal)
        self.links[0]["theta"] = heading
        pos2D = Manipulator.get_flat_pos(heading, self.goal)
        theta1, theta2 = self.inverse_kinematics(pos2D)
        if theta1 is not None and theta2 is not None:
            self.links[1]["theta"] = theta1
            self.links[2]["theta"] = theta2
        pass

    # Kinematics
    @staticmethod
    def apply_transformations(link):
        theta = link["theta"]
        d = link["d"]
        r = link["r"]
        alpha = link["alpha"]
        
        rotateZ(theta)
        translate(0, 0, d)
        translate(r, 0, 0)
        rotateX(alpha)
    @staticmethod
    def goal_heading(goal):
        projected_x = goal[0]
        projected_y = goal[1]
        angle = atan2(projected_y, projected_x)
        # println("Rotation along vertical axis (radians):", angle)
        # println("Rotation along vertical axis (degrees):" + str(degrees(angle)))
        return angle
    @staticmethod
    def get_flat_pos(heading, goal):
        # Unit vector along the heading (in the x-y plane)
        u_x = cos(heading)
        u_y = sin(heading)
        # Projection along the heading direction
        pos_along_heading = goal[0]*u_x + goal[1]*u_y
        # The vertical coordinate is already Z
        pos_z = goal[2]
        pos2D = [pos_along_heading, pos_z]
        # println("2D position:" + str(pos2D))
        return pos2D
    def inverse_kinematics(self, pos2D):
        # Inputs: link lengths from kinematics (assumed positive)
        l1 = self.links[1]["r"]
        l2 = self.links[2]["r"]
        
        # pos2D: [distance along heading, height]
        x = pos2D[0]
        y = -pos2D[1]
        
        # Distance squared from the origin in the 2D plane
        dist_sq = x*x + y*y
        
        # Check reachability
        if dist_sq > (l1 + l2)**2:
            println("Target unreachable")
            return None, None
        
        # Compute angle at elbow
        cos_theta2 = (dist_sq - l1*l1 - l2*l2) / (2 * l1 * l2)
        # Clamp to avoid floating point errors outside [-1,1]
        cos_theta2 = max(-1, min(1, cos_theta2))
        theta2 = acos(cos_theta2)
        
        # For elbow-up solution:
        k1 = l1 + l2 * cos(theta2)
        k2 = l2 * sin(theta2)
        theta1 = atan2(y, x) - atan2(k2, k1)
        
        return theta1, theta2
    
    # Drawing
    @staticmethod
    def draw_node(type=0, highlight=False, draw_axes=False):
        # Draw axes
        if draw_axes:
            strokeWeight(0.2 if highlight else 0.1)
            draw_reference_frame()

        # Drawing settings
        stroke(0, 0, 0)
        fill(128, 128, 128)
        if highlight:
            fill(255, 255, 0)
        elif type == 2:
            fill(154, 205, 50)
        
        # Draw the node based on the type
        if type == 0:
            draw_cylinder(0.5 * shape_scale, 2 * shape_scale)
        elif type == 1:
            box(1 * shape_scale, 1 * shape_scale, 1.5 * shape_scale)
        elif type == 2:
            strokeWeight(0)
            sphere(1 * shape_scale)
    @staticmethod
    def draw_connector(link):
        link_length = sqrt(link["r"]**2 + link["d"]**2)
            
        if link_length > 0:
            pushMatrix()
            
            # Math stuff
            rotateZ(link["theta"])
            translate(link["r"] / 2, 0, link["d"] / 2)
            angle_between = acos(link["d"] / link_length)
            axis_y = 1 if link["r"] >= 0 else -1
            rotate(angle_between, 0, axis_y, 0)

            # Draw the connector
            box(1 * shape_scale, 1 * shape_scale, link_length)

            popMatrix()

# Drawing setup
angleX = 0
angleY = 0
zoom = 10.0
axis_length = 10
shape_scale = 2
arm1 = Manipulator()
arm2 = Manipulator()
arm3 = Manipulator()

box_pos = [15, 0, 0]
box_dims = [8, 16, 4]
box_rot = [0, 0, 0]

# Goal positions around the box
top_goal1 = [0, 5, box_dims[2]/2 + shape_scale]  # for arm1
top_goal2 = [0, -5, box_dims[2]/2 + shape_scale]  # for arm2
bottom_goal = [0, 0, -box_dims[2]/2 - shape_scale]  # for arm3

arms = [
    {"arm": arm1, "pos": [0, 10, 0], "xrot": 0, "goal": top_goal1},
    {"arm": arm2, "pos": [0, -10, 0], "xrot": 0, "goal": top_goal2},
    {"arm": arm3, "pos": [0, 0, -10], "xrot": PI, "goal": bottom_goal},
]
selected_goal = 0


# Main functions
def set_up_drawing():
    global angleX, angleY
    background(255)
    smooth()
    lights()
    directionalLight(51, 102, 126, -1, 0, 0)

    # Set up origin
    translate(width/2, height/2, 0)

    # Apply zoom
    scale(zoom)
    
    # Rotate based on mouse drags
    rotateX(angleX)
    rotateY(angleY)
    rotateX(PI/2)
    rotateZ(PI/2)

    strokeWeight(0.1)
    draw_reference_frame()

def setup():
    size(600, 600, P3D)

def draw():
    set_up_drawing()

    control()

    # Draw the box
    pushMatrix()
    translate(box_pos[0], box_pos[1], box_pos[2])
    rotateZ(box_rot[2])
    rotateY(box_rot[1])
    rotateX(box_rot[0])
    draw_reference_frame()
    fill(139, 69, 19)  # Set the color to brown
    box(box_dims[0], box_dims[1], box_dims[2])
    popMatrix()

    # Set up arms
    for arm_data in arms:
        arm = arm_data["arm"]
        pos = arm_data["pos"]
        xrot = arm_data["xrot"]
        global_goal = arm_data["goal"]
        global_goal = local_to_global_box(global_goal, box_pos, box_rot)
        local_goal = global_to_local(global_goal, pos, xrot)
        pushMatrix()
        translate(pos[0], pos[1], pos[2])
        rotateX(xrot)
        arm.goal = local_goal
        arm.move_to_goal()
        arm.draw()
        arm.draw_goal()
        popMatrix()

# Control
def control():
    global selected_goal
    speed = 0.25
    avel = 0.025
    # If a digit key is pressed, update the selected link.
    if keyPressed:
        # Select the goal based on the digit key
        if str(key).isdigit():
            idx = int(key) - 1  # Adjust index to match key 1 to joint 0, etc.
            if 0 <= idx < len(arm1.links):
                selected_goal = idx
        
        # Control the goal position
        # arms[selected_goal]["goal"][0] += speed * (-1 if key == CODED and keyCode == UP   else 1 if key == CODED and keyCode == DOWN  else 0)
        # arms[selected_goal]["goal"][1] += speed * (-1 if key == CODED and keyCode == RIGHT else 1 if key == CODED and keyCode == LEFT else 0)
        # arms[selected_goal]["goal"][2] += speed * (-1 if key == 's' else 1 if key == 'w' else 0)

        # Control the box
        box_pos[0] += speed * (-1 if key == 'w' else 1 if key == 's' else 0)
        box_pos[1] += speed * (-1 if key == 'd' else 1 if key == 'a' else 0)
        box_pos[2] += speed * (-1 if key == 'x' else 1 if key == 'z' else 0)
        box_rot[0] += avel * (-1 if key == CODED and keyCode == LEFT else 1 if key == CODED and keyCode == RIGHT else 0)
        box_rot[1] += avel * (-1 if key == CODED and keyCode == UP else 1 if key == CODED and keyCode == DOWN else 0)
        box_rot[2] += avel * (-1 if key == ',' else 1 if key == '.' else 0)


        # Move to goal
        # if key == 'g':
        #     for arm in arms:
        #         arm["arm"].move_to_goal()

def mouseDragged():
    global angleX, angleY
    # Update rotation angles based on the change in mouse position.
    # Adjust the sensitivity by multiplying with a small factor (e.g., 0.01).
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01

# Helper functions
def draw_cylinder(radius, height, sides=24):
    strokeWeight(0)
    angle = TWO_PI / sides
    half_height = height / 2

    # Draw the top and bottom circles
    beginShape(TRIANGLE_FAN)
    vertex(0, 0, half_height)
    for i in range(sides + 1):
        x = cos(i * angle) * radius
        y = sin(i * angle) * radius
        vertex(x, y, half_height)
    endShape()

    beginShape(TRIANGLE_FAN)
    vertex(0, 0, -half_height)
    for i in range(sides + 1):
        x = cos(i * angle) * radius
        y = sin(i * angle) * radius
        vertex(x, y, -half_height)
    endShape()

    # Draw the side surface
    beginShape(QUAD_STRIP)
    for i in range(sides + 1):
        x = cos(i * angle) * radius
        y = sin(i * angle) * radius
        vertex(x, y, half_height)
        vertex(x, y, -half_height)
    endShape()

def global_to_local(global_goal, pos, xrot):
    # Translate global goal relative to the arm's origin
    dx = global_goal[0] - pos[0]
    dy = global_goal[1] - pos[1]
    dz = global_goal[2] - pos[2]
    
    # Apply the inverse rotation about the X-axis.
    # The rotation matrix for an X-axis rotation by an angle 'xrot' is:
    # [ 1      0           0      ]
    # [ 0 cos(xrot) -sin(xrot) ]
    # [ 0 sin(xrot)  cos(xrot) ]
    # So the inverse (or rotation by -xrot) is:
    local_y = cos(-xrot)*dy - sin(-xrot)*dz
    local_z = sin(-xrot)*dy + cos(-xrot)*dz
    
    # x coordinate is unchanged (assuming no additional rotation about Y or Z)
    local_x = dx
    return [local_x, local_y, local_z]

def local_to_global_box(local_offset, box_pos, box_rot):
    # unpack local offset values (assume center of box as origin)
    lx, ly, lz = local_offset[0], local_offset[1], local_offset[2]
    rx, ry, rz = box_rot[0], box_rot[1], box_rot[2]
    
    # First, rotate about the X-axis:
    x1 = lx
    y1 = ly * cos(rx) - lz * sin(rx)
    z1 = ly * sin(rx) + lz * cos(rx)
    
    # Then, rotate about the Y-axis:
    x2 = x1 * cos(ry) + z1 * sin(ry)
    y2 = y1
    z2 = -x1 * sin(ry) + z1 * cos(ry)
    
    # Then, rotate about the Z-axis:
    x3 = x2 * cos(rz) - y2 * sin(rz)
    y3 = x2 * sin(rz) + y2 * cos(rz)
    z3 = z2
    
    # Finally, translate by the global box position.
    global_x = box_pos[0] + x3
    global_y = box_pos[1] + y3
    global_z = box_pos[2] + z3
    return [global_x, global_y, global_z]

def draw_reference_frame():
    stroke(255, 0, 0) # x
    line(0,0,0, axis_length,0,0)
    stroke(0, 255, 0) # y
    line(0,0,0, 0,axis_length,0)
    stroke(0, 0, 255) # z
    line(0,0,0, 0,0,axis_length)
    stroke(0)