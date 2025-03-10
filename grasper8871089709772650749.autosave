# Drawing setup
angleX = 0
angleY = 0
zoom = 10.0
axis_length = 10
shape_scale = 1

# Kinematics
selected_link = 0
links = [
    {"r": 0, "alpha": -PI/2, "d": 0, "theta": 0, "type": 0},
    {"r": 10, "alpha": 0, "d": 0, "theta": -PI/4, "type": 0},
    {"r": 10, "alpha": 0, "d": 0, "theta": PI/2, "type": 0},
]
goal = [15, 0, 5]

# Main Processing functions
def setup():
    size(600, 600, P3D)

def draw():
    set_up_drawing()
    control()

    draw_goal()

    pushMatrix()
    # Draw base node at origin
    draw_node(0, selected_link == 0)
    
    for i in range(len(links)):
        link = links[i]
        # Draw the connector
        draw_connector(link)
        
        # Draw the node
        apply_transformations(link)
        draw_node(links[i+1]["type"] if i+1 < len(links) else 2, i == selected_link-1)
    popMatrix()

# Helper drawing functions
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

def draw_node(type=0, highlight=False):
    # Draw axes
    strokeWeight(0.2 if highlight else 0.1)
    stroke(255, 0, 0) # x
    line(0,0,0, axis_length,0,0)
    stroke(0, 255, 0) # y
    line(0,0,0, 0,axis_length,0)
    stroke(0, 0, 255) # z
    line(0,0,0, 0,0,axis_length)

    # Draw a prism facing Z
    stroke(0, 0, 0)

    # Choose color
    fill(128, 128, 128)
    if highlight:
        fill(255, 255, 0)
    elif type == 2:
        fill(154, 205, 50)
    
    # Draw the node based on the type
    if type == 0:
        draw_cylinder(1 * shape_scale, 4 * shape_scale)
    elif type == 1:
        box(2 * shape_scale, 2 * shape_scale, 3 * shape_scale)
    elif type == 2:
        strokeWeight(0)
        sphere(2 * shape_scale)

def draw_goal():
    # Draw a translucent sphere at the goal position
    pushMatrix()
    strokeWeight(0)
    fill(255, 0, 0, 100)
    translate(goal[0], goal[1], goal[2])
    sphere(2.5 * shape_scale)
    popMatrix()

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

# Forwards kinematics
def control():
    global selected_link
    speed = 0.25
    avel = 0.025
    # If a digit key is pressed, update the selected link.
    if keyPressed:
        # Select the joint based on the digit key
        if str(key).isdigit():
            idx = int(key) - 1  # Adjust index to match key 1 to joint 0, etc.
            if 0 <= idx < len(links):
                selected_link = idx

        # Control the selected joint (revolute or prismatic)
        if links[selected_link]["type"] == 0:
            links[selected_link]["theta"] += -avel if key == 'a' else avel if key == 'd' else 0
        else:
            links[selected_link]["d"] += -speed if key == 'a' else speed if key == 'd' else 0
        
        # Control the goal position
        goal[0] += speed * (-1 if key == CODED and keyCode == LEFT else 1 if key == CODED and keyCode == RIGHT else 0)
        goal[1] += speed * (-1 if key == CODED and keyCode == UP   else 1 if key == CODED and keyCode == DOWN  else 0)
        goal[2] += speed * (-1 if key == 's' else 1 if key == 'w' else 0)

        # Move to goal
        if key == 'g':
            move_to_goal()

def apply_transformations(link):
    theta = link["theta"]
    d = link["d"]
    r = link["r"]
    alpha = link["alpha"]
    
    rotateZ(theta)
    translate(0, 0, d)
    translate(r, 0, 0)
    rotateX(alpha)

# Inverse kinematics
def move_to_goal():
    heading = goal_heading()
    links[0]["theta"] = heading
    pos2D = get_flat_pos(heading)
    theta1, theta2 = inverse_kinematics(pos2D)
    if theta1 is not None and theta2 is not None:
        links[1]["theta"] = theta1
        links[2]["theta"] = theta2
    pass

def goal_heading():
    projected_x = goal[0]
    projected_y = goal[1]
    angle = atan2(projected_y, projected_x)
    # println("Rotation along vertical axis (radians):", angle)
    # println("Rotation along vertical axis (degrees):" + str(degrees(angle)))
    return angle

def get_flat_pos(heading):
    # Unit vector along the heading (in the x-y plane)
    u_x = cos(heading)
    u_y = sin(heading)
    # Projection along the heading direction
    pos_along_heading = goal[0]*u_x + goal[1]*u_y
    # The vertical coordinate is already Z
    pos_z = goal[2]
    pos2D = [pos_along_heading, pos_z]
    println("2D position:" + str(pos2D))
    return pos2D

def inverse_kinematics(pos2D):
    # Inputs: link lengths from kinematics (assumed positive)
    l1 = links[1]["r"]
    l2 = links[2]["r"]
    
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

# Camera control
def mouseDragged():
    global angleX, angleY
    # Update rotation angles based on the change in mouse position.
    # Adjust the sensitivity by multiplying with a small factor (e.g., 0.01).
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01
