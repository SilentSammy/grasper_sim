# Helper functions
def compute_box_moment(mass, dims):
    # dims: [width, height, depth]
    w, h, d = dims[0], dims[1], dims[2]
    I_x = (1/12.0) * mass * (h*h + d*d)
    I_y = (1/12.0) * mass * (w*w + d*d)
    I_z = (1/12.0) * mass * (w*w + h*h)
    return [I_x, I_y, I_z]

def draw_reference_frame():
    # Draws the axes to visualize the coordinate system
    stroke(255, 0, 0)  # x-axis (red)
    line(0, 0, 0, axis_length, 0, 0)
    stroke(0, 255, 0)  # y-axis (green)
    line(0, 0, 0, 0, axis_length, 0)
    stroke(0, 0, 255)  # z-axis (blue)
    line(0, 0, 0, 0, 0, axis_length)
    stroke(0)

# Global drawing parameters
last_time = 0
dt = 1  # Time step for dynamics
axis_length = 10
shape_scale = 2
zoom = 10.0
angleX = 0
angleY = 0

# Box parameters (global FoR)
box_pos = [15, 0, 0]
box_dims = [8, 16, 4]
box_rot = [0, 0, 0]  # [rotation about X, Y, Z]
box_mass = 1
box_moment = compute_box_moment(box_mass, box_dims)

# Dynamics variables
box_vel = [0, 0, 0]
box_accel = [0, 0, 0]  # The box will accelerate along the global Y axis
box_force = [0, 0, 0]
box_avel = [0, 0, 0]
box_aaccel = [0, 0, 0]
box_torque = [0, 0, 0]

# Goal box
goal_pos = box_pos[:]
goal_rot = [0, 0, 0]
goal_dims = [dim * 1.25 for dim in box_dims]
goal_vel = 20
goal_avel = PI/2

# PID control for positions
Kpp = 2
Kip = 0.3
Kdp = 4
Kpr = 0.5
Kir = 1
Kdr = 5
error = [0, 0, 0, 0, 0, 0]
prev_error = [0, 0, 0, 0, 0, 0]
error_sum = [0, 0, 0, 0, 0, 0]

def set_up_drawing():
    global last_time, dt

    # Update time step
    current_time = millis()
    dt = (current_time - last_time) / 1000.0 if last_time != 0 else 0
    last_time = current_time

    # Configure background, lights, and camera
    background(255)
    smooth()
    lights()
    directionalLight(51, 102, 126, -1, 0, 0)
    
    # Set origin at the center of the window
    translate(width/2, height/2, 0)
    
    # Apply zoom and view rotations
    scale(zoom)
    rotateX(angleX)
    rotateY(angleY)
    # Adjust for Processing's P3D default coordinate system
    rotateX(PI/2)
    rotateZ(PI/2)
    
    strokeWeight(0.1)
    draw_reference_frame()

def setup():
    size(600, 600, P3D)

def draw():
    set_up_drawing()
    control()  # Control box rotation via keys, if desired
    
    update_box()  # Update dynamics: velocity and position
    
    # Draw the box with its own transformations
    pushMatrix()
    translate(box_pos[0], box_pos[1], box_pos[2])
    rotateZ(box_rot[2])
    rotateY(box_rot[1])
    rotateX(box_rot[0])
    draw_reference_frame()
    fill(139, 69, 19)  # Brown color
    box(box_dims[0], box_dims[1], box_dims[2])
    popMatrix()

    # Draw the goal box
    pushMatrix()
    translate(goal_pos[0], goal_pos[1], goal_pos[2])
    rotateZ(goal_rot[2])
    rotateY(goal_rot[1])
    rotateX(goal_rot[0])
    draw_reference_frame()
    fill(0, 255, 0, 127)  # Green color with 50% transparency
    box(goal_dims[0], goal_dims[1], goal_dims[2])
    popMatrix()

def update_box():
    global box_vel, box_pos, box_avel, box_rot, box_accel, box_aaccel, dt, box_mass, box_force, box_torque, box_moment

    # Update translational dynamics: a = F / m
    for i in range(3):
        box_accel[i] = box_force[i] / box_mass
        box_vel[i] += box_accel[i] * dt
        box_pos[i] += box_vel[i] * dt

    # Update rotational dynamics: α = torque / moment of inertia
    for i in range(3):
        # Compute angular acceleration along each axis
        box_aaccel[i] = box_torque[i] / box_moment[i]
        box_avel[i] += box_aaccel[i] * dt
        box_rot[i] += box_avel[i] * dt

# User input handling
def control():
    global box_force, box_torque, box_vel, box_avel, dt, error, prev_error, error_sum, goal_pos, goal_vel, goal_avel, goal_rot
    force_mag = 10    # magnitude of translational force
    torque_mag = 10  # magnitude of torque

    if keyPressed:
        # move the goal based on keyboard input
        goal_pos[0] += (goal_vel if key == 's' else -goal_vel if key == 'w' else 0) * dt
        goal_pos[1] += (goal_vel if key == 'a' else -goal_vel if key == 'd' else 0) * dt
        goal_pos[2] += (goal_vel if key == 'z' else -goal_vel if key == 'x' else 0) * dt

        # Rotate the goal based on keyboard input
        goal_rot[0] += ((-goal_avel if key == CODED and keyCode == LEFT else goal_avel if key == CODED and keyCode == RIGHT else 0) * dt)
        goal_rot[1] += ((-goal_avel if key == CODED and keyCode == UP else goal_avel if key == CODED and keyCode == DOWN else 0) * dt)
        goal_rot[2] += ((-goal_avel if key == ',' else goal_avel if key == '.' else 0) * dt)

    # If g is pressed call PID()
    # if keyPressed and key == 'g':
    if True:
        PID()
    else:
        # set forces to 0
        box_force = [0, 0, 0]
        box_torque = [0, 0, 0]

        # Reset PID
        error = [0, 0, 0, 0, 0, 0]
        prev_error = [0, 0, 0, 0, 0, 0]
        error_sum = [0, 0, 0, 0, 0, 0]

def PID():
    global error, prev_error, error_sum, box_force, box_torque, box_pos, goal_pos, box_rot, goal_rot, dt
    # Update PID for translation (indices 0,1,2)
    for i in range(3):
        error[i] = goal_pos[i] - box_pos[i]
        error_sum[i] += error[i] * dt
        derivative = (error[i] - prev_error[i]) / dt if dt != 0 else 0
        output = Kpp * error[i] + Kip * error_sum[i] + Kdp * derivative
        box_force[i] = output
        prev_error[i] = error[i]

    # Update PID for rotation (indices 3,4,5)
    for i in range(3):
        # Compute rotational error for axis i using offset index i+3
        error[i+3] = goal_rot[i] - box_rot[i]
        error_sum[i+3] += error[i+3] * dt
        derivative = (error[i+3] - prev_error[i+3]) / dt if dt != 0 else 0
        # Scale the PID output by the moment of inertia on that axis
        output = (Kpr * error[i+3] + Kir * error_sum[i+3] + Kdr * derivative) * box_moment[i]
        box_torque[i] = output
        prev_error[i+3] = error[i+3]

def mouseDragged():
    global angleX, angleY
    # Adjust view rotation based on mouse dragging
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01
