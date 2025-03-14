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

angleX = 0
angleY = 0

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
    global box_force, box_torque
    force_mag = 10    # magnitude of translational force
    torque_mag = 10  # magnitude of torque

    if keyPressed:
        # Set translational force based on keys:
        box_force[0] = -force_mag if key == 'w' else force_mag if key == 's' else 0
        box_force[1] = -force_mag if key == 'd' else force_mag if key == 'a' else 0
        box_force[2] = -force_mag if key == 'x' else force_mag if key == 'z' else 0
        
        # Set torque based on keys:
        box_torque[0] = -torque_mag if key == CODED and keyCode == LEFT else torque_mag if key == CODED and keyCode == RIGHT else 0
        box_torque[1] = -torque_mag if key == CODED and keyCode == UP else torque_mag if key == CODED and keyCode == DOWN else 0
        box_torque[2] = -torque_mag if key == ',' else torque_mag if key == '.' else 0
    else:
        box_force = [0, 0, 0]
        box_torque = [0, 0, 0]

def mouseDragged():
    global angleX, angleY
    # Adjust view rotation based on mouse dragging
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01
