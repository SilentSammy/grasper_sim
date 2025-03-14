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
    strokeWeight(0.1)
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

# Box Dynamics in global FoR
box_pos = [15, 0, 0]
box_dims = [8, 16, 4]
box_rot = [0, 0, 0]  # [rotation about X, Y, Z]
box_mass = 1
box_moment = compute_box_moment(box_mass, box_dims)
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

# Box force positions
forces = [
    [0, 5, box_dims[2]/2, 0, 0, 0],
    [0, -5, box_dims[2]/2, 0, 0, 0],
    [0, 0, -box_dims[2]/2, 0, 0, 3]
]

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

    # Draw the forces
    for force in forces:
        draw_force(force)

    fill(139, 69, 19)  # Brown color with 50% transparency
    box(box_dims[0], box_dims[1], box_dims[2])

    popMatrix()

    # Draw the goal box
    pushMatrix()
    translate(goal_pos[0], goal_pos[1], goal_pos[2])
    rotateZ(goal_rot[2])
    rotateY(goal_rot[1])
    rotateX(goal_rot[0])
    # draw_reference_frame()
    fill(0, 255, 0, 127)  # Green color with 50% transparency
    # box(goal_dims[0], goal_dims[1], goal_dims[2])
    popMatrix()

def draw_force(force, force_scalar = 5):
    pushMatrix()
    strokeWeight(0.2)
    translate(force[0], force[1], force[2])
    fill(255, 0, 0)  # Red color
    sphere(0.25)

    # Draw the force vector
    stroke(255, 140, 0)  # Dark orange color
    line(0, 0, 0, -force[3] * force_scalar, -force[4] * force_scalar, -force[5] * force_scalar)

    popMatrix()

# Dynamics
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

def compute_net_force():
    global box_force, box_torque, forces, box_rot
    # Compute net force and torque as defined in the local coordinate frame:
    net_force_local = [0, 0, 0]
    net_torque_local = [0, 0, 0]
    for f in forces:
        # f[0:3] = actuator position (local coordinates, fixed relative to the box)
        # f[3:6] = actuator force (defined in the box's local frame)
        net_force_local[0] += f[3]
        net_force_local[1] += f[4]
        net_force_local[2] += f[5]
        
        # Local torque computed as r x F:
        net_torque_local[0] += f[1] * f[5] - f[2] * f[4]
        net_torque_local[1] += f[2] * f[3] - f[0] * f[5]
        net_torque_local[2] += f[0] * f[4] - f[1] * f[3]
    
    # Transform the net force from local to global frame using the box's rotation.
    R = rotation_matrix(box_rot[0], box_rot[1], box_rot[2])
    global_net_force = mat_vec_mult(R, net_force_local)
    
    # (Optionally, you might also transform the torque if required.)
    global_net_torque = mat_vec_mult(R, net_torque_local)
    
    box_force = global_net_force
    box_torque = global_net_torque

def compute_necessary_forces(desired_net_force, desired_net_torque):
    # Convert desired force and torque from global frame to local frame:
    R = rotation_matrix(box_rot[0], box_rot[1], box_rot[2])
    R_T = transpose(R)  # since R is orthonormal, its transpose is its inverse.
    desired_net_force_local = mat_vec_mult(R_T, desired_net_force)
    desired_net_torque_local = mat_vec_mult(R_T, desired_net_torque)

    forces[0][3:6] = [0, 0, 0]
    forces[1][3:6] = [0, 0, 0]
    forces[2][3:6] = [0, 0, 0]

    # For the Y component, we'll apply 0.25 to both of the top thrusters, and 0.5 to the bottom thruster.
    forces[0][4] += 0.25 * desired_net_force_local[1]
    forces[1][4] += 0.25 * desired_net_force_local[1]
    forces[2][4] += 0.5 * desired_net_force_local[1]

    # Same for the X component: 0.25 to both top thrusters, 0.5 to the bottom thruster.
    forces[0][3] += 0.25 * desired_net_force_local[0]
    forces[1][3] += 0.25 * desired_net_force_local[0]
    forces[2][3] += 0.5 * desired_net_force_local[0]

    # For the Z component, apply the full force to the bottom thruster if it's positive (pushing up), and distribute evenly to the top thrusters if it's negative (pushing down).
    if desired_net_force_local[2] > 0:
        forces[2][5] += desired_net_force_local[2]
    else:
        forces[0][5] += desired_net_force_local[2] * 0.5
        forces[1][5] += desired_net_force_local[2] * 0.5
    
    # For torque it's trickier. We need to consider the distance from the center of mass to the thrusters.
    # Let's start with Z. We only need to use top two thrusters.
    #   r = abs(forces[0][1]) (and similarly for forces[1][1]).
    r = abs(forces[0][1])
    if r != 0:
        F_torque = -desired_net_torque_local[2] / (2.0 * r)
    else:
        F_torque = 0
    # Apply the differential X force to generate torque:
    forces[0][3] += F_torque  # adds force in +X at Y = +r
    forces[1][3] -= F_torque  # subtracts force in +X at Y = -r

    # --- Torque allocation for rotation about Y (pitch) ---
    # Use the difference in Z positions.
    z_top = forces[0][2]  # assuming both top thrusters have the same z coordinate.
    if z_top != 0:
        # Extra force for each top thruster:
        F_torque_top = desired_net_torque_local[1] / (4.0 * z_top)
        # And for the bottom thruster:
        F_torque_bottom = -2.0 * F_torque_top
    else:
        F_torque_top = 0
        F_torque_bottom = 0

    # Add the differential X forces for rotation about Y.
    forces[0][3] += F_torque_top
    forces[1][3] += F_torque_top
    forces[2][3] += F_torque_bottom

    # --- Torque allocation for rotation about X (roll) ---
    if z_top != 0:
        F_torque_top_x = -desired_net_torque_local[0] / (4.0 * z_top)
        F_torque_bot_x = -2.0 * F_torque_top_x  # equals desired_net_torque_local[0] / (2.0 * z_top)
    else:
        F_torque_top_x = 0
        F_torque_bot_x = 0

    # Apply the extra Y forces for rotation about X:
    forces[0][4] += F_torque_top_x
    forces[1][4] += F_torque_top_x
    forces[2][4] += F_torque_bot_x

# Helpers
def transpose(mat):
    # Transpose a 3x3 matrix.
    return [[mat[j][i] for j in range(3)] for i in range(3)]

def rotation_matrix(rx, ry, rz):
    # Assuming rx, ry, rz are rotations about X, Y, Z axes respectively.
    # Note: The multiplication order matters; adjust based on your convention.
    cx = cos(rx)
    sx = sin(rx)
    cy = cos(ry)
    sy = sin(ry)
    cz = cos(rz)
    sz = sin(rz)
    
    # Rotation about X axis:
    Rx = [
        [1, 0, 0],
        [0, cx, -sx],
        [0, sx, cx]
    ]
    # Rotation about Y axis:
    Ry = [
        [cy, 0, sy],
        [0, 1, 0],
        [-sy, 0, cy]
    ]
    # Rotation about Z axis:
    Rz = [
        [cz, -sz, 0],
        [sz, cz, 0],
        [0, 0, 1]
    ]
    # Combine rotations: R = Rz * Ry * Rx
    Rzy = mat_mult(Rz, Ry)
    R = mat_mult(Rzy, Rx)
    return R

def mat_dot_vector(mat, vec):
    result = []
    for row in mat:
        s = 0
        for j in range(len(vec)):
            s += row[j] * vec[j]
        result.append(s)
    return result

def mat_mult(A, B):
    # Multiply two 3x3 matrices A and B.
    result = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            s = 0
            for k in range(3):
                s += A[i][k] * B[k][j]
            result[i][j] = s
    return result

def mat_vec_mult(mat, vec):
    # Multiply 3x3 matrix with a 3-element vector.
    result = [0, 0, 0]
    for i in range(3):
        s = 0
        for j in range(3):
            s += mat[i][j] * vec[j]
        result[i] = s
    return result

# User input handling
def control():
    global box_force, box_torque, box_vel, box_avel, dt, error, prev_error, error_sum, goal_pos, goal_vel, goal_avel, goal_rot
    force_mag = 10    # magnitude of translational force
    torque_mag = 10  # magnitude of torque

    desired_net_force = [0, 0, 0]
    desired_net_torque = [0, 0, 0]
    if keyPressed:
        # move the goal based on keyboard input
        # goal_pos[0] += (goal_vel if key == 's' else -goal_vel if key == 'w' else 0) * dt
        # goal_pos[1] += (goal_vel if key == 'a' else -goal_vel if key == 'd' else 0) * dt
        # goal_pos[2] += (goal_vel if key == 'z' else -goal_vel if key == 'x' else 0) * dt

        # Rotate the goal based on keyboard input
        # goal_rot[0] += ((-goal_avel if key == CODED and keyCode == LEFT else goal_avel if key == CODED and keyCode == RIGHT else 0) * dt)
        # goal_rot[1] += ((-goal_avel if key == CODED and keyCode == UP else goal_avel if key == CODED and keyCode == DOWN else 0) * dt)
        # goal_rot[2] += ((-goal_avel if key == ',' else goal_avel if key == '.' else 0) * dt)

        # Set translational force based on keys:
        desired_net_force[0] = -force_mag if key == 'w' else force_mag if key == 's' else 0
        desired_net_force[1] = -force_mag if key == 'd' else force_mag if key == 'a' else 0
        desired_net_force[2] = -force_mag if key == 'x' else force_mag if key == 'z' else 0
        
        # Set torque based on keys:
        desired_net_torque[0] = -torque_mag if key == CODED and keyCode == LEFT else torque_mag if key == CODED and keyCode == RIGHT else 0
        desired_net_torque[1] = -torque_mag if key == CODED and keyCode == UP else torque_mag if key == CODED and keyCode == DOWN else 0
        desired_net_torque[2] = -torque_mag if key == ',' else torque_mag if key == '.' else 0

    # Compute the necessary forces to achieve the desired net force and torque
    compute_necessary_forces(desired_net_force, desired_net_torque)

    # Compute net forces
    compute_net_force()

    # Compute differences between desired and obtained outcomes
    force_diff = [round(desired_net_force[i] - box_force[i], 2) for i in range(3)]
    torque_diff = [round(desired_net_torque[i] - box_torque[i], 2) for i in range(3)]
    force_diff_norm = round(sqrt(sum([x*x for x in force_diff])), 2)
    torque_diff_norm = round(sqrt(sum([x*x for x in torque_diff])), 2)

    # Define acceptable thresholds
    force_threshold = 0.1
    torque_threshold = 0.1

    # Print everything in one line
    print("F_diff: {} (|F|={:.3f}), T_diff: {} (|T|={:.3f}){}".format(
        force_diff, force_diff_norm, torque_diff, torque_diff_norm,
        " Warning: Diff too high!" if force_diff_norm > force_threshold or torque_diff_norm > torque_threshold else ""
    ))

    # If g is pressed call PID()
    # if keyPressed and key == 'g':
    #     PID()
    # else:
    #     # set forces to 0
    #     box_force = [0, 0, 0]
    #     box_torque = [0, 0, 0]

    #     # Reset PID
    #     error = [0, 0, 0, 0, 0, 0]
    #     prev_error = [0, 0, 0, 0, 0, 0]
    #     error_sum = [0, 0, 0, 0, 0, 0]

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
