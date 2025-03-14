class Box:
    def __init__(self):
        # Box Dynamics in global FoR
        self.box_pos = [15, 0, 0]
        self.box_dims = [8, 16, 4]
        self.box_rot = [0, 0, 0]  # [rotation about X, Y, Z]
        self.box_mass = 1
        self.box_moment = Box.compute_box_moment(self.box_mass, self.box_dims)
        self.box_vel = [0, 0, 0]
        self.box_accel = [0, 0, 0]  # The box will accelerate along the global Y axis
        self.box_force = [0, 0, 1]
        self.box_avel = [0, 0, 0]
        self.box_aaccel = [0, 0, 0]
        self.box_torque = [0, 0, 0]

        # Goal box
        self.goal_pos = self.box_pos[:]
        self.goal_rot = [0, 0, 0]
        self.goal_dims = [dim * 1.25 for dim in self.box_dims]
        self.goal_vel = 20
        self.goal_avel = PI/2

        # PID control for positions
        self.Kpp = 0.15
        self.Kip = 0.3
        self.Kdp = 8
        self.Kpr = 0.5
        self.Kir = 1
        self.Kdr = 15
        self.error = [0, 0, 0, 0, 0, 0]
        self.prev_error = [0, 0, 0, 0, 0, 0]
        self.error_sum = [0, 0, 0, 0, 0, 0]
    
        # Box force positions
        self.forces = [
            [0, 5, self.box_dims[2]/2, 0, 0, 0],
            [0, -5, self.box_dims[2]/2, 0, 0, 0],
            [0, 0, -self.box_dims[2]/2, 0, 0, 3]
        ]

    # Dynamics
    def update_box_dynamics(self):
        self.update_net_force()
        self.update_acceleration()
        self.update_velocity()
        self.update_position()

    def update_acceleration(self):
        # Update translational acceleration: a = F / m
        for i in range(3):
            self.box_accel[i] = self.box_force[i] / self.box_mass
        # Update rotational acceleration: α = torque / moment of inertia
        for i in range(3):
            self.box_aaccel[i] = self.box_torque[i] / self.box_moment[i]

    def update_velocity(self):
        # Update translational velocity: v = v + a * dt
        for i in range(3):
            self.box_vel[i] += self.box_accel[i] * dt
        # Update rotational velocity: ω = ω + α * dt
        for i in range(3):
            self.box_avel[i] += self.box_aaccel[i] * dt

    def update_position(self):
        # Update translational position: x = x + v * dt
        for i in range(3):
            self.box_pos[i] += self.box_vel[i] * dt
        # Update rotational position: θ = θ + ω * dt
        for i in range(3):
            self.box_rot[i] += self.box_avel[i] * dt

    def draw_box(self):
        # Draw the box with its own transformations
        pushMatrix()
        translate(self.box_pos[0], self.box_pos[1], self.box_pos[2])
        rotateZ(self.box_rot[2])
        rotateY(self.box_rot[1])
        rotateX(self.box_rot[0])
        draw_reference_frame()
        # Draw the forces
        for force in self.forces:
            self.draw_force(force)
        fill(139, 69, 19)
        box(self.box_dims[0], self.box_dims[1], self.box_dims[2])
        popMatrix()

        # Draw the goal box
        pushMatrix()
        translate(self.goal_pos[0], self.goal_pos[1], self.goal_pos[2])
        rotateZ(self.goal_rot[2])
        rotateY(self.goal_rot[1])
        rotateX(self.goal_rot[0])
        draw_reference_frame()
        fill(0, 255, 0, 127)  # Green color with 50% transparency
        box(self.goal_dims[0], self.goal_dims[1], self.goal_dims[2])
        popMatrix()

    def update_net_force(self):
        # Compute net force and torque as defined in the local coordinate frame:
        net_force_local = [0, 0, 0]
        net_torque_local = [0, 0, 0]
        for f in self.forces:
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
        R = rotation_matrix(self.box_rot[0], self.box_rot[1], self.box_rot[2])
        global_net_force = mat_vec_mult(R, net_force_local)
        
        # (Optionally, you might also transform the torque if required.)
        global_net_torque = mat_vec_mult(R, net_torque_local)
        
        self.box_force = global_net_force
        self.box_torque = global_net_torque

    def set_necessary_forces(self, desired_net_force, desired_net_torque):
        # Convert desired force and torque from global frame to local frame:
        R = rotation_matrix(self.box_rot[0], self.box_rot[1], self.box_rot[2])
        R_T = transpose(R)  # since R is orthonormal, its transpose is its inverse.
        desired_net_force_local = mat_vec_mult(R_T, desired_net_force)
        desired_net_torque_local = mat_vec_mult(R_T, desired_net_torque)

        self.forces[0][3:6] = [0, 0, -25]
        self.forces[1][3:6] = [0, 0, -25]
        self.forces[2][3:6] = [0, 0, 50]

        # For the Y component, we'll apply 0.25 to both of the top thrusters, and 0.5 to the bottom thruster.
        self.forces[0][4] += 0.25 * desired_net_force_local[1]
        self.forces[1][4] += 0.25 * desired_net_force_local[1]
        self.forces[2][4] += 0.5 * desired_net_force_local[1]

        # Same for the X component: 0.25 to both top thrusters, 0.5 to the bottom thruster.
        self.forces[0][3] += 0.25 * desired_net_force_local[0]
        self.forces[1][3] += 0.25 * desired_net_force_local[0]
        self.forces[2][3] += 0.5 * desired_net_force_local[0]

        # For the Z component, apply the full force to the bottom thruster if it's positive (pushing up), and distribute evenly to the top thrusters if it's negative (pushing down).
        if desired_net_force_local[2] > 0:
            self.forces[2][5] += desired_net_force_local[2]
        else:
            self.forces[0][5] += desired_net_force_local[2] * 0.5
            self.forces[1][5] += desired_net_force_local[2] * 0.5
        
        # For torque it's trickier. We need to consider the distance from the center of mass to the thrusters.
        # Let's start with Z. We only need to use top two thrusters.
        #   r = abs(self.forces[0][1]) (and similarly for self.forces[1][1]).
        r = abs(self.forces[0][1])
        if r != 0:
            F_torque = -desired_net_torque_local[2] / (2.0 * r)
        else:
            F_torque = 0
        # Apply the differential X force to generate torque:
        self.forces[0][3] += F_torque  # adds force in +X at Y = +r
        self.forces[1][3] -= F_torque  # subtracts force in +X at Y = -r

        # --- Torque allocation for rotation about Y (pitch) ---
        # Use the difference in Z positions.
        z_top = self.forces[0][2]  # assuming both top thrusters have the same z coordinate.
        if z_top != 0:
            # Extra force for each top thruster:
            F_torque_top = desired_net_torque_local[1] / (4.0 * z_top)
            # And for the bottom thruster:
            F_torque_bottom = -2.0 * F_torque_top
        else:
            F_torque_top = 0
            F_torque_bottom = 0

        # Add the differential X forces for rotation about Y.
        self.forces[0][3] += F_torque_top
        self.forces[1][3] += F_torque_top
        self.forces[2][3] += F_torque_bottom

        # --- Torque allocation for rotation about X (roll) ---
        if z_top != 0:
            F_torque_top_x = -desired_net_torque_local[0] / (4.0 * z_top)
            F_torque_bot_x = -2.0 * F_torque_top_x  # equals desired_net_torque_local[0] / (2.0 * z_top)
        else:
            F_torque_top_x = 0
            F_torque_bot_x = 0

        # Apply the extra Y forces for rotation about X:
        self.forces[0][4] += F_torque_top_x
        self.forces[1][4] += F_torque_top_x
        self.forces[2][4] += F_torque_bot_x

    def PID(self):
        pid_force = [0, 0, 0]
        pid_torque = [0, 0, 0]

        # Update PID for translation (indices 0,1,2)
        for i in range(3):
            self.error[i] = self.goal_pos[i] - self.box_pos[i]
            self.error_sum[i] += self.error[i] * dt
            derivative = (self.error[i] - self.prev_error[i]) / dt if dt != 0 else 0
            output = self.Kpp * self.error[i] + self.Kip * self.error_sum[i] + self.Kdp * derivative
            pid_force[i] = output
            self.prev_error[i] = self.error[i]

        # Update PID for rotation (indices 3,4,5)
        for i in range(3):
            # Compute rotational error for axis i using offset index i+3
            self.error[i+3] = self.goal_rot[i] - self.box_rot[i]
            self.error_sum[i+3] += self.error[i+3] * dt
            derivative = (self.error[i+3] - self.prev_error[i+3]) / dt if dt != 0 else 0
            # Scale the PID output by the moment of inertia on that axis
            output = (self.Kpr * self.error[i+3] + self.Kir * self.error_sum[i+3] + self.Kdr * derivative) * self.box_moment[i]
            pid_torque[i] = output
            self.prev_error[i+3] = self.error[i+3]

        return pid_force, pid_torque

    def stop_thrust(self):
        for force in self.forces:
            force[3:6] = [0, 0, 0]

    def thrust_to_goal(self):
        desired_net_force, desired_net_torque = self.PID()
        self.set_necessary_forces(desired_net_force, desired_net_torque)

    @staticmethod    
    def compute_box_moment(mass, dims):
        # dims: [width, height, depth]
        w, h, d = dims[0], dims[1], dims[2]
        I_x = (1/12.0) * mass * (h*h + d*d)
        I_y = (1/12.0) * mass * (w*w + d*d)
        I_z = (1/12.0) * mass * (w*w + h*h)
        return [I_x, I_y, I_z]

    @staticmethod
    def draw_force(force, force_scalar = 0.25):
        pushMatrix()
        translate(force[0], force[1], force[2])
        fill(255, 0, 0)  # Red color
        sphere(0.25)

        # Draw the force vector
        stroke(255, 140, 0)  # Dark orange color
        line(0, 0, 0, -force[3] * force_scalar, -force[4] * force_scalar, -force[5] * force_scalar)

        popMatrix()

# Drawer functions
def draw_reference_frame():
    # Draws the axes to visualize the coordinate system
    stroke(255, 0, 0)  # x-axis (red)
    line(0, 0, 0, axis_length, 0, 0)
    stroke(0, 255, 0)  # y-axis (green)
    line(0, 0, 0, 0, axis_length, 0)
    stroke(0, 0, 255)  # z-axis (blue)
    line(0, 0, 0, 0, 0, axis_length)
    stroke(0)

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

# Global drawing parameters
dt = 0 # REQUIRED FOR BOX AND MANIPULATOR CLASSES
last_time = 0
axis_length = 10
shape_scale = 2
zoom = 10.0
angleX = 0
angleY = 0

# Box
box_obj = Box()

# Main Processing functions
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
    
    control()  # Handle user input (goal position and rotation)
    box_obj.thrust_to_goal()  # Apply PID control to reach the goal
    box_obj.update_box_dynamics()  # Update force, accel, vel and pos, both linear and angular
    box_obj.draw_box() # Draw the box

# User input handling
def control():
    goal_vel = 10
    goal_avel = PI/2
    if keyPressed:
        # move the goal based on keyboard input
        box_obj.goal_pos[0] += (goal_vel if key == 's' else -goal_vel if key == 'w' else 0) * dt
        box_obj.goal_pos[1] += (goal_vel if key == 'a' else -goal_vel if key == 'd' else 0) * dt
        box_obj.goal_pos[2] += (goal_vel if key == 'z' else -goal_vel if key == 'x' else 0) * dt

        # Rotate the goal based on keyboard input
        box_obj.goal_rot[0] += ((-goal_avel if key == CODED and keyCode == LEFT else goal_avel if key == CODED and keyCode == RIGHT else 0) * dt)
        box_obj.goal_rot[1] += ((-goal_avel if key == CODED and keyCode == UP else goal_avel if key == CODED and keyCode == DOWN else 0) * dt)
        box_obj.goal_rot[2] += ((-goal_avel if key == ',' else goal_avel if key == '.' else 0) * dt)

def mouseDragged():
    global angleX, angleY
    # Adjust view rotation based on mouse dragging
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01
