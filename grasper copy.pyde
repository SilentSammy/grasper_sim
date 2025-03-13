class Manipulator:
    end_effector_radius = 2
    shape_scale = 2

    def __init__(self, pos, rot):
        self.pos = pos
        self.rot = rot
        self.links = [
            {"r": 0, "alpha": -PI/2, "d": 0, "theta": 0, "type": 0},
            {"r": 10, "alpha": 0, "d": 0, "theta": -PI/4, "type": 0},
            {"r": 10, "alpha": 0, "d": 0, "theta": PI/2, "type": 0},
        ]
        self.global_goal = [0, 0, 0] # in global FoR

    @property
    def local_goal(self):
        return Manipulator.global_to_local(self.global_goal, self.pos, self.rot[0])

    # Drawing
    def draw(self):
        pushMatrix()
        
        translate(self.pos[0], self.pos[1], self.pos[2])
        rotateX(self.rot[0])
        # we don't need to rotate about Y or Z for this example

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

    def draw_goal(self):
        # Draw a translucent sphere at the goal position
        pushMatrix()
        strokeWeight(0)
        fill(255, 0, 0, 100)
        translate(self.local_goal[0], self.local_goal[1], self.local_goal[2])
        sphere(1.25 * shape_scale)
        popMatrix()

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
            fill(154, 205, 50, 127)  # Lime green with 50% transparency
        
        # Draw the node based on the type
        if type == 0:
            draw_cylinder(0.5 * Manipulator.shape_scale, 2 * Manipulator.shape_scale)
        elif type == 1:
            box(1 * Manipulator.shape_scale, 1 * Manipulator.shape_scale, 1.5 * Manipulator.shape_scale)
        elif type == 2:
            strokeWeight(0)
            sphere(Manipulator.end_effector_radius)
            # strokeWeight(0.2)
            # line(0, 0, 0, 2, 0, 0)
    
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
            box(1 * Manipulator.shape_scale, 1 * Manipulator.shape_scale, link_length)

            popMatrix()

    # Kinematics
    def get_end_effector_transform(self):
        # Initialize the transformation matrix as an identity matrix
        transform = PMatrix3D()
        
        # Apply the transformations for each link
        for link in self.links:
            Manipulator.apply_transformations(link, transform)
        
        return transform

    def get_end_effector_facing(self, to_degrees=False):
        # Get the transformation matrix of the end effector
        transform = self.get_end_effector_transform()
        
        # Extract the rotation matrix elements from the transformation matrix
        m00 = transform.m00
        m01 = transform.m01
        m02 = transform.m02
        m10 = transform.m10
        m11 = transform.m11
        m12 = transform.m12
        m20 = transform.m20
        m21 = transform.m21
        m22 = transform.m22
        
        # Compute the Euler angles from the rotation matrix
        angles = [0, 0, 0]
        angles[0] = atan2(m21, m22)  # Rotation around X-axis
        angles[1] = atan2(-m20, sqrt(m21 * m21 + m22 * m22))  # Rotation around Y-axis
        angles[2] = atan2(m10, m00)  # Rotation around Z-axis
        
        # Convert to degrees if requested
        if to_degrees:
            angles = [degrees(angle) for angle in angles]
        
        return angles

    def move_to_goal(self):
        heading = Manipulator.goal_heading(self.local_goal)
        self.links[0]["theta"] = heading
        pos2D = Manipulator.get_flat_pos(heading, self.local_goal)
        theta1, theta2 = self.inverse_kinematics(pos2D)
        if theta1 is not None and theta2 is not None:
            self.links[1]["theta"] = theta1
            self.links[2]["theta"] = theta2
        pass

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
            # println("Target unreachable")
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
    
    @staticmethod
    def apply_transformations(link, transform=None):
        theta = link["theta"]
        d = link["d"]
        r = link["r"]
        alpha = link["alpha"]
        
        if transform is None:
            # Apply transformations to the current matrix stack
            rotateZ(theta)
            translate(0, 0, d)
            translate(r, 0, 0)
            rotateX(alpha)
        else:
            # Apply transformations to the provided PMatrix3D object
            transform.rotateZ(theta)
            transform.translate(0, 0, d)
            transform.translate(r, 0, 0)
            transform.rotateX(alpha)
    
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
    
    @staticmethod
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

class Box:
    def __init__(self):
        # Box Dynamics in global FoR
        self.box_pos = [15, 0, 0]
        self.box_rot = [0, 0, 0]  # [rotation about X, Y, Z]
        self.box_dims = [8, 16, 4]
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
            [0, 0, -self.box_dims[2]/2, 0, 0, 0]
        ]

    # Dynamics
    def update_box_dynamics(self):
        self.update_net_force()
        self.update_acceleration()
        self.update_velocity()
        self.update_position()

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

    # Drawing
    def draw():
        draw_box()
        draw_goal()

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
        strokeWeight(0.1)
        stroke(0)
        fill(139, 69, 19)
        box(self.box_dims[0], self.box_dims[1], self.box_dims[2])
        popMatrix()
    
    def draw_goal(self):
        # Draw the goal box
        pushMatrix()
        translate(self.goal_pos[0], self.goal_pos[1], self.goal_pos[2])
        rotateZ(self.goal_rot[2])
        rotateY(self.goal_rot[1])
        rotateX(self.goal_rot[0])
        draw_reference_frame()
        fill(0, 255, 0, 127)  # Green color with 50% transparency
        # box(self.goal_dims[0], self.goal_dims[1], self.goal_dims[2])
        popMatrix()

    @staticmethod
    def draw_force(force, force_scalar = 0.25):
        pushMatrix()
        translate(force[0], force[1], force[2])
        fill(255, 0, 0)  # Red color
        sphere(0.25)    

        # Draw the force vector
        strokeWeight(0.2)
        stroke(255, 140, 0)  # Dark orange color
        line(0, 0, 0, -force[3] * force_scalar, -force[4] * force_scalar, -force[5] * force_scalar)

        popMatrix()

    # Control
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

    # Helpers
    def get_box_transform(self):
        # Create a PMatrix3D representing the box's global transformation.
        transform = PMatrix3D()
        transform.translate(self.box_pos[0], self.box_pos[1], self.box_pos[2])
        # Note: The order of rotations here matches how the box is drawn.
        transform.rotateZ(self.box_rot[2])
        transform.rotateY(self.box_rot[1])
        transform.rotateX(self.box_rot[0])
        return transform

    def get_box_facing(self, to_degrees=False):
        # Get the transformation matrix of the box.
        transform = self.get_box_transform()

        # Extract the rotation matrix elements from the PMatrix3D.
        m00 = transform.m00
        m01 = transform.m01
        m02 = transform.m02
        m10 = transform.m10
        m11 = transform.m11
        m12 = transform.m12
        m20 = transform.m20
        m21 = transform.m21
        m22 = transform.m22

        # Compute Euler angles.
        # Here, we assume a rotation order similar to that used in the draw_box() method.
        angles = [0, 0, 0]
        angles[0] = atan2(m21, m22)                   # Rotation about X-axis (roll)
        angles[1] = atan2(-m20, sqrt(m21 * m21 + m22 * m22))  # Rotation about Y-axis (pitch)
        angles[2] = atan2(m10, m00)                    # Rotation about Z-axis (yaw)

        # Convert to degrees if requested.
        if to_degrees:
            angles = [degrees(angle) for angle in angles]
        return angles

    def local_to_global_box(self, local_offset):
        # unpack local offset values (assume center of box as origin)
        lx, ly, lz = local_offset[0], local_offset[1], local_offset[2]
        rx, ry, rz = self.box_rot[0], self.box_rot[1], self.box_rot[2]
        
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
        global_x = self.box_pos[0] + x3
        global_y = self.box_pos[1] + y3
        global_z = self.box_pos[2] + z3
        return [global_x, global_y, global_z]

    @staticmethod    
    def compute_box_moment(mass, dims):
        # dims: [width, height, depth]
        w, h, d = dims[0], dims[1], dims[2]
        I_x = (1/12.0) * mass * (h*h + d*d)
        I_y = (1/12.0) * mass * (w*w + d*d)
        I_z = (1/12.0) * mass * (w*w + h*h)
        return [I_x, I_y, I_z]

# Setup parameters
last_time = 0
axis_length = 10
zoom = 20.0
angleX = 0
angleY = 0

# Box
box_obj = Box()
# For the top surface:
contact_points = [
    [0, 5, box_obj.box_dims[2]/2 + Manipulator.end_effector_radius],
    [0, -5, box_obj.box_dims[2]/2 + Manipulator.end_effector_radius],
    [0, 0, -box_obj.box_dims[2]/2 - Manipulator.end_effector_radius],
]

arms = [
    Manipulator(pos = [0, 10, 0], rot = [0, 0, 0]),
    Manipulator(pos = [0, -10, 0], rot = [0, 0, 0]),
    Manipulator(pos = [0, 0, -10], rot = [PI, 0, 0]),
]

# Main functions
def set_up_drawing():
    global last_time, dt

    # Update time step
    current_time = millis()
    dt = (current_time - last_time) / 1000.0 if last_time != 0 else 0
    last_time = current_time

    # Set up the drawing environment
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

    # Draw the reference frame
    strokeWeight(0.1)
    draw_reference_frame()

def setup():
    global zoom
    
    # Option 1
    # fullScreen(P3D)
    # zoom = 20.0

    # Option 2
    size(800, 800, P3D)
    zoom = 10.0

start_angle = None
def draw():
    global start_angle
    set_up_drawing()

    control()  # Handle user input (goal position and rotation)
    box_obj.draw_box() # Draw the box

    end_effector = arms[-1].get_end_effector_facing(True)  # Returns angles in degrees
    box_facing = box_obj.get_box_facing(True)              # Returns angles in degrees
    difference = get_facing_difference(end_effector, box_facing)
    if start_angle is None:
        start_angle = difference
    else:
        contact_points[2][0] -= 0.001 * (start_angle[1] - difference[1])

    # Set up arms
    for i, arm in enumerate(arms):
        contact_point = contact_points[i][:]
        if i == 2:
            contact_point[0] -= 0.01 * (start_angle[1] - difference[1])
        global_goal = box_obj.local_to_global_box(contact_point)
        pushMatrix()
        arm.global_goal = global_goal
        arm.move_to_goal()
        
        arm.draw()
        # arm.draw_goal()
        popMatrix()

    # Example usage:
    print("Facing difference (degrees):", [round(angle, 2) for angle in difference])
    
def get_facing_difference(effector_angles, box_angles):
    diff = []
    for eff_angle, box_angle in zip(effector_angles, box_angles):
        delta = eff_angle - box_angle
        # Normalize delta to the range [-180, 180]
        while delta > 180:
            delta -= 360
        while delta < -180:
            delta += 360
        diff.append(delta)
    return diff

# Control
def control():
    control_box()

def control_box():
    box_speed = 10
    box_rot_speed = PI/2

    generic_control(box_speed*dt, box_rot_speed*dt, box_obj.box_pos, box_obj.box_rot)

def control_force():
    force_mag = 10
    torque_mag = 10

    desired_net_force = [0, 0, 0]
    desired_net_torque = [0, 0, 0]
    
    generic_control(force_mag, torque_mag, desired_net_force, desired_net_torque)

    box_obj.set_necessary_forces(desired_net_force, desired_net_torque)
    box_obj.update_box_dynamics()  # Update force, accel, vel and pos, both linear and angular

def control_goal():
    goal_vel = 10
    goal_avel = PI/2
    
    generic_control(goal_vel*dt, goal_avel*dt, box_obj.goal_pos, box_obj.goal_rot)
    
    box_obj.thrust_to_goal()  # Apply PID control to reach the goal
    box_obj.update_box_dynamics()  # Update force, accel, vel and pos, both linear and angular

def generic_control(lin_mag, rot_mag, lin_vals, rot_vals):
    if keyPressed:
        lin_vals[0] += (1 if key == 's' else -1 if key == 'w' else 0) * lin_mag
        lin_vals[1] += (1 if key == 'a' else -1 if key == 'd' else 0) * lin_mag
        lin_vals[2] += (1 if key == 'z' else -1 if key == 'x' else 0) * lin_mag

        rot_vals[0] += (-1 if key == CODED and keyCode == LEFT else 1 if key == CODED and keyCode == RIGHT else 0) * rot_mag
        rot_vals[1] += (-1 if key == CODED and keyCode == UP else 1 if key == CODED and keyCode == DOWN else 0) * rot_mag
        rot_vals[2] += (-1 if key == ',' else 1 if key == '.' else 0) * rot_mag

def mouseDragged():
    global angleX, angleY
    # Update rotation angles based on the change in mouse position.
    # Adjust the sensitivity by multiplying with a small factor (e.g., 0.01).
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01

# Drawing
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

def draw_reference_frame():
    stroke(255, 0, 0) # x
    line(0,0,0, axis_length,0,0)
    stroke(0, 255, 0) # y
    line(0,0,0, 0,axis_length,0)
    stroke(0, 0, 255) # z
    line(0,0,0, 0,0,axis_length)
    stroke(0)

def draw_reference_frame():
    # Draws the axes to visualize the coordinate system
    stroke(255, 0, 0)  # x-axis (red)
    line(0, 0, 0, axis_length, 0, 0)
    stroke(0, 255, 0)  # y-axis (green)
    line(0, 0, 0, 0, axis_length, 0)
    stroke(0, 0, 255)  # z-axis (blue)
    line(0, 0, 0, 0, 0, axis_length)
    stroke(0)

# Math
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
