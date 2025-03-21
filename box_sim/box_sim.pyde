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
        self._goal = [0, 0, 0] # in global FoR

    @property
    def local_goal(self):
        return Manipulator.global_to_local(self._goal, self.pos, self.rot[0])

    def draw_goal(self):
        # Draw a translucent sphere at the goal position
        pushMatrix()
        strokeWeight(0)
        fill(255, 0, 0, 100)
        translate(self.local_goal[0], self.local_goal[1], self.local_goal[2])
        sphere(1.25 * shape_scale)
        popMatrix()

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

    def move_to_goal(self):
        heading = Manipulator.goal_heading(self.local_goal)
        self.links[0]["theta"] = heading
        pos2D = Manipulator.get_flat_pos(heading, self.local_goal)
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
            fill(154, 205, 50, 127)  # Lime green with 50% transparency
        
        # Draw the node based on the type
        if type == 0:
            draw_cylinder(0.5 * Manipulator.shape_scale, 2 * Manipulator.shape_scale)
        elif type == 1:
            box(1 * Manipulator.shape_scale, 1 * Manipulator.shape_scale, 1.5 * Manipulator.shape_scale)
        elif type == 2:
            strokeWeight(0)
            sphere(Manipulator.end_effector_radius)
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

class Box:
    def __init__(self):
        # Box Dynamics in global FoR
        self.box_pos = [0, 0, 0]
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
        x_off = 0
        self.forces = [
            [0, 5, self.box_dims[2]/2, 0, 0, 0],
            [0, -5, self.box_dims[2]/2, 0, 0, 0],
            [0, 0, -self.box_dims[2]/2, 0, 0, 0],
        ]
        # self.forces = [
        #     [4 + x_off, 5,      self.box_dims[2] / 2.0,  0, 1.0, -0.4],  # Thruster 0 (Top)
        #     [4 + x_off, -5,     self.box_dims[2] / 2.0,  0, 1.0,  0.4],  # Thruster 1 (Top)
        #     [-2 - x_off, 0,    -self.box_dims[2] / 2.0,  0, 4.0,  0   ]   # Thruster 2 (Bottom)
        # ]
        # self.forces = [
        #     [4 + x_off, 5, self.box_dims[2] / 2.0, 0, 0, 0],  # Thruster 0 (Top)
        #     [4 + x_off, -5, self.box_dims[2] / 2.0, 0, 0, 0],  # Thruster 1 (Top)
        #     [-2 - x_off, 0, -self.box_dims[2] / 2.0, 0, 0, 0]   # Thruster 2 (Bottom)
        # ]
        self.set_gripping_force(0.05)
        # self.set_necessary_forces([0, 1, 0], [0, 0, 0])

    def set_gripping_force(self, mag):
        for i, force in enumerate(self.forces):
            # Compute the average position of the other 2 forces
            avg_pos = [0, 0, 0]
            for j, other_force in enumerate(self.forces):
                if i != j:
                    avg_pos[0] += other_force[0]
                    avg_pos[1] += other_force[1]
                    avg_pos[2] += other_force[2]
            avg_pos[0] /= 2
            avg_pos[1] /= 2
            avg_pos[2] /= 2

            # Compute the force vector
            force[3] += avg_pos[0] * mag
            force[4] += avg_pos[1] * mag
            force[5] += avg_pos[2] * mag
    
    def update_box_dynamics(self):
        self.update_net_force()
        self.update_acceleration()
        self.update_velocity()
        self.update_position()

    def update_net_force(self):
        # Compute net force and torque as defined in the global coordinate frame:
        net_force_local = [0.0, 0.0, 0.0]
        net_torque_local = [0.0, 0.0, 0.0]
        
        for thruster in self.forces:
            # Extract position and force vectors.
            r = thruster[0:3]  # position relative to CoM
            F = thruster[3:6]  # force vector
            
            # Sum forces.
            net_force_local[0] += F[0]
            net_force_local[1] += F[1]
            net_force_local[2] += F[2]
            
            # Compute torque = r x F (cross product).
            torque = [
                r[1] * F[2] - r[2] * F[1],
                r[2] * F[0] - r[0] * F[2],
                r[0] * F[1] - r[1] * F[0]
            ]
            
            # Sum torques.
            net_torque_local[0] += torque[0]
            net_torque_local[1] += torque[1]
            net_torque_local[2] += torque[2]
        
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
    def draw_force(force, force_scalar = 50):
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
    def set_Y_force_allocation(self, desired_net_force_local):
        # desired_net_force_local[1] is the desired net Y force in the local frame.
        F_d = desired_net_force_local[1]
        
        # Extract the X positions for each thruster.
        x0 = self.forces[0][0]
        x1 = self.forces[1][0]
        x2 = self.forces[2][0]
        
        # Choose a predetermined weight for the bottom thruster.
        beta = 0.5  # You can adjust this if desired.
        
        # Total weight for top thrusters must be 1 - beta.
        # We want: x0*W0 + x1*W1 + x2*beta = 0, with W0 + W1 = 1 - beta.
        # Solve for W0:
        if (x0 - x1) != 0:
            W0 = - (x1 * (1 - beta) + x2 * beta) / (x0 - x1)
        else:
            # If x0 equals x1, then they are symmetric in X; split evenly.
            W0 = (1 - beta) / 2.0
        W1 = (1 - beta) - W0
        
        # For debugging you might want to check:
        # print("Weights for Y force:", W0, W1, beta)
        
        # Now assign the Y force for each thruster.
        self.forces[0][4] = W0 * F_d
        self.forces[1][4] = W1 * F_d
        self.forces[2][4] = beta * F_d

    def set_necessary_forces(self, desired_net_force, desired_net_torque):
        # Convert desired force and torque from global frame to local frame:
        R = rotation_matrix(self.box_rot[0], self.box_rot[1], self.box_rot[2])
        R_T = transpose(R)  # since R is orthonormal, its transpose is its inverse.
        desired_net_force_local = mat_vec_mult(R_T, desired_net_force)
        desired_net_torque_local = mat_vec_mult(R_T, desired_net_torque)

        # self.forces[0][3:6] = [0, 0, -25]
        # self.forces[1][3:6] = [0, 0, -25]
        # self.forces[2][3:6] = [0, 0, 50]

        self.forces[0][3:6] = [0, 0, 0]
        self.forces[1][3:6] = [0, 0, 0]
        self.forces[2][3:6] = [0, 0, 0]

        # Determine the average position in X of the top the top thrusters
        avg_top_x = (self.forces[0][0] + self.forces[1][0]) / 2
        bottom_x = self.forces[2][0]

        # For the Y component, we'll apply 0.25 to both of the top thrusters, and 0.5 to the bottom thruster.
        # self.forces[0][4] += 0.25 * desired_net_force_local[1]
        # self.forces[1][4] += 0.25 * desired_net_force_local[1]
        # self.forces[2][4] += 0.5 * desired_net_force_local[1]
        self.set_Y_force_allocation(desired_net_force_local)

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

# Setup parameters
last_time = 0
axis_length = 10
zoom = 20.0
angleX = 0
angleY = 0

# Box
box_obj = Box()

# Main functions
def set_up_drawing():
    global last_time, dt

    # Update time step
    current_time = millis()
    dt = (current_time - last_time) / 1000.0 if last_time != 0 else 0
    last_time = current_time

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
    fullScreen(P3D)

def draw():
    set_up_drawing()

    # control_goal()  # Handle user input (goal position and rotation)
    # control_force()  # Handle user input (force and torque)
    box_obj.update_box_dynamics()  # Update force, accel, vel and pos, both linear and angular
    box_obj.draw_box() # Draw the box

# Control
def control_force():
    force_mag = 10
    torque_mag = 10

    desired_net_force = [0, 0, 0]
    desired_net_torque = [0, 0, 0]
    if keyPressed:
        desired_net_force[0] += (1 if key == 's' else -1 if key == 'w' else 0) * force_mag
        desired_net_force[1] += (1 if key == 'a' else -1 if key == 'd' else 0) * force_mag
        desired_net_force[2] += (1 if key == 'z' else -1 if key == 'x' else 0) * force_mag

        desired_net_torque[0] += (-1 if key == CODED and keyCode == LEFT else 1 if key == CODED and keyCode == RIGHT else 0) * torque_mag
        desired_net_torque[1] += (-1 if key == CODED and keyCode == UP else 1 if key == CODED and keyCode == DOWN else 0) * torque_mag
        desired_net_torque[2] += (-1 if key == ',' else 1 if key == '.' else 0) * torque_mag

    box_obj.set_necessary_forces(desired_net_force, desired_net_torque)

def control_goal():
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
    box_obj.thrust_to_goal()  # Apply PID control to reach the goal

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

def draw_reference_frame():
    stroke(255, 0, 0) # x
    line(0,0,0, axis_length,0,0)
    stroke(0, 255, 0) # y
    line(0,0,0, 0,axis_length,0)
    stroke(0, 0, 255) # z
    line(0,0,0, 0,0,axis_length)
    stroke(0)
