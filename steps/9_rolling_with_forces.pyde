import math

class Manipulator:
    instance_count = 0  # Class variable to track number of instances
    shape_scale = 2

    def __init__(self, pos, rot, end_effector_radius=0):
        self.id = Manipulator.instance_count
        Manipulator.instance_count += 1
        self.end_effector_radius = end_effector_radius
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
        self.draw_node(0)
        
        for i in range(len(self.links)):
            link = self.links[i]
            # Draw the connector
            self.draw_connector(link)
            
            # Draw the node
            Manipulator.apply_transformations(link)
            self.draw_node(self.links[i+1]["type"] if i+1 < len(self.links) else 2)
        popMatrix()

    def draw_goal(self):
        # Draw a translucent sphere at the goal position
        pushMatrix()
        strokeWeight(0)
        fill(255, 0, 0, 100)
        translate(self.local_goal[0], self.local_goal[1], self.local_goal[2])
        sphere(1.25 * shape_scale)
        popMatrix()

    def draw_node(self, type=0, highlight=False, draw_axes=False):
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
            if self.end_effector_radius == 0:
                translate(-1.5, 0, 0)
                rotateY(PI/2)
                draw_cone(0.75 * Manipulator.shape_scale, 1.5 * Manipulator.shape_scale)
            else:
                sphere(self.end_effector_radius)

    def draw_connector(self, link):
        link_length = sqrt(link["r"]**2 + link["d"]**2)

        if link_length > 0:
            pushMatrix()
            
            # Math stuff for orientation:
            rotateZ(link["theta"])
            translate(link["r"] / 2, 0, link["d"] / 2)
            angle_between = acos(link["d"] / link_length)
            axis_y = 1 if link["r"] >= 0 else -1
            rotate(angle_between, 0, axis_y, 0)

            # If this is the far end connector, remove 1.5 only from the far end.
            if self.end_effector_radius == 0 and self.links.index(link) == len(self.links) - 1:
                lr = 1.5 + 1.5
                # Shift the box in the drawing direction by half the amount to trim from the far end.
                translate(0, 0, -lr/2)
                # Reduce the length by 1.5.
                current_length = link_length - lr
            else:
                current_length = link_length
            
            # Draw the connector as a box (centered by default).
            box(1 * Manipulator.shape_scale, 1 * Manipulator.shape_scale, current_length)

            popMatrix()

    # Kinematics
    def get_end_effector_global_position(self):
        transform = self.get_end_effector_transform()
        pos = [transform.m03 + self.pos[0], transform.m13 + self.pos[1], transform.m23 + self.pos[2]]
        return pos

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
        ik_result = self.inverse_kinematics(pos2D)
        if ik_result != False:
            theta1, theta2 = ik_result
            self.links[1]["theta"] = theta1
            self.links[2]["theta"] = theta2

    def is_reachable(self, global_goal):
        local_goal = Manipulator.global_to_local(global_goal, self.pos, self.rot[0])
        heading = Manipulator.goal_heading(local_goal)
        pos2D = Manipulator.get_flat_pos(heading, local_goal)
        return self.inverse_kinematics(pos2D) != False

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
            # print("Target unreachable", self.id)
            return False
        else:
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
    def __init__(self, box_pos=[10, 0, 0], box_rot=[0, 0, 0]):
        # Box Dynamics in global FoR
        self.box_pos = box_pos
        self.box_rot = box_rot
        self.box_dims = [8, 16, 4]
        self.box_mass = 1
        self.box_moment = Box.compute_box_moment(self.box_mass, self.box_dims)
        self.box_vel = [0, 0, 0]
        self.box_accel = [0, 0, 0]  # The box will accelerate along the global Y axis
        self.box_force = [0, 0, 1]
        self.box_avel = [0, 0, 0]
        self.box_aaccel = [0, 0, 0]
        self.box_torque = [0, 0, 0]
        self.lin_damping = 0
        self.ang_damping = 0

        # Goal box
        self.goal_pos = self.box_pos[:]
        self.goal_rot = [0, 0, 0]
        self.goal_dims = [dim * 1.1 for dim in self.box_dims]
        self.goal_vel = 20
        self.goal_avel = PI/2

        # PID control for positions
        self.Kpp = 40
        self.Kip = 1
        self.Kdp = 10
        self.Kpr = 40
        self.Kir = 1
        self.Kdr = 10
        self.prev_measurement = self.box_pos[:]  # for translation
        self.prev_measurement_rot = self.box_rot[:]  # for rotation
        self.error = [0, 0, 0, 0, 0, 0]
        self.prev_error = [0, 0, 0, 0, 0, 0]
        self.error_sum = [0, 0, 0, 0, 0, 0]
        
        self.forces_enabled = True
        # Box force positions
        x_off = 0
        y_off = 0
        self.forces = [
            [0+x_off, 5+y_off, self.box_dims[2]/2, 0, 0, 0],
            [0+x_off, -5+y_off, self.box_dims[2]/2, 0, 0, 0],
            [0-x_off, 0-y_off, -self.box_dims[2]/2, 0, 0, 0]
        ]

    @property
    def wrapped_rot(self):
        return [ ((angle + PI) % (2 * PI)) - PI for angle in self.box_rot ]

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
            # Apply linear damping (e.g., exponential damping)
            self.box_vel[i] *= (1 - self.lin_damping * dt)
        # Update rotational velocity: ω = ω + α * dt
        for i in range(3):
            self.box_avel[i] += self.box_aaccel[i] * dt
            # Apply angular damping
            self.box_avel[i] *= (1 - self.ang_damping * dt)

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

    def draw_box(self, include_forces=True):
        # Draw the box with its own transformations
        pushMatrix()
        translate(self.box_pos[0], self.box_pos[1], self.box_pos[2])
        rotateZ(self.box_rot[2])
        rotateY(self.box_rot[1])
        rotateX(self.box_rot[0])
        draw_reference_frame()
        # Draw the forces
        if include_forces:
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
        noFill()
        box(self.goal_dims[0], self.goal_dims[1], self.goal_dims[2])
        popMatrix()

    def draw_force(self, force, force_scalar = 0.25):
        if not self.forces_enabled:
            return

        pushMatrix()
        translate(force[0], force[1], force[2])
        stroke(255, 165, 0)  # Brighter orange color
        sphere(0.25)

        # Draw the force vector
        strokeWeight(0.2)
        stroke(255, 140, 0)  # Dark orange color
        line(0, 0, 0, -force[3] * force_scalar, -force[4] * force_scalar, -force[5] * force_scalar)

        popMatrix()

    # Control
    def set_necessary_forces(self, desired_net_force, desired_net_torque):
        self.stop_thrust()
        if not self.forces_enabled:
            return

        # Convert desired force and torque from global frame to local frame:
        R = rotation_matrix(self.box_rot[0], self.box_rot[1], self.box_rot[2])
        R_T = transpose(R)  # R is orthonormal so its transpose is its inverse.
        desired_net_force_local = mat_vec_mult(R_T, desired_net_force)
        desired_net_torque_local = mat_vec_mult(R_T, desired_net_torque)

        # --- Translational Forces Allocation ---
        self.forces[0][4] += 0.25 * desired_net_force_local[1]
        self.forces[1][4] += 0.25 * desired_net_force_local[1]
        self.forces[2][4] += 0.5  * desired_net_force_local[1]

        self.forces[0][3] += 0.25 * desired_net_force_local[0]
        self.forces[1][3] += 0.25 * desired_net_force_local[0]
        self.forces[2][3] += 0.5  * desired_net_force_local[0]

        if desired_net_force_local[2] > 0:
            self.forces[2][5] += desired_net_force_local[2]
        else:
            self.forces[0][5] += 0.5 * desired_net_force_local[2]
            self.forces[1][5] += 0.5 * desired_net_force_local[2]

        # --- Helper Functions for Torque Allocation ---
        def allocate_yaw_torque(torque_z):
            r0 = abs(self.forces[0][1])
            r1 = abs(self.forces[1][1])
            if (r0 + r1) != 0:
                F_torque0 = -torque_z / (r0 + r1)
                F_torque1 =  torque_z / (r0 + r1)
            else:
                F_torque0 = 0
                F_torque1 = 0
            self.forces[0][3] += F_torque0
            self.forces[1][3] += F_torque1

        def allocate_pitch_torque(torque_y):
            z_top = self.forces[0][2]
            if z_top != 0:
                A = torque_y / 4.0
                F_torque_bottom = -A
            else:
                A = 0
                F_torque_bottom = 0

            y0 = self.forces[0][1]
            y1 = self.forces[1][1]
            y2 = self.forces[2][1]
            d0 = abs(y0 - y2) or 1e-6
            d1 = abs(y1 - y2) or 1e-6
            w0 = 1.0 / d0
            w1 = 1.0 / d1
            F_torque_top_0 = A * (w0 / (w0 + w1))
            F_torque_top_1 = A * (w1 / (w0 + w1))
            self.forces[0][3] += F_torque_top_0
            self.forces[1][3] += F_torque_top_1
            self.forces[2][3] += F_torque_bottom

        def allocate_roll_torque(torque_x):
            z_top = self.forces[0][2]
            if z_top != 0:
                F_torque_top_x = -torque_x / (4.0 * z_top)
                F_torque_bot_x = -2.0 * F_torque_top_x
            else:
                F_torque_top_x = 0
                F_torque_bot_x = 0
            self.forces[0][4] += F_torque_top_x
            self.forces[1][4] += F_torque_top_x
            self.forces[2][4] += F_torque_bot_x

            # Correct unwanted yaw from roll.
            x_off_val = self.forces[0][0]
            unwanted_yaw = 4.0 * x_off_val * F_torque_top_x
            desired_yaw_corr = -unwanted_yaw
            allocate_yaw_torque(desired_yaw_corr)

        def correct_yaw_from_translation():
            net_yaw = 0.0
            for thruster in self.forces:
                # thruster[0]: X position, thruster[1]: Y position
                # thruster[3]: F_x, thruster[4]: F_y
                net_yaw += thruster[0] * thruster[4] - thruster[1] * thruster[3]
            # Apply a corrective yaw torque equal to -net_yaw.
            allocate_yaw_torque(-net_yaw)

        def correct_pitch_roll_from_translation_z():
            net_pitch = 0.0  # Unwanted pitch torque (rotation about Y)
            net_roll = 0.0   # Unwanted roll torque (rotation about X)
            for thruster in self.forces:
                # thruster[0]: x position, thruster[1]: y position, thruster[5]: F_z
                net_pitch += -thruster[0] * thruster[5]
                net_roll  += thruster[1] * thruster[5]
            # Apply corrective torques to cancel these unwanted moments.
            # (We call the existing helper functions with the negative of the unwanted torque.)
            allocate_pitch_torque(-net_pitch)
            allocate_roll_torque(-net_roll)

        # --- Call the helper functions ---
        allocate_roll_torque(desired_net_torque_local[0])
        allocate_pitch_torque(desired_net_torque_local[1])
        correct_pitch_roll_from_translation_z()
        correct_yaw_from_translation()
        allocate_yaw_torque(desired_net_torque_local[2])

        # Apply grasping forces
        self.apply_grasping_forces()

    def apply_grasping_forces(self):
        # Extract positions:
        p0 = self.forces[0][0:3]
        p1 = self.forces[1][0:3]
        p2 = self.forces[2][0:3]
        
        # Compute midpoint m between p0 and p1:
        m = [ (p0[i] + p1[i]) / 2.0 for i in range(3) ]
        
        # Compute vector d from m to p2 (this defines the plane for grasping):
        d = [ p2[i] - m[i] for i in range(3) ]
        
        # Compute norm of d:
        norm_d = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        if norm_d == 0:
            # Fallback: if the points are degenerate, default to vertical.
            d_unit = [0, 0, 1]
        else:
            d_unit = [ d[i] / norm_d for i in range(3) ]
        
        # Apply grasping forces along the unit vector d_unit:
        # For the top thrusters, add a force of -25 in the d_unit direction.
        # For the bottom thruster, add a force of +50 in the d_unit direction.
        grasp_top = 25.0
        grasp_bot = -50.0

        for thruster_index in [0, 1]:
            self.forces[thruster_index][3] += grasp_top * d_unit[0]
            self.forces[thruster_index][4] += grasp_top * d_unit[1]
            self.forces[thruster_index][5] += grasp_top * d_unit[2]

        self.forces[2][3] += grasp_bot * d_unit[0]
        self.forces[2][4] += grasp_bot * d_unit[1]
        self.forces[2][5] += grasp_bot * d_unit[2]

    def PID(self):
        if not self.forces_enabled:
            # reset PID variables
            self.error_sum = [0, 0, 0, 0, 0, 0]
            self.prev_measurement = self.box_pos[:]
            self.prev_measurement_rot = self.box_rot[:]

        pid_force = [0, 0, 0]
        pid_torque = [0, 0, 0]
        
        # Translation PID: use difference in measurement (box_pos) for the derivative
        for i in range(3):
            error = self.goal_pos[i] - self.box_pos[i]
            self.error_sum[i] += error * dt
            # Compute derivative from measurement change
            measurement_derivative = -(self.box_pos[i] - self.prev_measurement[i]) / dt if dt != 0 else 0
            output = self.Kpp * error + self.Kip * self.error_sum[i] + self.Kdp * measurement_derivative
            pid_force[i] = output
            self.prev_measurement[i] = self.box_pos[i]
        
        # Rotation PID: use difference in measurement (box_rot) for the derivative
        for i in range(3):
            error = self.goal_rot[i] - self.wrapped_rot[i]
            self.error_sum[i+3] += error * dt
            measurement_derivative = -(self.wrapped_rot[i] - self.prev_measurement_rot[i]) / dt if dt != 0 else 0
            output = (self.Kpr * error + self.Kir * self.error_sum[i+3] + self.Kdr * measurement_derivative) * self.box_moment[i]
            pid_torque[i] = output
            self.prev_measurement_rot[i] = self.wrapped_rot[i]
        
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
    
    def get_box_transform(self):
        # Create a PMatrix3D representing the box's global transformation.
        transform = PMatrix3D()
        transform.translate(self.box_pos[0], self.box_pos[1], self.box_pos[2])
        # Note: The order of rotations here matches how the box is drawn.
        transform.rotateZ(self.box_rot[2])
        transform.rotateY(self.box_rot[1])
        transform.rotateX(self.box_rot[0])
        return transform
    
    @staticmethod
    def sqrt(n, tolerance=1e-10):
        if n < 0:
            raise ValueError("No se puede calcular la raíz cuadrada de un número negativo.")
        if n == 0:
            return 0.0
        guess = n
        while True:
            new_guess = (guess + n / guess) / 2.0
            if abs(new_guess - guess) < tolerance:
                return new_guess
            guess = new_guess
            
    @staticmethod
    def distance_3d(object_center, finger_position):
        x1, y1, z1 = object_center
        x2, y2, z2 = finger_position
        
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        
        square_distance = dx * dx + dy * dy + dz * dz
        return Box.sqrt(square_distance)

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

# Arms
contact = 1
end_effector_radius = 2
arms = [
    Manipulator([0, 4.5, 10], [0, 0, 0], end_effector_radius),
    Manipulator([0, -4.5, 10], [0, 0, 0], end_effector_radius),
    Manipulator([0, 0, -10], [PI, 0, 0], end_effector_radius),
]
arms[0].links[0]["theta"] = PI/9
arms[1].links[0]["theta"] = -PI/9
ghost_arms = [Manipulator(m.pos, m.rot, end_effector_radius) for m in arms]
start_angle = [None for _ in ghost_arms]
start_pos = [None for _ in ghost_arms]
box_is_reachable = True

# Box
backup_box = None
box_limits_pos = [[4, 12], [-6, 6], [-6, 6]]
box_limits_rot = [[-PI/4, PI/4], [-PI/4, PI/4], [-PI/4, PI/4]]
box_obj = Box([10, 0, 0], [0, 0, 0])
contact_points = [
    [0, 5, box_obj.box_dims[2]/2 + end_effector_radius],
    [0, -5, box_obj.box_dims[2]/2 + end_effector_radius],
    [0, 0, -box_obj.box_dims[2]/2 - end_effector_radius],
]

# Main functions
def record_starting_conditions():
    global backup_box
    # Record the start position of the box
    backup_box = Box(box_obj.box_pos[:], box_obj.box_rot[:])

    # Record the end effector positions of the arms
    for i, arm in enumerate(arms):
        start_pos[i] = arm.get_end_effector_global_position()

    # Record the starting angles of the ghost arms
    for i, ghost_arm in enumerate(ghost_arms):
        ghost_arm.global_goal = box_obj.local_to_global_box(contact_points[i])
        ghost_arm.move_to_goal()
        angles = get_line_angles_relative_to_box_axes(ghost_arm, box_obj)
        start_angle[i] = angles

def set_up_drawing():
    global last_time, dt

    # Update time step
    current_time = millis()
    dt = (current_time - last_time) / 1000.0 if last_time != 0 else 0
    last_time = current_time
    dt *= 1

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
    global zoom, start_angle

    monoFont = createFont("Courier New", 16)
    textFont(monoFont)

    # Option 1
    fullScreen(P3D)
    zoom = 18

    # Option 2
    # size(800, 800, P3D)
    # zoom = 12.5

    record_starting_conditions()

def draw():
    global start_angle
    pushMatrix()
    set_up_drawing()

    # Handle user input (goal position and rotation)
    control()

    # Move arms
    move_arms()
    
    # Draw the box
    box_obj.draw_box(True)
    box_obj.draw_goal()

    # Draw the arms
    for i, arm in enumerate(arms):
        arm.draw()
    popMatrix()

    display_hud()


# Control
def move_arms():
    global box_is_reachable
    rolled_contact_points = [get_rolled_contact_point(i) for i in range(len(arms))]

    # Determine if all arms can reach the goal.
    box_is_reachable = all([arm.is_reachable(box_obj.local_to_global_box(rolled_contact_points[i])) for i, arm in enumerate(arms)])
    
    # Ensure the box's rotation doesn't exceed 90 on any axis
    box_is_grabbable = all(abs(angle) <= PI/3 for angle in box_obj.wrapped_rot)

    box_is_reachable = box_is_reachable and box_is_grabbable

    if not box_is_reachable:
        actuate_gripper(-1)

    # Move the arms to the contact points
    for i, arm in enumerate(arms):
        contact_point = rolled_contact_points[i]
        box_obj.forces[i][0:2] = [contact_point[0], contact_point[1]]
        contact_point = box_obj.local_to_global_box(contact_point)
        extrapolated_goal = extrapolate_smooth(start_pos[i], contact_point, contact)
        arm.global_goal = extrapolated_goal
        arm.move_to_goal()

def control():
    control_goal()
    # control_box()
    # control_force()
    # box_obj.update_box_dynamics()
    pass

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

    # Show desired vs obtained torque
    for t in range(3):
        if abs(desired_net_torque[t]) > 1e-5 or abs(box_obj.box_torque[t]) > 1e-5:
            print("Desired torque:", round(desired_net_torque[t], 2), "Obtained torque:", round(box_obj.box_torque[t], 2))
    
    # Show desired vs obtained force
    for f in range(3):
        if abs(desired_net_force[f]) > 1e-5 or abs(box_obj.box_force[f]) > 1e-5:
            print("Desired force:", round(desired_net_force[f], 2), "Obtained force:", round(box_obj.box_force[f], 2))
    print()

    box_obj.set_necessary_forces(desired_net_force, desired_net_torque)
    box_obj.update_box_dynamics()  # Update force, accel, vel and pos, both linear and angular

def control_goal():
    goal_vel = 10
    goal_avel = PI/2
    
    generic_control(goal_vel*dt, goal_avel*dt, box_obj.goal_pos, box_obj.goal_rot)
    print("Goal position:", [round(coord, 1) for coord in box_obj.goal_pos], "Goal rotation:", [round(angle, 1) for angle in box_obj.goal_rot])

        # Limit the goal position and rotation:
    for i in range(3):
        box_obj.goal_pos[i] = max(min(box_limits_pos[i][1], box_obj.goal_pos[i]), box_limits_pos[i][0])
        box_obj.goal_rot[i] = max(min(box_limits_rot[i][1], box_obj.goal_rot[i]), box_limits_rot[i][0])

    box_obj.thrust_to_goal()  # Apply PID control to reach the goal
    box_obj.update_box_dynamics()  # Update force, accel, vel and pos, both linear and angular

def generic_control(lin_mag, rot_mag, lin_vals, rot_vals):
    global contact
    if keyPressed:
        lin_vals[0] += (1 if key == 's' else -1 if key == 'w' else 0) * lin_mag
        lin_vals[1] += (1 if key == 'a' else -1 if key == 'd' else 0) * lin_mag
        lin_vals[2] += (1 if key == 'z' else -1 if key == 'x' else 0) * lin_mag

        rot_vals[0] += (-1 if key == CODED and keyCode == LEFT else 1 if key == CODED and keyCode == RIGHT else 0) * rot_mag
        rot_vals[1] += (-1 if key == CODED and keyCode == UP else 1 if key == CODED and keyCode == DOWN else 0) * rot_mag
        rot_vals[2] += (-1 if key == ',' else 1 if key == '.' else 0) * rot_mag

        actuate_gripper((1 if key == 'c' else -1 if key == 'v' else 0))

        if key == 'r':
            box_obj.box_pos = backup_box.box_pos[:]
            box_obj.box_rot = backup_box.box_rot[:]
            box_obj.box_vel = [0, 0, 0]
            box_obj.box_avel = [0, 0, 0]

def actuate_gripper(speed):
    global contact
    contact = max(min(contact + speed * dt, 1), 0)
    box_obj.forces_enabled = (contact == 1)

def mouseDragged():
    global angleX, angleY
    # Update rotation angles based on the change in mouse position.
    # Adjust the sensitivity by multiplying with a small factor (e.g., 0.01).
    angleY += (mouseX - pmouseX) * 0.01
    angleX -= (mouseY - pmouseY) * 0.01

# Drawing
def display_hud():
    instructions = [
        "INSTRUCTIONS:",
        "Use the WASDZX keys for translation",
        "Use the ARROW_KEYS + ',' and '.' for rotation",
        "HOLD 'c' or 'v' to open/close the gripper",
        "Press 'r' to reset the box",
        "Drag the mouse to rotate the view",
    ]
    font_size = 24
    margin = 20  # pixels from the borders
    line_spacing = font_size * 1.2  # pixels between lines
    for i, line in enumerate(instructions[::-1]):
        y_pos = height - margin - i * line_spacing
        display_text(line, font_size, margin / float(width), y_pos / float(height), (0, 0, 0), h_align=LEFT, v_align=BOTTOM)

    # Credits
    credits = [
        u"TE3001B.101 - Equipo 4 Matutino",
        u"Samuel Cabrera         | A00838072",
        u"Jose Luis Urquieta     | A00835580",
        u"Felipe de Jesús García | A01705893",
        u"Uriel Ernesto Lemus    | A00835767",
        u"Santiago Lopez         | A01235819",
    ]

    for i, line in enumerate(credits):
        y_pos = margin + i * line_spacing
        display_text(line, font_size, margin / float(width), y_pos / float(height), (0, 0, 0), h_align=LEFT, v_align=TOP)

    # Show whether the box is reachable
    if not box_is_reachable:
        display_text("Box is unreachable.", 32, 0.5, 0.475, (255, 0, 0))
        display_text("Press 'r' to reset.", 32, 0.5, 0.525, (255, 0, 0))

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

def draw_cone(radius, height, sides=24):
    strokeWeight(0)
    angle = TWO_PI / sides
    half_height = height / 2

    # Draw the base circle
    beginShape(TRIANGLE_FAN)
    vertex(0, 0, -half_height)
    for i in range(sides + 1):
        x = cos(i * angle) * radius
        y = sin(i * angle) * radius
        vertex(x, y, -half_height)
    endShape()

    # Draw the side surface
    beginShape(TRIANGLE_FAN)
    vertex(0, 0, half_height)
    for i in range(sides + 1):
        x = cos(i * angle) * radius
        y = sin(i * angle) * radius
        vertex(x, y, -half_height)
    endShape()

def draw_reference_frame():
    # Draws the axes to visualize the coordinate system
    stroke(255, 0, 0)  # x-axis (red)
    line(0, 0, 0, axis_length, 0, 0)
    stroke(0, 255, 0)  # y-axis (green)
    line(0, 0, 0, 0, axis_length, 0)
    stroke(0, 0, 255)  # z-axis (blue)
    line(0, 0, 0, 0, 0, axis_length)
    stroke(0)

def display_text(msg, font_size, x_frac, y_frac, col=(255, 255, 255), h_align=CENTER, v_align=CENTER):
    textSize(font_size)
    textAlign(h_align, v_align)
    
    # Calculate actual pixel positions from fractions
    x = width * x_frac
    y = height * y_frac
    
    # Disable depth testing to ensure the text appears over the 3D graphics
    hint(DISABLE_DEPTH_TEST)
    fill(*col)
    text(msg, x, y)
    hint(ENABLE_DEPTH_TEST)

# Math
def extrapolate_smooth(pointA, pointB, t):
    # Use a cosine-easing function for smooth transitions.
    smooth_t = (1 - cos(t * PI)) / 2.0
    return [ pointA[j] + smooth_t * (pointB[j] - pointA[j]) for j in range(len(pointA)) ]

def extrapolate(pointA, pointB, t):
    # Returns pointA + t * (pointB - pointA)
    return [ pointA[j] + t * (pointB[j] - pointA[j]) for j in range(len(pointA)) ]

def get_rolled_contact_point(arm_index):
    contact_point = contact_points[arm_index][:]
    global_goal = box_obj.local_to_global_box(contact_point)
    ghost_arms[arm_index].global_goal = global_goal
    ghost_arms[arm_index].move_to_goal()
    # ghost_arms[i].draw()

    angleX, angleY = get_line_angles_relative_to_box_axes(ghost_arms[arm_index], box_obj)
    # print(degrees(angleX), degrees(angleY))

    contact_point[0] -= (-1 if arms[arm_index].pos[2] < 0 else 1) * (start_angle[arm_index][0] - angleX) * arms[arm_index].end_effector_radius
    contact_point[1] -= (-1 if arms[arm_index].pos[2] < 0 else 1) * (start_angle[arm_index][1] - angleY) * arms[arm_index].end_effector_radius

    return contact_point

def get_line_angles_relative_to_box_axes(arm, box):
    # Get the end effector’s global line direction (its local X axis) using the provided arm.
    eff_transform = arm.get_end_effector_transform()
    line_global = [eff_transform.m00, eff_transform.m10, eff_transform.m20]
    
    # Helper: Normalize a vector.
    def normalize(v):
        mag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        return [v[0]/mag, v[1]/mag, v[2]/mag] if mag != 0 else v
    line_norm = normalize(line_global)
    
    # Get the box’s transform and extract its XY plane’s axes using the provided box.
    box_transform = box.get_box_transform()
    # Plane's local X axis is given by the first column.
    plane_x = normalize([box_transform.m00, box_transform.m10, box_transform.m20])
    # Plane's local Y axis is given by the second column.
    plane_y = normalize([box_transform.m01, box_transform.m11, box_transform.m21])
    
    # Compute dot products to obtain cosine of the angles between the line and each axis.
    dot_x = line_norm[0]*plane_x[0] + line_norm[1]*plane_x[1] + line_norm[2]*plane_x[2]
    dot_y = line_norm[0]*plane_y[0] + line_norm[1]*plane_y[1] + line_norm[2]*plane_y[2]
    dot_x = max(-1, min(1, dot_x))
    dot_y = max(-1, min(1, dot_y))
    
    # Compute the angles (in radians) between the projected line and each axis.
    angle_rel_x = acos(dot_x)
    angle_rel_y = acos(dot_y)
    
    return angle_rel_x, angle_rel_y

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
