
curr_end  = self.arm.endeffector()
delta_pos = target - curr_end

if delta_pos.r > MAX_STEP:
    delta_pos.r = MAX_STEP


theta1 = self.arm.thetas[0]
theta2 = self.arm.thetas[1]
theta3 = self.arm.thetas[2]
theta4 = self.arm.thetas[3]
theta5 = self.arm.thetas[4]
theta6 = self.arm.thetas[5]

l1 = self.arm.lengths[0]
l2 = self.arm.lengths[1]
l3 = self.arm.lengths[2]
l4 = self.arm.lengths[3]
l5 = self.arm.lengths[4]
l6 = self.arm.lengths[5]


a_0_0 = -l1*np.sin(theta1) - l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1)
a_0_1 = -l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1)
a_1_0 = l1*np.cos(theta1) - l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2)
a_1_1 = -l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2)

J = np.array([[a_0_0, a_0_1], [a_1_0, a_1_1]])

iJ = np.linalg.inv(J)


delta_joints = iJ.dot(np.array([delta_pos.x, delta_pos.y]))
self.arm.move( delta_joints )