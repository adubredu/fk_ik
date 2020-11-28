import pybullet as p 
import pybullet_data
from kinematics import *
from config_parse import *
import time

direct = p.connect(p.GUI)  
delta = 0.001
c1 = 0.23;  c2 = -0.3084;
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')

dh_params = parse_dh_param_file('config/rx200_dh.csv')
m_mat, screw_list = parse_pox_param_file('config/rx200_pox.csv')
arm = p.loadURDF('interbotix_descriptions/urdf/rx200.urdf')
for i in range(p.getNumJoints(arm)):
	# print(p.getJointInfo(arm, i))
	# print(p.getJointState(arm, i))
	# print('')
# print([x[0] for x in p.getJointStates(arm, [0,1,2,3,4])])

# p.setRealTimeSimulation(1)

def control_joint(joint, value, minn, maxx):
	global arm
	if value < minn:
			value = minn
	if value > maxx:
		value = maxx
	p.setJointMotorControl2(arm, joint,
				controlMode=p.POSITION_CONTROL,targetPosition=value,
				force=3000)
	p.stepSimulation()

def rot_to_quat(phi):
	R = get_rot_matrix_from_euler(0,phi,0)
	tr = R[0,0]+R[1,1]+R[2,2]

	if tr > 0.0:
		S = np.sqrt(tr+1.0)*2;
		qw = 0.25*S
		qx = (R[2,1]-R[1,2])/S
		qy = (R[0,2]-R[2,0])/S
		qz = (R[1,0]-R[0,1])/S

	elif ((R[0,0]>R[1,1]) and (R[0,0]>R[2,2])):
		S = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
		qw = (R[2,1]-R[1,2])/S 
		qx = 0.25*S 
		qy = (R[0,1]+R[1,0])/S
		qz = (R[0,2]+R[2,0])/S 

	elif (R[1,1]>R[2,2]):
		S = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2
		qw = (R[0,2]-R[2,0])/S 
		qx = (R[0,1]+R[1,0])/S 
		qy = 0.25*S 
		qz = (R[1,2]+R[2,1])/S 

	else:
		S = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2
		qw = (R[1,0]-R[0,1])/S
		qx = (R[0,2]+R[2,0])/S 
		qy = (R[1,2]+R[2,1])/S
		qz = 0.25*S

	return (qx,qy,qz,qw)


def control_joint(joint, value, minn, maxx):
	global arm
	if value < minn:
			value = minn
	if value > maxx:
		value = maxx
	# p.setJointMotorControl2(arm, joint,
	# 			controlMode=p.POSITION_CONTROL,targetPosition=value,
	# 			force=3000)
	p.resetJointState(arm, joint,value)
	p.stepSimulation()


def teleop():
	while 1:
		keys = p.getKeyboardEvents()
		if ord('i') in keys:
			joint = 0
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('o') in keys:
			joint = 0
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('k') in keys:
			joint = 1
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('l') in keys:
			joint = 1
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord(',') in keys:
			joint = 2
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('.') in keys:
			joint = 2
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)
###
		if ord('z') in keys:
			joint = 3
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('x') in keys:
			joint = 3
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('b') in keys:
			joint = 4
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('n') in keys:
			joint = 4
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		# if ord('d') in keys:
		# 	joint = 10
		# 	angle = p.getJointState(arm, joint)[0]
		# 	angle -= delta
		# 	control_joint(joint, angle, -3.1415, 3.1415)

		# if ord('f') in keys:
		# 	joint = 10
		# 	angle = p.getJointState(arm, joint)[0]
		# 	angle += delta
		# 	control_joint(joint, angle, -3.1415, 3.1415)

		joint_angles = [x[0] for x in p.getJointStates(arm, [0,1,2,3,4])]
		

		# print('GROUND TRUTH EE POSE: ',p.getLinkState(arm, 11)[0], p.getEulerFromQuaternion(p.getLinkState(arm, 11)[1])[1])
		# print('FK DH POSE          : ',FK_dh(dh_params, joint_angles))
		# print('FK POX              : ',FK_pox(joint_angles, m_mat, screw_list))
		# print(' ')

		po = p.getLinkState(arm, 3)[0]
		# rot = quaternion_to_rot_matrix(p.getLinkState(arm, 4)[1])
		# pose = (po, rot)

		print('Pose xyz         : ', po)
		# print('end orientation: ',p.getEulerFromQuaternion(p.getLinkState(arm,11)[1]))
		print('GT Joint angles  : ', joint_angles)
		print('Spatial eu joints: ', spatial_joints_elbow_up(po))
		# print('Spatial ed joints: ', spatial_joints_elbow_down(po))
		# print('IK Joint angles: ', IK_geometric(dh_params, pose))
		print(' ')
		# spatial_joints_elbow_down(po)

def go_to_pose(pose):
	# jsx = full_ik(pose, np.pi/2.0)
	# orr2 = rot_to_quat(np.pi/2)
	# orr = p.getQuaternionFromEuler((0,np.pi/2,0))
	# jsx = p.calculateInverseKinematics(arm, 3, pose)#, orr2)
	# print('joints: ',js)
	jsx = list(spatial_joints_elbow_up(pose)); jsx[1]+=1.375; jsx[2]-=1.4
	
	# print('orr1: ',orr)
	# print('orr2: ',orr2)
	# print('orr2 euler: ',p.getEulerFromQuaternion(orr2))
	print('myik: ',jsx)
	control_joint(0, jsx[0], -3.1415, 3.1415)
	time.sleep(2)
	control_joint(1, jsx[1], -3.1415, 3.1415)
	time.sleep(2)
	control_joint(2, -jsx[2], -3.1415, 3.1415)
	time.sleep(2)
	# control_joint(3, jsx[3], -3.1415, 3.1415)
	# time.sleep(2)
	# control_joint(4, jsx[4], -3.1415, 3.1415)
	# time.sleep(2)
	# control_joint(10, jsx[5], -3.1415, 3.1415)
	# time.sleep(2)


if __name__=='__main__':
	# pass
	# teleop()
	# go_to_pose([0.24, 0, 0.3])
	# go_to_pose([0.083,0.184,0.187])
	go_to_pose([0.224,0.011,0.162])
	# go_to_pose([0.1533,-0.183,0.132])
	po = p.getLinkState(arm, 3)[0]
	print('end pose: ', po)
	print('end orientation: ',p.getEulerFromQuaternion(p.getLinkState(arm,11)[1]))
	# # print('end angs: ',[x[0] for x in p.getJointStates(arm, [0,2,4])])
	while True:
		pass