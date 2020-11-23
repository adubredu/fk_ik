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

# p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath('models')
p.loadURDF('floor/floor.urdf')
arm = p.loadURDF('dof.urdf')

# for i in range(p.getNumJoints(arm)):
# 	print(p.getJointInfo(arm, i))
# 	print('')
# print([x[0] for x in p.getJointStates(arm, [0,1,2,3,4])])

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

		if ord('z') in keys:
			joint = 0
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('x') in keys:
			joint = 0
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('b') in keys:
			joint = 2
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('n') in keys:
			joint = 2
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('d') in keys:
			joint = 4
			angle = p.getJointState(arm, joint)[0]
			angle -= delta
			control_joint(joint, angle, -3.1415, 3.1415)

		if ord('f') in keys:
			joint = 4
			angle = p.getJointState(arm, joint)[0]
			angle += delta
			control_joint(joint, angle, -3.1415, 3.1415)

		joint_angles = [x[0] for x in p.getJointStates(arm, [0,2,4])]
		

		# print('GROUND TRUTH EE POSE: ',p.getLinkState(arm, 11)[0], p.getEulerFromQuaternion(p.getLinkState(arm, 11)[1])[1])
		# print('FK DH POSE          : ',FK_dh(dh_params, joint_angles))
		# print('FK POX              : ',FK_pox(joint_angles, m_mat, screw_list))
		# print(' ')

		po = p.getLinkState(arm, 5)[0]
		# rot = quaternion_to_rot_matrix(p.getLinkState(arm, 4)[1])
		# pose = (po, rot)

		print('Pose xyz         : ', po)
		print('GT Joint angles  : ', joint_angles)
		# print('Spatial eu joints: ', spatial_joints_elbow_up(po))
		print('Spatial ed joints: ', spatial_joints_elbow_down(po))
		# print('IK Joint angles: ', IK_geometric(dh_params, pose))
		print(' ')
		# spatial_joints_elbow_down(po)

def go_to_pose(pose):
	jsx = spatial_joints_elbow_up(pose)
	# jsx = p.calculateInverseKinematics(arm, 5, pose)
	# print('joints: ',js)
	print('myik: ',jsx)
	control_joint(0, jsx[0], -3.1415, 3.1415)
	time.sleep(2)
	control_joint(2, jsx[1], -3.1415, 3.1415)
	time.sleep(2)
	control_joint(4, jsx[2], -3.1415, 3.1415)
	time.sleep(2)


if __name__=='__main__':
	# teleop()
	go_to_pose([-0.2,-0.15,0.83])
	# print('real ik: ', p.calculateInverseKinematics(arm, 5, [0.454095802429504, -0.0009409932501213479, 1.222695483034379]))
	po = p.getLinkState(arm, 5)[0]
	print('end pose: ', po)
	print('end angs: ',[x[0] for x in p.getJointStates(arm, [0,2,4])])
	while True:
		pass
# GT Joint angles  :  [-0.7091198761917801, 0.279587339552825, 1.6137901931029086]