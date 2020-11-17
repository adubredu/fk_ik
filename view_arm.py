import pybullet as p 
import pybullet_data
from kinematics import *
from config_parse import *

direct = p.connect(p.GUI)  
delta = 0.01
c1 = 0.23;  c2 = -0.3084;
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')

dh_params = parse_dh_param_file('config/rx200_dh.csv')
m_mat, screw_list = parse_pox_param_file('config/rx200_pox.csv')
arm = p.loadURDF('interbotix_descriptions/urdf/rx200.urdf')
# for i in range(p.getNumJoints(arm)):
# 	print(p.getJointInfo(arm, i))
# 	print(p.getJointState(arm, i))
# 	print('')
# print([x[0] for x in p.getJointStates(arm, [0,1,2,3,4])])

p.setRealTimeSimulation(1)

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

while 1:
	keys = p.getKeyboardEvents()

	if ord('q') in keys:
		print('waist')
		angle = p.getJointState(arm, 0)[0]
		angle -= delta
		control_joint(0, angle, -3.14159265359, 3.14159265359)

	if ord('w') in keys:
		angle = p.getJointState(arm, 0)[0]
		angle += delta
		control_joint(0, angle, -3.14159265359, 3.14159265359)

	if ord('a') in keys:
		angle = p.getJointState(arm, 1)[0]
		angle -= delta
		control_joint(1, angle, -1.86750229963, 1.93731546971)

	if ord('s') in keys:
		angle = p.getJointState(arm, 1)[0]
		angle += delta
		control_joint(1, angle, -1.86750229963, 1.93731546971)

	if ord('z') in keys:
		angle = p.getJointState(arm, 2)[0]
		angle -= delta
		control_joint(2, angle, -1.62315620435, 1.88495559215)

	if ord('x') in keys:
		angle = p.getJointState(arm, 2)[0]
		angle += delta
		control_joint(2, angle, -1.62315620435, 1.88495559215)

	if ord('e') in keys:
		angle = p.getJointState(arm, 3)[0]
		angle -= delta
		control_joint(3, angle, -2.14675497995, 1.74532925199)

	if ord('r') in keys:
		angle = p.getJointState(arm, 3)[0]
		angle += delta
		control_joint(3, angle, -2.14675497995, 1.74532925199)

	if ord('d') in keys:
		angle = p.getJointState(arm, 4)[0]
		angle -= delta
		control_joint(4, angle, -3.14159265359, 3.14159265359)

	if ord('f') in keys:
		angle = p.getJointState(arm, 4)[0]
		angle += delta
		control_joint(4, angle, -3.14159265359, 3.14159265359)
	
	joint_angles = [x[0] for x in p.getJointStates(arm, [0,1,2,3,4])]
	

	# print('GROUND TRUTH EE POSE: ',p.getLinkState(arm, 11)[0], p.getEulerFromQuaternion(p.getLinkState(arm, 11)[1])[1])
	# print('FK DH POSE          : ',FK_dh(dh_params, joint_angles))
	# print('FK POX              : ',FK_pox(joint_angles, m_mat, screw_list))
	# print(' ')

	po = p.getLinkState(arm, 11)[0]
	rot = quaternion_to_rot_matrix(p.getLinkState(arm, 11)[1])
	pose = (po, rot)

	print('GT Joint angles: ', joint_angles)
	print('IK Joint angles: ', IK_geometric(dh_params, pose))
	print(' ')