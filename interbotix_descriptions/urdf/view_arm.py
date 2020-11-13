import pybullet as p 
import pybullet_data

direct = p.connect(p.GUI)  #, options="--window_backend=2 --render_device=0")
#egl = p.loadPlugin("eglRendererPlugin")
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0,0,0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')

arm = p.loadURDF('rx200.urdf')

while 1:
	pass