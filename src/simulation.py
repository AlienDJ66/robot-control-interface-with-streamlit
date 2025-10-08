# simulation.py
import pybullet as p
import pybullet_data

def init_simulation():
    """
    Initialize PyBullet GUI, gravity, plane.
    """

    # Disconnect any existing connection to avoid multiple GUI error since now ui can change robot and restart new simulation
    if p.isConnected():
        p.disconnect()
        
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    p.setRealTimeSimulation(0)
    p.setTimeStep(1/240)

    # Set consistent camera
    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )

def create_target_marker(pos, radius=0.02, color=[1,0,0,1]):
    """
    Create a visual marker for the target position.
    :param pos: [x,y,z] position
    :param radius: sphere radius
    :param color: RGBA color
    :return: marker id
    """
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    marker_id = p.createMultiBody(
        baseMass=0,  # non-physical
        baseVisualShapeIndex=visual_shape_id,
        basePosition=pos
    )
    return marker_id
