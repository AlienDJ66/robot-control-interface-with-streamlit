# simulation.py
import pybullet as p
import pybullet_data

def init_simulation():
    """
    Initialize PyBullet GUI, gravity, plane.
    """
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    p.setRealTimeSimulation(1)

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
