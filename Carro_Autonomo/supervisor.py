from controller import Supervisor
import sys

TIME_STEP = 32

supervisor = Supervisor()

# do this once only
robot_node = supervisor.getFromDef("PIONEER_3DX")
if robot_node is None:
    sys.stderr.write("No DEF Pioneer 3-DX node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    values = trans_field.getSFVec3f()
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))