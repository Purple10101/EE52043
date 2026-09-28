# import the API and define the simulator address
import sim_scara_motion_controller_api as ssrc_api
scara = ssrc_api.sim_scara_motion_controller(host='127.0.0.1', port=9000)


# Test move joint command
scara.SCARA_MOVE_JOINT(0,90,1)
scara.SCARA_MOVE_JOINT(0,0,1)
scara.SCARA_MOVE_JOINT(1,90,1)
# scara.SCARA_MOVE_JOINT(1,0,1)
# scara.SCARA_MOVE_JOINT(2,90,1)
# scara.SCARA_MOVE_JOINT(2,0,1)
# scara.SCARA_MOVE_JOINT(3,0.075,1)
# scara.SCARA_MOVE_JOINT(3,0.025,1)

# # Test move joints command
# scara.SCARA_MOVE_JOINTS(90,30,90,0.05,1)

# # Test move coord command
# scara.SCARA_MOVE_COORD(0.1,0.1,90,0.05,1)