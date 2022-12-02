#!/usr/bin/python3
import copy

def planejar_cartesiano(move_group, scale = 1.0):
    waypoints = []

    wpose = move_group.get_current_pose().pose

    wpose.position.x += scale * 0.5  
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold
    
    return plan, fraction
