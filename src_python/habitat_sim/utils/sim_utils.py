# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Dict

# from habitat_sim.simulator import Simulator
import magnum as mn


def get_all_object_ids(sim) -> Dict[int, str]:
    """
    Construct and return a dict of object ids to descriptive strings with object and link names
    """
    rom = sim.get_rigid_object_manager()
    aom = sim.get_articulated_object_manager()

    object_id_map = {}

    for _object_handle, rigid_object in rom.get_objects_by_handle_substring("").items():
        object_id_map[rigid_object.object_id] = rigid_object.handle

    for _object_handle, ao in aom.get_objects_by_handle_substring("").items():
        object_id_map[ao.object_id] = ao.handle
        for object_id, link_ix in ao.link_object_ids.items():
            object_id_map[object_id] = ao.handle + " -- " + ao.get_link_name(link_ix)

    return object_id_map


def debug_draw_contact_points(sim) -> None:
    """
    Query contact information from the simulator and use the DebugLineRender to render a visualization of contact points.
    """
    dlr = sim.get_debug_line_render()
    yellow = mn.Color4(1.0, 1.0, 0.0, 1.0)
    red = mn.Color4(1.0, 0.0, 0.0, 1.0)
    cps = sim.get_physics_contact_points()
    active_contacts = [x for x in cps if x.is_active]
    for cp in active_contacts:
        dlr.draw_transformed_line(
            cp.position_on_b_in_ws,
            cp.position_on_b_in_ws + cp.contact_normal_on_b_in_ws * cp.contact_distance,
            yellow,
        )
        # dlr.draw_transformed_line(
        #     cp.position_on_b_in_ws
        #     + cp.contact_normal_on_b_in_ws * cp.contact_distance,
        #     cp.position_on_b_in_ws + cp.contact_normal_on_b_in_ws,
        #     red,
        # )
        dlr.draw_circle(cp.position_on_b_in_ws, 0.02, red)
