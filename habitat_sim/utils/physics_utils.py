#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, List

import magnum as mn
import sim_utilities as sutils
from magnum.platform.glfw import Application

from examples.settings import default_sim_settings
from examples.viewer import HabitatSimInteractiveViewer
from habitat_sim.simulator import Simulator


def get_contact_pairs(contact_points, link_resolution=True):
    """
    Compute a dictionary keyed by contact pair tuples containing max contact distance between pairs.
    If link_resolution is true, also include links in the key tuple (a, b, l_a, l_b)
    """
    contact_pairs = {}
    for cp in contact_points:
        pair = (
            (cp.object_id_a, cp.object_id_b)
            if not link_resolution
            else (cp.object_id_a, cp.object_id_b, cp.link_id_a, cp.link_id_b)
        )
        if pair not in contact_pairs:
            contact_pairs[pair] = cp.contact_distance
        else:
            contact_pairs[pair] = max(contact_pairs[pair], cp.contact_distance)
    return contact_pairs


def get_rigid_component_name(sim, object_id, link_id):
    """
    Computes a descriptive name for the rigid component identified by the passed object_id and link_id from a ContactPointData.
    example: 'ao--body_name--link_name' or 'ro--body_name'
    """
    # handle the stage
    if object_id == -1:
        return "stage"
    # check rigid object first
    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_id(object_id):
        ro = rom.get_object_by_id(object_id)
        short_handle = ro.handle.split("/")[-1]
        return f"ro--{short_handle}"
    # check articulated object
    aom = sim.get_articulated_object_manager()
    if aom.get_library_has_id(object_id):
        ao = aom.get_object_by_id(object_id)
        short_handle = ao.handle.split("/")[-1]
        link_name = ao.get_link_name(link_id)
        return f"ao--{short_handle}--{link_name}"
    # not a RO or AO, so bad object_id
    raise ValueError("object_id is not valid")


def check_contacts(sim: Simulator) -> List[Any]:
    """
    Check for contacts between objects in the scene and return the active contact pairs.
    """
    print("-------------- check contacts -------------")
    # ids_to_names = sutils.get_all_object_ids(self.sim)
    # aom = self.sim.get_articulated_object_manager()
    sim.perform_discrete_collision_detection()
    cps = sim.get_physics_contact_points()
    active_contacts = [x for x in cps if x.is_active]
    # inactive_contacts = [x for x in cps if not x.is_active]

    active_contact_pairs = get_contact_pairs(active_contacts)
    # inactive_contact_pairs = get_contact_pairs(inactive_contacts)

    print("Active contact pairs:")
    for contact_pair, max_dist in active_contact_pairs.items():
        print(
            f"    ({get_rigid_component_name(sim, contact_pair[0], contact_pair[2])} vs {get_rigid_component_name(sim, contact_pair[1], contact_pair[3])}): {max_dist}"
        )
        print(f"        : {contact_pair}: {max_dist}")

    # print("Active contact points:")
    # for cp in active_contacts:
    #     print(f"    {cp.contact_distance} ({cp.object_id_a} vs {cp.object_id_b})")
    # print("Inactive contact points:")
    # for cp in inactive_contacts:
    #     print(f"    {cp.contact_distance} ({cp.object_id_a} vs {cp.object_id_b})")
    print("-------------- done check contacts -------------")
    return active_contact_pairs


class PhysicsSceneValidator(HabitatSimInteractiveViewer):
    """
    Interactive viewer with
    """

    def __init__(self, dataset_path, scene_id) -> None:

        sim_settings: Dict[str, Any] = default_sim_settings
        sim_settings["scene"] = scene_id
        sim_settings["scene_dataset_config_file"] = dataset_path
        sim_settings["enable_physics"] = True

        super().__init__(sim_settings)

        self.simulating = False

        # the current scene's index in the list of available scene instances
        self.dataset_scene_index = -1

        # initialize the debug visualizer
        self.vdb = sutils.DebugVisualizer(self.sim, output_path="phys_analysis_output/")

    # def save_scene_instance(self):
    #    todo = 1

    def key_press_event(self, event) -> None:

        key = event.key
        pressed = Application.KeyEvent.Key

        if key == pressed.C:
            if event.modifiers & Application.InputEvent.Modifier.SHIFT:
                # check all scenes sequentially and stop when a contact is detected
                scene_ids = self.sim.metadata_mediator.get_scene_handles()

                for scene_ix in range(self.dataset_scene_index + 1, len(scene_ids)):
                    # load the next scene
                    self.sim_settings["scene"] = scene_ids[scene_ix]
                    print(
                        f"    -!- testing scene {scene_ix}/{len(scene_ids)} = {self.sim_settings['scene']}"
                    )
                    self.reconfigure_sim()
                    active_contact_pairs = check_contacts(self.sim)
                    self.dataset_scene_index = scene_ix
                    if len(active_contact_pairs) > 0:
                        break

                # handle wrapping
                if (self.dataset_scene_index + 1) == len(scene_ids):
                    print(
                        "---------------------------------------------------------------"
                    )
                    print(
                        f"On the last scene({self.dataset_scene_index}/{len(scene_ids)}), restarting on next request."
                    )
                    print(
                        "---------------------------------------------------------------"
                    )
                    self.dataset_scene_index = -1

            else:
                # single contact check on current scene
                check_contacts(self.sim)

        super().key_press_event(event)
        event.accepted = True

    def debug_draw(self):
        """
        Function called during "draw_event". Overwrite with custom debug draw calls.
        This version: draws all active contact points in red.
        Hint: use with paused simulation and 'check_contacts' to see a static profile of active contacts.
        """
        yellow = mn.Color4(1.0, 1.0, 0.0, 1.0)
        red = mn.Color4(1.0, 0.0, 0.0, 1.0)
        cps = self.sim.get_physics_contact_points()
        active_contacts = [x for x in cps if x.is_active]
        for cp in active_contacts:
            self.sim.get_debug_line_render().draw_transformed_line(
                cp.position_on_b_in_ws,
                cp.position_on_b_in_ws
                + cp.contact_normal_on_b_in_ws * cp.contact_distance,
                yellow,
            )
            self.sim.get_debug_line_render().draw_transformed_line(
                cp.position_on_b_in_ws
                + cp.contact_normal_on_b_in_ws * cp.contact_distance,
                cp.position_on_b_in_ws + cp.contact_normal_on_b_in_ws,
                red,
            )
            self.sim.get_debug_line_render().draw_circle(
                cp.position_on_b_in_ws, 0.02, red
            )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--dataset",
        default="default",
        type=str,
        metavar="DATASET",
        help="dataset configuration file to use (default: default)",
    )

    parser.add_argument(
        "--scene",
        default="NONE",
        type=str,
        help='scene/stage file to load (default: "NONE")',
    )

    args = parser.parse_args()

    PhysicsSceneValidator(dataset_path=args.dataset, scene_id=args.scene).exec()
