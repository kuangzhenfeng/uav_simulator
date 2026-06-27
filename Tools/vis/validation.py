#!/usr/bin/env python3
import logging

VALID_MODELS = {"Agri_AG20", "Agri_AG60", "Agri_AG100", "Map_SVPro", "Map_SVLiDAR"}
VALID_OBS_TYPES = {"Sphere", "Box", "Cylinder"}
VALID_MOVEMENTS = {"Static", "LinearVelocity", "PatrolLoop", "PatrolPingPong"}
VALID_MODES = {"Once", "Loop", "PingPong"}


def _is_vec3(value):
    return isinstance(value, (list, tuple)) and len(value) == 3 and all(isinstance(item, (int, float)) for item in value)


def validate_scenario_dto(dto):
    logger = logging.getLogger("vis_server.validator")

    def _fail(message):
        logger.warning("DTO validation failed: %s", message)
        return False, message

    if not isinstance(dto, dict):
        return _fail("dto must be an object")
    fleet = dto.get("fleet")
    if not isinstance(fleet, list) or len(fleet) == 0:
        return _fail("fleet must be a non-empty array")
    if len(fleet) > 16:
        return _fail("fleet too large (max 16)")
    for index, agent in enumerate(fleet):
        if not isinstance(agent, dict):
            return _fail(f"fleet[{index}] must be an object")
        if agent.get("model") not in VALID_MODELS:
            return _fail(f"fleet[{index}].model invalid")
        if not _is_vec3(agent.get("initPos", [0, 0, 0])):
            return _fail(f"fleet[{index}].initPos must be [x,y,z]")
        mode = agent.get("mode", "Once")
        if mode not in VALID_MODES:
            return _fail(f"fleet[{index}].mode invalid")
        for waypoint_index, waypoint in enumerate(agent.get("waypoints", [])):
            if not isinstance(waypoint, dict) or not _is_vec3(waypoint.get("pos")):
                return _fail(f"fleet[{index}].waypoints[{waypoint_index}].pos invalid")
    obstacles = dto.get("obstacles", [])
    if isinstance(obstacles, list):
        if len(obstacles) > 64:
            return _fail("too many obstacles (max 64)")
        for index, obstacle in enumerate(obstacles):
            if not isinstance(obstacle, dict):
                return _fail(f"obstacles[{index}] must be an object")
            if obstacle.get("type", "Box") not in VALID_OBS_TYPES:
                return _fail(f"obstacles[{index}].type invalid")
            if obstacle.get("movement", "Static") not in VALID_MOVEMENTS:
                return _fail(f"obstacles[{index}].movement invalid")
            if not _is_vec3(obstacle.get("center", [0, 0, 0])):
                return _fail(f"obstacles[{index}].center invalid")
            speed = obstacle.get("patrolSpeed", 300)
            if not isinstance(speed, (int, float)) or speed < 0:
                return _fail(f"obstacles[{index}].patrolSpeed must be >= 0")
    sim = dto.get("sim", {})
    if isinstance(sim, dict):
        slomo = sim.get("slomo", 0)
        if not isinstance(slomo, (int, float)) or slomo < 0:
            return _fail("sim.slomo must be >= 0")
    return True, None
