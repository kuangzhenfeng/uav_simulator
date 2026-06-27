#!/usr/bin/env python3
import json
import logging
import os

PRESET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "presets")


def preset_path(name):
    if not name or not isinstance(name, str):
        return None
    if not all(char.isalnum() or char in "_-" for char in name):
        return None
    return os.path.join(PRESET_DIR, f"{name}.json")


def list_presets():
    if not os.path.isdir(PRESET_DIR):
        return []
    out = []
    for filename in sorted(os.listdir(PRESET_DIR)):
        if filename.endswith(".json"):
            out.append(filename[:-5])
    return out


def read_preset(name):
    path = preset_path(name)
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError) as error:
        logging.getLogger("vis_server").warning("Preset read failed [%s]: %s", name, error)
        return None


def write_preset(name, dto):
    path = preset_path(name)
    if not path:
        return False, "invalid preset name"
    os.makedirs(PRESET_DIR, exist_ok=True)
    try:
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(dto, stream, ensure_ascii=False, indent=2)
        logging.getLogger("vis_server").info("Preset saved: %s", name)
        return True, None
    except OSError as error:
        logging.getLogger("vis_server").warning("Preset save failed [%s]: %s", name, error)
        return False, str(error)


def delete_preset(name):
    path = preset_path(name)
    if not path or not os.path.isfile(path):
        return False, "not found"
    try:
        os.remove(path)
        logging.getLogger("vis_server").info("Preset deleted: %s", name)
        return True, None
    except OSError as error:
        return False, str(error)
