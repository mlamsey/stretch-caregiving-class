import numpy as np
import json
import os
from user_interface import get_user_specification


def read_json(file_name):
    with open(file_name) as json_file:
        return json.load(json_file)


def write_json(file_name, user_dict):
    with open(file_name, 'w') as json_file:
        json.dump(user_dict, json_file, indent=2)


class User:
    def __init__(self, name=None, smpl_betas=np.zeros([1, 10]), rom_model=None):
        self.name = name
        self.smpl_betas = smpl_betas
        self.rom_model = rom_model

    def load_json(self, path=None):
        if path is None:
            path = os.path.expanduser("~" + "/matt.json")
        
        user = read_json(path)

        # extract
        self.name = user["name"]
        self.smpl_betas = user["smpl_betas"]
        self.rom_model = user["rom_model"]

    def save_json(self, path=None):
        user_dict = get_user_specification(self)
        if path == None:
            home = os.path.expanduser("~") + "/"
            file = input("Please input a filename, ending with .json: ")
            path = home + file

        write_json(path, user_dict)
