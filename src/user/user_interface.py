def get_user_specification(user):
    spec = dict()
    spec["name"] = user.name
    spec["smpl_betas"] = user.smpl_betas.tolist()
    spec["rom_model"] = user.rom_model
    return spec