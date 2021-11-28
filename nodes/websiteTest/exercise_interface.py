def get_exercise_list():
    return [
        "Sit and Reach",
        "Sit and Kick",
        "Stand and Reach",
    ]


def get_exercise_difficulties():
    return [
        "easy",
        "medium",
        "hard",
    ]


def get_exercise_directions():
    return [
        "left",
        "right",
    ]


def get_exercise_specification(name, direction, difficulty, duration, cognitive):
    assert name in get_exercise_list()
    assert direction in get_exercise_directions()
    assert difficulty in get_exercise_difficulties()

    data = {
        "name": name,
        "direction": direction,
        "difficulty": difficulty,
        "duration": duration,
        "cognitive": cognitive,
    }

    if name.lower() == "sit and reach":
        data["movement"] = _get_sit_and_reach_spec(direction, difficulty, duration)
    elif name.lower() == "sit and kick":
        data["movement"] = _get_sit_and_kick_spec(direction, difficulty, duration)
    elif name.lower() == "stand and reach":
        data["movement"] = _get_stand_and_reach_spec(direction, difficulty, duration)

    if cognitive:
        data["audio"] = _get_cognitive_active()
    else:
        data["audio"] = _get_cognitive_null()

    return data


# ---------------- #
# Helper Functions #
# ---------------- #


def _get_sit_and_reach_spec(direction, difficulty, duration):
    if difficulty == "easy":
        x = 0.2175  # m
    elif difficulty == "medium":
        x = 0.3175  # m
    elif difficulty == "hard":
        x = 0.4175  # m

    y = 0.0  # m
    a = 30.0  # degrees

    if direction == "left":
        x *= -1
        a *= -1

    return {
        "position": {"x": x, "y": y, "a": a},
        "poses": [
            {
                "start": {"arm_height": 0.8, "arm_extension": 0.55, "wrist_yaw": 0.0},
                "stop": {"arm_height": 0.8, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "duration": duration,
            }
        ],
    }


def _get_sit_and_kick_spec(direction, difficulty, duration):
    if difficulty == "easy":
        x = 0.2175  # m
    elif difficulty == "medium":
        x = 0.3175  # m
    elif difficulty == "hard":
        x = 0.4175  # m

    y = 0.0  # m
    a = -10.0  # degrees

    if direction == "left":
        x *= -1
        a *= -1

    return {
        "position": {"x": x, "y": y, "a": a},
        "poses": [
            {
                "start": {"arm_height": 0.4, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "stop": {"arm_height": 0.4, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "duration": duration,
            }
        ],
    }


def _get_stand_and_reach_spec(direction, difficulty, duration):
    if difficulty == "easy":
        x = 0.2175  # m
    elif difficulty == "medium":
        x = 0.3175  # m
    elif difficulty == "hard":
        x = 0.4175  # m

    y = 0.0  # m
    a = 30.0  # degrees

    if direction == "left":
        x *= -1
        a *= -1

    return {
        "position": {"x": x, "y": y, "a": a},
        "poses": [
            {
                "start": {"arm_height": 1.1, "arm_extension": 0.55, "wrist_yaw": 0.0},
                "stop": {"arm_height": 0.1, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "duration": duration,
            }
        ],
    }


def _get_cognitive_null():
    return {"active": False, "category": None, "unique": False}


def _get_cognitive_active():
    return {"active": True, "category": "colors", "unique": True}
