import numpy as np


def get_exercise_list():
    return [
        "home",
        "rest",
        "sit and reach",
        "sit and kick",
        "stand and reach",
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


def get_exercise_specification(
    name, direction=None, difficulty=None, duration=None, cognitive=None
):
    # TODO: refactor (this code is a mess)
    assert any(name.lower() == x.lower() for x in get_exercise_list())
    data = {"name": name.lower()}

    if name.lower() == "home":
        data["movement"] = _get_home_spec()
        data["audio"] = _get_cognitive_null()
        return data

    assert duration > 0.0
    if name.lower() == "rest":
        data["movement"] = _get_rest_spec(duration)
        data["audio"] = _get_cognitive_null()
        return data

    assert any(direction.lower() == x.lower() for x in get_exercise_directions())
    assert any(difficulty.lower() == x.lower() for x in get_exercise_difficulties())
    assert isinstance(cognitive, bool)
    data["settings"] = {
        "direction": direction.lower(),
        "difficulty": difficulty.lower(),
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


def _get_home_spec():
    return {
        "position": {"x": 0.0, "y": 0.0, "a": 0.0},
        "poses": [
            {
                "start": {"arm_height": 0.85, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "stop": {"arm_height": 0.85, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "duration": 0,
            }
        ],
    }


def _get_rest_spec(duration):
    return {
        "position": None,
        "poses": [
            {
                "start": None,
                "stop": None,
                "duration": duration,
            }
        ],
    }


def _get_sit_and_reach_spec(direction, difficulty, duration):
    x = 0.2175
    if difficulty == "easy":
        x = 0.2175  # m
    elif difficulty == "medium":
        x = 0.3175  # m
    elif difficulty == "hard":
        x = 0.4175  # m

    y = 0.0  # m
    a = np.deg2rad(30.0)  # rad

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
    x = 0.2175
    if difficulty == "easy":
        h = 0.3  # m
    elif difficulty == "medium":
        h = 0.4  # m
    elif difficulty == "hard":
        h = 0.5  # m

    x = 0.2  # m
    y = 0.0  # m
    a = np.deg2rad(0.0)  # rad

    if direction == "left":
        x *= -1
        a *= -1

    return {
        "position": {"x": x, "y": y, "a": a},
        "poses": [
            {
                "start": {"arm_height": h, "arm_extension": 0.0, "wrist_yaw": 0.0},
                "stop": {"arm_height": h + 0.1, "arm_extension": 0.0, "wrist_yaw": 0.0},
                "duration": duration,
            }
        ],
    }


def _get_stand_and_reach_spec(direction, difficulty, duration):
    x = 0.2175
    if difficulty == "easy":
        x = 0.2175  # m
    elif difficulty == "medium":
        x = 0.3175  # m
    elif difficulty == "hard":
        x = 0.4175  # m

    y = 0.0  # m
    a = np.deg2rad(30.0)  # rad

    if direction == "left":
        x *= -1
        a *= -1

    return {
        "position": {"x": x, "y": y, "a": a},
        "poses": [
            {
                "start": {"arm_height": 1.0, "arm_extension": 0.55, "wrist_yaw": 0.0},
                "stop": {"arm_height": 1.0, "arm_extension": 0.05, "wrist_yaw": 0.0},
                "duration": duration,
            }
        ],
    }


def _get_cognitive_null():
    return {"active": False, "category": None, "unique": False}


def _get_cognitive_active():
    return {"active": True, "category": "colors", "unique": True}


if __name__ == "__main__":
    import json
    import random
    import sys

    def _get_random_exercise():
        return get_exercise_specification(
            name=random.choice(get_exercise_list()),
            direction=random.choice(get_exercise_directions()),
            difficulty=random.choice(get_exercise_difficulties()),
            duration=random.randint(0, 60),
            cognitive=bool(random.randint(0, 1)),
        )

    # # random exercise
    # N = random.randint(1, 5)
    # routine = [_get_random_exercise() for _ in range(N)]

    routine = [
        # get_exercise_specification("sit and reach", "right", "medium", 8, True),
        # get_exercise_specification("sit and reach", "left", "hard", 8, True),
        # get_exercise_specification("stand and reach", "right", "hard", 8, True),
        # get_exercise_specification("stand and reach", "left", "medium", 8, True),
        # get_exercise_specification("sit and kick", "right", "hard", 8, True),
        # get_exercise_specification("sit and kick", "left", "medium", 8, True),
        get_exercise_specification("home"),
    ]

    json.dump(routine, sys.stdout, indent=2)
