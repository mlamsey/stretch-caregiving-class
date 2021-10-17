import os
from datetime import datetime
from enum import Enum


class LogState(Enum):

    EXERCISE_STARTED = 0
    CONTACT_DETECTED = 1
    EXERCISE_ENDED = 2


class GameLogger:
    def __init__(self):
        self.filename = os.path.join(
            "..", datetime.now().strftime("workout-%Y-%m-%d-%H%M.log")
        )

    def add_line(self, currExercise, logState):
        with open(self.filename, "a") as f:
            currTime = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            newString = "{}, {}, {}\n".format(currTime, currExercise, logState.name)
            f.write(newString)


