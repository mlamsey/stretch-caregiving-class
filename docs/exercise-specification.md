# Exercise Specification

The current specification has the following limitations:
- the base position is fixed during the exercise

## Elements:

```
Position:
{
  “x”: float,  # relative x position (m)
  “y”: float,  # relative y position (m)
  “a”: float,  # relative angle (degrees)
}
```

```
Pose:
{
  “arm_height”: float,  # arm height (m)
  “arm_extension”: float, # arm extension (m)
  “wrist_yaw”: float,  # wrist yaw (degrees)
}
```

```
PosePair:
{
  “start”: Pose,  # starting pose
  “stop”: Pose,  # ending pose
  “duration”: float,  # time between poses (s)
}
```

```
Audio:
{
  “recording”: bool,  # record audio
  “category”: str,  # options: ‘colors’
  “require_unique”: bool,  # require uniqueness
}
```

```
Exercise
{
  “position”: Position,
  “poses”: List[PosePair],
  “audio”: Audio,
}
```

```
ExerciseRoutine: List[Exercise]
```

## Example Routine

```
[
  {
    “position”: {“x”: 0.3175, “y”: 0.0, “a”: 30.0},
    “poses”:
    [
      {
        “start”:
        {
          “arm_height”: 0.8,
          “arm_extension”: 0.55,
          “wrist_yaw”: 0.0
        },
        “stop”:
        {
          “arm_height”: 0.8,
          “arm_extension”: 0.05,
          “wrist_yaw”: 0.0
        },
        “duration”: 8.0
      }
    ],
    “audio”:
    {
      “recording”: False,
      “category”: None,
      “require_unique”: None
    },
  },
  {
    “position”: {“x”: -0.3175, “y”: 0.0, “a”: -30.0},
    “poses”:
    [
      {
        “start”:
        {
          “arm_height”: 0.8,
          “arm_extension”: 0.55,
          “wrist_yaw”: 0.0
        },
        “stop”:
        {
          “arm_height”: 0.8,
          “arm_extension”: 0.05,
          “wrist_yaw”: 0.0
        },
        “duration”: 8.0
      }
    ],
    “audio”:
    {
      “recording”: False,
      “category”: None,
      “require_unique”: None
    },
  },
]
```
