# Model viamdemo:HelloMotion:hellomotion

A generic service module that provides arm motion commands: waving and picking up the largest detected object using a vision service.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "arm": "<string>",
  "gripper": "<string>",
  "pickup_vision": "<string>"
}
```

### Attributes

| Name             | Type   | Inclusion | Description                                                                                     |
|------------------|--------|-----------|-------------------------------------------------------------------------------------------------|
| `arm`            | string | Required  | The name of the arm component to control.                                                       |
| `gripper`        | string | Required  | The name of the gripper component to use for grabbing objects.                                   |
| `pickup_vision`  | string | Required  | The name of the vision service (e.g., `obstacles-pointcloud`) used to detect objects to pick up. |

### Example Configuration

```json
{
  "arm": "uf850",
  "gripper": "my-gripper",
  "pickup_vision": "my-obstacles-pointcloud"
}
```

## DoCommand

### `wave`

Moves the arm 20mm in the Y direction from its current pose.

```json
{
  "wave": true
}
```

### `pickup`

Uses the configured vision service to detect objects via `GetObjectPointClouds`, finds the largest object by point count, moves the arm to that object's pose offset by +400mm in Z, then closes the gripper. Returns the detected object's pose.

```json
{
  "pickup": true
}
```

Example response:

```json
{
  "pickup": true,
  "pose": {
    "x": 100.5,
    "y": -200.3,
    "z": 50.0
  }
}
```
