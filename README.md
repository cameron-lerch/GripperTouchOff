# Gripper Touch-Off

A Viam module that finds where a gripper's fingertips really are by touching them to the table. It then corrects the gripper's frame in the machine config, so motion planning knows where the fingertips are.

Motion planning keeps the gripper off the table by using the gripper's collision model. If the real fingertips reach further than the model (different fingers, an adapter plate, a frame offset that was only estimated), the planner will drive them into the table. On one xArm cell the real fingertips reached 24 mm past the gripper's model. The touch-off measures that error the way machinists set up a tool: they bring it down onto the work surface until it just pinches a sheet of paper.

```
reach = height of the gripper frame at contact − height of the table (− the paper's 0.1 mm)
```

There are two ways to touch:

| | **Jog with paper** | **Manual mode** |
|---|---|---|
| How | The closed gripper points straight down and comes down over bare table. You slide a sheet of paper under the fingertips and jog down in steps as small as 0.1 mm until the paper drags. | The arm goes into Viam's manual mode, so you can move it by hand. You rest the fingertips on the table with the gripper pointing straight down. |
| Needs | An arm the motion service can move | An arm that supports manual mode (`SetManualMode`), such as UFactory xArms |
| Precision | About the paper's thickness | About your hand: hold it still and level (the status card shows the live tilt) |

**Table height.** Either an RGB-D camera measures the table (and the module also picks bare spots to touch, clear of anything standing on it), or you give the table's height in the world frame.

**Updating the frame.** After the touches, the module moves the gripper's frame along its pointing axis, so the gripper's collision model ends exactly at the real fingertips. A gripper with no collision geometry gets its frame origin at the fingertips. The module writes the new frame to the machine's config through Viam's API:

- **Gripper defined in the part's config:** its `frame.translation` changes.
- **Gripper from a fragment:** the shared fragment is left alone. This machine gets a `fragment_mods` override instead.

The orientation never changes: touching a flat table measures distance along the pointing axis, not the angle of the gripper's mount.

You run it from the machine's **CONTROL** tab in the Viam app: each step is a button (*go to spot*, *jog down*, *record touch*, *finish and update*...), a switch picks the jog step, and a status card shows the fingertips' height above the table, the tilt, and what to do next.

## Contents

- [Set up](#set-up)
- [Run a touch-off](#run-a-touch-off)
- [DoCommand reference](#docommand-reference)
- [How the frame is updated](#how-the-frame-is-updated)
- [Safety](#safety)
- [Troubleshooting](#troubleshooting)
- [Development](#development)

## Set up

The module has a few parts. The Viam app draws its own controls for standard resource types (a **Push** button for a button component, its positions for a switch, readings for a sensor), so the touch-off's steps are small components that appear on the **CONTROL** tab:

| Model | API | Role |
|---|---|---|
| `touch-off` | generic service | The touch-off itself: moves, touches, the result, and the config update. Scripts drive it with DoCommand. |
| `setup` | discovery service | Offers all the controls below, already wired to a touch-off service, to add with one click. |
| `button` | button component | One step of the touch-off. Pushing it sends that step to the touch-off service. |
| `jog-step` | switch component | Chooses the jog step: 0.1, 0.5, 1 or 5 mm. |
| `status` | sensor component | The live state: fingertip gap, tilt, touches, result, and the next step. |

### 1. The touch-off service

Add the module from the registry (**CONFIGURE** tab → **+** → **Service** → search for *touch-off*), or add this to the machine's JSON:

```json
{
  "name": "touch-off",
  "api": "rdk:service:generic",
  "model": "cameronlerch:gripper-touch-off:touch-off",
  "attributes": {
    "arm": "arm",
    "gripper": "gripper",
    "camera": "cam"
  }
}
```

Before you start:

1. **The gripper needs a frame whose parent is the arm**, e.g. `"frame": {"parent": "arm", "translation": {"x": 0, "y": 0, "z": 105}, ...}`. By Viam's convention the gripper points along its frame's +z axis.
2. **Camera (optional).** If you use a camera to measure the table, it needs an accurate frame. For a camera on the arm, that means parented to the arm and calibrated, e.g. with the [camera-calibration](https://github.com/viam-labs/camera-calibration) module. The measured reach can only be as accurate as the table height the camera reports.
3. **No camera:** set `table_z_mm` to the table's height in the world frame. It is usually close to 0 for an arm standing on the table, but measure it: the base often sits a few millimeters off the surface.

### Attributes

| Attribute | Type | Default | Description |
|---|---|---|---|
| `arm` | string | **required** | The arm the gripper is mounted on. |
| `gripper` | string | **required** | The gripper component whose frame is corrected. |
| `motion` | string | `"builtin"` | The motion service used to move. |
| `camera` | string | | An RGB-D camera for `measure_table`. It needs a depth image (Viam's depth format or a 16-bit PNG) with intrinsics, or point clouds. |
| `depth_source` | string | | The camera's depth image source name, if it sends several images. |
| `table_z_mm` | number | | The table's height in the world frame, when no camera measures it. |
| `start_gap_mm` | number | `30` | The fingertips come straight down until the collision model says they're this high. Make it larger if the model may be more than this short of the real fingertips. |
| `measured_start_gap_mm` | number | `10` | The same, once a touch in this session has measured the reach. |
| `travel_height_mm` | number | `150` | Fingertip height above the table while moving between spots. |
| `max_step_mm` | number | `5` | The largest jog step. |
| `max_overtravel_mm` | number | `40` | Jogging stops this far below where contact was expected (10 mm once a touch has measured the reach). |
| `paper_thickness_mm` | number | `0.1` | Subtracted from jog touches. Manual touches assume bare table, unless `touch` is given `paper_mm`. |
| `close_gripper` | bool | `true` | Close the gripper before touching: the closed fingertips touch. |
| `expected_reach_mm` | number | | How far past the gripper frame the fingertips reach, if known roughly. **Required for the jog method when the gripper has no collision geometry.** When set, it is used instead of the model. |
| `contact_obstacles` | list of strings | `[]` | Obstacles (frame names, e.g. a configured `"table"`) that the gripper may touch on its straight way down. Use this when a configured table obstacle sits at or above the real table. |
| `max_tilt_deg` | number | `3` | Touches only count when the gripper points within this many degrees of straight down (at most 15). |
| `manual_mode_timeout_s` | number | `300` | The arm leaves manual mode by itself after this long. |
| `auto_apply` | bool | `true` | `finish` writes the new frame to the config. |
| `min_touches` | int | `1` | `finish` needs at least this many touches. |
| `depth_frames` | int | `5` | Number of depth frames combined (per-pixel median) to measure the table. |
| `min_depth_mm`, `max_depth_mm` | number | `150`, `1500` | Depth readings outside this range are ignored. |
| `table_fit_tolerance_mm` | number | `4` | Points within this distance of the fitted plane count as table. |
| `max_table_tilt_deg` | number | `10` | The steepest plane that is still accepted as the table. |
| `obstacle_height_mm` | number | `10` | Points this far above the table belong to things standing on it. |
| `spot_clearance_mm` | number | `80` | Contact spots stay this far from things on the table. |
| `min_base_distance_mm` | number | `150` | ...and this far, horizontally, from the arm's base (the world origin). |
| `move_timeout_s` | number | `60` | Deadline for each move. |

### 2. Add the controls

Add a **setup** service (**+** → **Service** → *discovery* → `cameronlerch:gripper-touch-off:setup`) that names the touch-off service:

```json
{
  "name": "touch-off-setup",
  "api": "rdk:service:discovery",
  "model": "cameronlerch:gripper-touch-off:setup",
  "attributes": {"touch_off": "touch-off"}
}
```

Save, open its **TEST** panel, and click **Add component** on each control it lists (the way the find-webcams module offers cameras). It only offers what your setup can use: no *measure table* button without a camera, and no manual-mode buttons if the arm doesn't support manual mode. The controls are named after the service, numbered in the order you use them:

| Name | Type | Pushing it / what it shows |
|---|---|---|
| `touch-off-0-status` | sensor | `state`, `next_step`, `fingertip_gap_mm`, `tilt_from_down_deg`, `jog_step_mm`, `table_z_mm`, `touches`, `last_touch_reach_mm`, and after finishing: `reach_mm`, `frame_shift_mm`, `old_frame_translation`, `new_frame_translation`, `applied`. |
| `touch-off-stop` | button | Stops the arm at once, cancels any move, and leaves manual mode. |
| `touch-off-1-measure-table` | button | Measures the table with the camera. |
| `touch-off-2-go-to-spot` | button | Closes the gripper, moves over the next bare spot, and comes down to `start_gap_mm` above the table. |
| `touch-off-2-start-manual-mode` | button | Puts the arm in manual mode, to move it by hand. |
| `touch-off-3-jog-step` | switch | The jog step: 0.1, 0.5, 1 or 5 mm (only steps up to `max_step_mm`). |
| `touch-off-3-jog-down`, `touch-off-3-jog-up` | button | Moves the fingertips one jog step down or up. |
| `touch-off-4-record-touch` | button | Records the contact. |
| `touch-off-5-leave-manual-mode` | button | Leaves manual mode. |
| `touch-off-6-preview-result` | button | Computes the result and the new frame without changing anything. |
| `touch-off-6-finish-and-update` | button | Computes the result and writes the new frame to the machine config. |
| `touch-off-7-undo-update` | button | Puts back the frame from before the last update. |
| `touch-off-7-clear-touches` | button | Forgets the touches and the result (keeps the table). |

Once they're added you can delete the setup service, or keep it to add controls again later.

### The controls

You can also add the controls by hand. Each takes `touch_off` (the touch-off service's name), and:

| Model | Attribute | Values |
|---|---|---|
| `button` | `action` (required) | `stop`, `measure_table`, `go_to_spot`, `start_manual`, `jog_down`, `jog_up`, `record_touch`, `stop_manual`, `preview`, `finish`, `undo`, `clear_touches` |
| `jog-step` | `steps_mm` | The step sizes, default `[0.1, 0.5, 1, 5]`. |
| `status` | | |

For example, a button that records a touch:

```json
{
  "name": "record-touch",
  "api": "rdk:component:button",
  "model": "cameronlerch:gripper-touch-off:button",
  "attributes": {"touch_off": "touch-off", "action": "record_touch"}
}
```

A refused push (e.g. *jog down* before going to a spot) shows the reason as the error, such as `Not jogging: go to a spot first (start_jog)`.

## Run a touch-off

### From the CONTROL tab

Open the machine's **CONTROL** tab. Keep the `touch-off-0-status` card in view: its `next_step` reading always says which button to push next, and `fingertip_gap_mm` shows how high the fingertips are above the table. Push `touch-off-stop` at any time to halt all motion and leave manual mode.

1. **Table height.** With the camera looking at bare table, push `1-measure-table`. Without a camera, the table height comes from `table_z_mm`.
2. **Touch.** Choose one method:
   - **Jog with paper.** Push `2-go-to-spot`. The gripper closes, moves over a spot the camera saw clear (or, with no camera, comes down where the gripper is), and stops when the model says the fingertips are 30 mm up. Slide a sheet of paper under them. Set `3-jog-step` and push `3-jog-down`: 5 mm while there's room, then 1 mm, then 0.5 or 0.1 mm. Stop the moment the paper drags, and push `4-record-touch`. The gripper goes back up. Repeat at more spots if you like: three spread-out touches also show how the table the arm touches is tilted relative to the one the camera sees.
   - **Manual mode.** Push `2-start-manual-mode` and bring the closed fingertips down onto the table by hand. Keep `tilt_from_down_deg` under `max_tilt_deg`, hold still, and push `4-record-touch`. Repeat elsewhere, then push `5-leave-manual-mode`.
3. **Update.** Push `6-preview-result` to see the measured reach, the collision model's reach, and the new frame (`new_frame_translation`) without changing anything. Then push `6-finish-and-update`. The machine picks up the new frame the next time it syncs its config with Viam, normally within seconds.
4. **Check.** Touch off once more and push `6-preview-result`: `frame_shift_mm` should now be close to 0.

`7-undo-update` puts back the frame from before the last update. It refuses if the frame has changed since then.

### With DoCommand

The buttons send DoCommands to the touch-off service, and you can send them yourself from its card on the **CONTROL** tab, one step at a time:

```json
{"command": "measure_table"}
{"command": "start_jog"}
{"command": "status"}
{"command": "jog", "mm": 5}
{"command": "jog", "mm": 1}
{"command": "touch"}
{"command": "finish"}
```

The same calls work from any SDK, e.g. in Python:

```python
touch_off = Generic.from_robot(machine, "touch-off")
await touch_off.do_command({"command": "start_jog", "wait": True})
```

## DoCommand reference

Send `{"command": "<name>", ...arguments}`. The `{"<name>": {...arguments}}` form also works.

| Command | Arguments | What it does |
|---|---|---|
| `status` | | State (`idle`, `moving`, `jogging`, `manual`), the next instruction, the table, the touches, the last result, and the live gripper pose: position, tilt, and estimated fingertip height above the table. Also the last update applied, which survives the service restarting. |
| `measure_table` | | Fits the table plane in the camera's depth image. Also finds things standing on the table, and bare spots to touch. |
| `set_table` | `z_mm` | Uses a level table at this height (world frame). |
| `start_jog` | `x`, `y` (optional), `wait` | Closes the gripper, moves over the next bare spot (or `x`, `y`), and comes straight down to `start_gap_mm` above the table. Returns at once and runs in the background, unless `wait` is true. |
| `jog` | `mm` or `up` | Moves the fingertips `mm` down (negative: up) in a straight line, at most `max_step_mm` at a time. Without `mm`, moves one jog step down (or up, with `"up": true`). Waits for the move to finish. |
| `set_jog_step` | `mm` | Sets the step `jog` uses without `mm` (what the `jog-step` switch does). |
| `capabilities` | | Whether a camera is configured and the arm supports manual mode, and `max_step_mm`. |
| `touch` | `paper_mm` (optional), `wait` | Records the contact. When jogging, the gripper then goes back up in the background. |
| `start_manual` / `stop_manual` | | Enters or leaves manual mode. |
| `finish` | `apply` (optional), `force` | Combines the touches (median) and returns the result and the new frame. With `apply` (default: `auto_apply`), writes the new frame to the config. Refuses frame changes over 150 mm unless `force` is set. |
| `undo` | | Restores the frame from before the last applied update. |
| `remove_touch` | `index` | Drops one touch (numbered from 0). |
| `reset` | `keep_table` (default `true`) | Clears the touches and the result. |
| `stop` | | Stops the arm, cancels any move, and leaves manual mode. Never waits behind another command. |

Errors come back as `FAILED_PRECONDITION` errors whose message is just the reason and what to do next, e.g. `Steps are limited to 5mm`.

## How the frame is updated

The gripper's collision model comes from its `GetGeometries`. The module computes how far that model reaches along the pointing axis (`modeled_reach_mm`). After the touches, the frame moves along the gripper's own +z axis by

```
frame_shift_mm = measured reach − modeled reach      (modeled reach = 0 with no geometry)
```

This moves the whole model with it, so the model's tip lands on the real fingertips. The shift is expressed in the frame's parent (usually the arm) and added to `frame.translation`.

For example, an xArm gripper mounted at `z: 105` with a 50 mm model, whose fingertips really reach 74 mm, moves to `z: 129`.

Where the change is written:

- **The gripper is in the part's `components`:** `frame.translation` is replaced. Nothing else in the config changes, including the order of its keys.
- **The gripper comes from a fragment the part imports:** the module appends this to the part's `fragment_mods`:
  ```json
  {"fragment_id": "<the fragment>", "mods": [{"$set": {"components.gripper.frame.translation": {"x": 0, "y": 0, "z": 129.1}}}]}
  ```
  It replaces any earlier override of the same translation and keeps your other mods. The fragment itself, shared with other machines, is not changed. Fragment variables (e.g. a `$variable` parent) are resolved before measuring.
- **Not supported:** grippers from nested fragments, from fragments imported with a `prefix`, or on machines without cloud credentials. In these cases `finish` returns the new frame for you to paste in by hand.

The write uses the credentials viam-server gives every module (`VIAM_API_KEY`, `VIAM_API_KEY_ID`, `VIAM_MACHINE_PART_ID`). It is refused if the config changed after the module read it, so a concurrent edit in the Viam app is never overwritten. Each update is logged to `touch_off_history.json` in the module's data directory, which is what `undo` reads.

## Safety

The touch-off deliberately brings the gripper close to the table. Stay at the cell with the e-stop in reach.

- **The first descent trusts the collision model.** It stops `start_gap_mm` (30 mm) above where the model puts the table. If the real fingertips may reach further than that past the model, raise `start_gap_mm` or set `expected_reach_mm`.
- **Jogging stops at a floor**: `max_overtravel_mm` below the expected contact. A wrong table height or reach estimate is refused, not driven through.
- **Steps are capped** at `max_step_mm`, and moves between spots avoid everything the camera saw on the table, plus the table itself.
- **`touch-off-stop`** (or `{"command": "stop"}`) halts the arm at once, even in the middle of another command.
- **Manual mode** ends by itself after `manual_mode_timeout_s`, and when the service closes. On some arms, entering manual mode while touching something raises a controller error (UFactory: C37). Enter it with the gripper clear of the table.
- **Closing the CONTROL tab doesn't stop a move that is already running**; push `touch-off-stop`.

## Troubleshooting

| Message | Cause and fix |
|---|---|
| `No plane within 10 deg of horizontal…` | The camera doesn't see enough bare table, or its frame is wrong. Point it at the table from 0.3–1 m away, and check the camera's frame. |
| `The motion service couldn't make the straight move: … collision … table` | A configured table obstacle is at or above the real table. Lower it, or add its name to `contact_obstacles`. |
| `No reachable spot` | The spots the camera found are out of reach. Measure the table from somewhere else, or move the gripper over bare table with the arm's own card and push `2-go-to-spot` (with no bare spot known it comes down where the gripper is; DoCommand `start_jog` also takes an `x` and `y`). |
| `Refused: that would put the fingertips more than 40mm past where contact was expected` | The paper should have dragged by now. Check the table height (camera frame, `table_z_mm`) and `expected_reach_mm`. |
| `The gripper is tilted …` | Manual touches only count with the gripper pointing straight down. Straighten it until `tilt_from_down_deg` is under `max_tilt_deg`. |
| `… isn't in this machine part's config or a fragment it imports directly` / `VIAM_API_KEY … not set` | The module can't write this config. Copy `frame.translation` from the `finish` result into the config by hand. |
| `The config's translation … differs from the frame system's` (a note in the result) | The machine isn't running the config the module read (e.g. a pending config change). Wait for the machine to reconfigure, then finish again. |

## Development

```sh
./setup.sh                 # .venv with the module's and the tests' requirements
make test                  # unit tests and a simulated cell (no robot needed)
./build.sh                 # dist/main (PyInstaller) and dist/archive.tar.gz
```

- **Tests.** The tests in `tests/` run the whole procedure against a simulated cell (`tests/sim.py`). In that cell, the real fingertips reach 24 mm past the collision model, the camera renders depth images of a table with a block on it, and the cloud config holds the gripper either in the part or in a fragment.
- **A local viam-server.** To try the module with simulated hardware, run a viam-server with a fake arm (`"arm-model": "ur5e"`) and a fake gripper parented to it, plus a local module pointing at `run.sh` (source) or `dist/archive.tar.gz` (the built package). Add the touch-off and setup services, add the controls the setup service offers, and drive them with the Python SDK's `Button`, `Switch` and `Sensor` clients.
- **Publish.** Change the `cameronlerch` namespace in `meta.json`, `src/touchoff/service.py` and this README to your organization's, then run `viam module create`, `viam module update`, and `viam module build start --version x.y.z`. Or push a semver tag with the GitHub workflow in `.github/workflows/`.

Layout:

```
src/main.py                  module entrypoint
src/touchoff/service.py      the Viam generic service: config, dependencies, DoCommand
src/touchoff/controls.py     the CONTROL-tab components (button, jog-step switch, status sensor) and the setup discovery service
src/touchoff/procedure.py    the touch-off state machine (moves, touches, finish/undo)
src/touchoff/geometry.py     poses, table plane fit, depth decoding, contact spots, reach math
src/touchoff/machine_config.py  reading and writing the gripper frame (part config or fragment_mods)
src/touchoff/settings.py     attributes and validation
tests/                       pytest suite and the simulated cell
```

The touch-off procedure comes from the ViamObjectReconstruction project, where it measured the fingertips of an xArm 850 before 3D scanning.
