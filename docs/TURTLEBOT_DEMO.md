# Running SPITE-D on a real TurtleBot

Thanks for testing this. The short version of what you're looking at:

The robot plans using a **roadmap** — a precomputed graph of places it
could be and motions between them. That works fine until something
moves. This package watches the depth camera, tracks moving obstacles,
predicts where they're going, and marks roadmap edges as safe or blocked
*before* the obstacle gets there, so the plan can reroute early.

**The robot does not drive in this demo.** It sits still and watches.
Nothing here publishes to `cmd_vel`. Everything upstream of actually
turning the wheels is running; the execution loop isn't built yet, and
the roadmap is for a free-flying box rather than a differential drive, so
its paths aren't drivable as-is anyway.

What you should see: a person walking through the space appears as a
tracked box, roadmap edges near them turn red, and the planned path
jumps to a different route.

---

## 1. Build

```bash
mkdir -p ~/spited_ws/src && cd ~/spited_ws/src
git clone --recursive https://github.com/sai197161/SPITE_D.git
cd ~/spited_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select spite_d
source install/setup.bash
```

If it complains that open-spite is missing, the `--recursive` didn't take:

```bash
cd ~/spited_ws/src/SPITE_D && git submodule update --init --recursive
```

Dependencies, if the build complains about missing headers:

```bash
sudo apt install libcgal-dev libompl-dev libeigen3-dev libfcl-dev \
                 nlohmann-json3-dev libopencv-dev
```

Sanity check before touching the robot — this needs no hardware:

```bash
ctest --test-dir build/spite_d        # expect 8/8 passing
```

## 2. Build a roadmap

This is a one-time offline step. It samples collision-free robot
positions in a 4 m × 4 m area and precomputes the geometry used for
certification.

```bash
./build/spite_d/build_roadmap --out ~/rm --grow 0.3 \
    --env-lo -2.0 --env-hi 2.0 \
    --z-lo 0.05 --z-hi 0.25 \
    --robot-half 0.14 0.15 0.10
```

- `--grow` is seconds spent sampling. 0.3 gives a few thousand vertices,
  which is plenty. Larger values make a denser graph and a slower demo.
- `--robot-half` is the robot's half-size in metres (TurtleBot3 Waffle is
  roughly 0.14 × 0.15 × 0.10). It only affects which samples are
  considered collision-free.
- `--z-lo/--z-hi` keep the graph in a thin slab just above the floor
  instead of filling a cube.

If there are fixed obstacles in the room you want the roadmap to avoid,
put them in a file, one axis-aligned box per line as
`xmin ymin zmin xmax ymax zmax`, and pass `--obstacles thatfile.txt`.

## 3. Find your depth topics

These vary by camera and driver, so check rather than assume:

```bash
ros2 topic list | grep -i depth
ros2 topic hz /camera/depth/image_raw     # or whatever yours is called
```

Note the image topic and its matching `camera_info` topic. You'll pass
both in below. Also check the encoding:

```bash
ros2 topic echo /camera/depth/image_raw --field encoding --once
```

`16UC1` means millimetres (use `depth_scale:=1000.0`, the default).
`32FC1` means metres and is converted automatically.

## 4. Place the camera

The roadmap is a 4 × 4 m square. The defaults assume the camera sits at
one edge of it, looking across:

```
        y
        ^      . . . . . . . .      roadmap slab, x,y in [-2, 2]
        |      . . . . . . . .
   cam  @----> . . . . . . . .
  (-2,0)   x   . . . . . . . .
```

So park the robot at one side of a roughly 4 m clear area, facing across
it. If you want to move it elsewhere, pass `camera_x`, `camera_y`,
`camera_z`, `camera_yaw` to match where it actually is — `camera_z` is
the camera's height off the floor, and `camera_yaw` is 0 when it faces
along +x.

## 5. Run it

```bash
ros2 launch spite_d turtlebot_demo.launch.py \
    roadmap_graph:=$HOME/rm/roadmap_graph.txt \
    roadmap_geoms:=$HOME/rm/roadmap_geoms.txt \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/depth/camera_info
```

RViz opens with the roadmap, tracked obstacles, and the planned path.
Then walk through the space in front of the robot.

---

## Checking it stage by stage

If something looks wrong, this narrows down *where*. Run each in a new
terminal with `source ~/spited_ws/install/setup.bash` first.

**Is depth arriving?**
```bash
ros2 topic hz /camera/depth/image_raw
```
Nothing → wrong topic name, or the camera driver isn't running.

**Is anything detected?**
```bash
ros2 topic echo /spite_d_perception/obstacles --once
```
Empty `obstacles: []` while someone is clearly standing there is the most
likely failure, and it's useful information — see tuning below.

**Are predictions being made?**
```bash
ros2 topic hz /spite_d_prediction/predictions
```

**Is the roadmap being certified?**
```bash
ros2 topic echo /spite_d_validity/path --once
```
The validity node also prints the roadmap size on startup.

**What's slow?** The terminal running `validity_node` is the one to
watch. Certification is the expensive part and we already know it's the
bottleneck — if the whole thing feels sluggish, that's expected and not
something you broke.

---

## Things that are likely to go wrong

**Nothing is detected.** The detector was written for simulated depth and
hasn't seen a real room before, so this is the most likely outcome and
the most useful thing to report. It histograms depth per image column and
looks for columns that agree, which finds upright objects against a
clear background. Real rooms have walls and furniture that also look like
upright objects. Worth trying:
- Stand 1.5–3 m away, clear of walls
- Check `min_dist`/`max_dist` cover your room's depth range
- `thresholdPoint` and `minLengthLine` control how strong a column has to
  be before it counts as an object — lower means more sensitive and more
  false positives

These aren't exposed as launch parameters yet. If detection is the
problem, say so and we'll add the knobs rather than you editing source.

**Obstacles appear in the wrong place.** Almost always the camera pose.
Check `camera_x/y/z/yaw` match where the robot actually is, and that
`camera_yaw` is 0 only if it genuinely faces +x.

**Track IDs change constantly.** Watch the number above each box in
RViz. If it jumps every second or two, the tracker is losing and
re-acquiring the same person. We know about this — it's on the list — but
how bad it is on real data is worth knowing.

**Everything is red.** Means the obstacle geometry is being inflated too
much or the roadmap sits in a place the person always occupies. Note
roughly what fraction of edges go red and we'll look at it.

**RViz is empty but topics are publishing.** Check the Fixed Frame is
`map` (top left of RViz).

---

## What's most useful to send back

Even a failure is a good result here — this is the first time any of it
has seen real hardware.

1. A rosbag, if you can — this is the single most valuable thing, because
   it lets everything be replayed offline without the robot:
   ```bash
   ros2 bag record /camera/depth/image_raw /camera/depth/camera_info \
                   /spite_d_perception/obstacles \
                   /spite_d_prediction/predictions \
                   /spite_d_validity/path
   ```
2. A screen recording of RViz, or just a photo of it
3. The terminal output from all three nodes
4. Roughly what you did — where you stood, how you walked, how far away
