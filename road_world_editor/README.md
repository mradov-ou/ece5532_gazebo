# Road World Editor

An RViz panel plugin for building road networks out of prefabricated road
segments, then exporting them as a Gazebo world (`.sdf`) and/or a Lanelet2 map
(`.osm`).

## 1. Starting the Editor

Build and source the workspace, then launch RViz with the bundled config:

```bash
rviz2 -d $(ros2 pkg prefix road_world_editor)/share/road_world_editor/rviz/road_world_editor.rviz
```

The config sets the Fixed Frame to `map` and already includes everything the
editor needs:

| Item | Purpose |
|------|---------|
| RoadWorldEditor panel | The control panel, docked on the left |
| MarkerArray display (topic `/markers`) | Road meshes, ports and the selection arrow |
| InteractiveMarkers display (namespace `/interactive_markers`) | The green drag handles |

To add the panel to some other RViz config instead, use
**Panels → Add New Panel → road_world_editor/RoadWorldEditor**, and add the
MarkerArray and InteractiveMarkers displays above by hand.

## 2. What You See in the 3D View

| Marker | Meaning |
|--------|---------|
| Road mesh | A white preview of each road segment. |
| Green cube | The drag handle for a segment. Click and drag it to move the segment around the ground plane (use the **Interact** tool, which is the default). |
| Cyan sphere | An unconnected port: an end of a road segment that can join another segment. |
| Red sphere | A connected port. |
| White arrow | Points down at the currently selected segment. All of the panel's per-model controls act on this segment. |

The selected segment is the one most recently added, dragged, or moved by
typing in X/Y. There is no separate "select" click: to select a segment, grab
its green cube (even a tiny drag is enough).

## 3. The Panel, Top to Bottom

### Add Model + model type list

Pick a segment type from the drop-down, then click **Add Model**.
Available types:

| Type | Notes |
|------|-------|
| Intersection | 4-way, ports 20 m from the center in each direction |
| Straight 10m | |
| Straight 20m | |
| Straight 50m | |
| Straight 100m | |
| Curved 50m | 90-degree curve, two-lane variants only |
| Curved 100m | 90-degree curve |
| Lane Merge | 50 m long |

The first segment is placed at the origin facing east (+X). Later ones are
placed 50 m east and 50 m north of the selected segment, with the same
orientation, so they don't land on top of it.

### X / Y

Shows the position of the selected segment in meters. Type a new value and
press Enter (or click away) to move it precisely.

### Variant

The lane configuration of the selected segment. Disabled for the
Intersection, which has no variants.

| Segment | Variants |
|---------|----------|
| Straight (all) / Curved 100m | `two_way_two_lane` (default), `one_way_two_lane`, `two_way_four_lane`, `one_way_four_lane` |
| Curved 50m | `two_way_two_lane`, `one_way_two_lane` |
| Lane Merge | `two_way_merge`, `one_way_merge` |

Four-lane variants switch the preview to a four-lane mesh where one exists.
The variant is used in both exports.

### Rotate CCW / Rotate CW

Rotates the selected segment 90 degrees about its own origin. Rotating
disconnects all of that segment's ports.

### Disconnect

Breaks every connection on the selected segment so it can be dragged away on
its own.

### Delete Model

Disconnects and removes the selected segment. Nothing is selected afterward.
**There is no undo.**

### World Origin: Lat / Lon / Alt

The geographic coordinates of the `map` frame origin (default 45.0, -81.0,
0.0). Used by both exports and stored in saved files.

### Save / Load

**Save** writes the whole road network (segments, positions, orientations,
variants, connections, and the world origin) to a `.json` file.
**Load replaces the current scene** with the contents of a `.json` file, so
save first if you need what is on screen.

### Export World

Writes a Gazebo `.sdf` world. It starts from
`ece5532_gazebo/worlds/road_world_template.sdf`, fills in the world origin as
the spherical coordinates, and adds one static model per segment referencing
the models in the `ece5532_gazebo` package.

### Export Lanelet Map

Writes a Lanelet2 `.osm` map of every lane in the network, using the world
origin as the projection origin. Lane boundary points that coincide (within
1 mm) are merged so adjacent lanelets share them.

The file dialogs remember the last folder used (stored in
`~/.rviz2/road_world_editor_dialog_dir`). A missing extension is added
automatically.

## 4. Connecting Segments

Segments snap together automatically:

1. Drag a segment so one of its cyan ports is within 10 m of an unconnected
   port on another segment.
2. Let go. About a quarter second after the drag stops, the port snaps onto
   the other port and both turn red.

Once connected, segments move as a group: dragging any segment in a connected
chain drags every segment attached to it. To pull one segment out of a chain,
select it and click **Disconnect** (or rotate it), then drag it away.

Typing a new X/Y value also moves the whole connected group, but does not
trigger snapping.

> **Tip:** Line up the orientation with Rotate CW/CCW *before* dragging a
> segment toward its neighbor, because rotating breaks connections.

## 5. Typical Workflow

1. Set the World Origin latitude/longitude/altitude.
2. Add an Intersection (or any starting segment).
3. Add segments one at a time, rotate them as needed, and drag each one until
   it snaps onto the network.
4. Set variants on segments that need more lanes or one-way traffic.
5. Save to `.json` regularly. This is the only format that can be loaded back
   into the editor.
6. **Export World** to get the `.sdf` for Gazebo, and **Export Lanelet Map**
   to get the matching `.osm` map.

The exported world can be run with the `ece5532_gazebo` bringup launch file:

```bash
ros2 launch ece5532_gazebo gazebo_bringup.launch.py world_sdf_file:=/path/to/world.sdf
```

## 6. Extra: Getting Lat/Lon of a Point

Use the **2D Pose Estimate** tool in the RViz toolbar (publishes on
`/initialpose`) and click on the map. The editor converts the clicked point to
latitude/longitude using the World Origin and prints it to the terminal RViz
was started from, e.g.:

```
Clicked point: latitude=45.00012345, longitude=-80.99987654
```
