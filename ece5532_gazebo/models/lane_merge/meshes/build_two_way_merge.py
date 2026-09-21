import bpy

# ============================================================
# Road parameters
# ============================================================
length = 50.0
width_start = 18.0    # wide end (Y=0)
width_end = 10.0       # narrow end (Y=length)
thickness = 0.1

# Lane marking parameters
LINE_WIDTH = 0.2
EDGE_OFFSET = 1.0         # distance from the road edge to each outer line's CENTER
NUM_DASH_REPEATS = 15
DASH_DUTY_CYCLE = 0.5     # fraction of each repeat unit that is painted - ADJUST if not 50/50

# ============================================================
# Material parameters (matched to one_way_two_lane.mtl / .png)
# ============================================================
ASPHALT_COLOR = (0.068, 0.068, 0.068, 1.0)    # matched to road segment asphalt tone in gz-sim - requires exporting WITH the PBR extension block (Pr/Pm/etc.) enabled, or Kd gets ignored and everything renders flat grey
LINE_COLOR = (1.0, 1.0, 1.0, 1.0)                       # outer lines - white
CENTER_LINE_COLOR = (1.0, 1.0, 0.0, 1.0)                # center line - sampled from two_way_two_lane.png, sRGB(255,255,0) -> linear
ROUGHNESS = 1.0    # from Ns 0 (fully matte)
SPECULAR = 0.5     # from Ks 0.5 0.5 0.5
IOR = 1.5          # from Ni 1.5

# ============================================================
# Derived geometry values
# ============================================================
hw_start = width_start / 2.0
hw_end = width_end / 2.0
hz = thickness / 2.0
half_line = LINE_WIDTH / 2.0
top_z = hz
bottom_z = -hz


def boundaries(hw):
    """The 8 X boundaries (left to right) of the 7 top-surface columns, for a given half-width."""
    edge_L = -hw
    lineL_outer = -(hw - EDGE_OFFSET) - half_line
    lineL_inner = -(hw - EDGE_OFFSET) + half_line
    centerL = -half_line
    centerR = half_line
    lineR_inner = (hw - EDGE_OFFSET) - half_line
    lineR_outer = (hw - EDGE_OFFSET) + half_line
    edge_R = hw
    return [edge_L, lineL_outer, lineL_inner, centerL, centerR, lineR_inner, lineR_outer, edge_R]


b0 = boundaries(hw_start)   # boundaries at Y=0
b1 = boundaries(hw_end)     # boundaries at Y=length

verts = []
faces = []
face_materials = []   # 0 = road/asphalt, 1 = white line - parallel to faces


def add_quad(v0, v1, v2, v3, material_index):
    idx = len(verts)
    verts.extend([v0, v1, v2, v3])
    faces.append((idx, idx + 1, idx + 2, idx + 3))
    face_materials.append(material_index)


# ============================================================
# Top surface - 6 static columns (left to right: margin, line,
# lane, [center handled separately], lane, line, margin)
# ============================================================
static_columns = [
    (0, 1, 0),  # left edge margin (asphalt)
    (1, 2, 1),  # left outer line
    (2, 3, 0),  # lane 1 (asphalt)
    (4, 5, 0),  # lane 2 (asphalt)
    (5, 6, 1),  # right outer line
    (6, 7, 0),  # right edge margin (asphalt)
]

for i0, i1, mat in static_columns:
    add_quad(
        (b0[i0], 0.0, top_z), (b0[i1], 0.0, top_z),
        (b1[i1], length, top_z), (b1[i0], length, top_z),
        mat,
    )

# Center dashed column - doesn't taper (pinned to X=0), subdivided along Y
unit_length = length / NUM_DASH_REPEATS
dash_length = unit_length * DASH_DUTY_CYCLE
cL, cR = b0[3], b0[4]  # == b1[3], b1[4] - center column boundaries are constant

for i in range(NUM_DASH_REPEATS):
    y_start = i * unit_length
    y_dash_end = y_start + dash_length
    y_end = (i + 1) * unit_length
    add_quad((cL, y_start, top_z), (cR, y_start, top_z),
             (cR, y_dash_end, top_z), (cL, y_dash_end, top_z), 2)          # painted dash (center = yellow, slot 2)
    if y_dash_end < y_end:
        add_quad((cL, y_dash_end, top_z), (cR, y_dash_end, top_z),
                 (cR, y_end, top_z), (cL, y_end, top_z), 0)                # gap (asphalt)

# ============================================================
# Bottom, end caps, and sides - plain road material
# ============================================================
add_quad((b0[0], 0.0, bottom_z), (b1[0], length, bottom_z),
         (b1[7], length, bottom_z), (b0[7], 0.0, bottom_z), 0)             # bottom

add_quad((b0[0], 0.0, bottom_z), (b0[7], 0.0, bottom_z),
         (b0[7], 0.0, top_z), (b0[0], 0.0, top_z), 0)                      # start end cap

add_quad((b1[7], length, bottom_z), (b1[0], length, bottom_z),
         (b1[0], length, top_z), (b1[7], length, top_z), 0)                # end end cap

add_quad((b0[0], 0.0, bottom_z), (b0[0], 0.0, top_z),
         (b1[0], length, top_z), (b1[0], length, bottom_z), 0)             # left side

add_quad((b0[7], 0.0, top_z), (b0[7], 0.0, bottom_z),
         (b1[7], length, bottom_z), (b1[7], length, top_z), 0)             # right side

# ============================================================
# Build the mesh/object
# ============================================================
mesh = bpy.data.meshes.new("two_way_merge")
mesh.from_pydata(verts, [], faces)
mesh.update()

for poly, mat_index in zip(mesh.polygons, face_materials):
    poly.material_index = mat_index

obj = bpy.data.objects.new("two_way_merge", mesh)
bpy.context.collection.objects.link(obj)

bpy.context.view_layer.objects.active = obj
obj.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.remove_doubles(threshold=0.0001)   # weld coincident verts between adjacent strips
bpy.ops.mesh.normals_make_consistent(inside=False)
bpy.ops.object.mode_set(mode='OBJECT')


# ============================================================
# Materials
# ============================================================
def configure_principled(mat, base_color):
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs['Base Color'].default_value = base_color
        bsdf.inputs['Roughness'].default_value = ROUGHNESS
        if 'Specular IOR Level' in bsdf.inputs:      # Blender 4.x naming
            bsdf.inputs['Specular IOR Level'].default_value = SPECULAR
        elif 'Specular' in bsdf.inputs:                # older Blender naming
            bsdf.inputs['Specular'].default_value = SPECULAR
        bsdf.inputs['IOR'].default_value = IOR
        bsdf.inputs['Alpha'].default_value = 1.0
    if hasattr(mat, "blend_method"):
        mat.blend_method = 'OPAQUE'
    return bsdf


road_mat = bpy.data.materials.get("road_texture.001")
if road_mat is None:
    road_mat = bpy.data.materials.new(name="road_texture.001")
bsdf = configure_principled(road_mat, ASPHALT_COLOR)

line_mat = bpy.data.materials.get("LaneMarkingWhite")
if line_mat is None:
    line_mat = bpy.data.materials.new(name="LaneMarkingWhite")
configure_principled(line_mat, LINE_COLOR)

center_line_mat = bpy.data.materials.get("CenterLineYellow")
if center_line_mat is None:
    center_line_mat = bpy.data.materials.new(name="CenterLineYellow")
configure_principled(center_line_mat, CENTER_LINE_COLOR)

obj.data.materials.append(road_mat)          # slot 0
obj.data.materials.append(line_mat)          # slot 1
obj.data.materials.append(center_line_mat)   # slot 2

print(f"Road segment built: {len(faces)} faces total, "
      f"{NUM_DASH_REPEATS} dashes of {dash_length:.4f}m each (duty cycle {DASH_DUTY_CYCLE}).")
print("Material slots:", [m.name for m in obj.data.materials])
