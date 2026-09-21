import bpy

# ============================================================
# This does NOT rebuild the intersection's geometry - it was
# hand-modeled (curb fillets, markers, stop lines) rather than
# parametric like the merge segments. Run this with
# intersection.blend open: it just walks the existing objects
# and re-points each one at one of three shared materials
# (asphalt / white line / yellow line), configured the same way
# build_two_way_merge.py / build_one_way_merge.py do, instead of
# the ~14 near-duplicate per-object materials Blender's exporter
# had accumulated (road_surface_color.002/.003/.004,
# road_marker_color/.001-.006, through_line_color/.001-.003).
# ============================================================

# ============================================================
# Material parameters (matched to the road segment models)
# ============================================================
ASPHALT_COLOR = (0.068, 0.068, 0.068, 1.0)    # matches ASPHALT_COLOR in build_two_way_merge.py / build_one_way_merge.py - keep these three in sync. Requires exporting WITH the PBR extension block (Pr/Pm/etc.) enabled, or Kd gets ignored and everything renders flat grey
LINE_COLOR = (1.0, 1.0, 1.0, 1.0)                 # stop lines & lane-approach markers - white
THROUGH_LINE_COLOR = (1.0, 1.0, 0.0, 1.0)         # through-lane guide lines - yellow, matches the two-way roads' CenterLineYellow
ROUGHNESS = 1.0    # from Ns 0 (fully matte)
SPECULAR = 0.5     # from Ks 0.5 0.5 0.5
IOR = 1.5          # from Ni 1.5

# Every road/merge material in this project leaves Metallic at 0 (non-metal,
# dielectric). The intersection's existing materials were exported with
# Pm 1.0 (fully metallic) - on a matte road surface that suppresses most of
# the diffuse response and reads as darker/flatter than the other segments
# regardless of Base Color, which is likely a second, independent cause of
# the mismatch. Force it back to 0 here.
METALLIC = 0.0

# ============================================================
# Which object gets which logical material.
# (road_edge1-4 = curb fillets, main_intersection = pavement fill,
#  road_marker1-4 = dashed lane-approach lines, stop_line1-4 = stop bars,
#  through_line1-4 = through-lane guide lines)
# ============================================================
ASPHALT_OBJECTS = [
    "road_edge1", "road_edge2", "road_edge3", "road_edge4", "main_intersection",
]
WHITE_LINE_OBJECTS = [
    "road_marker1", "road_marker2", "road_marker3", "road_marker4",
    "stop_line1", "stop_line2", "stop_line3", "stop_line4",
]
YELLOW_LINE_OBJECTS = [
    "through_line1", "through_line2", "through_line3", "through_line4",
]


def configure_principled(mat, base_color):
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs['Base Color'].default_value = base_color
        bsdf.inputs['Roughness'].default_value = ROUGHNESS
        bsdf.inputs['Metallic'].default_value = METALLIC
        if 'Specular IOR Level' in bsdf.inputs:      # Blender 4.x naming
            bsdf.inputs['Specular IOR Level'].default_value = SPECULAR
        elif 'Specular' in bsdf.inputs:                # older Blender naming
            bsdf.inputs['Specular'].default_value = SPECULAR
        bsdf.inputs['IOR'].default_value = IOR
        bsdf.inputs['Alpha'].default_value = 1.0
    if hasattr(mat, "blend_method"):
        mat.blend_method = 'OPAQUE'
    return bsdf


def get_or_make_material(name, color):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name=name)
    configure_principled(mat, color)
    return mat


road_mat = get_or_make_material("road_surface_color", ASPHALT_COLOR)
marker_mat = get_or_make_material("road_marker_color", LINE_COLOR)
through_mat = get_or_make_material("through_line_color", THROUGH_LINE_COLOR)

OBJECT_GROUPS = [
    (ASPHALT_OBJECTS, road_mat),
    (WHITE_LINE_OBJECTS, marker_mat),
    (YELLOW_LINE_OBJECTS, through_mat),
]

retargeted = 0
missing = []
for object_names, target_mat in OBJECT_GROUPS:
    for name in object_names:
        obj = bpy.data.objects.get(name)
        if obj is None:
            missing.append(name)
            continue
        obj.data.materials.clear()
        obj.data.materials.append(target_mat)
        for poly in obj.data.polygons:
            poly.material_index = 0
        retargeted += 1

# Drop the now-unused per-object duplicate materials left over from the
# original export (road_surface_color.002, road_marker_color.004, etc.)
for mat in list(bpy.data.materials):
    if mat.users == 0:
        bpy.data.materials.remove(mat)

print(f"Retargeted {retargeted} objects onto 3 shared materials.")
if missing:
    print(f"WARNING: {len(missing)} expected object(s) not found in scene: {missing}")
print("Material slots now in file:", sorted(m.name for m in bpy.data.materials))
