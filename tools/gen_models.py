# Procedural model generator for the PVO simulator.
# Run: /Applications/Blender.app/Contents/MacOS/Blender -b --python tools/gen_models.py
# Convention: meters, forward = +X, up = +Z (Blender). glTF export converts to Y-up.
import bpy, bmesh, math, os, sys
from mathutils import Vector, Matrix

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")
os.makedirs(OUT, exist_ok=True)


def lin(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


_mats = {}


def mat(name, rgb, rough=0.7, metal=0.0, emit=None):
    if name in _mats:
        return _mats[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (lin(rgb[0]), lin(rgb[1]), lin(rgb[2]), 1)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    _mats[name] = m
    return m


M = {
    "olive": mat("olive", (86, 94, 62)),
    "olive_dk": mat("olive_dk", (60, 66, 44)),
    "camo": mat("camo", (104, 100, 72)),
    "rubber": mat("rubber", (28, 28, 28), 0.9),
    "steel": mat("steel", (90, 92, 95), 0.4, 0.8),
    "glass": mat("glass", (20, 30, 38), 0.1),
    "white": mat("white", (214, 214, 206), 0.5),
    "grey": mat("grey", (150, 152, 150), 0.6),
    "grey_dk": mat("grey_dk", (70, 72, 74), 0.6),
    "red": mat("red", (150, 40, 32)),
    "panel": mat("panel", (48, 52, 46), 0.5),
    "concrete": mat("concrete", (168, 162, 150), 0.9),
    "concrete_dk": mat("concrete_dk", (120, 116, 108), 0.9),
    "brick": mat("brick", (140, 82, 64), 0.9),
    "roof": mat("roof", (72, 74, 78), 0.8),
    "window": mat("window", (40, 52, 64), 0.2),
    "tank_white": mat("tank_white", (196, 196, 188), 0.6),
    "rust": mat("rust", (122, 70, 44), 0.9),
    "bark": mat("bark", (46, 42, 38), 0.9),
    "pine": mat("pine", (34, 44, 36), 0.9),
    "leaf": mat("leaf", (70, 100, 48), 0.9),
    "drone": mat("drone", (122, 126, 120), 0.6),
    "drone_dk": mat("drone_dk", (54, 56, 56), 0.6),
    "black": mat("black", (18, 18, 18), 0.5),
}


def clear():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for me in list(bpy.data.meshes):
        bpy.data.meshes.remove(me)


def _fin(o, m, bevel=0.0, smooth=False):
    o.data.materials.clear()
    o.data.materials.append(m)
    if bevel > 0:
        mod = o.modifiers.new("bv", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        mod.limit_method = "ANGLE"
    if smooth:
        for p in o.data.polygons:
            p.use_smooth = True
    return o


def box(size, loc, m, bevel=0.03, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.scale = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return _fin(o, m, bevel)


def cyl(r, depth, loc, m, rot=(0, 0, 0), verts=24, bevel=0.0, smooth=True, r2=None):
    if r2 is None:
        bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=depth, location=loc, rotation=rot)
    else:
        bpy.ops.mesh.primitive_cone_add(vertices=verts, radius1=r, radius2=r2, depth=depth, location=loc, rotation=rot)
    o = bpy.context.active_object
    _fin(o, m, bevel, smooth)
    # keep caps flat-shaded: split by angle via edge marking
    o.data.update()
    return o


def cone(r1, r2, depth, loc, m, rot=(0, 0, 0), verts=24, smooth=True):
    return cyl(r1, depth, loc, m, rot, verts, 0.0, smooth, r2=r2)


def sphere(r, loc, m, scale=(1, 1, 1), sub=2):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=sub, radius=r, location=loc)
    o = bpy.context.active_object
    o.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return _fin(o, m, 0, True)


def poly_prism(pts2d, z0, z1, m, plane="XY", bevel=0.0):
    """Extrude a 2D polygon. plane XY: pts=(x,y) extruded along z. XZ: pts=(x,z) extruded along y."""
    bm = bmesh.new()
    bot, top = [], []
    for (a, b) in pts2d:
        if plane == "XY":
            bot.append(bm.verts.new((a, b, z0)))
            top.append(bm.verts.new((a, b, z1)))
        else:
            bot.append(bm.verts.new((a, z0, b)))
            top.append(bm.verts.new((a, z1, b)))
    n = len(pts2d)
    bm.faces.new(bot[::-1])
    bm.faces.new(top)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((bot[i], bot[j], top[j], top[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new("prism")
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new("prism", me)
    bpy.context.collection.objects.link(o)
    return _fin(o, m, bevel)


def lathe(profile, m, verts=32, loc=(0, 0, 0)):
    """profile: list of (radius, z). Revolve around Z."""
    bm = bmesh.new()
    rings = []
    for (r, z) in profile:
        ring = []
        for i in range(verts):
            a = 2 * math.pi * i / verts
            ring.append(bm.verts.new((loc[0] + r * math.cos(a), loc[1] + r * math.sin(a), loc[2] + z)))
        rings.append(ring)
    for k in range(len(rings) - 1):
        for i in range(verts):
            j = (i + 1) % verts
            bm.faces.new((rings[k][i], rings[k][j], rings[k + 1][j], rings[k + 1][i]))
    me = bpy.data.meshes.new("lathe")
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new("lathe", me)
    bpy.context.collection.objects.link(o)
    return _fin(o, m, 0, True)


def _join_all():
    objs = [o for o in bpy.data.objects if o.type == "MESH"]
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.convert(target="MESH")  # applies bevel modifiers
    if len(objs) > 1:
        bpy.ops.object.join()
    o = bpy.context.view_layer.objects.active
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return o


def _densify(o, spacing):
    """Split long edges so per-vertex AO has enough samples."""
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    for _ in range(7):
        long = [e for e in bm.edges if e.calc_length() > spacing]
        if not long:
            break
        bmesh.ops.subdivide_edges(bm, edges=long, cuts=1, use_grid_fill=True)
        bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 3])
    bm.to_mesh(o.data)
    bm.free()


def _bake_ao(o, ground, dist, mud_h, mud_rgb, floor_ao):
    scn = bpy.context.scene
    scn.render.engine = "CYCLES"
    scn.cycles.samples = 64
    scn.cycles.device = "CPU"
    if scn.world is None:
        scn.world = bpy.data.worlds.new("w")
    scn.world.light_settings.distance = dist
    plane = None
    if ground:
        zmin = min(v.co.z for v in o.data.vertices)
        bpy.ops.mesh.primitive_plane_add(size=400, location=(0, 0, max(zmin, -0.5)))
        plane = bpy.context.active_object
    attr = o.data.color_attributes.new("ao", "BYTE_COLOR", "CORNER")
    o.data.color_attributes.active_color = attr
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.bake(type="AO", target="VERTEX_COLORS")
    if plane:
        bpy.data.objects.remove(plane, do_unlink=True)
    # remap AO, add mud on the lower parts; the game shader does pow(c, 2.2)
    me = o.data
    for li, loop in enumerate(me.loops):
        a = attr.data[li].color[0]
        a = floor_ao + (1 - floor_ao) * a
        z = me.vertices[loop.vertex_index].co.z
        m = 0.0
        if mud_h > 0:
            t = max(0.0, min(1.0, 1 - z / mud_h))
            m = t * t * (3 - 2 * t)
        rgb = [a * (1 - m + m * mud_rgb[k]) for k in range(3)]
        attr.data[li].color = (rgb[0] ** (1 / 2.2), rgb[1] ** (1 / 2.2), rgb[2] ** (1 / 2.2), 1)


def export(name, spacing=0.3, ao=True, ground=True, dist=2.5, mud_h=0.0, mud_rgb=(0.62, 0.55, 0.45), floor_ao=0.3, smooth_angle=38):
    o = _join_all()
    if ao:
        _densify(o, spacing)
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(smooth_angle))
    if ao:
        _bake_ao(o, ground, dist, mud_h, mud_rgb, floor_ao)
    path = os.path.join(OUT, name + ".glb")
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.ops.export_scene.gltf(
        filepath=path, export_format="GLB", use_selection=True, export_apply=True,
        export_yup=True, export_normals=True, export_texcoords=False, export_materials="EXPORT",
        export_vertex_color="ACTIVE" if ao else "NONE",
    )
    print("EXPORTED", path, len(o.data.polygons), "tris", len(o.data.vertices), "verts")


# ---------------------------------------------------------------- helpers
def along_x(o, x=0.0, y=0.0, z=0.0):
    """Lathe objects revolve around Z; turn them to point along +X."""
    o.rotation_euler = (0, math.pi / 2, 0)
    o.location = (x, y, z)
    return o


def rod(a, b, r, m, verts=10):
    """Cylinder between two points."""
    a, b = Vector(a), Vector(b)
    d = b - a
    rot = d.to_track_quat("Z", "Y").to_euler()
    return cyl(r, d.length, (a + b) / 2, m, rot=rot, verts=verts)


def bolts_ring(cx, y, cz, r, n, m, size=0.035, side=1):
    for k in range(n):
        a = 2 * math.pi * k / n
        cyl(size, 0.05, (cx + r * math.cos(a), y + side * 0.02, cz + r * math.sin(a)), m, rot=(math.pi / 2, 0, 0), verts=6, smooth=False)


# ---------------------------------------------------------------- vehicles
def wheel(x, y, z, r, w, side):
    tire = cyl(r, w, (x, y, z), M["rubber"], rot=(math.pi / 2, 0, 0), verts=28, bevel=0.07)
    tire.modifiers["bv"].segments = 3
    # tread lugs
    for k in range(20):
        a = 2 * math.pi * k / 20
        box((0.1, w * 0.92, 0.05), (x + (r + 0.01) * math.cos(a), y, z + (r + 0.01) * math.sin(a)), M["rubber"], bevel=0.01, rot=(0, -a + math.pi / 2, 0))
    cyl(r * 0.6, w + 0.03, (x, y, z), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=24, bevel=0.02)
    cone(r * 0.32, r * 0.18, 0.16, (x, y + side * (w / 2 + 0.08), z), M["olive"], rot=(-side * math.pi / 2, 0, 0), verts=16)
    bolts_ring(x, y + side * (w / 2 + 0.02), z, r * 0.42, 8, M["steel"], side=side)


def truck(deck=True):
    # 8x8 heavy chassis, length ~11 m
    for x in [-3.7, -2.2, 1.6, 3.1]:
        for s in (-1, 1):
            wheel(x, s * 1.25, 0.62, 0.62, 0.5, s)
    # frame rails, axle housings
    for s in (-1, 1):
        box((10.6, 0.22, 0.4), (-0.1, s * 0.55, 0.85), M["olive_dk"], bevel=0.02)
    for x in [-3.7, -2.2, 1.6, 3.1]:
        cyl(0.12, 2.1, (x, 0, 0.62), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=10)
        sphere(0.22, (x, 0, 0.62), M["olive_dk"], sub=1)
    box((10.4, 2.3, 0.45), (-0.1, 0, 1.17), M["olive_dk"], bevel=0.04)
    # fenders with mud flaps
    for x in (-2.95, 2.35):
        for s in (-1, 1):
            box((2.6, 0.58, 0.1), (x, s * 1.25, 1.44), M["olive"], bevel=0.03)
            box((0.06, 0.55, 0.45), (x - 1.32, s * 1.25, 1.18), M["rubber"], bevel=0.01)
    # cab with sloped front
    poly_prism([(3.6, 1.4), (5.55, 1.4), (5.8, 2.35), (5.45, 3.35), (3.6, 3.35)], -1.3, 1.3, M["olive"], plane="XZ", bevel=0.06)
    box((0.06, 2.2, 0.72), (5.53, 0, 2.85), M["glass"], bevel=0.0, rot=(0, math.radians(-20), 0))
    box((0.04, 0.06, 0.75), (5.55, 0, 2.85), M["olive_dk"], bevel=0.0, rot=(0, math.radians(-20), 0))  # pillar
    box((0.25, 2.5, 0.06), (5.6, 0, 3.3), M["olive_dk"], bevel=0.01)  # sun visor
    for s in (-1, 1):
        box((1.1, 0.04, 0.6), (4.5, s * 1.31, 2.9), M["glass"], bevel=0.0)
        box((0.9, 0.03, 0.9), (4.0, s * 1.31, 2.15), M["olive_dk"], bevel=0.01)  # door seam panel
        box((0.12, 0.05, 0.03), (4.3, s * 1.34, 2.3), M["steel"], bevel=0.0)  # handle
        rod((5.2, s * 1.3, 2.9), (5.35, s * 1.62, 3.0), 0.025, M["steel"])  # mirror arm
        box((0.05, 0.22, 0.38), (5.35, s * 1.66, 2.9), M["black"], bevel=0.01)
        box((0.5, 0.3, 0.05), (3.9, s * 1.35, 1.6), M["steel"], bevel=0.01)  # step
        cyl(0.13, 0.08, (5.82, s * 0.85, 2.0), M["glass"], rot=(0, math.pi / 2, 0), verts=14)  # headlight
        cyl(0.15, 0.06, (5.79, s * 0.85, 2.0), M["olive_dk"], rot=(0, math.pi / 2, 0), verts=14)
    # grille
    for k in range(7):
        box((0.04, 0.06, 0.55), (5.86, -0.45 + k * 0.15, 1.95), M["black"], bevel=0.0)
    box((0.18, 2.3, 0.32), (5.85, 0, 1.6), M["olive_dk"], bevel=0.03)  # bumper
    for s in (-1, 1):
        box((0.12, 0.2, 0.12), (6.0, s * 0.7, 1.6), M["red"], bevel=0.02)  # tow hooks
    box((1.2, 1.6, 0.12), (4.4, 0, 3.4), M["olive_dk"], bevel=0.02)  # roof plate
    for s in (-1, 1):
        cyl(0.07, 0.08, (5.0, s * 0.5, 3.47), M["glass"], verts=10)  # roof lights
    # exhaust stack behind the cab
    rod((3.45, 1.0, 1.6), (3.45, 1.0, 3.7), 0.08, M["black"])
    cyl(0.12, 0.6, (3.45, 1.0, 2.6), M["steel"], verts=12)
    # fuel tanks between axles
    for s in (-1, 1):
        cyl(0.28, 1.5, (-0.2, s * 1.0, 1.0), M["olive"], rot=(0, math.pi / 2, 0), verts=16, bevel=0.03)
        for x in (-0.75, 0.35):
            cyl(0.29, 0.05, (x, s * 1.0, 1.0), M["black"], rot=(0, math.pi / 2, 0), verts=16)
    if deck:
        box((6.6, 2.55, 0.25), (-1.1, 0, 1.55), M["olive"], bevel=0.03)
        for k in range(12):
            box((0.05, 2.56, 0.03), (-4.2 + k * 0.55, 0, 1.68), M["olive_dk"], bevel=0.0)  # anti-slip ribs
        # stowed outrigger jacks
        for x in (-4.0, 1.8):
            for s in (-1, 1):
                box((0.3, 0.3, 0.9), (x, s * 1.35, 1.1), M["olive_dk"], bevel=0.03)
                cyl(0.1, 0.5, (x, s * 1.35, 0.5), M["steel"], verts=10)
                box((0.45, 0.45, 0.06), (x, s * 1.35, 0.26), M["olive_dk"], bevel=0.01)
        # equipment boxes, jerrycans
        box((1.0, 2.4, 0.9), (3.0, 0, 2.05), M["camo"], bevel=0.04)
        for s in (-1, 1):
            box((0.9, 0.02, 0.06), (3.0, s * 1.21, 2.35), M["steel"], bevel=0.0)  # latches
        box((0.8, 0.6, 0.5), (-4.0, 0.9, 1.95), M["camo"], bevel=0.03)
        for k in range(3):
            box((0.16, 0.34, 0.46), (-4.25 + k * 0.2, -0.95, 1.93), M["olive_dk"], bevel=0.02)


def launcher():
    clear()
    truck()
    export("launcher_base", spacing=0.55, mud_h=1.3)
    clear()
    # turret: origin = yaw pivot
    cyl(1.25, 0.3, (0, 0, 0.15), M["olive_dk"], verts=40, bevel=0.03)
    for k in range(24):
        a = 2 * math.pi * k / 24
        cyl(0.04, 0.06, (1.15 * math.cos(a), 1.15 * math.sin(a), 0.32), M["steel"], verts=6, smooth=False)
    box((3.2, 2.2, 0.35), (-0.4, 0, 0.45), M["olive"], bevel=0.05)
    box((0.9, 0.7, 0.6), (0.9, 0.6, 0.9), M["camo"], bevel=0.04)  # electronics box
    box((0.02, 0.5, 0.4), (1.36, 0.6, 0.9), M["olive_dk"], bevel=0.0)
    el = math.radians(50)
    d = Vector((math.cos(el), 0, math.sin(el)))
    n = Vector((-math.sin(el), 0, math.cos(el)))
    hinge = Vector((-1.6, 0, 0.75))
    L = 6.2
    rot = (0, math.pi / 2 - el, 0)
    for a in (-1, 1):
        for b in (-1, 1):
            c = hinge + d * (L / 2) + Vector((0, a * 0.42, 0)) + n * (b * 0.42)
            cyl(0.38, L, c, M["camo"], rot=rot, verts=28, bevel=0.03)
            cap = hinge + d * (L + 0.04) + Vector((0, a * 0.42, 0)) + n * (b * 0.42)
            cyl(0.34, 0.1, cap, M["olive_dk"], rot=rot, verts=24, bevel=0.02)
            cyl(0.2, 0.12, cap + d * 0.05, M["olive_dk"], rot=rot, verts=16)
            back = hinge - d * 0.04 + Vector((0, a * 0.42, 0)) + n * (b * 0.42)
            cyl(0.36, 0.1, back, M["olive_dk"], rot=rot, verts=24)
            for t in (0.12, 0.5, 0.88):
                rc = hinge + d * (L * t) + Vector((0, a * 0.42, 0)) + n * (b * 0.42)
                cyl(0.405, 0.14, rc, M["olive_dk"], rot=rot, verts=28, bevel=0.01)
    # cable conduit along the pack
    rod(hinge + Vector((0, 0, 0.05)), hinge + d * L * 0.9 + Vector((0, 0, 0.0)), 0.07, M["black"])
    # cradle and elevation rams
    box((0.3, 2.0, 0.3), (-1.6, 0, 0.75), M["steel"], bevel=0.03)
    for s in (-1, 1):
        a_ = Vector((0.6, s * 0.9, 0.6))
        b_ = hinge + d * 3.6 - n * 0.7 + Vector((0, s * 0.9, 0))
        mid = a_ + (b_ - a_) * 0.45
        rod(a_, mid, 0.13, M["olive_dk"], verts=14)
        rod(mid, b_, 0.07, M["steel"], verts=12)
        sphere(0.15, a_, M["olive_dk"], sub=1)
    export("launcher_turret", spacing=0.55)
    print("TUBE_HINGE", hinge, "LEN", L)


def radar():
    clear()
    truck(deck=False)
    box((6.4, 2.5, 1.8), (-1.2, 0, 2.3), M["camo"], bevel=0.06)  # shelter
    for i in range(5):
        box((0.04, 2.52, 1.4), (-4.0 + i * 1.4, 0, 2.35), M["olive_dk"], bevel=0)
    box((0.9, 0.04, 1.4), (-2.0, -1.27, 2.2), M["olive_dk"], bevel=0.01)  # door
    box((0.08, 0.05, 0.2), (-1.65, -1.31, 2.2), M["steel"], bevel=0.0)
    for k in range(5):
        box((0.5, 0.06, 0.04), (-2.0, -1.45, 1.35 + k * 0.22), M["steel"], bevel=0.0)  # ladder rungs
    for s in (-1, 1):
        rod((-2.25 + s * 0.25 + 0.25, -1.45, 1.3), (-2.25 + s * 0.25 + 0.25, -1.45, 2.3), 0.025, M["steel"], verts=6)
    box((0.7, 0.04, 0.6), (-3.6, 1.27, 2.4), M["grey_dk"], bevel=0.01)  # AC unit
    for k in range(6):
        box((0.6, 0.03, 0.03), (-3.6, 1.3, 2.17 + k * 0.09), M["black"], bevel=0.0)
    cyl(0.42, 0.3, (-1.2, 0, 3.3), M["olive_dk"], verts=20, bevel=0.02)
    cyl(0.32, 1.0, (-1.2, 0, 3.9), M["olive_dk"], verts=20)
    cyl(0.36, 0.08, (-1.2, 0, 4.35), M["steel"], verts=20)
    export("radar_base", spacing=0.55, mud_h=1.3)
    clear()
    # head: origin at rotation axis (top of mast)
    cyl(0.6, 0.35, (0, 0, 0.18), M["olive_dk"], verts=28, bevel=0.03)
    box((0.6, 0.6, 0.5), (0, 0, 0.55), M["olive"], bevel=0.04)
    tilt = math.radians(-20)
    R = Matrix.Rotation(tilt, 3, "Y")

    def at(p):
        p = Vector(p)
        p.rotate(R)
        return (0.25 + p.x, p.y, 1.9 + p.z)

    box((0.35, 4.2, 2.6), (0.25, 0, 1.9), M["olive"], rot=(0, tilt, 0), bevel=0.04)
    # rim frame
    for zz in (-1.32, 1.32):
        box((0.42, 4.3, 0.1), at((0, 0, zz)), M["olive_dk"], rot=(0, tilt, 0), bevel=0.02)
    for yy in (-2.12, 2.12):
        box((0.42, 0.1, 2.7), at((0, yy, 0)), M["olive_dk"], rot=(0, tilt, 0), bevel=0.02)
    # tile grid on the array face
    for iy in range(6):
        for iz in range(4):
            yy = -1.75 + iy * 0.7
            zz = -0.95 + iz * 0.63
            box((0.04, 0.62, 0.55), at((0.19, yy, zz)), M["panel"], bevel=0.01, rot=(0, tilt, 0))
    # back truss
    for s in (-1, 1):
        box((0.2, 0.12, 1.6), (-0.15, s * 1.0, 1.2), M["steel"], rot=(0, tilt, 0), bevel=0.01)
        rod((0, s * 0.25, 0.75), at((-0.2, s * 1.8, 1.0)), 0.04, M["steel"], verts=8)
        rod((0, s * 0.25, 0.75), at((-0.2, s * 1.8, -1.0)), 0.04, M["steel"], verts=8)
    box((0.3, 1.2, 0.5), at((-0.35, 0, 0)), M["camo"], rot=(0, tilt, 0), bevel=0.03)  # transceiver box
    export("radar_head", spacing=0.3)


def track_loop(y, w, m_link):
    """Track links around sprocket (front), idler (rear) and road wheels."""
    pts = []
    xs, xe = -3.0, 2.9
    top, bot = 1.0, 0.05
    rf = 0.42
    n_top = 30
    for i in range(n_top + 1):
        pts.append((xe - (xe - xs) * i / n_top, top, math.pi))
    for i in range(1, 10):
        a = math.pi / 2 + math.pi * i / 10
        pts.append((xs + rf * math.cos(a), (top + bot) / 2 + (top - bot) / 2 * math.sin(a), 0))
    for i in range(n_top + 1):
        pts.append((xs + (xe - xs) * i / n_top, bot, 0))
    for i in range(1, 10):
        a = -math.pi / 2 + math.pi * i / 10
        pts.append((xe + rf * math.cos(a), (top + bot) / 2 + (top - bot) / 2 * math.sin(a), 0))
    for k in range(len(pts)):
        p0 = Vector((pts[k][0], 0, pts[k][1]))
        p1 = Vector((pts[(k + 1) % len(pts)][0], 0, pts[(k + 1) % len(pts)][1]))
        mid = (p0 + p1) / 2
        dv = p1 - p0
        ang = math.atan2(dv.z, dv.x)
        box((dv.length * 0.92, w, 0.07), (mid.x, y, mid.z), m_link, bevel=0.012, rot=(0, -ang, 0))
        if k % 2 == 0:
            box((0.05, w * 0.3, 0.1), (mid.x, y, mid.z), M["steel"], bevel=0.0, rot=(0, -ang, 0))  # guide horn


def spaag():
    clear()
    # tracked hull
    poly_prism([(-3.4, 0.55), (2.6, 0.55), (3.5, 1.15), (3.2, 1.75), (-3.4, 1.75)], -1.2, 1.2, M["olive"], plane="XZ", bevel=0.06)
    for s in (-1, 1):
        y = s * 1.55
        track_loop(y, 0.52, M["rubber"])
        for i in range(6):
            x = -2.55 + i * 1.03
            cyl(0.33, 0.42, (x, y, 0.4), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=24, bevel=0.03)
            cyl(0.335, 0.08, (x, y, 0.4), M["rubber"], rot=(math.pi / 2, 0, 0), verts=24)
            cone(0.13, 0.07, 0.1, (x, y + s * 0.25, 0.4), M["olive"], rot=(-s * math.pi / 2, 0, 0), verts=12)
            box((0.9, 0.12, 0.08), (x + 0.3, s * 1.25, 0.55), M["olive_dk"], bevel=0.01, rot=(0, math.radians(15), 0))  # torsion arm
        # drive sprocket (front) with teeth, idler (rear), return rollers
        cyl(0.36, 0.4, (2.9, y, 0.52), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=24, bevel=0.03)
        for k in range(12):
            a = 2 * math.pi * k / 12
            box((0.1, 0.36, 0.1), (2.9 + 0.39 * math.cos(a), y, 0.52 + 0.39 * math.sin(a)), M["steel"], bevel=0.01, rot=(0, -a, 0))
        cyl(0.36, 0.4, (-3.0, y, 0.52), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=24, bevel=0.03)
        for x in (-1.6, 0.0, 1.6):
            cyl(0.11, 0.3, (x, y, 0.9), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=12)
        # side skirt with bolts
        box((6.9, 0.06, 0.5), (0, s * 1.84, 0.88), M["olive"], bevel=0.02)
        for k in range(10):
            cyl(0.03, 0.03, (-3.1 + k * 0.68, s * 1.875, 1.05), M["steel"], rot=(math.pi / 2, 0, 0), verts=6, smooth=False)
        box((6.8, 0.65, 0.07), (0, y, 1.12), M["olive"], bevel=0.02)  # fender
        # stowage boxes on fenders
        box((1.1, 0.5, 0.4), (-1.9, y, 1.36), M["camo"], bevel=0.04)
        box((0.8, 0.5, 0.35), (1.6, y, 1.33), M["camo"], bevel=0.04)
        # headlights, tow hooks
        cyl(0.1, 0.12, (3.25, s * 0.9, 1.55), M["glass"], rot=(0, math.pi / 2, 0), verts=12)
        box((0.15, 0.15, 0.12), (3.5, s * 0.7, 0.95), M["red"], bevel=0.02)
        # exhaust
        box((0.5, 0.3, 0.25), (-3.25, s * 0.75, 1.5), M["black"], bevel=0.03)
    # engine deck grille
    for k in range(9):
        box((0.06, 1.6, 0.04), (-3.0 + k * 0.18, 0, 1.77), M["black"], bevel=0.0)
    box((1.0, 0.8, 0.12), (2.6, 0.6, 1.62), M["olive_dk"], bevel=0.03)  # driver hatch
    cyl(0.03, 1.8, (-2.8, -0.9, 2.65), M["black"], verts=6)  # antenna
    export("spaag_base", spacing=0.55, mud_h=1.2)
    clear()
    # turret (origin = yaw pivot on hull roof)
    poly_prism([(-1.6, 0.0), (1.2, 0.0), (1.6, 0.4), (1.3, 1.1), (-1.6, 1.1)], -1.15, 1.15, M["olive"], plane="XZ", bevel=0.06)
    cyl(1.0, 0.12, (0, 0, 0.02), M["olive_dk"], verts=32)
    # hatches and periscopes
    for (x, y) in ((-0.6, 0.45), (-0.6, -0.45)):
        cyl(0.34, 0.1, (x, y, 1.15), M["olive_dk"], verts=20, bevel=0.02)
        cyl(0.08, 0.05, (x + 0.15, y, 1.22), M["steel"], verts=8)
        box((0.15, 0.2, 0.14), (x + 0.38, y, 1.17), M["glass"], bevel=0.01)
    # smoke dischargers
    for s in (-1, 1):
        for k in range(3):
            rod((0.9, s * 1.1, 0.75 + k * 0.13), (1.2, s * 1.3, 0.85 + k * 0.13), 0.055, M["olive_dk"], verts=10)
        box((1.6, 0.25, 0.5), (-0.6, s * 1.27, 0.55), M["camo"], bevel=0.04)  # bustle box
        for k in range(4):
            box((0.03, 0.04, 0.45), (-1.3 + k * 0.45, s * 1.4, 0.55), M["olive_dk"], bevel=0.0)
    # search radar on rear mast: frame + mesh panel
    cyl(0.08, 0.7, (-1.2, 0, 1.45), M["steel"], verts=10)
    cyl(0.16, 0.12, (-1.2, 0, 1.8), M["olive_dk"], verts=14)
    box((0.1, 1.6, 0.5), (-1.2, 0, 2.05), M["olive_dk"], bevel=0.02)
    for k in range(7):
        box((0.12, 0.03, 0.46), (-1.2, -0.72 + k * 0.24, 2.05), M["steel"], bevel=0.0)
    # tracking radar dish (front)
    dish = lathe([(0.0, 0.0), (0.2, 0.03), (0.35, 0.09), (0.45, 0.17)], M["olive_dk"], verts=28)
    along_x(dish, 1.38, 0, 0.75)
    cyl(0.04, 0.3, (1.6, 0, 0.75), M["steel"], rot=(0, math.pi / 2, 0), verts=8)
    box((0.3, 0.6, 0.5), (1.25, 0, 0.75), M["olive"], bevel=0.04)
    export("spaag_turret", spacing=0.3)
    clear()
    # guns (origin = trunnion), barrels along +X
    for s in (-1, 1):
        y = s * 1.42
        box((1.6, 0.42, 0.5), (0.3, y, 0), M["olive"], bevel=0.05)
        for k in range(3):
            box((0.03, 0.43, 0.3), (-0.2 + k * 0.4, y, 0.05), M["olive_dk"], bevel=0.0)
        cyl(0.13, 0.8, (1.45, y, 0.05), M["olive_dk"], rot=(0, math.pi / 2, 0), verts=16, bevel=0.02)  # recoil sleeve
        cyl(0.075, 2.6, (2.9, y, 0.05), M["steel"], rot=(0, math.pi / 2, 0), verts=14)
        cyl(0.09, 0.12, (2.2, y, 0.05), M["olive_dk"], rot=(0, math.pi / 2, 0), verts=14)
        # muzzle brake with ports
        cyl(0.1, 0.4, (4.05, y, 0.05), M["black"], rot=(0, math.pi / 2, 0), verts=14, bevel=0.01)
        for k in range(3):
            cyl(0.105, 0.04, (3.92 + k * 0.12, y, 0.05), M["steel"], rot=(0, math.pi / 2, 0), verts=14)
        # ammo feed chute
        rod((-0.3, y - s * 0.21, 0.1), (-0.6, y - s * 0.6, -0.3), 0.08, M["olive_dk"], verts=8)
    cyl(0.15, 2.8, (0.2, 0, 0), M["olive_dk"], rot=(math.pi / 2, 0, 0), verts=20)
    export("spaag_guns", spacing=0.3, ground=False)


# ---------------------------------------------------------------- airborne
def sam_missile():
    clear()
    body = lathe([(0.0, -2.75), (0.15, -2.75), (0.17, -2.6), (0.2, -2.45), (0.2, 1.4), (0.18, 1.8), (0.14, 2.2), (0.08, 2.6), (0.02, 2.8), (0.0, 2.82)], M["white"], verts=24)
    along_x(body)
    noz = lathe([(0.0, -2.6), (0.12, -2.6), (0.17, -2.85), (0.15, -2.88), (0.1, -2.7), (0.0, -2.7)], M["black"], verts=20)
    along_x(noz)
    cyl(0.205, 0.2, (1.1, 0, 0), M["grey_dk"], rot=(0, math.pi / 2, 0), verts=24)
    cyl(0.205, 0.15, (-1.7, 0, 0), M["red"], rot=(0, math.pi / 2, 0), verts=24)
    cyl(0.203, 0.03, (-0.4, 0, 0), M["grey"], rot=(0, math.pi / 2, 0), verts=24)
    for k in range(4):
        a = k * math.pi / 2 + math.pi / 4
        fin = poly_prism([(-2.55, 0.18), (-1.75, 0.18), (-2.3, 0.65), (-2.6, 0.65)], -0.015, 0.015, M["grey"], plane="XZ", bevel=0.006)
        fin.rotation_euler = (a, 0, 0)
        can = poly_prism([(1.3, 0.18), (1.65, 0.18), (1.35, 0.36)], -0.01, 0.01, M["grey"], plane="XZ")
        can.rotation_euler = (a, 0, 0)
    box((1.5, 0.05, 0.04), (0.2, 0, 0.2), M["grey"], bevel=0.01)  # cable tunnel
    export("sam_missile", spacing=0.4, ground=False, dist=0.6, floor_ao=0.5)


def wing_shape(pts, th, m, z=0.0, bevel=0.0):
    return poly_prism(pts, z - th / 2, z + th / 2, m, plane="XY", bevel=bevel)


def drone_delta():
    clear()
    # body ~3.5 m, span ~2.5 m, delta wing, pusher prop
    body = lathe([(0.0, -1.62), (0.1, -1.6), (0.14, -1.45), (0.19, -1.0), (0.2, 0.6), (0.19, 1.1), (0.15, 1.45), (0.09, 1.7), (0.03, 1.82), (0.0, 1.84)], M["drone"], verts=20)
    along_x(body)
    cyl(0.04, 0.12, (1.86, 0, 0), M["black"], rot=(0, math.pi / 2, 0), verts=8)  # seeker/fuse tip
    for x in (-0.5, 0.9):
        cyl(0.202, 0.015, (x, 0, 0), M["drone_dk"], rot=(0, math.pi / 2, 0), verts=20)  # panel joints
    wing_shape([(1.0, 0.0), (-1.4, 1.25), (-1.75, 1.25), (-1.6, 0.0), (-1.75, -1.25), (-1.4, -1.25)], 0.07, M["drone"], bevel=0.025)
    for s in (-1, 1):
        # winglets with an airfoil-like section
        poly_prism([(-1.42, -0.02), (-1.78, -0.02), (-1.88, 0.45), (-1.66, 0.45)], s * 1.25 - 0.025, s * 1.25 + 0.025, M["drone_dk"], plane="XZ", bevel=0.01)
        box((0.35, 0.03, 0.015), (-0.6, s * 0.75, -0.04), M["drone_dk"], bevel=0.0, rot=(0, 0, math.atan2(1.25, 2.4)))  # aileron line
    # engine + pusher prop
    cyl(0.16, 0.3, (-1.6, 0, 0.0), M["drone_dk"], rot=(0, math.pi / 2, 0), verts=16, bevel=0.01)
    for k in range(4):
        cyl(0.17, 0.02, (-1.5 - k * 0.06, 0, 0), M["black"], rot=(0, math.pi / 2, 0), verts=16)  # cooling fins
    cone(0.07, 0.02, 0.12, (-1.82, 0, 0), M["black"], rot=(0, -math.pi / 2, 0), verts=10)  # spinner
    for s in (-1, 1):
        box((0.02, 0.62, 0.075), (-1.8, s * 0.33, 0), M["black"], bevel=0.008, rot=(s * math.radians(12), 0, 0))
    rod((-1.4, 0.08, -0.12), (-1.7, 0.1, -0.2), 0.02, M["black"], verts=6)  # exhaust
    cyl(0.008, 0.25, (0.3, 0, 0.32), M["black"], verts=6)  # antenna
    export("drone_delta", spacing=0.6, ground=False, dist=0.5, floor_ao=0.5)


def cruise_missile():
    clear()
    body = lathe([(0.0, -3.05), (0.22, -3.05), (0.29, -2.9), (0.3, -2.5), (0.3, 1.9), (0.28, 2.3), (0.21, 2.7), (0.1, 3.0), (0.0, 3.08)], M["grey"], verts=24)
    along_x(body)
    for x in (-1.5, 0.0, 1.5):
        cyl(0.302, 0.02, (x, 0, 0), M["grey_dk"], rot=(0, math.pi / 2, 0), verts=24)
    cyl(0.2, 0.08, (-3.06, 0, 0), M["black"], rot=(0, math.pi / 2, 0), verts=16)
    wing_shape([(0.35, 0.0), (-0.1, 1.35), (-0.4, 1.35), (-0.25, 0.0), (-0.4, -1.35), (-0.1, -1.35)], 0.05, M["grey"], z=0.18, bevel=0.015)
    for k in range(3):
        a = [math.pi / 2, math.pi / 2 + 2.1, math.pi / 2 - 2.1][k]
        f = poly_prism([(-2.8, 0.25), (-2.2, 0.25), (-2.75, 0.75), (-2.95, 0.75)], -0.015, 0.015, M["grey_dk"], plane="XZ", bevel=0.005)
        f.rotation_euler = (a - math.pi / 2, 0, 0)
    box((0.8, 0.3, 0.2), (-1.8, 0, -0.32), M["grey_dk"], bevel=0.05)  # intake
    box((0.05, 0.26, 0.16), (-1.42, 0, -0.32), M["black"], bevel=0.0)
    export("cruise_missile", spacing=0.4, ground=False, dist=0.6, floor_ao=0.5)


# ---------------------------------------------------------------- scenery
def tree_pine():
    clear()
    cyl(0.18, 3.0, (0, 0, 1.5), M["bark"], verts=8)
    for i, (r, z, h) in enumerate([(2.4, 3.0, 4.0), (1.9, 5.2, 3.6), (1.3, 7.3, 3.0), (0.7, 9.0, 2.2)]):
        cone(r, 0.0, h, (0, 0, z + h / 2 - 0.6), M["pine"], verts=9, smooth=False)
    export("tree_pine", ao=False)


def branch(p0, d, length, r, depth, rng):
    p1 = p0 + d * length
    mid = (p0 + p1) / 2
    rot = d.to_track_quat("Z", "Y").to_euler()
    cone(r, r * 0.6, length, mid, M["bark"], rot=rot, verts=6, smooth=True)
    if depth == 0:
        return
    n = 2 if depth > 1 else 3
    for k in range(n):
        a = rng.uniform(0, 2 * math.pi)
        tilt = rng.uniform(0.45, 0.9)
        nd = (d * math.cos(tilt) + Vector((math.cos(a), math.sin(a), 0)) * math.sin(tilt)).normalized()
        nd.z = max(nd.z, 0.15)
        nd.normalize()
        start = p0 + d * length * rng.uniform(0.55, 0.95)
        branch(start, nd, length * rng.uniform(0.5, 0.7), r * 0.55, depth - 1, rng)


def tree_leaf():
    # bare late-autumn tree
    import random
    clear()
    rng = random.Random(7)
    branch(Vector((0, 0, -0.3)), Vector((0.05, 0.02, 1)).normalized(), 5.0, 0.32, 3, rng)
    export("tree_leaf", ao=False)


def power_plant():
    clear()
    # cooling tower (hyperboloid)
    prof = []
    for i in range(13):
        t = i / 12
        z = t * 42
        r = 16 - 9.5 * math.sin(t * math.pi * 0.8) + (t ** 3) * 3.5
        prof.append((r, z))
    lathe(prof, M["concrete"], verts=40, loc=(-20, 18, 0))
    lathe([(r * 0.93, z) for (r, z) in prof][::-1], M["concrete_dk"], verts=40, loc=(-20, 18, 0))
    # turbine hall
    box((44, 20, 16), (12, -6, 8), M["brick"], bevel=0.1)
    box((44.5, 20.5, 0.8), (12, -6, 16.2), M["roof"], bevel=0.05)
    for i in range(10):
        box((2.2, 0.2, 9), (-6 + i * 4.2, -16.05, 8), M["window"], bevel=0)
    # boiler house
    box((16, 16, 30), (-12, -8, 15), M["concrete_dk"], bevel=0.1)
    # chimneys
    for x in (0, 10):
        cone(2.6, 1.6, 70, (x, 12, 35), M["concrete"], verts=20)
        cyl(1.8, 3, (x, 12, 62), M["red"], verts=20)
        cyl(1.65, 3, (x, 12, 52), M["white"], verts=20)
    # switchyard
    for i in range(4):
        box((4, 3, 3.5), (34 + 0 * i, -22 + i * 8, 1.75), M["grey_dk"])
        cyl(0.3, 2.0, (34, -22 + i * 8, 4.4), M["grey"], verts=8)
    export("power_plant", spacing=5.0, dist=8, floor_ao=0.35)


def oil_depot():
    clear()
    for i, (x, y, r, h) in enumerate([(-18, -12, 9, 11), (2, -12, 9, 11), (22, -12, 9, 11), (-12, 12, 6.5, 9), (5, 12, 6.5, 9), (22, 12, 5, 8)]):
        cyl(r, h, (x, y, h / 2), M["tank_white"], verts=40)
        cone(r, r * 0.15, 1.3, (x, y, h + 0.65), M["grey"], verts=40)
        cyl(r + 0.05, 0.5, (x, y, h * 0.3), M["rust"], verts=40)
    # berm walls
    box((62, 1.0, 1.6), (2, -25, 0.8), M["concrete_dk"])
    box((62, 1.0, 1.6), (2, 25, 0.8), M["concrete_dk"])
    box((1.0, 50, 1.6), (-29, 0, 0.8), M["concrete_dk"])
    box((1.0, 50, 1.6), (33, 0, 0.8), M["concrete_dk"])
    # pipe rack
    for k in range(3):
        cyl(0.35, 58, (2, 0.6 * k - 0.6, 3.2), M["steel"], rot=(0, math.pi / 2, 0), verts=10)
    for x in range(-25, 31, 8):
        box((0.3, 2.2, 3.2), (x, 0, 1.6), M["rust"])
    export("oil_depot", spacing=5.0, dist=8, floor_ao=0.35)


def apartment():
    clear()
    W, D, H = 60, 13, 27
    box((W, D, H), (0, 0, H / 2), M["concrete"], bevel=0.08)
    for f in range(9):
        z = 1.8 + f * 3.0
        for s in (-1, 1):
            box((W - 2, 0.15, 1.35), (0, s * (D / 2 + 0.02), z), M["window"], bevel=0)
        box((W + 0.2, D + 0.2, 0.25), (0, 0, z + 1.2), M["concrete_dk"], bevel=0)
    box((W + 0.6, D + 0.6, 0.6), (0, 0, H + 0.3), M["roof"])
    for x in (-20, 0, 20):
        box((4, 3, 2.5), (x, 0, H + 1.6), M["concrete_dk"])
    export("apartment", spacing=5.0, dist=8, floor_ao=0.35)


def house():
    clear()
    box((9, 7, 3.2), (0, 0, 1.6), M["brick"], bevel=0.05)
    poly_prism([(-4.9, 3.1), (4.9, 3.1), (0, 5.6)], -3.9, 3.9, M["roof"], plane="XZ")
    box((0.6, 0.6, 1.4), (2.5, 1.5, 5.0), M["brick"])  # chimney
    box((1.2, 0.1, 1.1), (2, -3.52, 1.8), M["window"], bevel=0)
    box((1.2, 0.1, 1.1), (-2, -3.52, 1.8), M["window"], bevel=0)
    export("house", spacing=1.6, dist=3, floor_ao=0.35)


def contact_sheet():
    """Import all GLBs in a row and render a workbench preview."""
    clear()
    names = ["launcher_base", "launcher_turret", "radar_base", "radar_head", "spaag_base", "spaag_turret", "spaag_guns",
             "sam_missile", "drone_delta", "cruise_missile", "tree_pine", "tree_leaf", "house"]
    x = 0
    for nm in names:
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=os.path.join(OUT, nm + ".glb"))
        new = [o for o in bpy.data.objects if o not in before]
        for o in new:
            if o.parent is None:
                o.location.x += x
        x += 12
    scn = bpy.context.scene
    try:
        scn.render.engine = "BLENDER_WORKBENCH"
    except TypeError as e:
        print(e)
    scn.display.shading.light = "STUDIO"
    scn.display.shading.color_type = "MATERIAL"
    scn.display.shading.show_shadows = True
    scn.display.shading.show_cavity = True
    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    scn.collection.objects.link(cam)
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = x + 4
    cam.location = (x / 2 - 6, -60, 40)
    cam.rotation_euler = (math.radians(60), 0, 0)
    scn.camera = cam
    scn.render.resolution_x = 2400
    scn.render.resolution_y = 700
    scn.render.filepath = os.path.join(OUT, "..", "tools", "preview.png")
    bpy.ops.render.render(write_still=True)
    print("PREVIEW written")


launcher()
radar()
spaag()
sam_missile()
drone_delta()
cruise_missile()
tree_pine()
tree_leaf()
power_plant()
oil_depot()
apartment()
house()
contact_sheet()
