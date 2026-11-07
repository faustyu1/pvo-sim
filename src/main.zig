//! ПВО — 3D air-defence simulator. Zig 0.16 + raylib 6.
const std = @import("std");
const c = @import("c");
const vm = @import("vm.zig");
const sh = @import("shaders.zig");

const V3 = vm.V3;
const v3 = vm.v3;
const Xf = vm.Xf;

// ============================================================ constants
const TERR_N = 384;
const TERR_SIZE: f32 = 16000;
const CELL: f32 = TERR_SIZE / @as(f32, TERR_N);
const SHADOW_SIZE = 2048;
const SHADOW_SLOT = 14;
const G: f32 = 9.81;

const MAX_TGT = 96;
const MAX_MSL = 32;
const MAX_SHELL = 384;
const MAX_PART = 9000;
const MAX_TREES = 2200;
const MAX_PROPS = 96;

const RADAR_RANGE: f32 = 9000;
const GUN_RANGE: f32 = 3300;
const SHELL_SPEED: f32 = 1050;

// ============================================================ helpers
fn col(r: u8, gg: u8, b: u8, a: u8) c.Color {
    return .{ .r = r, .g = gg, .b = b, .a = a };
}
fn colf(r: f32, gg: f32, b: f32, a: f32) c.Color {
    return .{
        .r = @intFromFloat(vm.clampf(r, 0, 1) * 255),
        .g = @intFromFloat(vm.clampf(gg, 0, 1) * 255),
        .b = @intFromFloat(vm.clampf(b, 0, 1) * 255),
        .a = @intFromFloat(vm.clampf(a, 0, 1) * 255),
    };
}
const WHITE = col(255, 255, 255, 255);
const HUD = col(160, 255, 180, 255);
const HUD_DIM = col(110, 190, 130, 200);
const HUD_RED = col(255, 90, 70, 255);
const HUD_AMBER = col(255, 200, 80, 255);
const PANEL = col(6, 14, 10, 170);

fn matMul(l: c.Matrix, r: c.Matrix) c.Matrix {
    // raylib's Matrix stores fields as m0,m4,m8,m12,m1,... so go through names.
    const a = mArr(l);
    const b = mArr(r);
    var o: [16]f32 = undefined;
    for (0..4) |i| {
        for (0..4) |j| {
            var s: f32 = 0;
            for (0..4) |k| s += a[i * 4 + k] * b[k * 4 + j];
            o[i * 4 + j] = s;
        }
    }
    return .{
        .m0 = o[0], .m1 = o[1], .m2 = o[2], .m3 = o[3],
        .m4 = o[4], .m5 = o[5], .m6 = o[6], .m7 = o[7],
        .m8 = o[8], .m9 = o[9], .m10 = o[10], .m11 = o[11],
        .m12 = o[12], .m13 = o[13], .m14 = o[14], .m15 = o[15],
    };
}

fn mArr(m: c.Matrix) [16]f32 {
    return .{ m.m0, m.m1, m.m2, m.m3, m.m4, m.m5, m.m6, m.m7, m.m8, m.m9, m.m10, m.m11, m.m12, m.m13, m.m14, m.m15 };
}

fn copyZ(dst: []u8, src: [*c]const u8) void {
    var i: usize = 0;
    while (i + 1 < dst.len and src[i] != 0) : (i += 1) dst[i] = src[i];
    dst[i] = 0;
}

// ============================================================ terrain
var heights: [TERR_N + 1][TERR_N + 1]f32 = undefined;

fn terrainFn(x: f32, z: f32) f32 {
    const d = @sqrt(x * x + z * z);
    var h = (vm.fbm(x / 1900, z / 1900, 5, 1) - 0.5) * 150;
    h += (vm.fbm(x / 380, z / 380, 3, 7) - 0.5) * 18;
    h *= vm.lerpf(0.05, 1.0, vm.smooth(1400, 3400, d));
    const ridge = vm.fbm(x / 1400, z / 1400, 5, 3);
    h += vm.smooth(4300, 7700, d) * ridge * ridge * 1300;
    return h;
}

fn forestFn(x: f32, z: f32) f32 {
    const d = @sqrt(x * x + z * z);
    const f = vm.smooth(0.53, 0.62, vm.fbm(x / 800, z / 800, 4, 99));
    var ex: f32 = vm.smooth(220, 400, d);
    for (objects_static) |o| ex = @min(ex, vm.smooth(o.r + 60, o.r + 200, @sqrt((x - o.x) * (x - o.x) + (z - o.z) * (z - o.z))));
    return f * ex;
}

fn heightAt(x: f32, z: f32) f32 {
    const gx = vm.clampf((x + TERR_SIZE / 2) / CELL, 0, TERR_N - 0.001);
    const gz = vm.clampf((z + TERR_SIZE / 2) / CELL, 0, TERR_N - 0.001);
    const ix: usize = @intFromFloat(@floor(gx));
    const iz: usize = @intFromFloat(@floor(gz));
    const tx = gx - @floor(gx);
    const tz = gz - @floor(gz);
    const a = vm.lerpf(heights[iz][ix], heights[iz][ix + 1], tx);
    const b = vm.lerpf(heights[iz + 1][ix], heights[iz + 1][ix + 1], tx);
    return vm.lerpf(a, b, tz);
}

fn allocF(n: usize) [*]f32 {
    return @ptrCast(@alignCast(c.MemAlloc(@intCast(n * 4)).?));
}

fn buildTerrain() c.Model {
    const N = TERR_N;
    var nrm: [N + 1][N + 1]V3 = undefined;
    var forest: [N + 1][N + 1]u8 = undefined;
    var bigN: [N + 1][N + 1]u8 = undefined;
    var fieldN: [N + 1][N + 1]u8 = undefined;
    for (0..N + 1) |j| {
        for (0..N + 1) |i| {
            const x = -TERR_SIZE / 2 + @as(f32, @floatFromInt(i)) * CELL;
            const z = -TERR_SIZE / 2 + @as(f32, @floatFromInt(j)) * CELL;
            heights[j][i] = terrainFn(x, z);
            forest[j][i] = @intFromFloat(forestFn(x, z) * 255);
            // low-frequency shading noise baked per vertex (saves fbm in the fragment shader)
            bigN[j][i] = @intFromFloat(vm.clampf(vm.fbm(x * 0.0025, z * 0.0025, 3, 11) * 1.4 - 0.2, 0, 1) * 255);
            fieldN[j][i] = @intFromFloat(vm.clampf(vm.fbm(x * 0.0004 + 3.1, z * 0.0004, 3, 23), 0, 1) * 255);
        }
    }
    for (0..N + 1) |j| {
        for (0..N + 1) |i| {
            const il = if (i > 0) i - 1 else i;
            const ir = if (i < N) i + 1 else i;
            const jl = if (j > 0) j - 1 else j;
            const jr = if (j < N) j + 1 else j;
            nrm[j][i] = vm.norm(v3(heights[j][il] - heights[j][ir], 2 * CELL, heights[jl][i] - heights[jr][i]));
        }
    }
    const vcount: usize = N * N * 6;
    var mesh = std.mem.zeroes(c.Mesh);
    mesh.vertexCount = @intCast(vcount);
    mesh.triangleCount = @intCast(N * N * 2);
    const vs = allocF(vcount * 3);
    const ns = allocF(vcount * 3);
    const cs: [*]u8 = @ptrCast(c.MemAlloc(@intCast(vcount * 4)).?);
    var k: usize = 0;
    const order = [6][2]usize{ .{ 0, 0 }, .{ 0, 1 }, .{ 1, 0 }, .{ 1, 0 }, .{ 0, 1 }, .{ 1, 1 } };
    for (0..N) |j| {
        for (0..N) |i| {
            for (order) |o| {
                const ii = i + o[0];
                const jj = j + o[1];
                vs[k * 3 + 0] = -TERR_SIZE / 2 + @as(f32, @floatFromInt(ii)) * CELL;
                vs[k * 3 + 1] = heights[jj][ii];
                vs[k * 3 + 2] = -TERR_SIZE / 2 + @as(f32, @floatFromInt(jj)) * CELL;
                ns[k * 3 + 0] = nrm[jj][ii].x;
                ns[k * 3 + 1] = nrm[jj][ii].y;
                ns[k * 3 + 2] = nrm[jj][ii].z;
                cs[k * 4 + 0] = forest[jj][ii];
                cs[k * 4 + 1] = bigN[jj][ii];
                cs[k * 4 + 2] = fieldN[jj][ii];
                cs[k * 4 + 3] = 255;
                k += 1;
            }
        }
    }
    mesh.vertices = vs;
    mesh.normals = ns;
    mesh.colors = cs;
    c.UploadMesh(&mesh, false);
    return c.LoadModelFromMesh(mesh);
}

// ============================================================ world objects
const StaticObj = struct { x: f32, z: f32, r: f32 };
// defended objects: power plant, oil depot, city block
const objects_static = [_]StaticObj{
    .{ .x = -1450, .z = -950, .r = 90 },
    .{ .x = 1350, .z = 750, .r = 70 },
    .{ .x = 950, .z = -1150, .r = 110 },
    .{ .x = 0, .z = 0, .r = 70 },
    .{ .x = -650, .z = 1450, .r = 160 },
    .{ .x = 2300, .z = -350, .r = 120 },
};

const Obj = struct {
    name: [*:0]const u8,
    pos: V3,
    hp: f32 = 100,
    radius: f32,
    fires: [6]V3 = undefined,
    nfires: usize = 0,
    fireT: f32 = 0,
};

const PropKind = enum { power, depot, apartment, house };
const Prop = struct { kind: PropKind, pos: V3, yaw: f32, s: f32 = 1 };

const Tree = struct { pos: V3, yaw: f32, s: f32, pine: bool };

// ============================================================ dynamic entities
const TKind = enum { drone, cruise, decoy };

const Target = struct {
    alive: bool = false,
    id: u32 = 0,
    kind: TKind = .drone,
    pos: V3 = vm.zero,
    vel: V3 = vm.zero,
    speed: f32 = 50,
    hp: f32 = 3,
    goal: usize = 0,
    wps: [4]V3 = undefined,
    nwp: usize = 0,
    wi: usize = 0,
    alt: f32 = 200,
    roll: f32 = 0,
    diving: bool = false,
    dying: bool = false,
    detected: bool = false,
    seenPos: V3 = vm.zero,
    seenT: f32 = -100,
    classified: bool = false,
    fxT: f32 = 0,
    player: bool = false,
    trackT: f32 = 0, // continuous tracking time, fire control needs a stable track
};

const Missile = struct {
    alive: bool = false,
    pos: V3 = vm.zero,
    vel: V3 = vm.zero,
    t: f32 = 0,
    tgt: i32 = -1,
    tgtId: u32 = 0,
    lostT: f32 = 0,
    trailT: f32 = 0,
    id: u32 = 0,
};

const Shell = struct {
    alive: bool = false,
    pos: V3 = vm.zero,
    vel: V3 = vm.zero,
    fuse: f32 = 0,
    t: f32 = 0,
};

const Part = struct {
    alive: bool = false,
    pos: V3 = vm.zero,
    vel: V3 = vm.zero,
    life: f32 = 0,
    max: f32 = 1,
    s0: f32 = 1,
    s1: f32 = 1,
    c0: [4]f32 = .{ 1, 1, 1, 1 },
    c1: [4]f32 = .{ 1, 1, 1, 0 },
    add: bool = false,
    drag: f32 = 0,
    grav: f32 = 0,
};

const Launcher = struct {
    pos: V3,
    yaw: f32,
    tyaw: f32 = 0, // turret yaw relative to hull
    tubes: [4]bool = .{ true, true, true, true },
    reloadT: f32 = 0,
};

const Msg = struct { buf: [128]u8 = undefined, t: f32 = 0, color: c.Color = HUD };

const Game = struct {
    time: f32 = 0,
    wave: i32 = 0,
    waveT: f32 = 8,
    toSpawn: i32 = 0,
    spawnT: f32 = 0,
    waveBearing: f32 = 0,
    score: i32 = 0,
    kills: i32 = 0,
    leaks: i32 = 0,
    samFired: i32 = 0,
    nextId: u32 = 1,
    selected: i32 = -1,
    selId: u32 = 0,
    over: bool = false,
    paused: bool = false,
    warp: f32 = 1,

    launchers: [2]Launcher = undefined,
    reserve: i32 = 16,
    queue: [6]u32 = undefined,
    nqueue: usize = 0,
    autoSam: bool = false,
    autoT: f32 = 0,
    lastMissile: i32 = -1,

    gYaw: f32 = 0,
    gPitch: f32 = 0.2,
    gAmmo: i32 = 600,
    gReload: f32 = 0,
    gFireT: f32 = 0,
    gBurst: i32 = 0,
    gPause: f32 = 0,
    gBarrel: u1 = 0,
    gAuto: bool = true,
    gTgt: i32 = -1,

    // manual gunner seat (defence): the player aims and fires the SPAAG
    gunner: bool = false,
    gAimYaw: f32 = 0, // commanded, relative to the hull
    gAimPitch: f32 = 0.15,
    gFov: f32 = 40,
    gTrigger: bool = false,
    gRange: f32 = 1500, // fuse range from the laser rangefinder
    gLock: i32 = -1, // target nearest the boresight

    rYaw: f32 = 0,

    camMode: u8 = 0,
    camYaw: f32 = 2.3,
    camPitch: f32 = 0.32,
    camDist: f32 = 140,
    camFocus: V3 = v3(0, 10, 0),
    camPos: V3 = v3(100, 50, 100),
    camLook: V3 = vm.zero,

    msgs: [7]Msg = [_]Msg{.{}} ** 7,

    // attack mode
    player: i32 = -1,
    playerId: u32 = 0,
    pYaw: f32 = 0,
    pPitch: f32 = 0,
    pRoll: f32 = 0,
    pManeuver: f32 = 0,
    boost: f32 = 1,
    sortie: i32 = 0,
    strikes: i32 = 0,
    atkGoal: usize = 0,
    playerDead: bool = false,
    endT: f32 = 0,
    endPos: V3 = vm.zero,
    success: bool = false,
};

// ============================================================ app / menus / saves
const AppState = enum { menu, settings, records, briefing, playing, paused, result };
const Mode = enum { defense, attack };
var app: AppState = .menu;
var appPrev: AppState = .menu;
var mode: Mode = .defense;
var menuSel: usize = 0;
var quit = false;

const Settings = extern struct {
    rain: u8 = 1,
    shadows: u8 = 1,
    sens: u8 = 2,
    invertY: u8 = 0,
    hidpi: u8 = 0,
    diff: u8 = 1, // 0 easy, 1 normal, 2 hard
    pad: [2]u8 = .{ 0, 0 },
};
const Records = extern struct {
    defWave: i32 = 0,
    defScore: i32 = 0,
    atkStrikes: i32 = 0,
    atkScore: i32 = 0,
};
const SaveGame = extern struct {
    valid: u8 = 0,
    mode: u8 = 0,
    wave: i32 = 0,
    score: i32 = 0,
    kills: i32 = 0,
    leaks: i32 = 0,
    hp: [3]f32 = .{ 100, 100, 100 },
    reserve: i32 = 16,
    sortie: i32 = 0,
    strikes: i32 = 0,
};
const SaveFile = extern struct {
    magic: u32 = SAVE_MAGIC,
    settings: Settings = .{},
    records: Records = .{},
    game: SaveGame = .{},
};
const SAVE_MAGIC: u32 = 0x31564F50;
const SAVE_PATH = "pvo_save.dat";
var save: SaveFile = .{};

/// Pick a balance value by difficulty: easy / normal / hard.
fn diffK(e: f32, n: f32, h: f32) f32 {
    return switch (save.settings.diff) {
        0 => e,
        2 => h,
        else => n,
    };
}

fn loadSave() void {
    var size: c_int = 0;
    if (!c.FileExists(SAVE_PATH)) return;
    const data = c.LoadFileData(SAVE_PATH, &size);
    if (data == null) return;
    defer c.UnloadFileData(data);
    if (size != @sizeOf(SaveFile)) return;
    var s: SaveFile = undefined;
    @memcpy(std.mem.asBytes(&s), data[0..@sizeOf(SaveFile)]);
    if (s.magic == SAVE_MAGIC) save = s;
}

fn writeSave() void {
    _ = c.SaveFileData(SAVE_PATH, &save, @sizeOf(SaveFile));
}

fn storeGame() void {
    save.game = .{
        .valid = 1,
        .mode = @intFromEnum(mode),
        .wave = g.wave,
        .score = g.score,
        .kills = g.kills,
        .leaks = g.leaks,
        .hp = .{ objs[0].hp, objs[1].hp, objs[2].hp },
        .reserve = g.reserve,
        .sortie = g.sortie,
        .strikes = g.strikes,
    };
    writeSave();
}

fn updateRecords() void {
    if (mode == .defense) {
        save.records.defWave = @max(save.records.defWave, g.wave);
        save.records.defScore = @max(save.records.defScore, g.score);
    } else {
        save.records.atkStrikes = @max(save.records.atkStrikes, g.strikes);
        save.records.atkScore = @max(save.records.atkScore, g.score);
    }
    writeSave();
}

// attack-mode targets: power plant, oil depot, the SAM battery itself
const AtkGoal = struct { name: [*:0]const u8, pos: V3, r: f32 };
fn atkGoal(i: usize) AtkGoal {
    return switch (i) {
        0 => .{ .name = "ТЭЦ", .pos = objs[0].pos, .r = 75 },
        1 => .{ .name = "НЕФТЕБАЗА", .pos = objs[1].pos, .r = 55 },
        else => .{ .name = "ПОЗИЦИЯ ЗРК", .pos = ground(-10, 5), .r = 50 },
    };
}

// ============================================================ global state
var g: Game = .{};
var targets: [MAX_TGT]Target = [_]Target{.{}} ** MAX_TGT;
var missiles: [MAX_MSL]Missile = [_]Missile{.{}} ** MAX_MSL;
var shells: [MAX_SHELL]Shell = [_]Shell{.{}} ** MAX_SHELL;
var parts: [MAX_PART]Part = [_]Part{.{}} ** MAX_PART;
var partHead: usize = 0;
var trees: [MAX_TREES]Tree = undefined;
var ntrees: usize = 0;
var props: [MAX_PROPS]Prop = undefined;
var nprops: usize = 0;
var objs: [3]Obj = undefined;

var wind = v3(3.0, 0, 1.2);

const Models = struct {
    launcher_base: c.Model,
    launcher_turret: c.Model,
    radar_base: c.Model,
    radar_head: c.Model,
    spaag_base: c.Model,
    spaag_turret: c.Model,
    spaag_guns: c.Model,
    sam: c.Model,
    drone: c.Model,
    cruise: c.Model,
    pine: c.Model,
    leaf: c.Model,
    power: c.Model,
    depot: c.Model,
    apartment: c.Model,
    house: c.Model,
    terrain: c.Model,
    sky: c.Model,
};
var M: Models = undefined;

var lit: c.Shader = undefined;
var sky: c.Shader = undefined;
const Locs = struct {
    sunDir: c_int,
    sunColor: c_int,
    skyColor: c_int,
    groundColor: c_int,
    fogColor: c_int,
    viewPos: c_int,
    fogDensity: c_int,
    lightVP: c_int,
    shadowMap: c_int,
    shadowTexel: c_int,
    emissive: c_int,
    isTerrain: c_int,
    roads: c_int,
    roadCount: c_int,
};
var L: Locs = undefined;
var skyTime: c_int = 0;
var skyView: c_int = 0;
var shadowRT: c.RenderTexture2D = undefined;
var puff: c.Texture2D = undefined;
var font: c.Font = undefined;
var fontOk = false;
var debugShadow = false;
var renderTime: f32 = 0;
var terrainDebug = false;

// lighting
const sunDir = blk: {
    const d = v3(-0.62, 0.40, -0.42);
    const l = @sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    break :blk v3(d.x / l, d.y / l, d.z / l);
};
// overcast late autumn: weak diffuse sun, strong grey-blue sky fill, dense fog
const sunColor = [3]f32{ 0.75, 0.76, 0.80 };
const skyColor = [3]f32{ 0.46, 0.50, 0.56 };
const groundColor = [3]f32{ 0.12, 0.12, 0.11 };
const fogColor = [3]f32{ 0.36, 0.40, 0.44 };
const zenithColor = [3]f32{ 0.27, 0.30, 0.35 };
const FOG_DENSITY: f32 = 0.00045;
// display-space fog color for CPU-side particle fogging (same curve as shader finish())
var fogDisp = [3]f32{ 0.6, 0.64, 0.68 };

fn acesCpu(x: f32) f32 {
    return vm.clampf((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0, 1);
}
fn computeFogDisp() void {
    var cc: [3]f32 = undefined;
    for (0..3) |i| cc[i] = acesCpu(fogColor[i] * 0.85);
    const l = cc[0] * 0.299 + cc[1] * 0.587 + cc[2] * 0.114;
    const tint = [3]f32{ 0.95, 0.99, 1.05 };
    for (0..3) |i| fogDisp[i] = std.math.pow(f32, vm.lerpf(l, cc[i], 0.68) * tint[i], 1.0 / 2.2);
}

// drizzle: drops wrap in a box around the camera
fn drawRain(cam: c.Camera3D) void {
    c.rlDisableDepthMask();
    var i: u32 = 0;
    const box: f32 = 70;
    const fall: f32 = 11;
    while (i < 900) : (i += 1) {
        const hx = vm.hash2(@intCast(i), 1, 77);
        const hz = vm.hash2(@intCast(i), 2, 77);
        const hy = vm.hash2(@intCast(i), 3, 77);
        const sp = 0.8 + 0.4 * vm.hash2(@intCast(i), 4, 77);
        const x = cam.position.x + @mod(hx * box * 7 - cam.position.x + wind.x * renderTime * 0.4, box) - box / 2;
        const z = cam.position.z + @mod(hz * box * 7 - cam.position.z + wind.z * renderTime * 0.4, box) - box / 2;
        const y = cam.position.y + @mod(hy * box - renderTime * fall * sp - cam.position.y, box) - box / 2;
        const p = v3(x, y, z);
        const tail = vm.add(p, v3(-wind.x * 0.06, 0.9 * sp, -wind.z * 0.06));
        c.DrawLine3D(p, tail, col(185, 195, 205, 70));
    }
    c.rlEnableDepthMask();
}

// tube mouth offsets & direction in turret-local space (raylib Y-up)
var tubeMouth: [4]V3 = undefined;
const TUBE_EL: f32 = 50 * vm.deg;
const TURRET_ON_HULL = v3(-1.1, 1.675, 0);
const RADAR_ON_HULL = v3(-1.2, 4.7, 0);
const SPAAG_TURRET = v3(0, 1.75, 0);
const SPAAG_TRUNNION = v3(0.3, 0.75, 0);

var radarPos: V3 = undefined;
var radarYaw: f32 = 0.4;
var spaagPos: V3 = undefined;
var spaagYaw: f32 = -0.6;

// ============================================================ messages
fn msg(text: [*c]const u8, color: c.Color) void {
    var i: usize = g.msgs.len - 1;
    while (i > 0) : (i -= 1) g.msgs[i] = g.msgs[i - 1];
    copyZ(&g.msgs[0].buf, text);
    g.msgs[0].t = 7;
    g.msgs[0].color = color;
}

// ============================================================ particles
fn spawnPart(p: Part) void {
    var i: usize = 0;
    while (i < MAX_PART) : (i += 1) {
        partHead = (partHead + 1) % MAX_PART;
        if (!parts[partHead].alive) break;
    }
    parts[partHead] = p;
    parts[partHead].alive = true;
    parts[partHead].max = p.life;
}

fn smoke(pos: V3, vel: V3, life: f32, s0: f32, s1: f32, shade: f32, alpha: f32) void {
    spawnPart(.{ .pos = pos, .vel = vel, .life = life, .s0 = s0, .s1 = s1, .c0 = .{ shade, shade * 0.98, shade * 0.95, alpha }, .c1 = .{ shade * 1.05, shade * 1.04, shade * 1.02, 0 }, .drag = 0.6, .grav = -0.6 });
}

fn fire(pos: V3, vel: V3, life: f32, s0: f32, s1: f32, intensity: f32) void {
    spawnPart(.{ .pos = pos, .vel = vel, .life = life, .s0 = s0, .s1 = s1, .c0 = .{ 1.0 * intensity, 0.75 * intensity, 0.35 * intensity, 1 }, .c1 = .{ 0.6, 0.12, 0.02, 0 }, .add = true, .drag = 1.5, .grav = -2 });
}

fn explosion(pos: V3, big: f32, onGround: bool) void {
    // flash
    spawnPart(.{ .pos = pos, .life = 0.18, .s0 = 30 * big, .s1 = 55 * big, .c0 = .{ 1, 0.95, 0.8, 1 }, .c1 = .{ 1, 0.6, 0.2, 0 }, .add = true });
    var i: usize = 0;
    const nf: usize = @intFromFloat(14 * big);
    while (i < nf) : (i += 1) {
        const d = vm.rndDir();
        fire(vm.mad(pos, d, 2 * big), vm.scale(d, vm.rr(8, 30) * big), vm.rr(0.4, 1.1), 6 * big, vm.rr(10, 18) * big, 1.4);
    }
    i = 0;
    const ns: usize = @intFromFloat(18 * big);
    while (i < ns) : (i += 1) {
        var d = vm.rndDir();
        if (onGround and d.y < 0) d.y = -d.y;
        const shade: f32 = if (onGround) vm.rr(0.12, 0.25) else vm.rr(0.18, 0.32);
        smoke(vm.mad(pos, d, 3 * big), vm.scale(d, vm.rr(4, 14) * big), vm.rr(5, 10), 8 * big, vm.rr(26, 40) * big, shade, 0.85);
    }
    i = 0;
    while (i < @as(usize, @intFromFloat(24 * big))) : (i += 1) {
        const d = vm.rndDir();
        const up: f32 = if (onGround) 25 else 5;
        spawnPart(.{ .pos = pos, .vel = vm.add(vm.scale(d, vm.rr(20, 70)), v3(0, up, 0)), .life = vm.rr(0.8, 2.2), .s0 = 1.6, .s1 = 0.4, .c0 = .{ 1, 0.8, 0.4, 1 }, .c1 = .{ 1, 0.3, 0.05, 0 }, .add = true, .drag = 0.4, .grav = G });
    }
}

fn updateParts(dt: f32) void {
    for (&parts) |*p| {
        if (!p.alive) continue;
        p.life -= dt;
        if (p.life <= 0) {
            p.alive = false;
            continue;
        }
        const drag = @max(0, 1 - p.drag * dt);
        p.vel = vm.scale(p.vel, drag);
        p.vel.y -= p.grav * dt;
        if (!p.add) p.vel = vm.mad(p.vel, wind, dt * 0.25);
        p.pos = vm.mad(p.pos, p.vel, dt);
    }
}

fn fogFactor(p: V3, cam: V3) f32 {
    const d = vm.dist(p, cam);
    const h = @max(p.y, 0);
    return 1 - @exp(-d * FOG_DENSITY * (0.35 + 0.65 * @exp(-h * 0.0012)));
}

fn drawParts(cam: c.Camera3D) void {
    c.rlDisableDepthMask();
    // alpha pass then additive pass
    var pass: u8 = 0;
    while (pass < 2) : (pass += 1) {
        const additive = pass == 1;
        if (additive) c.BeginBlendMode(c.BLEND_ADDITIVE);
        for (&parts) |*p| {
            if (!p.alive or p.add != additive) continue;
            const t = 1 - p.life / p.max;
            const f = fogFactor(p.pos, cam.position);
            var cc: [4]f32 = undefined;
            for (0..4) |k| cc[k] = vm.lerpf(p.c0[k], p.c1[k], t);
            if (additive) {
                const k = 1 - f;
                cc = .{ cc[0] * k * cc[3], cc[1] * k * cc[3], cc[2] * k * cc[3], 1 };
            } else {
                // fake lighting: smoke lit by sun on top side
                cc[0] = vm.lerpf(cc[0] * 1.05, fogDisp[0], f);
                cc[1] = vm.lerpf(cc[1] * 1.0, fogDisp[1], f);
                cc[2] = vm.lerpf(cc[2] * 0.97, fogDisp[2], f);
                // soft fade-in
                cc[3] *= vm.smooth(0, 0.08, t);
            }
            const size = vm.lerpf(p.s0, p.s1, @sqrt(t));
            c.DrawBillboard(cam, puff, p.pos, size, colf(cc[0], cc[1], cc[2], cc[3]));
        }
        if (additive) c.EndBlendMode();
    }
    c.rlEnableDepthMask();
}

// ============================================================ setup
fn loadModel(path: [*c]const u8) c.Model {
    const m = c.LoadModel(path);
    var i: usize = 0;
    while (i < @as(usize, @intCast(m.materialCount))) : (i += 1) m.materials[i].shader = lit;
    return m;
}

fn initShaders() void {
    lit = c.LoadShaderFromMemory(sh.lit_vs, sh.lit_fs);
    sky = c.LoadShaderFromMemory(sh.sky_vs, sh.sky_fs);
    L = .{
        .sunDir = c.GetShaderLocation(lit, "sunDir"),
        .sunColor = c.GetShaderLocation(lit, "sunColor"),
        .skyColor = c.GetShaderLocation(lit, "skyColor"),
        .groundColor = c.GetShaderLocation(lit, "groundColor"),
        .fogColor = c.GetShaderLocation(lit, "fogColor"),
        .viewPos = c.GetShaderLocation(lit, "viewPos"),
        .fogDensity = c.GetShaderLocation(lit, "fogDensity"),
        .lightVP = c.GetShaderLocation(lit, "lightVP"),
        .shadowMap = c.GetShaderLocation(lit, "shadowMap"),
        .shadowTexel = c.GetShaderLocation(lit, "shadowTexel"),
        .emissive = c.GetShaderLocation(lit, "emissive"),
        .isTerrain = c.GetShaderLocation(lit, "isTerrain"),
        .roads = c.GetShaderLocation(lit, "roads"),
        .roadCount = c.GetShaderLocation(lit, "roadCount"),
    };
    for ([_]c.Shader{ lit, sky }) |s| {
        setV3(s, c.GetShaderLocation(s, "sunDir"), .{ sunDir.x, sunDir.y, sunDir.z });
        setV3(s, c.GetShaderLocation(s, "sunColor"), sunColor);
        setV3(s, c.GetShaderLocation(s, "skyColor"), skyColor);
        setV3(s, c.GetShaderLocation(s, "groundColor"), groundColor);
        setV3(s, c.GetShaderLocation(s, "fogColor"), fogColor);
        setF(s, c.GetShaderLocation(s, "fogDensity"), FOG_DENSITY);
    }
    setV3(sky, c.GetShaderLocation(sky, "zenithColor"), zenithColor);
    skyTime = c.GetShaderLocation(sky, "time");
    skyView = c.GetShaderLocation(sky, "viewPos");
    setF(lit, L.shadowTexel, 1.0 / @as(f32, SHADOW_SIZE));
    setF(lit, L.emissive, 0);
    setI(lit, L.isTerrain, 0);
    const roads = [_]f32{
        0,     0,     1350,  750,
        0,     0,     950,   -1150,
        950,   -1150, -1450, -950,
        -650,  1450,  0,     0,
        -8000, 420,   8000,  -260,
        2300,  -350,  1350,  750,
        -1450, -950,  -3200, -7000,
        -650,  1450,  -2500, 8000,
    };
    c.SetShaderValueV(lit, L.roads, &roads, c.SHADER_UNIFORM_VEC4, 8);
    setI(lit, L.roadCount, 8);
}

fn setV3(s: c.Shader, loc: c_int, v: [3]f32) void {
    c.SetShaderValue(s, loc, &v, c.SHADER_UNIFORM_VEC3);
}
fn setF(s: c.Shader, loc: c_int, v: f32) void {
    c.SetShaderValue(s, loc, &v, c.SHADER_UNIFORM_FLOAT);
}
fn setI(s: c.Shader, loc: c_int, v: c_int) void {
    c.SetShaderValue(s, loc, &v, c.SHADER_UNIFORM_INT);
}

fn initShadow() void {
    const fbo = c.rlLoadFramebuffer();
    const depth = c.rlLoadTextureDepth(SHADOW_SIZE, SHADOW_SIZE, false);
    c.rlFramebufferAttach(fbo, depth, c.RL_ATTACHMENT_DEPTH, c.RL_ATTACHMENT_TEXTURE2D, 0);
    if (!c.rlFramebufferComplete(fbo)) std.debug.print("shadow fbo incomplete\n", .{});
    shadowRT = std.mem.zeroes(c.RenderTexture2D);
    shadowRT.id = fbo;
    shadowRT.texture.width = SHADOW_SIZE;
    shadowRT.texture.height = SHADOW_SIZE;
    shadowRT.depth.id = depth;
    shadowRT.depth.width = SHADOW_SIZE;
    shadowRT.depth.height = SHADOW_SIZE;
    shadowRT.depth.format = 19;
    shadowRT.depth.mipmaps = 1;
}

fn initFont() void {
    var cps: [95 + 96 + 8]c_int = undefined;
    var n: usize = 0;
    var cp: c_int = 32;
    while (cp <= 126) : (cp += 1) {
        cps[n] = cp;
        n += 1;
    }
    cp = 0x400;
    while (cp < 0x460) : (cp += 1) {
        cps[n] = cp;
        n += 1;
    }
    for ([_]c_int{ 0xB0, 0x2014, 0x2116, 0x25B2, 0x25A0, 0x25A1, 0xB7, 0x2022 }) |e| {
        cps[n] = e;
        n += 1;
    }
    const paths = [_][*c]const u8{
        "assets/font.ttf", // PT Sans Narrow Bold (OFL), bundled for Windows/Linux
        "/System/Library/Fonts/Supplemental/Arial Narrow Bold.ttf",
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/Library/Fonts/Arial Unicode.ttf",
    };
    for (paths) |p| {
        if (c.FileExists(p)) {
            font = c.LoadFontEx(p, 48, &cps, @intCast(n));
            if (font.texture.id != 0) {
                c.GenTextureMipmaps(&font.texture);
                c.SetTextureFilter(font.texture, c.TEXTURE_FILTER_TRILINEAR);
                fontOk = true;
                return;
            }
        }
    }
    font = c.GetFontDefault();
}

fn txt(s: [*c]const u8, x: f32, y: f32, size: f32, color: c.Color) void {
    c.DrawTextEx(font, s, vm.v2(x, y), size, 0.5, color);
}
fn txtW(s: [*c]const u8, size: f32) f32 {
    return c.MeasureTextEx(font, s, size, 0.5).x;
}
fn txtC(s: [*c]const u8, x: f32, y: f32, size: f32, color: c.Color) void {
    txt(s, x - txtW(s, size) / 2, y, size, color);
}
fn txtR(s: [*c]const u8, x: f32, y: f32, size: f32, color: c.Color) void {
    txt(s, x - txtW(s, size), y, size, color);
}

fn ground(x: f32, z: f32) V3 {
    return v3(x, heightAt(x, z), z);
}

fn initWorld() void {
    // battery layout
    g.launchers[0] = .{ .pos = ground(-30, -14), .yaw = 0.35 };
    g.launchers[1] = .{ .pos = ground(-34, 20), .yaw = -0.25 };
    radarPos = ground(8, 34);
    spaagPos = ground(26, -30);

    // tube mouths in turret frame (Blender -> raylib: (x, z, -y))
    const ce = @cos(TUBE_EL);
    const se = @sin(TUBE_EL);
    var k: usize = 0;
    for ([_]f32{ -1, 1 }) |a| {
        for ([_]f32{ -1, 1 }) |b| {
            // hinge + d*L + (0,a*.42,0) + n*(b*.42) in Blender coords
            const bx = -1.6 + ce * 6.4 + (-se) * b * 0.42;
            const by = a * 0.42;
            const bz = 0.75 + se * 6.4 + ce * b * 0.42;
            tubeMouth[k] = v3(bx, bz, -by);
            k += 1;
        }
    }

    // defended objects
    objs[0] = .{ .name = "ТЭЦ", .pos = ground(objects_static[0].x, objects_static[0].z), .radius = 70 };
    objs[1] = .{ .name = "НЕФТЕБАЗА", .pos = ground(objects_static[1].x, objects_static[1].z), .radius = 50 };
    objs[2] = .{ .name = "ЖИЛОЙ КВАРТАЛ", .pos = ground(objects_static[2].x, objects_static[2].z), .radius = 90 };

    nprops = 0;
    addProp(.power, objs[0].pos, 0.5);
    addProp(.depot, objs[1].pos, -0.3);
    const cp = objs[2].pos;
    const blocks = [_][3]f32{ .{ -45, -40, 0.3 }, .{ 40, -55, 0.3 }, .{ -30, 40, 0.3 }, .{ 55, 30, 1.87 }, .{ -90, 0, 1.87 } };
    for (blocks) |b| {
        const cs = @cos(0.3);
        const sn = @sin(0.3);
        const x = cp.x + b[0] * cs - b[1] * sn;
        const z = cp.z + b[0] * sn + b[1] * cs;
        addProp(.apartment, ground(x, z), b[2]);
    }
    // villages
    for ([_][2]f32{ .{ -650, 1450 }, .{ 2300, -350 } }) |vc| {
        var i: usize = 0;
        while (i < 22) : (i += 1) {
            const row: f32 = @floatFromInt(i % 2);
            const col_: f32 = @floatFromInt(i / 2);
            const x = vc[0] + (col_ - 5) * 24 + vm.rr(-4, 4);
            const z = vc[1] + (row * 2 - 1) * 26 + vm.rr(-5, 5);
            addProp(.house, ground(x, z), (if (row > 0) vm.pi else 0) + vm.rr(-0.1, 0.1));
        }
    }

    // trees
    ntrees = 0;
    var tries: usize = 0;
    while (ntrees < MAX_TREES and tries < 120000) : (tries += 1) {
        const r = @sqrt(vm.rnd()) * 4600;
        const a = vm.rnd() * 2 * vm.pi;
        const x = @cos(a) * r;
        const z = @sin(a) * r;
        const f = forestFn(x, z);
        const lone = vm.rnd() < 0.004 and forestFn(x, z) == 0 and @sqrt(x * x + z * z) > 260 and awayFromProps(x, z);
        if (f > 0.35 and vm.rnd() < f or lone) {
            const nrmY = slopeY(x, z);
            if (nrmY < 0.85) continue;
            trees[ntrees] = .{ .pos = v3(x, heightAt(x, z) - 0.3, z), .yaw = vm.rnd() * 360, .s = vm.rr(0.75, 1.45), .pine = vm.rnd() < 0.62 };
            ntrees += 1;
        }
    }
}

fn awayFromProps(x: f32, z: f32) bool {
    for (objects_static) |o| {
        if ((x - o.x) * (x - o.x) + (z - o.z) * (z - o.z) < (o.r + 40) * (o.r + 40)) return false;
    }
    return true;
}

fn slopeY(x: f32, z: f32) f32 {
    const e: f32 = 8;
    const n = vm.norm(v3(heightAt(x - e, z) - heightAt(x + e, z), 2 * e, heightAt(x, z - e) - heightAt(x, z + e)));
    return n.y;
}

fn addProp(kind: PropKind, pos: V3, yaw: f32) void {
    if (nprops >= MAX_PROPS) return;
    props[nprops] = .{ .kind = kind, .pos = v3(pos.x, pos.y - 0.4, pos.z), .yaw = yaw };
    nprops += 1;
}

fn resetGame() void {
    const keepWarp = g.warp;
    g = .{};
    g.warp = keepWarp;
    for (&targets) |*t| t.* = .{};
    for (&missiles) |*m| m.* = .{};
    for (&shells) |*s| s.* = .{};
    for (&parts) |*p| p.alive = false;
    g.launchers[0] = .{ .pos = ground(-30, -14), .yaw = 0.35 };
    g.launchers[1] = .{ .pos = ground(-34, 20), .yaw = -0.25 };
    for (&objs) |*o| {
        o.hp = 100;
        o.nfires = 0;
    }
    if (mode == .defense) msg("Боевое дежурство. Ожидайте налёт.", HUD);
}

// ============================================================ waves & targets
fn startWave() void {
    g.wave += 1;
    const w = g.wave;
    const base: f32 = @floatFromInt(3 + 2 * w + (if (w >= 2) w - 1 else 0) + (if (w >= 3) w - 2 else 0));
    g.toSpawn = @intFromFloat(@max(2, @round(base * diffK(0.7, 1, 1.4))));
    g.spawnT = 0;
    g.waveBearing = vm.rnd() * 2 * vm.pi;
    g.reserve = @min(g.reserve + @as(i32, @intFromFloat(diffK(12, 8, 6))), @as(i32, @intFromFloat(diffK(32, 24, 18))));
    g.gAmmo = 600;
    if (app == .playing) storeGame();
    msg(c.TextFormat("ВОЛНА %d: радар фиксирует групповую цель", @as(c_int, w)), HUD_AMBER);
}

fn spawnTarget(kindIn: TKind, radius: f32) void {
    for (&targets) |*t| {
        if (t.alive) continue;
        t.* = .{};
        t.alive = true;
        t.id = g.nextId;
        g.nextId += 1;
        t.kind = kindIn;
        const bearing = g.waveBearing + vm.rr(-0.6, 0.6);
        const sx = @cos(bearing) * radius;
        const sz = @sin(bearing) * radius;
        t.goal = @intFromFloat(@floor(vm.rnd() * 2.999));
        // pick an alive goal if possible
        var tries: usize = 0;
        while (objs[t.goal].hp <= 0 and tries < 3) : (tries += 1) t.goal = (t.goal + 1) % 3;
        const gp = objs[t.goal].pos;
        switch (t.kind) {
            .drone => {
                t.speed = vm.rr(48, 58);
                t.hp = 3;
                t.alt = vm.rr(140, 380);
            },
            .decoy => {
                t.speed = vm.rr(42, 50);
                t.hp = 2;
                t.alt = vm.rr(150, 420);
            },
            .cruise => {
                t.speed = vm.rr(220, 245);
                t.hp = 4;
                t.alt = vm.rr(35, 55);
            },
        }
        t.speed *= diffK(0.85, 1, 1.15);
        t.pos = v3(sx, heightAt(sx, sz) + t.alt + 150, sz);
        // dogleg path: offset midpoint, approach point, then goal
        const mid = vm.lerp(t.pos, gp, 0.45);
        const side = vm.norm(v3(-(gp.z - t.pos.z), 0, gp.x - t.pos.x));
        const off = vm.rr(-1800, 1800);
        t.wps[0] = vm.mad(mid, side, off);
        const toGoal = vm.norm(v3(gp.x - t.wps[0].x, 0, gp.z - t.wps[0].z));
        t.wps[1] = vm.mad(gp, toGoal, -1300);
        t.wps[2] = gp;
        t.nwp = 3;
        t.wi = 0;
        t.vel = vm.scale(vm.norm(vm.sub(t.wps[0], t.pos)), t.speed);
        return;
    }
}

fn targetValid(i: i32, id: u32) bool {
    if (i < 0) return false;
    const t = &targets[@intCast(i)];
    return t.alive and t.id == id;
}

fn tracked(t: *const Target) bool {
    return t.alive and t.detected and g.time - t.seenT < 6.5;
}

fn hostile(t: *const Target) bool {
    return t.kind != .decoy or !t.classified;
}

fn killTarget(i: usize, byMissile: bool) void {
    const t = &targets[i];
    if (!t.alive or t.dying) return;
    if (t.player) {
        g.playerDead = true;
        g.success = false;
        g.endT = 3.5;
        g.endPos = t.pos;
        msg(if (byMissile) "Дрон поражён ЗУР" else "Дрон поражён огнём ЗСУ", HUD_RED);
        if (byMissile) {
            explosion(t.pos, 1.0, false);
            t.alive = false;
        } else {
            t.dying = true;
            explosion(t.pos, 0.45, false);
        }
        return;
    }
    if (mode == .attack) {
        if (byMissile) {
            explosion(t.pos, 1.0, false);
            t.alive = false;
        } else {
            t.dying = true;
            explosion(t.pos, 0.45, false);
        }
        msg(c.TextFormat("Ведомый #%02d сбит", @as(c_int, @intCast(t.id))), HUD_AMBER);
        return;
    }
    const pts: i32 = switch (t.kind) {
        .drone => 100,
        .cruise => 300,
        .decoy => 10,
    };
    g.score += pts;
    g.kills += 1;
    const name: [*c]const u8 = switch (t.kind) {
        .drone => "БПЛА",
        .cruise => "КР",
        .decoy => "ЛОЖНАЯ ЦЕЛЬ",
    };
    msg(c.TextFormat("Цель #%02d (%s) поражена %s", @as(c_int, @intCast(t.id)), name, if (byMissile) "ЗУР" else "ЗСУ"), HUD);
    if (byMissile) {
        explosion(t.pos, if (t.kind == .cruise) 1.6 else 1.0, false);
        t.alive = false;
    } else {
        t.dying = true;
        explosion(t.pos, 0.45, false);
    }
}

fn impact(t: *Target) void {
    if (t.player) {
        const ag = atkGoal(g.atkGoal);
        var p = t.pos;
        p.y = @max(p.y, heightAt(p.x, p.z) + 1);
        const hit = !t.dying and vm.distXZ(p, ag.pos) < ag.r + 25;
        explosion(p, if (hit) 2.6 else 1.4, true);
        t.alive = false;
        g.playerDead = true;
        g.endT = 3.5;
        g.endPos = p;
        g.success = hit;
        if (hit) {
            g.strikes += 1;
            g.score += 1000 + 250 * g.sortie;
            if (g.atkGoal < 2) {
                const o = &objs[g.atkGoal];
                o.hp = @max(0, o.hp - 35);
                if (o.nfires < o.fires.len) {
                    o.fires[o.nfires] = p;
                    o.nfires += 1;
                }
            } else {
                // battery hit: knock out the launchers' ready missiles
                for (&g.launchers) |*l| l.tubes = .{ false, false, false, false };
                g.reserve = 0;
            }
            msg(c.TextFormat("Цель поражена: %s", ag.name), HUD);
        } else if (!t.dying) {
            msg("Дрон упал вне цели", HUD_RED);
        }
        return;
    }
    const o = &objs[t.goal];
    const near = vm.distXZ(t.pos, o.pos) < o.radius + 60;
    const big: f32 = switch (t.kind) {
        .drone => 1.8,
        .cruise => 2.6,
        .decoy => 0.5,
    };
    var p = t.pos;
    p.y = @max(p.y, heightAt(p.x, p.z) + 1);
    explosion(p, big, true);
    if (near and t.kind != .decoy and !t.dying and o.hp > 0) {
        const dmg: f32 = if (t.kind == .cruise) 30 else 17;
        o.hp = @max(0, o.hp - dmg);
        if (mode == .defense) {
            g.leaks += 1;
            g.score -= @intFromFloat(dmg * 10);
        } else g.score += @intFromFloat(dmg * 5);
        if (o.nfires < o.fires.len) {
            o.fires[o.nfires] = p;
            o.nfires += 1;
        }
        if (o.hp <= 0) {
            msg(c.TextFormat("%s УНИЧТОЖЕН", o.name), HUD_RED);
        } else {
            msg(c.TextFormat("Прилёт: %s, ущерб %d%%", o.name, @as(c_int, @intFromFloat(dmg))), HUD_RED);
        }
    }
    t.alive = false;
}

fn updateTargets(dt: f32) void {
    for (&targets, 0..) |*t, idx| {
        if (!t.alive) continue;
        if (tracked(t)) t.trackT += dt else t.trackT = 0;
        t.fxT -= dt;
        if (t.dying) {
            t.vel.y -= G * dt;
            t.vel = vm.scale(t.vel, 1 - 0.15 * dt);
            t.pos = vm.mad(t.pos, t.vel, dt);
            t.roll += 3 * dt;
            if (t.fxT <= 0) {
                t.fxT = 0.03;
                fire(t.pos, vm.zero, 0.35, 2.5, 4, 1.2);
                smoke(t.pos, v3(0, 1, 0), 4, 2, 8, 0.15, 0.8);
            }
            if (t.pos.y <= heightAt(t.pos.x, t.pos.z)) {
                explosion(v3(t.pos.x, heightAt(t.pos.x, t.pos.z) + 1, t.pos.z), 0.8, true);
                t.alive = false;
            }
            continue;
        }
        if (t.player) {
            updatePlayer(t, dt);
            continue;
        }
        const goal = t.wps[@min(t.wi, t.nwp - 1)];
        var aim = goal;
        const dGoal = vm.distXZ(t.pos, goal);
        const terrH = heightAt(t.pos.x, t.pos.z);
        if (t.wi < t.nwp - 1) {
            if (dGoal < 250) t.wi += 1;
            // hold cruise altitude above terrain (look ahead)
            const ahead = vm.mad(t.pos, vm.norm(v3(t.vel.x, 0, t.vel.z)), if (t.kind == .cruise) 600 else 300);
            const hAhead = @max(terrH, heightAt(ahead.x, ahead.z));
            aim.y = hAhead + t.alt;
        } else {
            // terminal phase
            const diveR: f32 = if (t.kind == .cruise) 1500 else 900;
            if (dGoal < diveR) t.diving = true;
            if (!t.diving) {
                aim.y = @max(terrH, heightAt(goal.x, goal.z)) + t.alt;
            } else {
                aim = v3(goal.x, goal.y + 6, goal.z);
            }
        }
        const want = vm.norm(vm.sub(aim, t.pos));
        const cur = vm.norm(t.vel);
        const turnRate: f32 = switch (t.kind) {
            .cruise => if (t.diving) 0.45 else 0.14,
            else => if (t.diving) 0.9 else 0.25,
        };
        // rotate cur toward want by at most turnRate*dt
        const cosA = vm.clampf(vm.dot(cur, want), -1, 1);
        const ang = c.acosf(cosA);
        var nd = want;
        if (ang > turnRate * dt) nd = vm.norm(vm.lerp(cur, want, (turnRate * dt) / ang));
        // bank into turns
        const yawRate = vm.wrapPi(vm.yawOf(nd) - vm.yawOf(cur)) / @max(dt, 1e-4);
        t.roll = vm.lerpf(t.roll, vm.clampf(-yawRate * 1.6, -0.9, 0.9), 2 * dt);
        const spd = if (t.diving) t.speed * @as(f32, if (t.kind == .cruise) 1.1 else 1.6) else t.speed;
        t.vel = vm.scale(nd, spd);
        t.pos = vm.mad(t.pos, t.vel, dt);
        if (t.pos.y <= heightAt(t.pos.x, t.pos.z) + 0.5) {
            impact(t);
            if (g.selected == @as(i32, @intCast(idx))) g.selected = -1;
            continue;
        }
        if (t.diving and vm.dist(t.pos, v3(goal.x, goal.y + 6, goal.z)) < 12) impact(t);
    }
}

// ============================================================ attack mode
fn sensK() f32 {
    return 0.0012 * @as(f32, @floatFromInt(save.settings.sens + 1));
}

fn updatePlayer(t: *Target, dt: f32) void {
    // input: mouse + WASD/arrows. W/up = nose down (flight-style)
    var yawIn: f32 = 0;
    var pitchIn: f32 = 0;
    if (c.IsKeyDown(c.KEY_A) or c.IsKeyDown(c.KEY_LEFT)) yawIn += 1;
    if (c.IsKeyDown(c.KEY_D) or c.IsKeyDown(c.KEY_RIGHT)) yawIn -= 1;
    if (c.IsKeyDown(c.KEY_W) or c.IsKeyDown(c.KEY_UP)) pitchIn -= 1;
    if (c.IsKeyDown(c.KEY_S) or c.IsKeyDown(c.KEY_DOWN)) pitchIn += 1;
    const md = c.GetMouseDelta();
    const inv: f32 = if (save.settings.invertY == 1) -1 else 1;
    const realDt = @max(dt, 1e-4);
    const mouseYaw = -md.x * sensK() / realDt * 0.016;
    const mousePitch = -md.y * sensK() * inv / realDt * 0.016;
    const yawRate = vm.clampf(yawIn * 0.45 + mouseYaw * 25, -0.6, 0.6);
    const pitchRate = vm.clampf(pitchIn * 0.5 + mousePitch * 25, -0.7, 0.7);
    g.pManeuver = vm.lerpf(g.pManeuver, @abs(yawRate) + @abs(pitchRate), @min(1, 6 * dt));
    g.pYaw += yawRate * dt;
    g.pPitch = vm.clampf(g.pPitch + pitchRate * dt, -1.2, 0.5);
    g.pRoll = vm.lerpf(g.pRoll, vm.clampf(-yawRate * 1.4, -0.8, 0.8), @min(1, 4 * dt));
    t.roll = g.pRoll;

    var speed: f32 = 52;
    if (c.IsKeyDown(c.KEY_LEFT_SHIFT) and g.boost > 0) {
        speed = 74;
        g.boost = @max(0, g.boost - dt / 10);
    } else g.boost = @min(1, g.boost + dt / 40);
    // gain speed in a dive, lose it in a climb
    speed += -@sin(g.pPitch) * 45;
    const dir = vm.yawPitch(vm.zero, g.pYaw, g.pPitch).x;
    t.vel = vm.scale(dir, speed);
    t.pos = vm.mad(t.pos, t.vel, dt);
    t.pos.y = @min(t.pos.y, heightAt(t.pos.x, t.pos.z) + 1200);
    if (t.pos.y <= heightAt(t.pos.x, t.pos.z) + 0.5) impact(t);
    // engine exhaust haze
    t.fxT -= dt;
    if (t.fxT <= 0) {
        t.fxT = 0.05;
        smoke(vm.mad(t.pos, dir, -2.2), vm.zero, 1.2, 0.4, 1.6, 0.5, 0.25);
    }
}

fn startSortie() void {
    mode = .attack;
    const keepSortie = g.sortie;
    const keepStrikes = g.strikes;
    const keepScore = g.score;
    const keepGoal = g.atkGoal;
    resetGame();
    g.atkGoal = keepGoal;
    g.sortie = keepSortie;
    g.strikes = keepStrikes;
    g.score = keepScore;
    g.autoSam = true;
    g.gAuto = true;
    // harder every sortie
    g.reserve = 8 + 4 * g.sortie;
    const goal = atkGoal(g.atkGoal);
    g.waveBearing = vm.rnd() * 2 * vm.pi;
    const r: f32 = 6800;
    var sx = goal.pos.x + @cos(g.waveBearing) * r;
    var sz = goal.pos.z + @sin(g.waveBearing) * r;
    sx = vm.clampf(sx, -7000, 7000);
    sz = vm.clampf(sz, -7000, 7000);
    // wingmen first (they use the normal AI path)
    const nWing: usize = @intCast(@min(2 + g.sortie, 8));
    var k: usize = 0;
    while (k < nWing) : (k += 1) spawnTarget(if (k % 3 == 2) .decoy else .drone, 7000);
    // player
    for (&targets, 0..) |*t, i| {
        if (t.alive) continue;
        t.* = .{};
        t.alive = true;
        t.player = true;
        t.kind = .drone;
        t.id = g.nextId;
        g.nextId += 1;
        t.hp = diffK(10, 6, 3);
        t.speed = 52;
        t.pos = v3(sx, heightAt(sx, sz) + 140, sz);
        const to = vm.sub(goal.pos, t.pos);
        g.pYaw = vm.yawOf(to);
        g.pPitch = 0;
        g.boost = 1;
        t.vel = vm.scale(vm.yawPitch(vm.zero, g.pYaw, 0).x, 52);
        g.player = @intCast(i);
        g.playerId = t.id;
        break;
    }
    g.camMode = 0;
    g.playerDead = false;
    g.endT = 0;
    msg(c.TextFormat("Вылет %d. Цель: %s, %.1f км", @as(c_int, g.sortie + 1), goal.name, @as(f64, r / 1000)), HUD_AMBER);
}

fn playerTarget() ?*Target {
    if (g.player < 0) return null;
    const t = &targets[@intCast(g.player)];
    if (!t.alive or t.id != g.playerId) return null;
    return t;
}

fn finishSortie() void {
    if (g.success) g.sortie += 1;
    updateRecords();
    storeGame();
    app = .result;
    menuSel = 0;
    c.EnableCursor();
}

// ============================================================ radar
fn lineOfSight(a: V3, b: V3) bool {
    var i: usize = 1;
    while (i < 16) : (i += 1) {
        const p = vm.lerp(a, b, @as(f32, @floatFromInt(i)) / 16.0);
        if (heightAt(p.x, p.z) > p.y) return false;
    }
    return true;
}

fn updateRadar(dt: f32) void {
    const rate: f32 = 2 * vm.pi / 3.0;
    const prev = g.rYaw;
    g.rYaw = @mod(g.rYaw + rate * dt, 2 * vm.pi);
    const ant = vm.add(radarPos, v3(0, 7, 0));
    for (&targets) |*t| {
        if (!t.alive) continue;
        const d = vm.sub(t.pos, ant);
        const b = @mod(vm.yawOf(d) + 2 * vm.pi, 2 * vm.pi);
        const swept = if (g.rYaw >= prev) (b >= prev and b < g.rYaw) else (b >= prev or b < g.rYaw);
        if (!swept) continue;
        const r = vm.len(d);
        var rng: f32 = switch (t.kind) {
            .cruise => 6500,
            else => RADAR_RANGE,
        };
        // ground clutter: very low flyers are only picked up close in
        const agl = t.pos.y - heightAt(t.pos.x, t.pos.z);
        if (agl < 60) rng = @min(rng, 2800);
        if (t.player) {
            // the player's drone has a small radar cross-section
            rng = @min(rng, diffK(4500, 6000, 7500));
            if (agl < 60) rng = @min(rng, diffK(1100, 1700, 2400));
            if (agl < 30) rng = @min(rng, diffK(600, 1000, 1500));
        }
        if (r < rng and lineOfSight(ant, t.pos)) {
            if (!t.detected) {
                t.detected = true;
                const ty: [*c]const u8 = if (t.kind == .cruise) "скоростная" else "воздушная";
                msg(c.TextFormat("Обнаружена %s цель #%02d, Д %.1f км", ty, @as(c_int, @intCast(t.id)), @as(f64, r / 1000)), HUD_AMBER);
            }
            t.seenT = g.time;
            t.seenPos = t.pos;
            const clsR: f32 = if (t.kind == .decoy) 2600 else 5000;
            if (r < clsR) t.classified = true;
        }
    }
}

// ============================================================ SAM
fn launcherXf(l: *const Launcher) Xf {
    return vm.yawPitch(l.pos, l.yaw, 0).mul(vm.yawPitch(TURRET_ON_HULL, l.tyaw, 0));
}

fn requestLaunch() void {
    if (g.selected < 0 or !targetValid(g.selected, g.selId)) {
        msg("Нет выбранной цели (TAB / клик)", HUD_DIM);
        return;
    }
    const t = &targets[@intCast(g.selected)];
    if (!tracked(t)) {
        msg("Цель не сопровождается", HUD_DIM);
        return;
    }
    if (g.nqueue >= g.queue.len) return;
    g.queue[g.nqueue] = t.id;
    g.nqueue += 1;
}

fn findById(id: u32) i32 {
    for (&targets, 0..) |*t, i| {
        if (t.alive and t.id == id) return @intCast(i);
    }
    return -1;
}

fn missilesOn(id: u32) i32 {
    var n: i32 = 0;
    for (&missiles) |*m| {
        if (m.alive and m.tgtId == id) n += 1;
    }
    return n;
}

fn readyCount() i32 {
    var n: i32 = 0;
    for (&g.launchers) |*l| {
        for (l.tubes) |tb| {
            if (tb) n += 1;
        }
    }
    return n;
}

fn updateSam(dt: f32) void {
    // reload
    for (&g.launchers) |*l| {
        var empty: bool = false;
        for (l.tubes) |tb| {
            if (!tb) empty = true;
        }
        if (empty and g.reserve > 0) {
            l.reloadT += dt;
            if (l.reloadT > 9) {
                l.reloadT = 0;
                for (&l.tubes) |*tb| {
                    if (!tb.*) {
                        tb.* = true;
                        g.reserve -= 1;
                        break;
                    }
                }
            }
        } else l.reloadT = 0;
    }
    // auto mode: pick worst threat without a missile on it
    if (g.autoSam) {
        g.autoT -= dt;
        if (g.autoT <= 0 and g.nqueue == 0) {
            g.autoT = 0.6;
            var best: i32 = -1;
            var bestD: f32 = 1e9;
            for (&targets, 0..) |*t, i| {
                if (!tracked(t) or t.dying or !t.classified or t.kind == .decoy) continue;
                // fire-control solution takes time; slower in attack mode so the player has a chance
                const needT: f32 = if (mode == .attack) diffK(9, 6, 3.5) - @min(@as(f32, @floatFromInt(g.sortie)) * 0.5, diffK(2, 3, 2)) else diffK(1, 1.5, 2);
                if (t.trackT < needT) continue;
                const r = vm.len(t.pos);
                if (r > 7500) continue;
                if (missilesOn(t.id) > 0) continue;
                const dg = vm.dist(t.pos, objs[t.goal].pos);
                if (dg < bestD) {
                    bestD = dg;
                    best = @intCast(i);
                }
            }
            if (best >= 0 and readyCount() > 0) {
                g.queue[0] = targets[@intCast(best)].id;
                g.nqueue = 1;
                // attack mode: AI fires one missile at a time with a pause between launches
                if (mode == .attack) g.autoT = diffK(7, 4, 2);
            }
        }
    }
    // pick launcher & slew
    if (g.nqueue > 0) {
        const ti = findById(g.queue[0]);
        if (ti < 0 or readyCount() == 0) {
            if (readyCount() == 0) msg("ЗРК: нет готовых ракет", HUD_RED);
            popQueue();
            return;
        }
        const t = &targets[@intCast(ti)];
        var bestL: usize = 0;
        var bestErr: f32 = 1e9;
        for (&g.launchers, 0..) |*l, li| {
            var any = false;
            for (l.tubes) |tb| {
                if (tb) any = true;
            }
            if (!any) continue;
            const want = vm.wrapPi(vm.yawOf(vm.sub(t.pos, l.pos)) - l.yaw);
            const err = @abs(vm.wrapPi(want - l.tyaw));
            if (err < bestErr) {
                bestErr = err;
                bestL = li;
            }
        }
        const l = &g.launchers[bestL];
        const want = vm.wrapPi(vm.yawOf(vm.sub(t.pos, l.pos)) - l.yaw);
        l.tyaw = vm.approachAngle(l.tyaw, want, 50 * vm.deg * dt);
        if (@abs(vm.wrapPi(want - l.tyaw)) < 2 * vm.deg) {
            for (&l.tubes, 0..) |*tb, k| {
                if (tb.*) {
                    tb.* = false;
                    launchMissile(l, k, ti);
                    break;
                }
            }
            popQueue();
        }
    }
}

fn popQueue() void {
    var i: usize = 1;
    while (i < g.nqueue) : (i += 1) g.queue[i - 1] = g.queue[i];
    if (g.nqueue > 0) g.nqueue -= 1;
}

fn launchMissile(l: *const Launcher, tube: usize, ti: i32) void {
    const xf = launcherXf(l);
    const mouth = xf.point(tubeMouth[tube]);
    const dir = xf.dir(v3(@cos(TUBE_EL), @sin(TUBE_EL), 0));
    for (&missiles, 0..) |*m, mi| {
        if (m.alive) continue;
        m.* = .{ .alive = true, .pos = mouth, .vel = vm.scale(dir, 45), .tgt = ti, .tgtId = targets[@intCast(ti)].id, .id = @intCast(g.samFired) };
        g.samFired += 1;
        g.lastMissile = @intCast(mi);
        // launch blast
        var i: usize = 0;
        while (i < 26) : (i += 1) {
            const d = vm.rndDir();
            smoke(vm.mad(mouth, dir, -6), vm.add(vm.scale(d, vm.rr(4, 14)), vm.scale(dir, -12)), vm.rr(4, 8), 3, vm.rr(12, 20), vm.rr(0.75, 0.9), 0.75);
        }
        spawnPart(.{ .pos = mouth, .life = 0.25, .s0 = 14, .s1 = 22, .c0 = .{ 1, 0.9, 0.6, 1 }, .c1 = .{ 1, 0.5, 0.1, 0 }, .add = true });
        msg(c.TextFormat("Пуск ЗУР по цели #%02d", @as(c_int, @intCast(targets[@intCast(ti)].id))), HUD);
        return;
    }
}

fn updateMissiles(dt: f32) void {
    for (&missiles) |*m| {
        if (!m.alive) continue;
        m.t += dt;
        var speed = vm.len(m.vel);
        // motor: boost then sustain then coast
        if (m.t < 0.25) {
            speed += 60 * dt;
        } else if (m.t < 7) {
            speed = @min(speed + 190 * dt, 1150);
        } else speed -= 18 * dt;
        var dir = vm.norm(m.vel);
        // guidance
        if (!targetValid(m.tgt, m.tgtId) or targets[@intCast(m.tgt)].dying) {
            // try retarget
            m.lostT += dt;
            var best: i32 = -1;
            var bd: f32 = 2500;
            for (&targets, 0..) |*t, i| {
                if (!tracked(t) or t.dying) continue;
                const d = vm.sub(t.pos, m.pos);
                const r = vm.len(d);
                if (r < bd and vm.dot(vm.scale(d, 1 / r), dir) > 0.5) {
                    bd = r;
                    best = @intCast(i);
                }
            }
            if (best >= 0) {
                m.tgt = best;
                m.tgtId = targets[@intCast(best)].id;
                m.lostT = 0;
            }
        }
        var prevTgtPos: V3 = vm.zero;
        var hasT = false;
        if (targetValid(m.tgt, m.tgtId) and m.t > 0.7) {
            const t = &targets[@intCast(m.tgt)];
            hasT = true;
            prevTgtPos = t.pos;
            const R = vm.sub(t.pos, m.pos);
            const Vr = vm.sub(t.vel, m.vel);
            const omega = vm.scale(vm.cross(R, Vr), 1.0 / @max(vm.dot(R, R), 1));
            var acc = vm.scale(vm.cross(omega, m.vel), 4.0);
            acc.y += G;
            // remove along-track component
            acc = vm.sub(acc, vm.scale(dir, vm.dot(acc, dir)));
            const amax: f32 = 38 * G;
            const al = vm.len(acc);
            if (al > amax) acc = vm.scale(acc, amax / al);
            // early: blend toward LOS for a tidy pitch-over
            if (m.t < 1.6) {
                const want = vm.norm(R);
                dir = vm.norm(vm.lerp(dir, want, @min(1, 2.2 * dt)));
            }
            dir = vm.norm(vm.mad(dir, acc, dt / @max(speed, 50)));
        } else {
            dir = vm.norm(vm.mad(dir, v3(0, -G, 0), dt / @max(speed, 50)));
        }
        const old = m.pos;
        m.vel = vm.scale(dir, speed);
        m.pos = vm.mad(m.pos, m.vel, dt);

        // proximity fuse via closest approach over this step
        if (hasT) {
            const t = &targets[@intCast(m.tgt)];
            const a0 = vm.sub(old, vm.mad(t.pos, t.vel, -dt));
            const a1 = vm.sub(m.pos, t.pos);
            const da = vm.sub(a1, a0);
            const s = vm.clampf(-vm.dot(a0, da) / @max(vm.dot(da, da), 1e-6), 0, 1);
            const cd = vm.len(vm.mad(a0, da, s));
            if (cd < 14) {
                var pk: f32 = if (t.kind == .cruise) 0.82 else 0.93;
                // ground clutter spoils the fuse on low targets
                if (t.pos.y - heightAt(t.pos.x, t.pos.z) < 60) pk *= 0.6;
                // a hard break at the last moment makes the missile overshoot
                if (t.player) pk *= if (g.pManeuver > 0.5) diffK(0.2, 0.35, 0.55) else diffK(0.55, 0.75, 0.9);
                m.alive = false;
                const bp = vm.lerp(old, m.pos, s);
                if (vm.rnd() < pk) {
                    const idx: usize = @intCast(m.tgt);
                    killTarget(idx, true);
                    explosion(bp, 0.7, false);
                } else {
                    explosion(bp, 0.7, false);
                    msg("ЗУР: промах", HUD_AMBER);
                }
                continue;
            }
        }
        // trail
        m.trailT -= dt;
        const tail = vm.mad(m.pos, dir, -3);
        while (m.trailT <= 0) {
            m.trailT += 0.011;
            if (m.t < 7) {
                const jitter = vm.scale(vm.rndDir(), 0.6);
                smoke(vm.add(vm.lerp(old, tail, vm.rnd()), jitter), vm.add(vm.scale(vm.rndDir(), 1.2), v3(0, 0.4, 0)), vm.rr(5, 9), 1.6, vm.rr(7, 11), vm.rr(0.85, 0.95), 0.55);
            }
        }
        if (m.t < 7) {
            spawnPart(.{ .pos = tail, .life = 0.05, .s0 = 4.5, .s1 = 2, .c0 = .{ 1, 0.95, 0.75, 1 }, .c1 = .{ 1, 0.6, 0.2, 1 }, .add = true });
        }
        // termination
        const hg = heightAt(m.pos.x, m.pos.z);
        if (m.pos.y < hg) {
            explosion(v3(m.pos.x, hg + 1, m.pos.z), 0.6, true);
            m.alive = false;
        } else if (m.t > 30 or (m.t > 8 and speed < 250) or m.lostT > 2.5) {
            explosion(m.pos, 0.5, false);
            m.alive = false;
        }
    }
}

// ============================================================ SPAAG
fn spaagTurretXf() Xf {
    return vm.yawPitch(spaagPos, spaagYaw, 0).mul(vm.yawPitch(SPAAG_TURRET, g.gYaw, 0));
}
fn spaagGunsXf() Xf {
    return spaagTurretXf().mul(vm.yawPitch(SPAAG_TRUNNION, 0, g.gPitch));
}

fn updateGun(dt: f32) void {
    if (g.gAmmo <= 0) {
        g.gReload += dt;
        if (g.gReload > 15) {
            g.gReload = 0;
            g.gAmmo = 600;
            msg("ЗСУ: боекомплект пополнен", HUD_DIM);
        }
    }
    // target selection
    const gunPos = spaagGunsXf().p;
    if (g.gunner) {
        updateGunManual(dt, gunPos);
        return;
    }
    if (g.gAuto) {
        var best: i32 = -1;
        var bd: f32 = GUN_RANGE;
        for (&targets, 0..) |*t, i| {
            if (!tracked(t) or t.dying) continue;
            const d = vm.dist(t.pos, gunPos);
            // vs the player: gun needs a settled track and closer range
            if (t.player and (t.trackT < diffK(4, 2.5, 1.5) or d > diffK(1600, 2300, 3000))) continue;
            if (d < bd and lineOfSight(gunPos, t.pos)) {
                bd = d;
                best = @intCast(i);
            }
        }
        g.gTgt = best;
    } else g.gTgt = -1;

    var wantYaw: f32 = 0;
    var wantPitch: f32 = 0.15;
    var tof: f32 = 0;
    if (g.gTgt >= 0) {
        const t = &targets[@intCast(g.gTgt)];
        // iterative intercept
        var aim = t.pos;
        var k: usize = 0;
        while (k < 4) : (k += 1) {
            tof = vm.dist(aim, gunPos) / SHELL_SPEED;
            aim = vm.mad(t.pos, t.vel, tof);
        }
        aim.y += 0.5 * G * tof * tof;
        const d = vm.sub(aim, gunPos);
        wantYaw = vm.wrapPi(vm.yawOf(d) - spaagYaw);
        wantPitch = vm.pitchOf(d);
    }
    g.gYaw = vm.approachAngle(g.gYaw, wantYaw, 100 * vm.deg * dt);
    g.gPitch = vm.approachAngle(g.gPitch, vm.clampf(wantPitch, -0.05, 1.45), 70 * vm.deg * dt);

    g.gPause -= dt;
    g.gFireT -= dt;
    if (g.gTgt >= 0 and g.gAmmo > 0 and g.gPause <= 0) {
        const err = @abs(vm.wrapPi(wantYaw - g.gYaw)) + @abs(wantPitch - g.gPitch);
        if (err < 1.5 * vm.deg) {
            while (g.gFireT <= 0) {
                g.gFireT += 0.055;
                fireShell(tof);
                g.gBurst += 1;
                if (g.gBurst >= 22) {
                    g.gBurst = 0;
                    g.gPause = 0.7;
                    break;
                }
            }
        }
    }
    if (g.gFireT < 0) g.gFireT = 0;
}

/// Boresight direction of the commanded aim (world space).
fn gunnerDir() V3 {
    return vm.yawPitch(vm.zero, spaagYaw + g.gAimYaw, g.gAimPitch).x;
}

/// Eye point of the gunner sight, on the turret roof.
fn gunnerEye() V3 {
    return spaagTurretXf().point(v3(0.2, 1.75, 0));
}

/// Iterative intercept point (with gravity drop compensation) for a target.
fn leadPoint(t: *const Target, from: V3) struct { aim: V3, tof: f32 } {
    var aim = t.pos;
    var tof: f32 = 0;
    var k: usize = 0;
    while (k < 4) : (k += 1) {
        tof = vm.dist(aim, from) / SHELL_SPEED;
        aim = vm.mad(t.pos, t.vel, tof);
    }
    aim.y += 0.5 * G * tof * tof;
    return .{ .aim = aim, .tof = tof };
}

/// Visible to the gunner's optics: alive, not ours, in range and in line of sight.
fn gunnerSees(t: *const Target, eye: V3) bool {
    if (!t.alive or t.dying or t.player) return false;
    const d = vm.dist(t.pos, eye);
    return d < 4500 and lineOfSight(eye, t.pos);
}

fn updateGunManual(dt: f32, gunPos: V3) void {
    // laser rangefinder: lock the target closest to the boresight (or its lead point)
    const eye = gunnerEye();
    const dir = gunnerDir();
    var best: i32 = -1;
    var bestA: f32 = 3.5 * vm.deg;
    for (&targets, 0..) |*t, i| {
        if (!gunnerSees(t, eye)) continue;
        const lp = leadPoint(t, gunPos);
        const a1 = std.math.acos(vm.clampf(vm.dot(vm.norm(vm.sub(t.pos, eye)), dir), -1, 1));
        const a2 = std.math.acos(vm.clampf(vm.dot(vm.norm(vm.sub(lp.aim, eye)), dir), -1, 1));
        const a = @min(a1, a2);
        if (a < bestA) {
            bestA = a;
            best = @intCast(i);
        }
    }
    g.gLock = best;
    // fuse on the intercept point, not the current target position
    if (best >= 0) g.gRange = leadPoint(&targets[@intCast(best)], gunPos).tof * SHELL_SPEED;

    g.gYaw = vm.approachAngle(g.gYaw, g.gAimYaw, 120 * vm.deg * dt);
    g.gPitch = vm.approachAngle(g.gPitch, vm.clampf(g.gAimPitch, -0.05, 1.45), 90 * vm.deg * dt);

    g.gFireT -= dt;
    if (g.gTrigger and g.gAmmo > 0) {
        while (g.gFireT <= 0) {
            g.gFireT += 0.055;
            fireShell(g.gRange / SHELL_SPEED);
        }
    }
    if (g.gFireT < 0) g.gFireT = 0;
}

fn enterGunner(on: bool) void {
    g.gunner = on;
    if (on) {
        g.gAimYaw = g.gYaw;
        g.gAimPitch = @max(g.gPitch, 0.05);
        g.warp = 1;
        g.gTrigger = false;
        if (g.autoSam) {
            g.autoSam = false;
            msg("Наводчик ЗСУ. Авто-ЗРК выключен (T — включить)", HUD_AMBER);
        } else msg("Наводчик ЗСУ", HUD_AMBER);
        c.DisableCursor();
    } else {
        g.gTrigger = false;
        c.EnableCursor();
        msg("Командирский обзор", HUD_DIM);
    }
}

var noMouse = false; // screenshot runs: ignore whatever the real mouse does

fn gunnerInput() void {
    const md = if (noMouse) vm.v2(0, 0) else c.GetMouseDelta();
    const inv: f32 = if (save.settings.invertY == 1) -1 else 1;
    const k = 0.0009 * @as(f32, @floatFromInt(save.settings.sens + 1)) * (g.gFov / 40);
    g.gAimYaw = vm.wrapPi(g.gAimYaw - md.x * k);
    g.gAimPitch = vm.clampf(g.gAimPitch - md.y * k * inv, -0.05, 1.45);
    const wheel = c.GetMouseWheelMove();
    g.gFov = vm.clampf(g.gFov * @exp(-wheel * 0.15), 4, 55);
    if (c.IsKeyPressed(c.KEY_Z)) g.gFov = if (g.gFov > 12) 8 else 40;
    g.gTrigger = c.IsMouseButtonDown(c.MOUSE_BUTTON_LEFT);
    if (c.IsKeyPressed(c.KEY_F) or c.IsKeyPressed(c.KEY_SPACE)) {
        // hand the locked target to the SAM battery
        if (g.gLock >= 0 and tracked(&targets[@intCast(g.gLock)])) {
            setSelected(g.gLock);
            requestLaunch();
            msg("ЗРК: пуск по цели в прицеле", HUD);
        } else msg("Наведи прицел на цель с радарной отметкой", HUD_DIM);
    }
    if (c.IsKeyPressed(c.KEY_T)) {
        g.autoSam = !g.autoSam;
        msg(if (g.autoSam) "ЗРК: автоматический режим" else "ЗРК: ручной режим", HUD_DIM);
    }
    if (c.IsKeyPressed(c.KEY_N) and g.toSpawn == 0 and g.waveT > 1) g.waveT = 0.5;
}

fn gunnerCamera() c.Camera3D {
    const eye = gunnerEye();
    g.camPos = eye;
    g.camLook = vm.mad(eye, gunnerDir(), 100);
    return .{ .position = eye, .target = g.camLook, .up = v3(0, 1, 0), .fovy = g.gFov, .projection = c.CAMERA_PERSPECTIVE };
}

fn fireShell(tof: f32) void {
    if (g.gAmmo <= 0) return;
    g.gAmmo -= 1;
    const xf = spaagGunsXf();
    const side: f32 = if (g.gBarrel == 0) -1.42 else 1.42;
    g.gBarrel +%= 1;
    const muzzle = xf.point(v3(4.2, 0.05, side));
    const disp = vm.scale(vm.rndDir(), 0.0025);
    const dir = vm.norm(vm.add(xf.x, disp));
    for (&shells) |*s| {
        if (s.alive) continue;
        s.* = .{ .alive = true, .pos = muzzle, .vel = vm.scale(dir, SHELL_SPEED), .fuse = tof + vm.rr(-0.02, 0.02) };
        break;
    }
    spawnPart(.{ .pos = muzzle, .life = 0.06, .s0 = 3.0, .s1 = 1.5, .c0 = .{ 1, 0.85, 0.5, 1 }, .c1 = .{ 1, 0.5, 0.1, 0 }, .add = true });
    if (vm.rnd() < 0.4) smoke(muzzle, vm.add(vm.scale(dir, 6), vm.scale(vm.rndDir(), 1)), 1.5, 0.8, 3.5, 0.75, 0.3);
}

fn updateShells(dt: f32) void {
    for (&shells) |*s| {
        if (!s.alive) continue;
        s.t += dt;
        s.vel.y -= G * dt;
        s.pos = vm.mad(s.pos, s.vel, dt);
        if (s.t >= s.fuse or s.t > 4) {
            s.alive = false;
            // AHEAD-style airburst: dark flak puff + fragment cone
            smoke(s.pos, vm.scale(vm.rndDir(), 1), vm.rr(3, 5), 1.5, 5.5, 0.13, 0.85);
            spawnPart(.{ .pos = s.pos, .life = 0.08, .s0 = 3.5, .s1 = 1, .c0 = .{ 1, 0.8, 0.5, 1 }, .c1 = .{ 1, 0.4, 0.1, 0 }, .add = true });
            for (&targets, 0..) |*t, i| {
                if (!t.alive or t.dying) continue;
                const d = vm.dist(t.pos, s.pos);
                const hitK: f32 = if (t.player) diffK(0.15, 0.3, 0.45) else 0.55;
                if (d < 10 and vm.rnd() < hitK * (1 - d / 10)) {
                    t.hp -= 1;
                    if (t.hp <= 0) killTarget(i, false);
                }
            }
            continue;
        }
        if (s.pos.y < heightAt(s.pos.x, s.pos.z)) s.alive = false;
    }
}

// ============================================================ objects fx
fn updateObjects(dt: f32) void {
    for (&objs) |*o| {
        if (o.nfires == 0) continue;
        o.fireT -= dt;
        if (o.fireT > 0) continue;
        o.fireT = 0.07;
        for (o.fires[0..o.nfires]) |fp| {
            const p = vm.add(fp, v3(vm.rr(-4, 4), vm.rr(0, 3), vm.rr(-4, 4)));
            fire(p, v3(0, vm.rr(3, 7), 0), vm.rr(0.4, 0.9), 5, 2, 1.2);
            smoke(vm.add(p, v3(0, 4, 0)), v3(vm.rr(-1, 1), vm.rr(5, 9), vm.rr(-1, 1)), vm.rr(9, 15), 6, vm.rr(30, 45), vm.rr(0.08, 0.16), 0.75);
        }
    }
}

// ============================================================ update
fn update(dt: f32) void {
    g.time += dt;
    for (&g.msgs) |*m| m.t -= dt;

    // waves
    var aliveN: i32 = 0;
    for (&targets) |*t| {
        if (t.alive) aliveN += 1;
    }
    if (mode == .attack) {
        if (g.playerDead) {
            g.endT -= dt;
            if (g.endT <= 0 and app == .playing) finishSortie();
        }
    } else if (!g.over) {
        if (g.toSpawn > 0) {
            g.spawnT -= dt;
            if (g.spawnT <= 0) {
                g.spawnT = vm.rr(1.5, 4.5);
                const w = g.wave;
                const r = vm.rnd();
                var kind: TKind = .drone;
                if (w >= 2 and r < 0.18 + 0.02 * @as(f32, @floatFromInt(w))) kind = .cruise else if (w >= 3 and r > 0.75) kind = .decoy;
                spawnTarget(kind, if (kind == .cruise) 7600 else 7300);
                g.toSpawn -= 1;
            }
        } else if (aliveN == 0) {
            g.waveT -= dt;
            if (g.waveT <= 0) {
                startWave();
                g.waveT = 12;
            }
        }
    }

    updateTargets(dt);
    updateRadar(dt);
    updateSam(dt);
    updateMissiles(dt);
    updateGun(dt);
    updateShells(dt);
    updateObjects(dt);
    updateParts(dt);

    var allDead = true;
    for (&objs) |*o| {
        if (o.hp > 0) allDead = false;
    }
    if (allDead and !g.over and mode == .defense) {
        g.over = true;
        g.endT = 3;
        msg("Все объекты уничтожены", HUD_RED);
    }
    if (g.over and mode == .defense) {
        g.endT -= dt;
        if (g.endT <= 0 and app == .playing) {
            updateRecords();
            save.game.valid = 0;
            writeSave();
            app = .result;
            menuSel = 0;
            c.EnableCursor();
        }
    }
    if (g.selected >= 0 and !targetValid(g.selected, g.selId)) g.selected = -1;
}

// ============================================================ input & camera
fn selectNext() void {
    // cycle tracked targets ordered by distance
    var curD: f32 = -1;
    if (g.selected >= 0 and targetValid(g.selected, g.selId)) curD = vm.len(targets[@intCast(g.selected)].pos);
    var best: i32 = -1;
    var bd: f32 = 1e9;
    var first: i32 = -1;
    var fd: f32 = 1e9;
    for (&targets, 0..) |*t, i| {
        if (!tracked(t) or t.dying) continue;
        const d = vm.len(t.pos);
        if (d > curD + 0.01 and d < bd) {
            bd = d;
            best = @intCast(i);
        }
        if (d < fd) {
            fd = d;
            first = @intCast(i);
        }
    }
    if (best < 0) best = first;
    setSelected(best);
}

fn setSelected(i: i32) void {
    g.selected = i;
    if (i >= 0) g.selId = targets[@intCast(i)].id;
}

const Scope = struct { cx: f32, cy: f32, r: f32 };
fn scopeRect() Scope {
    const h: f32 = @floatFromInt(c.GetScreenHeight());
    const r: f32 = @min(150, h * 0.18);
    return .{ .cx = 24 + r, .cy = h - 24 - r, .r = r };
}

fn handleInput(cam: c.Camera3D, dtReal: f32) void {
    if (mode == .defense and c.IsKeyPressed(c.KEY_V)) enterGunner(!g.gunner);
    if (g.gunner) {
        gunnerInput();
        return;
    }
    if (c.IsKeyPressed(c.KEY_X)) g.warp = if (g.warp >= 4) 1 else g.warp * 2;
    if (mode == .attack) {
        const wheel = c.GetMouseWheelMove();
        g.camDist = vm.clampf(g.camDist * @exp(-wheel * 0.12), 10, 120);
        if (c.IsKeyPressed(c.KEY_C)) g.camMode = (g.camMode + 1) % 2;
        return;
    }
    if (c.IsKeyPressed(c.KEY_TAB)) selectNext();
    if (c.IsKeyPressed(c.KEY_F) or c.IsKeyPressed(c.KEY_SPACE)) requestLaunch();
    if (c.IsKeyPressed(c.KEY_T)) {
        g.autoSam = !g.autoSam;
        msg(if (g.autoSam) "ЗРК: автоматический режим" else "ЗРК: ручной режим", HUD_DIM);
    }
    if (c.IsKeyPressed(c.KEY_G)) {
        g.gAuto = !g.gAuto;
        msg(if (g.gAuto) "ЗСУ: огонь разрешён" else "ЗСУ: огонь запрещён", HUD_DIM);
    }
    if (c.IsKeyPressed(c.KEY_C)) g.camMode = (g.camMode + 1) % 3;
    if (c.IsKeyPressed(c.KEY_P)) g.paused = !g.paused;
    if (c.IsKeyPressed(c.KEY_H)) {
        g.camFocus = v3(0, 10, 0);
        g.camDist = 140;
        g.camMode = 0;
    }
    if (c.IsKeyPressed(c.KEY_N) and g.toSpawn == 0 and g.waveT > 1) g.waveT = 0.5;

    // mouse picking
    if (c.IsMouseButtonPressed(c.MOUSE_BUTTON_LEFT)) {
        const mp = c.GetMousePosition();
        const sc = scopeRect();
        var best: i32 = -1;
        var bd: f32 = 40;
        const inScope = (mp.x - sc.cx) * (mp.x - sc.cx) + (mp.y - sc.cy) * (mp.y - sc.cy) < sc.r * sc.r;
        for (&targets, 0..) |*t, i| {
            if (!tracked(t)) continue;
            var sp: c.Vector2 = undefined;
            if (inScope) {
                sp = scopePoint(sc, t.seenPos);
            } else {
                if (vm.dot(vm.sub(t.pos, cam.position), vm.sub(cam.target, cam.position)) <= 0) continue;
                sp = c.GetWorldToScreen(t.pos, cam);
            }
            const d = @sqrt((sp.x - mp.x) * (sp.x - mp.x) + (sp.y - mp.y) * (sp.y - mp.y));
            if (d < bd) {
                bd = d;
                best = @intCast(i);
            }
        }
        if (best >= 0) setSelected(best);
    }

    // orbit camera
    if (c.IsMouseButtonDown(c.MOUSE_BUTTON_RIGHT) or c.IsMouseButtonDown(c.MOUSE_BUTTON_MIDDLE)) {
        const d = c.GetMouseDelta();
        g.camYaw -= d.x * 0.005;
        g.camPitch = vm.clampf(g.camPitch + d.y * 0.004, -0.2, 1.45);
    }
    const wheel = c.GetMouseWheelMove();
    g.camDist = vm.clampf(g.camDist * @exp(-wheel * 0.12), 12, 6000);
    if (g.camMode == 0) {
        const spd = g.camDist * 1.2 * dtReal;
        const fwd = v3(-@sin(g.camYaw), 0, -@cos(g.camYaw));
        const right = v3(@cos(g.camYaw), 0, -@sin(g.camYaw));
        if (c.IsKeyDown(c.KEY_W)) g.camFocus = vm.mad(g.camFocus, fwd, spd);
        if (c.IsKeyDown(c.KEY_S)) g.camFocus = vm.mad(g.camFocus, fwd, -spd);
        if (c.IsKeyDown(c.KEY_D)) g.camFocus = vm.mad(g.camFocus, right, spd);
        if (c.IsKeyDown(c.KEY_A)) g.camFocus = vm.mad(g.camFocus, right, -spd);
        g.camFocus.x = vm.clampf(g.camFocus.x, -6000, 6000);
        g.camFocus.z = vm.clampf(g.camFocus.z, -6000, 6000);
        g.camFocus.y = vm.lerpf(g.camFocus.y, heightAt(g.camFocus.x, g.camFocus.z) + 8, @min(1, 6 * dtReal));
    }
}

fn attackCamera(dtReal: f32) c.Camera3D {
    var pos: V3 = undefined;
    var look: V3 = undefined;
    if (playerTarget()) |t| {
        if (g.camMode == 1) {
            // nose camera
            const xf = vm.lookFwd(t.pos, t.vel, t.roll);
            pos = xf.point(v3(2.2, 0.35, 0));
            look = vm.mad(pos, xf.x, 50);
            g.camPos = pos;
            g.camLook = look;
            return .{ .position = pos, .target = look, .up = xf.y, .fovy = 70, .projection = c.CAMERA_PERSPECTIVE };
        }
        const fwd = vm.yawPitch(vm.zero, g.pYaw, g.pPitch * 0.6).x;
        pos = vm.add(vm.mad(t.pos, fwd, -g.camDist), v3(0, g.camDist * 0.22 + 1.5, 0));
        look = vm.mad(t.pos, fwd, 25);
        const hmin = heightAt(pos.x, pos.z) + 2;
        if (pos.y < hmin) pos.y = hmin;
        g.camPos = vm.lerp(g.camPos, pos, @min(1, 10 * dtReal));
        g.camLook = vm.lerp(g.camLook, look, @min(1, 14 * dtReal));
    } else {
        // after the drone is gone: slow orbit around where it ended
        g.camYaw += dtReal * 0.25;
        pos = vm.add(g.endPos, v3(@sin(g.camYaw) * 110, 45, @cos(g.camYaw) * 110));
        pos.y = @max(pos.y, heightAt(pos.x, pos.z) + 5);
        g.camPos = vm.lerp(g.camPos, pos, @min(1, 3 * dtReal));
        g.camLook = vm.lerp(g.camLook, g.endPos, @min(1, 5 * dtReal));
    }
    return .{ .position = g.camPos, .target = g.camLook, .up = v3(0, 1, 0), .fovy = 62, .projection = c.CAMERA_PERSPECTIVE };
}

fn menuCamera(dtReal: f32) c.Camera3D {
    g.camYaw += dtReal * 0.035;
    const pos = v3(@sin(g.camYaw) * 95, heightAt(@sin(g.camYaw) * 95, @cos(g.camYaw) * 95) + 16, @cos(g.camYaw) * 95);
    g.camPos = pos;
    g.camLook = v3(-10, 6, 5);
    g.camFocus = v3(0, 10, 0);
    return .{ .position = pos, .target = g.camLook, .up = v3(0, 1, 0), .fovy = 50, .projection = c.CAMERA_PERSPECTIVE };
}

fn updateCamera(dtReal: f32) c.Camera3D {
    if (mode == .attack) return attackCamera(dtReal);
    if (g.gunner) return gunnerCamera();
    var focus = g.camFocus;
    var dist = g.camDist;
    var cm = g.camMode;
    if (cm == 1) {
        if (g.lastMissile >= 0 and missiles[@intCast(g.lastMissile)].alive) {
            const m = &missiles[@intCast(g.lastMissile)];
            focus = m.pos;
            dist = @min(g.camDist, 60);
        } else cm = 0;
    }
    if (cm == 2) {
        if (g.selected >= 0 and targetValid(g.selected, g.selId)) {
            focus = targets[@intCast(g.selected)].pos;
            dist = @min(g.camDist, 50);
        } else cm = 0;
    }
    const cp = @cos(g.camPitch);
    const off = v3(@sin(g.camYaw) * cp, @sin(g.camPitch), @cos(g.camYaw) * cp);
    var pos = vm.mad(focus, off, dist);
    const hmin = heightAt(pos.x, pos.z) + 3;
    if (pos.y < hmin) pos.y = hmin;
    const k: f32 = if (cm == 0) @min(1, 12 * dtReal) else @min(1, 9 * dtReal);
    g.camPos = vm.lerp(g.camPos, pos, k);
    g.camLook = vm.lerp(g.camLook, focus, @min(1, 14 * dtReal));
    if (cm != 0) {
        g.camPos = pos;
        g.camLook = focus;
    }
    return .{ .position = g.camPos, .target = g.camLook, .up = v3(0, 1, 0), .fovy = 55, .projection = c.CAMERA_PERSPECTIVE };
}

// ============================================================ rendering
fn drawXf(model: c.Model, xf: Xf, s: f32) void {
    var m = model;
    m.transform = xf.toMatrix(s);
    c.DrawModel(m, vm.zero, 1, WHITE);
}

fn drawYaw(model: c.Model, pos: V3, yaw: f32, s: f32) void {
    drawXf(model, vm.yawPitch(pos, yaw, 0), s);
}

var camFwdG: V3 = v3(1, 0, 0);

fn drawScene(camPos: V3, shadowPass: bool, focus: V3) void {
    // fog makes anything past ~3.5 km invisible anyway
    const maxD2: f32 = if (shadowPass) 450 * 450 else 3600 * 3600;
    const ref = if (shadowPass) focus else camPos;
    // trees
    var i: usize = 0;
    while (i < ntrees) : (i += 1) {
        const t = trees[i];
        const dx = t.pos.x - ref.x;
        const dz = t.pos.z - ref.z;
        const d2 = dx * dx + dz * dz;
        if (d2 > maxD2) continue;
        // cheap view-cone cull (skip trees behind the camera)
        if (!shadowPass and d2 > 60 * 60) {
            const inv = 1 / @sqrt(d2);
            if (dx * inv * camFwdG.x + dz * inv * camFwdG.z < 0.35) continue;
        }
        c.DrawModelEx(if (t.pine) M.pine else M.leaf, t.pos, v3(0, 1, 0), t.yaw, v3(t.s, t.s, t.s), WHITE);
    }
    // props (generated meshes: draw double-sided, the shader flips back-face normals)
    c.rlDisableBackfaceCulling();
    defer c.rlEnableBackfaceCulling();
    i = 0;
    while (i < nprops) : (i += 1) {
        const p = props[i];
        const dx = p.pos.x - ref.x;
        const dz = p.pos.z - ref.z;
        if (shadowPass and dx * dx + dz * dz > 900 * 900) continue;
        const m = switch (p.kind) {
            .power => M.power,
            .depot => M.depot,
            .apartment => M.apartment,
            .house => M.house,
        };
        drawYaw(m, p.pos, p.yaw, p.s);
    }
    // battery
    for (&g.launchers) |*l| {
        drawYaw(M.launcher_base, l.pos, l.yaw, 1);
        drawXf(M.launcher_turret, launcherXf(l), 1);
    }
    const rb = vm.yawPitch(radarPos, radarYaw, 0);
    drawXf(M.radar_base, rb, 1);
    drawXf(M.radar_head, rb.mul(vm.yawPitch(RADAR_ON_HULL, g.rYaw - radarYaw, 0)), 1);
    drawYaw(M.spaag_base, spaagPos, spaagYaw, 1);
    if (!g.gunner or shadowPass) {
        drawXf(M.spaag_turret, spaagTurretXf(), 1);
        drawXf(M.spaag_guns, spaagGunsXf(), 1);
    }
    // airborne
    for (&targets) |*t| {
        if (!t.alive) continue;
        if (t.player and g.camMode == 1 and !shadowPass) continue;
        const m = switch (t.kind) {
            .drone => M.drone,
            .decoy => M.drone,
            .cruise => M.cruise,
        };
        drawXf(m, vm.lookFwd(t.pos, t.vel, t.roll), if (t.kind == .decoy) 0.8 else 1);
    }
    for (&missiles) |*m| {
        if (!m.alive) continue;
        drawXf(M.sam, vm.lookFwd(m.pos, m.vel, 0), 1);
    }
}

fn drawTracers(cam: c.Camera3D) void {
    c.BeginBlendMode(c.BLEND_ADDITIVE);
    c.rlDisableDepthMask();
    for (&shells) |*s| {
        if (!s.alive) continue;
        const tail = vm.mad(s.pos, s.vel, -0.035);
        const f = 1 - fogFactor(s.pos, cam.position);
        c.DrawLine3D(tail, s.pos, colf(1.0 * f, 0.75 * f, 0.35 * f, 1));
        c.DrawBillboard(cam, puff, s.pos, 0.9, colf(1.0 * f, 0.7 * f, 0.3 * f, 1));
    }
    c.rlEnableDepthMask();
    c.EndBlendMode();
}

fn renderShadow(focus: V3) c.Matrix {
    const center = v3(@round(focus.x / 4) * 4, heightAt(focus.x, focus.z), @round(focus.z / 4) * 4);
    const lc = c.Camera3D{
        .position = vm.mad(center, sunDir, 1500),
        .target = center,
        .up = v3(0, 1, 0),
        .fovy = 520,
        .projection = c.CAMERA_ORTHOGRAPHIC,
    };
    c.BeginTextureMode(shadowRT);
    c.ClearBackground(WHITE);
    c.rlSetClipPlanes(10, 3500);
    c.BeginMode3D(lc);
    const view = c.rlGetMatrixModelview();
    const proj = c.rlGetMatrixProjection();
    if (save.settings.shadows == 1) drawScene(center, true, center);
    c.EndMode3D();
    c.EndTextureMode();
    c.rlSetClipPlanes(1.0, 30000);
    return matMul(view, proj);
}

fn camFwdFlat(cam: c.Camera3D) V3 {
    const f = vm.sub(cam.target, cam.position);
    return vm.norm(v3(f.x, 0, f.z));
}

fn render(cam: c.Camera3D) void {
    const focus = if (g.gunner) vm.mad(cam.position, camFwdFlat(cam), 120) else if (g.camMode == 0 and mode == .defense and app != .menu) g.camFocus else g.camLook;
    const lightVP = renderShadow(focus);
    {
        const f = vm.sub(cam.target, cam.position);
        camFwdG = vm.norm(v3(f.x, 0, f.z));
    }

    c.BeginDrawing();
    c.ClearBackground(colf(fogDisp[0], fogDisp[1], fogDisp[2], 1));
    c.rlSetClipPlanes(1.0, 30000);
    c.BeginMode3D(cam);

    const vp = [3]f32{ cam.position.x, cam.position.y, cam.position.z };
    c.SetShaderValue(lit, L.viewPos, &vp, c.SHADER_UNIFORM_VEC3);
    c.SetShaderValue(sky, skyView, &vp, c.SHADER_UNIFORM_VEC3);
    setF(sky, skyTime, renderTime);
    c.SetShaderValueMatrix(lit, L.lightVP, lightVP);
    c.rlActiveTextureSlot(SHADOW_SLOT);
    c.rlEnableTexture(shadowRT.depth.id);
    c.rlActiveTextureSlot(0);
    setI(lit, L.shadowMap, SHADOW_SLOT);

    // sky dome
    c.rlDisableBackfaceCulling();
    c.rlDisableDepthMask();
    drawXf(M.sky, .{ .p = cam.position }, 20000);
    c.rlEnableDepthMask();
    c.rlEnableBackfaceCulling();

    // terrain
    setI(lit, L.isTerrain, if (terrainDebug) 2 else 1);
    c.DrawModel(M.terrain, vm.zero, 1, WHITE);
    setI(lit, L.isTerrain, 0);

    drawScene(cam.position, false, focus);
    drawTracers(cam);
    drawParts(cam);
    if (save.settings.rain == 1) drawRain(cam);
    c.EndMode3D();

    switch (app) {
        .playing => if (mode == .attack) drawHudAttack(cam) else if (g.gunner) drawHudGunner(cam) else drawHud(cam),
        .paused => {
            if (mode == .attack) drawHudAttack(cam) else if (g.gunner) drawHudGunner(cam) else drawHud(cam);
            drawPause();
        },
        else => drawMenus(),
    }
    txtR(c.TextFormat("%d FPS", c.GetFPS()), @as(f32, @floatFromInt(c.GetScreenWidth())) - 8, 2, 12, HUD_DIM);
    if (debugShadow) {
        var t = shadowRT.depth;
        t.format = c.PIXELFORMAT_UNCOMPRESSED_R32;
        c.DrawTexturePro(t, .{ .x = 0, .y = 0, .width = SHADOW_SIZE, .height = SHADOW_SIZE }, .{ .x = 300, .y = 150, .width = 500, .height = 500 }, vm.v2(0, 0), 0, WHITE);
    }
    c.EndDrawing();
}

// ============================================================ HUD
fn scopePoint(sc: Scope, p: V3) c.Vector2 {
    const k = sc.r / RADAR_RANGE;
    return vm.v2(sc.cx + p.x * k, sc.cy + p.z * k);
}

fn kindName(t: *const Target) [*c]const u8 {
    if (!t.classified) return "ЦЕЛЬ";
    return switch (t.kind) {
        .drone => "БПЛА",
        .cruise => "КР",
        .decoy => "ЛОЖН",
    };
}

fn trackColor(t: *const Target) c.Color {
    if (!t.classified) return HUD_AMBER;
    return if (t.kind == .decoy) col(150, 160, 160, 255) else HUD_RED;
}

fn bracket(x: f32, y: f32, s: f32, color: c.Color, th: f32) void {
    const l = s * 0.45;
    const pts = [_][4]f32{
        .{ -1, -1, 1, 0 }, .{ -1, -1, 0, 1 },
        .{ 1, -1, -1, 0 }, .{ 1, -1, 0, 1 },
        .{ -1, 1, 1, 0 },  .{ -1, 1, 0, -1 },
        .{ 1, 1, -1, 0 },  .{ 1, 1, 0, -1 },
    };
    for (pts) |p| {
        const ax = x + p[0] * s;
        const ay = y + p[1] * s;
        c.DrawLineEx(vm.v2(ax, ay), vm.v2(ax + p[2] * l, ay + p[3] * l), th, color);
    }
}

fn drawHud(cam: c.Camera3D) void {
    const W: f32 = @floatFromInt(c.GetScreenWidth());
    const H: f32 = @floatFromInt(c.GetScreenHeight());
    const camFwd = vm.norm(vm.sub(cam.target, cam.position));

    // ---- world markers
    for (&targets, 0..) |*t, i| {
        if (!tracked(t) or t.dying) continue;
        if (vm.dot(vm.sub(t.pos, cam.position), camFwd) <= 0) continue;
        const sp = c.GetWorldToScreen(t.pos, cam);
        if (sp.x < -50 or sp.x > W + 50 or sp.y < -50 or sp.y > H + 50) continue;
        const sel = g.selected == @as(i32, @intCast(i));
        const cc = trackColor(t);
        const d = vm.dist(t.pos, cam.position);
        const s: f32 = vm.clampf(900 / @max(d, 1) * 6, 9, 30);
        bracket(sp.x, sp.y, if (sel) s + 6 else s, cc, if (sel) 2.5 else 1.4);
        txt(c.TextFormat("#%02d %s", @as(c_int, @intCast(t.id)), kindName(t)), sp.x + s + 6, sp.y - s, 15, cc);
        if (sel) {
            txt(c.TextFormat("%.1f км  %d м", @as(f64, vm.len(t.pos) / 1000), @as(c_int, @intFromFloat(t.pos.y - heightAt(t.pos.x, t.pos.z)))), sp.x + s + 6, sp.y - s + 15, 14, cc);
            const nm = missilesOn(t.id);
            if (nm > 0) txt(c.TextFormat("ЗУР x%d", @as(c_int, nm)), sp.x + s + 6, sp.y - s + 29, 14, HUD);
        }
    }
    // missile markers
    for (&missiles) |*m| {
        if (!m.alive) continue;
        if (vm.dot(vm.sub(m.pos, cam.position), camFwd) <= 0) continue;
        const sp = c.GetWorldToScreen(m.pos, cam);
        c.DrawCircleLines(@intFromFloat(sp.x), @intFromFloat(sp.y), 4, HUD);
    }

    // ---- top-left status
    c.DrawRectangleRounded(.{ .x = 16, .y = 16, .width = 270, .height = 112 }, 0.08, 6, PANEL);
    txt("ПВО  ·  ОБОРОНА ОБЪЕКТОВ", 28, 24, 18, HUD);
    txt(c.TextFormat("Волна  %d", @as(c_int, g.wave)), 28, 50, 17, HUD);
    txt(c.TextFormat("Счёт  %d", @as(c_int, g.score)), 150, 50, 17, HUD);
    txt(c.TextFormat("Сбито  %d", @as(c_int, g.kills)), 28, 72, 17, HUD);
    txt(c.TextFormat("Прилёты  %d", @as(c_int, g.leaks)), 150, 72, 17, if (g.leaks > 0) HUD_RED else HUD);
    txt(c.TextFormat("Время x%d%s", @as(c_int, @intFromFloat(g.warp)), @as([*c]const u8, if (g.paused) "   ПАУЗА" else "")), 28, 96, 15, HUD_DIM);
    const camNames = [_][*c]const u8{ "обзор", "ракета", "цель" };
    txt(c.TextFormat("Камера: %s", camNames[g.camMode]), 150, 96, 15, HUD_DIM);

    drawObjBars(W);
    drawMsgs(W, 66);

    // ---- wave banner
    if (!g.over and g.toSpawn == 0) {
        var aliveN: i32 = 0;
        for (&targets) |*t| {
            if (t.alive) aliveN += 1;
        }
        if (aliveN == 0) {
            txtC(c.TextFormat("Волна %d через %d с   (N — сразу)", @as(c_int, g.wave + 1), @as(c_int, @intFromFloat(@ceil(g.waveT)))), W / 2, H * 0.28, 24, HUD);
        }
    }

    drawScope();
    drawTrackTable(W);
    drawWeapons(W, H);

    // ---- help line
    const help = "ПКМ — вращение   Колесо — зум   WASD — смещение   TAB/клик — цель   F/Пробел — пуск ЗУР   T — авто-ЗРК   G — ЗСУ   V — наводчик ЗСУ   C — камера   X — ускорение   P — пауза";
    const hw = txtW(help, 14);
    c.DrawRectangleRounded(.{ .x = W / 2 - hw / 2 - 12, .y = H - 26, .width = hw + 24, .height = 21 }, 0.4, 4, PANEL);
    txtC(help, W / 2, H - 23, 14, HUD_DIM);

    if (g.over) {
        c.DrawRectangle(0, 0, @intFromFloat(W), @intFromFloat(H), col(0, 0, 0, 120));
        txtC("ОБЪЕКТЫ УНИЧТОЖЕНЫ", W / 2, H / 2 - 40, 44, HUD_RED);
        txtC(c.TextFormat("Волна %d   Счёт %d   Сбито %d", @as(c_int, g.wave), @as(c_int, g.score), @as(c_int, g.kills)), W / 2, H / 2 + 14, 22, HUD);
    }
}

fn drawScope() void {
    const sc = scopeRect();
    const cx: c_int = @intFromFloat(sc.cx);
    const cy: c_int = @intFromFloat(sc.cy);
    c.DrawCircle(cx, cy, sc.r + 8, PANEL);
    c.DrawCircle(cx, cy, sc.r, col(4, 22, 12, 210));
    // sweep wedge with fading trail
    var k: usize = 0;
    while (k < 28) : (k += 1) {
        const a0 = g.rYaw - @as(f32, @floatFromInt(k)) * 0.03;
        const alpha: u8 = @intFromFloat(70 * (1 - @as(f32, @floatFromInt(k)) / 28));
        // world yaw -> screen angle: screen = (cos(yaw), -sin(yaw)) in x/z => degrees for raylib (clockwise from +x)
        const s0 = -a0 / vm.deg;
        c.DrawCircleSector(vm.v2(sc.cx, sc.cy), sc.r, s0, s0 + 1.8, 2, col(120, 255, 150, alpha));
    }
    const ex = sc.cx + @cos(g.rYaw) * sc.r;
    const ey = sc.cy - @sin(g.rYaw) * sc.r;
    c.DrawLineEx(vm.v2(sc.cx, sc.cy), vm.v2(ex, ey), 1.5, col(160, 255, 180, 220));
    // range rings
    for ([_]f32{ 3000, 6000, 9000 }) |rr| {
        c.DrawCircleLines(cx, cy, sc.r * rr / RADAR_RANGE, col(90, 200, 120, 90));
    }
    c.DrawCircleLines(cx, cy, sc.r * GUN_RANGE / RADAR_RANGE, col(255, 200, 80, 60));
    c.DrawLine(cx - @as(c_int, @intFromFloat(sc.r)), cy, cx + @as(c_int, @intFromFloat(sc.r)), cy, col(90, 200, 120, 50));
    c.DrawLine(cx, cy - @as(c_int, @intFromFloat(sc.r)), cx, cy + @as(c_int, @intFromFloat(sc.r)), col(90, 200, 120, 50));
    txt("С", sc.cx - 4, sc.cy - sc.r - 4 - 14, 14, HUD_DIM);
    txt("9 км", sc.cx + sc.r * 0.72, sc.cy - sc.r * 0.72 - 6, 12, HUD_DIM);
    // objects
    for (&objs) |*o| {
        const p = scopePoint(sc, o.pos);
        c.DrawRectangleLines(@intFromFloat(p.x - 3), @intFromFloat(p.y - 3), 7, 7, if (o.hp > 0) HUD else HUD_RED);
    }
    c.DrawCircle(cx, cy, 2.5, HUD);
    // tracks: blip brightness decays since last sweep hit
    for (&targets, 0..) |*t, i| {
        if (!tracked(t)) continue;
        const p = scopePoint(sc, t.seenPos);
        const age = g.time - t.seenT;
        const a: f32 = vm.clampf(1 - age / 3.2, 0.25, 1);
        var cc = trackColor(t);
        cc.a = @intFromFloat(a * 255);
        c.DrawCircleV(p, 3, cc);
        // velocity leader
        const lv = vm.scale(vm.norm(t.vel), 10);
        c.DrawLineV(p, vm.v2(p.x + lv.x, p.y + lv.z), cc);
        if (g.selected == @as(i32, @intCast(i))) c.DrawRectangleLines(@intFromFloat(p.x - 6), @intFromFloat(p.y - 6), 13, 13, WHITE);
    }
    for (&missiles) |*m| {
        if (!m.alive) continue;
        const p = scopePoint(sc, m.pos);
        c.DrawCircleV(p, 1.8, WHITE);
    }
    txt("РЛС  ·  9 км", sc.cx - sc.r, sc.cy + sc.r + 2, 13, HUD_DIM);
}

fn drawTrackTable(W: f32) void {
    // collect up to 10 tracks sorted by range
    var idx: [10]usize = undefined;
    var n: usize = 0;
    var used: [MAX_TGT]bool = [_]bool{false} ** MAX_TGT;
    while (n < idx.len) {
        var best: i32 = -1;
        var bd: f32 = 1e12;
        for (&targets, 0..) |*t, i| {
            if (used[i] or !tracked(t) or t.dying) continue;
            const d = vm.len(t.pos);
            if (d < bd) {
                bd = d;
                best = @intCast(i);
            }
        }
        if (best < 0) break;
        used[@intCast(best)] = true;
        idx[n] = @intCast(best);
        n += 1;
    }
    const pw: f32 = 300;
    const px = W - pw - 16;
    const ph: f32 = 46 + @as(f32, @floatFromInt(@max(n, 1))) * 20;
    c.DrawRectangleRounded(.{ .x = px, .y = 16, .width = pw, .height = ph }, 0.05, 6, PANEL);
    txt("ЦЕЛИ", px + 12, 22, 17, HUD);
    txt("ТИП      Д,км   В,м   V,км/ч", px + 70, 25, 13, HUD_DIM);
    if (n == 0) txt("нет сопровождаемых", px + 12, 48, 15, HUD_DIM);
    for (idx[0..n], 0..) |ti, row| {
        const t = &targets[ti];
        const y = 46 + @as(f32, @floatFromInt(row)) * 20;
        const sel = g.selected == @as(i32, @intCast(ti));
        if (sel) c.DrawRectangle(@intFromFloat(px + 4), @intFromFloat(y - 1), @intFromFloat(pw - 8), 19, col(120, 255, 150, 40));
        const cc = trackColor(t);
        txt(c.TextFormat("#%02d", @as(c_int, @intCast(t.id))), px + 12, y, 15, cc);
        txt(kindName(t), px + 70, y, 15, cc);
        txt(c.TextFormat("%.1f", @as(f64, vm.len(t.pos) / 1000)), px + 128, y, 15, cc);
        txt(c.TextFormat("%d", @as(c_int, @intFromFloat(t.pos.y - heightAt(t.pos.x, t.pos.z)))), px + 175, y, 15, cc);
        txt(c.TextFormat("%d", @as(c_int, @intFromFloat(vm.len(t.vel) * 3.6))), px + 218, y, 15, cc);
        const nm = missilesOn(t.id);
        if (nm > 0) txt("ЗУР", px + 262, y, 13, HUD) else if (g.gTgt == @as(i32, @intCast(ti))) txt("ЗСУ", px + 262, y, 13, HUD_AMBER);
    }
}

fn drawWeapons(W: f32, H: f32) void {
    const pw: f32 = 300;
    const ph: f32 = 132;
    const px = W - pw - 16;
    const py = H - ph - 40;
    c.DrawRectangleRounded(.{ .x = px, .y = py, .width = pw, .height = ph }, 0.06, 6, PANEL);
    txt("ЗРК", px + 12, py + 8, 17, HUD);
    txt(if (g.autoSam) "АВТО" else "РУЧН", px + 54, py + 10, 14, if (g.autoSam) HUD_AMBER else HUD_DIM);
    txtR(c.TextFormat("резерв %d", @as(c_int, g.reserve)), px + pw - 12, py + 10, 14, HUD_DIM);
    for (&g.launchers, 0..) |*l, li| {
        const ly = py + 34 + @as(f32, @floatFromInt(li)) * 22;
        txt(c.TextFormat("ПУ-%d", @as(c_int, @intCast(li + 1))), px + 12, ly, 15, HUD_DIM);
        for (l.tubes, 0..) |tb, k| {
            const bx = px + 60 + @as(f32, @floatFromInt(k)) * 22;
            if (tb) c.DrawRectangle(@intFromFloat(bx), @intFromFloat(ly + 2), 16, 13, HUD) else c.DrawRectangleLines(@intFromFloat(bx), @intFromFloat(ly + 2), 16, 13, HUD_DIM);
        }
        if (l.reloadT > 0) {
            c.DrawRectangle(@intFromFloat(px + 160), @intFromFloat(ly + 6), @intFromFloat(120 * l.reloadT / 9), 5, HUD_AMBER);
            c.DrawRectangleLines(@intFromFloat(px + 160), @intFromFloat(ly + 5), 120, 7, HUD_DIM);
        }
    }
    const gy = py + 86;
    txt("ЗСУ", px + 12, gy, 17, HUD);
    txt(if (g.gAuto) "ОГОНЬ" else "ЗАПРЕТ", px + 54, gy + 2, 14, if (g.gAuto) HUD_AMBER else HUD_DIM);
    if (g.gAmmo > 0) {
        txtR(c.TextFormat("%d снар.", @as(c_int, g.gAmmo)), px + pw - 12, gy + 2, 14, HUD_DIM);
    } else {
        txtR(c.TextFormat("перезарядка %d с", @as(c_int, @intFromFloat(15 - g.gReload))), px + pw - 12, gy + 2, 14, HUD_AMBER);
    }
    c.DrawRectangle(@intFromFloat(px + 12), @intFromFloat(gy + 24), @intFromFloat((pw - 24) * @as(f32, @floatFromInt(g.gAmmo)) / 600), 5, HUD);
    if (g.selected >= 0) {
        const t = &targets[@intCast(g.selected)];
        txt(c.TextFormat("Выбрана #%02d  —  F: пуск", @as(c_int, @intCast(t.id))), px + 12, py - 22, 15, HUD);
    }
}

// ============================================================ attack HUD
fn blink(rate: f32) bool {
    return @mod(renderTime * rate, 1.0) < 0.6;
}

fn drawObjBars(W: f32) void {
    // ---- objects health (top center)
    const bw: f32 = 170;
    const total = bw * 3 + 24;
    var x0 = W / 2 - total / 2;
    for (&objs) |*o| {
        c.DrawRectangleRounded(.{ .x = x0, .y = 16, .width = bw, .height = 40 }, 0.15, 6, PANEL);
        txt(o.name, x0 + 10, 20, 15, if (o.hp > 0) HUD else HUD_RED);
        const f = o.hp / 100;
        c.DrawRectangle(@intFromFloat(x0 + 10), 41, @intFromFloat(bw - 20), 7, col(30, 40, 34, 220));
        const hc = if (f > 0.6) HUD else if (f > 0.3) HUD_AMBER else HUD_RED;
        c.DrawRectangle(@intFromFloat(x0 + 10), 41, @intFromFloat((bw - 20) * f), 7, hc);
        x0 += bw + 12;
    }
}

fn drawMsgs(W: f32, y0: f32) void {
    // ---- messages
    var my: f32 = y0;
    for (&g.msgs) |*m| {
        if (m.t <= 0) continue;
        const a: f32 = vm.clampf(m.t, 0, 1);
        var cc = m.color;
        cc.a = @intFromFloat(a * 255);
        const tw = txtW(&m.buf, 16);
        c.DrawRectangleRounded(.{ .x = W / 2 - tw / 2 - 10, .y = my - 1, .width = tw + 20, .height = 19 }, 0.4, 4, col(6, 14, 10, @intFromFloat(a * 150)));
        txtC(&m.buf, W / 2, my, 16, cc);
        my += 19;
    }
}

fn edgeMarker(cam: c.Camera3D, p: V3, W: f32, H: f32) struct { x: f32, y: f32, on: bool } {
    const fwd = vm.norm(vm.sub(cam.target, cam.position));
    const inFront = vm.dot(vm.sub(p, cam.position), fwd) > 0;
    var sp = c.GetWorldToScreen(p, cam);
    if (inFront and sp.x > 30 and sp.x < W - 30 and sp.y > 30 and sp.y < H - 30) return .{ .x = sp.x, .y = sp.y, .on = true };
    // off-screen: push to the border along the direction from screen center
    var dx = sp.x - W / 2;
    var dy = sp.y - H / 2;
    if (!inFront) {
        dx = -dx;
        dy = -dy;
    }
    const l = @max(@sqrt(dx * dx + dy * dy), 1);
    const k = @min((W / 2 - 50) / @max(@abs(dx / l), 1e-3), (H / 2 - 50) / @max(@abs(dy / l), 1e-3));
    sp = vm.v2(W / 2 + dx / l * k, H / 2 + dy / l * k);
    return .{ .x = sp.x, .y = sp.y, .on = false };
}

fn drawHudGunner(cam: c.Camera3D) void {
    const W: f32 = @floatFromInt(c.GetScreenWidth());
    const H: f32 = @floatFromInt(c.GetScreenHeight());
    const cx = W / 2;
    const cy = H / 2;
    const camFwd = vm.norm(vm.sub(cam.target, cam.position));
    const gunPos = spaagGunsXf().p;
    const SIGHT = col(255, 120, 60, 235);
    const SIGHT_DIM = col(255, 120, 60, 120);

    // optics vignette
    const vr = @min(W, H) * 0.47;
    c.DrawRing(vm.v2(cx, cy), vr, @max(W, H), 0, 360, 64, col(4, 6, 5, 200));
    c.DrawCircleLines(@intFromFloat(cx), @intFromFloat(cy), vr, SIGHT_DIM);

    // targets seen by the optics
    for (&targets, 0..) |*t, i| {
        if (!gunnerSees(t, cam.position)) continue;
        if (vm.dot(vm.sub(t.pos, cam.position), camFwd) <= 0) continue;
        const sp = c.GetWorldToScreen(t.pos, cam);
        if ((sp.x - cx) * (sp.x - cx) + (sp.y - cy) * (sp.y - cy) > vr * vr) continue;
        const locked = g.gLock == @as(i32, @intCast(i));
        const cc = if (locked) SIGHT else if (tracked(t)) trackColor(t) else HUD_DIM;
        bracket(sp.x, sp.y, if (locked) 16 else 11, cc, if (locked) 2.2 else 1.3);
        const d = vm.dist(t.pos, cam.position);
        txt(c.TextFormat("%s %.1f", kindName(t), @as(f64, d / 1000)), sp.x + 18, sp.y - 16, 14, cc);
        // lead pip: put the crosshair here
        const lp = leadPoint(t, gunPos);
        if (vm.dot(vm.sub(lp.aim, cam.position), camFwd) > 0) {
            const pp = c.GetWorldToScreen(lp.aim, cam);
            c.DrawCircleLines(@intFromFloat(pp.x), @intFromFloat(pp.y), if (locked) 7 else 5, cc);
            c.DrawLineEx(sp, pp, 1, col(cc.r, cc.g, cc.b, 90));
        }
    }

    // crosshair
    const gap: f32 = 10;
    const len: f32 = 40;
    c.DrawLineEx(vm.v2(cx - gap - len, cy), vm.v2(cx - gap, cy), 1.6, SIGHT);
    c.DrawLineEx(vm.v2(cx + gap, cy), vm.v2(cx + gap + len, cy), 1.6, SIGHT);
    c.DrawLineEx(vm.v2(cx, cy + gap), vm.v2(cx, cy + gap + len), 1.6, SIGHT);
    c.DrawCircle(@intFromFloat(cx), @intFromFloat(cy), 1.6, SIGHT);
    // mil ticks
    var k: i32 = 1;
    while (k <= 4) : (k += 1) {
        const off = @as(f32, @floatFromInt(k)) * vr * 0.18;
        c.DrawLineEx(vm.v2(cx - 5, cy + off), vm.v2(cx + 5, cy + off), 1, SIGHT_DIM);
        c.DrawLineEx(vm.v2(cx + off, cy - 4), vm.v2(cx + off, cy + 4), 1, SIGHT_DIM);
        c.DrawLineEx(vm.v2(cx - off, cy - 4), vm.v2(cx - off, cy + 4), 1, SIGHT_DIM);
    }
    // where the barrels actually point (they slew behind the sight)
    {
        const gx = spaagGunsXf();
        const bp = c.GetWorldToScreen(vm.mad(cam.position, gx.x, 2000), cam);
        const off = (bp.x - cx) * (bp.x - cx) + (bp.y - cy) * (bp.y - cy);
        if (off > 9 and off < vr * vr) c.DrawCircleLines(@intFromFloat(bp.x), @intFromFloat(bp.y), 4, SIGHT_DIM);
    }

    // readouts
    const zoom = 40 / g.gFov;
    const bx = cx - vr * 0.7;
    const by = cy + vr * 0.62;
    txt(c.TextFormat("ДАЛЬН %04d м", @as(c_int, @intFromFloat(g.gRange))), bx, by, 17, if (g.gLock >= 0) SIGHT else SIGHT_DIM);
    txt(c.TextFormat("x%.1f", @as(f64, zoom)), bx, by + 22, 16, SIGHT_DIM);
    txtR(c.TextFormat("БК %d", @as(c_int, g.gAmmo)), cx + vr * 0.7, by, 17, if (g.gAmmo > 100) SIGHT else HUD_RED);
    txtR(c.TextFormat("ЗУР %d + %d", @as(c_int, readyCount()), @as(c_int, g.reserve)), cx + vr * 0.7, by + 22, 16, SIGHT_DIM);
    if (g.gAmmo <= 0) txtC("ПЕРЕЗАРЯДКА", cx, cy + 60, 20, HUD_RED);
    const az = @mod((spaagYaw + g.gAimYaw) / vm.deg + 720, 360);
    txtC(c.TextFormat("АЗ %03d   УМ %+02d", @as(c_int, @intFromFloat(az)), @as(c_int, @intFromFloat(g.gAimPitch / vm.deg))), cx, cy - vr * 0.8, 15, SIGHT_DIM);

    // compass arrows to radar contacts outside the sight
    for (&targets) |*t| {
        if (!tracked(t) or t.dying or !hostile(t)) continue;
        const m = edgeMarker(cam, t.pos, W, H);
        const dx = m.x - cx;
        const dy = m.y - cy;
        if (dx * dx + dy * dy < vr * vr) continue;
        const l = @sqrt(dx * dx + dy * dy);
        const ex = cx + dx / l * (vr + 14);
        const ey = cy + dy / l * (vr + 14);
        c.DrawCircle(@intFromFloat(ex), @intFromFloat(ey), 4, trackColor(t));
    }

    c.DrawRectangleRounded(.{ .x = 16, .y = 16, .width = 270, .height = 74 }, 0.08, 6, PANEL);
    txt("НАВОДЧИК ЗСУ", 28, 24, 18, HUD);
    txt(c.TextFormat("Волна %d   Сбито %d   Счёт %d", @as(c_int, g.wave), @as(c_int, g.kills), @as(c_int, g.score)), 28, 48, 15, HUD);
    txt(if (g.autoSam) "ЗРК: авто" else "ЗРК: ручной", 28, 68, 14, HUD_DIM);
    drawObjBars(W);
    drawMsgs(W, 66);
    txtC("ЛКМ огонь · колесо/Z зум · F пуск ЗУР по цели в прицеле · V выйти", cx, H - 26, 14, HUD_DIM);
}

fn drawHudAttack(cam: c.Camera3D) void {
    const W: f32 = @floatFromInt(c.GetScreenWidth());
    const H: f32 = @floatFromInt(c.GetScreenHeight());
    const goal = atkGoal(g.atkGoal);
    const pt = playerTarget();

    // goal marker
    {
        const m = edgeMarker(cam, v3(goal.pos.x, goal.pos.y + 20, goal.pos.z), W, H);
        const s: f32 = 12;
        c.DrawLineEx(vm.v2(m.x, m.y - s), vm.v2(m.x + s, m.y), 2, HUD_AMBER);
        c.DrawLineEx(vm.v2(m.x + s, m.y), vm.v2(m.x, m.y + s), 2, HUD_AMBER);
        c.DrawLineEx(vm.v2(m.x, m.y + s), vm.v2(m.x - s, m.y), 2, HUD_AMBER);
        c.DrawLineEx(vm.v2(m.x - s, m.y), vm.v2(m.x, m.y - s), 2, HUD_AMBER);
        if (pt) |t| txt(c.TextFormat("%s  %.1f км", goal.name, @as(f64, vm.distXZ(t.pos, goal.pos) / 1000)), m.x + 16, m.y - 8, 15, HUD_AMBER);
    }
    // incoming missiles
    var threat: f32 = 1e9;
    var nThreat: i32 = 0;
    for (&missiles) |*m| {
        if (!m.alive or m.tgtId != g.playerId) continue;
        nThreat += 1;
        if (pt) |t| threat = @min(threat, vm.dist(m.pos, t.pos));
        const mk = edgeMarker(cam, m.pos, W, H);
        bracket(mk.x, mk.y, 10, HUD_RED, 2);
        if (pt) |t| txt(c.TextFormat("ЗУР %.1f", @as(f64, vm.dist(m.pos, t.pos) / 1000)), mk.x + 14, mk.y - 8, 14, HUD_RED);
    }
    // known battery position (intel)
    {
        const bm = edgeMarker(cam, v3(0, heightAt(0, 0) + 15, 0), W, H);
        if (bm.on) {
            c.DrawCircleLines(@intFromFloat(bm.x), @intFromFloat(bm.y), 8, HUD_RED);
            txt("ЗРК", bm.x + 12, bm.y - 7, 14, HUD_RED);
        }
    }

    if (pt) |t| {
        // flight-path marker
        const fp = c.GetWorldToScreen(vm.mad(t.pos, t.vel, 3), cam);
        c.DrawCircleLines(@intFromFloat(fp.x), @intFromFloat(fp.y), 7, HUD);
        c.DrawLineEx(vm.v2(fp.x - 18, fp.y), vm.v2(fp.x - 8, fp.y), 1.5, HUD);
        c.DrawLineEx(vm.v2(fp.x + 8, fp.y), vm.v2(fp.x + 18, fp.y), 1.5, HUD);
        c.DrawLineEx(vm.v2(fp.x, fp.y - 8), vm.v2(fp.x, fp.y - 14), 1.5, HUD);

        const agl = t.pos.y - heightAt(t.pos.x, t.pos.z);
        const spd = vm.len(t.vel) * 3.6;
        c.DrawRectangleRounded(.{ .x = 16, .y = 16, .width = 290, .height = 150 }, 0.06, 6, PANEL);
        txt(c.TextFormat("УДАРНЫЙ БПЛА  ·  ВЫЛЕТ %d", @as(c_int, g.sortie + 1)), 28, 24, 18, HUD);
        txt(c.TextFormat("Цель: %s", goal.name), 28, 50, 16, HUD_AMBER);
        txt(c.TextFormat("Дистанция  %.2f км", @as(f64, vm.distXZ(t.pos, goal.pos) / 1000)), 28, 72, 16, HUD);
        txt(c.TextFormat("Высота  %d м", @as(c_int, @intFromFloat(agl))), 28, 94, 16, if (agl < 60) HUD_AMBER else HUD);
        txt(c.TextFormat("Скорость  %d км/ч", @as(c_int, @intFromFloat(spd))), 160, 94, 16, HUD);
        txt("Форсаж", 28, 118, 14, HUD_DIM);
        c.DrawRectangle(90, 122, 190, 6, col(30, 40, 34, 220));
        c.DrawRectangle(90, 122, @intFromFloat(190 * g.boost), 6, HUD);
        txt(c.TextFormat("Счёт %d   Поражено %d   x%d", @as(c_int, g.score), @as(c_int, g.strikes), @as(c_int, @intFromFloat(g.warp))), 28, 140, 14, HUD_DIM);

        // threat warnings
        var wy = H * 0.2;
        if (nThreat > 0 and blink(4)) {
            txtC(c.TextFormat("ПУСК ЗУР!  %.1f км", @as(f64, threat / 1000)), W / 2, wy, 30, HUD_RED);
        }
        if (nThreat > 0 and threat < 1200) {
            txtC("РЕЗКИЙ МАНЁВР — ДЁРНИ МЫШЬ!", W / 2, wy - 34, 22, if (g.pManeuver > 0.5) HUD else HUD_AMBER);
        }
        wy += 36;
        if (g.gTgt == g.player and blink(3)) {
            txtC("ОБСТРЕЛ ЗСУ", W / 2, wy, 24, HUD_RED);
        }
        wy += 30;
        if (tracked(t)) {
            txtC("РЛС: ВЫ ОБНАРУЖЕНЫ", W / 2, wy, 18, HUD_AMBER);
        } else if (agl < 60) {
            txtC("малая высота — радар не видит", W / 2, wy, 16, HUD_DIM);
        }
        drawTacMap(t);
    }
    // messages
    var my: f32 = 66;
    for (&g.msgs) |*m| {
        if (m.t <= 0) continue;
        const a: f32 = vm.clampf(m.t, 0, 1);
        var cc = m.color;
        cc.a = @intFromFloat(a * 255);
        const tw = txtW(&m.buf, 16);
        c.DrawRectangleRounded(.{ .x = W / 2 - tw / 2 - 10, .y = my - 1, .width = tw + 20, .height = 19 }, 0.4, 4, col(6, 14, 10, @intFromFloat(a * 150)));
        txtC(&m.buf, W / 2, my, 16, cc);
        my += 19;
    }
    const help = "Мышь / WASD — управление   Shift — форсаж   Колесо — камера   C — вид из носа   X — ускорение   Esc — пауза";
    const hw = txtW(help, 14);
    c.DrawRectangleRounded(.{ .x = W / 2 - hw / 2 - 12, .y = H - 26, .width = hw + 24, .height = 21 }, 0.4, 4, PANEL);
    txtC(help, W / 2, H - 23, 14, HUD_DIM);
}

fn drawTacMap(pt: *const Target) void {
    const sc = scopeRect();
    const cx: c_int = @intFromFloat(sc.cx);
    const cy: c_int = @intFromFloat(sc.cy);
    c.DrawCircle(cx, cy, sc.r + 8, PANEL);
    c.DrawCircle(cx, cy, sc.r, col(4, 22, 12, 210));
    // map centered on the player, north up, 8 km radius
    const R: f32 = 8000;
    const k = sc.r / R;
    const P = struct {
        fn at(s: Scope, kk: f32, o: V3, p: V3) c.Vector2 {
            return vm.v2(s.cx + (p.x - o.x) * kk, s.cy + (p.z - o.z) * kk);
        }
    };
    for ([_]f32{ 2000, 4000, 6000 }) |rr| c.DrawCircleLines(cx, cy, sc.r * rr / R, col(90, 200, 120, 60));
    // SAM / gun envelopes around the battery
    const bp = P.at(sc, k, pt.pos, vm.zero);
    c.DrawCircleLinesV(bp, GUN_RANGE * k, col(255, 120, 80, 110));
    c.DrawCircleLinesV(bp, 7500 * k, col(255, 90, 70, 60));
    c.DrawCircleV(bp, 4, HUD_RED);
    const gp = P.at(sc, k, pt.pos, atkGoal(g.atkGoal).pos);
    c.DrawRectangleLines(@intFromFloat(gp.x - 4), @intFromFloat(gp.y - 4), 9, 9, HUD_AMBER);
    for (&targets) |*t| {
        if (!t.alive or t.player) continue;
        c.DrawCircleV(P.at(sc, k, pt.pos, t.pos), 2, HUD_DIM);
    }
    for (&missiles) |*m| {
        if (!m.alive) continue;
        c.DrawCircleV(P.at(sc, k, pt.pos, m.pos), 2.5, HUD_RED);
    }
    // player arrow
    const d = vm.norm(v3(pt.vel.x, 0, pt.vel.z));
    const tip = vm.v2(sc.cx + d.x * 10, sc.cy + d.z * 10);
    const l = vm.v2(sc.cx - d.x * 6 - d.z * 5, sc.cy - d.z * 6 + d.x * 5);
    const r = vm.v2(sc.cx - d.x * 6 + d.z * 5, sc.cy - d.z * 6 - d.x * 5);
    c.DrawTriangle(tip, r, l, WHITE);
    c.DrawTriangle(tip, l, r, WHITE);
    txt("КАРТА · 8 км", sc.cx - sc.r, sc.cy + sc.r + 2, 13, HUD_DIM);
}

// ============================================================ menus
fn menuList(items: []const [*c]const u8, enabled: []const bool, x: f32, y: f32, w: f32) ?usize {
    const n = items.len;
    if (menuSel >= n or !enabled[menuSel]) {
        menuSel = 0;
        while (menuSel < n - 1 and !enabled[menuSel]) menuSel += 1;
    }
    if (c.IsKeyPressed(c.KEY_DOWN) or c.IsKeyPressed(c.KEY_S)) {
        var i = menuSel;
        var k: usize = 0;
        while (k < n) : (k += 1) {
            i = (i + 1) % n;
            if (enabled[i]) break;
        }
        menuSel = i;
    }
    if (c.IsKeyPressed(c.KEY_UP) or c.IsKeyPressed(c.KEY_W)) {
        var i = menuSel;
        var k: usize = 0;
        while (k < n) : (k += 1) {
            i = (i + n - 1) % n;
            if (enabled[i]) break;
        }
        menuSel = i;
    }
    var clicked: ?usize = null;
    const mp = c.GetMousePosition();
    const md = c.GetMouseDelta();
    const moved = md.x != 0 or md.y != 0;
    for (items, 0..) |it, i| {
        const r = c.Rectangle{ .x = x, .y = y + @as(f32, @floatFromInt(i)) * 50, .width = w, .height = 42 };
        const hover = c.CheckCollisionPointRec(mp, r);
        if (hover and enabled[i] and moved) menuSel = i;
        const sel = menuSel == i;
        c.DrawRectangleRec(r, if (sel) col(150, 255, 175, 34) else col(0, 0, 0, 70));
        if (sel) c.DrawRectangle(@intFromFloat(r.x), @intFromFloat(r.y), 4, 42, HUD);
        const tc = if (!enabled[i]) col(110, 130, 115, 140) else if (sel) WHITE else HUD;
        txt(it, r.x + 20, r.y + 10, 21, tc);
        if (hover and enabled[i] and c.IsMouseButtonPressed(c.MOUSE_BUTTON_LEFT)) clicked = i;
    }
    if ((c.IsKeyPressed(c.KEY_ENTER) or c.IsKeyPressed(c.KEY_SPACE)) and enabled[menuSel]) clicked = menuSel;
    return clicked;
}

fn panelLeft(W: f32, H: f32) void {
    _ = W;
    // dark vertical band with soft edge, the scene stays visible on the right
    c.DrawRectangle(0, 0, 520, @intFromFloat(H), col(5, 8, 9, 200));
    c.DrawRectangleGradientH(520, 0, 220, @intFromFloat(H), col(5, 8, 9, 200), col(5, 8, 9, 0));
}

fn title(sub: [*c]const u8) void {
    txt("ПВО", 60, 70, 84, HUD);
    txt("ТАКТИЧЕСКИЙ СИМУЛЯТОР", 64, 160, 20, HUD_DIM);
    c.DrawRectangle(64, 192, 360, 2, col(120, 220, 140, 120));
    txt(sub, 64, 206, 18, HUD_AMBER);
}

fn goMenu() void {
    app = .menu;
    menuSel = 0;
    mode = .defense;
    resetGame();
    c.EnableCursor();
}

fn newDefense() void {
    mode = .defense;
    resetGame();
    g.waveT = 6;
    app = .playing;
    c.EnableCursor();
}

fn beginAttack() void {
    app = .playing;
    startSortie();
    g.camDist = 26;
    g.camPos = vm.add(targets[@intCast(g.player)].pos, v3(0, 20, 0));
    c.DisableCursor();
}

fn continueGame() void {
    const s = save.game;
    if (s.mode == @intFromEnum(Mode.attack)) {
        mode = .attack;
        resetGame();
        g.sortie = s.sortie;
        g.strikes = s.strikes;
        g.score = s.score;
        app = .briefing;
        menuSel = 0;
        return;
    }
    mode = .defense;
    resetGame();
    g.wave = s.wave;
    g.score = s.score;
    g.kills = s.kills;
    g.leaks = s.leaks;
    g.reserve = s.reserve;
    for (&objs, 0..) |*o, i| o.hp = s.hp[i];
    g.waveT = 6;
    app = .playing;
    msg(c.TextFormat("Загружено: волна %d", @as(c_int, s.wave)), HUD);
}

fn drawMenus() void {
    const W: f32 = @floatFromInt(c.GetScreenWidth());
    const H: f32 = @floatFromInt(c.GetScreenHeight());
    panelLeft(W, H);
    const lx: f32 = 60;
    const ly: f32 = 260;
    const lw: f32 = 400;
    switch (app) {
        .menu => {
            title("главное меню");
            const hasSave = save.game.valid == 1;
            const items = [_][*c]const u8{ "Продолжить", "Оборона — расчёт ПВО", "Оборона — наводчик ЗСУ", "Атака — ударный БПЛА", "Настройки", "Рекорды", "Выход" };
            const en = [_]bool{ hasSave, true, true, true, true, true, true };
            if (hasSave) {
                const s = save.game;
                const info = if (s.mode == 1) c.TextFormat("сохранение: атака, вылет %d, счёт %d", @as(c_int, s.sortie + 1), @as(c_int, s.score)) else c.TextFormat("сохранение: оборона, волна %d, счёт %d", @as(c_int, s.wave), @as(c_int, s.score));
                txt(info, lx, ly + 7 * 50 + 10, 15, HUD_DIM);
            }
            if (menuList(&items, &en, lx, ly, lw)) |i| switch (i) {
                0 => continueGame(),
                1 => newDefense(),
                2 => {
                    newDefense();
                    enterGunner(true);
                },
                3 => {
                    mode = .attack;
                    resetGame();
                    app = .briefing;
                    menuSel = 0;
                },
                4 => {
                    appPrev = .menu;
                    app = .settings;
                    menuSel = 0;
                },
                5 => {
                    app = .records;
                    menuSel = 0;
                },
                else => quit = true,
            };
        },
        .settings => {
            title("настройки");
            const st = &save.settings;
            const sensNames = [_][*c]const u8{ "Чувствительность мыши: низкая", "Чувствительность мыши: средняя", "Чувствительность мыши: высокая", "Чувствительность мыши: макс" };
            const diffNames = [_][*c]const u8{ "Сложность: лёгкая", "Сложность: нормальная", "Сложность: тяжёлая" };
            const items = [_][*c]const u8{
                diffNames[@min(st.diff, 2)],
                if (st.rain == 1) "Дождь: вкл" else "Дождь: выкл",
                if (st.shadows == 1) "Тени: вкл" else "Тени: выкл",
                sensNames[@min(st.sens, 3)],
                if (st.invertY == 1) "Инверсия Y: вкл" else "Инверсия Y: выкл",
                if (st.hidpi == 1) "Retina-разрешение: вкл (тяжело)" else "Retina-разрешение: выкл",
                "Назад",
            };
            const en = [_]bool{ true, true, true, true, true, true, true };
            txt("Retina применяется после перезапуска", lx, ly + 7 * 50 + 6, 14, HUD_DIM);
            if (menuList(&items, &en, lx, ly, lw)) |i| {
                switch (i) {
                    0 => st.diff = (st.diff + 1) % 3,
                    1 => st.rain ^= 1,
                    2 => st.shadows ^= 1,
                    3 => st.sens = (st.sens + 1) % 4,
                    4 => st.invertY ^= 1,
                    5 => st.hidpi ^= 1,
                    else => {
                        app = appPrev;
                        menuSel = 0;
                    },
                }
                writeSave();
            }
            if (c.IsKeyPressed(c.KEY_ESCAPE)) app = appPrev;
        },
        .records => {
            title("рекорды");
            const r = save.records;
            txt("ОБОРОНА", lx, ly, 20, HUD);
            txt(c.TextFormat("лучшая волна: %d", @as(c_int, r.defWave)), lx + 20, ly + 30, 18, HUD_DIM);
            txt(c.TextFormat("лучший счёт: %d", @as(c_int, r.defScore)), lx + 20, ly + 54, 18, HUD_DIM);
            txt("АТАКА", lx, ly + 100, 20, HUD);
            txt(c.TextFormat("поражено целей подряд: %d", @as(c_int, r.atkStrikes)), lx + 20, ly + 130, 18, HUD_DIM);
            txt(c.TextFormat("лучший счёт: %d", @as(c_int, r.atkScore)), lx + 20, ly + 154, 18, HUD_DIM);
            const items = [_][*c]const u8{"Назад"};
            const en = [_]bool{true};
            if (menuList(&items, &en, lx, ly + 220, lw) != null or c.IsKeyPressed(c.KEY_ESCAPE)) {
                app = .menu;
                menuSel = 0;
            }
        },
        .briefing => {
            title(c.TextFormat("брифинг · вылет %d", @as(c_int, g.sortie + 1)));
            txt("Выбери цель. ПВО противника работает автоматически:", lx, ly - 30, 15, HUD_DIM);
            const items = [_][*c]const u8{ "ТЭЦ", "Нефтебаза", "Позиция ЗРК (подавление ПВО)", "Назад" };
            const en = [_]bool{ objs[0].hp > 0, objs[1].hp > 0, true, true };
            if (menuList(&items, &en, lx, ly, lw)) |i| {
                if (i < 3) {
                    g.atkGoal = i;
                    beginAttack();
                } else goMenu();
            }
            const tips = [_][*c]const u8{
                "· ниже 60 м радар видит дрон с 1.7 км, ниже 30 м — с 1 км",
                "· ЗУР близко — резко дёрни мышь, ракета промахнётся",
                "· холмы закрывают от РЛС — используй рельеф",
                "· ЗСУ бьёт до 3.3 км, ЗУР — до 7.5 км",
                "· Shift — форсаж, пике разгоняет дрон",
                "· ведомые дроны отвлекают ПВО на себя",
            };
            for (tips, 0..) |t, i| txt(t, lx, ly + 220 + @as(f32, @floatFromInt(i)) * 22, 15, HUD_DIM);
        },
        .result => {
            if (mode == .attack) {
                title(if (g.success) "цель поражена" else "дрон потерян");
                txt(if (g.success) "ЦЕЛЬ ПОРАЖЕНА" else "ДРОН ПОТЕРЯН", lx, ly - 10, 34, if (g.success) HUD else HUD_RED);
                txt(c.TextFormat("Счёт %d   ·   поражено целей %d", @as(c_int, g.score), @as(c_int, g.strikes)), lx, ly + 36, 18, HUD_DIM);
                const items = [_][*c]const u8{ if (g.success) "Следующий вылет" else "Повторить вылет", "Главное меню" };
                const en = [_]bool{ true, true };
                if (menuList(&items, &en, lx, ly + 90, lw)) |i| {
                    if (i == 0) {
                        app = .briefing;
                        menuSel = 0;
                    } else goMenu();
                }
            } else {
                title("итог");
                txt("ОБЪЕКТЫ УНИЧТОЖЕНЫ", lx, ly - 10, 34, HUD_RED);
                txt(c.TextFormat("Волна %d   ·   счёт %d   ·   сбито %d", @as(c_int, g.wave), @as(c_int, g.score), @as(c_int, g.kills)), lx, ly + 36, 18, HUD_DIM);
                const items = [_][*c]const u8{ "Новая игра", "Главное меню" };
                const en = [_]bool{ true, true };
                if (menuList(&items, &en, lx, ly + 90, lw)) |i| {
                    if (i == 0) newDefense() else goMenu();
                }
            }
        },
        else => {},
    }
    txt("Zig + raylib · модели: Blender", 20, H - 26, 13, col(110, 140, 120, 160));
}

var savedFlash: f32 = 0;

fn drawPause() void {
    const W: f32 = @floatFromInt(c.GetScreenWidth());
    const H: f32 = @floatFromInt(c.GetScreenHeight());
    c.DrawRectangle(0, 0, @intFromFloat(W), @intFromFloat(H), col(0, 0, 0, 110));
    panelLeft(W, H);
    title("пауза");
    const items = [_][*c]const u8{ "Продолжить", "Сохранить игру", "Настройки", "Главное меню" };
    const en = [_]bool{ true, true, true, true };
    if (savedFlash > 0) txt("Игра сохранена", 60, 260 + 4 * 50 + 10, 16, HUD);
    if (menuList(&items, &en, 60, 260, 400)) |i| switch (i) {
        0 => resumeGame(),
        1 => {
            storeGame();
            savedFlash = 2;
        },
        2 => {
            appPrev = .paused;
            app = .settings;
            menuSel = 0;
        },
        else => {
            updateRecords();
            goMenu();
        },
    };
}

fn resumeGame() void {
    app = .playing;
    if (mode == .attack or g.gunner) c.DisableCursor();
}

// ============================================================ main
pub fn main() void {
    vm.seed(0xC0FFEE);
    const shotEnv = c.getenv("PVO_SHOT");
    const shotFrames: i32 = if (shotEnv != null) c.atoi(shotEnv) else 0;
    const camEnv = c.getenv("PVO_CAM");

    // launched from Finder/Explorer: run next to the executable so assets and saves are found
    if (!c.DirectoryExists("assets")) _ = c.ChangeDirectory(c.GetApplicationDirectory());
    loadSave();
    var flags: c_uint = c.FLAG_MSAA_4X_HINT | c.FLAG_WINDOW_RESIZABLE | c.FLAG_VSYNC_HINT;
    if (save.settings.hidpi == 1 or c.getenv("PVO_HIDPI") != null) flags |= c.FLAG_WINDOW_HIGHDPI;
    c.SetConfigFlags(flags);
    c.SetTraceLogLevel(c.LOG_WARNING);
    c.InitWindow(1600, 900, "ПВО — симулятор");
    c.SetExitKey(0);

    computeFogDisp();
    initShaders();
    initShadow();
    initFont();
    {
        const img = c.GenImageGradientRadial(64, 64, 0.0, col(255, 255, 255, 255), col(255, 255, 255, 0));
        puff = c.LoadTextureFromImage(img);
        c.UnloadImage(img);
        c.GenTextureMipmaps(&puff);
        c.SetTextureFilter(puff, c.TEXTURE_FILTER_TRILINEAR);
    }

    M = .{
        .launcher_base = loadModel("assets/launcher_base.glb"),
        .launcher_turret = loadModel("assets/launcher_turret.glb"),
        .radar_base = loadModel("assets/radar_base.glb"),
        .radar_head = loadModel("assets/radar_head.glb"),
        .spaag_base = loadModel("assets/spaag_base.glb"),
        .spaag_turret = loadModel("assets/spaag_turret.glb"),
        .spaag_guns = loadModel("assets/spaag_guns.glb"),
        .sam = loadModel("assets/sam_missile.glb"),
        .drone = loadModel("assets/drone_delta.glb"),
        .cruise = loadModel("assets/cruise_missile.glb"),
        .pine = loadModel("assets/tree_pine.glb"),
        .leaf = loadModel("assets/tree_leaf.glb"),
        .power = loadModel("assets/power_plant.glb"),
        .depot = loadModel("assets/oil_depot.glb"),
        .apartment = loadModel("assets/apartment.glb"),
        .house = loadModel("assets/house.glb"),
        .terrain = undefined,
        .sky = undefined,
    };
    M.terrain = buildTerrain();
    M.terrain.materials[0].shader = lit;
    M.sky = c.LoadModelFromMesh(c.GenMeshSphere(1, 24, 32));
    M.sky.materials[0].shader = sky;

    initWorld();
    goMenu();
    g.autoSam = true;

    const sceneEnv = c.getenv("PVO_SCENE");
    const scene: c_int = if (sceneEnv != null) c.atoi(sceneEnv) else 0;
    if (shotFrames > 0 and scene == 4) {
        app = .playing;
        g.wave = 1;
        g.waveBearing = 0.4;
        var i: usize = 0;
        while (i < 5) : (i += 1) spawnTarget(.drone, 1600);
        g.toSpawn = 0;
        enterGunner(true);
        c.EnableCursor();
        noMouse = true;
        const t = &targets[0];
        const d = vm.sub(t.pos, gunnerEye());
        g.gAimYaw = vm.wrapPi(vm.yawOf(d) - spaagYaw);
        g.gAimPitch = vm.pitchOf(d);
        g.gYaw = g.gAimYaw;
        g.gPitch = g.gAimPitch;
        const fe = c.getenv("PVO_FOV");
        if (fe != null) g.gFov = @floatFromInt(c.atoi(fe));
    } else if (shotFrames > 0 and scene == 2) {
        mode = .attack;
        resetGame();
        g.atkGoal = 2;
        beginAttack();
        // start close to the battery so the defence reacts within the clip
        const t = &targets[@intCast(g.player)];
        t.pos = v3(2600, heightAt(2600, -900) + 140, -900);
        g.pYaw = vm.yawOf(vm.sub(vm.zero, t.pos));
        g.camPos = vm.add(t.pos, v3(30, 10, 0));
        c.EnableCursor();
    } else if (shotFrames > 0 and scene == 0) {
        app = .playing;
        // scripted scene for screenshots
        g.wave = 2;
        g.waveBearing = 0.9;
        g.autoSam = true;
        var i: usize = 0;
        while (i < 7) : (i += 1) spawnTarget(if (i % 3 == 0) .cruise else .drone, 3400);
        g.toSpawn = 0;
        g.camYaw = 2.0;
        g.camPitch = 0.22;
        g.camDist = 120;
        if (camEnv != null) {
            const cm = c.atoi(camEnv);
            if (cm == 4 or cm == 5 or cm == 6) {
                // close-ups for model checks: SPAAG, launcher, radar
                g.camFocus = if (cm == 4) spaagPos else if (cm == 5) g.launchers[0].pos else radarPos;
                g.camFocus.y += 2;
                g.camDist = 16;
                g.camPitch = 0.25;
                g.camYaw = 0.7;
                const ye = c.getenv("PVO_YAW");
                if (ye != null) g.camYaw = @as(f32, @floatFromInt(c.atoi(ye))) / 100;
            } else if (cm == 3) {
                g.camFocus = objs[0].pos;
                g.camDist = 420;
                g.camPitch = 0.28;
                g.camYaw = 0.9;
            } else g.camMode = @intCast(@mod(cm, 3));
        }
        debugShadow = c.getenv("PVO_DBG") != null;
        if (c.getenv("PVO_NOSHADOW") != null) save.settings.shadows = 0;
        terrainDebug = c.getenv("PVO_TDBG") != null;
        if (c.getenv("PVO_NOROAD") != null) setI(lit, L.roadCount, 0);
        const distEnv = c.getenv("PVO_DIST");
        if (distEnv != null) g.camDist = @floatFromInt(c.atoi(distEnv));
    }

    var frame: i32 = 0;
    while (!quit and !c.WindowShouldClose()) {
        const dtReal: f32 = if (shotFrames > 0) 1.0 / 60.0 else @min(c.GetFrameTime(), 0.05);
        renderTime += dtReal;
        savedFlash -= dtReal;
        const cam0 = c.Camera3D{ .position = g.camPos, .target = g.camLook, .up = v3(0, 1, 0), .fovy = 55, .projection = c.CAMERA_PERSPECTIVE };
        var cam = cam0;
        switch (app) {
            .playing => {
                if (c.IsKeyPressed(c.KEY_ESCAPE)) {
                    app = .paused;
                    menuSel = 0;
                    c.EnableCursor();
                } else {
                    handleInput(cam0, dtReal);
                    if (!g.paused) {
                        const steps: usize = @intFromFloat(g.warp);
                        var s: usize = 0;
                        while (s < steps) : (s += 1) update(dtReal);
                    }
                }
                cam = updateCamera(dtReal);
            },
            .paused => {
                if (c.IsKeyPressed(c.KEY_ESCAPE)) resumeGame();
                if (mode == .attack) cam.up = v3(0, 1, 0);
            },
            .settings => {
                if (appPrev != .paused) cam = menuCamera(dtReal);
            },
            .result => {
                update(dtReal);
                cam = updateCamera(dtReal);
            },
            else => {
                // live background: the battery fights on its own behind the menu
                if (mode == .defense) update(dtReal) else updateParts(dtReal);
                cam = menuCamera(dtReal);
            },
        }
        render(cam);
        frame += 1;
        if (shotFrames > 0 and frame == shotFrames) {
            const rw = c.GetRenderWidth();
            const rh = c.GetRenderHeight();
            var img = c.Image{ .data = c.rlReadScreenPixels(rw, rh), .width = rw, .height = rh, .mipmaps = 1, .format = c.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
            if (rw > 2000) c.ImageResize(&img, @divTrunc(rw, 2), @divTrunc(rh, 2));
            std.debug.print("FPS {d} render {d}x{d}\n", .{ c.GetFPS(), rw, rh });
            _ = c.ExportImage(img, "shot.png");
            c.UnloadImage(img);
            break;
        }
    }
    c.CloseWindow();
}
