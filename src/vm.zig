//! Small vector / transform helpers over raylib's C structs.
const c = @import("c");

pub const V3 = c.Vector3;
pub const V2 = c.Vector2;

pub inline fn v3(x: f32, y: f32, z: f32) V3 {
    return .{ .x = x, .y = y, .z = z };
}
pub inline fn v2(x: f32, y: f32) V2 {
    return .{ .x = x, .y = y };
}
pub const zero = v3(0, 0, 0);
pub const up = v3(0, 1, 0);

pub inline fn add(a: V3, b: V3) V3 {
    return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
pub inline fn sub(a: V3, b: V3) V3 {
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
pub inline fn scale(a: V3, s: f32) V3 {
    return v3(a.x * s, a.y * s, a.z * s);
}
pub inline fn mad(a: V3, b: V3, s: f32) V3 {
    return v3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s);
}
pub inline fn dot(a: V3, b: V3) f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
pub inline fn cross(a: V3, b: V3) V3 {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
pub inline fn len(a: V3) f32 {
    return @sqrt(dot(a, a));
}
pub inline fn len2(a: V3) f32 {
    return dot(a, a);
}
pub inline fn dist(a: V3, b: V3) f32 {
    return len(sub(a, b));
}
pub inline fn distXZ(a: V3, b: V3) f32 {
    const dx = a.x - b.x;
    const dz = a.z - b.z;
    return @sqrt(dx * dx + dz * dz);
}
pub fn norm(a: V3) V3 {
    const l = len(a);
    if (l < 1e-6) return v3(1, 0, 0);
    return scale(a, 1.0 / l);
}
pub inline fn lerp(a: V3, b: V3, t: f32) V3 {
    return v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}
pub inline fn lerpf(a: f32, b: f32, t: f32) f32 {
    return a + (b - a) * t;
}
pub inline fn clampf(x: f32, a: f32, b: f32) f32 {
    return @max(a, @min(b, x));
}
pub inline fn smooth(e0: f32, e1: f32, x: f32) f32 {
    const t = clampf((x - e0) / (e1 - e0), 0, 1);
    return t * t * (3 - 2 * t);
}

/// Wrap angle to [-pi, pi].
pub fn wrapPi(a: f32) f32 {
    var x = a;
    while (x > pi) x -= 2 * pi;
    while (x < -pi) x += 2 * pi;
    return x;
}
pub const pi: f32 = 3.14159265358979;
pub const deg: f32 = pi / 180.0;

pub fn atan2(y: f32, x: f32) f32 {
    return c.atan2f(y, x);
}
pub fn asin(x: f32) f32 {
    return c.asinf(clampf(x, -1, 1));
}

/// Rotate `cur` toward `target` by at most `maxStep` radians.
pub fn approachAngle(cur: f32, target: f32, maxStep: f32) f32 {
    const d = wrapPi(target - cur);
    if (@abs(d) <= maxStep) return target;
    return cur + (if (d > 0) maxStep else -maxStep);
}

/// Rigid transform: rotation as three basis columns + translation.
pub const Xf = struct {
    x: V3 = v3(1, 0, 0),
    y: V3 = v3(0, 1, 0),
    z: V3 = v3(0, 0, 1),
    p: V3 = zero,

    pub fn point(t: Xf, l: V3) V3 {
        return add(t.p, add(add(scale(t.x, l.x), scale(t.y, l.y)), scale(t.z, l.z)));
    }
    pub fn dir(t: Xf, l: V3) V3 {
        return add(add(scale(t.x, l.x), scale(t.y, l.y)), scale(t.z, l.z));
    }
    /// parent * child
    pub fn mul(t: Xf, ch: Xf) Xf {
        return .{ .x = t.dir(ch.x), .y = t.dir(ch.y), .z = t.dir(ch.z), .p = t.point(ch.p) };
    }
    pub fn toMatrix(t: Xf, s: f32) c.Matrix {
        return .{
            .m0 = t.x.x * s, .m1 = t.x.y * s, .m2 = t.x.z * s, .m3 = 0,
            .m4 = t.y.x * s, .m5 = t.y.y * s, .m6 = t.y.z * s, .m7 = 0,
            .m8 = t.z.x * s, .m9 = t.z.y * s, .m10 = t.z.z * s, .m11 = 0,
            .m12 = t.p.x, .m13 = t.p.y, .m14 = t.p.z, .m15 = 1,
        };
    }
};

/// Yaw about +Y (yaw 0 = facing +X, positive yaw turns toward -Z), then pitch about local Z.
pub fn yawPitch(p: V3, yaw: f32, pitch: f32) Xf {
    const cy = @cos(yaw);
    const sy = @sin(yaw);
    const cp = @cos(pitch);
    const sp = @sin(pitch);
    const fwd = v3(cy * cp, sp, -sy * cp);
    const right = v3(sy, 0, cy);
    const u = cross(right, fwd);
    return .{ .x = fwd, .y = u, .z = right, .p = p };
}

/// Basis from a forward direction (model +X), with world up as reference, plus roll.
pub fn lookFwd(p: V3, fwd_in: V3, roll: f32) Xf {
    const f = norm(fwd_in);
    var r = cross(f, up);
    if (len2(r) < 1e-6) r = v3(0, 0, 1);
    r = norm(r);
    var u = cross(r, f);
    if (roll != 0) {
        const cr = @cos(roll);
        const sr = @sin(roll);
        const uu = add(scale(u, cr), scale(r, sr));
        const r2 = sub(scale(r, cr), scale(u, sr));
        u = uu;
        r = r2;
    }
    return .{ .x = f, .y = u, .z = r, .p = p };
}

/// Yaw angle (same convention as yawPitch) of a direction.
pub fn yawOf(d: V3) f32 {
    return atan2(-d.z, d.x);
}
pub fn pitchOf(d: V3) f32 {
    return atan2(d.y, @sqrt(d.x * d.x + d.z * d.z));
}

// ---------------------------------------------------------------- noise
pub fn hash2(ix: i32, iz: i32, sd: u32) f32 {
    var h: u32 = @as(u32, @bitCast(ix)) *% 374761393 +% @as(u32, @bitCast(iz)) *% 668265263 +% sd *% 2246822519;
    h = (h ^ (h >> 13)) *% 1274126177;
    h ^= h >> 16;
    return @as(f32, @floatFromInt(h & 0xffffff)) / 16777215.0;
}

pub fn vnoise(x: f32, z: f32, sd: u32) f32 {
    const fx = @floor(x);
    const fz = @floor(z);
    const ix: i32 = @intFromFloat(fx);
    const iz: i32 = @intFromFloat(fz);
    var tx = x - fx;
    var tz = z - fz;
    tx = tx * tx * (3 - 2 * tx);
    tz = tz * tz * (3 - 2 * tz);
    const a = hash2(ix, iz, sd);
    const b = hash2(ix + 1, iz, sd);
    const cc = hash2(ix, iz + 1, sd);
    const d = hash2(ix + 1, iz + 1, sd);
    return lerpf(lerpf(a, b, tx), lerpf(cc, d, tx), tz);
}

pub fn fbm(x: f32, z: f32, oct: u32, sd: u32) f32 {
    var sum: f32 = 0;
    var amp: f32 = 0.5;
    var f: f32 = 1;
    var i: u32 = 0;
    while (i < oct) : (i += 1) {
        sum += amp * vnoise(x * f, z * f, sd + i * 17);
        f *= 2.03;
        amp *= 0.5;
    }
    return sum;
}

// ---------------------------------------------------------------- random
var rng_state: u64 = 0x9E3779B97F4A7C15;
pub fn seed(s: u64) void {
    rng_state = s | 1;
}
pub fn rnd() f32 {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return @as(f32, @floatFromInt(rng_state >> 40)) / 16777216.0;
}
pub fn rr(a: f32, b: f32) f32 {
    return a + (b - a) * rnd();
}
pub fn rndDir() V3 {
    while (true) {
        const d = v3(rr(-1, 1), rr(-1, 1), rr(-1, 1));
        const l = len2(d);
        if (l > 0.01 and l <= 1) return scale(d, 1.0 / @sqrt(l));
    }
}
