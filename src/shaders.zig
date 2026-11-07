//! GLSL 330 sources. macOS gives a 4.1 core context (via Metal), so 330 is safe.

pub const lit_vs =
    \\#version 330
    \\in vec3 vertexPosition;
    \\in vec3 vertexNormal;
    \\in vec4 vertexColor;
    \\uniform mat4 mvp;
    \\uniform mat4 matModel;
    \\uniform mat4 matNormal;
    \\uniform mat4 lightVP;
    \\out vec3 fragPos;
    \\out vec3 fragNormal;
    \\out vec4 fragColor;
    \\out vec4 fragLight;
    \\void main() {
    \\    vec4 wp = matModel * vec4(vertexPosition, 1.0);
    \\    fragPos = wp.xyz;
    \\    fragNormal = normalize(mat3(matNormal) * vertexNormal);
    \\    fragColor = vertexColor;
    \\    fragLight = lightVP * wp;
    \\    gl_Position = mvp * vec4(vertexPosition, 1.0);
    \\}
;

pub const common_fs =
    \\uniform vec3 sunDir;
    \\uniform vec3 sunColor;
    \\uniform vec3 skyColor;
    \\uniform vec3 groundColor;
    \\uniform vec3 fogColor;
    \\uniform vec3 viewPos;
    \\uniform float fogDensity;
    \\vec3 aces(vec3 x) {
    \\    const float a = 2.51; const float b = 0.03; const float c = 2.43; const float d = 0.59; const float e = 0.14;
    \\    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
    \\}
    \\vec3 applyFog(vec3 col, vec3 p, vec3 rd) {
    \\    float d = length(p - viewPos);
    \\    // height-attenuated exponential fog
    \\    float h = max(p.y, 0.0);
    \\    float f = 1.0 - exp(-d * fogDensity * (0.55 + 0.45 * exp(-h * 0.0008)));
    \\    float sunAmt = max(dot(rd, sunDir), 0.0);
    \\    vec3 fc = fogColor * (1.0 + 0.12 * pow(sunAmt, 6.0));
    \\    return mix(col, fc, clamp(f, 0.0, 1.0));
    \\}
    \\vec3 finish(vec3 col) {
    \\    col = aces(col * 0.85);
    \\    float l = dot(col, vec3(0.299, 0.587, 0.114));
    \\    col = mix(vec3(l), col, 0.68);
    \\    col *= vec3(0.95, 0.99, 1.05);
    \\    return pow(col, vec3(1.0 / 2.2));
    \\}
;

pub const lit_fs =
    \\#version 330
    \\in vec3 fragPos;
    \\in vec3 fragNormal;
    \\in vec4 fragColor;
    \\in vec4 fragLight;
    \\uniform vec4 colDiffuse;
    \\uniform sampler2D shadowMap;
    \\uniform float shadowTexel;
    \\uniform float emissive;
    \\uniform int isTerrain;
    \\uniform vec4 roads[8];
    \\uniform int roadCount;
    \\out vec4 finalColor;
++ "\n" ++ common_fs ++ "\n" ++
    \\// integer hash: the fract-based one correlates along rays through the origin
    \\float h21(vec2 p) {
    \\    uvec2 q = uvec2(ivec2(floor(p)) + ivec2(65536));
    \\    uint h = q.x * 1597334677u ^ q.y * 3812015801u;
    \\    h = (h ^ (h >> 16u)) * 2246822519u;
    \\    h ^= h >> 13u;
    \\    return float(h & 0xffffffu) / 16777215.0;
    \\}
    \\float n2(vec2 p) {
    \\    vec2 i = floor(p); vec2 f = fract(p); f = f * f * (3.0 - 2.0 * f);
    \\    return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y);
    \\}
    \\float fbm(vec2 p) {
    \\    float s = 0.0; float a = 0.5;
    \\    for (int i = 0; i < 4; i++) { s += a * n2(p); p = p * 2.03 + vec2(1.7, 9.2); a *= 0.5; }
    \\    return s;
    \\}
    \\float segDist(vec2 p, vec2 a, vec2 b) {
    \\    vec2 pa = p - a; vec2 ba = b - a;
    \\    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    \\    return length(pa - ba * h);
    \\}
    \\vec3 terrainAlbedo(vec3 p, vec3 n, vec4 vc) {
    \\    float forest = vc.r;
    \\    float slope = 1.0 - n.y;
    \\    vec2 q = p.xz;
    \\    float big = vc.g; // low-frequency noise baked per vertex
    \\    // distance LOD: past ~2 km the fog hides fine noise, so skip computing it
    \\    float dv = length(p - viewPos);
    \\    float det = 0.5;
    \\    if (dv < 2200.0) det = mix(fbm(q * 0.06), 0.5, smoothstep(1600.0, 2200.0, dv));
    \\    float fine = 0.5;
    \\    if (dv < 900.0) fine = mix(n2(q * 0.7), 0.5, smoothstep(600.0, 900.0, dv));
    \\    vec3 grassA = vec3(0.26, 0.26, 0.20);
    \\    vec3 grassB = vec3(0.40, 0.35, 0.26);
    \\    vec3 col = mix(grassA, grassB, smoothstep(0.3, 0.7, big)) * (0.82 + 0.3 * det) * (0.95 + 0.1 * fine);
    \\    // agricultural patchwork
    \\    vec2 r = mat2(0.955, 0.296, -0.296, 0.955) * q;
    \\    vec2 sz = vec2(260.0, 160.0);
    \\    vec2 cell = floor(r / sz);
    \\    vec2 fr = fract(r / sz);
    \\    float hs = h21(cell + 17.0);
    \\    vec3 fc = hs < 0.25 ? vec3(0.46, 0.41, 0.31) : hs < 0.55 ? vec3(0.21, 0.18, 0.15) : hs < 0.75 ? vec3(0.27, 0.29, 0.21) : vec3(0.38, 0.34, 0.27);
    \\    float stripes = sin((hs > 0.5 ? r.x : r.y) * 1.3 + hs * 40.0) * 0.5 + 0.5;
    \\    fc *= (0.95 + 0.05 * stripes) * (0.88 + 0.24 * det);
    \\    vec2 em = min(fr, 1.0 - fr) * sz;
    \\    float edge = min(em.x, em.y);
    \\    float hedge = 1.0 - smoothstep(1.5, 4.5, edge);
    \\    float fieldMask = smoothstep(0.44, 0.52, vc.b) * (1.0 - smoothstep(0.06, 0.14, slope)) * (1.0 - smoothstep(4000.0, 6500.0, length(q)));
    \\    col = mix(col, fc, fieldMask);
    \\    col = mix(col, vec3(0.17, 0.17, 0.14) * (0.8 + 0.4 * fine), hedge * fieldMask * 0.85);
    \\    // forest floor
    \\    col = mix(col, vec3(0.11, 0.12, 0.10) * (0.75 + 0.5 * det), forest);
    \\    // wet dark patches / puddles
    \\    col *= 1.0 - 0.14 * smoothstep(0.6, 0.8, n2(q * 0.05 + 7.0) * 0.6 + det * 0.4);
    \\    // rock & snow
    \\    col = mix(col, vec3(0.44, 0.41, 0.37) * (0.8 + 0.4 * det), smoothstep(0.22, 0.42, slope));
    \\    col = mix(col, vec3(0.66, 0.68, 0.71), smoothstep(520.0, 640.0, p.y + det * 90.0) * (1.0 - smoothstep(0.35, 0.55, slope)));
    \\    vec3 preRoad = col;
    \\    // roads (pad drawn on top, so the battery sits on a clean gravel area)
    \\    for (int i = 0; i < 8; i++) {
    \\        if (i >= roadCount) break;
    \\        float d = segDist(q, roads[i].xy, roads[i].zw);
    \\        float w = i == 4 ? 4.2 : 2.8;
    \\        col = mix(col, vec3(0.45, 0.39, 0.30), 1.0 - smoothstep(w + 1.5, w + 5.0, d + det * 2.0));
    \\        col = mix(col, vec3(0.22, 0.22, 0.23) * (0.9 + 0.2 * fine), 1.0 - smoothstep(w - 0.6, w, d));
    \\        if (i == 4) col = mix(col, vec3(0.75, 0.72, 0.62), (1.0 - smoothstep(0.1, 0.25, d)) * step(0.5, fract(dot(q, normalize(roads[i].zw - roads[i].xy)) / 12.0)));
    \\    }
    \\    // gravel pad at the battery (only evaluated near it)
    \\    if (dot(q, q) < 70.0 * 70.0) {
    \\        float pad = 1.0 - smoothstep(50.0, 56.0, length(q + vec2(det * 6.0)));
    \\        float grit = n2(q * 3.1) * 0.5 + n2(q * 0.35) * 0.5;
    \\        vec3 padCol = mix(vec3(0.24, 0.23, 0.21), vec3(0.36, 0.34, 0.31), grit);
    \\        padCol = mix(padCol, preRoad * 0.8, smoothstep(0.55, 0.75, n2(q * 0.08)) * 0.6);
    \\        col = mix(col, padCol, pad);
    \\    }
    \\    // micro detail, faded with distance to avoid shimmering
    \\    if (dv < 400.0) {
    \\        float micro = n2(q * 4.3) * 0.55 + n2(q * 14.0) * 0.45;
    \\        col *= mix(0.8 + 0.4 * micro, 1.0, smoothstep(40.0, 400.0, dv));
    \\    }
    \\    return pow(col, vec3(2.2));
    \\}
    \\float shadow(vec3 n) {
    \\    vec3 p = fragLight.xyz / fragLight.w * 0.5 + 0.5;
    \\    if (p.x <= 0.0 || p.x >= 1.0 || p.y <= 0.0 || p.y >= 1.0 || p.z >= 1.0) return 1.0;
    \\    float ndl = max(dot(n, sunDir), 0.0);
    \\    float bias = mix(0.0009, 0.00015, ndl);
    \\    float s = 0.0;
    \\    for (int x = -1; x <= 1; x++)
    \\    for (int y = -1; y <= 1; y++) {
    \\        float d = texture(shadowMap, p.xy + vec2(x, y) * shadowTexel).r;
    \\        s += (p.z - bias > d) ? 0.0 : 1.0;
    \\    }
    \\    s /= 9.0;
    \\    // fade out near the shadow-map border
    \\    vec2 e = smoothstep(0.0, 0.06, p.xy) * smoothstep(0.0, 0.06, 1.0 - p.xy);
    \\    return mix(1.0, s, e.x * e.y);
    \\}
    \\void main() {
    \\    vec3 v = normalize(viewPos - fragPos);
    \\    // early out: fully fogged pixels skip all shading
    \\    float dd = length(fragPos - viewPos);
    \\    float fogF = 1.0 - exp(-dd * fogDensity * (0.55 + 0.45 * exp(-max(fragPos.y, 0.0) * 0.0008)));
    \\    if (fogF > 0.985) { finalColor = vec4(finish(applyFog(vec3(0.0), fragPos, -v)), colDiffuse.a); return; }
    \\    vec3 n = normalize(fragNormal);
    \\    if (!gl_FrontFacing) n = -n;
    \\    vec3 albedo = isTerrain == 1 ? terrainAlbedo(fragPos, n, fragColor) : colDiffuse.rgb * pow(fragColor.rgb, vec3(2.2));
    \\    float ndl = max(dot(n, sunDir), 0.0);
    \\    float sh = mix(0.35, 1.0, shadow(n));
    \\    vec3 hemi = mix(groundColor, skyColor, n.y * 0.5 + 0.5);
    \\    vec3 h = normalize(sunDir + v);
    \\    float spec = isTerrain == 1 ? 0.0 : pow(max(dot(n, h), 0.0), 48.0) * 0.12;
    \\    float rim = pow(1.0 - max(dot(n, v), 0.0), 4.0) * 0.08;
    \\    vec3 col = albedo * (sunColor * ndl * sh + hemi) + sunColor * spec * sh + skyColor * rim;
    \\    col += albedo * emissive;
    \\    col = applyFog(col, fragPos, -v);
    \\    finalColor = vec4(finish(col), colDiffuse.a);
    \\}
;

pub const sky_vs =
    \\#version 330
    \\in vec3 vertexPosition;
    \\uniform mat4 mvp;
    \\uniform mat4 matModel;
    \\out vec3 wpos;
    \\void main() {
    \\    wpos = (matModel * vec4(vertexPosition, 1.0)).xyz;
    \\    gl_Position = mvp * vec4(vertexPosition, 1.0);
    \\}
;

pub const sky_fs =
    \\#version 330
    \\in vec3 wpos;
    \\uniform float time;
    \\out vec4 finalColor;
++ "\n" ++ common_fs ++ "\n" ++
    \\uniform vec3 zenithColor;
    \\float h21(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
    \\float n2(vec2 p) {
    \\    vec2 i = floor(p); vec2 f = fract(p); f = f * f * (3.0 - 2.0 * f);
    \\    return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y);
    \\}
    \\float fbm(vec2 p) {
    \\    float s = 0.0; float a = 0.5;
    \\    for (int i = 0; i < 5; i++) { s += a * n2(p); p = p * 2.07 + vec2(1.7, 9.2); a *= 0.5; }
    \\    return s;
    \\}
    \\void main() {
    \\    vec3 rd = normalize(wpos - viewPos);
    \\    float y = rd.y;
    \\    float t = pow(clamp(y, 0.0, 1.0), 0.45);
    \\    vec3 col = mix(fogColor, zenithColor, t);
    \\    float sd = max(dot(rd, sunDir), 0.0);
    \\    // overcast: full cloud deck, sun only a faint brighter patch
    \\    if (y > 0.0) {
    \\        vec2 uv = rd.xz / (y + 0.05) * 0.35 + vec2(time * 0.006, time * 0.002);
    \\        float cl = fbm(uv);
    \\        float cl2 = fbm(uv * 2.7 + 5.0);
    \\        vec3 dark = zenithColor * 0.72;
    \\        vec3 light = mix(zenithColor, fogColor, 0.5) * 1.12;
    \\        vec3 cc = mix(dark, light, smoothstep(0.3, 0.75, cl * 0.7 + cl2 * 0.3));
    \\        cc += sunColor * pow(sd, 5.0) * 0.12;
    \\        col = mix(col, cc, smoothstep(0.0, 0.22, y));
    \\    }
    \\    if (y < 0.0) col = mix(fogColor, groundColor * 1.5, smoothstep(0.0, -0.3, y));
    \\    finalColor = vec4(finish(col), 1.0);
    \\}
;

/// For billboard particles: textured, tint already fogged on CPU.
pub const part_fs =
    \\#version 330
    \\in vec2 fragTexCoord;
    \\in vec4 fragColor;
    \\uniform sampler2D texture0;
    \\out vec4 finalColor;
    \\void main() {
    \\    vec4 t = texture(texture0, fragTexCoord);
    \\    finalColor = t * fragColor;
    \\}
;

/// Shadow pass: depth only, no shading.
pub const depth_vs =
    \\#version 330
    \\in vec3 vertexPosition;
    \\uniform mat4 mvp;
    \\void main() { gl_Position = mvp * vec4(vertexPosition, 1.0); }
;
pub const depth_fs =
    \\#version 330
    \\out vec4 finalColor;
    \\void main() { finalColor = vec4(1.0); }
;
