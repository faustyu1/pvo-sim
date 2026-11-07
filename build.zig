const std = @import("std");

const raylib_src = "vendor/raylib/src";

/// When targeting macOS explicitly (e.g. x86_64-macos on an arm64 Mac), Zig does
/// not look up the SDK by itself, so point it at the one xcrun reports.
fn addMacSdk(b: *std.Build, mod: *std.Build.Module) void {
    if (@import("builtin").os.tag != .macos) return;
    const sdk = std.mem.trim(u8, b.run(&.{ "xcrun", "--sdk", "macosx", "--show-sdk-path" }), " \n\r");
    mod.addSystemFrameworkPath(.{ .cwd_relative = b.pathJoin(&.{ sdk, "System/Library/Frameworks" }) });
    mod.addSystemIncludePath(.{ .cwd_relative = b.pathJoin(&.{ sdk, "usr/include" }) });
    mod.addLibraryPath(.{ .cwd_relative = b.pathJoin(&.{ sdk, "usr/lib" }) });
}

/// raylib 6.2 is vendored in vendor/raylib and compiled from source with
/// zig cc, so the game builds the same way on macOS, Windows and Linux.
fn buildRaylib(b: *std.Build, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode) *std.Build.Step.Compile {
    const mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    const os = target.result.os.tag;
    mod.addIncludePath(b.path(raylib_src));
    mod.addIncludePath(b.path(raylib_src ++ "/platforms"));
    mod.addIncludePath(b.path(raylib_src ++ "/external/glfw/include"));
    mod.addCMacro("PLATFORM_DESKTOP_GLFW", "");
    mod.addCMacro("GRAPHICS_API_OPENGL_33", "");
    inline for (.{ "RSHAPES", "RTEXTURES", "RTEXT", "RMODELS", "RAUDIO" }) |m| mod.addCMacro("SUPPORT_MODULE_" ++ m, "1");

    const flags = &[_][]const u8{"-std=c99"};
    mod.addCSourceFiles(.{
        .root = b.path(raylib_src),
        .files = &.{ "rcore.c", "rshapes.c", "rtextures.c", "rtext.c", "rmodels.c", "raudio.c" },
        .flags = flags,
    });
    switch (os) {
        .macos => {
            addMacSdk(b, mod);
            // rglfw.c pulls in the Cocoa backend, which is Objective-C
            mod.addCSourceFile(.{ .file = b.path(raylib_src ++ "/rglfw.c"), .flags = &.{ "-std=c99", "-ObjC" } });
            for ([_][]const u8{ "Foundation", "CoreServices", "CoreGraphics", "AppKit", "IOKit", "QuartzCore", "CoreVideo", "OpenGL", "CoreAudio", "AudioToolbox" }) |f|
                mod.linkFramework(f, .{});
        },
        .windows => {
            mod.addCSourceFile(.{ .file = b.path(raylib_src ++ "/rglfw.c"), .flags = flags });
            for ([_][]const u8{ "opengl32", "winmm", "gdi32", "user32", "shell32" }) |l| mod.linkSystemLibrary(l, .{});
        },
        else => {
            mod.addCMacro("_GLFW_X11", "");
            mod.addCSourceFile(.{ .file = b.path(raylib_src ++ "/rglfw.c"), .flags = flags });
            for ([_][]const u8{ "GL", "X11", "Xrandr", "Xinerama", "Xi", "Xcursor", "m", "pthread", "dl", "rt" }) |l| mod.linkSystemLibrary(l, .{});
        },
    }
    return b.addLibrary(.{ .name = "raylib", .root_module = mod, .linkage = .static });
}

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    // ReleaseFast by default: Debug is far too slow for the renderer/sim loops.
    const optimize = b.option(std.builtin.OptimizeMode, "optimize", "Optimization mode (default ReleaseFast)") orelse .ReleaseFast;

    const raylib = buildRaylib(b, target, optimize);

    const tc = b.addTranslateC(.{
        .root_source_file = b.path("src/c.h"),
        .target = target,
        .optimize = optimize,
    });
    tc.addIncludePath(b.path(raylib_src));

    const mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    mod.addImport("c", tc.createModule());
    mod.linkLibrary(raylib);
    if (target.result.os.tag == .macos) addMacSdk(b, mod);

    const exe = b.addExecutable(.{ .name = "pvo", .root_module = mod });
    // no console window on Windows
    if (target.result.os.tag == .windows) exe.subsystem = .windows;
    b.installArtifact(exe);
    b.installDirectory(.{ .source_dir = b.path("assets"), .install_dir = .bin, .install_subdir = "assets" });

    const run = b.addRunArtifact(exe);
    run.setCwd(b.path("."));
    run.step.dependOn(b.getInstallStep());
    b.step("run", "Run simulator").dependOn(&run.step);
}
