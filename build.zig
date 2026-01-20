const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const zozz_runtime = b.addModule("zozz_runtime", .{
        .root_source_file = b.path("src/zozz_runtime.zig"),
        .imports = &.{},
    });
    zozz_runtime.addIncludePath(b.path("cozz"));

    const zozz_offline = b.addModule("zozz_offline", .{
        .root_source_file = b.path("src/zozz_offline.zig"),
        .imports = &.{},
    });
    zozz_offline.addIncludePath(b.path("cozz"));

    const cpp_flags: []const []const u8 = &.{
        "-std=c++17",
        "-fno-exceptions",
    };

    const cozz_runtime = b.addStaticLibrary(.{
        .name = "cozz_runtime",
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(cozz_runtime);
    cozz_runtime.addIncludePath(b.path("ozz/include"));
    cozz_runtime.addCSourceFiles(.{
        .files = &.{
            "cozz/cozz_runtime.cpp",

            "ozz/src_fused/ozz_animation.cpp",
            // "ozz/src_fused/ozz_base.cpp",
            // "ozz/src_fused/ozz_geometry.cpp",
        },
        .flags = cpp_flags,
    });
    cozz_runtime.linkLibC();
    cozz_runtime.linkLibCpp();

    const cozz_offline = b.addStaticLibrary(.{
        .name = "cozz_offline",
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(cozz_offline);
    cozz_offline.addIncludePath(b.path("ozz/include"));
    cozz_offline.addCSourceFiles(.{
        .files = &.{
            "cozz/cozz_offline.cpp",

            "ozz/src_fused/ozz_animation_offline.cpp",
            "ozz/src_fused/ozz_animation_tools.cpp",
        },
        .flags = cpp_flags,
    });
    cozz_offline.linkLibC();
    cozz_offline.linkLibCpp();

    //
    // Tests
    //

    const test_step = b.step("test", "Run zozz tests");

    const tests_runtime = b.addTest(.{
        .name = "zozz_runtime_tests",
        .root_source_file = b.path("src/zozz_runtime.zig"),
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(tests_runtime);
    tests_runtime.addIncludePath(b.path("cozz"));
    tests_runtime.linkLibrary(cozz_runtime);
    test_step.dependOn(&b.addRunArtifact(tests_runtime).step);

    const tests_offline = b.addTest(.{
        .name = "zozz_offline_tests",
        .root_source_file = b.path("src/zozz_offline.zig"),
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(tests_offline);
    tests_offline.addIncludePath(b.path("cozz"));
    tests_offline.linkLibrary(cozz_offline);
    test_step.dependOn(&b.addRunArtifact(tests_offline).step);
}
