const std = @import("std");

const ExampleOptions = struct {
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    zozz_runtime: *std.Build.Module,
    cozz_runtime: *std.Build.Step.Compile,
};

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

    //
    // C++
    //

    const cozz_runtime = b.addLibrary(.{
        .name = "cozz_runtime",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    b.installArtifact(cozz_runtime);
    cozz_runtime.root_module.addIncludePath(b.path("ozz/include"));
    cozz_runtime.root_module.addCSourceFiles(.{
        .files = &.{
            "cozz/cozz_runtime.cpp",

            "ozz/src_fused/ozz_animation.cc",
            "ozz/src_fused/ozz_base.cc",
        },
        .flags = &.{
            "-std=c++20",
            "-fno-exceptions",
        },
    });
    cozz_runtime.root_module.link_libc = true;
    cozz_runtime.root_module.link_libcpp = true;

    const cozz_offline = b.addLibrary(.{
        .name = "cozz_offline",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    b.installArtifact(cozz_offline);
    cozz_offline.root_module.addIncludePath(b.path("ozz/include"));
    cozz_offline.root_module.addIncludePath(b.path("libs"));
    cozz_offline.root_module.addCSourceFiles(.{
        .files = &.{
            "cozz/cozz_offline.cpp",

            "ozz/src_fused/ozz_base.cc",
            "ozz/src_fused/ozz_animation.cc",
            "ozz/src_fused/ozz_animation_offline.cc",
        },
        .flags = &.{
            "-std=c++20",
        },
    });
    cozz_offline.root_module.link_libc = true;
    cozz_offline.root_module.link_libcpp = true;

    const importer = b.addExecutable(.{
        .name = "zozz-import",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/zozz_import.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    importer.root_module.addIncludePath(b.path("cozz"));
    importer.root_module.linkLibrary(cozz_offline);
    b.installArtifact(importer);
    const importer_step = b.step("importer", "Build the glTF/GLB to ozz importer");
    importer_step.dependOn(&b.addInstallArtifact(importer, .{}).step);

    //
    // Example
    //

    const build_examples = b.option(bool, "examples", "Build graphical examples") orelse false;
    if (build_examples) {
    const debug_skeleton = buildDebugSkeletonExample(b, .{
        .target = target,
        .optimize = optimize,
        .zozz_runtime = zozz_runtime,
        .cozz_runtime = cozz_runtime,
    });
    const install_debug_skeleton = b.addInstallArtifact(debug_skeleton, .{});

    const install_assets = b.addInstallDirectory(.{
        .source_dir = b.path("assets"),
        .install_dir = .{ .custom = "" },
        .install_subdir = "bin/assets",
    });
    b.getInstallStep().dependOn(&install_assets.step);

    const debug_skeleton_step = b.step("debug-skeleton", "Build the debug skeleton WGPU example");
    debug_skeleton_step.dependOn(&install_debug_skeleton.step);
    debug_skeleton_step.dependOn(&install_assets.step);

    const run_debug_skeleton = b.addRunArtifact(debug_skeleton);
    run_debug_skeleton.step.dependOn(&install_debug_skeleton.step);
    run_debug_skeleton.step.dependOn(&install_assets.step);

    const debug_skeleton_run_step = b.step("debug-skeleton-run", "Build and run the debug skeleton WGPU example");
    debug_skeleton_run_step.dependOn(&run_debug_skeleton.step);
    }

    //
    // Tests
    //

    const test_step = b.step("test", "Run zozz tests");

    const tests_runtime = b.addTest(.{
        .name = "zozz_runtime_tests",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/zozz_runtime.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    b.installArtifact(tests_runtime);
    tests_runtime.root_module.addIncludePath(b.path("cozz"));
    tests_runtime.root_module.linkLibrary(cozz_runtime);
    test_step.dependOn(&b.addRunArtifact(tests_runtime).step);

    const tests_offline = b.addTest(.{
        .name = "zozz_offline_tests",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/zozz_offline.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    b.installArtifact(tests_offline);
    tests_offline.root_module.addIncludePath(b.path("cozz"));
    tests_offline.root_module.linkLibrary(cozz_offline);
    test_step.dependOn(&b.addRunArtifact(tests_offline).step);
}

fn buildDebugSkeletonExample(b: *std.Build, options: ExampleOptions) *std.Build.Step.Compile {
    const exe = b.addExecutable(.{
        .name = "debug_skeleton_wgpu",
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/debug_skeleton_wgpu.zig"),
            .target = options.target,
            .optimize = options.optimize,
        }),
    });

    exe.root_module.addImport("zozz_runtime", options.zozz_runtime);
    exe.root_module.linkLibrary(options.cozz_runtime);

    const zglfw = b.dependency("zglfw", .{
        .target = options.target,
        .optimize = options.optimize,
    });
    exe.root_module.addImport("zglfw", zglfw.module("root"));
    exe.root_module.linkLibrary(zglfw.artifact("glfw"));

    const zgpu = b.dependency("zgpu", .{
        .target = options.target,
        .optimize = options.optimize,
    });
    exe.root_module.addImport("zgpu", zgpu.module("root"));
    exe.root_module.linkLibrary(zgpu.artifact("zdawn"));

    const zmath = b.dependency("zmath", .{
        .target = options.target,
        .optimize = options.optimize,
    });
    exe.root_module.addImport("zmath", zmath.module("root"));

    return exe;
}
