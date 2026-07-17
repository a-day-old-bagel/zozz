const std = @import("std");

const c = @cImport({
    @cInclude("cozz_offline.h");
});

pub const Error = error{
    InvalidArgument,
    Io,
    InvalidData,
    Ozz,
};

pub const Vec3 = extern struct { x: f32, y: f32, z: f32 };
pub const Quaternion = extern struct { x: f32, y: f32, z: f32, w: f32 };
pub const Transform = extern struct {
    translation: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    rotation: Quaternion = .{ .x = 0, .y = 0, .z = 0, .w = 1 },
    scale: Vec3 = .{ .x = 1, .y = 1, .z = 1 },
};
pub const Joint = extern struct {
    name: [*:0]const u8,
    parent: i32,
    rest_pose: Transform = .{},
};
pub const Vec3Key = extern struct { time: f32, value: Vec3 };
pub const QuaternionKey = extern struct { time: f32, value: Quaternion };
pub const Track = extern struct {
    translations: ?[*]const Vec3Key = null,
    translation_count: usize = 0,
    rotations: ?[*]const QuaternionKey = null,
    rotation_count: usize = 0,
    scales: ?[*]const Vec3Key = null,
    scale_count: usize = 0,

    pub fn init(translations: []const Vec3Key, rotations: []const QuaternionKey, scales: []const Vec3Key) Track {
        return .{
            .translations = if (translations.len == 0) null else translations.ptr,
            .translation_count = translations.len,
            .rotations = if (rotations.len == 0) null else rotations.ptr,
            .rotation_count = rotations.len,
            .scales = if (scales.len == 0) null else scales.ptr,
            .scale_count = scales.len,
        };
    }
};

comptime {
    std.debug.assert(@sizeOf(Vec3) == @sizeOf(c.ozz_offline_vec3_t));
    std.debug.assert(@sizeOf(Quaternion) == @sizeOf(c.ozz_offline_quat_t));
    std.debug.assert(@sizeOf(Transform) == @sizeOf(c.ozz_offline_transform_t));
    std.debug.assert(@sizeOf(Joint) == @sizeOf(c.ozz_offline_joint_t));
    std.debug.assert(@sizeOf(Track) == @sizeOf(c.ozz_offline_track_t));
}

pub fn lastError() []const u8 {
    return std.mem.span(c.ozz_offline_last_error());
}

fn check(result: c.ozz_offline_result_t) Error!void {
    return switch (result) {
        c.OZZ_OFFLINE_OK => {},
        c.OZZ_OFFLINE_ERR_INVALID_ARGUMENT => Error.InvalidArgument,
        c.OZZ_OFFLINE_ERR_IO => Error.Io,
        c.OZZ_OFFLINE_ERR_INVALID_DATA => Error.InvalidData,
        else => Error.Ozz,
    };
}

pub fn buildSkeleton(joints: []const Joint, output_path: [*:0]const u8) Error!void {
    try check(c.ozz_offline_build_skeleton(
        if (joints.len == 0) null else @ptrCast(joints.ptr),
        joints.len,
        output_path,
    ));
}

pub fn buildAnimation(name: [*:0]const u8, duration: f32, tracks: []const Track, output_path: [*:0]const u8) Error!void {
    try check(c.ozz_offline_build_animation(
        name,
        duration,
        if (tracks.len == 0) null else @ptrCast(tracks.ptr),
        tracks.len,
        output_path,
    ));
}

pub fn buildAdditiveAnimation(name: [*:0]const u8, duration: f32, tracks: []const Track, output_path: [*:0]const u8) Error!void {
    try check(c.ozz_offline_build_additive_animation(
        name,
        duration,
        if (tracks.len == 0) null else @ptrCast(tracks.ptr),
        tracks.len,
        output_path,
    ));
}

test "offline ABI rejects non-depth-first skeletons" {
    const joints = [_]Joint{
        .{ .name = "root", .parent = -1 },
        .{ .name = "bad", .parent = 1 },
    };
    try std.testing.expectError(Error.InvalidData, buildSkeleton(&joints, "unused.ozz"));
    try std.testing.expect(lastError().len != 0);
}

test "offline ABI rejects unsorted animation keys" {
    const keys = [_]Vec3Key{
        .{ .time = 0.5, .value = .{ .x = 0, .y = 0, .z = 0 } },
        .{ .time = 0.25, .value = .{ .x = 1, .y = 0, .z = 0 } },
    };
    const tracks = [_]Track{Track.init(&keys, &.{}, &.{})};
    try std.testing.expectError(Error.InvalidData, buildAnimation("bad", 1, &tracks, "unused.ozz"));
}
