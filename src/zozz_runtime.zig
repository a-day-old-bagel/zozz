const std = @import("std");

pub const c = @cImport({
    @cInclude("cozz_runtime.h");
});

pub const OzzError = error{
    InvalidArgument,
    Io,
    OzzFailure,
    Unknown,
};

fn mapResult(rc: c.ozz_result_t) OzzError!void {
    switch (rc) {
        c.OZZ_OK => return,
        c.OZZ_ERR_INVALID_ARGUMENT => return OzzError.InvalidArgument,
        c.OZZ_ERR_IO => return OzzError.Io,
        c.OZZ_ERR_OZZ => {
            std.debug.print("cozz error: {s}\n", .{std.mem.span(c.ozz_last_error())});
            return OzzError.OzzFailure;
        },
        else => {
            std.debug.print("cozz error (unknown): {s}\n", .{std.mem.span(c.ozz_last_error())});
            return OzzError.Unknown;
        },
    }
}

pub fn lastErrorZ() [:0]const u8 {
    return std.mem.span(c.ozz_last_error());
}

pub fn clearError() void {
    c.ozz_clear_error();
}

// --------------------
// Loaded runtime assets
// --------------------

pub const Skeleton = struct {
    handle: *c.ozz_skeleton_t,

    pub fn loadFromFileZ(path_z: [:0]const u8) !Skeleton {
        var out: ?*c.ozz_skeleton_t = null;
        try mapResult(c.ozz_skeleton_load_from_file(path_z.ptr, &out));
        return .{ .handle = out.? };
    }

    pub fn deinit(self: *Skeleton) void {
        c.ozz_skeleton_destroy(self.handle);
        self.* = undefined;
    }

    pub fn numJoints(self: Skeleton) i32 {
        return c.ozz_skeleton_num_joints(self.handle);
    }

    pub fn instanceBytes(self: Skeleton) usize {
        return c.ozz_instance_required_bytes(self.handle);
    }

    pub fn workspaceBytes(self: Skeleton) usize {
        return c.ozz_workspace_required_bytes(self.handle);
    }
};

pub const Animation = struct {
    handle: *c.ozz_animation_t,

    pub fn loadFromFileZ(path_z: [:0]const u8) !Animation {
        var out: ?*c.ozz_animation_t = null;
        try mapResult(c.ozz_animation_load_from_file(path_z.ptr, &out));
        return .{ .handle = out.? };
    }

    pub fn deinit(self: *Animation) void {
        c.ozz_animation_destroy(self.handle);
        self.* = undefined;
    }

    pub fn duration(self: Animation) f32 {
        return c.ozz_animation_duration(self.handle);
    }
};

// --------------------
// Layers
// --------------------

pub const LayerMode = enum(u32) {
    normal = c.OZZ_LAYER_NORMAL,
    additive = c.OZZ_LAYER_ADDITIVE,
};

pub const Layer = struct {
    anim: Animation,
    time_seconds: f32,
    wrap_time: bool = true,
    weight: f32,
    mode: LayerMode = .normal,
};

// --------------------
// Per-entity Instance
// --------------------

pub const Instance = struct {
    storage: []align(16) u8,
    handle: *c.ozz_instance_t,

    pub fn init(allocator: std.mem.Allocator, skel: Skeleton) !Instance {
        const bytes = skel.instanceBytes();
        const storage = try allocator.alignedAlloc(u8, 16, bytes);
        errdefer allocator.free(storage);

        var out: ?*c.ozz_instance_t = null;
        try mapResult(c.ozz_instance_init(storage.ptr, storage.len, skel.handle, &out));

        return .{ .storage = storage, .handle = out.? };
    }

    pub fn deinit(self: *Instance, allocator: std.mem.Allocator) void {
        c.ozz_instance_deinit(self.handle);
        allocator.free(self.storage);
        self.* = undefined;
    }

    pub fn setLayers(self: *Instance, layers: []const Layer) void {
        // Hard limit in the C wrapper is 8.
        if (layers.len > c.OZZ_MAX_LAYERS) @panic("too many layers (max 8)");

        var tmp: [c.OZZ_MAX_LAYERS]c.ozz_layer_desc_t = undefined;

        for (layers, 0..) |L, i| {
            tmp[i] = .{
                .anim = L.anim.handle,
                .time_seconds = L.time_seconds,
                .wrap_time = @intFromBool(L.wrap_time),
                .weight = L.weight,
                .mode = @intCast(@intFromEnum(L.mode)),
            };
        }

        c.ozz_instance_set_layers(
            self.handle,
            &tmp[0],
            @intCast(layers.len),
        );
    }

    // Optional: expose later if you want IK from Zig.
    // pub fn setIkJobs(...) void { ... }
};

// --------------------
// Per-worker Workspace
// --------------------

pub const Workspace = struct {
    storage: []align(16) u8,
    handle: *c.ozz_workspace_t,

    pub fn init(allocator: std.mem.Allocator, skel: Skeleton) !Workspace {
        const bytes = skel.workspaceBytes();
        const storage = try allocator.alignedAlloc(u8, 16, bytes);
        errdefer allocator.free(storage);

        var out: ?*c.ozz_workspace_t = null;
        try mapResult(c.ozz_workspace_init(storage.ptr, storage.len, skel.handle, &out));

        return .{ .storage = storage, .handle = out.? };
    }

    pub fn deinit(self: *Workspace, allocator: std.mem.Allocator) void {
        c.ozz_workspace_deinit(self.handle);
        allocator.free(self.storage);
        self.* = undefined;
    }

    pub fn palette3x4(self: Workspace) []const f32 {
        const ptr = c.ozz_workspace_palette_3x4(self.handle);
        const len = @as(usize, @intCast(c.ozz_workspace_palette_floats(self.handle)));
        return ptr[0..len];
    }
};

// --------------------
// Evaluate
// --------------------

pub fn evalModel3x4(inst: *Instance, ws: *Workspace) ![]const f32 {
    try mapResult(c.ozz_eval_model_3x4(inst.handle, ws.handle));
    return ws.palette3x4();
}

// --------------------
// Tests
// --------------------

test "ozz C ABI wrapper: load + 2-clip blend + 3x4 palette is sane" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var jog = try Animation.loadFromFileZ("assets/pab_jog_no_motion.ozz");
    defer jog.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws = try Workspace.init(A, skel);
    defer ws.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = jog, .time_seconds = 0.10, .wrap_time = true, .weight = 0.35, .mode = .normal },
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 0.65, .mode = .normal },
    });

    const palette = try evalModel3x4(&inst, &ws);

    const joints = @as(usize, @intCast(skel.numJoints()));
    try std.testing.expectEqual(@as(usize, 12) * joints, palette.len);

    var sum_abs: f32 = 0;
    for (palette) |v| sum_abs += @abs(v);
    try std.testing.expect(sum_abs > 0.001);

    for (0..joints) |j| {
        const base = j * 12;
        const tx = palette[base + 9];
        const ty = palette[base + 10];
        const tz = palette[base + 11];
        try std.testing.expect(std.math.isFinite(tx));
        try std.testing.expect(std.math.isFinite(ty));
        try std.testing.expect(std.math.isFinite(tz));
    }
}
