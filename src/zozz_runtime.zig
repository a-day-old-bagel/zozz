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

    pub fn findJointZ(self: Skeleton, name_z: [:0]const u8) i32 {
        return c.ozz_skeleton_find_joint(self.handle, name_z.ptr);
    }

    pub fn jointName(self: Skeleton, joint: i32) ?[:0]const u8 {
        const ptr = c.ozz_skeleton_joint_name(self.handle, joint);
        if (ptr == null) return null;
        return std.mem.span(ptr);
    }

    pub fn jointParent(self: Skeleton, joint: i32) i32 {
        return c.ozz_skeleton_joint_parent(self.handle, joint);
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

pub const Vec3 = extern struct {
    x: f32,
    y: f32,
    z: f32,
};

pub const IkJob = extern struct {
    kind: c.ozz_ik_kind_t,
    weight: f32,
    start_joint: i32,
    mid_joint: i32,
    end_joint: i32,
    target_ms: Vec3,
    pole_ms: Vec3,
    mid_axis_ls: Vec3,
    twist_angle: f32,
    soften: f32,
    aim_joint: i32,
    aim_target_ms: Vec3,
    forward_axis_ls: Vec3,
    up_axis_ls: Vec3,
    aim_pole_vector_ms: Vec3,

    pub fn aim(joint: i32, target_ms: Vec3, forward_axis_ls: Vec3, up_axis_ls: Vec3, weight: f32) IkJob {
        return aimWithPole(joint, target_ms, forward_axis_ls, up_axis_ls, .{ .x = 0, .y = 1, .z = 0 }, weight);
    }

    pub fn aimWithPole(
        joint: i32,
        target_ms: Vec3,
        forward_axis_ls: Vec3,
        up_axis_ls: Vec3,
        pole_vector_ms: Vec3,
        weight: f32,
    ) IkJob {
        return .{
            .kind = c.OZZ_IK_AIM,
            .weight = weight,
            .start_joint = -1,
            .mid_joint = -1,
            .end_joint = -1,
            .target_ms = .{ .x = 0, .y = 0, .z = 0 },
            .pole_ms = .{ .x = 0, .y = 0, .z = 0 },
            .mid_axis_ls = .{ .x = 0, .y = 0, .z = 1 },
            .twist_angle = 0,
            .soften = 1,
            .aim_joint = joint,
            .aim_target_ms = target_ms,
            .forward_axis_ls = forward_axis_ls,
            .up_axis_ls = up_axis_ls,
            .aim_pole_vector_ms = pole_vector_ms,
        };
    }

    pub fn twoBone(start_joint: i32, mid_joint: i32, end_joint: i32, target_ms: Vec3, pole_ms: Vec3, weight: f32) IkJob {
        return twoBoneAdvanced(
            start_joint,
            mid_joint,
            end_joint,
            target_ms,
            pole_ms,
            .{ .x = 0, .y = 0, .z = 1 },
            0,
            1,
            weight,
        );
    }

    pub fn twoBoneAdvanced(
        start_joint: i32,
        mid_joint: i32,
        end_joint: i32,
        target_ms: Vec3,
        pole_ms: Vec3,
        mid_axis_ls: Vec3,
        twist_angle: f32,
        soften: f32,
        weight: f32,
    ) IkJob {
        return .{
            .kind = c.OZZ_IK_TWO_BONE,
            .weight = weight,
            .start_joint = start_joint,
            .mid_joint = mid_joint,
            .end_joint = end_joint,
            .target_ms = target_ms,
            .pole_ms = pole_ms,
            .mid_axis_ls = mid_axis_ls,
            .twist_angle = twist_angle,
            .soften = soften,
            .aim_joint = -1,
            .aim_target_ms = .{ .x = 0, .y = 0, .z = 0 },
            .forward_axis_ls = .{ .x = 0, .y = 0, .z = 0 },
            .up_axis_ls = .{ .x = 0, .y = 0, .z = 0 },
            .aim_pole_vector_ms = .{ .x = 0, .y = 1, .z = 0 },
        };
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
    joint_weights: ?[]const f32 = null,
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
                .joint_weights = if (L.joint_weights) |weights| weights.ptr else null,
                .joint_weights_count = if (L.joint_weights) |weights| @intCast(weights.len) else 0,
            };
        }

        c.ozz_instance_set_layers(
            self.handle,
            &tmp[0],
            @intCast(layers.len),
        );
    }

    pub fn setIkJobs(self: *Instance, jobs: []const IkJob) void {
        if (jobs.len > c.OZZ_MAX_IK_JOBS) @panic("too many ik jobs (max 8)");

        var tmp: [c.OZZ_MAX_IK_JOBS]c.ozz_ik_job_t = undefined;

        for (jobs, 0..) |job, i| {
            tmp[i] = .{
                .kind = job.kind,
                .weight = job.weight,
                .start_joint = job.start_joint,
                .mid_joint = job.mid_joint,
                .end_joint = job.end_joint,
                .target_ms = .{ .x = job.target_ms.x, .y = job.target_ms.y, .z = job.target_ms.z },
                .pole_ms = .{ .x = job.pole_ms.x, .y = job.pole_ms.y, .z = job.pole_ms.z },
                .mid_axis_ls = .{ .x = job.mid_axis_ls.x, .y = job.mid_axis_ls.y, .z = job.mid_axis_ls.z },
                .twist_angle = job.twist_angle,
                .soften = job.soften,
                .aim_joint = job.aim_joint,
                .aim_target_ms = .{ .x = job.aim_target_ms.x, .y = job.aim_target_ms.y, .z = job.aim_target_ms.z },
                .forward_axis_ls = .{ .x = job.forward_axis_ls.x, .y = job.forward_axis_ls.y, .z = job.forward_axis_ls.z },
                .up_axis_ls = .{ .x = job.up_axis_ls.x, .y = job.up_axis_ls.y, .z = job.up_axis_ls.z },
                .aim_pole_vector_ms = .{ .x = job.aim_pole_vector_ms.x, .y = job.aim_pole_vector_ms.y, .z = job.aim_pole_vector_ms.z },
            };
        }

        c.ozz_instance_set_ik_jobs(self.handle, if (jobs.len == 0) null else &tmp[0], @intCast(jobs.len));
    }
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

pub fn evalModel3x4Reference(inst: *Instance, ws: *Workspace) ![]const f32 {
    try mapResult(c.ozz_eval_model_3x4_reference(inst.handle, ws.handle));
    return ws.palette3x4();
}

fn expectSlicesApproxEqAbs(expected: []const f32, actual: []const f32, tolerance: f32) !void {
    try std.testing.expectEqual(expected.len, actual.len);
    for (expected, actual) |exp, act| {
        try std.testing.expectApproxEqAbs(exp, act, tolerance);
    }
}

fn copyPalette(allocator: std.mem.Allocator, palette: []const f32) ![]f32 {
    const out = try allocator.alloc(f32, palette.len);
    @memcpy(out, palette);
    return out;
}

fn paletteTranslation(palette: []const f32, joint: usize) Vec3 {
    const base = joint * 12;
    return .{
        .x = palette[base + 9],
        .y = palette[base + 10],
        .z = palette[base + 11],
    };
}

fn paletteColumn(palette: []const f32, joint: usize, col: usize) Vec3 {
    const base = joint * 12 + col * 3;
    return .{
        .x = palette[base + 0],
        .y = palette[base + 1],
        .z = palette[base + 2],
    };
}

fn vec3Add(a: Vec3, b: Vec3) Vec3 {
    return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
}

fn paletteDifferenceL1(a: []const f32, b: []const f32) f32 {
    var sum: f32 = 0;
    for (a, b) |av, bv| sum += @abs(av - bv);
    return sum;
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

test "skeleton joint parent queries are structurally sane" {
    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    const joints = skel.numJoints();
    try std.testing.expect(joints > 0);

    var root_count: i32 = 0;
    for (0..@as(usize, @intCast(joints))) |joint_usize| {
        const joint: i32 = @intCast(joint_usize);
        try std.testing.expect(skel.jointName(joint) != null);

        const parent = skel.jointParent(joint);
        if (parent < 0) {
            root_count += 1;
            continue;
        }

        try std.testing.expect(parent < joints);
        try std.testing.expect(skel.jointName(parent) != null);
    }

    try std.testing.expect(root_count >= 1);
}

test "evalModel3x4 matches upstream reference for a single normal clip" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
}

test "evalModel3x4 matches upstream reference for mixed normal layers" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var jog = try Animation.loadFromFileZ("assets/pab_jog_no_motion.ozz");
    defer jog.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var run = try Animation.loadFromFileZ("assets/pab_run_no_motion.ozz");
    defer run.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = jog, .time_seconds = 0.10, .wrap_time = true, .weight = 0.20, .mode = .normal },
        .{ .anim = walk, .time_seconds = 0.25, .wrap_time = true, .weight = 0.35, .mode = .normal },
        .{ .anim = run, .time_seconds = 0.40, .wrap_time = true, .weight = 0.45, .mode = .normal },
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
}

test "evalModel3x4 matches upstream reference for additive hand poses" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var curl = try Animation.loadFromFileZ("assets/pab_curl_additive.ozz");
    defer curl.deinit();

    var splay = try Animation.loadFromFileZ("assets/pab_splay_additive.ozz");
    defer splay.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = curl, .time_seconds = 0.0, .wrap_time = true, .weight = 0.30, .mode = .additive },
        .{ .anim = splay, .time_seconds = 0.0, .wrap_time = true, .weight = 0.90, .mode = .additive },
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
}

test "evalModel3x4 matches upstream reference for negative additive weights" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var curl = try Animation.loadFromFileZ("assets/pab_curl_additive.ozz");
    defer curl.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = curl, .time_seconds = 0.0, .wrap_time = true, .weight = -0.50, .mode = .additive },
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
}

test "partial normal layer masks obey zero and one semantics" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var jog = try Animation.loadFromFileZ("assets/pab_jog_no_motion.ozz");
    defer jog.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_a = try Workspace.init(A, skel);
    defer ws_a.deinit(A);

    var ws_b = try Workspace.init(A, skel);
    defer ws_b.deinit(A);

    const joints: usize = @intCast(skel.numJoints());
    const zero_mask = try A.alloc(f32, joints);
    defer A.free(zero_mask);
    @memset(zero_mask, 0);

    const one_mask = try A.alloc(f32, joints);
    defer A.free(one_mask);
    @memset(one_mask, 1);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });
    const base = try copyPalette(A, try evalModel3x4(&inst, &ws_a));
    defer A.free(base);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = jog, .time_seconds = 0.25, .wrap_time = true, .weight = 0.5, .mode = .normal, .joint_weights = zero_mask },
    });
    const zeroed = try evalModel3x4(&inst, &ws_b);
    try expectSlicesApproxEqAbs(base, zeroed, 1e-4);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = jog, .time_seconds = 0.25, .wrap_time = true, .weight = 0.5, .mode = .normal },
    });
    const unmasked = try copyPalette(A, try evalModel3x4(&inst, &ws_a));
    defer A.free(unmasked);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = jog, .time_seconds = 0.25, .wrap_time = true, .weight = 0.5, .mode = .normal, .joint_weights = one_mask },
    });
    const ones = try evalModel3x4(&inst, &ws_b);
    try expectSlicesApproxEqAbs(unmasked, ones, 1e-4);
}

test "partial additive layer masks obey zero and one semantics" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var curl = try Animation.loadFromFileZ("assets/pab_curl_additive.ozz");
    defer curl.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_a = try Workspace.init(A, skel);
    defer ws_a.deinit(A);

    var ws_b = try Workspace.init(A, skel);
    defer ws_b.deinit(A);

    const joints: usize = @intCast(skel.numJoints());
    const zero_mask = try A.alloc(f32, joints);
    defer A.free(zero_mask);
    @memset(zero_mask, 0);

    const one_mask = try A.alloc(f32, joints);
    defer A.free(one_mask);
    @memset(one_mask, 1);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });
    const base = try copyPalette(A, try evalModel3x4(&inst, &ws_a));
    defer A.free(base);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = curl, .time_seconds = 0.0, .wrap_time = true, .weight = 0.6, .mode = .additive, .joint_weights = zero_mask },
    });
    const zeroed = try evalModel3x4(&inst, &ws_b);
    try expectSlicesApproxEqAbs(base, zeroed, 1e-4);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = curl, .time_seconds = 0.0, .wrap_time = true, .weight = 0.6, .mode = .additive },
    });
    const unmasked = try copyPalette(A, try evalModel3x4(&inst, &ws_a));
    defer A.free(unmasked);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = curl, .time_seconds = 0.0, .wrap_time = true, .weight = 0.6, .mode = .additive, .joint_weights = one_mask },
    });
    const ones = try evalModel3x4(&inst, &ws_b);
    try expectSlicesApproxEqAbs(unmasked, ones, 1e-4);
}

test "partial normal blend with sparse mask matches upstream reference" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var run = try Animation.loadFromFileZ("assets/pab_run_no_motion.ozz");
    defer run.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    const joints: usize = @intCast(skel.numJoints());
    const mask = try A.alloc(f32, joints);
    defer A.free(mask);

    for (mask, 0..) |*weight, j| {
        weight.* = switch (j % 4) {
            0 => 1.0,
            1 => 0.5,
            else => 0.0,
        };
    }

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
        .{ .anim = run, .time_seconds = 0.30, .wrap_time = true, .weight = 0.7, .mode = .normal, .joint_weights = mask },
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
}

test "wrap_time=true matches wrapped sample and wrap_time=false clamps" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_a = try Workspace.init(A, skel);
    defer ws_a.deinit(A);

    var ws_b = try Workspace.init(A, skel);
    defer ws_b.deinit(A);

    const dur = walk.duration();

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });
    const wrapped_base = try copyPalette(A, try evalModel3x4(&inst, &ws_a));
    defer A.free(wrapped_base);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = dur + 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });
    const wrapped_overflow = try evalModel3x4(&inst, &ws_b);
    try expectSlicesApproxEqAbs(wrapped_base, wrapped_overflow, 1e-4);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = dur, .wrap_time = false, .weight = 1.0, .mode = .normal },
    });
    const clamped_end = try copyPalette(A, try evalModel3x4(&inst, &ws_a));
    defer A.free(clamped_end);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = dur + 0.10, .wrap_time = false, .weight = 1.0, .mode = .normal },
    });
    const clamped_overflow = try evalModel3x4(&inst, &ws_b);
    try expectSlicesApproxEqAbs(clamped_end, clamped_overflow, 1e-4);
}

test "aim IK matches upstream reference and changes the pose" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    const head_joint = skel.findJointZ("Head");
    try std.testing.expect(head_joint >= 0);

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_base = try Workspace.init(A, skel);
    defer ws_base.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });
    inst.setIkJobs(&.{});

    const base_palette = try copyPalette(A, try evalModel3x4(&inst, &ws_base));
    defer A.free(base_palette);

    const head_pos = paletteTranslation(base_palette, @intCast(head_joint));
    const aim_target = vec3Add(head_pos, .{ .x = 0.75, .y = 0.25, .z = -0.50 });

    inst.setIkJobs(&[_]IkJob{
        IkJob.aimWithPole(
            head_joint,
            aim_target,
            .{ .x = 1, .y = 0, .z = 0 },
            .{ .x = 0, .y = 1, .z = 0 },
            .{ .x = 0, .y = 1, .z = 0 },
            1.0,
        ),
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
    try std.testing.expect(paletteDifferenceL1(base_palette, actual) > 1e-3);
}

test "two-bone IK matches upstream reference and changes the pose" {
    const A = std.testing.allocator;

    var skel = try Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    defer skel.deinit();

    const hip = skel.findJointZ("LeftUpLeg");
    const knee = skel.findJointZ("LeftLeg");
    const ankle = skel.findJointZ("LeftFoot");
    try std.testing.expect(hip >= 0);
    try std.testing.expect(knee >= 0);
    try std.testing.expect(ankle >= 0);

    var walk = try Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    defer walk.deinit();

    var inst = try Instance.init(A, skel);
    defer inst.deinit(A);

    var ws_base = try Workspace.init(A, skel);
    defer ws_base.deinit(A);

    var ws_actual = try Workspace.init(A, skel);
    defer ws_actual.deinit(A);

    var ws_reference = try Workspace.init(A, skel);
    defer ws_reference.deinit(A);

    inst.setLayers(&[_]Layer{
        .{ .anim = walk, .time_seconds = 0.10, .wrap_time = true, .weight = 1.0, .mode = .normal },
    });
    inst.setIkJobs(&.{});

    const base_palette = try copyPalette(A, try evalModel3x4(&inst, &ws_base));
    defer A.free(base_palette);

    const ankle_pos = paletteTranslation(base_palette, @intCast(ankle));
    const knee_pole = paletteColumn(base_palette, @intCast(knee), 1);
    const target = vec3Add(ankle_pos, .{ .x = 0.15, .y = -0.10, .z = 0.10 });

    inst.setIkJobs(&[_]IkJob{
        IkJob.twoBoneAdvanced(
            hip,
            knee,
            ankle,
            target,
            knee_pole,
            .{ .x = 0, .y = 0, .z = 1 },
            0.0,
            0.8,
            1.0,
        ),
    });

    const actual = try copyPalette(A, try evalModel3x4(&inst, &ws_actual));
    defer A.free(actual);

    const reference = try evalModel3x4Reference(&inst, &ws_reference);
    try expectSlicesApproxEqAbs(reference, actual, 1e-4);
    try std.testing.expect(paletteDifferenceL1(base_palette, actual) > 1e-3);
}
