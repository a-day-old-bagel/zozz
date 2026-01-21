const std = @import("std");
const Alloc = std.mem.Allocator;

pub const c = @cImport({
    @cInclude("cozz_runtime.h");
});

pub const OzzError = error{
    InvalidArgument,
    Io,
    OzzFailure,
    OutOfMemory,
    Unknown,
};

fn mapResult(rc: c.ozz_result_t) OzzError!void {
    switch (rc) {
        c.OZZ_OK => return,
        c.OZZ_ERR_INVALID_ARGUMENT => return OzzError.InvalidArgument,
        c.OZZ_ERR_IO => return OzzError.Io,
        c.OZZ_ERR_OZZ => return OzzError.OzzFailure,
        else => return OzzError.Unknown,
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

    pub fn loadFromFileZ(path_z: [:0]const u8) OzzError!Skeleton {
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

    pub fn soaLocalsBytes(self: Skeleton) usize {
        return c.ozz_soa_locals_bytes(self.handle);
    }
    pub fn soaLocalsAlign() usize {
        return c.ozz_soa_locals_align();
    }

    pub fn modelScratchBytes(self: Skeleton) usize {
        return c.ozz_model_scratch_bytes(self.handle);
    }
    pub fn modelScratchAlign(self: Skeleton) usize {
        _ = self;
        return c.ozz_model_scratch_align();
    }
};

pub const Animation = struct {
    handle: *c.ozz_animation_t,

    pub fn loadFromFileZ(path_z: [:0]const u8) OzzError!Animation {
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

    pub fn normalizeTime(self: Animation, t_seconds: f32, wrap: bool) f32 {
        return c.ozz_normalize_time(self.handle, t_seconds, @intFromBool(wrap));
    }
};

pub const BlendMode = enum(c_int) {
    normal = c.OZZ_BLEND_NORMAL,
    additive = c.OZZ_BLEND_ADDITIVE,
};

pub const EvalLayer = struct {
    anim: Animation,
    time_seconds: f32,
    wrap_time: bool = true,
    weight: f32,
    mode: BlendMode = .normal,

    /// Caller-provided sampled locals buffer (SoA). Must be sized to skel.soaLocalsBytes().
    sampled_locals_soa: []u8,
};

// --------------------
// Per-entity instance
// --------------------

pub const Instance = struct {
    skel: Skeleton,
    inst: *c.ozz_anim_instance_t,

    mem: []align(16) u8,
    output_locals_soa: ?[]u8 = null,
    model_scratch: []u8,
    palette_3x4: []f32,

    /// Hard limit for allocation-free stack layer array.
    pub const MaxLayers: usize = 4;

    /// Create an instance + allocate commonly-needed per-entity buffers.
    /// if include_output_locals, will populate output_locals_soa for later IK or whatever.
    pub fn initOwned(
        allocator: std.mem.Allocator,
        skel: Skeleton,
        include_output_locals: bool,
    ) OzzError!Instance {
        const need = c.ozz_anim_instance_required_bytes(skel.handle, @intFromBool(include_output_locals));
        const mem = allocator.alignedAlloc(u8, 16, need) catch return OzzError.OutOfMemory;
        errdefer allocator.free(mem);

        var out_inst: ?*c.ozz_anim_instance_t = null;
        try mapResult(c.ozz_anim_instance_init(
            mem.ptr,
            mem.len,
            skel.handle,
            @intFromBool(include_output_locals),
            &out_inst,
        ));

        var self: Instance = .{
            .skel = skel,
            .inst = out_inst.?,
            .mem = mem,
            .output_locals_soa = null,
            .model_scratch = undefined,
            .palette_3x4 = undefined,
        };

        if (include_output_locals) {
            var p: ?*anyopaque = null;
            var bytes: usize = 0;
            try mapResult(c.ozz_anim_instance_get_output_locals(self.inst, &p, &bytes));
            self.output_locals_soa = @as([*]u8, @ptrCast(p.?))[0..bytes];
        }

        const ms_bytes = skel.modelScratchBytes();
        const ms_align = skel.modelScratchAlign();
        self.model_scratch = allocator.alignedAlloc(u8, ms_align, ms_bytes) catch return OzzError.OutOfMemory;
        errdefer allocator.free(self.model_scratch);

        const joints: usize = @intCast(skel.numJoints());
        self.palette_3x4 = allocator.alloc(f32, joints * 12) catch return OzzError.OutOfMemory;
        errdefer allocator.free(self.palette_3x4);

        return self;
    }

    pub fn deinitOwned(self: *Instance, allocator: std.mem.Allocator) void {
        allocator.free(self.palette_3x4);
        allocator.free(self.model_scratch);
        allocator.free(self.mem);
        self.* = undefined;
    }

    pub fn numJoints(self: Instance) i32 {
        return self.skel.numJoints();
    }

    pub fn soaLocalsBytes(self: Instance) usize {
        return self.skel.soaLocalsBytes();
    }

    /// Sample one clip into a caller-provided SoA locals buffer.
    pub fn sampleLocalsSoa(
        self: Instance,
        anim: Animation,
        time_seconds: f32,
        wrap: bool,
        out_locals_soa: []u8,
    ) OzzError!void {
        const t = anim.normalizeTime(time_seconds, wrap);
        try mapResult(c.ozz_sample_locals_soa(
            self.inst,
            anim.handle,
            t,
            out_locals_soa.ptr,
            out_locals_soa.len,
        ));
    }

    /// Locals SoA -> model palette 3x4 into out_palette (float[12*joints]).
    pub fn localsToModel3x4(
        self: Instance,
        locals_soa: []const u8,
        model_scratch: []u8,
        out_palette_3x4: []f32,
    ) OzzError!void {
        const need_floats = @as(usize, @intCast(self.numJoints())) * 12;
        if (out_palette_3x4.len < need_floats) return OzzError.InvalidArgument;

        try mapResult(c.ozz_locals_to_model_3x4(
            self.inst,
            locals_soa.ptr,
            locals_soa.len,
            model_scratch.ptr,
            model_scratch.len,
            out_palette_3x4.ptr,
        ));
    }

    /// Helper to sample each layer into its provided sampled_locals_soa,
    /// blend into out_locals_soa, then output model palette 3x4.
    ///
    /// - layers.len must be <= MaxLayers.
    /// - out_locals_soa is usually self.output_locals_soa.
    /// - model_scratch is usually self.model_scratch.
    /// - out_palette_3x4 is usually self.palette_3x4.
    pub fn evalBlendModel3x4(
        self: Instance,
        layers: []const EvalLayer,
        out_locals_soa: []u8,
        model_scratch: []u8,
        out_palette_3x4: []f32,
    ) OzzError!void {
        if (layers.len == 0 or layers.len > MaxLayers) return OzzError.InvalidArgument;

        // Stack array of C layer structs (no heap).
        var tmp: [MaxLayers]c.ozz_eval_layer_t = undefined;

        for (layers, 0..) |L, i| {
            tmp[i] = .{
                .anim = L.anim.handle,
                .time_seconds = L.time_seconds,
                .wrap_time = @intFromBool(L.wrap_time),
                .weight = L.weight,
                .mode = @intCast(@intFromEnum(L.mode)),
                .sampled_locals_soa = L.sampled_locals_soa.ptr,
                .sampled_locals_bytes = L.sampled_locals_soa.len,
            };
        }

        const need_floats = @as(usize, @intCast(self.numJoints())) * 12;
        if (out_palette_3x4.len < need_floats) return OzzError.InvalidArgument;

        try mapResult(c.ozz_eval_blend_model_3x4(
            self.inst,
            &tmp[0],
            @intCast(layers.len),
            out_locals_soa.ptr,
            out_locals_soa.len,
            model_scratch.ptr,
            model_scratch.len,
            out_palette_3x4.ptr,
        ));
    }

    /// “Most common per-entity path” using owned buffers:
    /// - out locals: instance output_locals_soa must exist
    /// - model scratch: self.model_scratch
    /// - palette: self.palette_3x4
    pub fn evalOwnedBlendModel3x4(
        self: Instance,
        layers: []const EvalLayer,
    ) OzzError![]f32 {
        const out_locals = self.output_locals_soa orelse return OzzError.InvalidArgument;
        try self.evalBlendModel3x4(
            layers,
            out_locals,
            self.model_scratch,
            self.palette_3x4,
        );
        return self.palette_3x4;
    }
};

pub fn createLayerScratchBuffers(alloc: Alloc, skel: Skeleton, comptime num_layers: usize) OzzError![num_layers][]u8 {
    const bytes = skel.soaLocalsBytes();
    const alignment = Skeleton.soaLocalsAlign();

    var scratch: [num_layers][]u8 = undefined;

    return scratch;
}

pub const LayerScratch = struct {
    sampled: [Instance.MaxLayers][]u8,

    pub fn initOwned(
        allocator: std.mem.Allocator,
        skel: Skeleton,
    ) OzzError!LayerScratch {
        const bytes = skel.soaLocalsBytes();
        const alignment = Skeleton.soaLocalsAlign();

        const self: LayerScratch = undefined;
        for (self.sampled) |*buf| {
            buf.* = allocator.alignedAlloc(u8, alignment, bytes) catch return OzzError.OutOfMemory;
        }
        return self;
    }

    pub fn deinitOwned(self: *LayerScratch, allocator: std.mem.Allocator) void {
        for (self.sampled) |buf| allocator.free(buf);
        self.* = undefined;
    }
};

// --------------------
// Tests
// --------------------

test "ozz C ABI wrapper: load + 2-clip blend + 3x4 palette is sane" {
    // ---- Configure these paths to your extracted ozz sample assets ----
    // They must be NUL-terminated Zig strings ([:0]const u8).
    const Paths = struct {
        pub const skeleton: [:0]const u8 = "assets/ozz/skeleton.ozz\x00";
        pub const walk: [:0]const u8 = "assets/ozz/walk.ozz\x00";
        pub const jog: [:0]const u8 = "assets/ozz/jog.ozz\x00";
        // Optional:
        // pub const aim_add:  [:0]const u8 = "assets/ozz/aim_additive.ozz\x00";
    };

    var gpa_state = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const leak = gpa_state.deinit();
        if (leak == .leak) @panic("leak detected");
    }
    const gpa = gpa_state.allocator();

    // Load assets
    var skel = try Skeleton.loadFromFileZ(Paths.skeleton);
    defer skel.deinit();

    var walk = try Animation.loadFromFileZ(Paths.walk);
    defer walk.deinit();

    var jog = try Animation.loadFromFileZ(Paths.jog);
    defer jog.deinit();

    // Basic sanity checks
    try std.testing.expect(skel.numJoints() > 0);
    try std.testing.expect(@as(i32, @intFromFloat(walk.duration())) >= 0);
    try std.testing.expect(@as(i32, @intFromFloat(jog.duration())) >= 0);

    // Create per-entity instance (owned buffers)
    var inst = try Instance.initOwned(gpa, skel, true);
    defer inst.deinitOwned(gpa);

    // Per-entity sampled locals buffers (up to 4 layers)
    var scratch = try LayerScratch.initOwned(gpa, skel);
    defer scratch.deinitOwned(gpa);

    // We’ll blend walk<->jog at some speed alpha.
    const t: f32 = 1.2345;
    const alpha: f32 = 0.37;

    // Build layers (2 layers: walk + jog). Both "normal" blending.
    var layers_arr: [2]EvalLayer = .{
        .{
            .anim = walk,
            .time_seconds = t,
            .wrap_time = true,
            .weight = 1.0 - alpha,
            .mode = .normal,
            .sampled_locals_soa = scratch.sampled[0],
        },
        .{
            .anim = jog,
            .time_seconds = t,
            .wrap_time = true,
            .weight = alpha,
            .mode = .normal,
            .sampled_locals_soa = scratch.sampled[1],
        },
    };

    // Run evaluation into instance-owned buffers
    const palette1 = try inst.evalOwnedBlendModel3x4_Stack4(layers_arr[0..]);

    // ---- Validate palette output ----
    // - correct length
    // - all finite
    // - not all zeros
    try std.testing.expect(palette1.len == @as(usize, @intCast(skel.numJoints())) * 12);

    var any_nonzero = false;
    for (palette1) |v| {
        try std.testing.expect(std.math.isFinite(v));
        if (v != 0.0) any_nonzero = true;
    }
    try std.testing.expect(any_nonzero);

    // ---- Determinism check: same inputs twice => identical output ----
    // (This should hold for your wrapper if inputs and buffers are identical.)
    const palette_before = try gpa.dupe(f32, palette1);
    defer gpa.free(palette_before);

    const palette2 = try inst.evalOwnedBlendModel3x4_Stack4(layers_arr[0..]);

    try std.testing.expectEqual(palette_before.len, palette2.len);
    for (palette_before, 0..) |a, i| {
        const b = palette2[i];
        // exact equality is expected here; if it fails due to platform SIMD differences,
        // loosen to approxEqAbs.
        try std.testing.expectEqual(a, b);
    }
}

test "ozz wrapper: invalid layer count fails" {
    var gpa_state = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa_state.deinit();
    const gpa = gpa_state.allocator();

    // You can skip asset loading here; just create a dummy expectation that the function rejects.
    // But Instance needs a real skeleton to exist, so this test is left as a placeholder.
    _ = gpa;
    // Intentionally empty: Once you have assets, you can flesh this out.
}
