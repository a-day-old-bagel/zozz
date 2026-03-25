const std = @import("std");
const math = std.math;
const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;
const zm = @import("zmath");
const zozz = @import("zozz_runtime");

const window_title = "zozz: debug skeleton (wgpu)";
const target_marker_half_extent = 0.08;
const extra_debug_vertices = 38;
const joint_axis_extent = 0.12;

// zig fmt: off
const wgsl_vs =
\\  @group(0) @binding(0) var<uniform> object_to_clip: mat4x4<f32>;
\\  struct VertexOut {
\\      @builtin(position) position_clip: vec4<f32>,
\\      @location(0) color: vec3<f32>,
\\  }
\\  @vertex fn main(
\\      @location(0) position: vec3<f32>,
\\      @location(1) color: vec3<f32>,
\\  ) -> VertexOut {
\\      var output: VertexOut;
\\      output.position_clip = vec4(position, 1.0) * object_to_clip;
\\      output.color = color;
\\      return output;
\\  }
;
const wgsl_fs =
\\  @fragment fn main(
\\      @location(0) color: vec3<f32>,
\\  ) -> @location(0) vec4<f32> {
\\      return vec4(color, 1.0);
\\  }
// zig fmt: on
;

const Vertex = extern struct {
    position: [3]f32,
    color: [3]f32,
};

const AxisChoice = enum(u8) {
    pos_x,
    neg_x,
    pos_y,
    neg_y,
    pos_z,
    neg_z,
};

const Controls = struct {
    auto_blend: bool = false,
    blend: f32 = 0.0,
    enable_additive: bool = false,
    enable_head_aim: bool = true,
    enable_arm_ik: bool = true,
    orbit_camera: bool = true,
    show_target: bool = true,
    show_joint_axes: bool = true,
    arm_weight: f32 = 0.95,
    arm_soften: f32 = 0.50,
    head_weight: f32 = 0.75,
    target_side: f32 = 0.08,
    target_lift: f32 = 0.15,
    target_forward: f32 = 0.15,
    target_sway: f32 = 0.10,
    target_bob: f32 = 0.08,
    elbow_axis: AxisChoice = .pos_z,
    arm_pole_axis: AxisChoice = .neg_z,
    head_forward_axis: AxisChoice = .pos_y,
    head_up_axis: AxisChoice = .pos_x,
};

const InputState = struct {
    toggle_auto_blend: bool = false,
    toggle_additive: bool = false,
    toggle_arm_ik: bool = false,
    toggle_head_aim: bool = false,
    toggle_target: bool = false,
    toggle_orbit_camera: bool = false,
    toggle_joint_axes: bool = false,
    cycle_elbow_axis: bool = false,
    cycle_arm_pole_axis: bool = false,
    cycle_head_forward_axis: bool = false,
    cycle_head_up_axis: bool = false,
    blend_down: bool = false,
    blend_up: bool = false,
    arm_weight_down: bool = false,
    arm_weight_up: bool = false,
    arm_soften_down: bool = false,
    arm_soften_up: bool = false,
    head_weight_down: bool = false,
    head_weight_up: bool = false,
    target_forward_down: bool = false,
    target_forward_up: bool = false,
    target_lift_down: bool = false,
    target_lift_up: bool = false,
    reset_controls: bool = false,
    print_help: bool = false,
};

const ArmChain = struct {
    side_label: [:0]const u8,
    start_joint: i32,
    mid_joint: i32,
    end_joint: i32,

    fn isValid(self: ArmChain) bool {
        return self.start_joint >= 0 and self.mid_joint >= 0 and self.end_joint >= 0;
    }
};

const DemoState = struct {
    gctx: *zgpu.GraphicsContext,
    pipeline: zgpu.RenderPipelineHandle,
    bind_group: zgpu.BindGroupHandle,
    vertex_buffer: zgpu.BufferHandle,
    depth_texture: zgpu.TextureHandle,
    depth_texture_view: zgpu.TextureViewHandle,

    skeleton: zozz.Skeleton,
    walk: zozz.Animation,
    run: zozz.Animation,
    curl: zozz.Animation,
    splay: zozz.Animation,
    inst: zozz.Instance,
    ws: zozz.Workspace,

    joint_parents: []i32,
    line_vertices: []Vertex,

    head_joint: i32,
    arm_chain: ArmChain,
    controls: Controls,
    input: InputState,
    current_target_ms: [3]f32,
    current_hand_distance: f32,
};

fn init(allocator: std.mem.Allocator, window: *zglfw.Window) !DemoState {
    const gctx = try zgpu.GraphicsContext.create(
        allocator,
        .{
            .window = window,
            .fn_getTime = @ptrCast(&zglfw.getTime),
            .fn_getFramebufferSize = @ptrCast(&zglfw.Window.getFramebufferSize),
            .fn_getWin32Window = @ptrCast(&zglfw.getWin32Window),
            .fn_getX11Display = @ptrCast(&zglfw.getX11Display),
            .fn_getX11Window = @ptrCast(&zglfw.getX11Window),
            .fn_getWaylandDisplay = @ptrCast(&zglfw.getWaylandDisplay),
            .fn_getWaylandSurface = @ptrCast(&zglfw.getWaylandWindow),
            .fn_getCocoaWindow = @ptrCast(&zglfw.getCocoaWindow),
        },
        .{},
    );
    errdefer gctx.destroy(allocator);

    const bind_group_layout = gctx.createBindGroupLayout(&.{
        zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
    });
    defer gctx.releaseResource(bind_group_layout);

    const pipeline_layout = gctx.createPipelineLayout(&.{bind_group_layout});
    defer gctx.releaseResource(pipeline_layout);

    const pipeline = pipeline: {
        const vs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "vs");
        defer vs_module.release();

        const fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs");
        defer fs_module.release();

        const color_targets = [_]wgpu.ColorTargetState{.{
            .format = zgpu.GraphicsContext.swapchain_format,
        }};

        const vertex_attributes = [_]wgpu.VertexAttribute{
            .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
            .{ .format = .float32x3, .offset = @offsetOf(Vertex, "color"), .shader_location = 1 },
        };
        const vertex_buffers = [_]wgpu.VertexBufferLayout{.{
            .array_stride = @sizeOf(Vertex),
            .attribute_count = vertex_attributes.len,
            .attributes = &vertex_attributes,
        }};

        const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = vs_module,
                .entry_point = "main",
                .buffer_count = vertex_buffers.len,
                .buffers = &vertex_buffers,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .line_list,
            },
            .depth_stencil = &wgpu.DepthStencilState{
                .format = .depth32_float,
                .depth_write_enabled = true,
                .depth_compare = .less,
            },
            .fragment = &wgpu.FragmentState{
                .module = fs_module,
                .entry_point = "main",
                .target_count = color_targets.len,
                .targets = &color_targets,
            },
        };
        break :pipeline gctx.createRenderPipeline(pipeline_layout, pipeline_descriptor);
    };

    const bind_group = gctx.createBindGroup(bind_group_layout, &.{
        .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(zm.Mat) },
    });

    var skeleton = try zozz.Skeleton.loadFromFileZ("assets/pab_skeleton.ozz");
    errdefer skeleton.deinit();

    var walk = try zozz.Animation.loadFromFileZ("assets/pab_walk_no_motion.ozz");
    errdefer walk.deinit();

    var run = try zozz.Animation.loadFromFileZ("assets/pab_run_no_motion.ozz");
    errdefer run.deinit();

    var curl = try zozz.Animation.loadFromFileZ("assets/pab_curl_additive.ozz");
    errdefer curl.deinit();

    var splay = try zozz.Animation.loadFromFileZ("assets/pab_splay_additive.ozz");
    errdefer splay.deinit();

    var inst = try zozz.Instance.init(allocator, skeleton);
    errdefer inst.deinit(allocator);

    var ws = try zozz.Workspace.init(allocator, skeleton);
    errdefer ws.deinit(allocator);

    const joint_count = @as(usize, @intCast(skeleton.numJoints()));
    const joint_parents = try allocator.alloc(i32, joint_count);
    errdefer allocator.free(joint_parents);
    for (joint_parents, 0..) |*parent, joint| {
        parent.* = skeleton.jointParent(@intCast(joint));
    }

    const bone_vertices = if (joint_count > 0) (joint_count - 1) * 2 else 0;
    const line_vertices = try allocator.alloc(Vertex, bone_vertices + extra_debug_vertices);
    errdefer allocator.free(line_vertices);

    const vertex_buffer = gctx.createBuffer(.{
        .usage = .{ .copy_dst = true, .vertex = true },
        .size = @sizeOf(Vertex) * line_vertices.len,
    });

    const depth = createDepthTexture(gctx);

    return .{
        .gctx = gctx,
        .pipeline = pipeline,
        .bind_group = bind_group,
        .vertex_buffer = vertex_buffer,
        .depth_texture = depth.texture,
        .depth_texture_view = depth.view,
        .skeleton = skeleton,
        .walk = walk,
        .run = run,
        .curl = curl,
        .splay = splay,
        .inst = inst,
        .ws = ws,
        .joint_parents = joint_parents,
        .line_vertices = line_vertices,
        .head_joint = skeleton.findJointZ("Head"),
        .arm_chain = resolveArmChain(skeleton),
        .controls = .{},
        .input = .{},
        .current_target_ms = .{ 0.0, 1.2, -0.6 },
        .current_hand_distance = 0.0,
    };
}

fn deinit(allocator: std.mem.Allocator, demo: *DemoState) void {
    allocator.free(demo.line_vertices);
    allocator.free(demo.joint_parents);
    demo.ws.deinit(allocator);
    demo.inst.deinit(allocator);
    demo.splay.deinit();
    demo.curl.deinit();
    demo.run.deinit();
    demo.walk.deinit();
    demo.skeleton.deinit();
    demo.gctx.destroy(allocator);
    demo.* = undefined;
}

fn draw(demo: *DemoState) !void {
    const gctx = demo.gctx;
    const fb_width = gctx.swapchain_descriptor.width;
    const fb_height = gctx.swapchain_descriptor.height;
    const t: f32 = @floatCast(gctx.stats.time);

    const vertex_count = try updateSkeletonVertices(demo, t);
    if (vertex_count > 0) {
        const vertex_buffer = gctx.lookupResource(demo.vertex_buffer) orelse return;
        gctx.queue.writeBuffer(vertex_buffer, 0, Vertex, demo.line_vertices[0..vertex_count]);
    }

    const orbit = if (demo.controls.orbit_camera) 0.25 * t else 0.0;
    const eye = zm.f32x4(4.2 * @sin(orbit), 1.6, -4.2 * @cos(orbit), 1.0);
    const target = zm.f32x4(0.0, 1.0, 0.0, 1.0);
    const up = zm.f32x4(0.0, 1.0, 0.0, 0.0);
    const world_to_view = zm.lookAtLh(eye, target, up);
    const view_to_clip = zm.perspectiveFovLh(
        0.25 * math.pi,
        @as(f32, @floatFromInt(fb_width)) / @as(f32, @floatFromInt(fb_height)),
        0.01,
        200.0,
    );
    const object_to_clip = zm.mul(world_to_view, view_to_clip);

    const back_buffer_view = gctx.swapchain.getCurrentTextureView();
    defer back_buffer_view.release();

    const commands = commands: {
        const encoder = gctx.device.createCommandEncoder(null);
        defer encoder.release();

        pass: {
            const vb_info = gctx.lookupResourceInfo(demo.vertex_buffer) orelse break :pass;
            const pipeline = gctx.lookupResource(demo.pipeline) orelse break :pass;
            const bind_group = gctx.lookupResource(demo.bind_group) orelse break :pass;
            const depth_view = gctx.lookupResource(demo.depth_texture_view) orelse break :pass;

            const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                .view = back_buffer_view,
                .load_op = .clear,
                .store_op = .store,
                .clear_value = .{ .r = 0.08, .g = 0.09, .b = 0.12, .a = 1.0 },
            }};
            const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                .view = depth_view,
                .depth_load_op = .clear,
                .depth_store_op = .store,
                .depth_clear_value = 1.0,
            };
            const render_pass_info = wgpu.RenderPassDescriptor{
                .color_attachment_count = color_attachments.len,
                .color_attachments = &color_attachments,
                .depth_stencil_attachment = &depth_attachment,
            };
            const pass = encoder.beginRenderPass(render_pass_info);
            defer {
                pass.end();
                pass.release();
            }

            pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
            pass.setPipeline(pipeline);

            const mem = gctx.uniformsAllocate(zm.Mat, 1);
            mem.slice[0] = zm.transpose(object_to_clip);

            pass.setBindGroup(0, bind_group, &.{mem.offset});
            pass.draw(@intCast(vertex_count), 1, 0, 0);
        }

        break :commands encoder.finish(null);
    };
    defer commands.release();

    gctx.submit(&.{commands});

    if (gctx.present() == .swap_chain_resized) {
        gctx.releaseResource(demo.depth_texture_view);
        gctx.destroyResource(demo.depth_texture);

        const depth = createDepthTexture(gctx);
        demo.depth_texture = depth.texture;
        demo.depth_texture_view = depth.view;
    }
}

fn updateSkeletonVertices(demo: *DemoState, time_seconds: f32) !usize {
    const blend = if (demo.controls.auto_blend)
        0.5 + 0.5 * @sin(time_seconds * 0.35)
    else
        demo.controls.blend;
    const curl_weight = if (demo.controls.enable_additive)
        0.15 + 0.15 * (0.5 + 0.5 * @sin(time_seconds * 1.1))
    else
        0.0;
    const splay_weight = if (demo.controls.enable_additive)
        0.10 + 0.20 * (0.5 + 0.5 * @cos(time_seconds * 0.9))
    else
        0.0;

    demo.inst.setLayers(&[_]zozz.Layer{
        .{ .anim = demo.walk, .time_seconds = time_seconds, .wrap_time = true, .weight = 1.0 - blend, .mode = .normal },
        .{ .anim = demo.run, .time_seconds = time_seconds * 1.15, .wrap_time = true, .weight = blend, .mode = .normal },
        .{ .anim = demo.curl, .time_seconds = 0.0, .wrap_time = true, .weight = curl_weight, .mode = .additive },
        .{ .anim = demo.splay, .time_seconds = 0.0, .wrap_time = true, .weight = splay_weight, .mode = .additive },
    });
    demo.inst.setIkJobs(&.{});

    const base_palette = try zozz.evalModel3x4(&demo.inst, &demo.ws);

    const hand_joint = if (demo.arm_chain.isValid()) @as(usize, @intCast(demo.arm_chain.end_joint)) else @as(usize, 0);
    const hand_pos = if (demo.arm_chain.isValid())
        paletteTranslation(base_palette, hand_joint)
    else
        [3]f32{ 0.35, 1.15, -0.10 };
    const side_sign: f32 = if (hand_pos[0] >= 0.0) 1.0 else -1.0;
    const target_ms = [3]f32{
        hand_pos[0] + side_sign * demo.controls.target_side + demo.controls.target_sway * @sin(time_seconds * 0.9),
        hand_pos[1] + demo.controls.target_lift + demo.controls.target_bob * @sin(time_seconds * 1.2),
        hand_pos[2] + demo.controls.target_forward + demo.controls.target_sway * @cos(time_seconds * 0.9),
    };
    demo.current_target_ms = target_ms;

    var jobs: [2]zozz.IkJob = undefined;
    var job_count: usize = 0;

    if (demo.controls.enable_head_aim and demo.head_joint >= 0) {
        jobs[job_count] = zozz.IkJob.aimWithPole(
            demo.head_joint,
            toVec3(target_ms),
            axisChoiceVec3(demo.controls.head_forward_axis),
            axisChoiceVec3(demo.controls.head_up_axis),
            .{ .x = 0.0, .y = 1.0, .z = 0.0 },
            demo.controls.head_weight,
        );
        job_count += 1;
    }

    if (demo.controls.enable_arm_ik and demo.arm_chain.isValid()) {
        jobs[job_count] = zozz.IkJob.twoBoneAdvanced(
            demo.arm_chain.start_joint,
            demo.arm_chain.mid_joint,
            demo.arm_chain.end_joint,
            toVec3(target_ms),
            axisChoiceVec3(demo.controls.arm_pole_axis),
            axisChoiceVec3(demo.controls.elbow_axis),
            0.0,
            demo.controls.arm_soften,
            demo.controls.arm_weight,
        );
        job_count += 1;
    }

    demo.inst.setIkJobs(jobs[0..job_count]);
    const palette = try zozz.evalModel3x4(&demo.inst, &demo.ws);

    var vertex_count: usize = 0;
    for (demo.joint_parents, 0..) |parent, joint| {
        if (parent < 0) continue;

        const parent_pos = paletteTranslation(palette, @intCast(parent));
        const joint_pos = paletteTranslation(palette, joint);
        const color = lineColor(joint, demo.joint_parents.len);
        appendLine(demo.line_vertices, &vertex_count, parent_pos, joint_pos, color);
    }

    if (demo.controls.show_target) {
        appendTargetMarker(demo.line_vertices, &vertex_count, target_ms);
        if (demo.arm_chain.isValid()) {
            const hand_after_ik = paletteTranslation(palette, @intCast(demo.arm_chain.end_joint));
            demo.current_hand_distance = length3(sub3(target_ms, hand_after_ik));
            appendLine(demo.line_vertices, &vertex_count, hand_after_ik, target_ms, .{ 1.0, 0.72, 0.24 });
        } else {
            demo.current_hand_distance = 0.0;
        }
    } else if (demo.arm_chain.isValid()) {
        const hand_after_ik = paletteTranslation(palette, @intCast(demo.arm_chain.end_joint));
        demo.current_hand_distance = length3(sub3(target_ms, hand_after_ik));
    } else {
        demo.current_hand_distance = 0.0;
    }

    if (demo.controls.show_joint_axes) {
        if (demo.head_joint >= 0) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.head_joint),
                joint_axis_extent,
            );
        }
        if (demo.arm_chain.isValid()) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.arm_chain.start_joint),
                joint_axis_extent,
            );
        }
        if (demo.arm_chain.isValid()) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.arm_chain.mid_joint),
                joint_axis_extent,
            );
        }
        if (demo.arm_chain.isValid()) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.arm_chain.end_joint),
                joint_axis_extent,
            );
        }
    }

    appendAxesOrigin(demo.line_vertices, &vertex_count);

    return vertex_count;
}

fn resolveArmChain(skeleton: zozz.Skeleton) ArmChain {
    const left = ArmChain{
        .side_label = "Left",
        .start_joint = skeleton.findJointZ("LeftArm"),
        .mid_joint = skeleton.findJointZ("LeftForeArm"),
        .end_joint = skeleton.findJointZ("LeftHand"),
    };
    if (left.isValid()) return left;

    const right = ArmChain{
        .side_label = "Right",
        .start_joint = skeleton.findJointZ("RightArm"),
        .mid_joint = skeleton.findJointZ("RightForeArm"),
        .end_joint = skeleton.findJointZ("RightHand"),
    };
    if (right.isValid()) return right;

    return .{
        .side_label = "Missing",
        .start_joint = -1,
        .mid_joint = -1,
        .end_joint = -1,
    };
}

fn axisChoiceVec3(choice: AxisChoice) zozz.Vec3 {
    return switch (choice) {
        .pos_x => .{ .x = 1.0, .y = 0.0, .z = 0.0 },
        .neg_x => .{ .x = -1.0, .y = 0.0, .z = 0.0 },
        .pos_y => .{ .x = 0.0, .y = 1.0, .z = 0.0 },
        .neg_y => .{ .x = 0.0, .y = -1.0, .z = 0.0 },
        .pos_z => .{ .x = 0.0, .y = 0.0, .z = 1.0 },
        .neg_z => .{ .x = 0.0, .y = 0.0, .z = -1.0 },
    };
}

fn axisChoiceLabel(choice: AxisChoice) [:0]const u8 {
    return switch (choice) {
        .pos_x => "+X",
        .neg_x => "-X",
        .pos_y => "+Y",
        .neg_y => "-Y",
        .pos_z => "+Z",
        .neg_z => "-Z",
    };
}

fn nextAxisChoice(choice: AxisChoice) AxisChoice {
    return switch (choice) {
        .pos_x => .neg_x,
        .neg_x => .pos_y,
        .pos_y => .neg_y,
        .neg_y => .pos_z,
        .pos_z => .neg_z,
        .neg_z => .pos_x,
    };
}

fn keyDown(window: *zglfw.Window, key: zglfw.Key) bool {
    return switch (window.getKey(key)) {
        .press, .repeat => true,
        else => false,
    };
}

fn consumeEdge(window: *zglfw.Window, key: zglfw.Key, previous: *bool) bool {
    const down = keyDown(window, key);
    defer previous.* = down;
    return down and !previous.*;
}

fn stepClamped(value: *f32, delta: f32, min_value: f32, max_value: f32) bool {
    const next = std.math.clamp(value.* + delta, min_value, max_value);
    if (next == value.*) return false;
    value.* = next;
    return true;
}

fn onOff(value: bool) []const u8 {
    return if (value) "on" else "off";
}

fn printControls() void {
    std.debug.print(
        \\Controls:
        \\  1 auto blend   2 additive   3 arm IK   4 head aim   5 target marker   6 orbit camera   7 joint axes
        \\  Q elbow axis   Y arm pole axis   W head forward axis   E head up axis
        \\  A/D blend      S/F arm weight        G/H arm soften
        \\  X/C head weight
        \\  R/T target forward     V/B target lift
        \\  Space reset controls   / print this help   Esc quit
        \\
    , .{});
}

fn printResolvedJoints(demo: *const DemoState) void {
    std.debug.print("Resolved head joint: {d}\n", .{demo.head_joint});
    if (demo.arm_chain.isValid()) {
        std.debug.print(
            "Resolved {s} arm chain: {d} -> {d} -> {d}\n",
            .{ demo.arm_chain.side_label, demo.arm_chain.start_joint, demo.arm_chain.mid_joint, demo.arm_chain.end_joint },
        );
    } else {
        std.debug.print("Arm chain could not be resolved from skeleton joint names.\n", .{});
    }
}

fn printControlState(demo: *const DemoState) void {
    std.debug.print(
        "State: blend={d:.2} auto={s} add={s} arm={s} head={s} target={s} axes={s} orbit={s} arm_weight={d:.2} arm_soften={d:.2} hand_dist={d:.3} head_weight={d:.2} target_forward={d:.2} target_lift={d:.2} elbow={s} arm_pole={s} head_forward={s} head_up={s}\n",
        .{
            demo.controls.blend,
            onOff(demo.controls.auto_blend),
            onOff(demo.controls.enable_additive),
            onOff(demo.controls.enable_arm_ik),
            onOff(demo.controls.enable_head_aim),
            onOff(demo.controls.show_target),
            onOff(demo.controls.show_joint_axes),
            onOff(demo.controls.orbit_camera),
            demo.controls.arm_weight,
            demo.controls.arm_soften,
            demo.current_hand_distance,
            demo.controls.head_weight,
            demo.controls.target_forward,
            demo.controls.target_lift,
            axisChoiceLabel(demo.controls.elbow_axis),
            axisChoiceLabel(demo.controls.arm_pole_axis),
            axisChoiceLabel(demo.controls.head_forward_axis),
            axisChoiceLabel(demo.controls.head_up_axis),
        },
    );
}

fn handleInput(window: *zglfw.Window, demo: *DemoState) void {
    if (consumeEdge(window, .slash, &demo.input.print_help)) {
        printControls();
    }

    var changed = false;

    if (consumeEdge(window, .space, &demo.input.reset_controls)) {
        demo.controls = .{};
        changed = true;
    }
    if (consumeEdge(window, .one, &demo.input.toggle_auto_blend)) {
        demo.controls.auto_blend = !demo.controls.auto_blend;
        changed = true;
    }
    if (consumeEdge(window, .two, &demo.input.toggle_additive)) {
        demo.controls.enable_additive = !demo.controls.enable_additive;
        changed = true;
    }
    if (consumeEdge(window, .three, &demo.input.toggle_arm_ik)) {
        demo.controls.enable_arm_ik = !demo.controls.enable_arm_ik;
        changed = true;
    }
    if (consumeEdge(window, .four, &demo.input.toggle_head_aim)) {
        demo.controls.enable_head_aim = !demo.controls.enable_head_aim;
        changed = true;
    }
    if (consumeEdge(window, .five, &demo.input.toggle_target)) {
        demo.controls.show_target = !demo.controls.show_target;
        changed = true;
    }
    if (consumeEdge(window, .six, &demo.input.toggle_orbit_camera)) {
        demo.controls.orbit_camera = !demo.controls.orbit_camera;
        changed = true;
    }
    if (consumeEdge(window, .seven, &demo.input.toggle_joint_axes)) {
        demo.controls.show_joint_axes = !demo.controls.show_joint_axes;
        changed = true;
    }
    if (consumeEdge(window, .q, &demo.input.cycle_elbow_axis)) {
        demo.controls.elbow_axis = nextAxisChoice(demo.controls.elbow_axis);
        changed = true;
    }
    if (consumeEdge(window, .y, &demo.input.cycle_arm_pole_axis)) {
        demo.controls.arm_pole_axis = nextAxisChoice(demo.controls.arm_pole_axis);
        changed = true;
    }
    if (consumeEdge(window, .w, &demo.input.cycle_head_forward_axis)) {
        demo.controls.head_forward_axis = nextAxisChoice(demo.controls.head_forward_axis);
        changed = true;
    }
    if (consumeEdge(window, .e, &demo.input.cycle_head_up_axis)) {
        demo.controls.head_up_axis = nextAxisChoice(demo.controls.head_up_axis);
        changed = true;
    }
    if (consumeEdge(window, .a, &demo.input.blend_down)) {
        changed = stepClamped(&demo.controls.blend, -0.1, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .d, &demo.input.blend_up)) {
        changed = stepClamped(&demo.controls.blend, 0.1, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .s, &demo.input.arm_weight_down)) {
        changed = stepClamped(&demo.controls.arm_weight, -0.05, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .f, &demo.input.arm_weight_up)) {
        changed = stepClamped(&demo.controls.arm_weight, 0.05, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .g, &demo.input.arm_soften_down)) {
        changed = stepClamped(&demo.controls.arm_soften, -0.05, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .h, &demo.input.arm_soften_up)) {
        changed = stepClamped(&demo.controls.arm_soften, 0.05, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .x, &demo.input.head_weight_down)) {
        changed = stepClamped(&demo.controls.head_weight, -0.05, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .c, &demo.input.head_weight_up)) {
        changed = stepClamped(&demo.controls.head_weight, 0.05, 0.0, 1.0) or changed;
    }
    if (consumeEdge(window, .r, &demo.input.target_forward_down)) {
        changed = stepClamped(&demo.controls.target_forward, -0.05, -1.0, 0.5) or changed;
    }
    if (consumeEdge(window, .t, &demo.input.target_forward_up)) {
        changed = stepClamped(&demo.controls.target_forward, 0.05, -1.0, 0.5) or changed;
    }
    if (consumeEdge(window, .v, &demo.input.target_lift_down)) {
        changed = stepClamped(&demo.controls.target_lift, -0.05, -0.5, 0.5) or changed;
    }
    if (consumeEdge(window, .b, &demo.input.target_lift_up)) {
        changed = stepClamped(&demo.controls.target_lift, 0.05, -0.5, 0.5) or changed;
    }

    if (changed) {
        printControlState(demo);
    }
}

fn updateWindowTitle(window: *zglfw.Window, demo: *const DemoState) void {
    var buffer: [256]u8 = undefined;
    const title = std.fmt.bufPrintZ(
        &buffer,
        "{s} | blend {d:.2} | arm {s} {d:.2} s{d:.2} d{d:.3} | pole {s} | head {s} {d:.2} | add {s} | axes {s} | elbow {s}",
        .{
            window_title,
            demo.controls.blend,
            onOff(demo.controls.enable_arm_ik),
            demo.controls.arm_weight,
            demo.controls.arm_soften,
            demo.current_hand_distance,
            axisChoiceLabel(demo.controls.arm_pole_axis),
            onOff(demo.controls.enable_head_aim),
            demo.controls.head_weight,
            onOff(demo.controls.enable_additive),
            onOff(demo.controls.show_joint_axes),
            axisChoiceLabel(demo.controls.elbow_axis),
        },
    ) catch return;
    window.setTitle(title);
}

fn paletteTranslation(palette: []const f32, joint: usize) [3]f32 {
    const base = joint * 12;
    return .{
        palette[base + 9],
        palette[base + 10],
        palette[base + 11],
    };
}

fn paletteAxis(palette: []const f32, joint: usize, axis: usize) [3]f32 {
    const base = joint * 12 + axis * 3;
    return .{
        palette[base + 0],
        palette[base + 1],
        palette[base + 2],
    };
}

fn lineColor(joint: usize, joint_count: usize) [3]f32 {
    const denom = if (joint_count > 1) joint_count - 1 else 1;
    const t = @as(f32, @floatFromInt(joint)) / @as(f32, @floatFromInt(denom));
    return .{
        0.25 + 0.50 * t,
        0.80 - 0.30 * t,
        0.95 - 0.20 * t,
    };
}

fn appendTargetMarker(vertices: []Vertex, vertex_count: *usize, target_ms: [3]f32) void {
    appendLine(
        vertices,
        vertex_count,
        .{ target_ms[0] - target_marker_half_extent, target_ms[1], target_ms[2] },
        .{ target_ms[0] + target_marker_half_extent, target_ms[1], target_ms[2] },
        .{ 1.0, 0.4, 0.2 },
    );
    appendLine(
        vertices,
        vertex_count,
        .{ target_ms[0], target_ms[1] - target_marker_half_extent, target_ms[2] },
        .{ target_ms[0], target_ms[1] + target_marker_half_extent, target_ms[2] },
        .{ 1.0, 0.4, 0.2 },
    );
    appendLine(
        vertices,
        vertex_count,
        .{ target_ms[0], target_ms[1], target_ms[2] - target_marker_half_extent },
        .{ target_ms[0], target_ms[1], target_ms[2] + target_marker_half_extent },
        .{ 1.0, 0.4, 0.2 },
    );
}

fn appendAxesOrigin(vertices: []Vertex, vertex_count: *usize) void {
    appendLine(vertices, vertex_count, .{ 0, 0, 0 }, .{ 0.5, 0, 0 }, .{ 1, 0, 0 });
    appendLine(vertices, vertex_count, .{ 0, 0, 0 }, .{ 0, 0.5, 0 }, .{ 0, 1, 0 });
    appendLine(vertices, vertex_count, .{ 0, 0, 0 }, .{ 0, 0, 0.5 }, .{ 0, 0, 1 });
}

fn appendJointBasis(
    vertices: []Vertex,
    vertex_count: *usize,
    palette: []const f32,
    joint: usize,
    axis_extent: f32,
) void {
    const origin = paletteTranslation(palette, joint);
    const x_axis = paletteAxis(palette, joint, 0);
    const y_axis = paletteAxis(palette, joint, 1);
    const z_axis = paletteAxis(palette, joint, 2);

    appendLine(vertices, vertex_count, origin, addScaled3(origin, x_axis, axis_extent), .{ 1.0, 0.25, 0.25 });
    appendLine(vertices, vertex_count, origin, addScaled3(origin, y_axis, axis_extent), .{ 0.25, 1.0, 0.25 });
    appendLine(vertices, vertex_count, origin, addScaled3(origin, z_axis, axis_extent), .{ 0.25, 0.55, 1.0 });
}

fn appendLine(vertices: []Vertex, vertex_count: *usize, a: [3]f32, b: [3]f32, color: [3]f32) void {
    vertices[vertex_count.* + 0] = .{ .position = a, .color = color };
    vertices[vertex_count.* + 1] = .{ .position = b, .color = color };
    vertex_count.* += 2;
}

fn sub3(a: [3]f32, b: [3]f32) [3]f32 {
    return .{
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
    };
}

fn addScaled3(a: [3]f32, b: [3]f32, scale: f32) [3]f32 {
    return .{
        a[0] + b[0] * scale,
        a[1] + b[1] * scale,
        a[2] + b[2] * scale,
    };
}

fn normalized3(v: [3]f32, fallback: [3]f32) [3]f32 {
    const len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (len_sq <= 1e-8) return fallback;
    const inv_len = 1.0 / @sqrt(len_sq);
    return .{ v[0] * inv_len, v[1] * inv_len, v[2] * inv_len };
}

fn length3(v: [3]f32) f32 {
    return @sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

fn toVec3(v: [3]f32) zozz.Vec3 {
    return .{ .x = v[0], .y = v[1], .z = v[2] };
}

fn createDepthTexture(gctx: *zgpu.GraphicsContext) struct {
    texture: zgpu.TextureHandle,
    view: zgpu.TextureViewHandle,
} {
    const texture = gctx.createTexture(.{
        .usage = .{ .render_attachment = true },
        .dimension = .tdim_2d,
        .size = .{
            .width = gctx.swapchain_descriptor.width,
            .height = gctx.swapchain_descriptor.height,
            .depth_or_array_layers = 1,
        },
        .format = .depth32_float,
        .mip_level_count = 1,
        .sample_count = 1,
    });
    const view = gctx.createTextureView(texture, .{});
    return .{ .texture = texture, .view = view };
}

pub fn main() !void {
    try zglfw.init();
    defer zglfw.terminate();

    {
        var buffer: [1024]u8 = undefined;
        const path = std.fs.selfExeDirPath(buffer[0..]) catch ".";
        std.posix.chdir(path) catch {};
    }

    zglfw.windowHint(.client_api, .no_api);

    const window = try zglfw.createWindow(1400, 900, window_title, null);
    defer window.destroy();
    window.setSizeLimits(640, 480, -1, -1);

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const allocator = gpa.allocator();

    var demo = try init(allocator, window);
    defer deinit(allocator, &demo);
    printControls();
    printResolvedJoints(&demo);
    printControlState(&demo);

    while (!window.shouldClose() and window.getKey(.escape) != .press) {
        zglfw.pollEvents();
        handleInput(window, &demo);
        updateWindowTitle(window, &demo);
        try draw(&demo);
    }
}
