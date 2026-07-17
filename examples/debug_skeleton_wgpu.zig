const std = @import("std");
const math = std.math;
const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;
const zm = @import("zmath");
const zozz = @import("zozz_runtime");
const zozz_mesh = @import("zozz_mesh");

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
    auto_blend: bool = true,
    blend: f32 = 0.0,
    enable_additive: bool = false,
    enable_head_aim: bool = false,
    enable_arm_ik: bool = false,
    orbit_camera: bool = true,
    show_target: bool = true,
    show_joint_axes: bool = true,
    show_mesh: bool = true,
    arm_weight: f32 = 0.95,
    arm_soften: f32 = 0.60,
    arm_twist: f32 = 0.0,
    head_weight: f32 = 0.75,
    target_side: f32 = 0.08,
    target_lift: f32 = 0.25,
    target_forward: f32 = 0.25,
    target_sway: f32 = 0.10,
    target_bob: f32 = 0.08,
    // Derived from the UAL rest pose: the elbow's local +X maps to the
    // model-space bend-plane normal, the arm bends toward -Z, and the head's
    // local +Z/+Y axes are forward/up respectively.
    elbow_axis: AxisChoice = .pos_x,
    arm_pole_axis: AxisChoice = .neg_z,
    head_forward_axis: AxisChoice = .pos_z,
    head_up_axis: AxisChoice = .pos_y,
};

const InputState = struct {
    toggle_auto_blend: bool = false,
    toggle_additive: bool = false,
    toggle_arm_ik: bool = false,
    toggle_head_aim: bool = false,
    toggle_target: bool = false,
    toggle_orbit_camera: bool = false,
    toggle_joint_axes: bool = false,
    toggle_mesh: bool = false,
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
    arm_twist_down: bool = false,
    arm_twist_up: bool = false,
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
    mesh_pipeline: zgpu.RenderPipelineHandle,
    bind_group: zgpu.BindGroupHandle,
    vertex_buffer: zgpu.BufferHandle,
    mesh_vertex_buffer: zgpu.BufferHandle,
    mesh_index_buffer: zgpu.BufferHandle,
    depth_texture: zgpu.TextureHandle,
    depth_texture_view: zgpu.TextureViewHandle,

    skeleton: zozz.Skeleton,
    walk: zozz.Animation,
    run: zozz.Animation,
    recoil: zozz.Animation,
    inst: zozz.Instance,
    ws: zozz.Workspace,

    joint_parents: []i32,
    upper_body_mask: []f32,
    line_vertices: []Vertex,
    mesh: zozz_mesh.Mesh,
    skinned_vertices: []zozz_mesh.SkinnedVertex,
    mesh_vertices: []Vertex,
    overlay_first_vertex: usize,

    head_joint: i32,
    arm_chain: ArmChain,
    controls: Controls,
    input: InputState,
    current_target_ms: [3]f32,
    current_hand_distance: f32,
    locomotion_phase: f32,
    last_sample_time: ?f32,
};

fn init(allocator: std.mem.Allocator, io: std.Io, window: *zglfw.Window) !DemoState {
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
    }, .{});
    defer gctx.releaseResource(bind_group_layout);

    const pipeline_layout = gctx.createPipelineLayout(&.{bind_group_layout});
    defer gctx.releaseResource(pipeline_layout);

    const pipeline = pipeline: {
        const vs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "vs");
        defer vs_module.release();

        const fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs");
        defer fs_module.release();

        const color_targets = [_]wgpu.ColorTargetState{.{
            .format = zgpu.GraphicsContext.surface_texture_format,
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
                .entry_point = wgpu.StringView.cFromZig("main"),
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
                .depth_write_enabled = wgpu.True,
                .depth_compare = .less,
            },
            .fragment = &wgpu.FragmentState{
                .module = fs_module,
                .entry_point = wgpu.StringView.cFromZig("main"),
                .target_count = color_targets.len,
                .targets = &color_targets,
            },
        };
        break :pipeline gctx.createRenderPipeline(pipeline_layout, pipeline_descriptor);
    };
    const mesh_pipeline = createColorPipeline(gctx, pipeline_layout, .triangle_list);

    const bind_group = gctx.createBindGroup(bind_group_layout, &.{
        .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(zm.Mat) },
    });

    var skeleton = try zozz.Skeleton.loadFromFileZ("assets/ual_skeleton.ozz");
    errdefer skeleton.deinit();

    var walk = try zozz.Animation.loadFromFileZ("assets/ual_walk.ozz");
    errdefer walk.deinit();

    var run = try zozz.Animation.loadFromFileZ("assets/ual_run.ozz");
    errdefer run.deinit();
    var recoil = try zozz.Animation.loadFromFileZ("assets/ual_pistol_shoot_additive.ozz");
    errdefer recoil.deinit();

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
    const upper_body_mask = try allocator.alloc(f32, joint_count);
    errdefer allocator.free(upper_body_mask);
    const upper_body_root = skeleton.findJointZ("spine_03");
    for (upper_body_mask, 0..) |*weight, joint| {
        var ancestor: i32 = @intCast(joint);
        weight.* = 0;
        while (ancestor >= 0) : (ancestor = skeleton.jointParent(ancestor)) {
            if (ancestor == upper_body_root) {
                weight.* = 1;
                break;
            }
        }
    }

    const bone_vertices = if (joint_count > 0) (joint_count - 1) * 2 else 0;
    const line_vertices = try allocator.alloc(Vertex, bone_vertices + extra_debug_vertices);
    errdefer allocator.free(line_vertices);

    const vertex_buffer = gctx.createBuffer(.{
        .usage = .{ .copy_dst = true, .vertex = true },
        .size = @sizeOf(Vertex) * line_vertices.len,
    });
    var mesh = try zozz_mesh.Mesh.loadFromFile(allocator, io, "assets/ual_mesh.zmesh");
    errdefer mesh.deinit(allocator);
    if (mesh.jointCount() != joint_count) return error.MeshSkeletonMismatch;
    const skinned_vertices = try allocator.alloc(zozz_mesh.SkinnedVertex, mesh.vertices.len);
    errdefer allocator.free(skinned_vertices);
    const mesh_vertices = try allocator.alloc(Vertex, mesh.vertices.len);
    errdefer allocator.free(mesh_vertices);
    const mesh_vertex_buffer = gctx.createBuffer(.{ .usage = .{ .copy_dst = true, .vertex = true }, .size = @sizeOf(Vertex) * mesh_vertices.len });
    const mesh_index_buffer = gctx.createBuffer(.{ .usage = .{ .copy_dst = true, .index = true }, .size = @sizeOf(u32) * mesh.indices.len });
    gctx.queue.writeBuffer(gctx.lookupResource(mesh_index_buffer).?, 0, u32, mesh.indices);

    const depth = createDepthTexture(gctx);

    return .{
        .gctx = gctx,
        .pipeline = pipeline,
        .mesh_pipeline = mesh_pipeline,
        .bind_group = bind_group,
        .vertex_buffer = vertex_buffer,
        .mesh_vertex_buffer = mesh_vertex_buffer,
        .mesh_index_buffer = mesh_index_buffer,
        .depth_texture = depth.texture,
        .depth_texture_view = depth.view,
        .skeleton = skeleton,
        .walk = walk,
        .run = run,
        .recoil = recoil,
        .inst = inst,
        .ws = ws,
        .joint_parents = joint_parents,
        .upper_body_mask = upper_body_mask,
        .line_vertices = line_vertices,
        .mesh = mesh,
        .skinned_vertices = skinned_vertices,
        .mesh_vertices = mesh_vertices,
        .overlay_first_vertex = 0,
        .head_joint = skeleton.findJointZ("Head"),
        .arm_chain = resolveArmChain(skeleton),
        .controls = .{},
        .input = .{},
        .current_target_ms = .{ 0.0, 1.2, -0.6 },
        .current_hand_distance = 0.0,
        .locomotion_phase = 0.0,
        .last_sample_time = null,
    };
}

fn deinit(allocator: std.mem.Allocator, demo: *DemoState) void {
    allocator.free(demo.mesh_vertices);
    allocator.free(demo.skinned_vertices);
    demo.mesh.deinit(allocator);
    allocator.free(demo.line_vertices);
    allocator.free(demo.upper_body_mask);
    allocator.free(demo.joint_parents);
    demo.ws.deinit(allocator);
    demo.inst.deinit(allocator);
    demo.run.deinit();
    demo.recoil.deinit();
    demo.walk.deinit();
    demo.skeleton.deinit();
    demo.gctx.destroy(allocator);
    demo.* = undefined;
}

fn draw(demo: *DemoState) !void {
    const gctx = demo.gctx;
    const framebuffer_size = gctx.window_provider.fn_getFramebufferSize(gctx.window_provider.window);
    const fb_width: u32 = @intCast(framebuffer_size[0]);
    const fb_height: u32 = @intCast(framebuffer_size[1]);
    const t: f32 = @floatCast(gctx.stats.time);

    const vertex_count = try updateSkeletonVertices(demo, t);
    if (vertex_count > 0) {
        const vertex_buffer = gctx.lookupResource(demo.vertex_buffer) orelse return;
        gctx.queue.writeBuffer(vertex_buffer, 0, Vertex, demo.line_vertices[0..vertex_count]);
    }
    if (demo.controls.show_mesh) {
        const mesh_vertex_buffer = gctx.lookupResource(demo.mesh_vertex_buffer) orelse return;
        gctx.queue.writeBuffer(mesh_vertex_buffer, 0, Vertex, demo.mesh_vertices);
    }

    const orbit = if (demo.controls.orbit_camera) 0.25 * t else 0.0;
    const eye = zm.f32x4(3.2 * @sin(orbit), 1.6, -3.2 * @cos(orbit), 1.0);
    const target = zm.f32x4(0.0, 1.0, 0.0, 1.0);
    const up = zm.f32x4(0.0, 1.0, 0.0, 0.0);
    const world_to_view = zm.lookAtRh(eye, target, up);
    const view_to_clip = zm.perspectiveFovRh(
        0.25 * math.pi,
        @as(f32, @floatFromInt(fb_width)) / @as(f32, @floatFromInt(fb_height)),
        0.01,
        200.0,
    );
    const object_to_clip = zm.mul(world_to_view, view_to_clip);

    var surface_texture: wgpu.SurfaceTexture = .{};
    gctx.surface.getCurrentTexture(&surface_texture);
    const back_buffer = surface_texture.texture orelse return;
    defer back_buffer.release();
    const back_buffer_view = back_buffer.createView(.{});
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

            const mem = gctx.uniformsAllocate(zm.Mat, 1);
            mem.slice[0] = zm.transpose(object_to_clip);

            pass.setBindGroup(0, bind_group, &.{mem.offset});
            if (demo.controls.show_mesh) {
                const mesh_vb = gctx.lookupResourceInfo(demo.mesh_vertex_buffer) orelse break :pass;
                const mesh_ib = gctx.lookupResourceInfo(demo.mesh_index_buffer) orelse break :pass;
                const mesh_pipeline = gctx.lookupResource(demo.mesh_pipeline) orelse break :pass;
                pass.setVertexBuffer(0, mesh_vb.gpuobj.?, 0, mesh_vb.size);
                pass.setIndexBuffer(mesh_ib.gpuobj.?, .uint32, 0, mesh_ib.size);
                pass.setPipeline(mesh_pipeline);
                pass.drawIndexed(@intCast(demo.mesh.indices.len), 1, 0, 0, 0);
                if (vertex_count > demo.overlay_first_vertex) {
                    pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                    pass.setPipeline(pipeline);
                    pass.draw(
                        @intCast(vertex_count - demo.overlay_first_vertex),
                        1,
                        @intCast(demo.overlay_first_vertex),
                        0,
                    );
                }
            } else {
                pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                pass.setPipeline(pipeline);
                pass.draw(@intCast(vertex_count), 1, 0, 0);
            }
        }

        break :commands encoder.finish(null);
    };
    defer commands.release();

    gctx.submit(&.{commands});

    if (gctx.present() == .surface_reconfigured) {
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
    const dt = dt: {
        const previous = demo.last_sample_time orelse {
            demo.last_sample_time = time_seconds;
            break :dt 0.0;
        };
        demo.last_sample_time = time_seconds;
        break :dt @max(0.0, time_seconds - previous);
    };
    const walk_frequency = 1.0 / @max(demo.walk.duration(), 1e-5);
    const run_frequency = 1.0 / @max(demo.run.duration(), 1e-5);
    const locomotion_frequency = walk_frequency + (run_frequency - walk_frequency) * blend;
    demo.locomotion_phase = @mod(demo.locomotion_phase + dt * locomotion_frequency, 1.0);
    const locomotion_phase = demo.locomotion_phase;

    var recoil_layer = zozz.Layer.atRatio(
        demo.recoil,
        @mod(time_seconds / @max(demo.recoil.duration(), 1e-5), 1.0),
        if (demo.controls.enable_additive) 1.0 else 0.0,
        .additive,
    );
    recoil_layer.joint_weights = demo.upper_body_mask;
    demo.inst.setLayers(&[_]zozz.Layer{
        zozz.Layer.atRatio(demo.walk, locomotion_phase, 1.0 - blend, .normal),
        zozz.Layer.atRatio(demo.run, locomotion_phase, blend, .normal),
        recoil_layer,
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
            demo.controls.arm_twist,
            demo.controls.arm_soften,
            demo.controls.arm_weight,
        );
        job_count += 1;
    }

    demo.inst.setIkJobs(jobs[0..job_count]);
    const palette = try zozz.evalModel3x4(&demo.inst, &demo.ws);
    try demo.mesh.skin(palette, demo.skinned_vertices);
    for (demo.skinned_vertices, demo.mesh_vertices) |source, *target| {
        target.* = .{ .position = source.position, .color = source.color };
    }

    var vertex_count: usize = 0;
    for (demo.joint_parents, 0..) |parent, joint| {
        if (parent < 0) continue;

        const parent_pos = paletteTranslation(palette, @intCast(parent));
        const joint_pos = paletteTranslation(palette, joint);
        const color = lineColor(joint, demo.joint_parents.len);
        appendLine(demo.line_vertices, &vertex_count, parent_pos, joint_pos, color);
    }
    demo.overlay_first_vertex = vertex_count;

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

    if (demo.controls.enable_arm_ik and demo.arm_chain.isValid()) {
        const shoulder = paletteTranslation(palette, @intCast(demo.arm_chain.start_joint));
        const pole = axisChoiceArray(demo.controls.arm_pole_axis);
        appendLine(
            demo.line_vertices,
            &vertex_count,
            shoulder,
            addScaled3(shoulder, pole, 0.45),
            .{ 1.0, 0.20, 0.85 },
        );
    }

    if (demo.controls.show_joint_axes) {
        if (demo.controls.enable_head_aim and demo.head_joint >= 0) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.head_joint),
                joint_axis_extent,
            );
        }
        if (demo.controls.enable_arm_ik and demo.arm_chain.isValid()) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.arm_chain.start_joint),
                joint_axis_extent,
            );
        }
        if (demo.controls.enable_arm_ik and demo.arm_chain.isValid()) {
            appendJointBasis(
                demo.line_vertices,
                &vertex_count,
                palette,
                @intCast(demo.arm_chain.mid_joint),
                joint_axis_extent,
            );
        }
        if (demo.controls.enable_arm_ik and demo.arm_chain.isValid()) {
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
    const ual_left = ArmChain{
        .side_label = "Left",
        .start_joint = skeleton.findJointZ("upperarm_l"),
        .mid_joint = skeleton.findJointZ("lowerarm_l"),
        .end_joint = skeleton.findJointZ("hand_l"),
    };
    if (ual_left.isValid()) return ual_left;

    const left = ArmChain{
        .side_label = "Left",
        .start_joint = skeleton.findJointZ("LeftArm"),
        .mid_joint = skeleton.findJointZ("LeftForeArm"),
        .end_joint = skeleton.findJointZ("LeftHandMiddle3"),
    };
    if (left.isValid()) return left;

    const right = ArmChain{
        .side_label = "Right",
        .start_joint = skeleton.findJointZ("RightArm"),
        .mid_joint = skeleton.findJointZ("RightForeArm"),
        .end_joint = skeleton.findJointZ("RightHandMiddle3"),
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

fn axisChoiceArray(choice: AxisChoice) [3]f32 {
    const value = axisChoiceVec3(choice);
    return .{ value.x, value.y, value.z };
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
        \\  M mesh/skeleton   1 auto blend   2 additive recoil   3 arm IK   4 head aim   5 target marker   6 orbit camera   7 joint axes
        \\  Q elbow axis   Y arm pole axis   W head forward axis   E head up axis
        \\  A/D blend      S/F arm weight        G/H arm soften      J/K arm twist
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
        "State: blend={d:.2} auto={s} add={s} arm={s} head={s} target={s} axes={s} orbit={s} arm_weight={d:.2} arm_soften={d:.2} arm_twist={d:.2} hand_dist={d:.3} head_weight={d:.2} target_forward={d:.2} target_lift={d:.2} elbow={s} arm_pole={s} head_forward={s} head_up={s}\n",
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
            demo.controls.arm_twist,
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

    if (consumeEdge(window, .m, &demo.input.toggle_mesh)) {
        demo.controls.show_mesh = !demo.controls.show_mesh;
        changed = true;
    }

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
    if (consumeEdge(window, .j, &demo.input.arm_twist_down)) {
        changed = stepClamped(&demo.controls.arm_twist, -0.10, -math.pi, math.pi) or changed;
    }
    if (consumeEdge(window, .k, &demo.input.arm_twist_up)) {
        changed = stepClamped(&demo.controls.arm_twist, 0.10, -math.pi, math.pi) or changed;
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
        "{s} | view {s} | blend {d:.2} | arm {s} {d:.2} s{d:.2} twist {d:.2} d{d:.3} | pole {s} | head {s} {d:.2} | axes {s} | elbow {s}",
        .{
            window_title,
            if (demo.controls.show_mesh) "mesh" else "skeleton",
            demo.controls.blend,
            onOff(demo.controls.enable_arm_ik),
            demo.controls.arm_weight,
            demo.controls.arm_soften,
            demo.controls.arm_twist,
            demo.current_hand_distance,
            axisChoiceLabel(demo.controls.arm_pole_axis),
            onOff(demo.controls.enable_head_aim),
            demo.controls.head_weight,
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

fn createColorPipeline(gctx: *zgpu.GraphicsContext, layout: zgpu.PipelineLayoutHandle, topology: wgpu.PrimitiveTopology) zgpu.RenderPipelineHandle {
    const vs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "mesh-vs");
    defer vs_module.release();
    const fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "mesh-fs");
    defer fs_module.release();
    const color_targets = [_]wgpu.ColorTargetState{.{ .format = zgpu.GraphicsContext.surface_texture_format }};
    const vertex_attributes = [_]wgpu.VertexAttribute{
        .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
        .{ .format = .float32x3, .offset = @offsetOf(Vertex, "color"), .shader_location = 1 },
    };
    const vertex_buffers = [_]wgpu.VertexBufferLayout{.{ .array_stride = @sizeOf(Vertex), .attribute_count = vertex_attributes.len, .attributes = &vertex_attributes }};
    return gctx.createRenderPipeline(layout, .{
        .vertex = .{ .module = vs_module, .entry_point = wgpu.StringView.cFromZig("main"), .buffer_count = vertex_buffers.len, .buffers = &vertex_buffers },
        .primitive = .{ .front_face = .ccw, .cull_mode = .none, .topology = topology },
        .depth_stencil = &.{ .format = .depth32_float, .depth_write_enabled = wgpu.True, .depth_compare = .less },
        .fragment = &.{ .module = fs_module, .entry_point = wgpu.StringView.cFromZig("main"), .target_count = color_targets.len, .targets = &color_targets },
    });
}

fn createDepthTexture(gctx: *zgpu.GraphicsContext) struct {
    texture: zgpu.TextureHandle,
    view: zgpu.TextureViewHandle,
} {
    const framebuffer_size = gctx.window_provider.fn_getFramebufferSize(gctx.window_provider.window);
    const texture = gctx.createTexture(.{
        .usage = .{ .render_attachment = true },
        .dimension = .tdim_2d,
        .size = .{
            .width = @intCast(framebuffer_size[0]),
            .height = @intCast(framebuffer_size[1]),
            .depth_or_array_layers = 1,
        },
        .format = .depth32_float,
        .mip_level_count = 1,
        .sample_count = 1,
    });
    const view = gctx.createTextureView(texture, .{});
    return .{ .texture = texture, .view = view };
}

pub fn main(process_init: std.process.Init) !void {
    try zglfw.init();
    defer zglfw.terminate();

    {
        var buffer: [1024]u8 = undefined;
        const path_len = std.process.executableDirPath(process_init.io, buffer[0..]) catch 1;
        const path = if (path_len == 1) "." else buffer[0..path_len];
        std.process.setCurrentPath(process_init.io, path) catch {};
    }

    zglfw.windowHint(.client_api, .no_api);

    const window = try zglfw.createWindow(1400, 900, window_title, null, null);
    defer window.destroy();

    const allocator = process_init.gpa;

    var demo = try init(allocator, process_init.io, window);
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
