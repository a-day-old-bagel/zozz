const std = @import("std");
const math = std.math;
const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;
const zm = @import("zmath");
const zozz = @import("zozz_runtime");

const window_title = "zozz: debug skeleton (wgpu)";

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
    hip_joint: i32,
    knee_joint: i32,
    ankle_joint: i32,
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

    const line_vertices = try allocator.alloc(Vertex, if (joint_count > 0) (joint_count - 1) * 2 else 0);
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
        .hip_joint = skeleton.findJointZ("LeftUpLeg"),
        .knee_joint = skeleton.findJointZ("LeftLeg"),
        .ankle_joint = skeleton.findJointZ("LeftFoot"),
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

    const orbit = 0.25 * t;
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
    const blend = 0.5 + 0.5 * @sin(time_seconds * 0.35);
    const curl_weight = 0.15 + 0.15 * (0.5 + 0.5 * @sin(time_seconds * 1.1));
    const splay_weight = 0.10 + 0.20 * (0.5 + 0.5 * @cos(time_seconds * 0.9));

    demo.inst.setLayers(&[_]zozz.Layer{
        .{ .anim = demo.walk, .time_seconds = time_seconds, .wrap_time = true, .weight = 1.0 - blend, .mode = .normal },
        .{ .anim = demo.run, .time_seconds = time_seconds * 1.15, .wrap_time = true, .weight = blend, .mode = .normal },
        .{ .anim = demo.curl, .time_seconds = 0.0, .wrap_time = true, .weight = curl_weight, .mode = .additive },
        .{ .anim = demo.splay, .time_seconds = 0.0, .wrap_time = true, .weight = splay_weight, .mode = .additive },
    });
    demo.inst.setIkJobs(&.{});

    const base_palette = try zozz.evalModel3x4(&demo.inst, &demo.ws);

    var jobs: [2]zozz.IkJob = undefined;
    var job_count: usize = 0;

    if (demo.head_joint >= 0) {
        const head_pos = paletteTranslation(base_palette, @intCast(demo.head_joint));
        jobs[job_count] = zozz.IkJob.aim(
            demo.head_joint,
            .{
                .x = head_pos[0] + 0.65 * @sin(time_seconds * 0.8),
                .y = head_pos[1] + 0.25,
                .z = head_pos[2] - 0.60 - 0.15 * @cos(time_seconds * 0.8),
            },
            .{ .x = 0.0, .y = 1.0, .z = 0.0 },
            .{ .x = 1.0, .y = 0.0, .z = 0.0 },
            1.0,
        );
        job_count += 1;
    }

    if (demo.hip_joint >= 0 and demo.knee_joint >= 0 and demo.ankle_joint >= 0) {
        const ankle_pos = paletteTranslation(base_palette, @intCast(demo.ankle_joint));
        const knee_pole = paletteColumn(base_palette, @intCast(demo.knee_joint), 1);
        jobs[job_count] = zozz.IkJob.twoBone(
            demo.hip_joint,
            demo.knee_joint,
            demo.ankle_joint,
            .{
                .x = ankle_pos[0] + 0.10 * @sin(time_seconds * 1.0),
                .y = ankle_pos[1] - 0.08 + 0.05 * @sin(time_seconds * 1.7),
                .z = ankle_pos[2] + 0.10 * @cos(time_seconds * 1.0),
            },
            .{
                .x = knee_pole[0],
                .y = knee_pole[1],
                .z = knee_pole[2],
            },
            0.85,
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

        demo.line_vertices[vertex_count + 0] = .{ .position = parent_pos, .color = color };
        demo.line_vertices[vertex_count + 1] = .{ .position = joint_pos, .color = color };
        vertex_count += 2;
    }

    return vertex_count;
}

fn paletteTranslation(palette: []const f32, joint: usize) [3]f32 {
    const base = joint * 12;
    return .{
        palette[base + 9],
        palette[base + 10],
        palette[base + 11],
    };
}

fn paletteColumn(palette: []const f32, joint: usize, col: usize) [3]f32 {
    const base = joint * 12 + col * 3;
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

    while (!window.shouldClose() and window.getKey(.escape) != .press) {
        zglfw.pollEvents();
        try draw(&demo);
    }
}
