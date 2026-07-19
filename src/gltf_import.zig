const std = @import("std");
const offline = @import("zozz_offline.zig");

pub const ImportError = error{
    InvalidGltf,
    UnsupportedGltf,
    NoSkin,
    NoAnimation,
    InvalidAnimationIndex,
};

pub fn animationNames(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8) ![][]const u8 {
    const input = try std.Io.Dir.cwd().readFileAlloc(io, input_path, allocator, .unlimited);
    const document = try parseDocument(allocator, input);
    const animations = document.root.object.get("animations") orelse return error.NoAnimation;
    const names = try allocator.alloc([]const u8, animations.array.items.len);
    for (animations.array.items, 0..) |animation, i| {
        names[i] = if (animation.object.get("name")) |name| name.string else try std.fmt.allocPrint(allocator, "animation_{d}", .{i});
    }
    return names;
}

const Document = struct {
    root: std.json.Value,
    binary: []const u8,
};

fn number(value: std.json.Value) ImportError!f32 {
    return switch (value) {
        .float => |v| @floatCast(v),
        .integer => |v| @floatFromInt(v),
        else => error.InvalidGltf,
    };
}

fn index(value: std.json.Value) ImportError!usize {
    return switch (value) {
        .integer => |v| if (v >= 0) @intCast(v) else error.InvalidGltf,
        else => error.InvalidGltf,
    };
}

fn field(object: std.json.ObjectMap, name: []const u8) ImportError!std.json.Value {
    return object.get(name) orelse error.InvalidGltf;
}

fn parseDocument(allocator: std.mem.Allocator, bytes: []const u8) !Document {
    var json_bytes = bytes;
    var binary: []const u8 = &.{};
    if (bytes.len >= 12 and std.mem.eql(u8, bytes[0..4], "glTF")) {
        if (std.mem.readInt(u32, bytes[4..8], .little) != 2) return error.UnsupportedGltf;
        var cursor: usize = 12;
        if (cursor + 8 > bytes.len) return error.InvalidGltf;
        const json_len = std.mem.readInt(u32, bytes[cursor..][0..4], .little);
        const json_kind = std.mem.readInt(u32, bytes[cursor + 4 ..][0..4], .little);
        cursor += 8;
        if (json_kind != 0x4e4f534a or cursor + json_len > bytes.len) return error.InvalidGltf;
        json_bytes = bytes[cursor .. cursor + json_len];
        cursor += json_len;
        if (cursor + 8 <= bytes.len) {
            const bin_len = std.mem.readInt(u32, bytes[cursor..][0..4], .little);
            const bin_kind = std.mem.readInt(u32, bytes[cursor + 4 ..][0..4], .little);
            cursor += 8;
            if (bin_kind == 0x004e4942 and cursor + bin_len <= bytes.len)
                binary = bytes[cursor .. cursor + bin_len];
        }
    }
    const parsed = try std.json.parseFromSliceLeaky(std.json.Value, allocator, json_bytes, .{});
    if (parsed != .object) return error.InvalidGltf;
    return .{ .root = parsed, .binary = binary };
}

fn readBuffer(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8, document: Document) ![]const u8 {
    const buffers = (try field(document.root.object, "buffers")).array.items;
    if (buffers.len == 0 or buffers[0] != .object) return error.InvalidGltf;
    if (buffers[0].object.get("uri")) |uri_value| {
        if (uri_value != .string) return error.InvalidGltf;
        if (std.mem.startsWith(u8, uri_value.string, "data:")) {
            const marker = ";base64,";
            const marker_at = std.mem.indexOf(u8, uri_value.string, marker) orelse return error.UnsupportedGltf;
            const encoded = uri_value.string[marker_at + marker.len ..];
            const decoded = try allocator.alloc(u8, try std.base64.standard.Decoder.calcSizeForSlice(encoded));
            try std.base64.standard.Decoder.decode(decoded, encoded);
            return decoded;
        }
        const directory = std.fs.path.dirname(input_path) orelse ".";
        const path = try std.fs.path.join(allocator, &.{ directory, uri_value.string });
        return std.Io.Dir.cwd().readFileAlloc(io, path, allocator, .unlimited);
    }
    if (document.binary.len == 0) return error.InvalidGltf;
    return document.binary;
}

fn readAccessor(allocator: std.mem.Allocator, root: std.json.ObjectMap, buffer: []const u8, accessor_index: usize) ![]f32 {
    const accessors = (try field(root, "accessors")).array.items;
    if (accessor_index >= accessors.len or accessors[accessor_index] != .object) return error.InvalidGltf;
    const accessor = accessors[accessor_index].object;
    if (try index(try field(accessor, "componentType")) != 5126) return error.UnsupportedGltf;
    const count = try index(try field(accessor, "count"));
    const kind = (try field(accessor, "type")).string;
    const components: usize = if (std.mem.eql(u8, kind, "SCALAR")) 1 else if (std.mem.eql(u8, kind, "VEC2")) 2 else if (std.mem.eql(u8, kind, "VEC3")) 3 else if (std.mem.eql(u8, kind, "VEC4")) 4 else if (std.mem.eql(u8, kind, "MAT4")) 16 else return error.UnsupportedGltf;
    const view_index = try index(try field(accessor, "bufferView"));
    const views = (try field(root, "bufferViews")).array.items;
    if (view_index >= views.len or views[view_index] != .object) return error.InvalidGltf;
    const view = views[view_index].object;
    const view_offset = if (view.get("byteOffset")) |v| try index(v) else 0;
    const accessor_offset = if (accessor.get("byteOffset")) |v| try index(v) else 0;
    const packed_stride = components * @sizeOf(f32);
    const stride = if (view.get("byteStride")) |v| try index(v) else packed_stride;
    const start = view_offset + accessor_offset;
    if (stride < packed_stride or start + (if (count == 0) 0 else (count - 1) * stride + packed_stride) > buffer.len) return error.InvalidGltf;
    const result = try allocator.alloc(f32, count * components);
    for (0..count) |i| for (0..components) |component| {
        const offset = start + i * stride + component * 4;
        result[i * components + component] = @bitCast(std.mem.readInt(u32, buffer[offset..][0..4], .little));
    };
    return result;
}

const IntegerAccessor = struct { values: []u16, components: usize };

fn readIntegerAccessor(allocator: std.mem.Allocator, root: std.json.ObjectMap, buffer: []const u8, accessor_index: usize) !IntegerAccessor {
    const accessors = (try field(root, "accessors")).array.items;
    if (accessor_index >= accessors.len or accessors[accessor_index] != .object) return error.InvalidGltf;
    const accessor = accessors[accessor_index].object;
    const component_type = try index(try field(accessor, "componentType"));
    const component_bytes: usize = switch (component_type) {
        5121 => 1,
        5123 => 2,
        else => return error.UnsupportedGltf,
    };
    const count = try index(try field(accessor, "count"));
    const kind = (try field(accessor, "type")).string;
    const components: usize = if (std.mem.eql(u8, kind, "SCALAR")) 1 else if (std.mem.eql(u8, kind, "VEC4")) 4 else return error.UnsupportedGltf;
    const view_index = try index(try field(accessor, "bufferView"));
    const views = (try field(root, "bufferViews")).array.items;
    if (view_index >= views.len or views[view_index] != .object) return error.InvalidGltf;
    const view = views[view_index].object;
    const view_offset = if (view.get("byteOffset")) |v| try index(v) else 0;
    const accessor_offset = if (accessor.get("byteOffset")) |v| try index(v) else 0;
    const packed_stride = components * component_bytes;
    const stride = if (view.get("byteStride")) |v| try index(v) else packed_stride;
    const start = view_offset + accessor_offset;
    if (stride < packed_stride or start + (if (count == 0) 0 else (count - 1) * stride + packed_stride) > buffer.len) return error.InvalidGltf;
    const result = try allocator.alloc(u16, count * components);
    for (0..count) |i| for (0..components) |component| {
        const offset = start + i * stride + component * component_bytes;
        result[i * components + component] = if (component_bytes == 1) buffer[offset] else std.mem.readInt(u16, buffer[offset..][0..2], .little);
    };
    return .{ .values = result, .components = components };
}

pub fn importMesh(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8, output_path: []const u8) !void {
    const input = try std.Io.Dir.cwd().readFileAlloc(io, input_path, allocator, .unlimited);
    const document = try parseDocument(allocator, input);
    const root = document.root.object;
    const binary = try readBuffer(allocator, io, input_path, document);
    const nodes = (try field(root, "nodes")).array.items;
    const skins = (try field(root, "skins")).array.items;
    if (skins.len == 0) return error.NoSkin;
    const skin = skins[0].object;
    const skin_nodes = (try field(skin, "joints")).array.items;
    const parents = try allocator.alloc(i32, nodes.len);
    @memset(parents, -1);
    for (nodes, 0..) |node_value, parent_index| if (node_value == .object and node_value.object.get("children") != null) {
        for (node_value.object.get("children").?.array.items) |child| parents[try index(child)] = @intCast(parent_index);
    };
    const node_to_skin = try allocator.alloc(i32, nodes.len);
    @memset(node_to_skin, -1);
    for (skin_nodes, 0..) |node_value, i| node_to_skin[try index(node_value)] = @intCast(i);
    const old_to_new = try allocator.alloc(i32, skin_nodes.len);
    @memset(old_to_new, -1);
    var written: usize = 0;
    while (written < skin_nodes.len) {
        var progress = false;
        for (skin_nodes, 0..) |node_value, old_i| if (old_to_new[old_i] < 0) {
            var p = parents[try index(node_value)];
            while (p >= 0 and node_to_skin[@intCast(p)] < 0) p = parents[@intCast(p)];
            if (p < 0 or old_to_new[@intCast(node_to_skin[@intCast(p)])] >= 0) {
                old_to_new[old_i] = @intCast(written);
                written += 1;
                progress = true;
            }
        };
        if (!progress) return error.InvalidGltf;
    }
    const inverse_accessor = skin.get("inverseBindMatrices") orelse return error.InvalidGltf;
    const inverse_old = try readAccessor(allocator, root, binary, try index(inverse_accessor));
    if (inverse_old.len != skin_nodes.len * 16) return error.InvalidGltf;
    const inverse = try allocator.alloc(f32, inverse_old.len);
    for (0..skin_nodes.len) |old_i| @memcpy(inverse[@as(usize, @intCast(old_to_new[old_i])) * 16 ..][0..16], inverse_old[old_i * 16 ..][0..16]);

    const meshes = (try field(root, "meshes")).array.items;
    if (meshes.len == 0) return error.InvalidGltf;
    const primitives = (try field(meshes[0].object, "primitives")).array.items;
    var vertex_count: usize = 0;
    var index_count: usize = 0;
    for (primitives) |primitive_value| {
        const attributes = (try field(primitive_value.object, "attributes")).object;
        const positions_accessor = (try field(root, "accessors")).array.items[try index(try field(attributes, "POSITION"))].object;
        vertex_count += try index(try field(positions_accessor, "count"));
        const index_accessor = (try field(root, "accessors")).array.items[try index(try field(primitive_value.object, "indices"))].object;
        index_count += try index(try field(index_accessor, "count"));
    }
    const vertex_stride: usize = 68;
    const header_size: usize = 24;
    const total = header_size + vertex_count * vertex_stride + skin_nodes.len * 16 * 4 + index_count * 4;
    const output = try allocator.alloc(u8, total);
    @memset(output, 0);
    @memcpy(output[0..4], "ZMSH");
    std.mem.writeInt(u32, output[4..8], 1, .little);
    std.mem.writeInt(u32, output[8..12], @intCast(vertex_count), .little);
    std.mem.writeInt(u32, output[12..16], @intCast(index_count), .little);
    std.mem.writeInt(u32, output[16..20], @intCast(skin_nodes.len), .little);
    std.mem.writeInt(u32, output[20..24], @intCast(vertex_stride), .little);
    var vertex_base: usize = 0;
    var index_base: usize = 0;
    for (primitives) |primitive_value| {
        const primitive = primitive_value.object;
        const attributes = (try field(primitive, "attributes")).object;
        const positions = try readAccessor(allocator, root, binary, try index(try field(attributes, "POSITION")));
        const normals = try readAccessor(allocator, root, binary, try index(try field(attributes, "NORMAL")));
        const uvs = try readAccessor(allocator, root, binary, try index(try field(attributes, "TEXCOORD_0")));
        const joints = try readIntegerAccessor(allocator, root, binary, try index(try field(attributes, "JOINTS_0")));
        const weights = try readAccessor(allocator, root, binary, try index(try field(attributes, "WEIGHTS_0")));
        const count = positions.len / 3;
        if (normals.len != count * 3 or uvs.len != count * 2 or joints.values.len != count * 4 or weights.len != count * 4) return error.InvalidGltf;
        var color = [3]f32{ 0.7, 0.7, 0.7 };
        if (primitive.get("material")) |material_value| {
            const materials = (try field(root, "materials")).array.items;
            const material_i = try index(material_value);
            if (material_i < materials.len) if (materials[material_i].object.get("pbrMetallicRoughness")) |pbr| if (pbr.object.get("baseColorFactor")) |factor| {
                color = .{ try number(factor.array.items[0]), try number(factor.array.items[1]), try number(factor.array.items[2]) };
            };
        }
        for (0..count) |v| {
            const offset = header_size + (vertex_base + v) * vertex_stride;
            var cursor = offset;
            for (positions[v * 3 ..][0..3]) |value| {
                std.mem.writeInt(u32, output[cursor..][0..4], @bitCast(value), .little);
                cursor += 4;
            }
            for (normals[v * 3 ..][0..3]) |value| {
                std.mem.writeInt(u32, output[cursor..][0..4], @bitCast(value), .little);
                cursor += 4;
            }
            for (uvs[v * 2 ..][0..2]) |value| {
                std.mem.writeInt(u32, output[cursor..][0..4], @bitCast(value), .little);
                cursor += 4;
            }
            for (joints.values[v * 4 ..][0..4]) |joint| {
                if (joint >= old_to_new.len) return error.InvalidGltf;
                std.mem.writeInt(u16, output[cursor..][0..2], @intCast(old_to_new[joint]), .little);
                cursor += 2;
            }
            for (weights[v * 4 ..][0..4]) |value| {
                std.mem.writeInt(u32, output[cursor..][0..4], @bitCast(value), .little);
                cursor += 4;
            }
            for (color) |value| {
                std.mem.writeInt(u32, output[cursor..][0..4], @bitCast(value), .little);
                cursor += 4;
            }
        }
        const indices = try readIntegerAccessor(allocator, root, binary, try index(try field(primitive, "indices")));
        if (indices.components != 1) return error.InvalidGltf;
        for (indices.values, 0..) |source_index, i| {
            const offset = header_size + vertex_count * vertex_stride + skin_nodes.len * 64 + (index_base + i) * 4;
            std.mem.writeInt(u32, output[offset..][0..4], @intCast(vertex_base + source_index), .little);
        }
        vertex_base += count;
        index_base += indices.values.len;
    }
    var inverse_cursor = header_size + vertex_count * vertex_stride;
    for (inverse) |value| {
        std.mem.writeInt(u32, output[inverse_cursor..][0..4], @bitCast(value), .little);
        inverse_cursor += 4;
    }
    try std.Io.Dir.cwd().writeFile(io, .{ .sub_path = output_path, .data = output });
}

fn nodeTransform(node: std.json.ObjectMap) !offline.Transform {
    if (node.get("matrix") != null) return error.UnsupportedGltf;
    var result: offline.Transform = .{};
    if (node.get("translation")) |value| {
        const a = value.array.items;
        if (a.len != 3) return error.InvalidGltf;
        result.translation = .{ .x = try number(a[0]), .y = try number(a[1]), .z = try number(a[2]) };
    }
    if (node.get("rotation")) |value| {
        const a = value.array.items;
        if (a.len != 4) return error.InvalidGltf;
        result.rotation = .{ .x = try number(a[0]), .y = try number(a[1]), .z = try number(a[2]), .w = try number(a[3]) };
    }
    if (node.get("scale")) |value| {
        const a = value.array.items;
        if (a.len != 3) return error.InvalidGltf;
        result.scale = .{ .x = try number(a[0]), .y = try number(a[1]), .z = try number(a[2]) };
    }
    return result;
}

pub fn import(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8, skeleton_path: []const u8, animation_path: []const u8, animation_index: usize) !void {
    return importAnimation(allocator, io, input_path, skeleton_path, animation_path, animation_index, false, null);
}

pub fn importRange(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8, skeleton_path: []const u8, animation_path: []const u8, animation_index: usize, time_range: [2]f32) !void {
    return importAnimation(allocator, io, input_path, skeleton_path, animation_path, animation_index, false, time_range);
}

pub fn importAdditive(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8, skeleton_path: []const u8, animation_path: []const u8, animation_index: usize) !void {
    return importAnimation(allocator, io, input_path, skeleton_path, animation_path, animation_index, true, null);
}

fn importAnimation(allocator: std.mem.Allocator, io: std.Io, input_path: []const u8, skeleton_path: []const u8, animation_path: []const u8, animation_index: usize, additive: bool, time_range: ?[2]f32) !void {
    const input = try std.Io.Dir.cwd().readFileAlloc(io, input_path, allocator, .unlimited);
    const document = try parseDocument(allocator, input);
    const root = document.root.object;
    const binary = try readBuffer(allocator, io, input_path, document);
    const nodes = (try field(root, "nodes")).array.items;
    const skins_value = root.get("skins") orelse return error.NoSkin;
    if (skins_value.array.items.len == 0) return error.NoSkin;
    const skin = skins_value.array.items[0].object;
    const skin_nodes = (try field(skin, "joints")).array.items;
    if (skin_nodes.len == 0) return error.NoSkin;

    const node_to_skin = try allocator.alloc(i32, nodes.len);
    @memset(node_to_skin, -1);
    for (skin_nodes, 0..) |node_value, i| {
        const node_index = try index(node_value);
        if (node_index >= nodes.len) return error.InvalidGltf;
        node_to_skin[node_index] = @intCast(i);
    }
    const parents = try allocator.alloc(i32, nodes.len);
    @memset(parents, -1);
    for (nodes, 0..) |node_value, parent_index| if (node_value == .object) {
        if (node_value.object.get("children")) |children| for (children.array.items) |child| {
            const child_index = try index(child);
            if (child_index >= nodes.len) return error.InvalidGltf;
            parents[child_index] = @intCast(parent_index);
        };
    };

    // glTF skin order is not hierarchical, while ozz track order is depth-first.
    const order = try allocator.alloc(usize, skin_nodes.len);
    const old_to_new = try allocator.alloc(i32, skin_nodes.len);
    @memset(old_to_new, -1);
    var written: usize = 0;
    while (written < order.len) {
        var progress = false;
        for (skin_nodes, 0..) |node_value, old_index| if (old_to_new[old_index] < 0) {
            var p = parents[try index(node_value)];
            while (p >= 0 and node_to_skin[@intCast(p)] < 0) p = parents[@intCast(p)];
            if (p < 0 or old_to_new[@intCast(node_to_skin[@intCast(p)])] >= 0) {
                order[written] = old_index;
                old_to_new[old_index] = @intCast(written);
                written += 1;
                progress = true;
            }
        };
        if (!progress) return error.InvalidGltf;
    }

    const joints = try allocator.alloc(offline.Joint, order.len);
    for (order, 0..) |old_index, new_index| {
        const node_index = try index(skin_nodes[old_index]);
        const node = nodes[node_index].object;
        const name = if (node.get("name")) |v| v.string else try std.fmt.allocPrint(allocator, "joint_{d}", .{node_index});
        const name_z = try allocator.dupeZ(u8, name);
        var p = parents[node_index];
        while (p >= 0 and node_to_skin[@intCast(p)] < 0) p = parents[@intCast(p)];
        joints[new_index] = .{
            .name = name_z.ptr,
            .parent = if (p < 0) -1 else old_to_new[@intCast(node_to_skin[@intCast(p)])],
            .rest_pose = try nodeTransform(node),
        };
    }
    const skeleton_z = try allocator.dupeZ(u8, skeleton_path);
    try offline.buildSkeleton(joints, skeleton_z.ptr);

    const animations_value = root.get("animations") orelse return error.NoAnimation;
    if (animation_index >= animations_value.array.items.len) return error.InvalidAnimationIndex;
    const animation = animations_value.array.items[animation_index].object;
    const samplers = (try field(animation, "samplers")).array.items;
    const channels = (try field(animation, "channels")).array.items;
    const tracks = try allocator.alloc(offline.Track, joints.len);
    for (tracks, joints) |*track, joint| {
        const translation = try allocator.create(offline.Vec3Key);
        translation.* = .{ .time = 0, .value = joint.rest_pose.translation };
        const rotation = try allocator.create(offline.QuaternionKey);
        rotation.* = .{ .time = 0, .value = joint.rest_pose.rotation };
        const scale = try allocator.create(offline.Vec3Key);
        scale.* = .{ .time = 0, .value = joint.rest_pose.scale };
        track.* = .{
            .translations = @ptrCast(translation),
            .translation_count = 1,
            .rotations = @ptrCast(rotation),
            .rotation_count = 1,
            .scales = @ptrCast(scale),
            .scale_count = 1,
        };
    }
    if (time_range) |range| if (!std.math.isFinite(range[0]) or !std.math.isFinite(range[1]) or range[0] < 0 or range[1] <= range[0]) return error.InvalidAnimationRange;
    var duration: f32 = if (time_range) |range| range[1] - range[0] else 0;
    for (channels) |channel_value| {
        const channel = channel_value.object;
        const sampler_index = try index(try field(channel, "sampler"));
        if (sampler_index >= samplers.len) return error.InvalidGltf;
        const sampler = samplers[sampler_index].object;
        if (sampler.get("interpolation")) |v| if (!std.mem.eql(u8, v.string, "LINEAR")) return error.UnsupportedGltf;
        const target = (try field(channel, "target")).object;
        const target_node = try index(try field(target, "node"));
        if (target_node >= node_to_skin.len or node_to_skin[target_node] < 0) continue;
        const track_index: usize = @intCast(old_to_new[@intCast(node_to_skin[target_node])]);
        const times = try readAccessor(allocator, root, binary, try index(try field(sampler, "input")));
        const values = try readAccessor(allocator, root, binary, try index(try field(sampler, "output")));
        if (times.len == 0) continue;
        if (time_range == null) duration = @max(duration, times[times.len - 1]);
        const bounds = try animationKeyBounds(times, time_range);
        const key_count = bounds[1] - bounds[0] + 1;
        const time_origin = if (time_range) |range| range[0] else 0;
        const path = (try field(target, "path")).string;
        if (std.mem.eql(u8, path, "rotation")) {
            if (values.len != times.len * 4) return error.InvalidGltf;
            const keys = try allocator.alloc(offline.QuaternionKey, key_count);
            for (keys, bounds[0]..) |*key, i| key.* = .{ .time = if (times.len == 1) 0 else times[i] - time_origin, .value = .{ .x = values[i * 4], .y = values[i * 4 + 1], .z = values[i * 4 + 2], .w = values[i * 4 + 3] } };
            tracks[track_index].rotations = keys.ptr;
            tracks[track_index].rotation_count = keys.len;
        } else if (std.mem.eql(u8, path, "translation") or std.mem.eql(u8, path, "scale")) {
            if (values.len != times.len * 3) return error.InvalidGltf;
            const keys = try allocator.alloc(offline.Vec3Key, key_count);
            for (keys, bounds[0]..) |*key, i| key.* = .{ .time = if (times.len == 1) 0 else times[i] - time_origin, .value = .{ .x = values[i * 3], .y = values[i * 3 + 1], .z = values[i * 3 + 2] } };
            if (path[0] == 't') {
                tracks[track_index].translations = keys.ptr;
                tracks[track_index].translation_count = keys.len;
            } else {
                tracks[track_index].scales = keys.ptr;
                tracks[track_index].scale_count = keys.len;
            }
        }
    }
    if (duration <= 0) return error.InvalidGltf;
    const animation_name = if (animation.get("name")) |v| v.string else "animation";
    const animation_name_z = try allocator.dupeZ(u8, animation_name);
    const animation_z = try allocator.dupeZ(u8, animation_path);
    if (additive)
        try offline.buildAdditiveAnimation(animation_name_z.ptr, duration, tracks, animation_z.ptr)
    else
        try offline.buildAnimation(animation_name_z.ptr, duration, tracks, animation_z.ptr);
}

fn animationKeyBounds(times: []const f32, time_range: ?[2]f32) ![2]usize {
    const range = time_range orelse return .{ 0, times.len - 1 };
    if (times.len == 1) return .{ 0, 0 };
    const tolerance: f32 = 0.0001;
    var first: ?usize = null;
    var last: ?usize = null;
    for (times, 0..) |time, i| {
        if (first == null and @abs(time - range[0]) <= tolerance) first = i;
        if (@abs(time - range[1]) <= tolerance) last = i;
    }
    if (first == null or last == null or last.? <= first.?) return error.AnimationRangeMustAlignWithKeys;
    return .{ first.?, last.? };
}
