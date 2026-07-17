const std = @import("std");

pub const SourceVertex = struct {
    position: [3]f32,
    normal: [3]f32,
    uv: [2]f32,
    joints: [4]u16,
    weights: [4]f32,
    color: [3]f32,
};

pub const SkinnedVertex = extern struct {
    position: [3]f32,
    normal: [3]f32,
    color: [3]f32,
};

pub const Mesh = struct {
    vertices: []SourceVertex,
    indices: []u32,
    inverse_bind_matrices: []f32,

    pub fn loadFromFile(allocator: std.mem.Allocator, io: std.Io, path: []const u8) !Mesh {
        const data = try std.Io.Dir.cwd().readFileAlloc(io, path, allocator, .unlimited);
        defer allocator.free(data);
        if (data.len < 24 or !std.mem.eql(u8, data[0..4], "ZMSH") or std.mem.readInt(u32, data[4..8], .little) != 1) return error.InvalidMesh;
        const vertex_count: usize = std.mem.readInt(u32, data[8..12], .little);
        const index_count: usize = std.mem.readInt(u32, data[12..16], .little);
        const joint_count: usize = std.mem.readInt(u32, data[16..20], .little);
        const stride: usize = std.mem.readInt(u32, data[20..24], .little);
        if (stride != 68 or data.len != 24 + vertex_count * stride + joint_count * 64 + index_count * 4) return error.InvalidMesh;
        const vertices = try allocator.alloc(SourceVertex, vertex_count);
        errdefer allocator.free(vertices);
        for (vertices, 0..) |*vertex, i| {
            var cursor = 24 + i * stride;
            for (&vertex.position) |*value| { value.* = readFloat(data, cursor); cursor += 4; }
            for (&vertex.normal) |*value| { value.* = readFloat(data, cursor); cursor += 4; }
            for (&vertex.uv) |*value| { value.* = readFloat(data, cursor); cursor += 4; }
            for (&vertex.joints) |*value| { value.* = std.mem.readInt(u16, data[cursor..][0..2], .little); cursor += 2; }
            for (&vertex.weights) |*value| { value.* = readFloat(data, cursor); cursor += 4; }
            for (&vertex.color) |*value| { value.* = readFloat(data, cursor); cursor += 4; }
            for (vertex.joints) |joint| if (joint >= joint_count) return error.InvalidMesh;
        }
        const inverse_bind_matrices = try allocator.alloc(f32, joint_count * 16);
        errdefer allocator.free(inverse_bind_matrices);
        var cursor = 24 + vertex_count * stride;
        for (inverse_bind_matrices) |*value| { value.* = readFloat(data, cursor); cursor += 4; }
        const indices = try allocator.alloc(u32, index_count);
        errdefer allocator.free(indices);
        for (indices) |*value| { value.* = std.mem.readInt(u32, data[cursor..][0..4], .little); cursor += 4; if (value.* >= vertex_count) return error.InvalidMesh; }
        return .{ .vertices = vertices, .indices = indices, .inverse_bind_matrices = inverse_bind_matrices };
    }

    pub fn deinit(self: *Mesh, allocator: std.mem.Allocator) void {
        allocator.free(self.indices);
        allocator.free(self.inverse_bind_matrices);
        allocator.free(self.vertices);
        self.* = undefined;
    }

    pub fn jointCount(self: Mesh) usize { return self.inverse_bind_matrices.len / 16; }

    pub fn skin(self: Mesh, palette: []const f32, output: []SkinnedVertex) !void {
        if (palette.len != self.jointCount() * 12 or output.len != self.vertices.len) return error.InvalidSkinningInput;
        for (self.vertices, output) |source, *target| {
            var position = [3]f32{ 0, 0, 0 };
            var normal = [3]f32{ 0, 0, 0 };
            for (0..4) |influence| {
                const weight = source.weights[influence];
                if (weight == 0) continue;
                const joint: usize = source.joints[influence];
                const matrix = skinMatrix(palette[joint * 12 ..][0..12], self.inverse_bind_matrices[joint * 16 ..][0..16]);
                const p = transformPoint(matrix, source.position);
                const n = transformVector(matrix, source.normal);
                for (0..3) |axis| { position[axis] += p[axis] * weight; normal[axis] += n[axis] * weight; }
            }
            const length = @sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            if (length > 1e-8) {
                for (0..3) |axis| normal[axis] /= length;
            }
            target.* = .{ .position = position, .normal = normal, .color = source.color };
        }
    }
};

fn readFloat(data: []const u8, offset: usize) f32 { return @bitCast(std.mem.readInt(u32, data[offset..][0..4], .little)); }

fn skinMatrix(model3x4: []const f32, inverse: []const f32) [16]f32 {
    const model = [16]f32{
        model3x4[0], model3x4[1], model3x4[2], 0,
        model3x4[3], model3x4[4], model3x4[5], 0,
        model3x4[6], model3x4[7], model3x4[8], 0,
        model3x4[9], model3x4[10], model3x4[11], 1,
    };
    var result: [16]f32 = undefined;
    for (0..4) |column| for (0..4) |row| {
        result[column * 4 + row] = 0;
        for (0..4) |k| result[column * 4 + row] += model[k * 4 + row] * inverse[column * 4 + k];
    };
    return result;
}

fn transformPoint(matrix: [16]f32, point: [3]f32) [3]f32 {
    return .{
        matrix[0] * point[0] + matrix[4] * point[1] + matrix[8] * point[2] + matrix[12],
        matrix[1] * point[0] + matrix[5] * point[1] + matrix[9] * point[2] + matrix[13],
        matrix[2] * point[0] + matrix[6] * point[1] + matrix[10] * point[2] + matrix[14],
    };
}

fn transformVector(matrix: [16]f32, vector: [3]f32) [3]f32 {
    return .{
        matrix[0] * vector[0] + matrix[4] * vector[1] + matrix[8] * vector[2],
        matrix[1] * vector[0] + matrix[5] * vector[1] + matrix[9] * vector[2],
        matrix[2] * vector[0] + matrix[6] * vector[1] + matrix[10] * vector[2],
    };
}

test "identity bind pose preserves a weighted vertex" {
    var vertices = [_]SourceVertex{.{
        .position = .{ 1, 2, 3 }, .normal = .{ 0, 1, 0 }, .uv = .{ 0, 0 },
        .joints = .{ 0, 0, 0, 0 }, .weights = .{ 1, 0, 0, 0 }, .color = .{ 1, 1, 1 },
    }};
    var indices = [_]u32{0};
    var inverse = [_]f32{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    const mesh: Mesh = .{ .vertices = &vertices, .indices = &indices, .inverse_bind_matrices = &inverse };
    const palette = [_]f32{ 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 };
    var output: [1]SkinnedVertex = undefined;
    try mesh.skin(&palette, &output);
    try std.testing.expectEqualSlices(f32, &vertices[0].position, &output[0].position);
    try std.testing.expectEqualSlices(f32, &vertices[0].normal, &output[0].normal);
}

test "model palette translation is applied after inverse bind" {
    var vertices = [_]SourceVertex{.{
        .position = .{ 1, 2, 3 }, .normal = .{ 1, 0, 0 }, .uv = .{ 0, 0 },
        .joints = .{ 0, 0, 0, 0 }, .weights = .{ 1, 0, 0, 0 }, .color = .{ 1, 1, 1 },
    }};
    var indices = [_]u32{0};
    var inverse = [_]f32{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -1, -2, -3, 1 };
    const mesh: Mesh = .{ .vertices = &vertices, .indices = &indices, .inverse_bind_matrices = &inverse };
    const palette = [_]f32{ 1, 0, 0, 0, 1, 0, 0, 0, 1, 4, 6, 8 };
    var output: [1]SkinnedVertex = undefined;
    try mesh.skin(&palette, &output);
    try std.testing.expectApproxEqAbs(@as(f32, 4), output[0].position[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 6), output[0].position[1], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 8), output[0].position[2], 1e-6);
}
