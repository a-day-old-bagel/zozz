const std = @import("std");
const gltf = @import("gltf_import.zig");

pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    const args = try init.minimal.args.toSlice(allocator);
    if (args.len == 3 and std.mem.eql(u8, args[1], "--list")) {
        const names = try gltf.animationNames(allocator, init.io, args[2]);
        for (names, 0..) |name, i| std.debug.print("{d}: {s}\n", .{ i, name });
        return;
    }
    if (args.len == 4 and std.mem.eql(u8, args[1], "--all")) {
        const names = try gltf.animationNames(allocator, init.io, args[2]);
        try std.Io.Dir.cwd().createDirPath(init.io, args[3]);
        const skeleton_path = try std.fs.path.join(allocator, &.{ args[3], "skeleton.ozz" });
        for (names, 0..) |name, i| {
            const filename = try safeFilename(allocator, name);
            const animation_path = try std.fs.path.join(allocator, &.{ args[3], filename });
            try gltf.import(allocator, init.io, args[2], skeleton_path, animation_path, i);
            std.debug.print("wrote {s}\n", .{animation_path});
        }
        return;
    }
    if (args.len < 4 or args.len > 5) return usage(args[0]);
    const animation_index = if (args.len == 5) selector: {
        break :selector std.fmt.parseInt(usize, args[4], 10) catch {
            const names = try gltf.animationNames(allocator, init.io, args[1]);
            for (names, 0..) |name, i| if (std.mem.eql(u8, name, args[4])) break :selector i;
            return error.InvalidAnimationName;
        };
    } else 0;
    try gltf.import(allocator, init.io, args[1], args[2], args[3], animation_index);
}

fn usage(exe: []const u8) error{InvalidArguments} {
    std.log.err(
        "usage:\n  {s} --list <input.gltf|input.glb>\n  {s} --all <input.gltf|input.glb> <output-directory>\n  {s} <input> <skeleton.ozz> <animation.ozz> [animation-index-or-name]",
        .{ exe, exe, exe },
    );
    return error.InvalidArguments;
}

fn safeFilename(allocator: std.mem.Allocator, name: []const u8) ![]const u8 {
    const result = try allocator.alloc(u8, name.len + ".ozz".len);
    for (name, 0..) |character, i| result[i] = if (std.ascii.isAlphanumeric(character) or character == '-' or character == '_') character else '_';
    @memcpy(result[name.len..], ".ozz");
    return result;
}
