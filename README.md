## zozz
This is a Zig wrapper around an opinionated subset of [the ozz-animation library](https://github.com/guillaumeblanc/ozz-animation/).

It's likely that only a small chunk of the functionality and API of ozz will be exposed, on an as-needed basis for my own projects, unless there is some great interest in a more complete or purist wrapper later on.

Currently, zozz works with Zig 0.16.0

There is a small graphical integration sample that uses `zglfw + zgpu + zmath` through `build.zig.zon` package dependencies and draws the animated skeleton as debug lines:

```sh
zig build debug-skeleton-run
```

That sample lives in `examples/debug_skeleton_wgpu.zig` and uses the bundled `.ozz` assets under `assets/`.

Runtime layer sampling uses upstream-style normalized phase through `Layer.atRatio(...)`.
