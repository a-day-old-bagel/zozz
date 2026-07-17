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

## Offline compiler

`zozz-import` compiles a glTF 2.0 skin and one animation into the runtime ozz
archives consumed by `zozz_runtime`:

```sh
zig build importer
zig-out/bin/zozz-import character.glb character-skeleton.ozz walk.ozz
zig-out/bin/zozz-import character.gltf character-skeleton.ozz run.ozz 1
zig-out/bin/zozz-import character.glb character-skeleton.ozz idle.ozz Idle_Loop
```

The optional final argument selects an animation by zero-based index or exact
name. Animation discovery and bulk extraction are also available:

```sh
zig-out/bin/zozz-import --list character.glb
zig-out/bin/zozz-import --all character.glb compiled-animations
```

`--all` writes `skeleton.ozz` and one sanitized, name-based `.ozz` file per
animation into the output directory. The
importer supports `.glb`, `.gltf` with an external buffer, and base64 data-URI
buffers. Animation accessors must use float components and `LINEAR`
interpolation. Joint rest poses must currently be expressed as TRS; node
`matrix` transforms, sparse accessors, cubic-spline/step interpolation, morph
animation, mesh conversion, and retargeting are not yet supported.

Direct FBX import is not built in. Ozz's bundled FBX headers depend on the
proprietary Autodesk FBX SDK, but neither that SDK nor its implementation is
included here. Convert FBX to glTF/GLB with a tool such as Blender first, then
run `zozz-import`. This keeps zozz's normal build redistributable and offline.

Graphical example dependencies are optional so offline-only builds do not try
to fetch them. Use `zig build -Dexamples=true debug-skeleton` when those
dependencies are available. The default test command is simply:

```sh
zig build test
```
