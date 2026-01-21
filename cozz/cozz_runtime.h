#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum ozz_result_t {
  OZZ_OK = 0,
  OZZ_ERR = 1,
  OZZ_ERR_INVALID_ARGUMENT = 2,
  OZZ_ERR_IO = 3,
  OZZ_ERR_OZZ = 4,
} ozz_result_t;

const char* ozz_last_error(void);
void ozz_clear_error(void);

// Loaded runtime data
typedef struct ozz_skeleton_t ozz_skeleton_t;
typedef struct ozz_animation_t ozz_animation_t;

// Persistent evaluation instance: contains SamplingJob::Context and (optionally) a default output locals buffer.
// The instance itself is stored inside caller-provided memory.
typedef struct ozz_anim_instance_t ozz_anim_instance_t;

// ---------- Loading ----------
ozz_result_t ozz_skeleton_load_from_file(const char* path, ozz_skeleton_t** out_skel);
ozz_result_t ozz_animation_load_from_file(const char* path, ozz_animation_t** out_anim);
void ozz_skeleton_destroy(ozz_skeleton_t* skel);
void ozz_animation_destroy(ozz_animation_t* anim);

// ---------- Introspection ----------
int32_t ozz_skeleton_num_joints(const ozz_skeleton_t* skel);
int32_t ozz_animation_num_tracks(const ozz_animation_t* anim);
float   ozz_animation_duration(const ozz_animation_t* anim);

// ---------- Scratch sizing ----------
// Ozz SoA local transforms buffer size/alignment (SoaTransform[(num_joints+3)/4])
size_t ozz_soa_locals_bytes(const ozz_skeleton_t* skel);
size_t ozz_soa_locals_align(void);

// Model scratch buffer size/alignment (Float4x4[num_joints])
size_t ozz_model_scratch_bytes(const ozz_skeleton_t* skel);
size_t ozz_model_scratch_align(void);

// ---------- Instance memory sizing/init ----------
// Instance memory includes only the instance struct + SamplingJob::Context + (optionally) one SoA output locals buffer.
//
// If you prefer total control, you can set include_output_locals=0 and always provide out_locals_soa buffers yourself.
size_t ozz_anim_instance_required_bytes(const ozz_skeleton_t* skel, int include_output_locals);

// `mem` must be at least required_bytes, with reasonable alignment (16 is fine).
ozz_result_t ozz_anim_instance_init(
  void* mem, size_t mem_bytes,
  const ozz_skeleton_t* skel,
  int include_output_locals,
  ozz_anim_instance_t** out_inst
);

// If include_output_locals!=0, you can get the built-in output locals SoA buffer from the instance.
ozz_result_t ozz_anim_instance_get_output_locals(
  ozz_anim_instance_t* inst,
  void** out_locals_soa,
  size_t* out_bytes
);

// ---------- Time handling ----------
// If wrap_time != 0, time wraps into [0, duration). Else clamps [0, duration].
float ozz_normalize_time(const ozz_animation_t* anim, float time_seconds, int wrap_time);

// ---------- Sampling ----------
// Samples one animation into a caller-provided SoA locals buffer.
ozz_result_t ozz_sample_locals_soa(
  ozz_anim_instance_t* inst,
  const ozz_animation_t* anim,
  float normalized_time_seconds, // you usually pass ozz_normalize_time(...)
  void* out_locals_soa,
  size_t out_locals_bytes
);

// ---------- Blending ----------
// A simple blend stack: N layers blended additively or normally into out_locals_soa.
//
// This is a “generic enough” primitive: you can build walk/jog/run (N=3) and later layer upper-body (additive).
typedef enum ozz_blend_mode_t {
  OZZ_BLEND_NORMAL = 0,
  OZZ_BLEND_ADDITIVE = 1
} ozz_blend_mode_t;

typedef struct ozz_blend_layer_t {
  const void* locals_soa;     // sampled locals SoA
  float weight;               // 0..1
  ozz_blend_mode_t mode;      // normal or additive
} ozz_blend_layer_t;

// Blends layers into out_locals_soa (SoA).
ozz_result_t ozz_blend_locals_soa(
  ozz_anim_instance_t* inst,
  const ozz_blend_layer_t* layers,
  int32_t layer_count,
  void* out_locals_soa,
  size_t out_locals_bytes
);

// ---------- Local->Model + pack to 3x4 ----------
// out_3x4_col_major is float[12 * num_joints]. Format described below.
ozz_result_t ozz_locals_to_model_3x4(
  ozz_anim_instance_t* inst,
  const void* locals_soa,
  size_t locals_bytes,
  void* model_scratch,          // Float4x4[num_joints] scratch (caller-owned)
  size_t model_scratch_bytes,
  float* out_3x4_col_major      // 12 * num_joints floats
);

// Convenience: sample N clips, blend, and output 3x4 in one call.
// You provide sampled_locals_soa[i] buffers for each layer (caller-owned).
typedef struct ozz_eval_layer_t {
  const ozz_animation_t* anim;
  float time_seconds;     // unnormalized
  int wrap_time;          // normalize per-layer
  float weight;
  ozz_blend_mode_t mode;
  void* sampled_locals_soa;
  size_t sampled_locals_bytes;
} ozz_eval_layer_t;

ozz_result_t ozz_eval_blend_model_3x4(
  ozz_anim_instance_t* inst,
  const ozz_eval_layer_t* layers,
  int32_t layer_count,
  void* out_locals_soa,          // can be instance output or caller buffer
  size_t out_locals_bytes,
  void* model_scratch,
  size_t model_scratch_bytes,
  float* out_3x4_col_major
);

#ifdef __cplusplus
} // extern "C"
#endif
