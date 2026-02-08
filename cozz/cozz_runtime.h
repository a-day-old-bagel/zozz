
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

typedef struct ozz_skeleton_t ozz_skeleton_t;
typedef struct ozz_animation_t ozz_animation_t;

typedef struct ozz_instance_t ozz_instance_t;   // per-entity persistent state
typedef struct ozz_workspace_t ozz_workspace_t; // per-worker scratch/output

enum { OZZ_MAX_LAYERS = 8 };
enum { OZZ_MAX_IK_JOBS = 8 };

typedef enum ozz_layer_mode_t {
  OZZ_LAYER_NORMAL = 0,
  OZZ_LAYER_ADDITIVE = 1,
} ozz_layer_mode_t;

typedef struct ozz_layer_desc_t {
  const ozz_animation_t* anim;
  float time_seconds;
  int wrap_time; // 0 clamp, !=0 wrap
  float weight;
  ozz_layer_mode_t mode;
} ozz_layer_desc_t;

typedef struct ozz_vec3_t { float x, y, z; } ozz_vec3_t;

typedef enum ozz_ik_kind_t {
  OZZ_IK_NONE = 0,
  OZZ_IK_TWO_BONE = 1,
  OZZ_IK_AIM = 2,
} ozz_ik_kind_t;

typedef struct ozz_ik_job_t {
  ozz_ik_kind_t kind;
  float weight;

  // TWO_BONE
  int32_t start_joint;
  int32_t mid_joint;
  int32_t end_joint;
  ozz_vec3_t target_ms;
  ozz_vec3_t pole_ms;

  // AIM
  int32_t aim_joint;
  ozz_vec3_t aim_target_ms;
  ozz_vec3_t forward_axis_ls;
  ozz_vec3_t up_axis_ls;
} ozz_ik_job_t;

// Loading
ozz_result_t ozz_skeleton_load_from_file(const char* path, ozz_skeleton_t** out_skel);
ozz_result_t ozz_animation_load_from_file(const char* path, ozz_animation_t** out_anim);
void ozz_skeleton_destroy(ozz_skeleton_t* skel);
void ozz_animation_destroy(ozz_animation_t* anim);

int32_t ozz_skeleton_num_joints(const ozz_skeleton_t* skel);
float   ozz_animation_duration(const ozz_animation_t* anim);

// Instance (persistent, per entity)
size_t ozz_instance_required_bytes(const ozz_skeleton_t* skel);
ozz_result_t ozz_instance_init(void* mem, size_t mem_bytes, const ozz_skeleton_t* skel, ozz_instance_t** out_inst);

void ozz_instance_set_layers(ozz_instance_t* inst, const ozz_layer_desc_t* layers, int32_t count);
void ozz_instance_set_ik_jobs(ozz_instance_t* inst, const ozz_ik_job_t* jobs, int32_t count);

// Workspace (scratch/output, per worker thread or per batch)
size_t ozz_workspace_required_bytes(const ozz_skeleton_t* skel);
ozz_result_t ozz_workspace_init(void* mem, size_t mem_bytes, const ozz_skeleton_t* skel, ozz_workspace_t** out_ws);

// Evaluate: writes palette into workspace
// Palette format: float[12*num_joints], column-major 3x4 per joint.
ozz_result_t ozz_eval_model_3x4(ozz_instance_t* inst, ozz_workspace_t* ws);

// Access palette from workspace (valid until next eval on that workspace)
const float* ozz_workspace_palette_3x4(const ozz_workspace_t* ws);
int32_t      ozz_workspace_palette_floats(const ozz_workspace_t* ws); // = 12*num_joints

#ifdef __cplusplus
} // extern "C"
#endif
