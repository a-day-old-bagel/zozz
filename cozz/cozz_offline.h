#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum ozz_offline_result_t {
  OZZ_OFFLINE_OK = 0,
  OZZ_OFFLINE_ERR = 1,
  OZZ_OFFLINE_ERR_INVALID_ARGUMENT = 2,
  OZZ_OFFLINE_ERR_IO = 3,
  OZZ_OFFLINE_ERR_INVALID_DATA = 4,
} ozz_offline_result_t;

typedef struct ozz_offline_vec3_t { float x, y, z; } ozz_offline_vec3_t;
typedef struct ozz_offline_quat_t { float x, y, z, w; } ozz_offline_quat_t;
typedef struct ozz_offline_transform_t {
  ozz_offline_vec3_t translation;
  ozz_offline_quat_t rotation;
  ozz_offline_vec3_t scale;
} ozz_offline_transform_t;

/* Joints must be depth-first ordered. A parent must precede its children. */
typedef struct ozz_offline_joint_t {
  const char* name;
  int32_t parent; /* -1 for a root */
  ozz_offline_transform_t rest_pose;
} ozz_offline_joint_t;

typedef struct ozz_offline_vec3_key_t {
  float time;
  ozz_offline_vec3_t value;
} ozz_offline_vec3_key_t;
typedef struct ozz_offline_quat_key_t {
  float time;
  ozz_offline_quat_t value;
} ozz_offline_quat_key_t;

typedef struct ozz_offline_track_t {
  const ozz_offline_vec3_key_t* translations;
  size_t translation_count;
  const ozz_offline_quat_key_t* rotations;
  size_t rotation_count;
  const ozz_offline_vec3_key_t* scales;
  size_t scale_count;
} ozz_offline_track_t;

const char* ozz_offline_last_error(void);
void ozz_offline_clear_error(void);

ozz_offline_result_t ozz_offline_build_skeleton(
    const ozz_offline_joint_t* joints, size_t joint_count,
    const char* output_path);

/* There must be exactly one track per joint in the matching skeleton order. */
ozz_offline_result_t ozz_offline_build_animation(
    const char* name, float duration,
    const ozz_offline_track_t* tracks, size_t track_count,
    const char* output_path);

/* Converts the supplied absolute tracks to deltas relative to their first
   frame before building the runtime animation. */
ozz_offline_result_t ozz_offline_build_additive_animation(
    const char* name, float duration,
    const ozz_offline_track_t* tracks, size_t track_count,
    const char* output_path);

#ifdef __cplusplus
} /* extern "C" */
#endif
