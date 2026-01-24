
#include "cozz_runtime.h"

#include <string>
#include <cmath>
#include <new>
#include <cstring>
#include <cstdint>

#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/blending_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/ik_two_bone_job.h"
#include "ozz/animation/runtime/ik_aim_job.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/soa_float4x4.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/span.h"
#include "ozz/base/maths/vec_float.h"

static thread_local std::string g_last_error;

static ozz_result_t set_err(ozz_result_t code, const char* msg) {
  g_last_error = msg ? msg : "";
  return code;
}
const char* ozz_last_error(void) { return g_last_error.c_str(); }
void ozz_clear_error(void) { g_last_error.clear(); }

// ---- Opaque handles ----
struct ozz_skeleton_t { ozz::animation::Skeleton skel; };
struct ozz_animation_t { ozz::animation::Animation anim; };

// ---- bump-alloc into caller memory ----
static inline uintptr_t align_up_uintptr(uintptr_t p, size_t a) {
  const uintptr_t mask = (uintptr_t)(a - 1);
  return (p + mask) & ~mask;
}
static inline void* align_up_ptr(void* p, size_t a) {
  return (void*)align_up_uintptr((uintptr_t)p, a);
}
template <typename T>
static inline T* bump_alloc(void*& cursor, size_t& left, size_t count = 1) {
  void* aligned = align_up_ptr(cursor, alignof(T));
  uintptr_t cur = (uintptr_t)aligned;
  uintptr_t end = (uintptr_t)cursor + left;
  size_t bytes = sizeof(T) * count;
  if (cur + bytes > end) return nullptr;
  cursor = (void*)(cur + bytes);
  left = (size_t)(end - (cur + bytes));
  return (T*)aligned;
}

static inline int32_t num_soa_from_joints(int32_t n) { return (n + 3) / 4; }

template <typename T>
static ozz_result_t load_ozz_object_from_file(const char* path, T* out_obj) {
  if (!path || !out_obj) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null arg");
  ozz::io::File file(path, "rb");
  if (!file.opened()) return set_err(OZZ_ERR_IO, "open failed");
  ozz::io::IArchive ar(&file);
  if (!ar.TestTag<T>()) return set_err(OZZ_ERR_OZZ, "tag mismatch");
  ar >> *out_obj;
  if (!ar.ok()) return set_err(OZZ_ERR_OZZ, "read failed");
  return OZZ_OK;
}

ozz_result_t ozz_skeleton_load_from_file(const char* path, ozz_skeleton_t** out_skel) {
  ozz_clear_error();
  if (!out_skel) return set_err(OZZ_ERR_INVALID_ARGUMENT, "out_skel null");
  auto* h = new (std::nothrow) ozz_skeleton_t();
  if (!h) return set_err(OZZ_ERR, "oom");
  ozz_result_t r = load_ozz_object_from_file(path, &h->skel);
  if (r != OZZ_OK) { delete h; return r; }
  *out_skel = h;
  return OZZ_OK;
}

ozz_result_t ozz_animation_load_from_file(const char* path, ozz_animation_t** out_anim) {
  ozz_clear_error();
  if (!out_anim) return set_err(OZZ_ERR_INVALID_ARGUMENT, "out_anim null");
  auto* h = new (std::nothrow) ozz_animation_t();
  if (!h) return set_err(OZZ_ERR, "oom");
  ozz_result_t r = load_ozz_object_from_file(path, &h->anim);
  if (r != OZZ_OK) { delete h; return r; }
  *out_anim = h;
  return OZZ_OK;
}

void ozz_skeleton_destroy(ozz_skeleton_t* skel) { delete skel; }
void ozz_animation_destroy(ozz_animation_t* anim) { delete anim; }

int32_t ozz_skeleton_num_joints(const ozz_skeleton_t* skel) {
  return skel ? (int32_t)skel->skel.num_joints() : 0;
}
float ozz_animation_duration(const ozz_animation_t* anim) {
  return anim ? anim->anim.duration() : 0.0f;
}

// ---- Instance + Workspace ----
struct ozz_instance_t {
  const ozz::animation::Skeleton* skel;
  int32_t num_joints;
  int32_t num_soa;

  ozz::animation::SamplingJob::Context sampling_ctx;

  ozz::math::SoaTransform* accum; // persistent pose (SoA)

  ozz_layer_desc_t layers[OZZ_MAX_LAYERS];
  int32_t layer_count;

  ozz_ik_job_t ik[OZZ_MAX_IK_JOBS];
  int32_t ik_count;
};

struct ozz_workspace_t {
  const ozz::animation::Skeleton* skel;
  int32_t num_joints;
  int32_t num_soa;

  ozz::math::SoaTransform* temp;   // scratch (SoA)
  ozz::math::Float4x4* model;      // scratch
  float* palette;                 // output: 12*num_joints floats
};

size_t ozz_instance_required_bytes(const ozz_skeleton_t* skel_h) {
  if (!skel_h) return 0;
  const int32_t n = (int32_t)skel_h->skel.num_joints();
  const int32_t ns = num_soa_from_joints(n);

  size_t bytes = 0;
  auto bump = [&](size_t sz, size_t al) { bytes = (bytes + (al - 1)) & ~(al - 1); bytes += sz; };

  bump(sizeof(ozz_instance_t), alignof(ozz_instance_t));
  bump(sizeof(ozz::math::SoaTransform) * (size_t)ns, alignof(ozz::math::SoaTransform));
  return bytes;
}

ozz_result_t ozz_instance_init(void* mem, size_t mem_bytes, const ozz_skeleton_t* skel_h, ozz_instance_t** out_inst) {
  ozz_clear_error();
  if (!mem || !skel_h || !out_inst) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null arg");

  void* cur = mem;
  size_t left = mem_bytes;

  ozz_instance_t* inst = bump_alloc<ozz_instance_t>(cur, left, 1);
  if (!inst) return set_err(OZZ_ERR_INVALID_ARGUMENT, "mem too small (inst)");
  new (inst) ozz_instance_t();

  inst->skel = &skel_h->skel;
  inst->num_joints = (int32_t)skel_h->skel.num_joints();
  inst->num_soa = num_soa_from_joints(inst->num_joints);
  inst->sampling_ctx.Resize(inst->num_joints);

  inst->accum = bump_alloc<ozz::math::SoaTransform>(cur, left, (size_t)inst->num_soa);
  if (!inst->accum) return set_err(OZZ_ERR_INVALID_ARGUMENT, "mem too small (accum)");

  inst->layer_count = 0;
  inst->ik_count = 0;

  *out_inst = inst;
  return OZZ_OK;
}

void ozz_instance_set_layers(ozz_instance_t* inst, const ozz_layer_desc_t* layers, int32_t count) {
  if (!inst) return;
  if (!layers || count <= 0) { inst->layer_count = 0; return; }
  if (count > OZZ_MAX_LAYERS) count = OZZ_MAX_LAYERS;
  inst->layer_count = count;
  for (int32_t i = 0; i < count; ++i) inst->layers[i] = layers[i];
}

void ozz_instance_set_ik_jobs(ozz_instance_t* inst, const ozz_ik_job_t* jobs, int32_t count) {
  if (!inst) return;
  if (!jobs || count <= 0) { inst->ik_count = 0; return; }
  if (count > OZZ_MAX_IK_JOBS) count = OZZ_MAX_IK_JOBS;
  inst->ik_count = count;
  for (int32_t i = 0; i < count; ++i) inst->ik[i] = jobs[i];
}

size_t ozz_workspace_required_bytes(const ozz_skeleton_t* skel_h) {
  if (!skel_h) return 0;
  const int32_t n = (int32_t)skel_h->skel.num_joints();
  const int32_t ns = num_soa_from_joints(n);

  size_t bytes = 0;
  auto bump = [&](size_t sz, size_t al) { bytes = (bytes + (al - 1)) & ~(al - 1); bytes += sz; };

  bump(sizeof(ozz_workspace_t), alignof(ozz_workspace_t));
  bump(sizeof(ozz::math::SoaTransform) * (size_t)ns, alignof(ozz::math::SoaTransform)); // temp
  bump(sizeof(ozz::math::Float4x4) * (size_t)n, alignof(ozz::math::Float4x4));          // model
  bump(sizeof(float) * (size_t)(12 * n), alignof(float));                               // palette
  return bytes;
}

ozz_result_t ozz_workspace_init(void* mem, size_t mem_bytes, const ozz_skeleton_t* skel_h, ozz_workspace_t** out_ws) {
  ozz_clear_error();
  if (!mem || !skel_h || !out_ws) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null arg");

  void* cur = mem;
  size_t left = mem_bytes;

  ozz_workspace_t* ws = bump_alloc<ozz_workspace_t>(cur, left, 1);
  if (!ws) return set_err(OZZ_ERR_INVALID_ARGUMENT, "mem too small (ws)");
  new (ws) ozz_workspace_t();

  ws->skel = &skel_h->skel;
  ws->num_joints = (int32_t)skel_h->skel.num_joints();
  ws->num_soa = num_soa_from_joints(ws->num_joints);

  ws->temp = bump_alloc<ozz::math::SoaTransform>(cur, left, (size_t)ws->num_soa);
  if (!ws->temp) return set_err(OZZ_ERR_INVALID_ARGUMENT, "mem too small (temp)");

  ws->model = bump_alloc<ozz::math::Float4x4>(cur, left, (size_t)ws->num_joints);
  if (!ws->model) return set_err(OZZ_ERR_INVALID_ARGUMENT, "mem too small (model)");

  ws->palette = bump_alloc<float>(cur, left, (size_t)(12 * ws->num_joints));
  if (!ws->palette) return set_err(OZZ_ERR_INVALID_ARGUMENT, "mem too small (palette)");

  *out_ws = ws;
  return OZZ_OK;
}

const float* ozz_workspace_palette_3x4(const ozz_workspace_t* ws) { return ws ? ws->palette : nullptr; }
int32_t ozz_workspace_palette_floats(const ozz_workspace_t* ws) { return ws ? (12 * ws->num_joints) : 0; }

// ---- helpers ----
static inline float wrap_or_clamp(float t, float dur, int wrap) {
  if (dur <= 0.0f) return 0.0f;
  if (wrap) { t = std::fmod(t, dur); if (t < 0.0f) t += dur; return t; }
  if (t < 0.0f) return 0.0f;
  if (t > dur) return dur;
  return t;
}

static inline void store_3x4_col_major(const ozz::math::Float4x4& m, float* out12) {
  alignas(16) float c0[4], c1[4], c2[4], c3[4];
  ozz::math::Store3PtrU(m.cols[0], c0);
  ozz::math::Store3PtrU(m.cols[1], c1);
  ozz::math::Store3PtrU(m.cols[2], c2);
  ozz::math::Store3PtrU(m.cols[3], c3);
  out12[0]=c0[0]; out12[1]=c0[1]; out12[2]=c0[2];
  out12[3]=c1[0]; out12[4]=c1[1]; out12[5]=c1[2];
  out12[6]=c2[0]; out12[7]=c2[1]; out12[8]=c2[2];
  out12[9]=c3[0]; out12[10]=c3[1]; out12[11]=c3[2];
}

static inline ozz::math::Float3 f3(const ozz_vec3_t& v) { return ozz::math::Float3(v.x, v.y, v.z); }

static ozz_result_t sample_into(ozz_instance_t* inst, const ozz_animation_t* anim_h, float time_s, int wrap, ozz::math::SoaTransform* out) {
  if (!anim_h) return OZZ_ERR_INVALID_ARGUMENT;
  if ((int32_t)anim_h->anim.num_tracks() != inst->num_joints) return OZZ_ERR_INVALID_ARGUMENT;

  const float dur = anim_h->anim.duration();
  const float t = wrap_or_clamp(time_s, dur, wrap);
  const float ratio = (dur > 0.0f) ? (t / dur) : 0.0f;

  ozz::animation::SamplingJob job;
  job.animation = &anim_h->anim;
  job.context = &inst->sampling_ctx;
  job.ratio = ratio;
  job.output = ozz::span<ozz::math::SoaTransform>(out, inst->num_soa);

  return job.Run() ? OZZ_OK : OZZ_ERR_OZZ;
}

static ozz_result_t locals_to_model(const ozz_instance_t* inst, const ozz::math::SoaTransform* locals, ozz::math::Float4x4* out_model) {
  ozz::animation::LocalToModelJob job;
  job.skeleton = inst->skel;
  job.input = ozz::span<const ozz::math::SoaTransform>(locals, inst->num_soa);
  job.output = ozz::span<ozz::math::Float4x4>(out_model, inst->num_joints);
  return job.Run() ? OZZ_OK : OZZ_ERR_OZZ;
}

// ---- main eval ----
ozz_result_t ozz_eval_model_3x4(ozz_instance_t* inst, ozz_workspace_t* ws) {
  ozz_clear_error();
  if (!inst || !ws) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null inst/ws");
  if (inst->skel != ws->skel) return set_err(OZZ_ERR_INVALID_ARGUMENT, "inst/ws skeleton mismatch");
  if (inst->num_joints != ws->num_joints) return set_err(OZZ_ERR_INVALID_ARGUMENT, "inst/ws size mismatch");

  if (inst->layer_count <= 0) return set_err(OZZ_ERR_INVALID_ARGUMENT, "no layers");

  bool have_normal = false;
  float sum_normal = 0.0f;

  // 1) accumulate normal layers into inst->accum using ws->temp as scratch
  for (int32_t i = 0; i < inst->layer_count; ++i) {
    const ozz_layer_desc_t& L = inst->layers[i];
    if (!L.anim || L.weight <= 0.0f) continue;
    if (L.mode == OZZ_LAYER_ADDITIVE) continue;

    ozz_result_t r = sample_into(inst, L.anim, L.time_seconds, L.wrap_time, ws->temp);
    if (r != OZZ_OK) return set_err(r, "sample failed");

    if (!have_normal) {
      std::memcpy(inst->accum, ws->temp, sizeof(ozz::math::SoaTransform) * (size_t)inst->num_soa);
      have_normal = true;
      sum_normal = L.weight;
    } else {
      ozz::animation::BlendingJob::Layer two[2];
      two[0].transform = ozz::span<const ozz::math::SoaTransform>(inst->accum, inst->num_soa);
      two[0].weight = sum_normal;
      two[1].transform = ozz::span<const ozz::math::SoaTransform>(ws->temp, inst->num_soa);
      two[1].weight = L.weight;

      ozz::animation::BlendingJob job;
      job.threshold = 0.0f;
      job.layers = ozz::span<const ozz::animation::BlendingJob::Layer>(two, 2);
      job.additive_layers = ozz::span<const ozz::animation::BlendingJob::Layer>();
      job.output = ozz::span<ozz::math::SoaTransform>(inst->accum, inst->num_soa);

      if (!job.Run()) return set_err(OZZ_ERR_OZZ, "blend failed");
      sum_normal += L.weight;
    }
  }

  if (!have_normal) return set_err(OZZ_ERR_INVALID_ARGUMENT, "no normal layers with weight > 0");

  // 2) apply additive layers one-by-one on top of accum
  for (int32_t i = 0; i < inst->layer_count; ++i) {
    const ozz_layer_desc_t& L = inst->layers[i];
    if (!L.anim || L.weight <= 0.0f) continue;
    if (L.mode != OZZ_LAYER_ADDITIVE) continue;

    ozz_result_t r = sample_into(inst, L.anim, L.time_seconds, L.wrap_time, ws->temp);
    if (r != OZZ_OK) return set_err(r, "sample failed");

    ozz::animation::BlendingJob::Layer base;
    base.transform = ozz::span<const ozz::math::SoaTransform>(inst->accum, inst->num_soa);
    base.weight = 1.0f;

    ozz::animation::BlendingJob::Layer add;
    add.transform = ozz::span<const ozz::math::SoaTransform>(ws->temp, inst->num_soa);
    add.weight = L.weight;

    ozz::animation::BlendingJob job;
    job.threshold = 0.0f;
    job.layers = ozz::span<const ozz::animation::BlendingJob::Layer>(&base, 1);
    job.additive_layers = ozz::span<const ozz::animation::BlendingJob::Layer>(&add, 1);
    job.output = ozz::span<ozz::math::SoaTransform>(inst->accum, inst->num_soa);

    if (!job.Run()) return set_err(OZZ_ERR_OZZ, "additive blend failed");
  }

  // 3) IK: compute model, apply jobs (in-place on accum), recompute model
  if (inst->ik_count > 0) {
    ozz_result_t r = locals_to_model(inst, inst->accum, ws->model);
    if (r != OZZ_OK) return set_err(r, "local->model pre-IK failed");

    for (int32_t i = 0; i < inst->ik_count; ++i) {
      const ozz_ik_job_t& J = inst->ik[i];
      if (J.kind == OZZ_IK_TWO_BONE) {
        ozz::animation::IKTwoBoneJob job;
        job.weight = J.weight;
        job.joint = J.start_joint;
        job.mid_joint = J.mid_joint;
        job.end_joint = J.end_joint;
        job.target = f3(J.target_ms);
        job.pole_vector = f3(J.pole_ms);

        job.skeleton = inst->skel;
        job.input = ozz::span<const ozz::math::SoaTransform>(inst->accum, inst->num_soa);
        job.output = ozz::span<ozz::math::SoaTransform>(inst->accum, inst->num_soa);
        job.model_space_matrices = ozz::span<const ozz::math::Float4x4>(ws->model, inst->num_joints);

        if (!job.Run()) return set_err(OZZ_ERR_OZZ, "IKTwoBone failed");
      } else if (J.kind == OZZ_IK_AIM) {
        ozz::animation::IKAimJob job;
        job.weight = J.weight;
        job.joint = J.aim_joint;
        job.target = f3(J.aim_target_ms);
        job.forward = f3(J.forward_axis_ls);
        job.up = f3(J.up_axis_ls);

        job.skeleton = inst->skel;
        job.input = ozz::span<const ozz::math::SoaTransform>(inst->accum, inst->num_soa);
        job.output = ozz::span<ozz::math::SoaTransform>(inst->accum, inst->num_soa);
        job.model_space_matrices = ozz::span<const ozz::math::Float4x4>(ws->model, inst->num_joints);

        if (!job.Run()) return set_err(OZZ_ERR_OZZ, "IKAim failed");
      }
    }
  }

  // 4) final local->model + palette pack
  {
    ozz_result_t r = locals_to_model(inst, inst->accum, ws->model);
    if (r != OZZ_OK) return set_err(r, "local->model failed");
    for (int32_t i = 0; i < inst->num_joints; ++i) {
      store_3x4_col_major(ws->model[i], ws->palette + (size_t)i * 12u);
    }
  }

  return OZZ_OK;
}
