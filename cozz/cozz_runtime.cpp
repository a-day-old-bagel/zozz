#include "cozz_runtime.h"

#include <string>
#include <cmath>
#include <new>
#include <cstdint>

#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/blending_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/soa_float4x4.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/span.h"

static thread_local std::string g_last_error;
static ozz_result_t set_err(ozz_result_t code, const char* msg) {
  g_last_error = msg ? msg : "";
  return code;
}
const char* ozz_last_error(void) { return g_last_error.c_str(); }
void ozz_clear_error(void) { g_last_error.clear(); }

// ---- Opaque types ----
struct ozz_skeleton_t { ozz::animation::Skeleton skel; };
struct ozz_animation_t { ozz::animation::Animation anim; };

// ---- Simple bump allocator into caller memory ----
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
  const uintptr_t cur = (uintptr_t)aligned;
  const uintptr_t end = (uintptr_t)cursor + left;
  const size_t bytes = sizeof(T) * count;
  if (cur + bytes > end) return nullptr;
  cursor = (void*)(cur + bytes);
  left = (size_t)(end - (cur + bytes));
  return (T*)aligned;
}

// ---- Load helpers ----
template <typename T>
static ozz_result_t load_ozz_object_from_file(const char* path, T* out_obj) {
  if (!path || !out_obj) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null argument");

  ozz::io::File file(path, "rb");
  if (!file.opened()) return set_err(OZZ_ERR_IO, "failed to open file");

  ozz::io::IArchive archive(&file);
  if (!archive.TestTag<T>()) return set_err(OZZ_ERR_OZZ, "archive tag mismatch");
  archive >> *out_obj;
  return OZZ_OK;
}

ozz_result_t ozz_skeleton_load_from_file(const char* path, ozz_skeleton_t** out_skel) {
  ozz_clear_error();
  if (!out_skel) return set_err(OZZ_ERR_INVALID_ARGUMENT, "out_skel is null");
  auto* h = new (std::nothrow) ozz_skeleton_t();
  if (!h) return set_err(OZZ_ERR, "oom");
  const ozz_result_t r = load_ozz_object_from_file(path, &h->skel);
  if (r != OZZ_OK) { delete h; return r; }
  *out_skel = h;
  return OZZ_OK;
}

ozz_result_t ozz_animation_load_from_file(const char* path, ozz_animation_t** out_anim) {
  ozz_clear_error();
  if (!out_anim) return set_err(OZZ_ERR_INVALID_ARGUMENT, "out_anim is null");
  auto* h = new (std::nothrow) ozz_animation_t();
  if (!h) return set_err(OZZ_ERR, "oom");
  const ozz_result_t r = load_ozz_object_from_file(path, &h->anim);
  if (r != OZZ_OK) { delete h; return r; }
  *out_anim = h;
  return OZZ_OK;
}

void ozz_skeleton_destroy(ozz_skeleton_t* skel) { delete skel; }
void ozz_animation_destroy(ozz_animation_t* anim) { delete anim; }

int32_t ozz_skeleton_num_joints(const ozz_skeleton_t* skel) { return skel ? (int32_t)skel->skel.num_joints() : 0; }
int32_t ozz_animation_num_tracks(const ozz_animation_t* anim) { return anim ? (int32_t)anim->anim.num_tracks() : 0; }
float   ozz_animation_duration(const ozz_animation_t* anim) { return anim ? anim->anim.duration() : 0.0f; }

// ---- Sizing helpers ----
static inline int32_t num_soa_from_joints(int32_t num_joints) { return (num_joints + 3) / 4; }

size_t ozz_soa_locals_bytes(const ozz_skeleton_t* skel) {
  if (!skel) return 0;
  const int32_t n = (int32_t)skel->skel.num_joints();
  const int32_t ns = num_soa_from_joints(n);
  return sizeof(ozz::math::SoaTransform) * (size_t)ns;
}
size_t ozz_soa_locals_align(void) { return alignof(ozz::math::SoaTransform); }

size_t ozz_model_scratch_bytes(const ozz_skeleton_t* skel) {
  if (!skel) return 0;
  const int32_t n = (int32_t)skel->skel.num_joints();
  return sizeof(ozz::math::Float4x4) * (size_t)n;
}
size_t ozz_model_scratch_align(void) { return alignof(ozz::math::Float4x4); }

// ---- Instance ----
struct ozz_anim_instance_t {
  const ozz::animation::Skeleton* skel;
  int32_t num_joints;
  int32_t num_soa;
  ozz::animation::SamplingJob::Context sampling_ctx;

  // Optional output locals stored inside instance memory if requested.
  ozz::math::SoaTransform* output_locals_soa;
};

size_t ozz_anim_instance_required_bytes(const ozz_skeleton_t* skel, int include_output_locals) {
  if (!skel) return 0;
  const int32_t n = (int32_t)skel->skel.num_joints();
  const int32_t ns = num_soa_from_joints(n);

  size_t bytes = 0;
  auto bump = [&](size_t size, size_t align) {
    bytes = (bytes + (align - 1)) & ~(align - 1);
    bytes += size;
  };

  bump(sizeof(ozz_anim_instance_t), alignof(ozz_anim_instance_t));
  if (include_output_locals) {
    bump(sizeof(ozz::math::SoaTransform) * (size_t)ns, alignof(ozz::math::SoaTransform));
  }
  return bytes;
}

ozz_result_t ozz_anim_instance_init(
  void* mem, size_t mem_bytes,
  const ozz_skeleton_t* skel_h,
  int include_output_locals,
  ozz_anim_instance_t** out_inst)
{
  ozz_clear_error();
  if (!mem || !skel_h || !out_inst) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null argument");

  void* cursor = mem;
  size_t left = mem_bytes;

  ozz_anim_instance_t* inst = bump_alloc<ozz_anim_instance_t>(cursor, left, 1);
  if (!inst) return set_err(OZZ_ERR_INVALID_ARGUMENT, "insufficient instance memory");
  new (inst) ozz_anim_instance_t();

  inst->skel = &skel_h->skel;
  inst->num_joints = (int32_t)skel_h->skel.num_joints();
  inst->num_soa = num_soa_from_joints(inst->num_joints);

  inst->sampling_ctx.Resize(inst->num_joints);

  inst->output_locals_soa = nullptr;
  if (include_output_locals) {
    inst->output_locals_soa = bump_alloc<ozz::math::SoaTransform>(cursor, left, (size_t)inst->num_soa);
    if (!inst->output_locals_soa) return set_err(OZZ_ERR_INVALID_ARGUMENT, "insufficient memory for output locals");
  }

  *out_inst = inst;
  return OZZ_OK;
}

ozz_result_t ozz_anim_instance_get_output_locals(
  ozz_anim_instance_t* inst,
  void** out_locals_soa,
  size_t* out_bytes)
{
  ozz_clear_error();
  if (!inst || !out_locals_soa || !out_bytes) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null argument");
  if (!inst->output_locals_soa) return set_err(OZZ_ERR_INVALID_ARGUMENT, "instance has no built-in output locals");
  *out_locals_soa = (void*)inst->output_locals_soa;
  *out_bytes = sizeof(ozz::math::SoaTransform) * (size_t)inst->num_soa;
  return OZZ_OK;
}

// ---- Time normalize ----
static inline float wrap_or_clamp(float t, float dur, int wrap) {
  if (dur <= 0.0f) return 0.0f;
  if (wrap) {
    t = std::fmod(t, dur);
    if (t < 0.0f) t += dur;
    return t;
  }
  if (t < 0.0f) return 0.0f;
  if (t > dur) return dur;
  return t;
}

float ozz_normalize_time(const ozz_animation_t* anim, float time_seconds, int wrap_time) {
  if (!anim) return 0.0f;
  return wrap_or_clamp(time_seconds, anim->anim.duration(), wrap_time);
}

// ---- Sampling into caller SoA buffer ----
ozz_result_t ozz_sample_locals_soa(
  ozz_anim_instance_t* inst,
  const ozz_animation_t* anim_h,
  float normalized_time_seconds,
  void* out_locals_soa,
  size_t out_locals_bytes)
{
  ozz_clear_error();
  if (!inst || !anim_h || !out_locals_soa) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null argument");

  const int32_t n = inst->num_joints;
  if ((int32_t)anim_h->anim.num_tracks() != n) {
    return set_err(OZZ_ERR_INVALID_ARGUMENT, "animation track count != skeleton joint count");
  }

  const size_t need = sizeof(ozz::math::SoaTransform) * (size_t)inst->num_soa;
  if (out_locals_bytes < need) return set_err(OZZ_ERR_INVALID_ARGUMENT, "out_locals_soa too small");

  const float dur = anim_h->anim.duration();
  const float ratio = (dur > 0.0f) ? (normalized_time_seconds / dur) : 0.0f;

  auto* out = (ozz::math::SoaTransform*)out_locals_soa;

  ozz::animation::SamplingJob job;
  job.animation = &anim_h->anim;
  job.context = &inst->sampling_ctx;
  job.ratio = ratio;
  job.output = ozz::span<ozz::math::SoaTransform>(out, inst->num_soa);

  if (!job.Run()) return set_err(OZZ_ERR_OZZ, "SamplingJob failed");
  return OZZ_OK;
}

// ---- Blending ----
ozz_result_t ozz_blend_locals_soa(
  ozz_anim_instance_t* inst,
  const ozz_blend_layer_t* layers,
  int32_t layer_count,
  void* out_locals_soa,
  size_t out_locals_bytes)
{
  ozz_clear_error();
  if (!inst || !layers || layer_count <= 0 || !out_locals_soa)
    return set_err(OZZ_ERR_INVALID_ARGUMENT, "invalid argument");

  const size_t need = sizeof(ozz::math::SoaTransform) * (size_t)inst->num_soa;
  if (out_locals_bytes < need) return set_err(OZZ_ERR_INVALID_ARGUMENT, "out_locals_soa too small");

  // Build BlendingJob layers on the stack (no heap).
  // Ozz's BlendingJob::Layer is a simple struct; we fill an array of them.
  // For safety, cap layer count (you can raise this).
  constexpr int32_t kMaxLayers = 16;
  if (layer_count > kMaxLayers) return set_err(OZZ_ERR_INVALID_ARGUMENT, "too many layers");

  ozz::animation::BlendingJob::Layer job_layers[kMaxLayers];

  for (int32_t i = 0; i < layer_count; ++i) {
    if (!layers[i].locals_soa) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null layer locals");

    job_layers[i].transform = ozz::span<const ozz::math::SoaTransform>(
      (const ozz::math::SoaTransform*)layers[i].locals_soa, inst->num_soa);
    job_layers[i].weight = layers[i].weight;

    // In ozz, additive blending is represented as a separate array of additive layers,
    // not a flag on each layer. To keep the C ABI simple, we’ll do:
    // - one pass for normal layers
    // - one pass for additive layers
    // That avoids heap and keeps predictable behavior.
  }

  auto* out = (ozz::math::SoaTransform*)out_locals_soa;

  // First: normal layers
  ozz::animation::BlendingJob job;
  job.threshold = 0.0f;
  job.output = ozz::span<ozz::math::SoaTransform>(out, inst->num_soa);

  // Split layers into normal vs additive without allocating:
  ozz::animation::BlendingJob::Layer normal[kMaxLayers];
  ozz::animation::BlendingJob::Layer additive[kMaxLayers];
  int32_t n_normal = 0;
  int32_t n_add = 0;

  for (int32_t i = 0; i < layer_count; ++i) {
    const auto span = ozz::span<const ozz::math::SoaTransform>(
      (const ozz::math::SoaTransform*)layers[i].locals_soa, inst->num_soa);

    ozz::animation::BlendingJob::Layer L;
    L.transform = span;
    L.weight = layers[i].weight;

    if (layers[i].mode == OZZ_BLEND_ADDITIVE) additive[n_add++] = L;
    else normal[n_normal++] = L;
  }

  job.layers = ozz::span<const ozz::animation::BlendingJob::Layer>(normal, n_normal);
  job.additive_layers = ozz::span<const ozz::animation::BlendingJob::Layer>(additive, n_add);

  if (!job.Run()) return set_err(OZZ_ERR_OZZ, "BlendingJob failed");
  return OZZ_OK;
}

// ---- Local->Model + pack to 3x4 ----
static inline void store_3x4_col_major(const ozz::math::Float4x4& m, float* out12) {
  ozz::math::Store3PtrU(m.cols[0], out12 + 0);  // col0.xyz
  ozz::math::Store3PtrU(m.cols[1], out12 + 3);  // col1.xyz
  ozz::math::Store3PtrU(m.cols[2], out12 + 6);  // col2.xyz
  ozz::math::Store3PtrU(m.cols[3], out12 + 9);  // col3.xyz (translation)
}

ozz_result_t ozz_locals_to_model_3x4(
  ozz_anim_instance_t* inst,
  const void* locals_soa,
  size_t locals_bytes,
  void* model_scratch,
  size_t model_scratch_bytes,
  float* out_3x4_col_major)
{
  ozz_clear_error();
  if (!inst || !locals_soa || !model_scratch || !out_3x4_col_major)
    return set_err(OZZ_ERR_INVALID_ARGUMENT, "null argument");

  const size_t need_locals = sizeof(ozz::math::SoaTransform) * (size_t)inst->num_soa;
  if (locals_bytes < need_locals) return set_err(OZZ_ERR_INVALID_ARGUMENT, "locals_soa too small");

  const size_t need_model = sizeof(ozz::math::Float4x4) * (size_t)inst->num_joints;
  if (model_scratch_bytes < need_model) return set_err(OZZ_ERR_INVALID_ARGUMENT, "model_scratch too small");

  auto* locals = (const ozz::math::SoaTransform*)locals_soa;
  auto* models = (ozz::math::Float4x4*)model_scratch;

  ozz::animation::LocalToModelJob job;
  job.skeleton = inst->skel;
  job.input = ozz::span<const ozz::math::SoaTransform>(locals, inst->num_soa);
  job.output = ozz::span<ozz::math::Float4x4>(models, inst->num_joints);

  if (!job.Run()) return set_err(OZZ_ERR_OZZ, "LocalToModelJob failed");

  for (int32_t i = 0; i < inst->num_joints; ++i) {
    store_3x4_col_major(models[i], out_3x4_col_major + (size_t)i * 12u);
  }
  return OZZ_OK;
}

// ---- Convenience: sample+blend+output ----
ozz_result_t ozz_eval_blend_model_3x4(
  ozz_anim_instance_t* inst,
  const ozz_eval_layer_t* layers,
  int32_t layer_count,
  void* out_locals_soa,
  size_t out_locals_bytes,
  void* model_scratch,
  size_t model_scratch_bytes,
  float* out_3x4_col_major)
{
  ozz_clear_error();
  if (!inst || !layers || layer_count <= 0) return set_err(OZZ_ERR_INVALID_ARGUMENT, "invalid args");
  if (!out_locals_soa || !model_scratch || !out_3x4_col_major) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null output");

  // 1) sample each layer into its provided sampled_locals_soa
  // 2) build blend layers array and blend into out_locals_soa
  // 3) locals->model and pack 3x4

  constexpr int32_t kMaxLayers = 16;
  if (layer_count > kMaxLayers) return set_err(OZZ_ERR_INVALID_ARGUMENT, "too many layers");

  ozz_blend_layer_t blend_layers[kMaxLayers];

  for (int32_t i = 0; i < layer_count; ++i) {
    if (!layers[i].anim || !layers[i].sampled_locals_soa) return set_err(OZZ_ERR_INVALID_ARGUMENT, "null layer");
    const float t = ozz_normalize_time(layers[i].anim, layers[i].time_seconds, layers[i].wrap_time);

    ozz_result_t r = ozz_sample_locals_soa(
      inst, layers[i].anim, t,
      layers[i].sampled_locals_soa, layers[i].sampled_locals_bytes);
    if (r != OZZ_OK) return r;

    blend_layers[i].locals_soa = layers[i].sampled_locals_soa;
    blend_layers[i].weight = layers[i].weight;
    blend_layers[i].mode = layers[i].mode;
  }

  ozz_result_t rblend = ozz_blend_locals_soa(
    inst, blend_layers, layer_count, out_locals_soa, out_locals_bytes);
  if (rblend != OZZ_OK) return rblend;

  return ozz_locals_to_model_3x4(
    inst, out_locals_soa, out_locals_bytes,
    model_scratch, model_scratch_bytes,
    out_3x4_col_major);
}
