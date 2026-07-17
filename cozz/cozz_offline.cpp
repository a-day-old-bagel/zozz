#include "cozz_offline.h"

#include <new>
#include <string>

#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

namespace {
thread_local std::string g_error;

ozz_offline_result_t fail(ozz_offline_result_t result, const char* message) {
  g_error = message ? message : "";
  return result;
}

ozz::math::Transform convert(const ozz_offline_transform_t& value) {
  ozz::math::Transform result;
  result.translation = {value.translation.x, value.translation.y, value.translation.z};
  result.rotation = {value.rotation.x, value.rotation.y, value.rotation.z,
                     value.rotation.w};
  result.scale = {value.scale.x, value.scale.y, value.scale.z};
  return result;
}

bool write_skeleton(const char* path, const ozz::animation::Skeleton& skeleton) {
  ozz::io::File file(path, "wb");
  if (!file.opened()) return false;
  ozz::io::OArchive archive(&file);
  archive << skeleton;
  return true;
}

bool write_animation(const char* path, const ozz::animation::Animation& animation) {
  ozz::io::File file(path, "wb");
  if (!file.opened()) return false;
  ozz::io::OArchive archive(&file);
  archive << animation;
  return true;
}
}  // namespace

const char* ozz_offline_last_error(void) { return g_error.c_str(); }
void ozz_offline_clear_error(void) { g_error.clear(); }

ozz_offline_result_t ozz_offline_build_skeleton(
    const ozz_offline_joint_t* joints, size_t joint_count,
    const char* output_path) {
  ozz_offline_clear_error();
  if ((!joints && joint_count) || !output_path)
    return fail(OZZ_OFFLINE_ERR_INVALID_ARGUMENT, "null skeleton argument");

  ozz::animation::offline::RawSkeleton raw;
  ozz::vector<ozz::animation::offline::RawSkeleton::Joint*> linear;
  linear.reserve(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    if (!joints[i].name || joints[i].parent < -1 ||
        joints[i].parent >= static_cast<int32_t>(i))
      return fail(OZZ_OFFLINE_ERR_INVALID_DATA,
                  "joint name is null or parent order is invalid");
    auto* children = joints[i].parent < 0
                         ? &raw.roots
                         : &linear[static_cast<size_t>(joints[i].parent)]->children;
    children->resize(children->size() + 1);
    auto* joint = &children->back();
    joint->name = joints[i].name;
    joint->transform = convert(joints[i].rest_pose);
    linear.push_back(joint);
  }
  if (!raw.Validate())
    return fail(OZZ_OFFLINE_ERR_INVALID_DATA, "invalid raw skeleton");
  ozz::animation::offline::SkeletonBuilder builder;
  auto skeleton = builder(raw);
  if (!skeleton)
    return fail(OZZ_OFFLINE_ERR_INVALID_DATA, "skeleton build failed");
  if (!write_skeleton(output_path, *skeleton))
    return fail(OZZ_OFFLINE_ERR_IO, "could not write skeleton archive");
  return OZZ_OFFLINE_OK;
}

ozz_offline_result_t ozz_offline_build_animation(
    const char* name, float duration, const ozz_offline_track_t* tracks,
    size_t track_count, const char* output_path) {
  ozz_offline_clear_error();
  if (!name || (!tracks && track_count) || !output_path || duration <= 0.f)
    return fail(OZZ_OFFLINE_ERR_INVALID_ARGUMENT, "invalid animation argument");
  ozz::animation::offline::RawAnimation raw;
  raw.name = name;
  raw.duration = duration;
  raw.tracks.resize(track_count);
  for (size_t i = 0; i < track_count; ++i) {
    const auto& source = tracks[i];
    if ((!source.translations && source.translation_count) ||
        (!source.rotations && source.rotation_count) ||
        (!source.scales && source.scale_count))
      return fail(OZZ_OFFLINE_ERR_INVALID_ARGUMENT, "null animation key array");
    auto& target = raw.tracks[i];
    target.translations.resize(source.translation_count);
    for (size_t k = 0; k < source.translation_count; ++k)
      target.translations[k] = {source.translations[k].time,
                                {source.translations[k].value.x,
                                 source.translations[k].value.y,
                                 source.translations[k].value.z}};
    target.rotations.resize(source.rotation_count);
    for (size_t k = 0; k < source.rotation_count; ++k)
      target.rotations[k] = {source.rotations[k].time,
                             {source.rotations[k].value.x,
                              source.rotations[k].value.y,
                              source.rotations[k].value.z,
                              source.rotations[k].value.w}};
    target.scales.resize(source.scale_count);
    for (size_t k = 0; k < source.scale_count; ++k)
      target.scales[k] = {source.scales[k].time,
                          {source.scales[k].value.x, source.scales[k].value.y,
                           source.scales[k].value.z}};
  }
  if (!raw.Validate())
    return fail(OZZ_OFFLINE_ERR_INVALID_DATA, "invalid raw animation");
  ozz::animation::offline::AnimationBuilder builder;
  auto animation = builder(raw);
  if (!animation)
    return fail(OZZ_OFFLINE_ERR_INVALID_DATA, "animation build failed");
  if (!write_animation(output_path, *animation))
    return fail(OZZ_OFFLINE_ERR_IO, "could not write animation archive");
  return OZZ_OFFLINE_OK;
}
