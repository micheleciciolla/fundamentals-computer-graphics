//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, frenv of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "yocto_raytrace.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_shading.h>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Check texture size
static vec2i texture_size(const raytrace_texture* texture) {
  if (!texture->hdr.empty()) {
    return texture->hdr.imsize();
  } else if (!texture->ldr.empty()) {
    return texture->ldr.imsize();
  } else {
    return zero2i;
  }
}

// Evaluate a texture
static vec4f lookup_texture(const raytrace_texture* texture, const vec2i& ij,
    bool ldr_as_linear = false) {
  if (!texture->hdr.empty()) {
    return texture->hdr[ij];
  } else if (!texture->ldr.empty()) {
    return ldr_as_linear ? byte_to_float(texture->ldr[ij])
                         : srgb_to_rgb(byte_to_float(texture->ldr[ij]));
  } else {
    return {1, 1, 1, 1};
  }
}

// Evaluate a texture
static vec4f eval_texture(const raytrace_texture* texture, const vec2f& uv,
    bool ldr_as_linear = false, bool no_interpolation = false,
    bool clamp_to_edge = false) {
  auto output = zero4f;
  // get texture
  if (!texture) return {1, 1, 1, 1};

  // get yimg::image width/height
  auto size = texture_size(texture);

  // get coordinates normalized for tiling
  auto s = 0.0f, t = 0.0f;
  if (clamp_to_edge) {
    s = clamp(uv.x, 0.0f, 1.0f) * size.x;
    t = clamp(uv.y, 0.0f, 1.0f) * size.y;
  } else {
    s = fmod(uv.x, 1.0f) * size.x;
    if (s < 0) s += size.x;
    t = fmod(uv.y, 1.0f) * size.y;
    if (t < 0) t += size.y;
  }

  // get yimg::image coordinates and residuals
  auto i = clamp((int)s, 0, size.x - 1), j = clamp((int)t, 0, size.y - 1);
  auto ii = (i + 1) % size.x, jj = (j + 1) % size.y;
  auto u = s - i, v = t - j;

  if (no_interpolation) return lookup_texture(texture, {i, j}, ldr_as_linear);

  // handle interpolation
  output = lookup_texture(texture, {i, j}, ldr_as_linear) * (1 - u) * (1 - v) +
           lookup_texture(texture, {i, jj}, ldr_as_linear) * (1 - u) * v +
           lookup_texture(texture, {ii, j}, ldr_as_linear) * u * (1 - v) +
           lookup_texture(texture, {ii, jj}, ldr_as_linear) * u * v;
  return vec4f(output);
}

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const raytrace_camera* camera, const vec2f& image_uv) {
  // YOUR CODE GOES HERE - DONE

  // get ray from camera to point (u,v)
  // return object of type ray3f

  auto q = vec3f{camera->film.x * (0.5 - image_uv.x),
      camera->film.y * (image_uv.y - 0.5), camera->lens};
  auto e = vec3f{0};
  auto d = normalize(-q - e);
  return ray3f{
      transform_point(camera->frame, e), transform_direction(camera->frame, d)};
}

// Eval position
static vec3f eval_position(
    const raytrace_shape* shape, int element, const vec2f& uv) {
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return interpolate_triangle(shape->positions[t.x], shape->positions[t.y],
        shape->positions[t.z], uv);
  } else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return interpolate_line(shape->positions[l.x], shape->positions[l.y], uv.x);
  } else if (!shape->points.empty()) {
    return shape->positions[shape->points[element]];
  } else {
    return zero3f;
  }
}

// Shape element normal.
static vec3f eval_element_normal(const raytrace_shape* shape, int element) {
  // YOUR CODE GOES HERE - DONE

  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];

    return triangle_normal(
        shape->positions[t.x], shape->positions[t.y], shape->positions[t.z]);

  } else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return line_tangent(shape->positions[l.x], shape->positions[l.y]);
  } else if (!shape->points.empty()) {
    return {0, 0, 1};
  } else {
    return {0, 0, 0};
  }
}

// Eval normal
static vec3f eval_normal(
    const raytrace_shape* shape, int element, const vec2f& uv) {
  // YOUR CODE GOES HERE - DONE

  if (shape->normals.empty()) return eval_element_normal(shape, element);

  //----------------------------------------------------------------------
  // common layout
  //----------------------------------------------------------------------
  // case triangle
  if (!shape->triangles.empty()) {
    // the intersected triangle
    auto t = shape->triangles[element];
    // computed as barycentric coords
    return normalize(interpolate_triangle(
        shape->normals[t.x], shape->normals[t.y], shape->normals[t.z], uv));
  }
  // case line
  else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return normalize(
        interpolate_line(shape->normals[l.x], shape->normals[l.y], uv.x));
  }
  // last possible shape case is point
  else if (!shape->points.empty()) {
    return normalize(shape->normals[shape->points[element]]);
  } else {
    return {0, 0, 1};
  }
  //----------------------------------------------------------------------
}

// Eval texcoord
static vec2f eval_texcoord(
    const raytrace_shape* shape, int element, const vec2f& uv) {
  // YOUR CODE GOES HERE -----------------------

  //----------------------------------------------------------------------
  // common layout
  //----------------------------------------------------------------------
  // case triangle

  if (shape->texcoords.empty()) return uv;

  if (!shape->triangles.empty()) {
    // the intersected triangle
    auto t = shape->triangles[element];
    return (interpolate_triangle(shape->texcoords[t.x], shape->texcoords[t.y],
        shape->texcoords[t.z], uv));
  }
  // case line
  else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return (
        interpolate_line(shape->texcoords[l.x], shape->texcoords[l.y], uv.x));
  }
  // last possible shape case is point
  else if (!shape->points.empty()) {
    return (shape->texcoords[shape->points[element]]);
  } else {
    return {0, 0};
  }
  //----------------------------------------------------------------------
}

// Evaluate all environment color.
// environment lookup
static vec3f eval_environment(const raytrace_scene* scene, const ray3f& ray) {
  // YOUR CODE GOES HERE -----------------------

  auto curr_emission = zero3f;
  for (auto environment : scene->environments) {
    // obtain color using spherical coordinates
    auto local_dir = transform_direction(inverse(environment->frame), ray.d);
    auto texcoord  = vec2f{atan2(local_dir.z, local_dir.x) / (2 * pif),
        acos(clamp(local_dir.y, -1.0f, 1.0f)) / pif};

    if (texcoord.x < 0) texcoord.x += 1;

    // l' output viene sommato al valore della texure dell' environment
    curr_emission += (environment->emission *
                      xyz(eval_texture(environment->emission_tex, texcoord)));
  }
  return curr_emission;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SHAPE/SCENE BVH
// -----------------------------------------------------------------------------
namespace yocto {

// primitive used to sort bvh entries
struct raytrace_bvh_primitive {
  bbox3f bbox      = invalidb3f;
  vec3f  center    = zero3f;
  int    primitive = 0;
};

// Splits a BVH node. Returns split position and axis.
static pair<int, int> split_middle(
    vector<raytrace_bvh_primitive>& primitives, int start, int end) {
  // initialize split axis and position
  auto axis = 0;
  auto mid  = (start + end) / 2;

  // compute primintive bounds and size
  auto cbbox = invalidb3f;
  for (auto i = start; i < end; i++) cbbox = merge(cbbox, primitives[i].center);
  auto csize = cbbox.max - cbbox.min;
  if (csize == zero3f) return {mid, axis};

  // split along largest
  if (csize.x >= csize.y && csize.x >= csize.z) axis = 0;
  if (csize.y >= csize.x && csize.y >= csize.z) axis = 1;
  if (csize.z >= csize.x && csize.z >= csize.y) axis = 2;

  // split the space in the middle along the largest axis
  mid = (int)(std::partition(primitives.data() + start, primitives.data() + end,
                  [axis, middle = center(cbbox)[axis]](auto& primitive) {
                    return primitive.center[axis] < middle;
                  }) -
              primitives.data());

  // if we were not able to split, just break the primitives in half
  if (mid == start || mid == end) {
    // throw runtime_error("bad bvh split");
    mid = (start + end) / 2;
  }

  return {mid, axis};
}

// Maximum number of primitives per BVH node.
const int bvh_max_prims = 4;

// Build BVH nodes
static void build_bvh(vector<raytrace_bvh_node>& nodes,
    vector<raytrace_bvh_primitive>&              primitives) {
  // prepare to build nodes
  nodes.clear();
  nodes.reserve(primitives.size() * 2);

  // queue up first node
  auto queue = std::deque<vec3i>{{0, 0, (int)primitives.size()}};
  nodes.emplace_back();

  // create nodes until the queue is empty
  while (!queue.empty()) {
    // grab node to work on
    auto next = queue.front();
    queue.pop_front();
    auto nodeid = next.x, start = next.y, end = next.z;

    // grab node
    auto& node = nodes[nodeid];

    // compute bounds
    node.bbox = invalidb3f;
    for (auto i = start; i < end; i++)
      node.bbox = merge(node.bbox, primitives[i].bbox);

    // split into two children
    if (end - start > bvh_max_prims) {
      // get split
      auto [mid, axis] = split_middle(primitives, start, end);

      // make an internal node
      node.internal = true;
      node.axis     = axis;
      node.num      = 2;
      node.start    = (int)nodes.size();
      nodes.emplace_back();
      nodes.emplace_back();
      queue.push_back({node.start + 0, start, mid});
      queue.push_back({node.start + 1, mid, end});
    } else {
      // Make a leaf node
      node.internal = false;
      node.num      = end - start;
      node.start    = start;
    }
  }

  // cleanup
  nodes.shrink_to_fit();
}

static void init_bvh(raytrace_shape* shape, const raytrace_params& params) {
  // build primitives
  auto primitives = vector<raytrace_bvh_primitive>{};
  if (!shape->points.empty()) {
    for (auto idx = 0; idx < shape->points.size(); idx++) {
      auto& p             = shape->points[idx];
      auto& primitive     = primitives.emplace_back();
      primitive.bbox      = point_bounds(shape->positions[p], shape->radius[p]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  } else if (!shape->lines.empty()) {
    for (auto idx = 0; idx < shape->lines.size(); idx++) {
      auto& l         = shape->lines[idx];
      auto& primitive = primitives.emplace_back();
      primitive.bbox = line_bounds(shape->positions[l.x], shape->positions[l.y],
          shape->radius[l.x], shape->radius[l.y]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  } else if (!shape->triangles.empty()) {
    for (auto idx = 0; idx < shape->triangles.size(); idx++) {
      auto& primitive = primitives.emplace_back();
      auto& t         = shape->triangles[idx];
      primitive.bbox  = triangle_bounds(
          shape->positions[t.x], shape->positions[t.y], shape->positions[t.z]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  }

  // build nodes
  if (shape->bvh) delete shape->bvh;
  shape->bvh = new raytrace_bvh_tree{};
  build_bvh(shape->bvh->nodes, primitives);

  // set bvh primitives
  shape->bvh->primitives.reserve(primitives.size());
  for (auto& primitive : primitives) {
    shape->bvh->primitives.push_back(primitive.primitive);
  }
}

void init_bvh(raytrace_scene* scene, const raytrace_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1 + (int)scene->shapes.size()};

  // shapes
  for (auto idx = 0; idx < scene->shapes.size(); idx++) {
    if (progress_cb) progress_cb("build shape bvh", progress.x++, progress.y);
    init_bvh(scene->shapes[idx], params);
  }

  // handle progress
  if (progress_cb) progress_cb("build scene bvh", progress.x++, progress.y);

  // instance bboxes
  auto primitives = vector<raytrace_bvh_primitive>{};
  auto object_id  = 0;
  for (auto instance : scene->instances) {
    auto& primitive = primitives.emplace_back();
    primitive.bbox  = instance->shape->bvh->nodes.empty()
                         ? invalidb3f
                         : transform_bbox(instance->frame,
                               instance->shape->bvh->nodes[0].bbox);
    primitive.center    = center(primitive.bbox);
    primitive.primitive = object_id++;
  }

  // build nodes
  if (scene->bvh) delete scene->bvh;
  scene->bvh = new raytrace_bvh_tree{};
  build_bvh(scene->bvh->nodes, primitives);

  // set bvh primitives
  scene->bvh->primitives.reserve(primitives.size());
  for (auto& primitive : primitives) {
    scene->bvh->primitives.push_back(primitive.primitive);
  }

  // handle progress
  if (progress_cb) progress_cb("build bvh", progress.x++, progress.y);
}

// Intersect ray with a bvh->
static bool intersect_shape_bvh(raytrace_shape* shape, const ray3f& ray_,
    int& element, vec2f& uv, float& distance, bool find_any) {
  // get bvh and shape pointers for fast access
  auto bvh = shape->bvh;

  // check empty
  if (bvh->nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = bvh->nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to procenvd along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else if (!shape->points.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& p = shape->points[shape->bvh->primitives[idx]];
        if (intersect_point(
                ray, shape->positions[p], shape->radius[p], uv, distance)) {
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape->lines.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& l = shape->lines[shape->bvh->primitives[idx]];
        if (intersect_line(ray, shape->positions[l.x], shape->positions[l.y],
                shape->radius[l.x], shape->radius[l.y], uv, distance)) {
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape->triangles.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& t = shape->triangles[shape->bvh->primitives[idx]];
        if (intersect_triangle(ray, shape->positions[t.x],
                shape->positions[t.y], shape->positions[t.z], uv, distance)) {
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh->
static bool intersect_scene_bvh(const raytrace_scene* scene, const ray3f& ray_,
    int& instance, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
  // get bvh and scene pointers for fast access
  auto bvh = scene->bvh;

  // check empty
  if (bvh->nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = bvh->nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to procenvd along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto instance_ = scene->instances[scene->bvh->primitives[idx]];
        auto inv_ray   = transform_ray(
            inverse(instance_->frame, non_rigid_frames), ray);
        if (intersect_shape_bvh(
                instance_->shape, inv_ray, element, uv, distance, find_any)) {
          hit      = true;
          instance = scene->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh->
static bool intersect_instance_bvh(const raytrace_instance* instance,
    const ray3f& ray, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
  auto inv_ray = transform_ray(inverse(instance->frame, non_rigid_frames), ray);
  return intersect_shape_bvh(
      instance->shape, inv_ray, element, uv, distance, find_any);
}

raytrace_intersection intersect_scene_bvh(const raytrace_scene* scene,
    const ray3f& ray, bool find_any, bool non_rigid_frames) {
  auto intersection = raytrace_intersection{};
  intersection.hit  = intersect_scene_bvh(scene, ray, intersection.instance,
      intersection.element, intersection.uv, intersection.distance, find_any,
      non_rigid_frames);
  return intersection;
}
raytrace_intersection intersect_instance_bvh(const raytrace_instance* instance,
    const ray3f& ray, bool find_any, bool non_rigid_frames) {
  auto intersection = raytrace_intersection{};
  intersection.hit = intersect_instance_bvh(instance, ray, intersection.element,
      intersection.uv, intersection.distance, find_any, non_rigid_frames);
  return intersection;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

static vec4f get_vector_4f(vec3f vector_3f) {
  // -----------------------------------------------------------------------------
  // CUSTOM MADE : extracts 3 elements from input vector and add 1 to return
  // vec4f
  // -----------------------------------------------------------------------------
  // rgb_to_rgba does the same
  return {vector_3f.x, vector_3f.y, vector_3f.z, 1};
}

// Raytrace renderer.
static vec4f shade_raytrace(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // -----------------------------------------------------------------------------
  // shade_raytrace
  // -----------------------------------------------------------------------------

  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) {
    auto env = eval_environment(scene, ray);
    return vec4f{env.x, env.y, env.z, 1};
  }
  // -----------------------------------------------------------------------------
  // getting geometries
  // -----------------------------------------------------------------------------
  auto instance = scene->instances[isec.instance];

  auto shape    = instance->shape;
  auto material = instance->material;

  // calcoli posizione e normale usando il suo frame di riferimento
  auto position = transform_point(
      instance->frame, eval_position(shape, isec.element, isec.uv));
  auto normal = transform_direction(
      instance->frame, eval_normal(shape, isec.element, isec.uv));
  auto outgoing = -ray.d;

  // used in refraction for non thin material
  auto temp = dot(outgoing, normal);

  // handling normals and lines
  if (!instance->shape->lines.empty()) {
    // Tangent lines
    normal = orthonormalize(outgoing, normal);
  } else if (!instance->shape->triangles.empty()) {
    // Flip the normal if normal and outgoing are in opposite directions
    if (dot(outgoing, normal) < 0) normal = -normal;
  }

  // -----------------------------------------------------------------------------
  // getting properties
  // -----------------------------------------------------------------------------
  auto texcoord = eval_texcoord(shape, isec.element, isec.uv);
  auto color    = material->color *
               xyz(eval_texture(material->color_tex, texcoord));
  auto emission = material->emission *
                  xyz(eval_texture(material->emission_tex, texcoord));
  auto specular = material->specular *
                  xyz(eval_texture(material->specular_tex, texcoord)).x;
  auto metallic = material->metallic *
                  (eval_texture(material->metallic_tex, texcoord)).x;
  auto roughness = material->roughness *
                   (eval_texture(material->roughness_tex, texcoord)).x;
  // Professor's Hint
  roughness         = pow(roughness, 2);
  auto transmission = material->transmission *
                      (eval_texture(material->transmission_tex, texcoord)).x;

  // Hadling opcity
  // opacity stored in texture
  auto opacity = material->opacity;
  if (material->opacity_tex)
    opacity = opacity * (eval_texture(material->opacity_tex, texcoord)).x;

  // accumulating emission in the output vector radiance_output
  auto radiance_output = emission;

  // check exit
  if (bounce >= params.bounces) {
    return {radiance_output.x, radiance_output.y, radiance_output.z, 1};
  }

  // opcity check
  // approximating continuing the ray
  if (rand1f(rng) > opacity) {
    auto incoming = -outgoing;
    // recursive call
    return shade_raytrace(scene, {position, incoming}, bounce + 1, rng, params);
  }

  // -----------------------------------------------------------------------------
  // handling material characteristics
  // slide - 499
  // -----------------------------------------------------------------------------
  if (transmission) {
    //
    // Polished dielectrics scatter light both reflecting it and transmitting it
    //
    // slide 115
    //

    // -----------------------------------------------------------------------------
    // for thin materials fresnel_schlick approximates well the reflecting
    // behaviour
    // -----------------------------------------------------------------------------
    //
    // *** thin - fat decision ***
    //
    if (material->thin) {
      // do not refract
      //
      // random picking one direction based on fresnel term
      //
      // slide 517
      //
      if (rand1f(rng) <
          mean(fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing))) {
        //
        // incoming is obtained through reflect function
        //
        auto incoming = reflect(outgoing, normal);
        radiance_output += xyz(shade_raytrace(
            scene, {position, incoming}, bounce + 1, rng, params));
      } else {
        //
        // incoming is simply the opposite of the outgoing
        //
        auto incoming = -outgoing; // transmission
        radiance_output += color *
                           xyz(shade_raytrace(scene, {position, incoming},
                               bounce + 1, rng, params));
      }
    } else {
      //
      // for fat materials
      // check shade_refract
      //

      // ior of the material we're entering
      auto ior = reflectivity_to_eta(vec3f{0.04, 0.04, 0.04});

      // 
      // eta1 = air medium
      // eta1 first medium, eta2 second medium
      //

      auto eta = 1.f / mean(ior);

      //
      // RANDOM PICKING : Polished dielectrics scatter light both reflecting it and transmitting it
      //

      if (rand1f(rng) <
          mean(fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing))) {
        //
        auto incoming = reflect(outgoing, normal);
        radiance_output += xyz(shade_raytrace(
            scene, {position, incoming}, bounce + 1, rng, params));
      } else {
        
        // temp = dot(outgoing,normal) 
        // if < 0 you're leavig surface

        auto incoming = temp < 0 ? refract(outgoing, normal, 1 / eta) // RAY LEAVING SURFACE = 1/ETA
                                 : refract(outgoing, normal, eta);
        radiance_output += color *
                           xyz(shade_raytrace(scene, {position, incoming},
                               bounce + 1, rng, params));
      }
    }

  } else if (metallic && !roughness) {
    auto incoming = reflect(outgoing, normal);
    auto fs       = fresnel_schlick(color, normal, outgoing);
    radiance_output += fs * xyz(shade_raytrace(scene, {position, incoming},
                                bounce + 1, rng, params));
  } else if (metallic && roughness) {

    //
    // *** ROUGH METALS ***
    //
    // Rough metals scatter light around the reflected direction
    // Their scattering is modeled by considered tiny micro-facets, each of
    // which scatters light like a polished mirror
    //
    // slide 510
    //
    // direction coming inside
    
    auto incoming = sample_hemisphere(normal, rand2f(rng));
    auto halfway  = normalize(outgoing + incoming);
    auto fs       = fresnel_schlick(color, halfway, outgoing);

    auto micro_distribution = microfacet_distribution(
        roughness, normal, halfway);

    auto micro_shadow = microfacet_shadowing(
        roughness, normal, halfway, outgoing, incoming);

    auto den_term = 4 * abs(dot(normal, incoming)) * abs(dot(normal, outgoing));
    // Each summation term slide 508 (slide 105 slide prof)
    auto temp_term = fs * micro_distribution * micro_shadow / den_term;

    // Computing finally the lighting (recursively)
    radiance_output += (2 * pif) * abs(dot(normal, incoming)) * temp_term *
                       xyz(shade_raytrace(scene, {position, incoming},
                           bounce + 1, rng, params));

  } else if (specular) {
    auto incoming = sample_hemisphere(normal, rand2f(rng));
    auto halfway  = normalize(outgoing + incoming);
    auto fs       = fresnel_schlick({0.04, 0.04, 0.04}, halfway, outgoing);

    // same as before
    auto micro_distribution = microfacet_distribution(
        roughness, normal, halfway);
    auto micro_shadow = microfacet_shadowing(
        roughness, normal, halfway, outgoing, incoming);

    auto den_term = 4 * abs(dot(normal, incoming)) * abs(dot(normal, outgoing));

    auto temp_term = fs * micro_distribution * micro_shadow / den_term;

    // recursive call to compute light
    radiance_output += (2 * pif) * (color / pif * (1 - fs) + temp_term) *
                       abs(dot(normal, incoming)) *
                       xyz(shade_raytrace(scene, {position, incoming},
                           bounce + 1, rng, params));

  } else {
    //
    // *** MATTE SURFACE ***
    //
    // matte materials scatter light in all directions
    //
    auto incoming = sample_hemisphere(normal, rand2f(rng));

    radiance_output += (2 * pif) * (color / pif) * abs(dot(normal, incoming)) *
                       xyz(shade_raytrace(scene, {position, incoming},
                           bounce + 1, rng, params));
  }
  // output return
  return vec4f{radiance_output.x, radiance_output.y, radiance_output.z, 1.0f};
}

// Eyelight for quick previewing.
static vec4f shade_eyelight(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // -----------------------------------------------------------------------------
  // shade_eyelight : following slides
  // -----------------------------------------------------------------------------
  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) return zero4f;
  // eval_environment(scene, ray)
  auto object = scene->instances[isec.instance];

  // change coordinates
  auto normal = transform_direction(
      object->frame, eval_normal(object->shape, isec.element, isec.uv));

  // bring it to [0,1] range
  normal = normal * 0.5f + 0.5f;
  // -----------------------------------------------------------------------------

  // incoming direction -ray.d
  //
  auto shadde = abs(dot(normal, -ray.d));
  return get_vector_4f(object->material->color * shadde);
}

static vec4f shade_normal(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // YOUR CODE GOES HERE - DONE

  // -----------------------------------------------------------------------------
  // common layout
  // -----------------------------------------------------------------------------
  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) return zero4f;
  auto object = scene->instances[isec.instance];

  // change coordinates
  auto normal = transform_direction(
      object->frame, eval_normal(object->shape, isec.element, isec.uv));

  // bring it to [0,1] range
  normal = normal * 0.5f + 0.5f;

  return get_vector_4f(normal);
  // -----------------------------------------------------------------------------
}

static vec4f shade_texcoord(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // YOUR CODE GOES HERE - CODE

  // -----------------------------------------------------------------------------
  // common layout
  // -----------------------------------------------------------------------------
  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) return zero4f;
  auto object = scene->instances[isec.instance];

  // change coordinates ve2f
  auto texture_coordinates = eval_texcoord(
      object->shape, isec.element, isec.uv);

  // bring it to [0,1] range fmod to-do
  // force to [0,1] range
  // floating point modulus used to repeat textures

  return {fmod(texture_coordinates.x, 1), fmod(texture_coordinates.y, 1), 0, 1};
  // -----------------------------------------------------------------------------
}
//}  // namespace yocto

static vec4f shade_color(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // YOUR CODE GOES HERE - DONE
  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) return zero4f;

  auto object         = scene->instances[isec.instance];
  auto color_from_obj = object->material->color;
  // return {color_from_obj.x, color_from_obj.y, color_from_obj.z, 1};
  return get_vector_4f(color_from_obj);
}

// trying toon shader
static vec3f get_toon_effect(vec3f incoming, vec3f color, vec3f normal,
    vec3f outgoing, int inshadow, vec3f light_pos_wrld) {

  //
  // get_toon_effect :
  //
  // here i followed the proposed tutorial on unity, it has been foundamental to
  // understand and set world posiiton of the light aswell as specular colors,
  // ambient light, glossness. Honestly i've searched a lot on the web to better
  // understand how to do it. The final color is the sum of four components
  // calculated in the script.
  //
  // https://torchinsky.me/cel-shading/
  //

  float light_world_dot_normal = dot(light_pos_wrld, normal);

  float shadow         = 2;
  float light_intensty = smoothstep(0, 0.1, light_world_dot_normal * shadow); // to obtain shadow color cut

  auto ambient_clor = vec4f{0.3, 0.3, 0.3, 1};
  auto light_clor   = vec4f{0.3, 0.3, 0.3, 1};
  auto light        = light_intensty * inshadow;

  // FOLLOWING ROYSTAN TUTORIAL
  auto  specular_clor = vec4f{0.3, 0.3, 0.3, 1};
  float glossness     = 12;

  auto  halfway            = normalize(light_pos_wrld + outgoing);
  float normal_dot_halfway = dot(normal, halfway);
  float specular_intensty  = pow(
      normal_dot_halfway * light_intensty, glossness * glossness);

  float specular_intensty_smooth = smoothstep(0.005, 0.01, specular_intensty);
  auto  specular                 = specular_intensty_smooth * specular_clor;

  // Rim lighting is the addition of illumination to the edges of an object 
  // to simulate reflected light or 
  // backlighting. It is especially useful for toon shaders to help the object's 
  // silhouette stand out among the flat shaded surfaces.

  auto  rim_dot  = 1 - dot(outgoing, normal);
  auto  rim_clor = vec4f{1, 1, 1, 1};
  float rim_amnt = 0.9;

  // following tutorial

  float rim_throld   = 0.1;
  float rim_intensty = rim_dot * pow(light_world_dot_normal, rim_throld);
  rim_intensty = smoothstep(rim_amnt - 0.01, rim_amnt + 0.01, rim_intensty);
  auto rim     = rim_intensty * rim_clor;

  // getting the final result
  return color * xyz(light + ambient_clor + rim + specular);
}

static vec4f shade_myshader(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // -----------------------------------------------------------------------------
  // shade_myshader :
  // this is my custom made shader, the idea here was to merge toon effect with
  // warm yellow lamp effect
  // -----------------------------------------------------------------------------

  // as usual checking intersections and extracting element
  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) {
    auto env = eval_environment(scene, ray);
    return {env.x, env.y, env.z, 1};
  }

  // -----------------------------------------------------------------------------
  // getting info about the instance
  // -----------------------------------------------------------------------------
  auto instance = scene->instances[isec.instance];
  auto shape    = instance->shape;
  auto material = instance->material;
  auto position = transform_point(
      instance->frame, eval_position(shape, isec.element, isec.uv));
  auto normal = transform_direction(
      instance->frame, eval_normal(shape, isec.element, isec.uv));
  auto outgoing = -ray.d;
  
  // color and texture
  auto texcoord = eval_texcoord(shape, isec.element, isec.uv);
  auto color    = material->color *
               xyz(eval_texture(material->color_tex, texcoord));

  // -----------------------------------------------------------------------------
  // the output vector is accumulating color
  // -----------------------------------------------------------------------------
  auto radiance_output = color;

  // taking care of lines
  if (!instance->shape->lines.empty()) {
    // Tangent lines
    normal = orthonormalize(outgoing, normal);
  } else if (!instance->shape->triangles.empty()) {
    // Flip the normal
    if (dot(outgoing, normal) < 0) normal = -normal;
  }

  // -----------------------------------------------------------------------------
  // here is what i made, i'm multipling R and G channels with a random number.
  // B is excluded to give a warm effect
  // -----------------------------------------------------------------------------
  float rand_n = rand1f(rng);
  radiance_output *= {rand_n, (1 - rand_n), 0.0};

  // exit of recursive call
  if (bounce >= params.bounces) {
    return {radiance_output.x, radiance_output.y, radiance_output.z, 1};
  }

  // incoming
  auto incoming = sample_hemisphere(normal, rand2f(rng));

  // -----------------------------------------------------------------------------
  // here is where i merge the toon effect with the warm lamp idea
  // -----------------------------------------------------------------------------

  // ised to understand if there's shadow
  // basically i check if there's intersection with the light position i will
  // define in the get_toon_effect

  vec3f light_pos_wrld = {10.0, 16.0, 6.0};

  auto  inshadow_isec  = intersect_scene_bvh(scene, {position, light_pos_wrld});
  // 0 if point is inshadow state
  auto inshadow = inshadow_isec.hit ? 0 : 1;

  // TO GET A PURE WARM LAMP EFFECT COMMENT THIS LINE 990
  // if this is COMMENTED the result should be considerd TOON_OF
  // if active the result is TOON_ON
  // -----------------------------------------------------------------------------
  radiance_output *= get_toon_effect(
      incoming, color, normal, outgoing, inshadow, light_pos_wrld);
  // -----------------------------------------------------------------------------

  radiance_output +=
      (pif) * (color)*abs(dot(normal, incoming)) *
      xyz(shade_myshader(scene, {position, incoming}, bounce + 1, rng, params));

  return get_vector_4f(radiance_output);
}

// Raytrace renderer.
static vec4f shade_refract(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  // -----------------------------------------------------------------------------
  // this call is used to test refracting behaviour of not thin surface
  // it's the same script as raytrace but in this case i'm activating refract if
  // !material->thin
  // -----------------------------------------------------------------------------

  auto isec = intersect_scene_bvh(scene, ray);
  if (!isec.hit) {
    auto env = eval_environment(scene, ray);
    return vec4f{env.x, env.y, env.z, 1};
  }

  // Getting geometries
  auto instance = scene->instances[isec.instance];

  auto shape    = instance->shape;
  auto material = instance->material;

  auto position = transform_point(
      instance->frame, eval_position(shape, isec.element, isec.uv));
  auto normal = transform_direction(
      instance->frame, eval_normal(shape, isec.element, isec.uv));
  auto outgoing = -ray.d;

  // used in refraction for non thin material
  auto temp = dot(outgoing, normal);

  if (!instance->shape->lines.empty()) {
    // Tangent lines
    normal = orthonormalize(outgoing, normal);
  } else if (!instance->shape->triangles.empty()) {
    // Flip the normal
    if (dot(outgoing, normal) < 0) normal = -normal;
  }

  // Properties
  auto texcoord = eval_texcoord(shape, isec.element, isec.uv);
  auto color    = material->color *
               xyz(eval_texture(material->color_tex, texcoord));
  auto emission = material->emission *
                  xyz(eval_texture(material->emission_tex, texcoord));
  auto specular = material->specular *
                  xyz(eval_texture(material->specular_tex, texcoord)).x;
  auto metallic = material->metallic *
                  (eval_texture(material->metallic_tex, texcoord)).x;
  auto roughness = material->roughness *
                   (eval_texture(material->roughness_tex, texcoord)).x;

  // Professor Hint
  roughness = pow(roughness, 2);

  auto transmission = material->transmission *
                      (eval_texture(material->transmission_tex, texcoord)).x;

  // Hadling opcity
  auto opacity = material->opacity;
  if (material->opacity_tex)
    opacity = opacity * (eval_texture(material->opacity_tex, texcoord)).x;

  auto radiance_output = emission;

  if (bounce >= params.bounces) {
    return {radiance_output.x, radiance_output.y, radiance_output.z, 1};
  }

  // opcity check
  if (rand1f(rng) > opacity) {
    auto incoming = -outgoing;
    // recursive call
    return shade_refract(scene, {position, incoming}, bounce + 1, rng, params);
  }

  if (transmission) {
    // -----------------------------------------------------------------------------
    // the vert critical difference is here, if material is not thin we apply
    // the refracting basically this is the same script as shade_raytrace, but
    // with !material->thin down here the recursive calls are called to
    // shade_refract
    // -----------------------------------------------------------------------------

    if (!material->thin) {
      // do not refract
      if (rand1f(rng) <
          mean(fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing))) {
        //
        auto incoming = reflect(outgoing, normal);
        radiance_output += xyz(shade_refract(
            scene, {position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = -outgoing;
        radiance_output += color *
                           xyz(shade_refract(scene, {position, incoming},
                               bounce + 1, rng, params));
      }
    } else {
      // for fat materials
      // check shade_refract

      // index of refraction
      auto ior = reflectivity_to_eta(vec3f{0.04, 0.04, 0.04});
      auto eta = 1.f / mean(ior);

      if (rand1f(rng) <
          // random picking
          mean(fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing))) {
        
        auto incoming = reflect(outgoing, normal);
        radiance_output += xyz(shade_refract(
            scene, {position, incoming}, bounce + 1, rng, params));
      } else {
        // else random picking
        auto incoming = temp < 0 ? refract(outgoing, normal, 1 / eta)
                                 : refract(outgoing, normal, eta);
        radiance_output += color *
                           xyz(shade_refract(scene, {position, incoming},
                               bounce + 1, rng, params));
      }
    }
    //
    // *** POLISHED METALS ***
    //
  } else if (metallic && !roughness) {
    auto incoming = reflect(outgoing, normal);
    // fresnel_schlick models the chage of color
    auto fs = fresnel_schlick(color, normal, outgoing);
    radiance_output += fs * xyz(shade_refract(scene, {position, incoming},
                                bounce + 1, rng, params));
  } else if (metallic && roughness) {
    // direction coming inside
    auto incoming = sample_hemisphere(normal, rand2f(rng));
    auto halfway  = normalize(outgoing + incoming);
    auto fs       = fresnel_schlick(color, halfway, outgoing);

    auto micro_distribution = microfacet_distribution(
        roughness, normal, halfway);

    auto micro_shadow = microfacet_shadowing(
        roughness, normal, halfway, outgoing, incoming);

    auto den_term = 4 * abs(dot(normal, incoming)) * abs(dot(normal, outgoing));
    // Each summation term
    auto temp_term = fs * micro_distribution * micro_shadow / den_term;

    // Computing finally the lighting (recursively)
    radiance_output += (2 * pif) * abs(dot(normal, incoming)) * temp_term *
                       xyz(shade_refract(scene, {position, incoming},
                           bounce + 1, rng, params));
  } else if (specular) {
    //
    // *** ROUGH PLASTIC ***
    //
    auto incoming = sample_hemisphere(normal, rand2f(rng));
    auto halfway  = normalize(outgoing + incoming);
    auto fs       = fresnel_schlick({0.04, 0.04, 0.04}, halfway, outgoing);

    // same as before
    auto micro_distribution = microfacet_distribution(
        roughness, normal, halfway);
    auto micro_shadow = microfacet_shadowing(
        roughness, normal, halfway, outgoing, incoming);

    auto den_term = 4 * abs(dot(normal, incoming)) * abs(dot(normal, outgoing));

    auto temp_term = fs * micro_distribution * micro_shadow / den_term;

    // recursive call to compute light
    radiance_output += (2 * pif) * (color / pif * (1 - fs) + temp_term) *
                       abs(dot(normal, incoming)) *
                       xyz(shade_refract(scene, {position, incoming},
                           bounce + 1, rng, params));

  } else {
    auto incoming = sample_hemisphere(normal, rand2f(rng));

    radiance_output += (2 * pif) * (color / pif) * abs(dot(normal, incoming)) *
                       xyz(shade_refract(scene, {position, incoming},
                           bounce + 1, rng, params));
  }

  return vec4f{radiance_output.x, radiance_output.y, radiance_output.z, 1.0f};
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const raytrace_scene* scene,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    // -----------------------------------------------------------------------------
    // here i added the two new shaders,the one to test my toon_warm_lampshader
    // the second to test the refracting behaviour
    // .h file has been modified as well
    // -----------------------------------------------------------------------------
    case raytrace_shader_type::myshader: return shade_myshader;
    case raytrace_shader_type::refractor: return shade_refract;

    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Trace a block of samples
void render_sample(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const vec2i& ij,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE - DONE
  return render_samples(state, scene, camera, params);
}

// Init a sequence of random number generators.
void init_state(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const raytrace_params& params) {
  auto image_size =
      (camera->film.x > camera->film.y)
          ? vec2i{params.resolution,
                (int)round(params.resolution * camera->film.y / camera->film.x)}
          : vec2i{
                (int)round(params.resolution * camera->film.x / camera->film.y),
                params.resolution};
  state->render.assign(image_size, zero4f);
  state->accumulation.assign(image_size, zero4f);
  state->samples.assign(image_size, 0);
  state->rngs.assign(image_size, {});
  auto init_rng = make_rng(1301081);
  for (auto& rng : state->rngs) {
    rng = make_rng(params.seed, rand1i(init_rng, 1 << 31) / 2 + 1);
  }
}

// Progressively compute an image by calling trace_samples multiple times.
void render_samples(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const raytrace_params& params) {
  // detect which shader
  auto shader = get_shader(params);

  if (!params.noparallel) {
    // -----------------------------------------------------------------------------
    // this is activating when NOPARALLEL IS FALSE
    // -----------------------------------------------------------------------------

    for (auto j = 0; j < state->render.imsize().y; j++) {
      for (auto i = 0; i < state->render.imsize().x; i++) {
        // take one pixel
        auto puv = rand2f(state->rngs[i, j]);

        //
        auto p  = vec2f{(float)i, (float)j} + puv;
        auto uv = vec2f{
            p.x / state->render.imsize().x, p.y / state->render.imsize().y};

        // generate ray
        auto ray = eval_camera(camera, uv);
        // call shader img
        auto color = shader(scene, ray, 0, (state->rngs[i, j]), params);
        //
        // clamp to max value
        // normalize color field in the appropriate ranges
        // params.clamp
        //
        if (length(xyz(color)) > params.clamp)
          color = normalize(color) * params.clamp;

        //
        // update OUTPUT
        //
        // update accumulation, samples and render
        //

        state->accumulation[{i, j}] += color;
        state->samples[{i, j}] += 1;

        // update output image
        state->render[{i, j}] = (state->accumulation[{i, j}]) /
                                (state->samples[{i, j}]);
      };
    }

  } else {
    // -----------------------------------------------------------------------------
    // noparallel = TRUE
    // -----------------------------------------------------------------------------

    parallel_for(state->render.imsize().x, state->render.imsize().y,
        [state, scene, camera, shader, &params](int i, int j) {
          // obtain pixel coordinates
          auto statey = state->render.imsize().y;
          auto statex = state->render.imsize().x;

          auto v_ij = vec2i{i, j};

          // randm number
          auto puv = rand2f(state->rngs[v_ij]);
          // 
          auto d   = vec2f{(float)i, (float)j} + puv;

          // pixel coordinates 
          auto uv = vec2f{d.x / statex, d.y / statey};

          // as before ray and color obtained from shader
          auto ray   = eval_camera(camera, uv);
          auto color = shader(scene, ray, 0, state->rngs[i, j], params);

          //
          // clamp to max value
          // normalize color field in the appropriate ranges
          // params.clamp
          //

          if (length(xyz(color)) > params.clamp) {
            color = normalize(color) * params.clamp;
          }

          //
          // update OUTPUT
          //
          // update accumulation, samples and render
          //

          state->accumulation[v_ij] += color;
          state->samples[v_ij] += 1;
          state->render[v_ij] = (state->accumulation[v_ij]) /
                                (state->samples[v_ij]);
        });
  }
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// SCENE CREATION
// -----------------------------------------------------------------------------
namespace yocto {

// cleanup
raytrace_shape::~raytrace_shape() {
  if (bvh) delete bvh;
}

// cleanup
raytrace_scene::~raytrace_scene() {
  if (bvh) delete bvh;
  for (auto camera : cameras) delete camera;
  for (auto instance : instances) delete instance;
  for (auto shape : shapes) delete shape;
  for (auto material : materials) delete material;
  for (auto texture : textures) delete texture;
  for (auto environment : environments) delete environment;
}

// Add element
raytrace_camera* add_camera(raytrace_scene* scene) {
  return scene->cameras.emplace_back(new raytrace_camera{});
}
raytrace_texture* add_texture(raytrace_scene* scene) {
  return scene->textures.emplace_back(new raytrace_texture{});
}
raytrace_shape* add_shape(raytrace_scene* scene) {
  return scene->shapes.emplace_back(new raytrace_shape{});
}
raytrace_material* add_material(raytrace_scene* scene) {
  return scene->materials.emplace_back(new raytrace_material{});
}
raytrace_instance* add_instance(raytrace_scene* scene) {
  return scene->instances.emplace_back(new raytrace_instance{});
}
raytrace_environment* add_environment(raytrace_scene* scene) {
  return scene->environments.emplace_back(new raytrace_environment{});
}

// Set cameras
void set_frame(raytrace_camera* camera, const frame3f& frame) {
  camera->frame = frame;
}
void set_lens(raytrace_camera* camera, float lens, float aspect, float film) {
  camera->lens = lens;
  camera->film = aspect >= 1 ? vec2f{film, film / aspect}
                             : vec2f{film * aspect, film};
}
void set_focus(raytrace_camera* camera, float aperture, float focus) {
  camera->aperture = aperture;
  camera->focus    = focus;
}

// Add texture
void set_texture(raytrace_texture* texture, const image<vec4b>& img) {
  texture->ldr = img;
  texture->hdr = {};
}
void set_texture(raytrace_texture* texture, const image<vec4f>& img) {
  texture->ldr = {};
  texture->hdr = img;
}

// Add shape
void set_points(raytrace_shape* shape, const vector<int>& points) {
  shape->points = points;
}
void set_lines(raytrace_shape* shape, const vector<vec2i>& lines) {
  shape->lines = lines;
}
void set_triangles(raytrace_shape* shape, const vector<vec3i>& triangles) {
  shape->triangles = triangles;
}
void set_positions(raytrace_shape* shape, const vector<vec3f>& positions) {
  shape->positions = positions;
}
void set_normals(raytrace_shape* shape, const vector<vec3f>& normals) {
  shape->normals = normals;
}
void set_texcoords(raytrace_shape* shape, const vector<vec2f>& texcoords) {
  shape->texcoords = texcoords;
}
void set_radius(raytrace_shape* shape, const vector<float>& radius) {
  shape->radius = radius;
}

// Add instance
void set_frame(raytrace_instance* instance, const frame3f& frame) {
  instance->frame = frame;
}
void set_shape(raytrace_instance* instance, raytrace_shape* shape) {
  instance->shape = shape;
}
void set_material(raytrace_instance* instance, raytrace_material* material) {
  instance->material = material;
}

// Add material
void set_emission(raytrace_material* material, const vec3f& emission,
    raytrace_texture* emission_tex) {
  material->emission     = emission;
  material->emission_tex = emission_tex;
}
void set_color(raytrace_material* material, const vec3f& color,
    raytrace_texture* color_tex) {
  material->color     = color;
  material->color_tex = color_tex;
}
void set_specular(raytrace_material* material, float specular,
    raytrace_texture* specular_tex) {
  material->specular     = specular;
  material->specular_tex = specular_tex;
}
void set_metallic(raytrace_material* material, float metallic,
    raytrace_texture* metallic_tex) {
  material->metallic     = metallic;
  material->metallic_tex = metallic_tex;
}
void set_ior(raytrace_material* material, float ior) { material->ior = ior; }
void set_transmission(raytrace_material* material, float transmission,
    bool thin, float trdepth, raytrace_texture* transmission_tex) {
  material->transmission     = transmission;
  material->thin             = thin;
  material->trdepth          = trdepth;
  material->transmission_tex = transmission_tex;
}
void set_thin(raytrace_material* material, bool thin) { material->thin = thin; }
void set_roughness(raytrace_material* material, float roughness,
    raytrace_texture* roughness_tex) {
  material->roughness     = roughness;
  material->roughness_tex = roughness_tex;
}
void set_opacity(
    raytrace_material* material, float opacity, raytrace_texture* opacity_tex) {
  material->opacity     = opacity;
  material->opacity_tex = opacity_tex;
}
void set_scattering(raytrace_material* material, const vec3f& scattering,
    float scanisotropy, raytrace_texture* scattering_tex) {
  material->scattering     = scattering;
  material->scanisotropy   = scanisotropy;
  material->scattering_tex = scattering_tex;
}

// Add environment
void set_frame(raytrace_environment* environment, const frame3f& frame) {
  environment->frame = frame;
}
void set_emission(raytrace_environment* environment, const vec3f& emission,
    raytrace_texture* emission_tex) {
  environment->emission     = emission;
  environment->emission_tex = emission_tex;
}

}  // namespace yocto