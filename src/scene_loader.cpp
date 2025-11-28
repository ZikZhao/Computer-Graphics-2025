#include "scene_loader.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <sstream>

#define STB_IMAGE_IMPLEMENTATION
#include "../libs/stb_image.h"

namespace {
struct IndexTriple {
    int v;
    int vt;
    int vn;
};
static IndexTriple parse_index_token(const std::string& t) {
    IndexTriple idx{-1, -1, -1};
    if (t.find('/') != std::string::npos) {
        std::vector<std::string> parts;
        std::stringstream ss(t);
        std::string p;
        while (std::getline(ss, p, '/')) parts.push_back(p);
        if (!parts.empty() && !parts[0].empty()) idx.v = std::stoi(parts[0]);
        if (parts.size() >= 2 && !parts[1].empty()) idx.vt = std::stoi(parts[1]);
        if (parts.size() >= 3 && !parts[2].empty()) idx.vn = std::stoi(parts[2]);
    } else {
        bool digits_only = !t.empty() && std::all_of(t.begin(), t.end(), [](char c) { return c >= '0' && c <= '9'; });
        if (digits_only) idx.v = std::stoi(t);
    }
    return idx;
}
}  // namespace

void SceneLoader::LoadObj(Model& model, const std::string& filename) {
    // OBJ loader: parse geometry, materials and per-object settings
    // Stream the file and dispatch each directive to update the Model
    auto current_obj = model.objects_.end();
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "mtllib") {
            // Material library — preload materials referenced by objects
            std::string relative_path;
            iss >> relative_path;
            std::string material_filename = (std::filesystem::path(filename).parent_path() / relative_path).string();
            model.load_materials(std::move(material_filename));
        } else if (type == "o") {
            // Object boundary — new mesh group with its own material/shading state
            std::string name;
            iss >> name;
            model.objects_.emplace_back(name);
            current_obj = std::prev(model.objects_.end());
        } else if (type == "usemtl") {
            // Bind material to current object; preserve shading mode
            assert(current_obj != model.objects_.end());
            std::string colour_name;
            iss >> colour_name;
            assert(model.materials_.find(colour_name) != model.materials_.end());
            auto prev_shading = current_obj->material.shading;
            current_obj->material = model.materials_[colour_name];
            current_obj->material.shading = prev_shading;
        } else if (type == "shading" || type == "Shading") {
            // Select shading model (Flat/Gouraud/Phong)
            std::string mode;
            iss >> mode;
            if (current_obj != model.objects_.end()) {
                if (mode == "Flat") {
                    current_obj->material.shading = Material::Shading::FLAT;
                } else if (mode == "Gouraud") {
                    current_obj->material.shading = Material::Shading::GOURAUD;
                } else if (mode == "Phong") {
                    current_obj->material.shading = Material::Shading::PHONG;
                }
            }
        } else if (type == "v") {
            // Vertex position
            assert(current_obj != model.objects_.end());
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertices_.emplace_back(x, y, z);
        } else if (type == "vt") {
            // Texture coordinate
            FloatType u, v;
            iss >> u >> v;
            model.texture_coords_.emplace_back(u, v);
        } else if (type == "vn") {
            // Vertex normal (normalized)
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertex_normals_.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "f") {
            // Triangle face — parse indices (v/vt/vn) and create Face with material
            assert(current_obj != model.objects_.end());
            glm::vec3 vertice[3];
            std::uint32_t vt_indices[3];
            int vi_idx[3];
            int normal_indices[3];
            bool has_normals = false;
            for (int i = 0; i < 3; i++) {
                int vertex_index;
                char slash;
                iss >> vertex_index >> slash;
                vertice[i] = model.vertices_[vertex_index - 1];
                vi_idx[i] = vertex_index - 1;
                vt_indices[i] = std::numeric_limits<std::uint32_t>::max();
                normal_indices[i] = -1;
                if (int c = iss.peek(); c >= '0' && c <= '9') {
                    int tex_idx;
                    iss >> tex_idx;
                    if (tex_idx > 0) {
                        vt_indices[i] = static_cast<std::uint32_t>(tex_idx - 1);
                    }
                }
                if (iss.peek() == '/') {
                    iss >> slash;
                    if (int c = iss.peek(); c >= '0' && c <= '9') {
                        int normal_idx;
                        iss >> normal_idx;
                        normal_indices[i] = normal_idx - 1;
                        has_normals = true;
                    }
                }
            }
            Face new_face{
                .v_indices =
                    {static_cast<std::uint32_t>(vi_idx[0]),
                     static_cast<std::uint32_t>(vi_idx[1]),
                     static_cast<std::uint32_t>(vi_idx[2])},
                .vt_indices = {vt_indices[0], vt_indices[1], vt_indices[2]},
                .vn_indices =
                    {(has_normals && normal_indices[0] >= 0) ? static_cast<std::uint32_t>(normal_indices[0])
                                                             : std::numeric_limits<std::uint32_t>::max(),
                     (has_normals && normal_indices[1] >= 0) ? static_cast<std::uint32_t>(normal_indices[1])
                                                             : std::numeric_limits<std::uint32_t>::max(),
                     (has_normals && normal_indices[2] >= 0) ? static_cast<std::uint32_t>(normal_indices[2])
                                                             : std::numeric_limits<std::uint32_t>::max()},
                .material = current_obj->material,
                .face_normal = glm::vec3(0.0f)};
            current_obj->faces.emplace_back(std::move(new_face));
        }
    }
    // Finalize — compute geometric face normals and cache faces per model
    model.compute_face_normals();
    model.cache_faces();
}

void SceneLoader::LoadSceneTxt(Model& model, const std::string& filename) {
    // Text scene loader: environment directive, materials, object blocks and embedded geometry
    auto current_obj = model.objects_.end();
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    auto current_material = model.materials_.end();
    int vertex_offset = 0;
    int tex_offset = 0;
    int normal_offset = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "Environ") {
            // Environment map hint (handled at World level)
            std::string hdr;
            iss >> hdr;
        } else if (type == "Include") {
            // Include another scene file (relative path)
            std::string rel;
            iss >> rel;
            if (!rel.empty()) {
                std::string inc_path = (std::filesystem::path(filename).parent_path() / rel).string();
                LoadSceneTxt(model, inc_path);
            }
        } else if (type == "Material") {
            // Begin a new material block
            std::string name;
            iss >> name;
            current_material = model.materials_.emplace(name, Material{}).first;
        } else if (type == "Colour") {
            // Albedo (diffuse base color)
            if (current_material != model.materials_.end()) {
                FloatType r, g, b;
                iss >> r >> g >> b;
                current_material->second.base_color = glm::vec3(r, g, b);
            }
        } else if (type == "Metallic") {
            // Metallic factor for specular reflection weighting
            if (current_material != model.materials_.end()) {
                FloatType m;
                iss >> m;
                current_material->second.metallic = std::clamp(m, 0.0f, 1.0f);
            }
        } else if (type == "IOR") {
            // Index of refraction for dielectrics
            if (current_material != model.materials_.end()) {
                FloatType i;
                iss >> i;
                current_material->second.ior = std::max(1.0f, i);
            }
        } else if (type == "AtDistance") {
            // Absorption length td; used to derive sigma_a when not provided
            if (current_material != model.materials_.end()) {
                FloatType td;
                iss >> td;
                current_material->second.td = std::max(0.0f, td);
            }
        } else if (type == "TransimissionWeight") {
            // Transmission weight tw for mixing reflection/refraction
            if (current_material != model.materials_.end()) {
                FloatType tw;
                iss >> tw;
                current_material->second.tw = std::clamp(tw, 0.0f, 1.0f);
            }
        } else if (type == "Emission") {
            // Emissive color (area lights)
            if (current_material != model.materials_.end()) {
                FloatType r, g, b;
                iss >> r >> g >> b;
                current_material->second.emission = glm::vec3(std::max(0.0f, r), std::max(0.0f, g), std::max(0.0f, b));
            }
        } else if (type == "Texture") {
            // Bind a PPM texture to the material
            if (current_material != model.materials_.end()) {
                std::string tex_name;
                iss >> tex_name;
                std::string texture_filename = (std::filesystem::path(filename).parent_path() / tex_name).string();
                current_material->second.texture = std::make_shared<Texture>(model.load_texture(texture_filename));
            }
        } else if (type == "Object") {
            // Begin an object block; record offsets for local indexing within this file
            std::string rest;
            std::getline(iss, rest);
            std::string name = rest;
            if (!name.empty() && name[0] == '#') {
                name.erase(0, 1);
            }
            while (!name.empty() && (name[0] == ' ' || name[0] == '\t')) name.erase(0, 1);
            model.objects_.emplace_back(name);
            current_obj = std::prev(model.objects_.end());
            vertex_offset = static_cast<int>(model.vertices_.size());
            tex_offset = static_cast<int>(model.texture_coords_.size());
            normal_offset = static_cast<int>(model.vertex_normals_.size());
        } else if (type == "Use") {
            // Assign current material to the object (preserving shading)
            if (current_obj != model.objects_.end()) {
                std::string mat_name;
                iss >> mat_name;
                auto it = model.materials_.find(mat_name);
                if (it != model.materials_.end()) {
                    auto prev_shading = current_obj->material.shading;
                    current_obj->material = it->second;
                    current_obj->material.shading = prev_shading;
                }
            }
        } else if (type == "Shading") {
            // Select shading model
            if (current_obj != model.objects_.end()) {
                std::string mode;
                iss >> mode;
                if (mode == "Flat") {
                    current_obj->material.shading = Material::Shading::FLAT;
                } else if (mode == "Gouraud") {
                    current_obj->material.shading = Material::Shading::GOURAUD;
                } else if (mode == "Phong") {
                    current_obj->material.shading = Material::Shading::PHONG;
                }
            }
        } else if (type == "Vertex") {
            // Vertex position with optional inline normal ("x y z / nx ny nz")
            if (current_obj != model.objects_.end()) {
                FloatType x, y, z;
                iss >> x >> y >> z;
                model.vertices_.emplace_back(x, y, z);
                model.vertex_normals_by_vertex_.emplace_back(glm::vec3(0.0f));
                char c;
                if (iss >> c) {
                    if (c == '/') {
                        FloatType xn, yn, zn;
                        iss >> xn >> yn >> zn;
                        model.vertex_normals_by_vertex_.back() = glm::normalize(glm::vec3(xn, yn, zn));
                    } else {
                        iss.putback(c);
                    }
                }
            }
        } else if (type == "TextureCoord") {
            // UV coordinate
            FloatType u, v;
            iss >> u >> v;
            model.texture_coords_.emplace_back(u, v);
        } else if (type == "Normal") {
            // Vertex normal (normalized)
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertex_normals_.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "Face") {
            // Triangle face — parse tokenized indices allowing local offsets, build a Face
            if (current_obj == model.objects_.end()) continue;
            std::string tok[3];
            iss >> tok[0] >> tok[1] >> tok[2];
            glm::vec3 vpos[3];
            std::uint32_t vt_out[3] = {
                std::numeric_limits<std::uint32_t>::max(),
                std::numeric_limits<std::uint32_t>::max(),
                std::numeric_limits<std::uint32_t>::max()};
            int vi_out[3] = {0, 0, 0};
            int vn_out[3] = {-1, -1, -1};
            bool has_vn = false;
            for (int i = 0; i < 3; ++i) {
                const std::string& t = tok[i];
                IndexTriple tri = parse_index_token(t);
                int v_i = tri.v;
                int vt_i = tri.vt;
                int vn_i = tri.vn;
                if (vn_i >= 0) has_vn = true;
                int v_global = vertex_offset + (v_i >= 0 ? v_i : 0);
                vpos[i] = model.vertices_[v_global];
                vi_out[i] = v_global;
                if (vt_i >= 0) {
                    int vt_global = tex_offset + vt_i;
                    vt_out[i] = static_cast<std::uint32_t>(vt_global);
                }
                if (vn_i >= 0) {
                    int vn_global = normal_offset + vn_i;
                    vn_out[i] = vn_global;
                }
            }
            // If normals are absent, defer to per-vertex cached normals when available; mark missing as -1
            if (!has_vn) {
                for (int i = 0; i < 3; ++i) {
                    int vidx = vi_out[i];
                    if (vidx >= 0 && static_cast<std::size_t>(vidx) < model.vertex_normals_by_vertex_.size()) {
                        if (glm::length(model.vertex_normals_by_vertex_[vidx]) > 0.001f) {
                            vn_out[i] = -1;
                        }
                    }
                }
                for (int i = 0; i < 3; ++i) {
                    if (vn_out[i] < 0) {
                        vn_out[i] = -1;
                    }
                }
            }
            Face new_face{
                .v_indices =
                    {static_cast<std::uint32_t>(vi_out[0]),
                     static_cast<std::uint32_t>(vi_out[1]),
                     static_cast<std::uint32_t>(vi_out[2])},
                .vt_indices = {vt_out[0], vt_out[1], vt_out[2]},
                .vn_indices =
                    {vn_out[0] >= 0 ? static_cast<std::uint32_t>(vn_out[0]) : std::numeric_limits<std::uint32_t>::max(),
                     vn_out[1] >= 0 ? static_cast<std::uint32_t>(vn_out[1]) : std::numeric_limits<std::uint32_t>::max(),
                     vn_out[2] >= 0 ? static_cast<std::uint32_t>(vn_out[2])
                                    : std::numeric_limits<std::uint32_t>::max()},
                .material = current_obj->material,
                .face_normal = glm::vec3(0.0f)};
            model.objects_.back().faces.emplace_back(std::move(new_face));
        }
    }
    // Finalize — compute face normals and cache flattened face list
    model.compute_face_normals();
    model.cache_faces();
}
