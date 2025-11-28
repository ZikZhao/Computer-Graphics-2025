#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>
#include <filesystem>
#include <sstream>
#include <fstream>
#include "scene_loader.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "../libs/stb_image.h"

namespace {
struct IndexTriple { int v; int vt; int vn; };
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
        bool digits_only = !t.empty() && std::all_of(t.begin(), t.end(), [](char c){ return c >= '0' && c <= '9'; });
        if (digits_only) idx.v = std::stoi(t);
    }
    return idx;
}
}

void SceneLoader::LoadObj(Model& model, const std::string& filename) {
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
            std::string relative_path;
            iss >> relative_path;
            std::string material_filename = (std::filesystem::path(filename).parent_path() / relative_path).string();
            model.load_materials(std::move(material_filename));
        } else if (type == "l") {
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.light_position_ = glm::vec3(x, y, z);
            model.has_light_ = true;
        } else if (type == "o") {
            std::string name;
            iss >> name;
            model.objects_.emplace_back(name);
            current_obj = std::prev(model.objects_.end());
        } else if (type == "usemtl") {
            assert(current_obj != model.objects_.end());
            std::string colour_name;
            iss >> colour_name;
            assert(model.materials_.find(colour_name) != model.materials_.end());
            auto prev_shading = current_obj->material.shading;
            current_obj->material = model.materials_[colour_name];
            current_obj->material.shading = prev_shading;
        } else if (type == "shading" || type == "Shading") {
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
            assert(current_obj != model.objects_.end());
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertices_.emplace_back(x, y, z);
        } else if (type == "vt") {
            FloatType u, v;
            iss >> u >> v;
            model.texture_coords_.emplace_back(u, v);
        } else if (type == "vn") {
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertex_normals_.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "f") {
            assert(current_obj != model.objects_.end());
            glm::vec3 vertice[3];
            std::uint8_t tex_indices[3];
            glm::vec2 tex_coords[3];
            int vi_idx[3];
            int normal_indices[3];
            bool has_normals = false;
            for (int i = 0; i < 3; i++) {
                int vertex_index;
                char slash;
                iss >> vertex_index >> slash;
                vertice[i] = model.vertices_[vertex_index - 1];
                vi_idx[i] = vertex_index - 1;
                tex_indices[i] = 0;
                tex_coords[i] = glm::vec2(0.0f, 0.0f);
                normal_indices[i] = -1;
                if (int c = iss.peek(); c >= '0' && c <= '9') {
                    int tex_idx;
                    iss >> tex_idx;
                    tex_indices[i] = tex_idx;
                    if (tex_idx > 0 && static_cast<size_t>(tex_idx) <= model.texture_coords_.size()) {
                        tex_coords[i] = model.texture_coords_[tex_idx - 1];
                    } else {
                        tex_coords[i] = glm::vec2(0.0f, 0.0f);
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
                { vertice[0], vertice[1], vertice[2] },
                { tex_indices[0], tex_indices[1], tex_indices[2] },
                { tex_coords[0], tex_coords[1], tex_coords[2] },
                current_obj->material,
                { vi_idx[0], vi_idx[1], vi_idx[2] },
                { glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f) },
                glm::vec3(0.0f)
            };
            if (has_normals) {
                for (int i = 0; i < 3; i++) {
                    if (normal_indices[i] >= 0 && static_cast<size_t>(normal_indices[i]) < model.vertex_normals_.size()) {
                        new_face.vertex_normals[i] = model.vertex_normals_[normal_indices[i]];
                    }
                }
            }
            current_obj->faces.emplace_back(std::move(new_face));
        }
    }
    model.compute_face_normals();
    model.cache_faces();
}

void SceneLoader::LoadSceneTxt(Model& model, const std::string& filename) {
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
            std::string hdr;
            iss >> hdr;
        } else if (type == "Include") {
            std::string rel;
            iss >> rel;
            if (!rel.empty()) {
                std::string inc_path = (std::filesystem::path(filename).parent_path() / rel).string();
                LoadSceneTxt(model, inc_path);
            }
        } else if (type == "Material") {
            std::string name;
            iss >> name;
            current_material = model.materials_.emplace(name, Material{}).first;
        } else if (type == "Colour") {
            if (current_material != model.materials_.end()) {
                FloatType r, g, b;
                iss >> r >> g >> b;
                current_material->second.base_color = glm::vec3(r, g, b);
            }
        } else if (type == "Metallic") {
            if (current_material != model.materials_.end()) {
                FloatType m;
                iss >> m;
                current_material->second.metallic = std::clamp(m, 0.0f, 1.0f);
            }
        } else if (type == "IOR") {
            if (current_material != model.materials_.end()) {
                FloatType i;
                iss >> i;
                current_material->second.ior = std::max(1.0f, i);
            }
        } else if (type == "AtDistance") {
            if (current_material != model.materials_.end()) {
                FloatType td;
                iss >> td;
                current_material->second.td = std::max(0.0f, td);
            }
        } else if (type == "TransimissionWeight") {
            if (current_material != model.materials_.end()) {
                FloatType tw;
                iss >> tw;
                current_material->second.tw = std::clamp(tw, 0.0f, 1.0f);
            }
        } else if (type == "Emission") {
            if (current_material != model.materials_.end()) {
                FloatType r, g, b;
                iss >> r >> g >> b;
                current_material->second.emission = glm::vec3(std::max(0.0f, r), std::max(0.0f, g), std::max(0.0f, b));
            }
        } else if (type == "Texture") {
            if (current_material != model.materials_.end()) {
                std::string tex_name;
                iss >> tex_name;
                std::string texture_filename = (std::filesystem::path(filename).parent_path() / tex_name).string();
                current_material->second.texture = std::make_shared<Texture>(model.load_texture(texture_filename));
            }
        } else if (type == "Object") {
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
            FloatType u, v;
            iss >> u >> v;
            model.texture_coords_.emplace_back(u, v);
        } else if (type == "Normal") {
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertex_normals_.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "Face") {
            if (current_obj == model.objects_.end()) continue;
            std::string tok[3];
            iss >> tok[0] >> tok[1] >> tok[2];
            glm::vec3 vpos[3];
            std::uint8_t tex_idx_out[3] = {0,0,0};
            glm::vec2 uv_out[3] = {glm::vec2(0.0f), glm::vec2(0.0f), glm::vec2(0.0f)};
            int vi_out[3] = {0,0,0};
            glm::vec3 vnorm_out[3] = {glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f)};
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
                    tex_idx_out[i] = static_cast<std::uint8_t>(vt_i);
                    if (vt_global >= 0 && static_cast<std::size_t>(vt_global) < model.texture_coords_.size()) {
                        uv_out[i] = model.texture_coords_[vt_global];
                    }
                }
                if (vn_i >= 0) {
                    int vn_global = normal_offset + vn_i;
                    if (vn_global >= 0 && static_cast<std::size_t>(vn_global) < model.vertex_normals_.size()) {
                        vnorm_out[i] = model.vertex_normals_[vn_global];
                    }
                }
            }
            if (!has_vn) {
                for (int i = 0; i < 3; ++i) {
                    int vidx = vi_out[i];
                    if (vidx >= 0 && static_cast<std::size_t>(vidx) < model.vertex_normals_by_vertex_.size()) {
                        if (glm::length(model.vertex_normals_by_vertex_[vidx]) > 0.001f) {
                            vnorm_out[i] = model.vertex_normals_by_vertex_[vidx];
                        }
                    }
                }
                glm::vec3 fn = CalculateNormal(vpos[0], vpos[1], vpos[2]);
                for (int i = 0; i < 3; ++i) {
                    if (glm::length(vnorm_out[i]) < 0.001f) {
                        vnorm_out[i] = fn;
                    }
                }
            }
            Face new_face{
                { vpos[0], vpos[1], vpos[2] },
                { tex_idx_out[0], tex_idx_out[1], tex_idx_out[2] },
                { uv_out[0], uv_out[1], uv_out[2] },
                current_obj->material,
                { vi_out[0], vi_out[1], vi_out[2] },
                { vnorm_out[0], vnorm_out[1], vnorm_out[2] },
                glm::vec3(0.0f)
            };
            model.objects_.back().faces.emplace_back(std::move(new_face));
        }
    }
    model.compute_face_normals();
    model.cache_faces();
}

