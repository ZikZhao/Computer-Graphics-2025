#pragma once
#include <string>

class Model;

namespace SceneLoader {
/**
 * @brief Loads an OBJ file into the provided model.
 * @param model Destination model to populate.
 * @param filename Path to the OBJ file.
 */
void LoadObj(Model& model, const std::string& filename);

/**
 * @brief Loads a text scene file into the provided model.
 * @param model Destination model to populate.
 * @param filename Path to the scene text file.
 */
void LoadSceneTxt(Model& model, const std::string& filename);
};  // namespace SceneLoader
