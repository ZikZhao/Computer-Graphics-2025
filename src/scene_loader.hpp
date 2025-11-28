#pragma once
#include <string>

#include "world.hpp"

/**
 * @brief Utility for loading OBJ and text-based scene descriptions into `Model`.
 */
class SceneLoader {
public:
    /**
     * @brief Loads an OBJ file into the provided model.
     * @param model Destination model to populate.
     * @param filename Path to the OBJ file.
     */
    static void LoadObj(Model& model, const std::string& filename);

    /**
     * @brief Loads a text scene file into the provided model.
     * @param model Destination model to populate.
     * @param filename Path to the scene text file.
     */
    static void LoadSceneTxt(Model& model, const std::string& filename);
};
