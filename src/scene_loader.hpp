#pragma once
#include <string>
#include "world.hpp"

class SceneLoader {
public:
    static void LoadObj(Model& model, const std::string& filename);
    static void LoadSceneTxt(Model& model, const std::string& filename);
};

