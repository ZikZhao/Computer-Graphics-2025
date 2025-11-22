#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include "SDL.h"

class DrawingWindow {

public:
	size_t width;
	size_t height;

private:
	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_Texture *texture;
	std::vector<uint32_t> pixelBuffer;

public:
	DrawingWindow() noexcept = delete;
	DrawingWindow(int w, int h, bool fullscreen) noexcept;
	void renderFrame() noexcept;
	void savePPM(const std::string &filename) const;
	void saveBMP(const std::string &filename) const;
	bool pollForInputEvents(SDL_Event &event) noexcept;
	void exitCleanly() noexcept;
	void setPixelColour(size_t x, size_t y, uint32_t colour) noexcept;
	uint32_t getPixelColour(size_t x, size_t y) noexcept;
	void clearPixels() noexcept;
};

void printMessageAndQuit(const std::string &message, const char *error) noexcept;
