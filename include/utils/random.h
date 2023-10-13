#pragma once

#include <random>
#include <glad/glad.h>

// Returns a float between 0 and 1
GLfloat rand_float()
{
    std::mt19937 s_RandomEngine;
	std::uniform_int_distribution<std::mt19937::result_type> s_Distribution;
    s_RandomEngine.seed(std::random_device()());
    GLfloat r = (float)s_Distribution(s_RandomEngine) / (float)std::numeric_limits<uint32_t>::max();
    return r;
}