/*
 * Anti-doublon
 */
#ifndef __TOOLSNINJA__
#define __TOOLSNINJA__

/*
 * External Includes
 */
#include <iostream>
#include <cassert>
#include <cstring>
#include <vector>
#include <ctime>
#include <fstream>
#include <cstdio>

/*
 * Internal Includes
 */
#include "ImageRGB.hpp"
#include "ioJPG.hpp"
#include "exif.h"

/*
 * Namespaces
 */
using namespace kn;
using namespace Eigen;

std::string fileToString(const std::string& filename);

void drawPixels(ImageRGB8u &image, const std::vector<Eigen::Vector2i> &pixels);

void exifParsingError(const int parseSuccess);

void loadImages(const int argc, char **argv, std::vector<ImageRGB8u> &images, std::vector<double> &exposure);

#endif