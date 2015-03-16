/*
 * Anti-doublon
 */
#ifndef __FUNCTIONSNINJA__
#define __FUNCTIONSNINJA__

/*
 * External Includes
 */
#include <iostream>
#include <cassert>
#include <cstring>
#include <vector>
#include <ctime>
#include <fstream>
#include <Eigen/Dense>

/*
 * Internal Includes
 */
#include "ImageRGB.hpp"

/*
 * Namespaces
 */
using namespace Eigen;
using namespace kn;

std::vector <Vector2i> pickPixels(uint nbPixels, int width, int height);

ImageRGB8u drawImageRGB(MatrixXi imageR, MatrixXi imageG, MatrixXi imageB);

ImageRGB8u drawImageNB(MatrixXi imageNB);

double calculeWeight(int z, int zmin, int zmax);

std::vector<MatrixXi> recopieImages( const std::vector <ImageRGB8u> & images, int canal );

std::vector<MatrixXi> recopieImagesNB( const std::vector <ImageRGB8u> & images );

VectorXd responseRecovery ( const std::vector <MatrixXi> & images,
							const std::vector <double> & exposure,
							const std::vector <Vector2i> & pixels,
							const int valueMin,
							const int valueMax,
							const double lambda);

MatrixXd computeRadianceMap (   const std::vector <MatrixXi> & images,
								const VectorXd &g,
								const std::vector <double> & exposure,
								const int valueMin,
								const int valueMax);

MatrixXi toneMapping(  const MatrixXd & imageHDR,
					   const int valueMin,
					   const int valueMax);


#endif