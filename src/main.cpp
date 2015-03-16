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
#include <Eigen/Dense>

/*
 * Internal Includes
 */
#include "ImageRGB.hpp"
#include "ioJPG.hpp"
#include "exif.h"
#include "functionsHDR.hpp"
#include "tools.hpp"

/*
 * Namespaces
 */
using namespace kn;
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // check arguments
  if(argc < 3){
    std::cerr << "usage : " << argv[0] << " image_1.jpg ... image_n.jpg" << std::endl;
    std::cerr << "or    : " << argv[0] << " dirname/*.jpg" << std::endl;
    exit(0); 
  }

  // load images and exposure time
  std::vector<ImageRGB8u> images;
  std::vector<double> exposure;
  loadImages(argc, argv, images, exposure);

  //////////////////////////////////////////////////////////////////////////////////////
  //Notre code

  //choix des N pixels pris sur chaque image
  std::vector <Vector2i> pixels;
  uint N = 100; //nombre de pixles qu'on veut prendre en compte.

  //on vérifie que l'image est assez grande pour en prendre 100
  if (N > (uint) (images[0].width() * images[0].height()))
    N = (uint) (images[0].width() * images[0].height());

  //on vérifie que P*N > 256 pour la robustesse de l'algorithme
  if(N * images.size() < 256)
  {
    std::cout << "Il n'y a pas assez d'images ou les images sont trop petites pour utiliser cet algorithme." << std::endl;
    exit(1);
  }

  //choix des pixels aléatoirement
  pixels = pickPixels(N, images[0].width(), images[0].height());

  std::cout << "Copying Image ... ";

  //On recopie les images dans des matrices Eigen pour chaque canal
  std::vector <MatrixXi> imagesMatR;
  std::vector <MatrixXi> imagesMatG;
  std::vector <MatrixXi> imagesMatB;
  std::vector <MatrixXi> imagesMatNB;

  imagesMatR = recopieImages(images, 0);
  imagesMatG = recopieImages(images, 1);
  imagesMatB = recopieImages(images, 2);
  imagesMatNB= recopieImagesNB(images);

  std::cout << "done" << std::endl << std::endl;
  std::cout << "Response Recovery ... ";

  // On retrouve la fonction inverse de la courbe de réponse des photosites
  VectorXd gR = responseRecovery(imagesMatR, exposure, pixels, 0, 255, 1000.00);
  VectorXd gG = responseRecovery(imagesMatG, exposure, pixels, 0, 255, 1000.00);
  VectorXd gB = responseRecovery(imagesMatB, exposure, pixels, 0, 255, 1000.00);
  VectorXd gNB = responseRecovery(imagesMatNB, exposure, pixels, 0, 255, 1000.00);

  std::cout << "done" << std::endl << std::endl;
  std::cout << "Compute Radiance Map ... ";

  // On calcule l'image d'irradiance
  MatrixXd imageHDR_R = computeRadianceMap (imagesMatR, gR, exposure, 0, 255);
  MatrixXd imageHDR_G = computeRadianceMap (imagesMatG, gG, exposure, 0, 255);
  MatrixXd imageHDR_B = computeRadianceMap (imagesMatB, gB, exposure, 0, 255);
  MatrixXd imageHDR_NB = computeRadianceMap (imagesMatNB, gNB, exposure, 0, 255);

  std::cout << "done" << std::endl << std::endl;
  std::cout << "Tone Mapping ... ";

  // On fait un tone mapping sur l'image HDR
  MatrixXi imageInterpoleR = toneMapping(imageHDR_R, 0, 255);
  MatrixXi imageInterpoleG = toneMapping(imageHDR_G, 0, 255);
  MatrixXi imageInterpoleB = toneMapping(imageHDR_B, 0, 255);
  MatrixXi imageInterpoleNB = toneMapping(imageHDR_NB, 0, 255);

  std::cout << "done" << std::endl << std::endl;
  std::cout << "Draw Image RGB ... ";

  // On dessine l'image de sortie en couleurs
  ImageRGB8u imageRGB = drawImageRGB(imageInterpoleR, imageInterpoleG, imageInterpoleB);

  std::cout << "done" << std::endl << std::endl;

   std::cout << "Draw Image NB ... ";

  // On dessine l'image de sortie en niveau de gris
  ImageRGB8u imageNB = drawImageNB(imageInterpoleNB);

  std::cout << "done" << std::endl << std::endl;

  //////////////////////////////////////////////////////////////////////////////////////
    
  //sauver l'image
  saveJPG(imageRGB,"output/imageRGB.jpg");
  saveJPG(imageNB,"output/imageNB.jpg");

  return 0;
}



