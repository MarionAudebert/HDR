/*
 * External Includes
 */
#include <stdio.h>
#include <cfloat>

/*
 * Internal Includes
 */
#include "functionsHDR.hpp"
#include "mtrand.h"

/*
 * Namespaces
 */
using namespace Eigen;
using namespace kn;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	Fonction qui choisis aléatoirement des pixels dans une image. 
	Elle renvoie un vecteur contenant les coordonnées de ces pixels. 
	Pour choisir aléatoirement les pixels, on utilise l'algorithme
	de Mersenne Twister en initialisant la graine avec le temps.
*/
std::vector <Vector2i> pickPixels(uint nbPixels, int width, int height)
{
	std::vector <Vector2i> pixels;

	//On utilise l'algorithme Mersenne Twister pour choisir les 
	//pixels aléatoirement, en initialisant la graine avec le temps
	MTRand_int32 irand;
	irand.seed(time(0));

	for (uint i = 0; i < nbPixels; ++i)
	{
		Vector2i vec;
		vec(0) = irand() % width;
		vec(1) = irand() % height;
		pixels.push_back(vec);
	}

	return pixels;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	Dessine l'image RGB a partir d'une matrice par canal
*/
ImageRGB8u drawImageRGB(MatrixXi imageR, MatrixXi imageG, MatrixXi imageB)
{
	ImageRGB8u image(imageR.rows(), imageR.cols());
	// On dessine l'image de sortie en couleurs
	for(uint x = 0; x < image.width(); ++x)
	{
		for(uint y = 0; y < image.height(); ++y)
		{
		  image(x,y)[0] = imageR(x,y);  // R
		  image(x,y)[1] = imageG(x,y);  // G
		  image(x,y)[2] = imageB(x,y);  // B
		}
	}

	return image;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	Dessine l'image en niveau de gris a partir d'une matrice
*/
ImageRGB8u drawImageNB(MatrixXi imageNB)
{
	ImageRGB8u image(imageNB.rows(), imageNB.cols());
	// On dessine l'image de sortie en couleurs
	for(uint x = 0; x < image.width(); ++x)
	{
		for(uint y = 0; y < image.height(); ++y)
		{
		  image(x,y)[0] = imageNB(x,y);  // R
		  image(x,y)[1] = imageNB(x,y);  // G
		  image(x,y)[2] = imageNB(x,y);  // B
		}
	}

	return image;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
	Fonction qui renvoie le poid d'un pixel suivant sa valeur en prenant en compte
	les valeurs min et max que peuvent prendre les pixels. Cela permet de donné plus
	de précision aux données en ne prenant pas en compte les valeurs qui sont aux
	extrémités (les pixels très noirs ou très blancs) 
*/
double calculeWeight(int z, int zmin, int zmax)
{
	double zmid = (double) (zmin + zmax) / 2.;

	if ((double) z < zmid)
		return z - zmin;
	else
		return zmax - z;
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	Fonction permettant de recopier l'image de type ImageRGB8u dans une matrice Eigen 
	suivant un canal R, G ou B.
*/
std::vector<MatrixXi> recopieImages( const std::vector <ImageRGB8u> & images, int canal )
{
	std::vector<MatrixXi> imagesMat;
	for (uint img = 0; img < images.size(); ++img)
	{
		MatrixXi mat = MatrixXi::Zero(images[0].width(), images[0].height());
		for (uint i = 0; i < images[0].width(); ++i)
		{
			for (uint j = 0; j < images[0].height(); ++j)
			{
				mat(i,j) = images[img].at(i, j)[canal];
			}
		}
		imagesMat.push_back(mat);
	}
	return imagesMat;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	Ancienne version de la fonction recopieImages qui permettait d'avoir une image en noir et blanc
*/
std::vector<MatrixXi> recopieImagesNB( const std::vector <ImageRGB8u> & images )
{
	std::vector<MatrixXi> imagesMat;
	for (uint img = 0; img < images.size(); ++img)
	{
		MatrixXi mat = MatrixXi::Zero(images[0].width(), images[0].height());
		for (uint i = 0; i < images[0].width(); ++i)
		{
			for (uint j = 0; j < images[0].height(); ++j)
			{
				//Luminance = 0,2126 × Rouge + 0,7152 × Vert + 0,0722 × Bleu.
				mat(i,j) = 0.2126 * images[img].at(i, j)[0] + 0.7152 * images[img].at(i, j)[1] + 0.0722 * images[img].at(i, j)[2];
			}
		}
		imagesMat.push_back(mat);
	}
	return imagesMat;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
	Fonction qui renvoie le vecteur g, correspondant à chaque valeur que peut prendre
	la fonction inverse de la réponse des photosites (domaine de définition = [0-255]). 
	images : correspond à un vector de chaque image sous forme de matrices Eigen
	exposure : vector contenant chaque temps d'exposition pour chaque image
	pixels : vector contenant les coordonnées des pixels choisis
	valueMin : valeur minimum dans l'image
	valueMax : valeur maximum dans l'image
	lambda : variable expérimentale. Après plusieurs test, lambda = 1000 donne de bons résultats
*/
VectorXd responseRecovery ( const std::vector <MatrixXi> & images,
							const std::vector <double> & exposure,
							const std::vector <Vector2i> & pixels,
							const int valueMin,
							const int valueMax,
							const double lambda)
{	
	//nombre de values de 0 à 255
	uint countValues = valueMax - valueMin + 1;

	//nombre de pixels pris pour une image
	uint N = pixels.size();

	//nombre d'image prit en compte
	uint P = images.size();

	//vecteur contenant les g(Zij) qu'on retourne à la fin de la fonction
	VectorXd g = VectorXd::Zero(countValues);

	//Matrice A
	MatrixXd A = MatrixXd::Zero(N*P + countValues - 1, countValues + N);

	//Vecteur x (g(256) ln(Ei))
	VectorXd x = VectorXd::Zero(A.cols());

	//Vecteur b  ln deltaTj
	VectorXd b = VectorXd::Zero(A.rows());

	//pour chaque image 
	for (uint im = 0; im < P; ++im)
	{
		//pour chaque pixels selectionnés
		for (uint pix = 0; pix < N; ++pix)
		{
			//on recupere les coordonnées du pixel qu'on veut 
			uint coordX = pixels[pix](0);
			uint coordY = pixels[pix](1);

			//on recupere la valeur du pixel qu'on veut
			uint Z = images[im](coordX, coordY);

			//on calcule son poid
			uint wZ = calculeWeight(Z, valueMin, valueMax);

			A(pix + N * im, Z) = wZ;		//w(z)*g(0)

			A(pix + N * im, countValues + pix) = -wZ; 	//-w(z) * ln(Ei)

			//on remplit b avec les delta t des images
			b(pix + N * im) = log(exposure[im]) * wZ;
		}
	}

	uint col = 0;
	
	for (uint lig = N * P; lig < A.rows(); ++lig)
	{
		//on remplit la dernière partie de b par des 0
		b(lig) = 0;

		//on calcule le poid pour le z en cours
		uint wZ = calculeWeight(col+1, valueMin, valueMax);

		//et la partie basse de A
		A(lig,col) = lambda * wZ;
		A(lig,col+1) = - 2 * lambda * wZ;
		A(lig,col+2) = lambda * wZ;

		++col;
	}

	//on met g(128) = 1
	A(A.rows() - 1, 128) = 1;
	b(A.rows() - 1) = 1;


	x = (A.transpose() * A).inverse() * A.transpose() * b;

	return x.head(256);		//retourne l'ensemble des valeurs prises par g
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
	Fonction qui calcule une image d'irradiance à partir de la fonction inverse de la réponse des photosites
	images : correspond à un vector de chaque image sous forme de matrices Eigen
	exposure : vector contenant chaque temps d'exposition pour chaque image
	g : vector contenant toutes les valeurs que peut prendre la fonction inverse de la réponse des photosites
	valueMin : valeur minimum dans l'image
	valueMax : valeur maximum dans l'image
*/
MatrixXd computeRadianceMap (   const std::vector <MatrixXi> & images,
								const VectorXd &g,
								const std::vector <double> & exposure,
								const int valueMin,
								const int valueMax)							
{
	MatrixXd radiance = MatrixXd::Zero(images[0].rows(),images[0].cols());

	for (uint i = 0; i < images[0].rows(); ++i)
	{
		for (uint j = 0; j < images[0].cols(); ++j)
		{
			double sommeTop = 0;
			double sommeBottom = 0;
			
			for (uint im = 0; im < images.size(); ++im)
			{
				uint Z = images[im](i,j);
				uint wZ = calculeWeight(Z, valueMin, valueMax);
				sommeTop += wZ * ( g(Z) - log(exposure[im]) );
				sommeBottom += wZ;
			}
			radiance(i,j) = sommeTop / sommeBottom;
		}
	}

	return radiance;	//retourne l'image HDR
}

MatrixXi toneMapping(  const MatrixXd & imageHDR,
					   const int valueMin,
					   const int valueMax)	
{
	MatrixXi imageInterpole = MatrixXi::Zero(imageHDR.rows(), imageHDR.cols());

	//on cherche le min et le max de l'image hdr
	double min = DBL_MAX;	//on prend le plus grand double
	double max = -DBL_MAX;	//on prend le plus petit double

	for (uint i = 0; i < imageHDR.rows(); ++i)
	{
		for (uint j = 0; j < imageHDR.cols(); j++)
		{
			if (imageHDR(i,j) != imageHDR(i,j))		//on ne veux pas prendre en compte
				continue;							//les nan pour calculer le min et le max
			if (imageHDR(i,j) < min)
				min = imageHDR(i,j);
			if (imageHDR(i,j) > max)
				max = imageHDR(i,j);
		}
	}

	int nbNAN = 0;

	double diviseur = max - min;
	if ((max - min) == 0)
		diviseur = 1;

	for (uint i = 0; i < imageInterpole.rows(); ++i)
	{
		for (uint j = 0; j < imageInterpole.cols(); ++j)
		{
			if ( imageHDR(i,j) != imageHDR(i,j))
			{
				imageInterpole(i,j) = 255;
				nbNAN ++;
			}
			else imageInterpole(i,j) = (int) ( ( (imageHDR(i,j) - min) * (valueMax - valueMin) ) / diviseur);
		}
	}
	//std::cerr << "\tmin       : " << min << std::endl;
	//std::cerr << "\tmax       : " << max << std::endl;
	//std::cerr << "\tnb de nan : " << nbNAN << std::endl;

	return imageInterpole;
}