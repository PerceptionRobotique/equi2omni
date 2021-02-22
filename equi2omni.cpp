/*!
 \file equi2omni.cpp
 \brief Equirectangular to central omnidirectional mapping (RGB image format), exploiting PeR core and io modules
 * example of command line :

  ./equi2omni ../data/omni_400_400.xml ../media/ 94 104 400 400 0

 \param xmlFic the omnidirectional camera calibration xml file
 \param imagesDir directory where equirectangular images to read (with 6 digits before the extension) are and where the output omni images will be written (with character 'o' before the 6 digits)
 \param iFirst the number of the first image to transform
 \param iLast the number of the last image to transform
 \param width the width (pixels) of the output omnidirectional image
 \param height the height (pixels) of the output omnidirectional image
 \param camOri omnidirectional camera axis orientation (degrees): 0 (default) downward, 180 upward
 *
 \author Guillaume CARON
 \version 0.1
 \date February - 2021
 */

// VISP includes
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

#include <per/prOmni.h>
#include <per/prEquirectangular.h>

#include <per/prOmniXML.h>

#include <fstream>

#define FILE_EXT "png"

#define VERBOSE

// global variables
vpImage<vpRGBa> Io;

double camOri;
/*unsigned */int ehaut, elarg, ohaut, olarg;
prCameraModel *ocam, *ecam;

unsigned int nbPixelso, nbPixelse;
int *coordMapping[4] = {NULL, NULL, NULL, NULL}; // omni and equi image pixel coordinates

// functions prototypes
void init();
void imageTransform(vpImage<vpRGBa>& equiImage);

int main (int argc, char **argv)
{
    std::ostringstream s;
    std::string filename;

    //1. Loading omnidirectional an camera from an XML file got from the MV calibration software
    if(argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no XML omnidirectional camera file given" << std::endl;
#endif
        return -1;
    }
    
    //Create a camera
    ocam = new prOmni();

    // Load the camera parameters from the XML file
    {
        prOmniXML fromFile(argv[1]);
        
        fromFile >> (*((prOmni *)ocam));
    }

#ifdef VERBOSE
    std::cout << "Loading the XML file to an empty rig..." << std::endl;

    // If a sensor is loaded, print its parameters
    std::cout << "the omnidirectional camera intrinsic parameters are alpha_u = " << ocam->getau() << " ; alpha_v = " << ocam->getav() << " ; u_0 = " << ocam->getu0() << " ; v_0 = " << ocam->getv0()  << " ; xi = " << ((prOmni *)ocam)->getXi() << std::endl;

#endif

  //2. Create the equirectangular camera from an equirectangular image
  //Loading an image supposed equirectangular
  vpImage<vpRGBa> Ie;

    if(argc < 3)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -2;
    }

    if(argc < 4)
    {
#ifdef VERBOSE
        std::cout << "no first image index given" << std::endl;
#endif
        return -3;
    }

    if(argc < 5)
    {
#ifdef VERBOSE
        std::cout << "no last image index given" << std::endl;
#endif
        return -4;
    }

    unsigned int iFirst = atoi(argv[3]);
    unsigned int iLast = atoi(argv[4]);

    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << argv[2] << std::setw(6) << std::setfill('0') << iFirst << "." << FILE_EXT;
    filename = s.str();

    try
    {
      vpImageIo::read(Ie, filename);
    }
    catch(vpException e)
    {
#ifdef VERBOSE
        std::cout << "unable to load the input image file" << std::endl;
#endif
        return -5;
    }

    ehaut = Ie.getHeight();
    elarg = Ie.getWidth();
    nbPixelse = ehaut*elarg;

    ecam = new prEquirectangular(elarg*0.5/M_PI, ehaut*0.5/(M_PI*0.5), elarg*0.5, ehaut*0.5);


    //3. Create the omnidirectional image from provided size
    if(argc < 7)
    {
#ifdef VERBOSE
        std::cout << "no omni image width or height given" << std::endl;
#endif
        return -6;
    }

    olarg = atoi(argv[5]);
    ohaut = atoi(argv[6]);

    nbPixelso = olarg*ohaut;

    //get omnicamera orientation with respect to the equirectangular one
    camOri = 0.0;
    if(argc < 8)
    {
#ifdef VERBOSE
        std::cout << "no omni camera orientation with respect to the equirectangular one given" << std::endl;
#endif
    }
    else
      camOri = atof(argv[7])*M_PI/180.;

  init();

  //4. Open, transform an save every images

  for(unsigned int i = iFirst ; i <= iLast ; i++)
  {
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << argv[2] << std::setw(6) << std::setfill('0') << i << "." << FILE_EXT;
    filename = s.str();

    try
    {
      vpImageIo::read(Ie, filename);
    }
    catch(vpException e)
    {
#ifdef VERBOSE
        std::cout << "unable to load the input image file" << std::endl;
#endif
        return -6;
    }

    imageTransform(Ie);

    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << argv[2] << "o" << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(Io, filename);
  }

  //after killing ros node
  for(unsigned int c = 0 ; c < 4 ; c++)
    if(coordMapping[c] != NULL)
    {
      delete [] coordMapping[c];
      coordMapping[c] = NULL;
    }

  return 0;
}

void init()
{
  if( (nbPixelso == 0) || (nbPixelse == 0) )
  {
    std::cout << "equi2omni::init: nbPixels == 0" << std::endl;
    return;
  }

  // Get pixel coordinates mapping
  for(unsigned int c = 0 ; c < 4 ; c++)
    coordMapping[c] = new int[nbPixelso];

  // Compute the mapping
  double Xs, Ys, Zs;
  int *pt_uo = coordMapping[0], *pt_vo = coordMapping[1], *pt_ue = coordMapping[2], *pt_ve = coordMapping[3];

  vpHomogeneousMatrix ceMco(0,0,0,camOri,0,0);
  //std::cout << ceMco << std::endl;

  prCartesian3DPointVec sP;

  for(unsigned int vo = 0; vo < ohaut ; vo++)
    for(unsigned int uo = 0; uo < olarg ; uo++, pt_ue++, pt_ve++, pt_uo++, pt_vo++)
    {
      prPointFeature P;
      P.setPixUV(uo,vo);
      ocam->pixelMeterConversion(P);
      if(((prOmni *)ocam)->projectImageSphere(P, Xs, Ys, Zs) == 0)
      {
        //P.set_X(Xs); P.set_Y(Ys); P.set_Z(Zs);
        P.setWorldCoordinates(Xs, Ys, Zs);
        P.changeFrame(ceMco);

        ((prEquirectangular *)ecam)->project3DImage(P); // projectSphereImage
        ecam->meterPixelConversion(P);
      }
      else
      {
        P.setPixUV(0,0);
      }

       *pt_uo = uo;
       *pt_vo = vo;
       *pt_ue = P.get_u();
       *pt_ve = P.get_v();
    }

  // Initialize the image to be created
  Io.resize(ohaut, olarg, false);
}

void imageTransform(vpImage<vpRGBa>& equiImage)
{
#ifdef VERBOSE
    std::cout << "equi2omni::imageTransform" << std::endl;
#endif

  int *pt_uo = coordMapping[0], *pt_vo = coordMapping[1], *pt_ue = coordMapping[2], *pt_ve = coordMapping[3];

  //Optimized version with specific knowledge on the equirect coordinates (a few ms less)
  vpRGBa *pt_equi = equiImage.bitmap, *pt_omni = Io.bitmap;

  unsigned int pix = 0;
  for(unsigned int p = 0; p < nbPixelso ; p++, pt_omni++, pt_ue++, pt_ve++)
  {
    *pt_omni = *(pt_equi + ((*pt_ve)*elarg+(*pt_ue))); //fastest
  }

}

