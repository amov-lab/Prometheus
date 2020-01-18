#ifndef IMAGE_H
#define IMAGE_H

#include <cstdlib>
#include <climits>
#include <cstring>
#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

// use imRef to access image data.
#define imRef(im, x, y) (im->access[y][x])
  
// use imPtr to get pointer to image data.
#define imPtr(im, x, y) &(im->access[y][x])

#define BUF_SIZE 256

typedef unsigned char uchar;
typedef struct { uchar r, g, b; } rgb;

inline bool operator==(const rgb &a, const rgb &b) {
  return ((a.r == b.r) && (a.g == b.g) && (a.b == b.b));
}

// image class
template <class T> class image {
public:

  // create image
  image(const int width, const int height, const bool init = false);

  // delete image
  ~image();

  // init image
  void init(const T &val);

  // deep copy
  image<T> *copy() const;
  
  // get image width/height
  int width() const { return w; }
  int height() const { return h; }
  
  // image data
  T *data;
  
  // row pointers
  T **access;
  
private:
  int w, h;
};

template <class T> image<T>::image(const int width, const int height, const bool init) {
  w = width;
  h = height;
  data = new T[w * h];  // allocate space for image data
  access = new T*[h];   // allocate space for row pointers
  
  // initialize row pointers
  for (int i = 0; i < h; i++)
    access[i] = data + (i * w);  
  
  // init to zero
  if (init)
    memset(data, 0, w * h * sizeof(T));
}

template <class T> image<T>::~image() {
  delete [] data; 
  delete [] access;
}

template <class T> void image<T>::init(const T &val) {
  T *ptr = imPtr(this, 0, 0);
  T *end = imPtr(this, w-1, h-1);
  while (ptr <= end)
    *ptr++ = val;
}


template <class T> image<T> *image<T>::copy() const {
  image<T> *im = new image<T>(w, h, false);
  memcpy(im->data, data, w * h * sizeof(T));
  return im;
}

class pnm_error {};

void pnm_read(std::ifstream &file, char *buf) {
  char doc[BUF_SIZE];
  char c;
  
  file >> c;
  while (c == '#') {
    file.getline(doc, BUF_SIZE);
    file >> c;
  }
  file.putback(c);
  
  file.width(BUF_SIZE);
  file >> buf;
  file.ignore();
}

image<uchar> *loadPGM(const char *name) {
  char buf[BUF_SIZE];
  
  // read header
  std::ifstream file(name, std::ios::in | std::ios::binary);
  pnm_read(file, buf);
  if (strncmp(buf, "P5", 2)) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  pnm_read(file, buf);
  int width = atoi(buf);
  pnm_read(file, buf);
  int height = atoi(buf);

    cout<<width <<" "<<height<<endl;
  pnm_read(file, buf);
  if (atoi(buf) > UCHAR_MAX) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  // read data
  image<uchar> *im = new image<uchar>(width, height);
  file.read((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));

  return im;
}

void savePGM(image<uchar> *im, const char *name) {
  int width = im->width();
  int height = im->height();
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
  file.write((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));
}

image<uchar> *loadJPG(const char *name) {
  Mat img = imread(name,0);
  int width = img.cols;
  int height = img.rows;
  image<uchar> *im;

  // read data
  if(width>0 && height>0)
  {
    im = new image<uchar>(width, height);
  }else
  {
    cout << "ROI too small." <<endl;
    throw pnm_error();
  }

  int p, p0;
  for(int i=0;i<height;i++)
  {
    for(int j=0;j<width;j++)
    {
      p=i*width+j;
      im->data[p] = img.data[p];
    }
  }

  return im;
}


Mat saveJPG(float *data, int width, int height, float scalefactor=256.0) {

  Mat img(height,width,CV_8UC1);
  cout<<"saving file.."<<endl;

  int p;
  for(int i=0;i<height;i++)
  {
    for(int j=0;j<width;j++)
    {
      p=i*width+j;
      img.at<unsigned char>(i,j) = data[p];
    }
  }
  return img;
}


#endif
