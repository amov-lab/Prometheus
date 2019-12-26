#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

class Descriptor {
  
public:
  
  // constructor creates filters
  Descriptor(uint8_t* I,int32_t width,int32_t height,int32_t bpl,bool half_resolution);
  
  // deconstructor releases memory
  ~Descriptor();
  
  // descriptors accessible from outside
  uint8_t* I_desc;
  
private:

  // build descriptor I_desc from I_du and I_dv
  void createDescriptor(uint8_t* I_du,uint8_t* I_dv,int32_t width,int32_t height,int32_t bpl,bool half_resolution);

};

#endif
