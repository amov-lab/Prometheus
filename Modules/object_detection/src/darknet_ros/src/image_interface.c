/*
 * image_interface.c
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "darknet_ros/image_interface.h"

image **load_alphabet_with_file(char *datafile) {
  int i, j;
  const int nsize = 8;
  image **alphabets = calloc(nsize, sizeof(image));
  char* labels = "/labels/%d_%d.png";
  char * files = (char *) malloc(1 + strlen(datafile)+ strlen(labels) );
  strcpy(files, datafile);
  strcat(files, labels);
  for(j = 0; j < nsize; ++j){
    alphabets[j] = calloc(128, sizeof(image));
    for(i = 32; i < 127; ++i){
      char buff[256];
      sprintf(buff, files, i, j);
      alphabets[j][i] = load_image_color(buff, 0, 0);
    }
  }
  return alphabets;
}
