
  
/*
 * Created on Sun Aug 15 2021
 *
 * Author: EpsAvlc
 */

#ifndef LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_
#define LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

namespace pcl{
struct LivoxPointXyzrtlt{
  float x;            /**< x             */
  float y;            /**< y             */
  float z;            /**< z             */
  
  float intensity; /**< intensity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
  double timestamp; /**< Timestamp         */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EIGEN_ALIGN16;
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPointXyzrtlt,  
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint8_t, tag, tag)
                                   (uint8_t, line, line)
                                   (double, timestamp, timestamp)
)

#endif
