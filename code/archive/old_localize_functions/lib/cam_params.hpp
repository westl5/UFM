#pragma once
#ifndef INCLUDED_PIXART_CAM_PARAMS_HPP
#define INCLUDED_PIXART_CAM_PARAMS_HPP

#include <cmath>

//camera parameters taken from PixArt datasheet (PAJ7025R3 version)

namespace pixart
{

  namespace cam_params
  {

    static const constexpr double effective_focal_length = 0.378e-3; // meters
    static const constexpr double fno = 2.93e-3;                    // meters
    static const constexpr double image_circle = 1.6e-3;              // meters
    static const constexpr double back_focal_length = 0.76e-3;         // meters
    static const constexpr double image_area = 1.524e-3;                // meters (seems to refer to diagonal length of sensor)
    static const constexpr double pixels_x = 98;                        // pixels
    static const constexpr double pixels_y = 98;                        // pixels
    static const constexpr double pixel_width = 11e-6;                  // meters
    static const constexpr double pixel_height = 11e-6;                 // meters

    static double focal_length_x_pixels(double image_pixels_x = pixels_x)
    {
      double image_width = pixel_width * pixels_x;
      return effective_focal_length * image_pixels_x / image_width;
    }

    static double focal_length_y_pixels(double image_pixels_y = pixels_y)
    {
      double image_height = pixel_height * pixels_y;
      return effective_focal_length * image_pixels_y / image_height;
    }

  } // cam_params

  struct settings
  {
    uint16_t resolution_x;
    uint16_t resolution_y;
  };

} // pixart

#endif  // INCLUDED_PIXART_CAM_PARAMS_HPP