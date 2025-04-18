/*
 * Solves for pose and renders a 3D representation of the target LED board.
 *
 * Notes:
 * ------
 * - OpenCV 4.0.1 is assumed. Careful attention must be paid to
 *   solvePnPRANSAC(), whose function signature differed in earlier versions
 *   but in a way that the compiler would not catch (an integer parameter
 *   changed to a double).
 * - Need to read up on 2D-3D point correspondence algorithms. A promising
 *   paper is: "A Quick 3D-to-2D Points Matching based on the Perspective
 *   Projection" by Songxiang Gu, Clifford Lindsay, Michael A. Gennert, and
 *   Michael A. King.
 */
//bandaid
#include <cstdio>
#include <cstring>
#include <Arduino.h>
#include <SPI.h> 
//end of bandaid 

#include "localize.hpp"
#include "render.hpp"
#include "cam_params.hpp"
//#include "apps/object_visualizer/render.hpp"
//#include "pixart/cam_params.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>

//---------------------------bandaid fix for now to get rid of errors --------------------------------------------------------
struct PA_object
{
  uint16_t area; 
  uint16_t cx;
  uint16_t cy;
  uint8_t average_brightness;
  uint8_t max_brightness;
  uint8_t range;
  uint8_t radius;
  uint8_t boundary_left;
  uint8_t boundary_right;
  uint8_t boundary_up;
  uint8_t boundary_down;
  uint8_t aspect_ratio;
  uint8_t vx;
  uint8_t vy;
  
  //renders boundary information into a symbol based visualization of the object
  void render(char *output, int pitch, char symbol)
  {
    int lx = min(97, (int) boundary_left);
    int rx = min(97, (int) boundary_right);
    int uy = min(97, (int) boundary_up);
    int dy = min(97, (int) boundary_down);
    Serial.print(lx,DEC);Serial.print(",");Serial.print(rx,DEC);Serial.print(",");Serial.print(uy,DEC);Serial.print(",");Serial.print(dy,DEC);Serial.print("\n");
    for (int y = uy; y <= dy; y++)
    {
      for (int x = lx; x <= rx; x++)
      {
        output[y * pitch + x] = symbol;
      }
    }
  }
  //prints relevant object information over serial
  void print()
  {
    char buffer[1024];
    char *ptr = buffer;
    ptr += sprintf(ptr, "center          = (%d,%d)\n", cx, cy);
    ptr += sprintf(ptr, "area            = %d\n", area);
    ptr += sprintf(ptr, "avg. brightness = %d\n", average_brightness);
    ptr += sprintf(ptr, "max brightness  = %d\n", max_brightness);
    ptr += sprintf(ptr, "range           = %d\n", range);
    ptr += sprintf(ptr, "radius          = %d\n", radius);
    ptr += sprintf(ptr, "boundary        = (%d,%d,%d,%d)\n", boundary_left, boundary_right, boundary_up, boundary_down);
    ptr += sprintf(ptr, "aspect          = %d\n", aspect_ratio);
    ptr += sprintf(ptr, "vx              = %d\n", vx);
    ptr += sprintf(ptr, "vy              = %d\n", vy);
    Serial.print(buffer);
  }
  //loads object data from sensor into PA_object struct
  void load(const uint8_t *data, int format)
  {
    memset(this, 0, sizeof(this));

    // Formats 1-4
    area = data[0] | ((data[1] & 0x3f) << 8);
    cx = data[2] | ((data[3] & 0x0f) << 8);
    cy = data[4] | ((data[5] & 0x0f) << 8);

    // Format 1, 3
    if (format == 1 || format == 3)
    {
      average_brightness = data[6];
      max_brightness = data[7];
      range = data[8] >> 4;
      radius = data[8] & 0xf;
    }

    if (format == 1 || format == 4)
    {
      int offset = format == 4 ? 3 : 0;
      boundary_left = data[9 - offset] & 0x7f;
      boundary_right = data[10 - offset] & 0x7f;
      boundary_up = data[11 - offset] & 0x7f;
      boundary_down = data[12 - offset] & 0x7f;
      aspect_ratio = data[13 - offset];
      vx = data[14 - offset];
      vy = data[15 - offset];
    }
  }

  PA_object(const uint8_t *data, int format)
  {
    load(data, format);
  }

  PA_object()
  {
  }
};
//---------------------------------------end of bandaid fix --------------------------------------------------------

class localize_impl
{
public:
  localize_impl(int width, int height, const std::string &solver_name, bool use_ransac)
      : m_use_ransac(use_ransac),
      m_target_points
      {
        // Positions of target LEDs in object-local space (world units), in
        // progressive scan order (top to bottom, left to right) as they would
        // appear when facing the camera
        { -0.5f * k_target_width, 0.5f * k_target_height, 0 },  // top left
        { 0.5f * k_target_width, 0.5f * k_target_height, 0 },   // top right corner
        { -0.5f * k_target_width, -0.5f * k_target_height, 0 }, // bottom left corner
        { 0.5f * k_target_width, -0.5f * k_target_height, 0 }   // bottom right corner
      }
  {
    assert(m_leds.size() == m_target_points.size());

    std::map<std::string, int> solver_flags_by_name
    {
      { "iterative",  cv::SOLVEPNP_ITERATIVE },
      { "p3p",        cv::SOLVEPNP_P3P },
      { "ap3p",       cv::SOLVEPNP_AP3P },
      { "epnp",       cv::SOLVEPNP_EPNP },
      { "dls",        cv::SOLVEPNP_DLS },
      { "upnp",       cv::SOLVEPNP_UPNP }
    };

    auto it = solver_flags_by_name.find(solver_name);
    if (it == solver_flags_by_name.end())
    {
      throw std::runtime_error("Invalid PnP solver algorithm: ");
    }
    m_solver_algo = it->second;
  }

  void init(const pixart::settings &settings)
  {
    float fx = pixart::cam_params::focal_length_x_pixels(settings.resolution_x);
    float fy = pixart::cam_params::focal_length_y_pixels(settings.resolution_y);
    float cx = 0.5f * settings.resolution_x;
    float cy = 0.5f * settings.resolution_y;
    m_camera_intrinsic = (cv::Mat_<float>(3, 3) <<
      fx, 0,  cx,
      0,  fy, cy,
      0,  0,  1);
  }

  void update(const std::array<PA_object, 16> &objs)
  {
    canonicalize_leds(objs);
    perspective_update();
    //draw_test_scene();
  }

private:
  int m_solver_algo;
  bool m_use_ransac;

  cv::Mat m_camera_intrinsic;
  static constexpr float k_target_width = 8e-2f;
  static constexpr float k_target_height = 3e-2f;
  const std::vector<cv::Point3f> m_target_points;

  struct led_position
  {
    int cx = 0;
    int cy = 0;
    int idx = -1;
  };

  std::array<led_position, 4> m_leds;

  static int is_on_screen(const PA_object &led)
  {
    return led.cx < 0xfff && led.cy < 0xfff;
  }

  template <typename T1, typename T2>
  static int distance(const T1 &led1, const T2 &led2)
  {
    int dx = led1.cx - led2.cx;
    int dy = led1.cy - led2.cy;
    return dx*dx + dy*dy;
  }

  struct edge
  {
    int idx1; // top-most (vertical edge) or left-most (horizontal edge)
    int idx2;
    float midpoint_x;
    float midpoint_y;

    void set_vertical(int led1_idx, int led2_idx, const std::array<PA_object, 16> &objs)
    {
      if (objs[led1_idx].cy < objs[led2_idx].cy)
      {
        idx1 = led1_idx;
        idx2 = led2_idx;
      }
      else
      {
        idx1 = led2_idx;
        idx2 = led1_idx;
      }
      midpoint_x = 0.5f * (objs[led1_idx].cx + objs[led2_idx].cx);
      midpoint_y = 0.5f * (objs[led1_idx].cy + objs[led2_idx].cy);
    }

    void set_horizontal(int led1_idx, int led2_idx, const std::array<PA_object, 16> &objs)
    {
      if (objs[led1_idx].cx < objs[led2_idx].cx)
      {
        idx1 = led1_idx;
        idx2 = led2_idx;
      }
      else
      {
        idx1 = led2_idx;
        idx2 = led1_idx;
      }
      midpoint_x = 0.5f * (objs[led1_idx].cx + objs[led2_idx].cx);
      midpoint_y = 0.5f * (objs[led1_idx].cy + objs[led2_idx].cy);
    }
  };

  void sort_by_distance_from(int base_idx, std::array<int, 3> &neighbors_out, const std::array<PA_object, 16> &objs)
  {
    // Neighbors of base
    int j = 0;
    for (int i = 0; i < neighbors_out.size() + 1; i++)
    {
      if (i != base_idx)
      {
        neighbors_out[j++] = i;
      }
    }

    // Sort by distance from base
    std::sort(neighbors_out.begin(), neighbors_out.end(),
      [base_idx, &objs](int idx1, int idx2)
      {
        return localize_impl::distance(objs[base_idx], objs[idx1]) < localize_impl::distance(objs[base_idx], objs[idx2]);
      });
  }

  void identify_leds(const std::array<PA_object, 16> &objs)
  {
    edge vertical[2];
    edge horizontal[2];

    // Identify first vertical and horizontal edges
    std::array<int, 3> neighbors;
    int corner_idx = 0;
    sort_by_distance_from(corner_idx, neighbors, objs);
    vertical[0].set_vertical(corner_idx, neighbors[0], objs); // vertical edge is the shorter one
    horizontal[0].set_horizontal(corner_idx, neighbors[1], objs);

    // The remaining unused corner will form remaining two edges
    corner_idx = neighbors[2];
    sort_by_distance_from(corner_idx, neighbors, objs);
    vertical[1].set_vertical(corner_idx, neighbors[0], objs);
    horizontal[1].set_horizontal(corner_idx, neighbors[1], objs);

    // Sort vertical edges from left to right
    if (vertical[0].midpoint_x > vertical[1].midpoint_x)
    {
      std::swap(vertical[0], vertical[1]);
    }

    // Now we know which image points correspond to the 4 corners in scan order
    // Note: horizontal edges were never actually needed
    m_leds[0].idx = vertical[0].idx1;
    m_leds[1].idx = vertical[1].idx1;
    m_leds[2].idx = vertical[0].idx2;
    m_leds[3].idx = vertical[1].idx2;
    for (int i = 0; i < 4; i++)
    {
      m_leds[i].cx = objs[m_leds[i].idx].cx;
      m_leds[i].cy = objs[m_leds[i].idx].cy;
    }
  }

  int find_shortest_distance(const std::array<led_position, 4> &leds)
  {
    int shortest = std::numeric_limits<int>::max();

    for (int i = 0; i < leds.size(); i++)
    {
      if (leds[i].idx < 0)
      {
        // Not assigned
        continue;
      }

      for (int j = i + 1; j < leds.size(); j++)
      {
        if (leds[j].idx < 0)
        {
          continue;
        }
        shortest = std::min(shortest, distance(leds[i], leds[j]));
      }
    }

    return shortest;
  }

  size_t match_to_prior(const std::array<PA_object, 16> &objs, int distance_threshold)
  {
    size_t num_matched = 0;

    std::array<bool, 16> used;
    std::fill_n(used.begin(), used.size(), false);

    // For each LED identified in previous frame...
    for (int i = 0; i < m_leds.size(); i++)
    {
      led_position &prev_frame = m_leds[i];
      if (prev_frame.idx < 0)
      {
        // Previous frame position did not exist. Cannot track.
        continue;
      }

      // Greedily find nearest LED in *this* frame that has not yet been used
      int best_idx = -1;
      int best_distance = std::numeric_limits<int>::max();
      for (int j = 0; j < objs.size(); j++)
      {
        if (used[j] || !is_on_screen(objs[j]))
        {
          continue;
        }

        int dist = distance (prev_frame, objs[j]);
        if (dist < best_distance && dist < distance_threshold)
        {
          best_idx = j;
          best_distance = dist;
        }
      }

      // If we found something, update
      if (best_idx >= 0)
      {
        m_leds[i].idx = best_idx;
        m_leds[i].cx = objs[best_idx].cx;
        m_leds[i].cy = objs[best_idx].cy;
        num_matched += 1;
        used[best_idx] = true;
      }
    }

    return num_matched;
  }

  void canonicalize_leds(const std::array<PA_object, 16> &objs)
  {
    // Try matching to prior frame. Use half the distance of the vertical edge
    // (which should be the shortest distance between LEDs in sensor frame) as
    // threshold.
    int threshold = find_shortest_distance(m_leds) / 4; // 4 because all distances are square distances
    size_t num_matched = match_to_prior(objs, threshold);

    // If could not match all, perform ab initio identification
    if (num_matched < m_leds.size())
    {
      identify_leds(objs);
    }
  }

  void perspective_update()
  {
    // Image points from sensor
    std::vector<cv::Point2f> image_points;
    for (auto &led: m_leds)
    {
      image_points.emplace_back(float(led.cx), float(led.cy));
    }

    // Solve for model-view transform from image points
    cv::Mat rodrigues;
    cv::Mat translation;
    bool result = solve_pnp(m_target_points, image_points, rodrigues, translation);

    // Render
    if (result)
    {
      // Convert Rodrigues vector to 3x3 matrix
      cv::Mat rotation;
      cv::Rodrigues(rodrigues, rotation);

      using namespace render;
      
      // // Set up camera
      // clear();
      // float horizontal_fov = 45;
      // float aspect = float(width()) / float(height());
      // set_camera(45, aspect, vector3(0, 0, 0), euler3::zero());

      // // Apply modelview transform. Flip Z to convert OpenCV camera -> OpenGL
      // // coordinates. Flip Y so that +y is down, as in sensor frame.
      // node::scale scale(vector3(1,-1,-1));
      // node::transform transform(rotation, translation);

      // // Draw board
      // draw_paddle(vector3::zero(), euler3::zero());
    }
  }

  bool solve_pnp(const std::vector<cv::Point3f> &object_points, const std::vector<cv::Point2f> &image_points, cv::Mat &rotation, cv::Mat &translation)
  {
    if (m_use_ransac)
    {
      int iterations = 100;
      float reprojection_error = 8;
      double confidence = 0.99;
      return cv::solvePnPRansac(object_points, image_points, m_camera_intrinsic, cv::Mat(), rotation, translation, false, iterations, reprojection_error, confidence, cv::noArray(), m_solver_algo);
    }
    else
    {
      return cv::solvePnP(object_points, image_points, m_camera_intrinsic, cv::Mat(), rotation, translation, false, m_solver_algo);
    }
  }

  // void draw_paddle(render::vector3 position, render::euler3 rotation)
  // {
  //   using namespace render;

  //   color3 paddle_color(0.7f, 0.2f, 0.2f);
  //   color3 handle_color(0.6f, 0.6f, 0.2f);

  //   float paddle_diameter = 16.5e-2f; // excluding handle
  //   float paddle_radius = 0.5f * paddle_diameter;
  //   float paddle_thickness = 1e-2f;

  //   node::transform transform(position, vector3::one(), rotation);

  //   // Ping pong paddle
  //   {
  //       // Paddle
  //       {
  //         node::rotate rotate(euler3(90, 0, 0));
  //         node::cylinder paddle(vector3::zero(), paddle_radius, paddle_thickness, euler3::zero(), paddle_color);
  //       }

  //       // Handle
  //       float handle_height = 10e-2f;
  //       float handle_width = 2.5e-2f;
  //       node::translate handle_attach_position(vector3(0, 1, 0) * (-paddle_radius + 2e-2f));
  //       node::translate handle_center(vector3::up() * -handle_height * 0.5f);
  //       node::box handle(vector3::zero(), vector3(handle_width, handle_height, handle_width), euler3::zero(), handle_color);
  //   }

  //   // LED board affixed to paddle
  //   draw_led_board(vector3::forward() * -(0.5f * paddle_thickness + 0.001f), euler3::zero());
  // }

  // void draw_led_board(render::vector3 position, render::euler3 rotation)
  // {
  //   using namespace render;

  //   float board_width = 8e-2f;
  //   float board_height = 3e-2f;
  //   color3 board_color = color3(0, 0.5f, 0);
  //   color3 ir_color = color3(0.7f, 0, 0.9f);    // purple
  //   color3 power_color = color3(0.9f, 0.8f, 0); // amber

  //   // Position and orientation
  //   node::transform transform(position, vector3::one(), rotation);

  //   // Breadboard
  //   node::box board(vector3::zero(), vector3(board_width, board_height, 0.002f), euler3::zero(), board_color);

  //   // Board components
  //   {
  //     float led_x = 0.4f; // % of board dimensions
  //     float led_y = 0.4f;
  //     float led_width = 0.05f;
  //     float led_height = led_width * (board_width / board_height);
  //     float led_depth = 2.0f;

  //     // Move pivot point to surface of board
  //     node::translate surface(vector3(0, 0, 0.5f));

  //     // IR LEDs
  //     { node::box led(vector3(-led_x, led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }
  //     { node::box led(vector3(led_x, led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }
  //     { node::box led(vector3(led_x, -led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }
  //     { node::box led(vector3(-led_x, -led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }

  //     // Power LEDs
  //     { node::box led(vector3(0, led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), power_color); }
  //   }
  // }

  // float m_distance = 0;
  // float m_ya = 0;

//   void draw_test_scene()
//   {
//     clear();

//     {
//       using namespace render;
//       float horizontal_fov = 60;
//       float aspect = float(width()) / float(height());
//       set_camera(60, aspect, vector3(0, -0.1f, 0), euler3(30, 0, 0));
//       draw_paddle(vector3::forward() * m_distance, euler3::up() * m_ya);
//     }

//     m_distance += .05f * (1 / 60.0f) ;
//     m_ya += 1;
//   }

};

// namespace localize
// {
//   std::shared_ptr<i_window> create(const util::config::Node &config)
//   {
//     int width = config[k_resolution]["width"].ValueAs<int>();
//     int height = config[k_resolution]["height"].ValueAs<int>();
//     std::string solver_name = util::to_lower(config[k_solver].ValueAs<std::string>());
//     bool use_ransac = config[k_ransac].ValueAs<bool>();
//     return std::make_shared<localize_impl>(width, height, solver_name, use_ransac);
//   }
// }