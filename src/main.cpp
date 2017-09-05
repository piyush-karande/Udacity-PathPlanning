#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y - y), (map_x - x) );

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //starting lane
  int lane = 1;

  // Reference Velocity
  double ref_vel = 0.0; //m/s

  // Timer for restricting lane changes withing 10 secs of each other
  chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();

  // Lane check flag
  bool check_lanes = false;

  h.onMessage([& check_lanes, &start, &lane, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Number of points remaining from the previously generated trajectory
          int prev_size = previous_path_x.size();

          chrono::time_point<std::chrono::system_clock> end;

          // Set the car's s value to end of path
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          /*****
          Perfoming checks and setting behaviour of the car
          *****/

          // Define different flags for determining behavior
          bool too_close = false;
          bool check_lane_left = false;
          bool check_lane_right = false;
          double front_car_vel = 2.0;
          double curr_front_car_dist = 50.0;
          int count = 0;

          // Check for cars in lane
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {

              // State of car found in lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double curr_car_speed = sqrt(vx * vx + vy * vy);
              double curr_car_s = sensor_fusion[i][5];

              // predict car's position in future. 0.02 is update step size in seconds
              curr_car_s += (double)prev_size * 0.02 * curr_car_speed;

              // If car is too close
              if ((curr_car_s > car_s) && (curr_car_s - car_s < 30)) {
                // Set too_close flag
                too_close = true;

                // Check if the current car is closer than previously found cars in lane
                // and update the distance and velocity of the car in front
                if (curr_front_car_dist > curr_car_s - car_s) {
                  count += 1;
                  curr_front_car_dist = curr_car_s - car_s;
                  front_car_vel = curr_car_speed * 2.24;
                  if (count > 1) {
                    cout << "Updated front car, dist: " << curr_front_car_dist << ", vel: " << front_car_vel << "mph" << endl ;
                  }
                }


              }
            }
          }

          // Setting min_vel of car when another car is too close in same lane
          double min_vel = front_car_vel - 2.0;
          if (curr_front_car_dist < 15) { min_vel = 0.0; }

          // Speed control if too close to a car
          if (too_close && (ref_vel > min_vel)) {
            ref_vel -= .224; // Decrease ref_vel, .224mph every 20msecs is approximately 5m/s
            if (count > 1) {
              cout << "Num cars too close: " << count << endl;
            }

            // Start planning for lane change if speed below 45mph
            if (ref_vel < 45) { check_lanes = true; }
          } else if (ref_vel < 49.50) {
            ref_vel += 0.224; // If no car is too close maintain speed right below 50mph
          } else if (ref_vel > 49.0) { check_lanes = false; } // Dont check lanes if velocity is > 49mph

          // Check elapsed time sinc eprevious lane change
          end = std::chrono::system_clock::now();
          chrono::duration<double> elapsed_seconds = end - start;
          double time_secs = elapsed_seconds.count();

          // Restric lane changes within 10 secs of each other
          if (time_secs < 10) { check_lanes = false; }

          // Set which lanes to check
          if (check_lanes) {
            if (lane == 0) { check_lane_right = true;}
            if (lane == 1) { check_lane_right = true; check_lane_left = true;}
            if (lane == 2) { check_lane_left = true;}
          }

          // Flags for changing lanes
          bool move_left = false;
          bool move_right = false;

          // Distances of cars found in left or right lane
          vector<double> car_dist_left;
          vector<double> car_dist_right;

          // Checking lane on left for safely changing lanes
          if (check_lane_left) {

            // Assuming no cars in front or back
            bool no_car_front = true;
            bool no_car_behind = true;

            // Setting which lane to check
            int lane_to_check = lane - 1;
            int car_count_left = 0;

            // Iterating through sensor fusion data to find cars in lane
            for (int i = 0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              if (d < (2 + 4 * lane_to_check + 2) && d > (2 + 4 * lane_to_check - 2)) {

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double curr_car_speed = sqrt(vx * vx + vy * vy);
                double curr_car_s = sensor_fusion[i][5];

                // predict car's position in future
                curr_car_s += (double)prev_size * 0.02 * curr_car_speed;

                // Car too close behind or in front
                if ((car_s > curr_car_s) && (car_s - curr_car_s < 10)) {
                  no_car_behind = false;
                  car_count_left += 1;

                } else if ((curr_car_s > car_s) && (curr_car_s - car_s < 30)) {
                  no_car_front = false;
                  car_count_left += 1;

                } else if (curr_car_s > car_s) { // Add car distances that are in front but not too close
                  car_dist_left.push_back(curr_car_s - car_s);
                }
              }
            }
            // If no cars found in the vicinity set flag to change lane
            if (no_car_front && no_car_behind) {
              cout << "No car on left" << endl;
              move_left = true;
            }
          }

          // Checking lane on right for safely changing lanes
          // Same as previous if but for right lane
          if (check_lane_right) {
            bool no_car_front = true;
            bool no_car_behind = true;

            int lane_to_check = lane + 1;
            int car_count_right = 0;
            for (int i = 0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              if (d < (2 + 4 * lane_to_check + 2) && d > (2 + 4 * lane_to_check - 2)) {

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double curr_car_speed = sqrt(vx * vx + vy * vy);
                double curr_car_s = sensor_fusion[i][5];

                // predict car's position in future
                curr_car_s += (double)prev_size * 0.02 * curr_car_speed;

                if ((car_s > curr_car_s) && (car_s - curr_car_s < 10)) {

                  no_car_behind = false;
                  car_count_right += 1;

                } else if ((curr_car_s > car_s) && (curr_car_s - car_s < 30)) {

                  no_car_front = false;
                  car_count_right += 1;

                } else if (curr_car_s > car_s) {
                  car_dist_right.push_back(curr_car_s - car_s);
                }
              }
            }

            if (no_car_front && no_car_behind) {
              cout << "No car on right" << endl;
              move_right = true;
            }
          }

          /* Sanity check
          if (time_secs - int(time_secs) < 0.02) {
            cout << "Elapsed time in seconds: " << int(time_secs) << "s" << endl;
            cout << "Checking lanes? " << check_lanes << endl;
            cout << "Checking left? " << check_lane_left << endl;
            cout << "Checking right? " << check_lane_right << endl << endl;
          }
          */

          // If both flags are set, pick a lane to move to
          if (move_left && move_right) {

            cout << "Picking a lane.. " << endl;

            // Check the closest car in adjacent lanes
            double min_dist_left = 0;
            double min_dist_right = 0;

            // Check the average distance of cars in the adjacent lanes
            double avg_dist_left = 0;
            double avg_dist_right = 0;

            // If no car is present in adjance lane, move to that lane with a preference to left lane
            if (car_dist_left.size() == 0 ) {
              min_dist_left = 1000;
              cout << "No cars detected on left side" << endl;
            } else if (car_dist_right.size() == 0 ) {
              min_dist_right = 1000;
              cout << "No cars detected on right side" << endl;
            } else {
              double avg_dist_left = accumulate(car_dist_left.begin(), car_dist_left.end(), 0.0) / car_dist_left.size();
              double avg_dist_right = accumulate(car_dist_right.begin(), car_dist_right.end(), 0.0) / car_dist_right.size();

              sort(car_dist_left.begin(), car_dist_left.end());
              sort(car_dist_right.begin(), car_dist_right.end());

              min_dist_left = car_dist_left[0];
              min_dist_right = car_dist_right[0];

              cout << "Left Avg dist: " << avg_dist_left << ", Closest car at: " << min_dist_left << endl;
              cout << "Right Avg dist: " << avg_dist_right << ", Closest car at: " << min_dist_right << endl;

            }

            // Move to the lane where the closest car is futher away
            if (min_dist_left > min_dist_right) {
              cout << "Moving one lane to the left!!" << endl << endl;
              if (lane == 1) { lane = 0; }
              else if (lane == 2) { lane = 1; }
              start = std::chrono::system_clock::now(); // restart timer
              check_lanes = false; // set check_lanes to false
            } else {
              cout << "Moving one lane to the right!!" << endl << endl;
              if (lane == 0) { lane = 1; }
              else if (lane == 1) { lane = 2; }
              start = std::chrono::system_clock::now();
              check_lanes = false;
            }
          } else if (move_left) { // Move left
            cout << "Moving one lane to the left!!" << endl << endl;
            if (lane == 1) { lane = 0; }
            else if (lane == 2) { lane = 1; }
            start = std::chrono::system_clock::now();
            check_lanes = false;

          } else if (move_right) { // Move right
            cout << "Moving one lane to the right!!" << endl << endl;
            if (lane == 0) { lane = 1; }
            else if (lane == 1) { lane = 2; }
            start = std::chrono::system_clock::now();
            check_lanes = false;

          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          /*****
          Trajectory generation using splines after defining logic for behavior
          The following set of code is similar to the one in walk through video
          *****/

          // Vector of evenly spaced points to create a trajectory spline 
          vector<double> ptsx;
          vector<double> ptsy;

          // Cars referece state
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          // Set initial points for creating spline
          // Use car state if previous path has less than 2 points left
          if (prev_size < 2) {
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);
          } else { // Use last two points from previous trajectory for smoothness 
            double ref_prev_x = previous_path_x[prev_size - 2];
            double ref_prev_y = previous_path_y[prev_size - 2];

            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);

            ptsx.push_back(ref_prev_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_prev_y);
            ptsy.push_back(ref_y);

          }

          // Add evenly spaced point after reference to create a trajectory spline
          vector<double> wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(wp0[0]);
          ptsx.push_back(wp1[0]);
          ptsx.push_back(wp2[0]);

          ptsy.push_back(wp0[1]);
          ptsy.push_back(wp1[1]);
          ptsy.push_back(wp2[1]);


          // Convert to car's local coordinates
          for (int i = 0; i < ptsx.size(); i++) {
            double trans_x  = ptsx[i] - ref_x;
            double trans_y = ptsy[i] - ref_y;

            ptsx[i] = (trans_x * cos(0 - ref_yaw) - trans_y * sin(0 - ref_yaw));
            ptsy[i] = (trans_x * sin(0 - ref_yaw) + trans_y * cos(0 - ref_yaw));

          }

          // Create spline
          tk::spline s;

          // Add points to spline
          s.set_points(ptsx, ptsy);

          // Add points from leftover path to next
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Set the horizon for trajectory in meters
          double horizon_x = 30.0;
          double horizon_y = s(horizon_x);
          double horizon_dist = sqrt(horizon_x * horizon_x + horizon_y * horizon_y);

          double x_add_on = 0;

          for (int i = 0; i <= 50 - previous_path_x.size(); i++) {

            // Calculate number of points at this instant
            double num_points = (horizon_dist / (0.02 * ref_vel / 2.24)); // 2.24 for mph to m/s conversion
            double curr_x = x_add_on + horizon_x / num_points;
            double curr_y = s(curr_x);

            x_add_on = curr_x;

            double x_ref = curr_x;
            double y_ref = curr_y;

            //Convert back to global coordinates
            curr_x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            curr_y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            curr_x += ref_x;
            curr_y += ref_y;

            next_x_vals.push_back(curr_x);
            next_y_vals.push_back(curr_y);

          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
  size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
  char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}