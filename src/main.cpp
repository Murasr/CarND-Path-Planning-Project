#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//! Horizon distance in meters
double horizon_front = 50;

//! Current vehicle velocity in mph
double cur_vel = 1.0;

//! Method which detects whether a collision is possible at N timestamps ahead
//! @param - f_lane - Target lane
//! @param - car_s - S position of car
//! @param - sensor_fusion - Sensor Fusion Data of all objects
//! @param - num_timestamps_ahead - Number of timestamps ahead to consider for detecting collision
bool predictCollision(int f_lane, double car_s, vector<vector<double>> sensor_fusion, int num_timestamps_ahead, 
                      vector<double>& map_waypoints_x, vector<double>& map_waypoints_y)
{
  bool isCollidable = false;
  
  for(int i=0; i<sensor_fusion.size(); i++)
  {
    double d = sensor_fusion[i][6];
    
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    
    double x_vel = sensor_fusion[i][3];
    double y_vel = sensor_fusion[i][4];
    
    double x_future = x + num_timestamps_ahead * 0.02 * x_vel;
    double y_future = y + num_timestamps_ahead * 0.02 * y_vel;
    double yaw = atan2(y_future - y, x_future - x);
    
    vector<double> other_vehicle_frenet = getFrenet(x_future, y_future, yaw, map_waypoints_x, map_waypoints_y);

    double ego_d = f_lane*4 + 2;
    double ego_d_future = other_vehicle_frenet[1]; //f_lane*4 + 2;

    //! Consider only vehicles within the target lane
    if( ((d > (ego_d - 2)) && (d < (ego_d + 2))) /*|| ((d > (ego_d_future - 2)) && (d < (ego_d_future + 2)))*/ )
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double veh_speed = sqrt(vx * vx + vy * vy);

      double s = sensor_fusion[i][5];
      s += num_timestamps_ahead * 0.02 * veh_speed;
      
      double s_present = sensor_fusion[i][5];
      double s_future = s;//other_vehicle_frenet[0];

      //! Detect collision only if the target is in front of ego vehicle and distance is less than the horizon limit
      if( (s_future > car_s) && ((s_future - car_s) < horizon_front) )
      {
        isCollidable = true;
        break;
      }
    }
  }
  
  return isCollidable;
}


//! Method which generates the trajectory given car position and target lane
//! @param - car_s - S position of car
//! @param - f_lane - Target lane
//! @param - map_waypoints_s, map_waypoints_x, map_waypoints_y - For conversion from Frenet to XY coordinates
//! @param - spline_xs, spline_ys - Trailing (older) points in the spline for smoother transition
//! @param - previous_path_x, previous_path_y - List of previous X and previous Y points to be prefixed to the trajectory
//! @param - f_traj - [out] - The output trajectory
void generateTrajectory(double car_s, int f_lane, 
                        vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, 
                        vector<double> spline_xs, vector<double> spline_ys, 
                        vector<double>& previous_path_x, vector<double>& previous_path_y, 
                        STrajectory& f_traj)
{
   //! Horizon of 3*horizon_front metres
  vector<double> wp1 = getXY(car_s + horizon_front, (f_lane*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> wp2 = getXY(car_s + horizon_front*1.5, (f_lane*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> wp3 = getXY(car_s + horizon_front*2, (f_lane*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  spline_xs.push_back(wp1[0]);
  spline_xs.push_back(wp2[0]);
  spline_xs.push_back(wp3[0]);

  spline_ys.push_back(wp1[1]);
  spline_ys.push_back(wp2[1]);
  spline_ys.push_back(wp3[1]);

  double ref_x = spline_xs[1];
  double ref_y = spline_ys[1];
  double ref_yaw = atan2(ref_y - spline_ys[0], ref_x - spline_xs[0]);

  //! Do the translation and rotation to transform the origin
  for(int count = 0; count < spline_xs.size(); count++)
  {
    double trans_x = spline_xs[count] - ref_x;
    double trans_y = spline_ys[count] - ref_y;

    spline_xs[count] = trans_x * cos(-ref_yaw) - trans_y * sin(-ref_yaw);
    spline_ys[count] = trans_x * sin(-ref_yaw) + trans_y * cos(-ref_yaw);
  }

  //! store the previous paths
  for(int count = 0; count < previous_path_x.size(); count++)
  {
    f_traj.next_x_vals.push_back(previous_path_x[count]);
    f_traj.next_y_vals.push_back(previous_path_y[count]);
  }

  ::tk::spline s;
  //printf("set_spline\n");
  s.set_points(spline_xs, spline_ys);

  double x = horizon_front;
  double y = s(x);
  double dist = distance(x, y, 0, 0);

  //2.237 is the factor to convert mph to m/s
  double N = dist/(0.02 * cur_vel / 2.237);
  double x_step = x/N;

  //! Calculate the remaining points and append to trajectory
  for(int count = 1; count <= 50 - previous_path_x.size(); count++)
  {
    double x_val = x_step*count;
    double y_val = s(x_val);

    double x_rotated = x_val * cos(ref_yaw) - y_val * sin(ref_yaw);
    double y_rotated = x_val * sin(ref_yaw) + y_val * cos(ref_yaw);

    f_traj.next_x_vals.push_back(x_rotated + ref_x);
    f_traj.next_y_vals.push_back(y_rotated + ref_y);
  }
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  int cur_lane = 1;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &cur_lane, &cur_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          
          auto previousPathSize = previous_path_x.size();
            
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          //! Store the last point as the current point
          if(previousPathSize > 0)
            car_s = end_path_s;
          
          //! by default, speed should be increased.
          bool increase_speed = true;
          
          //! Predict collision if any 
          bool isCollisionExpected = predictCollision(cur_lane, car_s, sensor_fusion, previousPathSize, map_waypoints_x, map_waypoints_y);
          
          //! Do not increase speed in case of collision
          increase_speed = !isCollisionExpected;
          
          //! Increase speed only within the speed limit 
          if(increase_speed && cur_vel < 48)
          {
            cur_vel += cur_vel * 0.030;
          }
          //! Decrease speed until the minimum speed limit
          else if(!increase_speed && cur_vel > 20)
          {
            cur_vel -= cur_vel * 0.030;
          }

          //! Build path planning points using spline
          vector<double> spline_xs;
          vector<double> spline_ys;
          
          if(previousPathSize < 2)
          {
            double yawRateRadians = deg2rad(car_yaw);
            
            double pre_x = car_x - cos(yawRateRadians);
            double pre_y = car_y - sin(yawRateRadians);
            
            spline_xs.push_back(pre_x);
            spline_xs.push_back(car_x);
            
            spline_ys.push_back(pre_y);
            spline_ys.push_back(car_y);
          }
          else
          {
            spline_xs.push_back(previous_path_x[previousPathSize-2]);
            spline_xs.push_back(previous_path_x[previousPathSize-1]);
            
            spline_ys.push_back(previous_path_y[previousPathSize-2]);
            spline_ys.push_back(previous_path_y[previousPathSize-1]);
          }
          
          //! If the velocity is less than 35 and collision is expected, try to change lanes
          //! The limit of 35 is given so that the vehicle slows down before changing lanes
          if(cur_vel < 35 && isCollisionExpected)
          {
            switch(cur_lane)
            {
               case 0:
                {
                  //! If current lane is 0, the new lane would be 1.
                  int new_lane = 1;
                  
                  //! Generate the trajectory for the lane change
                  STrajectory lane_change_traj_r;
                  generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj_r);
        
                  //! Predict collisions during lane change - This is done by predicting collision at the end point and mid point of the trajectory
                  int size = lane_change_traj_r.next_x_vals.size();
                  if(size/2 > 1)
                  {
                      //! Collision at end point
                      double y1 = lane_change_traj_r.next_y_vals[size-1];
                      double y2 = lane_change_traj_r.next_y_vals[size-2];
                      double x1 = lane_change_traj_r.next_x_vals[size-1];
                      double x2 = lane_change_traj_r.next_x_vals[size-2];
                      double yaw = atan2(y1 - y2,  x1 - x2);
                    
                      vector<double> new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                      bool isCollisionEndPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y);
 
                      //! collision at midpoint
                      y1 = lane_change_traj_r.next_y_vals[size/2-1];
                      y2 = lane_change_traj_r.next_y_vals[size/2-2];
                      x1 = lane_change_traj_r.next_x_vals[size/2-1];
                      x2 = lane_change_traj_r.next_x_vals[size/2-2];
                      yaw = atan2(y1 - y2,  x1 - x2);
                    
                      new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                      bool isCollisionMidPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y); 
                    
                      //! Do lane change only if no collision is predicted
                      if(!isCollisionEndPoint && !isCollisionMidPoint)
                        cur_lane = new_lane;
                  }
                  break;
                }
              case 1:
                {
                  //! If current lane is 1, new lane would be 0 for left lane change
                  int new_lane = 0;
                  
                  {
                    //! Generate the trajectory for the lane change
                    STrajectory lane_change_traj;
                    generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj);

                    //! Predict collisions during lane change - This is done by predicting collision at the end point and mid point of the trajectory
                    int size = lane_change_traj.next_x_vals.size();
                    if(size/2 > 1)
                    {
                        //! Collision at end point
                        double y1 = lane_change_traj.next_y_vals[size-1];
                        double y2 = lane_change_traj.next_y_vals[size-2];
                        double x1 = lane_change_traj.next_x_vals[size-1];
                        double x2 = lane_change_traj.next_x_vals[size-2];
                        double yaw = atan2(y1 - y2,  x1 - x2);

                        vector<double> new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                        bool isCollisionEndPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y);

                        //! collision at midpoint
                        y1 = lane_change_traj.next_y_vals[size/2-1];
                        y2 = lane_change_traj.next_y_vals[size/2-2];
                        x1 = lane_change_traj.next_x_vals[size/2-1];
                        x2 = lane_change_traj.next_x_vals[size/2-2];
                        yaw = atan2(y1 - y2,  x1 - x2);

                        new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                        bool isCollisionMidPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y); 

                        //! Do lane change only if no collision is predicted
                        if(!isCollisionEndPoint && !isCollisionMidPoint)
                          cur_lane = new_lane;

                        //double yaw = atan2(lane_change_traj.next_y_vals[size-1] - lane_change_traj.next_y_vals[size-2], lane_change_traj.next_x_vals[size-1] - lane_change_traj.next_x_vals[size-2]);
                        //vector<double> new_car_frenet = getFrenet(lane_change_traj.next_x_vals[size-1], lane_change_traj.next_y_vals[size-1], yaw, map_waypoints_x, map_waypoints_y);
                        //if(!isCollision(new_lane, new_car_frenet[0], sensor_fusion, size))
                          //lane = new_lane;
                    }
                  }

                  //! If left lane change is not possible, then check for right lane change
                  if(cur_lane != new_lane)
                  {
                    //! If current lane is 1, new lane would be 2 for right lane change
                    new_lane = 2;
                    
                    //! Generate the trajectory for the lane change
                    STrajectory lane_change_traj;
                    generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj);

                    //! Predict collisions during lane change - This is done by predicting collision at the end point and mid point of the trajectory
                    int size = lane_change_traj.next_x_vals.size();
                    if(size/2 > 1)
                    {
                        //! Collision at end point
                        double y1 = lane_change_traj.next_y_vals[size-1];
                        double y2 = lane_change_traj.next_y_vals[size-2];
                        double x1 = lane_change_traj.next_x_vals[size-1];
                        double x2 = lane_change_traj.next_x_vals[size-2];
                        double yaw = atan2(y1 - y2,  x1 - x2);

                        vector<double> new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                        bool isCollisionEndPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y);

                        //! collision at midpoint
                        y1 = lane_change_traj.next_y_vals[size/2-1];
                        y2 = lane_change_traj.next_y_vals[size/2-2];
                        x1 = lane_change_traj.next_x_vals[size/2-1];
                        x2 = lane_change_traj.next_x_vals[size/2-2];
                        yaw = atan2(y1 - y2,  x1 - x2);

                        new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                        bool isCollisionMidPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y); 

                        //! Do lane change only if no collision is predicted
                        if(!isCollisionEndPoint && !isCollisionMidPoint)
                          cur_lane = new_lane;

                        //double yaw = atan2(lane_change_traj.next_y_vals[size-1] - lane_change_traj.next_y_vals[size-2], lane_change_traj.next_x_vals[size-1] - lane_change_traj.next_x_vals[size-2]);
                        //vector<double> new_car_frenet = getFrenet(lane_change_traj.next_x_vals[size-1], lane_change_traj.next_y_vals[size-1], yaw, map_waypoints_x, map_waypoints_y);
                        //if(!isCollision(new_lane, new_car_frenet[0], sensor_fusion, size))
                          //lane = new_lane;
                    }
                    
                    //STrajectory lane_change_traj;
                    //generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj);

                    //if(lane_change_traj.next_x_vals.size() > 1)
                    //{
                    //    int size = lane_change_traj.next_x_vals.size();
                    //    double yaw = atan2(lane_change_traj.next_y_vals[size-1] - lane_change_traj.next_y_vals[size-2], lane_change_traj.next_x_vals[size-1] - lane_change_traj.next_x_vals[size-2]);
                    //    vector<double> new_car_frenet = getFrenet(lane_change_traj.next_x_vals[size-1], lane_change_traj.next_y_vals[size-1], yaw, map_waypoints_x, map_waypoints_y);
                    //    if(!isCollision(new_lane, new_car_frenet[0], sensor_fusion, size))
                    //     lane = new_lane;
                    //}
                  }
                  break;
                }
                case 2:
                {
                  //! If current lane is 2, new lane would be 1 for lane change
                  int new_lane = 1;
                  
                  //! Generate the trajectory for the lane change
                  STrajectory lane_change_traj_l;
                  generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj_l);

                  //! Predict collisions during lane change - This is done by predicting collision at the end point and mid point of the trajectory
                  int size = lane_change_traj_l.next_x_vals.size();
                  if(size/2 > 1)
                  {
                      //! Collision at end point
                      double y1 = lane_change_traj_l.next_y_vals[size-1];
                      double y2 = lane_change_traj_l.next_y_vals[size-2];
                      double x1 = lane_change_traj_l.next_x_vals[size-1];
                      double x2 = lane_change_traj_l.next_x_vals[size-2];
                      double yaw = atan2(y1 - y2,  x1 - x2);
                    
                      vector<double> new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                      bool isCollisionEndPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y);
 
                      //! collision at midpoint
                      y1 = lane_change_traj_l.next_y_vals[size/2-1];
                      y2 = lane_change_traj_l.next_y_vals[size/2-2];
                      x1 = lane_change_traj_l.next_x_vals[size/2-1];
                      x2 = lane_change_traj_l.next_x_vals[size/2-2];
                      yaw = atan2(y1 - y2,  x1 - x2);
                    
                      new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                      bool isCollisionMidPoint = predictCollision(new_lane, new_car_frenet[0], sensor_fusion, size, map_waypoints_x, map_waypoints_y); 
                    
                      //! Do lane change only if no collision is predicted
                      if(!isCollisionEndPoint && !isCollisionMidPoint)
                        cur_lane = new_lane;
                     
                      //double yaw = atan2(lane_change_traj_l.next_y_vals[size-1] - lane_change_traj_l.next_y_vals[size-2], lane_change_traj_l.next_x_vals[size-1] - lane_change_traj_l.next_x_vals[size-2]);
                      //vector<double> new_car_frenet = getFrenet(lane_change_traj_l.next_x_vals[size-1], lane_change_traj_l.next_y_vals[size-1], yaw, map_waypoints_x, map_waypoints_y);
                      //if(!isCollision(new_lane, new_car_frenet[0], sensor_fusion, size))
                      //  lane = new_lane;
                  }
                  break;
                }
            }
          }
          
          //! Generate the trajectory for the new target lane
          STrajectory traj;
          generateTrajectory(car_s, cur_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, traj);

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          /*
          double dist_step_in_m = 0.3;
          for(int count = 0; count < 50; count++)
          {
            double next_s = car_s + (count+1)*dist_step_in_m;
            double next_d = (lane * 4) + 2;
            vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(next_xy[0]);
            next_y_vals.push_back(next_xy[1]);
          }*/


          //! Send the trajectory to the output
          msgJson["next_x"] = traj.next_x_vals;
          msgJson["next_y"] = traj.next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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