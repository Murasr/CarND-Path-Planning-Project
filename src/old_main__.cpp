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
double ref_vel = 1.0;

bool isCollision(int f_lane, double car_s, vector<vector<double>> sensor_fusion, int previousPathSize)
{
  bool isCollidable = false;
  
  for(int i=0; i<sensor_fusion.size(); i++)
  {
    double d = sensor_fusion[i][6];

    double ego_d = f_lane*4 + 2;

    if( (d > (ego_d - 2)) && (d < (ego_d + 2)) )
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double veh_speed = sqrt(vx * vx + vy * vy);

      double s = sensor_fusion[i][5];
      s += previousPathSize * 0.02 * veh_speed;

      if( (s > car_s) && (s - car_s) < horizon_front )
      {
        isCollidable = true;
        break;
      }
    }
  }
  
  
  return isCollidable;
}

struct STrajectory
{
  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

void generateTrajectory(double car_s, int f_lane, vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double> spline_xs, vector<double> spline_ys, vector<double>& previous_path_x, vector<double>& previous_path_y, STrajectory& f_traj)
{
   //! Horizon of 150 metres
  vector<double> wp1 = getXY(car_s + horizon_front, (f_lane*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> wp2 = getXY(car_s + horizon_front*2, (f_lane*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> wp3 = getXY(car_s + horizon_front*3, (f_lane*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  spline_xs.push_back(wp1[0]);
  spline_xs.push_back(wp2[0]);
  spline_xs.push_back(wp3[0]);

  spline_ys.push_back(wp1[1]);
  spline_ys.push_back(wp2[1]);
  spline_ys.push_back(wp3[1]);

  double ref_x = spline_xs[1];
  double ref_y = spline_ys[1];
  double ref_yaw = atan2(ref_y - spline_ys[0], ref_x - spline_xs[0]);

  for(int count = 0; count < spline_xs.size(); count++)
  {
    double trans_x = spline_xs[count] - ref_x;
    double trans_y = spline_ys[count] - ref_y;

    spline_xs[count] = trans_x * cos(-ref_yaw) - trans_y * sin(-ref_yaw);
    spline_ys[count] = trans_x * sin(-ref_yaw) + trans_y * cos(-ref_yaw);
  }

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

  double N = dist/(0.02 * ref_vel / 2.237);
  double x_step = x/N;

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
  
  int lane = 1;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
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

          
          if(previousPathSize > 0)
            car_s = end_path_s;
          
          bool increase_speed = true;
          bool isCollisionExpected = isCollision(lane, car_s, sensor_fusion, previousPathSize);
          increase_speed = !isCollisionExpected;
          
          if(increase_speed && ref_vel < 45)
          {
            ref_vel += ref_vel * 0.025;
          }
          else if(!increase_speed && ref_vel > 20)
          {
            ref_vel -= ref_vel * 0.025;
          }

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
          
          if(ref_vel < 35 && isCollisionExpected)
          {
            switch(lane)
            {
               case 0:
                {
                  int new_lane = 1;
                  STrajectory lane_change_traj_r;
                  generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj_r);
        
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
                      bool isCollisionEndPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size);
 
                      //! collision at midpoint
                      y1 = lane_change_traj_r.next_y_vals[size/2-1];
                      y2 = lane_change_traj_r.next_y_vals[size/2-2];
                      x1 = lane_change_traj_r.next_x_vals[size/2-1];
                      x2 = lane_change_traj_r.next_x_vals[size/2-2];
                      yaw = atan2(y1 - y2,  x1 - x2);
                    
                      new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                      bool isCollisionMidPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size); 
                    
                      if(!isCollisionEndPoint && !isCollisionMidPoint)
                        lane = new_lane;
                  }
                  break;
                }
              case 1:
                {
                  int new_lane = 0;
                  
                  {
                    STrajectory lane_change_traj;
                    generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj);

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
                        bool isCollisionEndPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size);

                        //! collision at midpoint
                        y1 = lane_change_traj.next_y_vals[size/2-1];
                        y2 = lane_change_traj.next_y_vals[size/2-2];
                        x1 = lane_change_traj.next_x_vals[size/2-1];
                        x2 = lane_change_traj.next_x_vals[size/2-2];
                        yaw = atan2(y1 - y2,  x1 - x2);

                        new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                        bool isCollisionMidPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size); 

                        if(!isCollisionEndPoint && !isCollisionMidPoint)
                          lane = new_lane;

                        //double yaw = atan2(lane_change_traj.next_y_vals[size-1] - lane_change_traj.next_y_vals[size-2], lane_change_traj.next_x_vals[size-1] - lane_change_traj.next_x_vals[size-2]);
                        //vector<double> new_car_frenet = getFrenet(lane_change_traj.next_x_vals[size-1], lane_change_traj.next_y_vals[size-1], yaw, map_waypoints_x, map_waypoints_y);
                        //if(!isCollision(new_lane, new_car_frenet[0], sensor_fusion, size))
                          //lane = new_lane;
                    }
                  }

                  if(lane != new_lane)
                  {
                    new_lane = 2;
                    STrajectory lane_change_traj;
                    generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj);

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
                        bool isCollisionEndPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size);

                        //! collision at midpoint
                        y1 = lane_change_traj.next_y_vals[size/2-1];
                        y2 = lane_change_traj.next_y_vals[size/2-2];
                        x1 = lane_change_traj.next_x_vals[size/2-1];
                        x2 = lane_change_traj.next_x_vals[size/2-2];
                        yaw = atan2(y1 - y2,  x1 - x2);

                        new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                        bool isCollisionMidPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size); 

                        if(!isCollisionEndPoint && !isCollisionMidPoint)
                          lane = new_lane;

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
                  int new_lane = 1;
                  STrajectory lane_change_traj_l;
                  generateTrajectory(car_s, new_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, lane_change_traj_l);

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
                      bool isCollisionEndPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size);
 
                      //! collision at midpoint
                      y1 = lane_change_traj_l.next_y_vals[size/2-1];
                      y2 = lane_change_traj_l.next_y_vals[size/2-2];
                      x1 = lane_change_traj_l.next_x_vals[size/2-1];
                      x2 = lane_change_traj_l.next_x_vals[size/2-2];
                      yaw = atan2(y1 - y2,  x1 - x2);
                    
                      new_car_frenet = getFrenet(x1, y1, yaw, map_waypoints_x, map_waypoints_y);
                      bool isCollisionMidPoint = isCollision(new_lane, new_car_frenet[0], sensor_fusion, size); 
                    
                      if(!isCollisionEndPoint && !isCollisionMidPoint)
                        lane = new_lane;
                     
                      //double yaw = atan2(lane_change_traj_l.next_y_vals[size-1] - lane_change_traj_l.next_y_vals[size-2], lane_change_traj_l.next_x_vals[size-1] - lane_change_traj_l.next_x_vals[size-2]);
                      //vector<double> new_car_frenet = getFrenet(lane_change_traj_l.next_x_vals[size-1], lane_change_traj_l.next_y_vals[size-1], yaw, map_waypoints_x, map_waypoints_y);
                      //if(!isCollision(new_lane, new_car_frenet[0], sensor_fusion, size))
                      //  lane = new_lane;
                  }
                  break;
                }
            }
          }
          
          STrajectory traj;
          generateTrajectory(car_s, lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, spline_xs, spline_ys, previous_path_x, previous_path_y, traj);

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