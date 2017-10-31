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
#include "tools.h"
#include "map.h"
#include "vehicle.h"
#include "vehiclepathplanner.h"

using namespace std;
using json = nlohmann::json;

namespace
{
  /** Checks if the SocketIO event has JSON data.
   * If there is data the JSON object in string format will be returned,
   *  else the empty string "" will be returned.
   */
  string hasData(string s)
  {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
      return s.substr(b1, b2 - b1 + 2);
    }
    return "";
  }
}


//=========================================== Main ===========================================


int main()
{
  uWS::Hub h;
  Map map("../data/highway_map.csv");
  VehiclePathPlanner planner(map, 100.0, Tools::mph2mps(49.5));
  static int lane = 1;
  static double refV = 0.0;

  h.onMessage([&map, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // Own car
          Vehicle car(
            j[1]["x"], j[1]["y"],
            j[1]["s"], j[1]["d"],
            Tools::deg2rad(j[1]["yaw"]), Tools::mph2mps(j[1]["speed"]),
            Trajectory(j[1]["previous_path_x"], j[1]["previous_path_y"]),
            j[1]["end_path_s"], j[1]["end_path_d"]);

          // Other cars on the same side of the road
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<Vehicle> obstacles;
          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            Vehicle v(
              sensor_fusion[i][1], sensor_fusion[i][2],
              sensor_fusion[i][5], sensor_fusion[i][6],
              Tools::vangle(sensor_fusion[i][3], sensor_fusion[i][4]),
              Tools::vabs(sensor_fusion[i][3], sensor_fusion[i][4]));
            obstacles.push_back(v);
          }

          // Update planner
          planner.updateState(car, obstacles);
          Trajectory trajectory = planner.getTrajectory();

          json msgJson;
          msgJson["next_x"] = trajectory.x;
          msgJson["next_y"] = trajectory.y;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
