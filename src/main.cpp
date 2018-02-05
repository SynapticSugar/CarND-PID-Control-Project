#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

static double gTargetSpeed = 40.5;  // maximum vehicle speed
static double g_old_cte = 0.0;
static double g_old_speed = 0.0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  PID pid;
  PID speed_pid;
  // TODO: Initialize the pid variable.
  // 0.236462, 8.32424, 7.17711e-05 from first optimization 3100 steps
  //pid.Init(atof(argv[1]),atof(argv[2]),atof(argv[3]), true, 1e-10);
  //pid.Init(0.275034, 11.7865, 7.85047e-05, true, 1e-10);
  //pid.Init(0.272284, 11.8581, 7.06542e-05, true, 1e-10);
  //speed_pid.Init(0.6, 3, 0.00001, true, 1e-6);
  if (argc > 3) {
    pid.Init(atof(argv[1]), atof(argv[2]), atof(argv[3]), true, 1e-3, "cte");
  }
  else {
    //pid.Init(0.12, 0.11, 1e-5, true, 1e-4, "cte");   // manual tune
    //pid.Init(0.156251, 0.114194, 1.16198e-05, false, 1e-4);
    //pid.Init(0.187446, 0.123924, 1.23201e-05, false, 1e-4, "cte");
    pid.Init(0.193261, 0.101981, 1.17019e-05, false, 1e-4, "cte"); // 40 kph
    //pid.Init(0.297632, 0.139153, 2.84947e-05, false, 1e-4, "cte"); // 50 kph
    //pid.Init(0.205894, 0.106183, 1.98027e-05, false, 1e-4, "cte"); // 60 kph
    //pid.Init(0.10318, 0.0780235, 3.76043e-05, flase, 1e-4, "cte"); // 70 kph
    //pid.Init(0.0850997, 0.0780235, 2.65018e-05, false, 1e-4, "cte"); // 80 kph
  }
  //speed_pid.Init(1.00561, 0.2655, 0.000315668, false, 1e-1);
  //speed_pid.Init(0.378761, 0.1, 0.00015, true, 1e-4);
  //speed_pid.Init(0.416637, 0.105738, 0.000158857, false, 1e-3, "speed");
  //speed_pid.Init(0.833274, 0.211476, 0.000158857, true, 1e-3, "speed");
  speed_pid.Init(0.940517, 0.00378754, 0, false, 1e-3, "speed");

  h.onMessage([&pid,&speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          // not used, avoid compiler warning
          (void)angle;

          double steer_value;
          double throttle_value;
          double max_cte = 4;

          //
          // Steering PID Update
          //
          // simple low pass filter on cte
          cte = .1 * cte + .9 * g_old_cte;
          g_old_cte = cte;
          // update
          pid::return_code code = pid.UpdateError(cte, (abs(cte)>max_cte));
          steer_value  = pid.TotalError();
          // wrap steering between -1 and 1
          /*
          if (steer_value > 1)
            steer_value = 1;
          if (steer_value < -1)
            steer_value = -1;
          */
          //
          // Throttle PID Update
          //
          // simple low pass filter on speed
          speed = .1 * speed + .9 * g_old_speed;
          g_old_speed = speed;
          // update
          speed_pid.UpdateError(speed - gTargetSpeed, (abs(cte)>max_cte));
          throttle_value  = speed_pid.TotalError();
          // wrap throttle between -1 and 1
          /*
          if(throttle_value > 1){
              throttle_value = 1.0;
          }else if(throttle_value < -1){
              throttle_value = -1.0;
          }
          */
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // handle twiddle resets here
          if (code == pid::return_code::RESTART)
          {
            restart(ws);
          } else if (code == pid::return_code::FINISHED) {
            exit(0);
          }

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
