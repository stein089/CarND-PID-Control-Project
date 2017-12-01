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

const int RUNS = 10;
const int N_PER_RUN = 50;


double best_error = -1;
double p [3] = {0.2, 0, 1}; 
double dp [3] = { 0.2, 0.001, 1}; 
double best_p [3] = { -1, -1, -1}; 

int twiddle_index = 0;
int count = 0;
int run_count = 0;

bool flag_first_try = true;
bool disable_messages = true;


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


int main()
{
  uWS::Hub h;
  PID steer_pid;
  PID throttle_pid;

  steer_pid.Init(p[0], p[1], p[2]);
  throttle_pid.Init(0.05, 0.005, 0);

  h.onMessage([&steer_pid, &throttle_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (disable_messages)
      return;

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
        //  double angle = std::stod(j[1]["steering_angle"].get<std::string>());

        //  std::cout << "Read! CTE: " << cte << " Speed: " << speed <<" Angle: " << angle << std::endl;
        //  std::cout << "Total Error " << steer_pid.TotalError() << std::endl;

          steer_pid.UpdateError(cte);
          count++;
          double steer_value = -steer_pid.Kp * steer_pid.p_error - steer_pid.Kd * steer_pid.d_error - steer_pid.Ki * steer_pid.i_error;
          if (steer_value < -1)
            steer_value = -1;
          if (steer_value > 1)
            steer_value = 1;

          double set_speed = 20;

          throttle_pid.UpdateError(speed-set_speed);
          double throttle_value = -throttle_pid.Kp * throttle_pid.p_error - throttle_pid.Kd * throttle_pid.d_error - throttle_pid.Ki * throttle_pid.i_error;


          if (count > N_PER_RUN)
          {
            count = 0;
            run_count++;
            double total_error = steer_pid.TotalError();
            std::cout << "TotalError: " << total_error << std::endl;


            if (flag_first_try)   // First try
            {
              if (total_error < best_error || best_error < 0){
                best_error = total_error;
                std::copy(p,p + 3, best_p);
                dp[twiddle_index%3] = dp[twiddle_index%3] * 1.1;
                p[twiddle_index%3] = p[twiddle_index%3] + dp[twiddle_index%3];
                steer_pid.Init(p[0], p[1], p[2]);
                std::cout << "NEW RUN - twiddle_index = " << twiddle_index << std::endl;
                std::cout << "Kp=" << p[0] << "  Ki=" << p[1] << "  Kd=" << p[2] << std::endl;
                twiddle_index++;
              }
              else
              {
                p[twiddle_index%3] = p[twiddle_index%3] - 2 * dp[twiddle_index%3];
                steer_pid.Init(p[0], p[1], p[2]);
                std::cout << "NEW RUN - twiddle_index = " << twiddle_index << std::endl;
                std::cout << "Kp=" << p[0] << "  Ki=" << p[1] << "  Kd=" << p[2] << std::endl;
                flag_first_try = false; // go to second try after next iteration
              }
            }
            else
            {
              if (total_error < best_error || best_error < 0){
                best_error = total_error;
                std::copy(p,p + 3, best_p);
                dp[twiddle_index%3] = dp[twiddle_index%3] * 1.1;
                p[twiddle_index%3] = p[twiddle_index%3] + dp[twiddle_index%3];
                steer_pid.Init(p[0], p[1], p[2]);
                std::cout << "NEW RUN - twiddle_index = " << twiddle_index << std::endl;
                std::cout << "Kp=" << p[0] << "  Ki=" << p[1] << "  Kd=" << p[2] << std::endl;
                flag_first_try = true;
                twiddle_index++;
              }
              else  // second try
              {
                p[twiddle_index%3] = p[twiddle_index%3] + dp[twiddle_index%3];
                dp[twiddle_index%3] = dp[twiddle_index%3] * 0.9;
                p[twiddle_index%3] = p[twiddle_index%3] + dp[twiddle_index%3];
                steer_pid.Init(p[0], p[1], p[2]);
                std::cout << "NEW RUN - twiddle_index = " << twiddle_index << std::endl;
                std::cout << "Kp=" << p[0] << "  Ki=" << p[1] << "  Kd=" << p[2] << std::endl;
                flag_first_try = true; 
                twiddle_index++;
              }
            }

            if (run_count > RUNS) {
              std::cout << "FINISHED RUNS!!!"  << std::endl;
              std::cout << "Best Error: " << best_error << std::endl;
              std::cout << "Kp=" << best_p[0] << "  Ki=" << best_p[1] << "  Kd=" << best_p[2] << std::endl;
              exit(1);
            }


            throttle_pid.Init(0.05, 0.005, 0);
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            disable_messages = true;  // wait until messages work again
          }
          else  // during normal operation - send update values
          {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
           // msgJson["speed"] = 5.0;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
           // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
    disable_messages = false;
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
