#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#include <fstream>
#include <iostream>


// for convenience
using nlohmann::json;
using std::string;
static int count_ = 0;
static int iteration = 0;


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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  double best_err=10000000;
  std::ofstream outfile;
  outfile.open("twiddle.txt");
  double twiddle_error=0;
  int iteration=0;
  bool if_update_=false;
  int current_index=0;
  bool if_reverse_search=false;
  bool if_update_dp=true;
  bool if_update_checked=false;
  bool if_1st_iter=true;
  bool if_run_update=true;

  PID pid;
  //double p[3] = {0.05, 0.0001, 1.5};
  //double p[3] = {.03, .00031, 1.29};
  double p[3] = {0.059, 0.00029, 1.35};

  //double pl[3]= {0.2,3.05349, 0.000270736 };
  double dp[3] = {.00005, .00001, 0.001};
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(p[0], p[1], p[2]);
  
  h.onMessage([&pid,&dp,&best_err,&p,&twiddle_error,&iteration,&if_update_,&current_index,&if_reverse_search,&if_update_dp,&if_update_checked,&if_1st_iter,&if_run_update](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    double up_CTE_THRESHOLD = 2;
    double THROTTLE_CEIL = .4;
    double THROTTLE_FLOOR = 0.3;
    double SPEED_MAX = 50.0;
    double SPEED_MIN = 0.0;
    double low_THROTTLE = 0.3;
    double throttle_c;
    double throttle_c_inv;
    double max_sample=10000;
    double stable_sample=10;


    throttle_c=0.3;

    double db_tol=0.1;
    double sum_dp=0;



    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          steer_value = pid.Signal();


          if (fabs(cte) > up_CTE_THRESHOLD) {
            // Go slow when the cte is high
            throttle_c = low_THROTTLE-(fabs(cte)-1)*low_THROTTLE/10;
          }
          else {
            // Otherwise, use the inverse of the steering value as the throttle, with a max of 100
            throttle_c = fmin(1 / fabs(steer_value), SPEED_MAX);

            // Normalize the throttle value from [0, 100] to [0.45, 1.0]
            // normalized_x = ((ceil - floor) * (x - minimum))/(maximum - minimum) + floor
            throttle_c = ((THROTTLE_CEIL - THROTTLE_FLOOR) * (throttle_c - SPEED_MIN)) / (SPEED_MAX - SPEED_MIN) + THROTTLE_FLOOR;
//            std::cout << "**** increase throttle ***** " << throttle_c_inv << " normal : " << throttle_c << std::endl;
 
          }
         throttle_c=.33;



          //twiddle algorithm

          if (count_ < max_sample){
            ++count_; 
            // let error become stable for a couple of try
            if (count_ > stable_sample){
              twiddle_error+=fabs(cte);
      //        std::cout<<count_<<" twiddle_error:"<< twiddle_error<<"; cte:"<<cte<<std::endl;
            }

          }else{
            // count to max sample and turn on if_update_ flag 
            count_=0;
            if_update_=true;
          }

          if(if_update_){
            if(if_1st_iter){
              best_err=twiddle_error;
              if_1st_iter=false;
            }
            if(!if_update_checked){
              std::cout<< "run update-b, "<< twiddle_error<<" P  "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] <<std::endl;
          //    std::cout<<"if_update_dp="<<if_update_dp<<std::endl;
              if(if_update_dp){
                p[current_index]=p[current_index]+dp[current_index];
              }
              
              pid.update_param(p[0], p[1], p[2]);
              if_update_checked=true;
              if_run_update=true;
              twiddle_error=0;   
              if_update_=false;         
   //           std::cout<< "run update-b2, "<< twiddle_error<<" P  "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] <<std::endl;
            }else{
              if_run_update=false;
              if_update_checked=false;
              std::cout<< "run update-a, "<<" P  "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] <<" error: "<< twiddle_error <<" b e "<<best_err<<std::endl;
            }
            
          }

   
          // update parameter live
          if(!if_run_update && if_update_){
     //     std::cout<< iteration<< " "<< current_index<<" P before: "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] << " error "<<twiddle_error<<" b e "<<best_err<<std::endl;
            sum_dp=dp[0]+dp[1]+dp[2];
            
          // twiddle 
            if(sum_dp>db_tol){
              
              if(twiddle_error<best_err){
                // update dp to larger
                best_err=twiddle_error;
                dp[current_index] *= 1.1;
                if_update_dp=true;
                std::cout<< iteration<< " "<< current_index<<" forward P "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] << " error "<<twiddle_error<<" b e "<<best_err<<std::endl;
              }
              else{
                std::cout<<"in reverse: "<<" if_reverse_search ="<<if_reverse_search<<std::endl;
                if(!if_reverse_search){
                  // reverse search
                  if_reverse_search=true;
          //        std::cout<<"current_index -b "<< current_index<<"  P "<<p[current_index]<<" dP "<<dp[current_index]<<std::endl;
                  p[current_index]=p[current_index]-2*dp[current_index];
         //         std::cout<<"current_index -a "<< current_index<<"  P "<<p[current_index]<<" dP "<<dp[current_index]<<std::endl;
                  pid.update_param(p[0], p[1], p[2]);
                  if_update_dp=false;
                  std::cout<< iteration<< " "<< current_index<<" reverse P "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp*-2 "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] << " error "<<twiddle_error<<" b e "<<best_err<<std::endl;
                }
                else{
                  if_reverse_search=false;
                  // check reverse search results
                  if(twiddle_error<best_err){
                    best_err=twiddle_error;
                    dp[current_index] *= 1.1;
                    std::cout<< iteration<< " "<< current_index<<" reverse P "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp*1.1 "<<dp[0]<<" "<<dp[1]<<" "<<dp[2] << " error "<<twiddle_error<<" b e "<<best_err<<std::endl;
                  }
                  else{
                    p[current_index]=p[current_index]+dp[current_index];
                    pid.update_param(p[0], p[1], p[2]);
                    dp[current_index] *= .9;
                    std::cout<< iteration<< " "<< current_index<<" reverse P "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp*.9 "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<< " error "<<twiddle_error<<" b e "<<best_err<<std::endl;
                  }
                  if_update_dp=true;
                }
              }
            }
            else{
              std::cout<<"sum_dp:"<< sum_dp<<"; db_tol:"<<db_tol<<std::endl;
              std::cout<< " Current parameter is good, don't need to update"<<std::endl;
              dp[0]=0;
              dp[1]=0;
              dp[2]=0;
            }
          //set count and error to zero
            count_=0;
            twiddle_error=0;
            if_update_=false;
            if(if_update_dp){
              if_update_=true;
              //std::cout<< "next_index"<<std::endl;
     //         p[current_index]=p[current_index]+dp[current_index];
    //          pid.update_param(p[0], p[1], p[2]);
              ++current_index;
              if(current_index>2){
                // loop through three index
                current_index=0;
                ++iteration;
              }              
//              std::cout<< iteration<< " "<< current_index<<" update P "<<p[0]<<" "<<p[1]<<" "<<p[2]<<" dp "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<< " error "<<twiddle_error<<" b e "<<best_err<<std::endl;
            }
          }

          //END 
          
          // DEBUG
          // std::cout <<"iteration: "<<iteration<<" count: "<<count_<< " CTE: " << cte << " Steering Value: " << steer_value 
          //           << std::endl;


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_c;
          msgJson["cte"] = cte;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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