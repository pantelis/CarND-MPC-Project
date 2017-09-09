#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    int ns = 6;
    int na = 2;

    MPC mpc(ns, na);

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    //ptsx and ptsy are the waypoints  (6 waypoints are sent by the simulator in GCS)
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];

                    // (x,y) coordimates of the car are received from the simulator in GCS
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    // psi and velocity
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    v *= 0.44704; //convert miles per hour to meter per second!!!
                    double delta = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];

                    auto num_waypoints = static_cast<unsigned int>(ptsx.size());

                    Eigen::VectorXd waypoints_x_lcs = Eigen::VectorXd::Zero(num_waypoints);
                    Eigen::VectorXd waypoints_y_lcs = Eigen::VectorXd::Zero(num_waypoints);

                    // express the waypoints relative to the car (x,y)
                    for (int i=0; i < num_waypoints; i++){

                        double translate_x = ptsx[i] - px;
                        double translate_y = ptsy[i] - py;

                        waypoints_x_lcs[i] = translate_x * cos(psi) + translate_y * sin(psi);
                        waypoints_y_lcs[i] = translate_y * cos(psi) - translate_x * sin(psi);
                    }

                    // Fit a 3rd order polynomial to the above x and y coordinates
                    auto coeffs = polyfit(waypoints_x_lcs, waypoints_y_lcs, 3);


                    // Predict the state variables at t+100ms (latency)
                    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
                    double latency_ms = 100;
                    double latency = latency_ms/1000.;

                    // Calculate the current cte
                    /* cte = desired_y - actual_y
                            = polyeval(coeffs,px)-py
                            = polyeval(coeffs,0) because px=py=0 */
                    double cte = polyeval(coeffs, 0);

                    // Calculate the current epsi
                    // epsi =  actual psi-desired psi
                    // double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px,2));
                    // = -atan(coeffs[1]), because px=py=0
                    double epsi = -atan(coeffs[1]);

                    delta = -delta; // change of sign because turning left is negative sign in simulator but positive yaw for MPC
                    state[0] = v * cos(psi) * latency; // x: px + v * cos(psi) * latency;
                    state[1] = 0; // y: py + v * sin(psi) * latency; // y: car coordinates were transformed such that the car moves along the x axis in the model
                    state[2] = (v / 2.67)  * delta * latency; // psi: psi + (v / 2.67)  * delta * latency;
                    state[3] = v + throttle * latency;
                    state[4] = cte + v * sin(epsi) * latency;
                    state[5] = epsi + state[2];

                    // Call ipopt solver
                    auto vars = mpc.Solve(state, coeffs);

                    double steering_value = -vars[0]/deg2rad(25.);
                    double throttle_value = vars[1];

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steering_value;
                    msgJson["throttle"] = throttle_value;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i=1; i < num_waypoints; i++){
                        next_x_vals.push_back(waypoints_x_lcs[i]);
                        next_y_vals.push_back(waypoints_y_lcs[i]);
                    }

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    for (int i = 2; i < vars.size(); i++)
                    {
                        if (i%2 == 0)
                        {
                            mpc_x_vals.push_back(vars[i]);
                        } else
                            mpc_y_vals.push_back(vars[i]);
                    }

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
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
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
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
