#include <zmq.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include "kalman.hpp"
#include "params.hpp"

int main ()
{
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);

    socket.bind ("tcp://*:5555");
    bool first_time = true;
    
	int n = 3; // Number of states
	int m = 1; // Number of measurements

	double dt = 1.0/30; // Time step

	Eigen::MatrixXd A(n, n); // System dynamics matrix
	Eigen::MatrixXd C(m, n); // Output matrix
	Eigen::MatrixXd Q(n, n); // Process noise covariance
	Eigen::MatrixXd R(m, m); // Measurement noise covariance
	Eigen::MatrixXd P(n, n); // Estimate error covariance
	A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
	C << 1, 0, 0;
	Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
	R << 5;
	P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
	KalmanFilter kf(dt, A, C, Q, R, P);
	Eigen::VectorXd y(m);
    
    while (true) {
        zmq::message_t reply;
        socket.recv (&reply);
        Measurement* mes = static_cast<Measurement*>(reply.data());
		std::cout << mes->x << "\t";
		if (first_time) {
			first_time = false;
			Eigen::VectorXd x0(n);
			x0 << mes->x, 0, -9.81;
			kf.init(0.0, x0);
		}
		y << mes->x;
		kf.update(y);
		std::cout << kf.state().transpose()[0] << std::endl;
		double dx = x_c - mes->x;
		double dy = y_c - mes->y;
		double dl = sqrt(dx*dx+dy*dy);
		double target_vx, target_vy;
		if (dl < v_max){
			target_vx = dx;
			target_vy = dy;
		}
		else {
			target_vx = dx / dl * v_max;
			target_vy = dy / dl * v_max;
		}
		double target_ax = target_vx - mes->vx;
		double target_ay = target_vy - mes->vy;
		double target_a = sqrt(target_ax*target_ax + target_ay*target_ay);
		double speed = sqrt(mes->vx*mes->vx+mes->vy*mes->vy);
		double ar; 
		double at; 
		if (speed > epsilon && target_a > epsilon){
			ar = (mes->vx*target_ax+mes->vy*target_ay) / speed;
			at = (mes->vy*target_ax-mes->vx*target_ay) / speed;
		}
		else {
			ar = target_ay;
			at = target_ax;
		}
		Command comm;
		comm.at = at;
		comm.ar = ar;
		//std::cout << at<<"\t" << ar<< "\t"<< target_ax << "\t" << target_ay << std::endl;
		zmq::message_t cmd ((sizeof(struct Command)));
        memcpy (cmd.data (), &comm, sizeof(struct Command));
        socket.send (cmd);
    }
    return 0;
}
