#include <zmq.hpp>
#include <string>
#include <iostream>
#include <cmath>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)	Sleep(n)
#endif

#include "params.hpp"


class Car {
	public:
		double x, y, vx, vy;
		Car(double x0, double y0, double vx0, double vy0){
			x = x0;
			y = y0;
			vx = vx0;
			vy = vy0;
		}
		
		void update(double ar, double at){
			ar = (ar > a_r_max_plus) ? a_r_max_plus : ar;
			ar = (ar < -a_r_max_minus) ? -a_r_max_minus: ar;
			at = (at > a_t_max) ? a_t_max : at;
			at = (at < -a_t_max) ? -a_t_max: at;
			double speed = sqrt(vx*vx + vy*vy);
			if (speed < epsilon){
				vy += ar;
				vx += at;
			}
			else {
				double sinv = vy / speed;
				double cosv = vx / speed;
				double dvx = ar * cosv + at * sinv;
				double dvy = ar * sinv - at * cosv;
				vx += dvx;
				vy += dvy;
			}
			double new_v = sqrt(vx*vx + vy*vy);
			if (new_v > v_max){
				vx = vx / new_v * v_max;
				vy = vy / new_v * v_max;
			}
			x += vx;
			y += vy;
		}
};


int main () {
    Car *car = new Car(x_0, y_0, v_x_0, v_y_0);
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REQ);
    socket.connect ("tcp://localhost:5555");

    while (true) {
		Measurement mes;
		mes.x = car->x;
		mes.y = car->y;
		mes.vx = car->vx;
		mes.vy = car->vy;
		std::cout << mes.x << "\t" << mes.y << "\t" << mes.vx << "\t" << mes.vy << std::endl;

		zmq::message_t telemetry ((sizeof(struct Measurement)));
        memcpy (telemetry.data (), &mes, sizeof(struct Measurement));
        socket.send (telemetry);

    	sleep(1);

        zmq::message_t cmd;
        socket.recv (&cmd);
        Command* com = static_cast<Command*>(cmd.data());
        car->update(com->ar, com->at);

    }
    return 0;
}
