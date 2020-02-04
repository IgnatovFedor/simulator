const double v_max = 1.0;
const double a_r_max_plus = 1.0;
const double a_r_max_minus = 1.0;
const double a_t_max = 1.0;
const double delta_ar = 0.0,
             delta_at = 0.0,
             delta_xy = 0.0,
             delta_v = 0.0;

const double x_0 = 0.0,
             y_0 = 0.0,
             v_x_0 = 0.0,
             v_y_0 = 0.0,
             y_c = 10.0,
             x_c = 10.0;

const double epsilon = 1e-12;

struct Measurement {
	double x;
	double y;
	double vx;
	double vy;
};

struct Command {
	double ar;
	double at;
};
