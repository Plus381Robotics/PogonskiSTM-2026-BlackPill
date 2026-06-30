/*
 * control.c
 *
 *  Created on: Mar 13, 2026
 *      Author: lazar
 */

#include "main.h"

static void reset_movement();
static void rotate();
static void curve_controller();
static void go_to_xy();
void reset_goal(goal_type *goal_ptr);
static void disassemble();

uint8_t set_goal_reset = 0;

double V_MIN_, V_MAX_, W_MIN_, W_MAX_, V_MIN_ACC_, W_MIN_ACC_, V_MIN_STACKED_;
double L_, L_MIN_, L_MAX_;
double STACKED_TIME_;
double D_TOL_, D_LONG_TOL_, D_PROJ_TOL_, D_SHORT_TOL_;
double d_tol_perc_ = 1.0, phi_tol_perc_ = 1.0, PHI_TOL_;
double eta_;
double v0_, v_max_temp_, w_max_temp_;
double starting_coeff_v_, stopping_coeff_v_, starting_coeff_w_,
		stopping_coeff_w_;
double stopping_distance_ = 0, starting_distance_ = 0; // [m]
double stopping_angle_ = 0, starting_angle_ = 0;       // [rad]
double d_tol_perc_, phi_tol_perc_;
int8_t direction_, obstacle_dir_ = 0;
double x_base_, y_base_, phi_base_, v_base_, w_base_;
double x_ref_, y_ref_, phi_ref_, v_ref_, w_ref_, v_ctrl_, w_ctrl_;
double prev_v_, prev_w_;
double a_, alpha_;
int8_t reg_type_;
double dt_;
double v_right_ref_, v_left_ref_;
double v_right_, v_left_;
double MOTOR_V_MAX_;
double phi_error_, x_error_, y_error_;
int8_t movement_state_ = 0;
double J_MAX_, j_max_temp_, J_MAX_STOP_;
double J_ROT_MAX_, j_rot_max_temp_, J_ROT_MAX_STOP_;
double slowing_coeff_ = 1.0;
int8_t reg_phase_ = 0;
double distance_, distance_proj_;        // [m]
uint8_t obstacle_ = 0;
uint8_t obstacle_status_changed_ = 0;
double P_w_;
unsigned stacked_cnt_ = 0;
double V_SLOWED_MAX_ = 0.75;
volatile pid v_loop, w_loop;
int8_t prev_type = 0;
uint8_t obst_f_ = 0, obst_b_ = 0;
int8_t obst_in_loop_budz_ = 0;

bezier bezier_var_;
bezier *ctrl_bezier_ = &bezier_var_;
double s_, prev_K_;
double dir_phi_offset = 0.0;
// TODO: namesti parametre:
double L_drive_ = 0.18, k_heading_ = 6.0, k_lateral_ = 6.0;

uint8_t get_set_goal_reset() {
	return set_goal_reset;
}

double get_s() {
	return s_;
}

double get_distance() {
	return distance_;
}

void move_init() {
	STACKED_TIME_ = 0.06;

	dt_ = 0.002;
	V_MIN_ = 0.15;
	V_MAX_ = 1.5;
	V_MIN_ACC_ = 1.5;
	V_MIN_STACKED_ = 0.01;
	W_MIN_ = 0.628;
	W_MAX_ = 12.57;
	W_MIN_ACC_ = 3.14;
	V_SLOWED_MAX_ = 0.75;
	MOTOR_V_MAX_ = 1.6;
	L_ = 0.1545;
//		L_MAX_ = 0.1935;
//		L_MIN_ = 0.1155;
	L_MAX_ = 0.1545;
	L_MIN_ = 0.1545;
//	eta_ = 0.01;
//	P_w_ = 16.0;
	P_w_ = 10.0;
	J_MAX_ = 24.0;
	J_MAX_STOP_ = 15.0;
	J_ROT_MAX_ = 300.0;
	J_ROT_MAX_STOP_ = 100.0;
	D_TOL_ = 0.02; // absolute distance from target
	D_PROJ_TOL_ = 0.005; // projected distance from target
	D_LONG_TOL_ = 0.12; // distance before rotation is used fully
	D_SHORT_TOL_ = 0.03; // minimal distance for rotation during translation
	PHI_TOL_ = 0.0157; // absolute angle from target

	v_max_temp_ = V_MAX_;
	w_max_temp_ = W_MAX_;
	j_max_temp_ = J_MAX_;
	j_rot_max_temp_ = J_ROT_MAX_;

	init_pid(&v_loop, 5.0, 0.005, 0.1, 1680, 420);
	init_pid(&w_loop, 20.0, 0.01, 2.0, 1680, 420); // bilo 52, 0.02, 2.8, 420
}

void control_loop() {
	x_base_ = get_x();
	y_base_ = get_y();
	phi_base_ = get_phi();
	w_base_ = get_w();
	v_base_ = get_v();
//	L_ = correct_param(L_, fabs(w_ref_) - fabs(w_base_), eta_, L_MIN_, L_MAX_);
	obstacle_dir_ = 0;
	obst_f_ = obstacle_ & 0b01;
	obst_b_ = (obstacle_ >> 1) & 0b01;

	switch (reg_type_) {
	case -1:
		rotate();
		break;
	case 0:
		v_ref_ = 0;
		w_ref_ = 0;
		reset_movement();
		break;
	case 1:
		go_to_xy();
		break;
	case 2:
		curve_controller();
		break;
	}

	if ((obst_f_ && direction_ == 1) || (obst_b_ && direction_ == -1)) {
		v_ref_ = 0.0;
		reset_pid(&v_loop);
	}

	a_ = (v_base_ - prev_v_) / dt_;
	alpha_ = (w_base_ - prev_w_) / dt_;

	prev_v_ = v_base_;
	prev_w_ = w_base_;

}

static double vel_ramp(double signal, double reference, double acc) {
	double err = reference - signal;

	if (fabs(err) > acc)
		signal += get_sign(err) * acc;
	else
		signal = reference;
	return signal;
}

void velocity_loop() {
	// ulaz: referenca za brzine levog i desnog: v_ref_, w_ref_
	double v_err = v_ref_ - get_v();
	double w_err = w_ref_ - get_w();
	v_ctrl_ = calc_pid(&v_loop, v_err);
	w_ctrl_ = calc_pid(&w_loop, w_err);

//	v_ctrl_ = vel_ramp(v_ctrl_, calc_pid(&v_loop, v_err), 0.05);
//	w_ctrl_ = vel_ramp(w_ctrl_, calc_pid(&w_loop, w_err), 0.15);

	// izlaz pwm
//	v_right_ = vel_ramp(v_right_, v_ctrl_ + w_ctrl_ * L_ * 0.5, 0.1);
//	v_left_ = vel_ramp(v_left_, v_ctrl_ - w_ctrl_ * L_ * 0.5, 0.1);
	v_right_ = v_ctrl_ + w_ctrl_ * L_ * 0.5;
	v_left_ = v_ctrl_ - w_ctrl_ * L_ * 0.5;
	scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);

	pwm_right(v_right_);
	pwm_left(v_left_);
}

double get_v_r() {
	return v_right_;
}

double get_v_l() {
	return v_left_;
}

static void curve_controller() {
	// Init
	if (movement_state_ == 0) {
		movement_state_ = 1;
		dir_phi_offset = (direction_ - 1) * M_PI * 0.5;
		init_bezier(ctrl_bezier_, x_base_, y_base_, phi_base_, x_ref_, y_ref_,
				phi_ref_, 1.0);
		s_ = 0.0;
		prev_K_ = K(ctrl_bezier_, s_);
	}
	// Position on the curve
	s_ = s(ctrl_bezier_, x_base_, y_base_, phi_base_ + dir_phi_offset, v_base_,
			s_, dt_, 0.1);
	// distance calc budz
	x_error_ = x_ref_ - x_base_;
	y_error_ = y_ref_ - y_base_;
	distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);

	vec2 P_cur = P(ctrl_bezier_, s_);
	double exc = P_cur.x - x_base_;
	double eyc = P_cur.y - y_base_;
	vec2 T_cur = T_norm(ctrl_bezier_, s_);
	double tangential_distance = exc * T_cur.x + eyc * T_cur.y;
	if (distance_ < 0.1)
		distance_ = tangential_distance;
	// End condition
	if (distance_ < 0.0 || s_ >= 0.999f)
	{
		movement_state_ = -1;
	}
	// Curve params
	double K_cur = K(ctrl_bezier_, s_);
//	vec2 N_cur = N_norm(ctrl_bezier_, s_);
	double dK_ds = Kdot(ctrl_bezier_, s_);  // Analytical dK/ds
	double ds_dt = v_base_
			/ (sqrt(T_cur.x * T_cur.x + T_cur.y * T_cur.y) + 1e-9);
	double kdot = dK_ds * ds_dt;  // dK/dt = (dK/ds) * (ds/dt)
	// Linear velocity reference
	double v_ff = v_max_temp_
			/ fmax(fabs(1 + L_drive_ / 2 * K_cur),
					fabs(1 - L_drive_ / 2 * K_cur));
	double v_stop = velocity_synthesis(distance_ * direction_, v_base_, a_,
			j_max_temp_, stopping_distance_, v_max_temp_, V_MIN_, dt_, 0.0, 0.0,
			V_SLOWED_MAX_, V_MIN_ACC_);
	double a = j_max_temp_ * dt_;
	double v_kdot = (2.0 * a) / (L_drive_ * fabs(kdot) + 1e-6);
	v_kdot = clamp(v_kdot, 0.4, v_max_temp_);
	v_ref_ = min3(v_ff, v_stop, v_kdot) * direction_;
	// Angular velocity reference
	double w_ff = fabs(v_ref_) * K_cur;
	double ex = x_base_ - P_cur.x;
	double ey = y_base_ - P_cur.y;
	double e_lateral = ex * T_cur.y - ey * T_cur.x;
	double cur_phi_ref = atan2(T_cur.y, T_cur.x) + dir_phi_offset;
	double e_heading = wrap(cur_phi_ref - phi_base_, -M_PI, M_PI);
	double w_fb = k_heading_ * e_heading + k_lateral_ * e_lateral * v_ref_;
	w_ref_ = w_ff + w_fb;
	prev_K_ = K_cur;
}

static void rotate() {
	phi_error_ = wrap(phi_ref_ - phi_base_, -M_PI, M_PI);
	if (fabs(phi_error_) < PHI_TOL_ * phi_tol_perc_
			&& fabs(get_w()) < W_MIN_ * 2.0) {
		movement_state_ = -1;
	}
	double W_MIN_ACC_temp_ = W_MIN_ACC_;
	if (movement_state_ == 0) {
		starting_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(j_rot_max_temp_)
				* starting_coeff_w_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		slowing_coeff_ = clamp(
				pow(fabs(phi_error_) / (starting_angle_ + stopping_angle_),
						2.0 / 3.0), 0.0, 1.0);

		W_MIN_ACC_temp_ = clamp(slowing_coeff_, 0.6, 1.0) * W_MIN_ACC_;
		w_max_temp_ *= slowing_coeff_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		reset_pid(&v_loop);
		reset_pid(&w_loop);
		movement_state_ = 1;
	}
	v_ref_ = 0;
	w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_, j_rot_max_temp_,
			stopping_angle_, w_max_temp_, W_MIN_, dt_, 0.0, 0, 0.0,
			W_MIN_ACC_temp_);
}

static void go_to_xy() {
	if (movement_state_ == 0) {
		movement_state_ = 1;
	}
	double W_MIN_ACC_temp_ = W_MIN_ACC_;

	x_error_ = x_ref_ - x_base_;
	y_error_ = y_ref_ - y_base_;
	phi_error_ = wrap(
			atan2(y_error_, x_error_) - phi_base_
					+ (direction_ - 1) * M_PI * 0.5, -M_PI, M_PI);
	switch (reg_phase_) {
	case 0:
		v_ref_ = 0;
		w_ref_ = 0;
		starting_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(j_rot_max_temp_)
				* starting_coeff_w_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		slowing_coeff_ = clamp(
				pow(fabs(phi_error_) / (starting_angle_ + stopping_angle_),
						2.0 / 3.0), 0.0, 1.0);

		W_MIN_ACC_temp_ = clamp(slowing_coeff_, 0.6, 1.0) * W_MIN_ACC_;
		// Calculate new parameters
		w_max_temp_ *= slowing_coeff_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		reset_pid(&v_loop);
		reset_pid(&w_loop);
		reg_phase_ = 1;
		break;
	case 1:
		if (fabs(phi_error_) < PHI_TOL_ * 3.0 && fabs(get_w()) < W_MIN_ * 2.0) {
			reg_phase_ = 2;
			w_max_temp_ = W_MAX_;
		}
		v_ref_ = 0;
		w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_,
				j_rot_max_temp_, stopping_angle_, w_max_temp_, W_MIN_, dt_, 0.0,
				0, 0.0, W_MIN_ACC_temp_);
		break;
	case 2:
		v_ref_ = 0;
		w_ref_ = 0;
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);

		starting_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(j_max_temp_)
				* starting_coeff_v_;
		stopping_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(J_MAX_STOP_)
				* stopping_coeff_v_;

		slowing_coeff_ = clamp(
				pow(distance_ / (starting_distance_ + stopping_distance_),
						2.0 / 3.0), 0.0, 1.0);

		reset_pid(&v_loop);
		reset_pid(&w_loop);
		// Calculate new parameters
		v_max_temp_ *= slowing_coeff_;
		stopping_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(J_MAX_STOP_)
				* stopping_coeff_v_;

		reg_phase_ = 3;
		break;
	case 3:
		if (direction_ == 1)
			obst_in_loop_budz_ = obst_f_;
		else
			obst_in_loop_budz_ = obst_b_;
		if (distance_proj_ < D_PROJ_TOL_ * d_tol_perc_
				&& fabs(get_v()) < V_MIN_ * 2.0
				&& fabs(get_w()) < W_MIN_ * 2.0) {
			if (fabs(distance_) < D_TOL_ * d_tol_perc_) {
				movement_state_ = -1;
				break;
			} else {
				movement_state_ = -5;
				break;
			}
		} else if (obst_in_loop_budz_ && fabs(v_base_) < V_MIN_ * 2.0
				&& fabs(w_base_) < W_MIN_ * 2.0) {
			movement_state_ = -4;
			break;
		} else if (stacked(STACKED_TIME_, v_base_, V_MIN_STACKED_, 1.0 / dt_,
				&stacked_cnt_)) {
			movement_state_ = -3;
			break;
		}
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);
//		if (obstacle_status_changed_) {
//			v0_ = v_base_;
//			obstacle_status_changed_ = 0;
//		}
		v_ref_ = velocity_synthesis(distance_proj_ * direction_, v_base_, a_,
				j_max_temp_, stopping_distance_, v_max_temp_, V_MIN_, dt_, 0.0,
				obst_in_loop_budz_, V_SLOWED_MAX_, V_MIN_ACC_);
		w_ref_ = P_w_
				* clamp(
						(distance_ - D_SHORT_TOL_)
								/ (D_LONG_TOL_ - D_SHORT_TOL_), 0.0, 1.0)
				* phi_error_;
		break;
	}
}

void move_goal(goal_type *goal) {
	uint8_t new_obstacle = goal->obstacle;
	if (new_obstacle != obstacle_)
		obstacle_status_changed_ = 1;
	else
		obstacle_status_changed_ = 0;
	obstacle_ = goal->obstacle;
	goal->status = movement_state_;
//	if (prev_type == 10 && goal->type != 10)
//		pwm_init();
	if (goal->type == 0) {
		reset_movement();
	} else if (goal->type == 10) {
		disassemble();
	} else if (goal->status == 0) {
		goal->status = 1;
		x_base_ = get_x();
		y_base_ = get_y();
		phi_base_ = get_phi();
		v_max_temp_ = clamp(goal->v_max, V_MIN_, V_MAX_);
		w_max_temp_ = clamp(goal->w_max, W_MIN_, W_MAX_);
		starting_coeff_v_ = clamp(goal->start_coeff_v, 1.0, 10.0);
		stopping_coeff_v_ = clamp(goal->stop_coeff_v, 1.0, 10.0);
		starting_coeff_w_ = clamp(goal->start_coeff_w, 1.0, 10.0);
		stopping_coeff_w_ = clamp(goal->stop_coeff_w, 1.0, 10.0);
		d_tol_perc_ = clamp(goal->distance_tolerance_percentage, 1.0, 10.0);
		phi_tol_perc_ = clamp(goal->angle_tolerance_percentage, 1.0, 10.0);
		direction_ = goal->direction;

		switch (goal->type) {
		// Rotate to Phi
		case -1:
			x_ref_ = x_base_;
			y_ref_ = y_base_;
			phi_ref_ = goal->phi;
			reg_type_ = -1;
			break;
			// Reset
		case 0:
			reset_goal(goal);
			// Move to XY
		case 1:
			x_ref_ = goal->x;
			y_ref_ = goal->y;
			phi_ref_ = 0.0;
			reg_type_ = 1;
			break;
		case 2:
			x_ref_ = goal->x;
			y_ref_ = goal->y;
			phi_ref_ = goal->phi;
			reg_type_ = 2;
			break;
		}
	}
	prev_type = goal->type;
}
//mqjmune
void reset_goal(goal_type *goal_ptr) {
	reset_movement();
	goal_ptr->type = 0;
	goal_ptr->x = get_x();
	goal_ptr->y = get_y();
	goal_ptr->phi = get_phi();
	goal_ptr->direction = 0;
	goal_ptr->v_max = V_MAX_;
	goal_ptr->w_max = V_MIN_;
	goal_ptr->distance_tolerance_percentage = 1.0;
	goal_ptr->angle_tolerance_percentage = 1.0;
	goal_ptr->start_coeff_v = 1.0;
	goal_ptr->start_coeff_w = 1.0;
	goal_ptr->stop_coeff_v = 1.0;
	goal_ptr->stop_coeff_w = 1.0;
	goal_ptr->status = 0;
}

static void reset_movement() {
	obst_in_loop_budz_ = 0;
	movement_state_ = 0;
	stacked_cnt_ = 0;
	x_ref_ = x_base_;
	y_ref_ = y_base_;
	phi_ref_ = phi_base_;
	direction_ = 1;
	j_max_temp_ = J_MAX_;
	j_rot_max_temp_ = J_ROT_MAX_;
	v_max_temp_ = V_MAX_;
	w_max_temp_ = W_MAX_;
	starting_coeff_v_ = 1.0;
	stopping_coeff_v_ = 1.0;
	starting_coeff_w_ = 1.0;
	stopping_coeff_w_ = 1.0;
	d_tol_perc_ = 1.0;
	phi_tol_perc_ = 1.0;
	reg_type_ = 0;
	reg_phase_ = 0;
	reset_pid(&v_loop);
	reset_pid(&w_loop);
}

static void disassemble() {
	movement_state_ = 0;
	stacked_cnt_ = 0;
	x_ref_ = x_base_;
	y_ref_ = y_base_;
	phi_ref_ = phi_base_;
	direction_ = 1;
	j_max_temp_ = J_MAX_;
	j_rot_max_temp_ = J_ROT_MAX_;
	v_max_temp_ = V_MAX_;
	w_max_temp_ = W_MAX_;
	starting_coeff_v_ = 1.0;
	stopping_coeff_v_ = 1.0;
	starting_coeff_w_ = 1.0;
	stopping_coeff_w_ = 1.0;
	d_tol_perc_ = 1.0;
	phi_tol_perc_ = 1.0;
	reg_type_ = 0;
	reg_phase_ = 0;
	reset_pid(&v_loop);
	reset_pid(&w_loop);
	pwm_kill();
	time_stop();
}
