/*
 * control.c
 *
 *  Created on: Mar 13, 2026
 *      Author: lazar
 */

#include "main.h"

static void reset_movement();
static void rotate();
static void go_to_xy();
static void reset_goal(goal_type *goal_ptr);

double V_MIN_, V_MAX_, W_MIN_, W_MAX_;
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
double x_ref_, y_ref_, phi_ref_, v_ref_, w_ref_;
double prev_v_, prev_w_;
double a_, alpha_;
int8_t reg_type_;
double dt_;
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

void move_init() {
// TODO: vrati na 0.5 ili manje
	STACKED_TIME_ = 100.0;

	dt_ = 0.001;
	V_MIN_ = 0.1;
	V_MAX_ = 1.5;
	W_MIN_ = 0.314;
	W_MAX_ = 9.42;
	V_SLOWED_MAX_ = 0.75;
	MOTOR_V_MAX_ = 2.0;
	L_ = 0.1545;
	L_MAX_ = 0.1935;
	L_MIN_ = 0.1155;
	eta_ = 0.01;
	P_w_ = 10.0;
	J_MAX_ = 25.0;
	J_MAX_STOP_ = 50.0;
	J_ROT_MAX_ = 500.0;
	J_ROT_MAX_STOP_ = 1000.0;
	D_TOL_ = 0.005; // absolute distance from target
	D_PROJ_TOL_ = 0.0025; // projected distance from target
	D_LONG_TOL_ = 0.1; // distance before rotation is used fully
	D_SHORT_TOL_ = 0.05; // minimal distance for rotation during translation
	PHI_TOL_ = 0.0157; // absolute angle from target

	v_max_temp_ = V_MAX_;
	w_max_temp_ = W_MAX_;
	j_max_temp_ = J_MAX_;
	j_rot_max_temp_ = J_ROT_MAX_;
}

void control_loop() {
	x_base_ = get_x();
	y_base_ = get_y();
	phi_base_ = get_phi();
	w_base_ = get_w();
	v_base_ = get_v();
	L_ = correct_param(L_, fabs(w_ref_) - fabs(w_base_), eta_, L_MIN_, L_MAX_);
	obstacle_dir_ = 0;
	switch (reg_type_) {
	case -1:
		rotate();
		break;
	case 0:
		v_ref_ = 0;
		w_ref_ = 0;
		break;
	case 1:
		go_to_xy();
		break;
	}

	v_right_ = v_ref_ + w_ref_ * L_ * 0.5;
	v_left_ = v_ref_ - w_ref_ * L_ * 0.5;
	double scale_factor = scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);
	v_right_ *= scale_factor;
	v_left_ *= scale_factor;
	pwm_right(v_right_);
	pwm_left(v_left_);

	a_ = (v_base_ - prev_v_) / dt_;
	alpha_ = (w_base_ - prev_w_) / dt_;

	prev_v_ = v_base_;
	prev_w_ = w_base_;
}

double get_v_r() {
	return v_right_;
}

double get_v_l() {
	return v_left_;
}

static void rotate() {
	phi_error_ = wrap(phi_ref_ - phi_base_, -M_PI, M_PI);

	if (movement_state_ == 0) {
		starting_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(j_rot_max_temp_)
				* starting_coeff_w_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		slowing_coeff_ = clamp(
				pow(fabs(phi_error_) / (starting_angle_ + stopping_angle_),
						2.0 / 3.0), 0.0, 1.0);

		w_max_temp_ *= slowing_coeff_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		movement_state_ = 1;
	}
	v_ref_ = 0;
	w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_, j_rot_max_temp_,
			stopping_angle_, w_max_temp_, W_MIN_, dt_, 0.0, 0, 0.0);
	if (fabs(phi_error_) < PHI_TOL_ * phi_tol_perc_) {
		reset_movement();
		movement_state_ = -1;
	}
}

static void go_to_xy() {
	if (movement_state_ == 0) {
		movement_state_ = 1;
	}

	x_error_ = x_ref_ - x_base_;
	y_error_ = y_ref_ - y_base_;
	phi_error_ = wrap(
			atan2(y_error_, x_error_) - phi_base_
					+ (direction_ - 1) * M_PI * 0.5, -M_PI, M_PI);
	switch (reg_phase_) {
	case 0:
		starting_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(j_rot_max_temp_)
				* starting_coeff_w_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		slowing_coeff_ = clamp(
				pow(fabs(phi_error_) / (starting_angle_ + stopping_angle_),
						2.0 / 3.0), 0.0, 1.0);

		// Calculate new parameters
		w_max_temp_ *= slowing_coeff_;
		stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_)
				* stopping_coeff_w_;

		reg_phase_ = 1;
		break;
	case 1:
		v_ref_ = 0;
		w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_,
				j_rot_max_temp_, stopping_angle_, w_max_temp_, W_MIN_, dt_, 0.0,
				0, 0.0);
		if (fabs(phi_error_) < PHI_TOL_) {
			reg_phase_ = 2;
			w_max_temp_ = W_MAX_;
		}
		break;
	case 2:
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);

		starting_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(j_max_temp_)
				* starting_coeff_v_;
		stopping_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(J_MAX_STOP_)
				* stopping_coeff_v_;

		slowing_coeff_ = clamp(
				pow(distance_ / (starting_distance_ + stopping_distance_),
						2.0 / 3.0), 0.0, 1.0);

		// Calculate new parameters
		v_max_temp_ *= slowing_coeff_;
		stopping_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(J_MAX_STOP_)
				* stopping_coeff_v_;

		reg_phase_ = 3;
		break;
	case 3:
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);
		if (obstacle_status_changed_) {
			v0_ = v_base_;
			obstacle_status_changed_ = 0;
		}
		v_ref_ = velocity_synthesis(distance_proj_ * direction_, v_base_, a_,
				j_max_temp_, stopping_distance_, v_max_temp_, V_MIN_, dt_, v0_,
				obstacle_, V_SLOWED_MAX_);
		w_ref_ = P_w_
				* clamp(
						(distance_ - D_SHORT_TOL_)
								/ (D_LONG_TOL_ - D_SHORT_TOL_), 0.0, 1.0)
				* phi_error_;

		obstacle_dir_ = get_sign(v_ref_);
		if (distance_proj_ < D_PROJ_TOL_ * d_tol_perc_
				&& fabs(distance_) < D_TOL_ * d_tol_perc_) {
			reset_movement();
			movement_state_ = -1;
		} else if (obstacle_ == 1 && fabs(v_base_) < V_MIN_
				&& fabs(w_base_) < W_MIN_) {
			reset_movement();
			movement_state_ = -4;
		} else if (stacked(STACKED_TIME_, v_base_, V_MIN_, 1.0 / dt_,
				&stacked_cnt_)) {
			reset_movement();
			movement_state_ = -3;
		}
		break;
	}
}

void move_goal(goal_type *goal) {
	goal->status = movement_state_;
	if (goal->status < 0)
		reset_goal(goal);
	else if (goal->status == 0) {
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
			// Rotate to XY
		case -2:
			x_ref_ = x_base_;
			y_ref_ = y_base_;
			phi_ref_ = wrap(
					atan2(goal->y - y_base_, goal->x - x_base_)
							+ (goal->direction - 1) * M_PI * 0.5, -M_PI, M_PI);
			reg_type_ = -1;
			break;
			// Move to XY
		case 1:
			x_ref_ = goal->x;
			y_ref_ = goal->y;
			phi_ref_ = 0.0;
			reg_type_ = 1;
			break;
			// Move on Direction
		case 2:
			x_ref_ = x_base_ + goal->direction * goal->y * cos(phi_base_);
			y_ref_ = y_base_ + goal->direction * goal->y * sin(phi_base_);
			phi_ref_ = phi_base_;
			reg_type_ = 1;
			break;
			// Move on Direction Snapped
		case 3:
			x_ref_ = x_base_
					+ goal->direction * goal->y
							* cos(snap_angle(phi_base_, goal->phi));
			y_ref_ = y_base_
					+ goal->direction * goal->y
							* sin(snap_angle(phi_base_, goal->phi));
			phi_ref_ = snap_angle(phi_base_, goal->phi);
			reg_type_ = 1;
			break;
			// Move on Angle
		case 4:
			x_ref_ = x_base_ + goal->direction * goal->y * cos(goal->phi);
			y_ref_ = y_base_ + goal->direction * goal->y * sin(goal->phi);
			phi_ref_ = goal->phi;
			reg_type_ = 1;
			break;
		}
	}
}

static void reset_goal(goal_type *goal_ptr) {
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
//	goal_ptr->status = 0; // -1 = success; -2 = canceled; -3 = interrupted; -4 = timed out
}

static void reset_movement() {
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
}
