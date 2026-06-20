/*
 * curve.c
 *
 *  Created on: Jun 20, 2026
 *      Author: lazar
 */
#define FRENET_ITERATIONS 3
#include "lib.h"

void init_bezier(bezier *bezier_ptr, double x0, double y0, double phi0,
		double x3, double y3, double phi3, double tangent_scale) {
	bezier_ptr->P0.x = x0;
	bezier_ptr->P0.y = y0;
	bezier_ptr->P3.x = x3;
	bezier_ptr->P3.y = y3;

	double dx = x3 - x0;
	double dy = y3 - y0;
	double D = sqrt(dx * dx + dy * dy);
	double d = D * tangent_scale;

	bezier_ptr->P1.x = x0 + d * cos(phi0);
	bezier_ptr->P1.y = y0 + d * sin(phi0);
	bezier_ptr->P2.x = x3 - d * cos(phi3);
	bezier_ptr->P2.y = y3 - d * sin(phi3);
}

vec2 P(bezier *bezier_ptr, double s) {
	double u = 1 - s;
	vec2 P_vec;

	P_vec.x = ((u * u * u) * bezier_ptr->P0.x
			+ 3 * (u * u) * s * bezier_ptr->P1.x
			+ 3 * u * (s * s) * bezier_ptr->P2.x
			+ (s * s * s) * bezier_ptr->P3.x);
	P_vec.y = ((u * u * u) * bezier_ptr->P0.y
			+ 3 * (u * u) * s * bezier_ptr->P1.y
			+ 3 * u * (s * s) * bezier_ptr->P2.y
			+ (s * s * s) * bezier_ptr->P3.y);

	return P_vec;
}

double dx(bezier *bezier_ptr, double s) {
	double u = 1 - s;
	return (3 * u * u * (bezier_ptr->P1.x - bezier_ptr->P0.x)
			+ 6 * u * s * (bezier_ptr->P2.x - bezier_ptr->P1.x)
			+ 3 * s * s * (bezier_ptr->P3.x - bezier_ptr->P2.x));
}

double dy(bezier *bezier_ptr, double s) {
	double u = 1 - s;
	return (3 * u * u * (bezier_ptr->P1.y - bezier_ptr->P0.y)
			+ 6 * u * s * (bezier_ptr->P2.y - bezier_ptr->P1.y)
			+ 3 * s * s * (bezier_ptr->P3.y - bezier_ptr->P2.y));
}

vec2 T_norm(bezier *bezier_ptr, double s) {
	vec2 T;
	T.x = dx(bezier_ptr, s);
	T.y = dy(bezier_ptr, s);
	double norm = sqrt(T.x * T.x + T.y * T.y) + 1e-9;
	T.x /= norm;
	T.y /= norm;

	return T;
}

vec2 N_norm(bezier *bezier_ptr, double s) {
	vec2 N;
	vec2 T = T_norm(bezier_ptr, s);
	N.x = -T.y;
	N.y = T.x;
	return N;
}

double K(bezier *bezier_ptr, double s) {
	double u = 1 - s;

	double ddx = 6 * u
			* (bezier_ptr->P2.x - 2 * bezier_ptr->P1.x + bezier_ptr->P0.x)
			+ 6 * s
					* (bezier_ptr->P3.x - 2 * bezier_ptr->P2.x
							+ bezier_ptr->P1.x);
	double ddy = 6 * u
			* (bezier_ptr->P2.y - 2 * bezier_ptr->P1.y + bezier_ptr->P0.y)
			+ 6 * s
					* (bezier_ptr->P3.y - 2 * bezier_ptr->P2.y
							+ bezier_ptr->P1.y);
	vec2 dPoint;
	dPoint.x = dx(bezier_ptr, s);
	dPoint.y = dy(bezier_ptr, s);

	double num = dPoint.x * ddy - dPoint.y * ddx;
	double den = pow((dPoint.x * dPoint.x + dPoint.y * dPoint.y), 1.5) + 1e-9;
	return num / den;
}

double Frenet(bezier *bezier_ptr, double x, double y, double s0) {
	double s = s0;
	for (uint8_t i = 0; i < FRENET_ITERATIONS; i++) {
		vec2 P_vec = P(bezier_ptr, s);
		vec2 T = T_norm(bezier_ptr, s);
		double f = (P_vec.x - x) * T.x + (P_vec.y - y) * T.y;
		double eps = 5e-4;
		// [1e-5, 1e-5]
		double s2 = clamp(s + eps, 0.0, 1.0);
		vec2 P_2 = P(bezier_ptr, s2);
		vec2 T_2 = T_norm(bezier_ptr, s2);
		double f2 = (P_2.x - x) * T_2.x + (P_2.y - y) * T_2.y;
		double df = (f2 - f) / eps;
		s -= f / (df + 1e-9);
		s = clamp(s, 0.0, 1.0);
	}
	return s;
}

// alpha default is 0.1
double s(bezier *bezier_ptr, double x, double y, double phi, double v,
		double s_prev, double dt, double alpha) {
	double s_raw = Frenet(bezier_ptr, x, y, s_prev);
	vec2 T = T_norm(bezier_ptr, s_raw);
	double deltaSback = alpha * fabs(v * cos(phi) * T.x + v * sin(phi) * T.y)
			* dt;
	return clamp(s_raw, fmax(0.0, s_prev - deltaSback), 1.0);
}
