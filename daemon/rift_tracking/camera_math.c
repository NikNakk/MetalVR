// The camera geometry OpenHMD's tracker takes from OpenCV (its rift-sensor-opencv.cpp): undoing and applying a
// sensor's lens distortion, and fitting a device's pose to the LEDs found in a picture. Built without OpenCV,
// OpenHMD compiles these calls away and the tracker can never match a pose, so they are provided here, with the
// same names and meanings, and HAVE_OPENCV is defined for its sources.
//
// The lens models are OpenCV's: for the CV1's sensor the fisheye model (theta_d = theta * (1 + k1 theta^2 +
// k2 theta^4 + k3 theta^6 + k4 theta^8)), for the DK2's the pinhole model with radial and tangential terms
// (k1, k2, p1, p2, k3).
#include <math.h>
#include <string.h>

#include "rift-sensor-opencv.h"

#define MAX_POINTS 64

/* ---- lens models, in normalized image coordinates (the plane at z = 1) ---- */

static void distort(const rift_sensor_camera_params *calib, double a, double b, double *x, double *y)
{
	const double *k = calib->dist_coeffs;
	if (calib->dist_fisheye) {
		const double r = sqrt(a * a + b * b);
		if (r < 1e-12) {
			*x = a;
			*y = b;
			return;
		}
		const double t = atan(r), t2 = t * t;
		const double td = t * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
		*x = a * td / r;
		*y = b * td / r;
	} else {
		const double r2 = a * a + b * b;
		const double radial = 1 + r2 * (k[0] + r2 * (k[1] + r2 * k[4]));
		*x = a * radial + 2 * k[2] * a * b + k[3] * (r2 + 2 * a * a);
		*y = b * radial + k[2] * (r2 + 2 * b * b) + 2 * k[3] * a * b;
	}
}

static void undistort(const rift_sensor_camera_params *calib, double x, double y, double *a, double *b)
{
	const double *k = calib->dist_coeffs;
	if (calib->dist_fisheye) {
		double td = sqrt(x * x + y * y);
		if (td < 1e-12) {
			*a = x;
			*b = y;
			return;
		}
		if (td > M_PI / 2)
			td = M_PI / 2;
		/* Newton's method on theta_d(theta) */
		double t = td;
		for (int i = 0; i < 10; i++) {
			const double t2 = t * t;
			const double f = t * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3])))) - td;
			const double df = 1 + t2 * (3 * k[0] + t2 * (5 * k[1] + t2 * (7 * k[2] + t2 * 9 * k[3])));
			const double step = f / df;
			t -= step;
			if (fabs(step) < 1e-12)
				break;
		}
		const double scale = tan(t) / sqrt(x * x + y * y);
		*a = x * scale;
		*b = y * scale;
	} else {
		/* fixed-point iteration, as OpenCV does */
		double u = x, v = y;
		for (int i = 0; i < 10; i++) {
			const double r2 = u * u + v * v;
			const double radial = 1 + r2 * (k[0] + r2 * (k[1] + r2 * k[4]));
			const double dx = 2 * k[2] * u * v + k[3] * (r2 + 2 * u * u);
			const double dy = k[2] * (r2 + 2 * v * v) + 2 * k[3] * u * v;
			u = (x - dx) / radial;
			v = (y - dy) / radial;
		}
		*a = u;
		*b = v;
	}
}

/* ---- poses ---- */

static void rotation_matrix(const double q[4], double m[9])
{
	const double x = q[0], y = q[1], z = q[2], w = q[3];
	m[0] = 1 - 2 * y * y - 2 * z * z;
	m[1] = 2 * x * y - 2 * w * z;
	m[2] = 2 * x * z + 2 * w * y;
	m[3] = 2 * x * y + 2 * w * z;
	m[4] = 1 - 2 * x * x - 2 * z * z;
	m[5] = 2 * y * z - 2 * w * x;
	m[6] = 2 * x * z - 2 * w * y;
	m[7] = 2 * y * z + 2 * w * x;
	m[8] = 1 - 2 * x * x - 2 * y * y;
}

/* a point of the device, in the camera's frame */
static void transform(const double m[9], const double t[3], const vec3f *p, double out[3])
{
	for (int r = 0; r < 3; r++)
		out[r] = m[r * 3] * p->x + m[r * 3 + 1] * p->y + m[r * 3 + 2] * p->z + t[r];
}

void rift_project_points(rift_led *leds, int num_led_pos, rift_sensor_camera_params *calib, posef *pose,
		vec3f *out_points)
{
	const double *A = calib->camera_matrix.m;
	const double q[4] = { pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w };
	const double t[3] = { pose->pos.x, pose->pos.y, pose->pos.z };
	double m[9];
	rotation_matrix(q, m);

	for (int i = 0; i < num_led_pos; i++) {
		double c[3], x, y;
		transform(m, t, &leds[i].pos, c);
		if (fabs(c[2]) < 1e-9)
			c[2] = 1e-9;
		distort(calib, c[0] / c[2], c[1] / c[2], &x, &y);
		out_points[i].x = (float) (A[0] * x + A[2]);
		out_points[i].y = (float) (A[4] * y + A[5]);
		out_points[i].z = 0;
	}
}

void undistort_points(struct blob *blobs, int num_blobs, vec3f *out_points, rift_sensor_camera_params *calib)
{
	const double *A = calib->camera_matrix.m;
	for (int i = 0; i < num_blobs; i++) {
		double a, b;
		undistort(calib, (blobs[i].x - A[2]) / A[0], (blobs[i].y - A[5]) / A[4], &a, &b);
		out_points[i].x = (float) a;
		out_points[i].y = (float) b;
		out_points[i].z = 1.0;
	}
}

/* ---- fitting a pose to labelled LEDs ---- */

struct fit {
	int count;
	vec3f model[MAX_POINTS];   /* the LED in the device's frame */
	double seen[MAX_POINTS][2]; /* where it is in the picture, undistorted */
	bool used[MAX_POINTS];
};

/* The LEDs of this device that blobs are labelled as, each once */
static void collect(struct fit *fit, struct blob *blobs, int num_blobs, int device_id, rift_led *leds, int num_leds,
		rift_sensor_camera_params *calib)
{
	const double *A = calib->camera_matrix.m;
	uint64_t taken = 0;
	fit->count = 0;
	for (int i = 0; i < num_blobs && fit->count < MAX_POINTS; i++) {
		int led_id = blobs[i].led_id;
		if (LED_OBJECT_ID(led_id) != device_id)
			continue;
		led_id = LED_LOCAL_ID(led_id);
		if (led_id < 0 || led_id >= num_leds || led_id >= 64 || (taken & (1ULL << led_id)))
			continue;
		taken |= 1ULL << led_id;
		fit->model[fit->count] = leds[led_id].pos;
		undistort(calib, (blobs[i].x - A[2]) / A[0], (blobs[i].y - A[5]) / A[4], &fit->seen[fit->count][0],
				&fit->seen[fit->count][1]);
		fit->used[fit->count] = true;
		fit->count++;
	}
}

/* q = exp(d) * q0: the pose turned by the small rotation vector d */
static void turned(const double q0[4], const double d[3], double q[4])
{
	const double angle = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
	double e[4] = { d[0] * 0.5, d[1] * 0.5, d[2] * 0.5, 1.0 };
	if (angle > 1e-9) {
		const double s = sin(angle * 0.5) / angle;
		e[0] = d[0] * s;
		e[1] = d[1] * s;
		e[2] = d[2] * s;
		e[3] = cos(angle * 0.5);
	}
	q[0] = e[3] * q0[0] + e[0] * q0[3] + e[1] * q0[2] - e[2] * q0[1];
	q[1] = e[3] * q0[1] - e[0] * q0[2] + e[1] * q0[3] + e[2] * q0[0];
	q[2] = e[3] * q0[2] + e[0] * q0[1] - e[1] * q0[0] + e[2] * q0[3];
	q[3] = e[3] * q0[3] - e[0] * q0[0] - e[1] * q0[1] - e[2] * q0[2];
	const double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int i = 0; i < 4; i++)
		q[i] /= n;
}

/* The reprojection residuals of the used points, two each; their sum of squares, or a huge number for a pose
 * that puts a point behind the camera */
static double residuals(const struct fit *fit, const double q[4], const double t[3], double *r)
{
	double m[9], sum = 0;
	int n = 0;
	rotation_matrix(q, m);
	for (int i = 0; i < fit->count; i++) {
		if (!fit->used[i])
			continue;
		double c[3];
		transform(m, t, &fit->model[i], c);
		if (c[2] < 1e-6)
			return 1e30;
		const double dx = c[0] / c[2] - fit->seen[i][0], dy = c[1] / c[2] - fit->seen[i][1];
		if (r) {
			r[n++] = dx;
			r[n++] = dy;
		}
		sum += dx * dx + dy * dy;
	}
	return sum;
}

/* Solves the 6x6 system a x = b in place; false when it is singular */
static bool solve6(double a[6][6], double b[6])
{
	for (int col = 0; col < 6; col++) {
		int pivot = col;
		for (int row = col + 1; row < 6; row++)
			if (fabs(a[row][col]) > fabs(a[pivot][col]))
				pivot = row;
		if (fabs(a[pivot][col]) < 1e-18)
			return false;
		if (pivot != col) {
			for (int k = 0; k < 6; k++) {
				const double tmp = a[col][k];
				a[col][k] = a[pivot][k];
				a[pivot][k] = tmp;
			}
			const double tmp = b[col];
			b[col] = b[pivot];
			b[pivot] = tmp;
		}
		for (int row = col + 1; row < 6; row++) {
			const double f = a[row][col] / a[col][col];
			for (int k = col; k < 6; k++)
				a[row][k] -= f * a[col][k];
			b[row] -= f * b[col];
		}
	}
	for (int row = 5; row >= 0; row--) {
		for (int k = row + 1; k < 6; k++)
			b[row] -= a[row][k] * b[k];
		b[row] /= a[row][row];
	}
	return true;
}

/* Levenberg-Marquardt on the six pose parameters (a rotation vector applied to the pose, and its position),
 * from the pose given. Returns false, leaving the pose alone, when it cannot improve it. */
static bool levenberg_marquardt(const struct fit *fit, double q[4], double t[3])
{
	int used = 0;
	for (int i = 0; i < fit->count; i++)
		used += fit->used[i] ? 1 : 0;
	if (used < 4)
		return false;

	double r[MAX_POINTS * 2], rp[MAX_POINTS * 2], rm[MAX_POINTS * 2];
	static const double h = 1e-6;
	double error = residuals(fit, q, t, r);
	if (error >= 1e30)
		return false;
	double lambda = 1e-3;
	bool improved = false;

	for (int iteration = 0; iteration < 30; iteration++) {
		/* the Jacobian by central differences, a column a parameter */
		double J[MAX_POINTS * 2][6];
		bool ok = true;
		for (int p = 0; p < 6 && ok; p++) {
			double d[3] = { 0, 0, 0 }, qp[4], qm[4], tp[3], tm[3];
			memcpy(tp, t, sizeof(tp));
			memcpy(tm, t, sizeof(tm));
			memcpy(qp, q, sizeof(qp));
			memcpy(qm, q, sizeof(qm));
			if (p < 3) {
				d[p] = h;
				turned(q, d, qp);
				d[p] = -h;
				turned(q, d, qm);
			} else {
				tp[p - 3] += h;
				tm[p - 3] -= h;
			}
			if (residuals(fit, qp, tp, rp) >= 1e30 || residuals(fit, qm, tm, rm) >= 1e30)
				ok = false;
			for (int i = 0; i < used * 2; i++)
				J[i][p] = (rp[i] - rm[i]) / (2 * h);
		}
		if (!ok)
			break;

		double JtJ[6][6] = { { 0 } }, Jtr[6] = { 0 };
		for (int i = 0; i < used * 2; i++)
			for (int a = 0; a < 6; a++) {
				Jtr[a] += J[i][a] * r[i];
				for (int b = 0; b < 6; b++)
					JtJ[a][b] += J[i][a] * J[i][b];
			}

		bool stepped = false;
		for (int attempt = 0; attempt < 8 && !stepped; attempt++) {
			double a[6][6], step[6];
			memcpy(a, JtJ, sizeof(a));
			for (int k = 0; k < 6; k++) {
				a[k][k] += lambda * (JtJ[k][k] > 1e-12 ? JtJ[k][k] : 1e-12);
				step[k] = -Jtr[k];
			}
			if (!solve6(a, step)) {
				lambda *= 10;
				continue;
			}
			double qn[4], tn[3] = { t[0] + step[3], t[1] + step[4], t[2] + step[5] };
			turned(q, step, qn);
			const double next = residuals(fit, qn, tn, rp);
			if (next < error) {
				const double gain = error - next;
				memcpy(q, qn, sizeof(qn));
				memcpy(t, tn, sizeof(tn));
				memcpy(r, rp, sizeof(double) * (size_t) used * 2);
				error = next;
				lambda *= 0.1;
				stepped = improved = true;
				if (gain < 1e-14)
					return true;
			} else {
				lambda *= 10;
			}
		}
		if (!stepped)
			break;
	}
	return improved;
}

static void read_pose(const posef *pose, double q[4], double t[3])
{
	q[0] = pose->orient.x;
	q[1] = pose->orient.y;
	q[2] = pose->orient.z;
	q[3] = pose->orient.w;
	t[0] = pose->pos.x;
	t[1] = pose->pos.y;
	t[2] = pose->pos.z;
}

static void write_pose(posef *pose, const double q[4], const double t[3])
{
	pose->orient.x = (float) q[0];
	pose->orient.y = (float) q[1];
	pose->orient.z = (float) q[2];
	pose->orient.w = (float) q[3];
	pose->pos.x = (float) t[0];
	pose->pos.y = (float) t[1];
	pose->pos.z = (float) t[2];
}

/* The pose of a device from the blobs labelled as its LEDs, starting from the pose given, which the tracker
 * always has when it calls this (OpenCV's solvePnPRansac with an extrinsic guess). Points more than three
 * pixels off after a first fit are left out of a second. */
bool estimate_initial_pose(struct blob *blobs, int num_blobs, int device_id, rift_led *leds, int num_leds,
		rift_sensor_camera_params *calib, posef *pose, int *num_leds_out, int *num_inliers,
		bool use_extrinsic_guess)
{
	struct fit fit;
	collect(&fit, blobs, num_blobs, device_id, leds, num_leds, calib);
	if (num_leds_out)
		*num_leds_out = fit.count;
	if (fit.count < 4 || !use_extrinsic_guess)
		return false;

	double q[4], t[3];
	read_pose(pose, q, t);
	levenberg_marquardt(&fit, q, t);

	const double limit = 3.0 / calib->camera_matrix.m[0];
	double m[9];
	int inliers = 0;
	rotation_matrix(q, m);
	for (int i = 0; i < fit.count; i++) {
		double c[3];
		transform(m, t, &fit.model[i], c);
		const double dx = c[0] / c[2] - fit.seen[i][0], dy = c[1] / c[2] - fit.seen[i][1];
		fit.used[i] = c[2] > 1e-6 && dx * dx + dy * dy < limit * limit;
		inliers += fit.used[i] ? 1 : 0;
	}
	if (inliers >= 4 && inliers < fit.count)
		levenberg_marquardt(&fit, q, t);
	if (num_inliers)
		*num_inliers = inliers;

	write_pose(pose, q, t);
	return true;
}

bool refine_pose(struct blob *blobs, int num_blobs, int device_id, rift_led *leds, int num_leds,
		rift_sensor_camera_params *calib, posef *pose)
{
	struct fit fit;
	collect(&fit, blobs, num_blobs, device_id, leds, num_leds, calib);
	if (fit.count < 4)
		return false;
	double q[4], t[3];
	read_pose(pose, q, t);
	levenberg_marquardt(&fit, q, t);
	write_pose(pose, q, t);
	return true;
}
