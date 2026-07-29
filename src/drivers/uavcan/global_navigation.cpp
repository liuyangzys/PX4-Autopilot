#include "global_navigation.hpp"

#include <mathlib/mathlib.h>
#include <matrix/matrix/Quaternion.hpp>

using namespace matrix;

UavcanGlobalNavigation::UavcanGlobalNavigation(uavcan::INode &node) :
	_node(node),
	_publisher(node),
	_timer(node)
{
	_publisher.setPriority(uavcan::TransferPriority::MiddleLower);
}

int UavcanGlobalNavigation::init()
{
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanGlobalNavigation::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / RATE_HZ));
	}

	return 0;
}

void UavcanGlobalNavigation::periodic_update(const uavcan::TimerEvent &)
{
	vehicle_global_position_s global_position{};
	vehicle_local_position_s local_position{};
	vehicle_attitude_s attitude{};
	vehicle_angular_velocity_s angular_velocity{};

	const bool global_position_valid = _vehicle_global_position_sub.copy(&global_position)
					   && global_position.lat_lon_valid
					   && global_position.alt_valid
					   && PX4_ISFINITE(global_position.lon)
					   && PX4_ISFINITE(global_position.lat)
					   && PX4_ISFINITE(global_position.alt_ellipsoid)
					   && PX4_ISFINITE(global_position.alt);

	const bool local_position_available = _vehicle_local_position_sub.copy(&local_position);

	const bool local_velocity_valid = local_position_available
					  && local_position.v_xy_valid
					  && local_position.v_z_valid
					  && PX4_ISFINITE(local_position.vx)
					  && PX4_ISFINITE(local_position.vy)
					  && PX4_ISFINITE(local_position.vz);

	const bool local_height_agl_valid = local_position_available
					    && local_position.dist_bottom_valid
					    && PX4_ISFINITE(local_position.dist_bottom);

	const bool local_acceleration_valid = local_position_available
					      && PX4_ISFINITE(local_position.ax)
					      && PX4_ISFINITE(local_position.ay)
					      && PX4_ISFINITE(local_position.az);

	if (!_vehicle_attitude_sub.copy(&attitude)
	    || !_vehicle_angular_velocity_sub.copy(&angular_velocity)
	    || !PX4_ISFINITE(attitude.q[0]) || !PX4_ISFINITE(attitude.q[1])
	    || !PX4_ISFINITE(attitude.q[2]) || !PX4_ISFINITE(attitude.q[3])
	    || !PX4_ISFINITE(angular_velocity.xyz[0]) || !PX4_ISFINITE(angular_velocity.xyz[1])
	    || !PX4_ISFINITE(angular_velocity.xyz[2])) {
		return;
	}

	const Quatf q_body_to_ned{attitude.q};
	const float nan = NAN;
	const double nan_double = NAN;
	Vector3f velocity_body{nan, nan, nan};
	Vector3f acceleration_body{nan, nan, nan};

	if (local_velocity_valid) {
		velocity_body = q_body_to_ned.rotateVectorInverse(
					Vector3f{local_position.vx, local_position.vy, local_position.vz});
	}

	if (local_acceleration_valid) {
		acceleration_body = q_body_to_ned.rotateVectorInverse(
					Vector3f{local_position.ax, local_position.ay, local_position.az});
	}

	uavcan::navigation::GlobalNavigationSolution message{};
	message.timestamp.usec = _node.getUtcTime().toUSec();
	message.longitude = global_position_valid ? global_position.lon : nan_double;
	message.latitude = global_position_valid ? global_position.lat : nan_double;
	message.height_ellipsoid = global_position_valid ? global_position.alt_ellipsoid : nan;
	message.height_msl = global_position_valid ? global_position.alt : nan;
	message.height_agl = local_height_agl_valid ? local_position.dist_bottom : nan;
	message.height_baro = nan;
	message.qnh_hpa = nan;

	message.orientation_xyzw[0] = attitude.q[1];
	message.orientation_xyzw[1] = attitude.q[2];
	message.orientation_xyzw[2] = attitude.q[3];
	message.orientation_xyzw[3] = attitude.q[0];

	message.linear_velocity_body[0] = velocity_body(0);
	message.linear_velocity_body[1] = velocity_body(1);
	message.linear_velocity_body[2] = velocity_body(2);
	message.angular_velocity_body[0] = angular_velocity.xyz[0];
	message.angular_velocity_body[1] = angular_velocity.xyz[1];
	message.angular_velocity_body[2] = angular_velocity.xyz[2];
	message.linear_acceleration_body[0] = acceleration_body(0);
	message.linear_acceleration_body[1] = acceleration_body(1);
	message.linear_acceleration_body[2] = acceleration_body(2);

	(void)_publisher.broadcast(message);
}
