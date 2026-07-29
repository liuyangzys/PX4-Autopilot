#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uavcan/navigation/GlobalNavigationSolution.hpp>
#include <uavcan/uavcan.hpp>

class UavcanGlobalNavigation
{
public:
	UavcanGlobalNavigation(uavcan::INode &node);

	int init();

private:
	static constexpr unsigned RATE_HZ = 50;

	typedef uavcan::MethodBinder<UavcanGlobalNavigation *, void (UavcanGlobalNavigation::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	void periodic_update(const uavcan::TimerEvent &);

	uavcan::INode &_node;
	uavcan::Publisher<uavcan::navigation::GlobalNavigationSolution> _publisher;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
};
