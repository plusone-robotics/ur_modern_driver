#include "ur_modern_driver/ros/trajectory_follower.h"
#include <endian.h>
#include <cmath>
#include <ros/ros.h>

static const int32_t MULT_JOINTSTATE_ = 1000000;
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string TRAJECTORY_REPLACE("{{TRAJECTORY_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def driverProg():
	MULT_jointstate = {{JOINT_STATE_REPLACE}}

	SERVO_IDLE = 0
	SERVO_RUNNING = 1
	cmd_servo_state = SERVO_IDLE
	cmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	
	def set_servo_setpoint(q):
		enter_critical
		cmd_servo_state = SERVO_RUNNING
		cmd_servo_q = q
		exit_critical
	end
	
	thread servoThread():
		state = SERVO_IDLE
		while True:
			enter_critical
			q = cmd_servo_q
			do_brake = False
			if (state == SERVO_RUNNING) and (cmd_servo_state == SERVO_IDLE):
				do_brake = True
			end
			state = cmd_servo_state
			cmd_servo_state = SERVO_IDLE
			exit_critical
			if do_brake:
				stopj(1.0)
				sync()
			elif state == SERVO_RUNNING:
				servoj(q, {{SERVO_J_REPLACE}})
			else:
				sync()
			end
		end
	end

  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread_servo = run servoThread()
  keepalive = 1
  while keepalive > 0:
	  params_mult = socket_read_binary_integer(6+1)
	  if params_mult[0] > 0:
		  q = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
		  keepalive = params_mult[7]
		  set_servo_setpoint(q)
	  end
  end
  sleep(.1)
  socket_close()
  kill thread_servo
end
)";

static const std::string TRAJECTORY_PROGRAM = R"(
def driverProg():
  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

{{TRAJECTORY_REPLACE}}

  socket_close()
end
)";

TrajectoryFollower::TrajectoryFollower(URCommander &commander, std::string &reverse_ip, int reverse_port,
                                       bool version_3)
  : running_(false)
  , commander_(commander)
  , server_(reverse_port)
  , reverse_ip_(reverse_ip)
  , reverse_port_(reverse_port)
  , servoj_time_(0.008)
  , servoj_lookahead_time_(0.03)
  , servoj_gain_(300.)
  , max_acceleration_(4.0 * M_PI)
{
  ros::param::get("~servoj_time", servoj_time_);
  ros::param::get("~servoj_lookahead_time", servoj_lookahead_time_);
  ros::param::get("~servoj_gain", servoj_gain_);
  ros::param::get("~max_acceleration", max_acceleration_); 

  std::string res(POSITION_PROGRAM);
  res.replace(res.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));

  std::ostringstream out;
  out << "t=" << std::fixed << std::setprecision(4) << servoj_time_;
  if (version_3)
    out << ", lookahead_time=" << servoj_lookahead_time_ << ", gain=" << servoj_gain_;

  res.replace(res.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());
  res.replace(res.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), reverse_ip);
  res.replace(res.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  program_ = res;
}

bool TrajectoryFollower::start()
{
  if (running_)
    return true;  // not sure

  LOG_INFO("Uploading trajectory program to robot");

  if (!commander_.uploadProg(program_))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }

  LOG_DEBUG("Awaiting incoming robot connection");

  if (!server_.accept())
  {
    LOG_ERROR("Failed to accept incoming robot connection");
    return false;
  }

  LOG_DEBUG("Robot successfully connected");
  return (running_ = true);
}

bool TrajectoryFollower::computeVelocityAndAccel(double dphi, double dt,
						 double max_vel, double max_accel,
						 double& vel, double& accel)
{

  if (dt < 1e-4)
  {
    // TODO: There may be a case where this calls for maximum effort, but in
    //       many cases it may just be for two very closely spaced positions
    vel = 1.0;
    accel = 1.0;
    return true;
  }
  // Compute velocity and acceleration for trapezoidal velocity profile which
  // meets the target angular displacement, dphi (stop-start-stop), in the
  // specified time, dt.
  
  const double min_accel = 1.0;
  
  // If acceleration limits aren't exceeded, use 10% accel/decel durations
  accel = dphi / (0.09 * dt*dt);
  if (accel > max_accel)
  {
    // if acceleration limit is exceeded, use the max accel
    accel = max_accel;
    if (accel - 4 * dphi / dt < 0)
    {
      // If the max accel is not sufficient to achieve the displacement set the
      // acceleration and velocity to the max and return false
      double min_req_accel = 4 * dphi / dt;
      ROS_ERROR("Minimum acceleration required for move: %lf, current maximum "
		"acceleration: %lf", min_req_accel, max_accel);
      vel = max_vel;
      return false;
    }
  }
  else if (accel < min_accel)
  {
    // But don't use an accel less than a minimum
    accel = min_accel;
  }

  // Compute the velocity for the constant velocity portion of the trapezoidal
  // velocity profile.
  vel = accel*dt/2.0 - std::sqrt(accel*accel*dt*dt/4.0 - accel*dphi);
  return true;
}

bool TrajectoryFollower::startSmoothTrajectory(const std::vector<TrajectoryPoint> &trajectory)
{
  using namespace std::chrono;
  typedef duration<double> double_seconds;

  std::string program(TRAJECTORY_PROGRAM);
  program.replace(program.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), reverse_ip_);
  program.replace(program.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port_));

  // Replace with joints
  auto total_time = 0.0;
  auto previous_point = trajectory[0];
  auto blend_radius = 0.05; // in meters

  const double MAX_VELOCITY = M_PI;  // 180 deg/s (from the manual for e-series)

  std::ostringstream trajectory_str;
  
  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    auto point = trajectory[i];
    if (i == 0)
    {
      auto duration = point.time_from_start;
      auto duration_seconds = duration_cast<double_seconds>(duration).count();
      auto p = point.positions;
      trajectory_str << "  movej([" << p[0] << ", " << p[1] << ", "
        << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "], v="
        << 1.0 << ", r=" << 0.0 << ")" << '\n';
      previous_point = point;
      total_time += duration_seconds;
    }
    else if (i == (trajectory.size() - 1))
    {
      auto duration = point.time_from_start - previous_point.time_from_start;
      auto duration_seconds = duration_cast<double_seconds>(duration).count(); 
      auto p = point.positions;

      double v;
      double a;
      double dphi = 0;
      auto p0 = trajectory[i-1].positions;
      for (size_t j = 0; j < 6; ++j)
      {
	double da = std::abs(p[j] - p0[j]);
	if (da > dphi)
	{
	  dphi = da;
	}
      }
      
      computeVelocityAndAccel(dphi, duration_seconds,
			      MAX_VELOCITY, max_acceleration_, v, a);
      
      trajectory_str << "  movej([" << p[0] << ", " << p[1] << ", "
        << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "], a="
	<< a << ", v=" << v << ", r=" << 0.0 << ")" << '\n';
      previous_point = point;
      total_time += duration_seconds;
    }
    else
    {
      auto duration = point.time_from_start - previous_point.time_from_start;
      auto duration_seconds = duration_cast<double_seconds>(duration).count();
      auto p = point.positions;

      double v;
      double a;
      double dphi = 0;
      auto p0 = trajectory[i-1].positions;
      for (size_t j = 0; j < 6; ++j)
      {
	double da = std::abs(p[j] - p0[j]);
	if (da > dphi)
	{
	  dphi = da;
	}
      }
      
      computeVelocityAndAccel(dphi, duration_seconds,
			      MAX_VELOCITY, max_acceleration_, v, a);
      
      trajectory_str << "  movej([" << p[0] << ", " << p[1] << ", "
        << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "], a="
	<< a << ", v=" << v << ", r=" << blend_radius << ")" << '\n';

      previous_point = point;
      total_time += duration_seconds;
    }
  }
  trajectory_str << "  stopj(3.14)" << '\n';

  program.replace(
    program.find(TRAJECTORY_REPLACE),
    TRAJECTORY_REPLACE.length(),
    trajectory_str.str());

  ROS_INFO_STREAM(program);

  server_.bind();

  commander_.uploadProg(program);

  // Using a thread for the server accept() allows us to timeout the
  // connection if it is hung, without changing the approach to use select()
  // or poll().
  timeout_canceled_ = false;
  server_thread_ = std::thread(&TrajectoryFollower::serverThread, this);

  ROS_INFO_STREAM("Total Trajectory Time: " << total_time);

  // Check for a timeout on the server connection. The timeout will be canceled
  // immediately if the connection can be made. The connection will hang if the
  // robot is in local/pendant control mode.
  ros::Time timeout = ros::Time::now() + ros::Duration(1.0);
  while (ros::Time::now() < timeout)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timeout_canceled_)
    {
      server_thread_.join();
      break;
    }
    usleep(10);
  }

  // The trajectory process is locked
  if (!timeout_canceled_)
  {
    ROS_ERROR("Trajectory execution process has hung.");
    return false;
  }
  
  return (running_ = true);
}

void TrajectoryFollower::serverThread()
{
  // The following server_.accept() is not strictly necessary, but without it
  // the robot pauses for a noticeable amount of time before executing the 
  // trajectory. With this, it executes immediately.
  server_.accept();
  // If accept() returns, then we can just return, but first make sure that the
  // timeout has been canceled.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_canceled_ = true;
  }
}

bool TrajectoryFollower::startTimedTrajectory(const std::vector<TrajectoryPoint> &trajectory)
{
  using namespace std::chrono;
  typedef duration<double> double_seconds;

  std::string program(TRAJECTORY_PROGRAM);
  program.replace(program.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), reverse_ip_);
  program.replace(program.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port_));

  // Replace with joints
  auto total_time = 0.0;
  auto previous_point = trajectory[0];
  auto blend_radius = 0.0;  // blend_radius doesn't do anything in a timed trajectory
  std::ostringstream trajectory_str;
  
  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    auto point = trajectory[i];
    if (i == 0)
    {
      auto duration = point.time_from_start;
      auto duration_seconds = duration_cast<double_seconds>(duration).count();
      auto p = point.positions;
      trajectory_str << "  movej([" << p[0] << ", " << p[1] << ", "
        << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "], t="
        << duration_seconds << ", r=" << blend_radius << ")" << '\n';
      ROS_ERROR_STREAM(duration_seconds);
      previous_point = point;
      total_time += duration_seconds;
    }
    else if (i == (trajectory.size() - 1) || i == (trajectory.size() - 2))
    {
      auto duration = point.time_from_start - previous_point.time_from_start;
      auto duration_seconds = duration_cast<double_seconds>(duration).count();
      auto p = point.positions;
      trajectory_str << "  movej([" << p[0] << ", " << p[1] << ", "
        << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "], t="
        << duration_seconds << ", r=" << 0.0 << ")" << '\n';
      ROS_ERROR_STREAM(duration_seconds);
      previous_point = point;
      total_time += duration_seconds;
    }
    else
    {
      auto duration = point.time_from_start - previous_point.time_from_start;
      auto duration_seconds = duration_cast<double_seconds>(duration).count();
      auto p = point.positions;
      trajectory_str << "  movej([" << p[0] << ", " << p[1] << ", "
        << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "], t="
        << duration_seconds << ", r=" << blend_radius << ")" << '\n';
      ROS_ERROR_STREAM(duration_seconds);
      previous_point = point;
      total_time += duration_seconds;
    }
  }
  trajectory_str << "  stopj(3.14)" << '\n';

  program.replace(
    program.find(TRAJECTORY_REPLACE),
    TRAJECTORY_REPLACE.length(),
    trajectory_str.str());

  ROS_INFO_STREAM(program);

  server_.bind();

  commander_.uploadProg(program);

  // Using a thread for the server accept() allows us to timeout the
  // connection if it is hung, without changing the approach to use select()
  // or poll().
  timeout_canceled_ = false;
  server_thread_ = std::thread(&TrajectoryFollower::serverThread, this);

  ROS_INFO_STREAM("Total Trajectory Time: " << total_time);

  // Check for a timeout on the server connection. The timeout will be canceled
  // immediately if the connection can be made. The connection will hang if the
  // robot is in local/pendant control mode.
  ros::Time timeout = ros::Time::now() + ros::Duration(1.0);
  while (ros::Time::now() < timeout)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timeout_canceled_)
    {
      break;
    }
  }

  // The trajectory process is locked
  if (!timeout_canceled_)
  {
    ROS_ERROR("Trajectory execution process has hung.");
    return false;
  }
  
  ros::Duration(total_time * 1.05).sleep();
  
  return (running_ = true);
}


bool TrajectoryFollower::execute(std::array<double, 6> &positions, bool keep_alive)
{
  if (!running_)
    return false;

  //  LOG_INFO("servoj([%f,%f,%f,%f,%f,%f])", positions[0], positions[1], positions[2], positions[3], positions[4],
  //  positions[5]);

  last_positions_ = positions;

  uint8_t buf[sizeof(uint32_t) * 7];
  uint8_t *idx = buf;

  for (auto const &pos : positions)
  {
    int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE_);
    val = htobe32(val);
    idx += append(idx, val);
  }

  int32_t val = htobe32(static_cast<int32_t>(keep_alive));
  append(idx, val);

  size_t written;
  return server_.write(buf, sizeof(buf), written);
}

double TrajectoryFollower::interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel)
{
  using std::pow;
  double a = p0_pos;
  double b = p0_vel;
  double c = (-3 * a + 3 * p1_pos - 2 * T * b - T * p1_vel) / pow(T, 2);
  double d = (2 * a - 2 * p1_pos + T * b + T * p1_vel) / pow(T, 3);
  return a + b * t + c * pow(t, 2) + d * pow(t, 3);
}

bool TrajectoryFollower::execute(std::array<double, 6> &positions)
{
  return execute(positions, true);
}

bool TrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
  if (!running_)
    return false;

  using namespace std::chrono;
  typedef duration<double> double_seconds;
  typedef high_resolution_clock Clock;
  typedef Clock::time_point Time;

  auto &last = trajectory[trajectory.size() - 1];
  auto &prev = trajectory[0];

  Time t0 = Clock::now();
  Time latest = t0;

  std::array<double, 6> positions;

  for (auto const &point : trajectory)
  {
    // skip t0
    if (&point == &prev)
      continue;

    if (interrupt)
      break;

    auto duration = point.time_from_start - prev.time_from_start;
    double d_s = duration_cast<double_seconds>(duration).count();

    // interpolation loop
    while (!interrupt)
    {
      latest = Clock::now();
      auto elapsed = latest - t0;

      if (point.time_from_start <= elapsed)
        break;

      if (last.time_from_start <= elapsed)
        return true;

      double elapsed_s = duration_cast<double_seconds>(elapsed - prev.time_from_start).count();
      for (size_t j = 0; j < positions.size(); j++)
      {
        positions[j] =
            interpolate(elapsed_s, d_s, prev.positions[j], point.positions[j], prev.velocities[j], point.velocities[j]);
      }

      if (!execute(positions, true))
        return false;

      std::this_thread::sleep_for(std::chrono::milliseconds((int)((servoj_time_ * 1000) / 4.)));
    }

    prev = point;
  }

  // In theory it's possible the last position won't be sent by
  // the interpolation loop above but rather some position between
  // t[N-1] and t[N] where N is the number of trajectory points.
  // To make sure this does not happen the last position is sent
  return execute(last.positions, true);
}

void TrajectoryFollower::stop()
{
  if (!running_)
    return;

  // std::array<double, 6> empty;
  // execute(empty, false);

  server_.disconnectClient();
  running_ = false;
}
