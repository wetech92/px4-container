/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_DRYDEN_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_DRYDEN_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Wind.pb.h"

class GustModelBase
{
private:
    std::default_random_engine random_generator_;
    std::uniform_real_distribution<double> gust_dist_;

    bool initialized_;
    double dt_;

    double alpha_;
    double beta_;
    double delta_;
    double gamma_;

    double u_km1;
    double u_km2;
    double y_km1;
    double y_km2;

    double run(const double &dt)
    {
        if (initialized_)
        {
            double C1 = 1.0 + 2 * delta_ / dt + 4 * gamma_ / dt / dt;
            double C2 = 2.0 - 8 * gamma_ / dt / dt;
            double C3 = 1.0 - 2 * delta_ / dt + 4 * gamma_ / dt / dt;
            double C4 = alpha_ + 2 * beta_ / dt;
            double C5 = 2 * alpha_;
            double C6 = alpha_ - 2 * beta_ / dt;

            double u_k = gust_dist_(random_generator_);
            double y_k = (C4 * u_k + C5 * u_km1 + C6 * u_km2 - C2 * y_km1 - C3 * y_km2) / C1;

            u_km2 = u_km1;
            u_km1 = u_k;
            y_km2 = y_km1;
            y_km1 = y_k;

            return y_k;
        }
        else
            return 0.0;
    }

public:
    GustModelBase() : dt_(0.05)
    {
        random_generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
        gust_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        initialized_ = false;
    }

    void initializeParameters(const double &V,
                              const double &L,
                              const double &sigma)
    {
        double b = 2 * sqrt(3) * L / V;
        double c = 2 * L / V;

        alpha_ = sigma * sqrt(2 * L / 3.141592 / V);
        beta_ = alpha_ * b;
        delta_ = 2 * c;
        gamma_ = c * c;

        u_km1 = 0;
        u_km2 = 0;
        y_km1 = 0;
        y_km2 = 0;

        initialized_ = true;
    }

    double integrate(const double &dt)
    {
        if (dt > dt_)
        {
            double t = 0;
            double y = 0;
            while (t < dt)
            {
                double t_inc = std::min(dt_, dt - t);
                y = run(t_inc);
                t += t_inc;
            }
            return y;
        }
        else
        {
            return run(dt);
        }
    }
};

class DrydenWind
{
public:
    DrydenWind() : initialized_(false) {}

    void initialize(const double &wx_nominal, const double &wy_nominal, const double &wz_nominal,
                    const double &wx_sigma,   const double &wy_sigma,   const double &wz_sigma,
                    const double &altitude=2.0)
    {
        wx_nominal_ = wx_nominal;
        wy_nominal_ = wy_nominal;
        wz_nominal_ = wz_nominal;

        double Lz_ft = 3.281 * altitude;
        double Lx_ft = Lz_ft / pow(0.177 + 0.000823 * Lz_ft, 1.2);
        double Ly_ft = Lx_ft;

        wx_gust_.initializeParameters(10.0, Lx_ft / 3.281, wx_sigma);
        wy_gust_.initializeParameters(10.0, Ly_ft / 3.281, wy_sigma);
        wz_gust_.initializeParameters(10.0, Lz_ft / 3.281, wz_sigma);

        initialized_ = true;
    }

    void getWind(const double &dt, double *wx_nominal, double * wy_nominal, double * wz_nominal)
    {
        if (initialized_)
        {
            *wx_nominal += wx_gust_.integrate(dt);
            *wy_nominal += wy_gust_.integrate(dt);
            *wz_nominal += wz_gust_.integrate(dt);
        }
        else
        {
            *wx_nominal = 0.0f;
            *wy_nominal = 0.0f;
            *wz_nominal = 0.0f;
        }
    }

private:
    GustModelBase wx_gust_;
    GustModelBase wy_gust_;
    GustModelBase wz_gust_;

    double wx_nominal_;
    double wy_nominal_;
    double wz_nominal_;

    bool initialized_;
};
namespace gazebo {


// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";

static constexpr double kDefaultWindVelocityMean = 0.0;
static constexpr double kDefaultWindVelocityMax = 100.0;
static constexpr double kDefaultWindVelocityVariance = 0.0;
static constexpr double kDefaultWindGustVelocityMean = 0.0;
static constexpr double kDefaultWindGustVelocityMax = 10.0;
static constexpr double kDefaultWindGustVelocityVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
static constexpr double kDefaultWindDirectionVariance = 0.0;
static constexpr double kDefaultWindGustDirectionVariance = 0.0;



/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboDrydenPlugin : public WorldPlugin {
 public:
  GazeboDrydenPlugin()
      : WorldPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_("world_wind"),
        wind_velocity_mean_(kDefaultWindVelocityMean),
        wind_velocity_max_(kDefaultWindVelocityMax),
        wind_velocity_variance_(kDefaultWindVelocityVariance),
        wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
        wind_gust_velocity_max_(kDefaultWindGustVelocityMax),
        wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
        wind_direction_mean_(kDefaultWindDirectionMean),
        wind_direction_variance_(kDefaultWindDirectionVariance),
        wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
        wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
        frame_id_(kDefaultFrameId),
        pub_interval_(0.5),
        node_handle_(NULL) {}

  virtual ~GazeboDrydenPlugin();




 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;

  std::string namespace_;

  std::string frame_id_;
  std::string wind_pub_topic_;

  double wind_velocity_mean_;
  double wind_velocity_max_;
  double wind_velocity_variance_;
  double wind_gust_velocity_mean_;
  double wind_gust_velocity_max_;
  double wind_gust_velocity_variance_;
  double pub_interval_;
  std::default_random_engine wind_velocity_generator_;
  std::normal_distribution<double> wind_velocity_distribution_;
  std::default_random_engine wind_gust_velocity_generator_;
  std::normal_distribution<double> wind_gust_velocity_distribution_;

  ignition::math::Vector3d wind_direction_mean_;
  ignition::math::Vector3d wind_gust_direction_mean_;
  double wind_direction_variance_;
  double wind_gust_direction_variance_;
  std::default_random_engine wind_direction_generator_;
  std::normal_distribution<double> wind_direction_distribution_X_;
  std::normal_distribution<double> wind_direction_distribution_Y_;
  std::normal_distribution<double> wind_direction_distribution_Z_;
  std::default_random_engine wind_gust_direction_generator_;
  std::normal_distribution<double> wind_gust_direction_distribution_X_;
  std::normal_distribution<double> wind_gust_direction_distribution_Y_;
  std::normal_distribution<double> wind_gust_direction_distribution_Z_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;
  common::Time last_time_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;




  physics_msgs::msgs::Wind wind_msg;
};
}




#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_DRYDEN_PLUGIN_H
