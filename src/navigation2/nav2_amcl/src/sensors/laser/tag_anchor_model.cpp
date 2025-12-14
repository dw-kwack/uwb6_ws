////////// 2025.07.30 by Noh begin
#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
namespace nav2_amcl
{
TagAnchorModel::TagAnchorModel(double sigma_hit, double sigma_azimuth, size_t max_beams, 
                               std::vector<geometry_msgs::msg::Point> &anchors_, map_t * map)
: Laser(max_beams, map)
{
  sigma_hit_ = sigma_hit;
  sigma_azimuth_ = sigma_azimuth;

  if (sigma_azimuth_ > 0.0) {
    hasAzimuth = true;
  } else {
    hasAzimuth = false;
  }

  anchors = anchors_;
}

double TagAnchorModel::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
// Compute the importance factor of a pose-estimating particle sample 
// using distance/azimuth data from the fixed anchors.

  TagAnchorModel * self = reinterpret_cast<TagAnchorModel *>(data->laser);

  double hit_fctor = (double) (0.5) / (self->sigma_hit_ * self->sigma_hit_);
  double hit_norm = (double) (0.3989422804014327) / self->sigma_hit_;  // 1/(sqrt(2pi)*sigma

  double azi_fctor = (double) (0.5) / (self->sigma_azimuth_ * self->sigma_azimuth_);
  double azi_norm = (double) (0.3989422804014327) / self->sigma_azimuth_;  // 1/(sqrt(2pi)*sigma

  double total_weight = 0.0;
  
  // Iterate through particles
  for (int j = 0; j < set->sample_count; j++) {

    // Compute the weight of each particle
 
    pf_sample_t * sample = set->samples + j;
    pf_vector_t pose = sample->pose;

    // Take account of the uwb tag pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose_, pose);

    double p = 1.0;
    
    // the range finder is assumed to have Gaussian noise only
    for (int i = 0; i < data->range_count; i++) {

      // Range correction
      // data->ranges[i][0] is the 3D distance between the sennsor and the anchor.
      double measureValue = data->ranges[i][0];

      // Check for NaN and ignore max range readings
      if (std::isnan(measureValue) || measureValue >= data->range_max) {
        continue;
      }
      
      // Compute the true range to which is the distance between the robot and the anchor
      const auto & anchor = self->anchors[i];
      double dx = anchor.x - pose.v[0];
      double dy = anchor.y - pose.v[1];
      double dz = anchor.z; // assuming sensor z = 0
      
      double trueValue = sqrt(dx * dx + dy * dy + dz * dz);
        
      // difference between the true range and the measured range
      double err = measureValue - trueValue;

      // Gaussian model
       double pz = hit_norm * std::exp(-(err * err * hit_fctor ));
    
      // random measurements : Uwb two-way ranging cannot have random measurements
      // pz += self->z_rand_ * z_rand_mult;

      // Azimuth correction
      if (self->hasAzimuth)
      {
        measureValue = data->ranges[i][1];
        trueValue = std::atan2(dy, dx) - pose.v[2];       // [-pi, pi] radians
        
      // difference between the true azimuth and the measured azimuth
        err = measureValue - trueValue;

        pz *= azi_norm * std::exp(-(err * err * azi_fctor ));
      }

      assert(pz >= 0.0 && pz <= 1.0);
      
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      // p += pz * pz * pz;

      // theoretically correct P(z|x)
      p *= pz;
	  }

    sample->weight *= p;
    total_weight += sample->weight;
  }

   return total_weight;
}

bool TagAnchorModel::sensorUpdate(pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
	return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t)sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
////////// end