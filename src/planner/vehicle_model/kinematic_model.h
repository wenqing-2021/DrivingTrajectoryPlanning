#include "kinematic_model.pb.h"

namespace planning {
namespace vehicle_model {

class KinematicModel {
  public:
    KinematicModel(const kinematic_model::VehicleParam& vehicle_param);
    ~KinematicModel() = default;

    void InitState(const kinematic_model::StateVar& state_var);
    void UpdateState(const kinematic_model::ControlVar& control_var, const double& delta_time);

  private:
    kinematic_model::VehicleParam vehicle_param_;
    kinematic_model::StateVar     state_var_;
    kinematic_model::ControlVar   control_var_;

  private:
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    double steer_angle_{0.0};
    double velocity_{0.0};
};

}   // namespace vehicle_model
}   // namespace planning