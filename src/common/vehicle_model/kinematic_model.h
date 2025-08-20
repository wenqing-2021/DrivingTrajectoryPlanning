#include "kinematic_model.pb.h"
#include <Eigen/Core>

namespace planning {
namespace vehicle_model {

using opt_status  = Eigen::MatrixXd;
using opt_control = Eigen::MatrixXd;
class KinematicModel {
  public:
    KinematicModel(const kinematic_model::VehicleParam& vehicle_param);
    ~KinematicModel() = default;

    void InitState(const kinematic_model::StateVar& state_var);
    void UpdateState(const kinematic_model::ControlVar& control_var, const double& delta_time);
    const kinematic_model::VehicleParam& GetVehicleParam() const { return vehicle_param_; }

  public:
    static std::size_t GetStateSize() {
        static const std::size_t kStateSize = kinematic_model::StateVar::descriptor()->field_count();
        return kStateSize;
    }

    static std::size_t GetControlSize() {
        static const std::size_t kControlSize = kinematic_model::ControlVar::descriptor()->field_count();
        return kControlSize;
    }

  private:
    kinematic_model::VehicleParam vehicle_param_;
    kinematic_model::StateVar     state_var_;
    kinematic_model::ControlVar   control_var_;

  private:
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    double velocity_{0.0};
    double steer_angle_{0.0};
    double acceleration_{0.0};
};

}   // namespace vehicle_model
}   // namespace planning