#include "VehicleModel/VehicleModelInterface.hpp"

#include "transform.hpp"
class VehicleModelBicycle : public IVehicleModel
{
public:
    VehicleModelBicycle()
    {
        // Initialize position, orientation, velocity, angular velocity, and acceleration to zero
        this->position = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->orientation = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Initialize torques, steering angles, wheel orientations, and wheel speeds to zero
        this->torques = { 0.0, 0.0, 0.0, 0.0 };
        this->steeringAngles = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelOrientations = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelspeeds = { 0.0, 0.0, 0.0, 0.0 };
    }

    bool readConfig(ConfigElement& config)
    {
        // Read configuration parameters from the provided config element
        auto configModel = config["simple_bicycle_model"];
        configModel["kinematics"].getElement<double>(&this->lf, "lf");
        configModel["kinematics"].getElement<double>(&this->lr, "lr");
        configModel["kinematics"].getElement<double>(&this->sf, "sf");
        configModel["kinematics"].getElement<double>(&this->sr, "sr");

        configModel["tire"].getElement<double>(&this->Blat, "Blat");
        configModel["tire"].getElement<double>(&this->Clat, "Clat");
        configModel["tire"].getElement<double>(&this->Dlat, "Dlat");
        configModel["tire"].getElement<double>(&this->Elat, "Elat");

        configModel["aero"].getElement<double>(&this->cla, "cla");
        configModel["aero"].getElement<double>(&this->cda, "cda");
        configModel["aero"].getElement<double>(&this->aeroArea, "aeroArea");

        configModel.getElement<double>(&this->m, "m");
        configModel.getElement<double>(&this->Izz, "Izz");
        configModel.getElement<double>(&this->wheelRadius, "wheelRadius");
        configModel.getElement<double>(&this->gearRatio, "gearRatio");
        configModel.getElement<double>(&this->innerSteeringRatio, "innerSteeringRatio");
        configModel.getElement<double>(&this->innerSteeringRatio, "innerSteeringRatio");
        return true;
    }

    // Getters for various vehicle states
    Eigen::Vector3d getPosition()
    {
        Eigen::Vector3d ret = this->position;
        return ret;
    }

    Eigen::Vector3d getOrientation() { return this->orientation; }

    Eigen::Vector3d getVelocity() { return this->velocity; }

    Eigen::Vector3d getAcceleration() { return this->acceleration; }

    Eigen::Vector3d getAngularVelocity() { return this->angularVelocity; }

    Eigen::Vector3d getAngularAcceleration() { return this->angularAcceleration; }

    Wheels getSteeringAngles() { return this->steeringAngles; }

    double getSteeringWheelAngle()
    {
        // Calculate the steering wheel angle based on the front left steering angle and ratios (ackermann probably)
        return (this->steeringAngles.FL > 0) ? this->steeringAngles.FL / this->innerSteeringRatio
                                             : this->steeringAngles.FL / this->outerSteeringRatio;
    }

    Wheels getWheelspeeds() { return this->wheelspeeds; }

    Wheels getWheelOrientations() { return this->wheelOrientations; }

    Wheels getTorques() { return this->torques; }

    // Setters for various vehicle states
    void setTorques(Wheels in) { this->torques = in; }

    void setRpmSetpoints(Wheels in) { this->rpmSetpoints = in; }

    void setMaxTorques(Wheels in) { this->maxTorques = in; }

    void setMinTorques(Wheels in) { this->minTorques = in; }

    void setSteeringSetpointFront(double in) { setSteeringFront(in); }

    void setSteeringSetpointRear(double in) { return; }

    void setSteeringFront(double in)
    {
        // Set the front steering angles based on the input and steering ratios
        double avgRatio = 0.5 * (this->innerSteeringRatio + this->outerSteeringRatio);
        if (in > 0)
        {
            this->steeringAngles.FL = this->innerSteeringRatio * in / avgRatio;
            this->steeringAngles.FR = this->outerSteeringRatio * in / avgRatio;
        }
        else
        {
            this->steeringAngles.FL = this->outerSteeringRatio * in / avgRatio;
            this->steeringAngles.FR = this->innerSteeringRatio * in / avgRatio;
        }
        return;
    }

    void setPosition(Eigen::Vector3d position) { this->position = position; }
    void setOrientation(Eigen::Vector3d orientation) { this->orientation = orientation; }

    double processSlipAngleLat(double alpha)
    {
        // Calculate the lateral force coefficient (bounded at -1 to 1) using the tire model parameters and slip angle
        return std::sin(Clat * std::atan(Blat * alpha - Elat * (Blat * alpha - std::atan(Blat * alpha))));
    }

    // Calculate dynamic states (ax, ay, rdot) based on the current state and time step
    Eigen::Vector3d getDynamicStates(double dt)
    {
        double l = this->lr + this->lf;
        double vx = this->velocity.x();
        double vy = this->velocity.y();
        double ax = this->acceleration.x();
        double ay = this->acceleration.y();
        double r = this->angularVelocity.z();
        // Downforce
        double F_aero_downforce = 0.5 * 1.29 * this->aeroArea * this->cla * (vx * vx);
        double F_aero_drag = 0.5 * 1.29 * this->aeroArea * this->cda * (vx * vx);
        double g = 9.81;
        double steeringFront = 0.5 * (this->steeringAngles.FL + this->steeringAngles.FR);

        // Calculate normal forces on the front and rear axles
        double Fz_Front = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lr / l));
        double Fz_Rear = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lf / l));

        Eigen::Vector3d vCog = this->velocity;
        Eigen::Vector3d omega = this->angularVelocity;

        // Position vectors of the wheels relative to the center of gravity
        Eigen::Vector3d rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
        Eigen::Vector3d rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
        Eigen::Vector3d rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
        Eigen::Vector3d rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);
        Eigen::Vector3d rFront = Eigen::Vector3d(lf, 0.0, 0.0);
        Eigen::Vector3d rRear = Eigen::Vector3d(-lf, 0.0, 0.0);

        // Calculate the velocities of the wheels
        Eigen::Vector3d vFL = vCog + omega.cross(rFL);
        Eigen::Vector3d vFR = vCog + omega.cross(rFR);
        Eigen::Vector3d vRL = vCog + omega.cross(rRL);
        Eigen::Vector3d vRR = vCog + omega.cross(rRR);
        Eigen::Vector3d vFront = vCog + omega.cross(rFront);
        Eigen::Vector3d vRear = vCog + omega.cross(rRear);

        double rpm2ms = this->wheelRadius * 2.0 * M_PI / 60;

        // Check if the vehicle is at a standstill
        bool stillstand = (vCog.norm() < 0.1) && (std::abs(this->angularVelocity.z()) < 0.001);

        // Calculate tire side slip angles
        double eps = 0.00001;
        double kappaFront = std::atan2(vFront.y(), std::max(std::abs(vFront.x()), eps)) - steeringFront;
        double kappaRear = std::atan2(vRear.y(), std::max(std::abs(vRear.x()), eps));

        if (stillstand)
        {
            kappaFront = 0.0;
            kappaRear = 0.0;
        }

        // Calculate longitudinal forces on the wheels
        double Fx_FL = this->gearRatio * this->torques.FL / this->wheelRadius;
        double Fx_FR = this->gearRatio * this->torques.FR / this->wheelRadius;
        double Fx_RL = this->gearRatio * this->torques.RL / this->wheelRadius;
        double Fx_RR = this->gearRatio * this->torques.RR / this->wheelRadius;

        // Apply forces only if the torques are significant or the vehicle is moving
        Fx_FL *= (((this->torques.FL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_FR *= (((this->torques.FR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RL *= (((this->torques.RL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RR *= (((this->torques.RR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;

        // Calculate lateral forces on the front and rear axles
        double Dlat_Front = this->Dlat * Fz_Front;
        double Dlat_Rear = this->Dlat * Fz_Rear;
        double Fy_Front = Dlat_Front * processSlipAngleLat(kappaFront);
        double Fy_Rear = Dlat_Rear * processSlipAngleLat(kappaRear);

        // Convert wheel speeds to RPM
        this->wheelspeeds.FL = vFL.x() / rpm2ms;
        this->wheelspeeds.FR = vFR.x() / rpm2ms;
        this->wheelspeeds.RL = vRL.x() / rpm2ms;
        this->wheelspeeds.RR = vRR.x() / rpm2ms;

        // Calculate longitudinal and lateral accelerations
        double axTires = (std::cos(this->steeringAngles.FL) * Fx_FL + std::cos(this->steeringAngles.FR) * Fx_FR + Fx_RL
                             + Fx_RR - std::sin(steeringFront) * Fy_Front)
            / m;
        double axModel = axTires - F_aero_drag / m;

        double ayTires = (std::sin(this->steeringAngles.FL) * Fx_FL + std::sin(this->steeringAngles.FR) * Fx_FR
                             + std::cos(steeringFront) * Fy_Front + Fy_Rear)
            / m;
        double ayModel = (ayTires);

        // Calculate the rate of change of yaw rate (angular acceleration)
        double rdotFx
            = 0.5 * this->sf * (-Fx_FL * std::cos(this->steeringAngles.FL) + Fx_FR * std::cos(this->steeringAngles.FR))
            + this->lf * (Fx_FL * std::sin(this->steeringAngles.FL) + Fx_FR * std::sin(this->steeringAngles.FR))
            + 0.5 * this->sr * (Fx_RR * std::cos(this->steeringAngles.RR) - Fx_RL * std::cos(this->steeringAngles.RL))
            - this->lr * (Fx_RL * std::sin(this->steeringAngles.RL) + Fx_RR * std::sin(this->steeringAngles.RR));
        double rdotFy = this->lf * (Fy_Front * std::cos(steeringFront)) - this->lr * (Fy_Rear);
        double rdot = (1 / Izz * (rdotFx + rdotFy));

        // Return the calculated dynamic states
        Eigen::Vector3d ret(axModel, ayModel, rdot);
        return ret;
    }

    // Integrate the vehicle state forward in time by dt
    void forwardIntegrate(double dt)
    {
        // Calculate friction forces
        Eigen::Vector3d friction(std::min(200.0, 2000.0 * std::abs(this->velocity.x())),
            std::min(200.0, 2000.0 * std::abs(this->velocity.y())),
            std::min(200.0, 2000.0 * std::abs(this->velocity.z())));
        friction[0] = (this->velocity.x() > 0) ? friction.x() : -friction.x();
        friction[1] = (this->velocity.y() > 0) ? friction.y() : -friction.y();
        friction[2] = (this->velocity.z() > 0) ? friction.z() : -friction.z();

        // Update position based on velocity and orientation
        Eigen::AngleAxisd yawAngle(this->orientation.z(), Eigen::Vector3d::UnitZ());
        this->position += (yawAngle.matrix() * this->velocity) * dt;

        // Set torques to maximum torques
        this->torques = this->maxTorques;

        // Get dynamic states
        Eigen::Vector3d xdotdyn = getDynamicStates(dt);

        // Update orientation based on angular velocity
        this->orientation += Eigen::Vector3d(0.0, 0.0, dt * angularVelocity.z());

        // Update acceleration based on dynamic states and friction
        this->acceleration = Eigen::Vector3d(xdotdyn[0] - friction.x() / m, xdotdyn[1], 0.0);

        // Update angular velocity and angular acceleration
        this->angularVelocity = (this->angularVelocity + Eigen::Vector3d(0.0, 0.0, xdotdyn[2] * dt));
        this->angularAcceleration = Eigen::Vector3d(0.0, 0.0, xdotdyn[2]);

        // Update velocity based on acceleration and angular velocity
        this->velocity += dt * (this->acceleration - this->angularVelocity.cross(this->velocity));

        // Update wheel orientations based on wheel speeds
        this->wheelOrientations.FL = std::fmod(
            this->wheelOrientations.FL + (this->wheelspeeds.FL / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.FR = std::fmod(
            this->wheelOrientations.FR + (this->wheelspeeds.FR / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.RL = std::fmod(
            this->wheelOrientations.RL + (this->wheelspeeds.RL / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.RR = std::fmod(
            this->wheelOrientations.RR + (this->wheelspeeds.RR / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
    }

private:
    // Vehicle parameters
    double lr = 0.72; // Distance from the center of gravity to the rear axle
    double lf = 0.78; // Distance from the center of gravity to the front axle
    double sf = 1.15; // Track width front
    double sr = 1.15; // Track width rear

    // Tire model parameters
    double Blat = 9.63;
    double Clat = -1.39;
    double Dlat = 1.6;
    double Elat = 1.0;

    // Aerodynamic parameters
    double cla = 3.7;
    double cda = 1.1;
    double aeroArea = 1.1;

    // Vehicle mass and inertia
    double m = 178.0;
    double Izz = 111.0;

    // Wheel and steering parameters
    double wheelRadius = 0.206;
    double gearRatio = 12.23;
    double innerSteeringRatio = 0.255625;
    double outerSteeringRatio = 0.20375;

    // Wheel torques and speeds
    Wheels minTorques = { -0.0, -0.0, -0.0, -0.0 };
    Wheels maxTorques = { 0.0, 0.0, 0.0, 0.0 };
    Wheels rpmSetpoints = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFx = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFy = { 0.0, 0.0, 0.0, 0.0 };
};
