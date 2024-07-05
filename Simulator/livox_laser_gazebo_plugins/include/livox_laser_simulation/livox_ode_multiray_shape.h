//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
#define SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/util/system.hh>
#include <gazebo/ode/common.h>

namespace gazebo{
namespace physics{
class GZ_PHYSICS_VISIBLE LivoxOdeMultiRayShape : public MultiRayShape{
    /// \brief Constructor.
    /// \param[in] _parent Parent Collision.
    public: explicit LivoxOdeMultiRayShape(CollisionPtr _parent);

    /// \brief Destructor.
    public: virtual ~LivoxOdeMultiRayShape();

    // Documentation inherited.
    public: virtual void UpdateRays();

    public: virtual void Init();

    public: std::vector<RayShapePtr> &RayShapes(){return rays;}
    /// \brief Ray-intersection callback.
    /// \param[in] _data Pointer to user data.
    /// \param[in] _o1 First geom to check for collisions.
    /// \param[in] _o2 Second geom to check for collisions.
    private: static void UpdateCallback(void *_data, dGeomID _o1,
                                        dGeomID _o2);

    /// \brief Add a ray to the collision.
    /// \param[in] _start Start of a ray.
    /// \param[in] _end End of a ray.
    public: void AddRay(const ignition::math::Vector3<double> &_start,
                        const ignition::math::Vector3<double> &_end);

    /// \brief Space to contain the ray space, for efficiency.
    private: dSpaceID superSpaceId;

    /// \brief Ray space for collision detector.
    private: dSpaceID raySpaceId;

 private:
    std::vector<RayShapePtr> livoxRays;
};
}
}


#endif  // SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
