#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_FACTORY_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "excavator_collision/collision_checker_interface.hpp"

namespace tramcar {
namespace planning {

using CollisionCheckerCreator =
    std::function<std::unique_ptr<CollisionChecker>()>;

// Runtime registry for collision backends. The default backend is
// "urdf_fcl"; a DH-native backend can be added later by registering another
// creator, without touching the planner or controller.
class CollisionCheckerFactory {
 public:
  static CollisionCheckerFactory& Instance();

  CollisionCheckerFactory(const CollisionCheckerFactory&) = delete;
  CollisionCheckerFactory& operator=(const CollisionCheckerFactory&) = delete;

  void Register(const std::string& type, CollisionCheckerCreator creator);
  std::unique_ptr<CollisionChecker> Create(const std::string& type) const;
  std::vector<std::string> AvailableTypes() const;

 private:
  CollisionCheckerFactory();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_FACTORY_HPP_
