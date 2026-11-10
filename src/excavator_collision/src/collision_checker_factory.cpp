#include "excavator_collision/collision_checker_factory.hpp"

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "excavator_collision/fcl_collision_checker.hpp"

namespace excavator {
namespace planning {

struct CollisionCheckerFactory::Impl {
  mutable std::mutex mutex;
  std::unordered_map<std::string, CollisionCheckerCreator> creators;
};

CollisionCheckerFactory& CollisionCheckerFactory::Instance() {
  static CollisionCheckerFactory factory;
  return factory;
}

CollisionCheckerFactory::CollisionCheckerFactory()
    : impl_(std::make_unique<Impl>()) {}

void CollisionCheckerFactory::Register(const std::string& type,
                                       CollisionCheckerCreator creator) {
  if (type.empty() || !creator) {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->creators[type] = std::move(creator);
}

std::unique_ptr<CollisionChecker> CollisionCheckerFactory::Create(
    const std::string& type) const {
  CollisionCheckerCreator creator;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto found = impl_->creators.find(type);
    if (found == impl_->creators.end()) {
      return nullptr;
    }
    creator = found->second;
  }
  return creator ? creator() : nullptr;
}

std::vector<std::string> CollisionCheckerFactory::AvailableTypes() const {
  std::vector<std::string> types;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    types.reserve(impl_->creators.size());
    for (const auto& entry : impl_->creators) {
      types.push_back(entry.first);
    }
  }
  std::sort(types.begin(), types.end());
  return types;
}

namespace {
struct FclRegistrar {
  FclRegistrar() {
    CollisionCheckerFactory::Instance().Register(
        "fcl", []() -> std::unique_ptr<CollisionChecker> {
          return std::make_unique<FclCollisionChecker>();
        });
  }
};

const FclRegistrar kFclRegistrar;
}  // namespace

}  // namespace planning
}  // namespace excavator
