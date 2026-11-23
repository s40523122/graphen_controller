#include "graphen_arm_ik/arm_model.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace graphen::arm_ik {

namespace {

// IK 模型的碰撞位元：只檢查「手臂 ↔ 手臂」與「手臂 ↔ 其他（軀幹、腿、頭、障礙物）」。
// MuJoCo 在 (contype1 & conaffinity2) || (contype2 & conaffinity1) 時檢查一對 geom：
//   手臂：contype 1、conaffinity 1|2；其他：contype 2、conaffinity 1 → 其他 ↔ 其他不檢查
constexpr int kArmContype = 1;
constexpr int kArmConaffinity = 1 | 2;
constexpr int kOtherContype = 2;
constexpr int kOtherConaffinity = 1;

struct SpecDeleter {
  void operator()(mjSpec * spec) const {mj_deleteSpec(spec);}
};
struct DataDeleter {
  void operator()(mjData * data) const {mj_deleteData(data);}
};

mjtGeom to_geom_type(Obstacle::Shape shape) {
  switch (shape) {
    case Obstacle::Shape::kBox:
      return mjGEOM_BOX;
    case Obstacle::Shape::kSphere:
      return mjGEOM_SPHERE;
    case Obstacle::Shape::kCylinder:
      return mjGEOM_CYLINDER;
    case Obstacle::Shape::kCapsule:
      return mjGEOM_CAPSULE;
  }
  return mjGEOM_BOX;
}

// MJCF 依 arm_ik 的需要修改後編譯（說明見 ArmModel）
mjModel * compile_model(const ArmConfig & config) {
  std::array<char, 1024> error{};
  std::unique_ptr<mjSpec, SpecDeleter> spec(
    mj_parseXML(config.mjcf.c_str(), nullptr, error.data(), static_cast<int>(error.size())));
  if (!spec) {
    throw std::invalid_argument("cannot load MJCF '" + config.mjcf + "': " + error.data());
  }

  mjsBody * world = mjs_findBody(spec.get(), "world");
  mjsBody * root = mjs_findBody(spec.get(), config.root_body.c_str());
  if (!root) {
    throw std::invalid_argument("root_body '" + config.root_body + "' is not in the MJCF");
  }
  // 機器人的基座：root_body 往上直到 worldbody 的下一層。障礙物掛在它下面，位姿以基座座標系表示
  mjsBody * base = root;
  for (mjsBody * parent = mjs_getParent(base->element); parent && parent != world;
    parent = mjs_getParent(base->element))
  {
    base = parent;
  }

  // 先收集再刪除，避免邊走訪邊修改
  std::vector<mjsElement *> to_delete;
  for (mjsElement * el = mjs_firstElement(spec.get(), mjOBJ_JOINT); el;
    el = mjs_nextElement(spec.get(), el))
  {
    if (mjs_asJoint(el)->type == mjJNT_FREE) {
      to_delete.push_back(el);
    }
  }
  // keyframe 的 qpos 包含 free joint，移除 free joint 後長度不符；IK 模型用不到 keyframe
  for (mjsElement * el = mjs_firstElement(spec.get(), mjOBJ_KEY); el;
    el = mjs_nextElement(spec.get(), el))
  {
    to_delete.push_back(el);
  }
  if (!config.collision.include_floor) {
    for (mjsElement * el = mjs_firstChild(world, mjOBJ_GEOM, 0); el; el = mjs_nextChild(world, el, 0)) {
      if (mjs_asGeom(el)->type == mjGEOM_PLANE) {
        to_delete.push_back(el);
      }
    }
  }
  for (mjsElement * el : to_delete) {
    if (mjs_delete(spec.get(), el) != 0) {
      throw std::invalid_argument("cannot modify MJCF: " + std::string(mjs_getError(spec.get())));
    }
  }

  if (!config.scene.empty()) {
    mjsBody * scene = mjs_addBody(base, nullptr);
    mjs_setName(scene->element, "arm_ik_scene");
    for (const auto & obstacle : config.scene) {
      mjsGeom * geom = mjs_addGeom(scene, nullptr);
      mjs_setName(geom->element, ("arm_ik_scene/" + obstacle.name).c_str());
      geom->type = to_geom_type(obstacle.shape);
      for (int i = 0; i < 3; ++i) {
        geom->size[i] = obstacle.size[static_cast<size_t>(i)];
        geom->pos[i] = obstacle.pose.translation()[i];
      }
      const Eigen::Quaterniond q(obstacle.pose.linear());
      geom->quat[0] = q.w();
      geom->quat[1] = q.x();
      geom->quat[2] = q.y();
      geom->quat[3] = q.z();
      // 位元在編譯後依是否屬於手臂設定；這裡只要非 0（參與碰撞）
      geom->contype = kOtherContype;
      geom->conaffinity = kOtherConaffinity;
    }
  }

  mjModel * model = mj_compile(spec.get(), nullptr);
  if (!model) {
    throw std::invalid_argument(
      "cannot compile MJCF '" + config.mjcf + "': " + std::string(mjs_getError(spec.get())));
  }
  return model;
}

int find_id(const mjModel * model, mjtObj type, const std::string & name, const std::string & what) {
  const int id = mj_name2id(model, type, name.c_str());
  if (id < 0) {
    throw std::invalid_argument(what + " '" + name + "' is not in the MJCF");
  }
  return id;
}

bool is_descendant(const mjModel * model, int body, int ancestor) {
  for (int b = body; b > 0; b = model->body_parentid[b]) {
    if (b == ancestor) {
      return true;
    }
  }
  return ancestor == 0;
}

}  // namespace

std::shared_ptr<const ArmModel> ArmModel::load(const ArmConfig & config) {
  // 建構子是 private，不能用 make_shared
  std::shared_ptr<ArmModel> self(new ArmModel());
  self->config_ = config;
  self->model_ = compile_model(config);
  const mjModel * m = self->model_;
  self->nbody_ = static_cast<size_t>(m->nbody);
  self->root_body_ = find_id(m, mjOBJ_BODY, config.root_body, "root_body");

  const double margin = config.solver.joint_limit_margin;
  for (const auto & definition : config.arms) {
    const auto where = "arms." + definition.name;
    Arm arm;
    arm.definition = definition;
    arm.tip_body = find_id(m, mjOBJ_BODY, definition.tip_body, where + ".tip_body");
    arm.elbow_body = find_id(m, mjOBJ_BODY, definition.elbow_body, where + ".elbow_body");
    arm.shoulder_body = find_id(m, mjOBJ_BODY, definition.shoulder_body, where + ".shoulder_body");
    for (size_t i = 0; i < kArmDof; ++i) {
      const auto & name = definition.joints[i];
      const int joint = find_id(m, mjOBJ_JOINT, name, where + ".joints");
      if (m->jnt_type[joint] != mjJNT_HINGE) {
        throw std::invalid_argument(where + ": joint '" + name + "' is not a hinge");
      }
      // 每個 joint 都必須在 root → tip 的鏈上，否則 Jacobian 對末端沒有作用
      if (!is_descendant(m, arm.tip_body, m->jnt_bodyid[joint]) ||
        !is_descendant(m, m->jnt_bodyid[joint], self->root_body_))
      {
        throw std::invalid_argument(
          where + ": joint '" + name + "' is not on the chain from root_body to tip_body");
      }
      arm.qpos_adr[i] = m->jnt_qposadr[joint];
      arm.dof_adr[i] = m->jnt_dofadr[joint];
      if (m->jnt_limited[joint]) {
        arm.lower[i] = m->jnt_range[2 * joint] + margin;
        arm.upper[i] = m->jnt_range[2 * joint + 1] - margin;
        if (arm.lower[i] > arm.upper[i]) {
          throw std::invalid_argument(
            where + ": joint '" + name + "' range is narrower than joint_limit_margin");
        }
      } else {
        arm.lower[i] = -std::numeric_limits<double>::infinity();
        arm.upper[i] = std::numeric_limits<double>::infinity();
      }
    }
    for (size_t i = 0; i < kArmDof; ++i) {
      if (definition.seed[i] < arm.lower[i] || definition.seed[i] > arm.upper[i]) {
        throw std::invalid_argument(
          where + ".seed[" + std::to_string(i) + "] is outside the joint range (minus margin)");
      }
    }
    // 第一個 joint 所在 body 的整個子樹都屬於這隻手臂（含手掌、手指）
    const int first = m->jnt_bodyid[m->dof_jntid[arm.dof_adr[0]]];
    arm.bodies.assign(self->nbody_, false);
    for (int b = 0; b < m->nbody; ++b) {
      arm.bodies[static_cast<size_t>(b)] = is_descendant(m, b, first);
    }
    if (self->arm_index(definition.name) >= 0) {
      throw std::invalid_argument("arm '" + definition.name + "' is defined twice");
    }
    self->arms_.push_back(std::move(arm));
  }

  // 碰撞位元（見 kArmContype）。模擬用的 MJCF 通常關閉自碰撞（例如 make_mjcf.py 的
  // robot contype=1 / conaffinity=0，只和地面碰撞），這裡重新設定；純顯示的 geom（兩者皆 0）不變
  mjModel * mutable_model = self->model_;
  for (int g = 0; g < m->ngeom; ++g) {
    if (mutable_model->geom_contype[g] == 0 && mutable_model->geom_conaffinity[g] == 0) {
      continue;
    }
    bool in_arm = false;
    for (const auto & arm : self->arms_) {
      in_arm = in_arm || arm.bodies[static_cast<size_t>(m->geom_bodyid[g])];
    }
    mutable_model->geom_contype[g] = in_arm ? kArmContype : kOtherContype;
    mutable_model->geom_conaffinity[g] = in_arm ? kArmConaffinity : kOtherConaffinity;
  }
  // broadphase 用的是 body 層級的位元（該 body 所有 geom 的 OR），編譯時算好，要跟著重算
  for (int b = 0; b < m->nbody; ++b) {
    mutable_model->body_contype[b] = 0;
    mutable_model->body_conaffinity[b] = 0;
  }
  for (int g = 0; g < m->ngeom; ++g) {
    mutable_model->body_contype[m->geom_bodyid[g]] |= mutable_model->geom_contype[g];
    mutable_model->body_conaffinity[m->geom_bodyid[g]] |= mutable_model->geom_conaffinity[g];
  }

  // 允許的接觸
  self->allowed_.assign(self->nbody_ * self->nbody_, false);
  // 運動鏈上相距 ignore_within_links 個 link 以內的 body（例如軀幹與肩部第二節）在關節附近的
  // mesh 本來就重疊，不算碰撞。父子（距離 1）MuJoCo 本身就會略過
  for (int b = 1; b < m->nbody; ++b) {
    int ancestor = b;
    for (int k = 0; k < config.collision.ignore_within_links && ancestor > 0; ++k) {
      ancestor = m->body_parentid[ancestor];
      if (ancestor > 0) {
        self->allow(b, ancestor);
      }
    }
  }
  for (const auto & [a, b] : config.collision.allowed_pairs) {
    self->allow(find_id(m, mjOBJ_BODY, a, "collision.allowed_pairs"),
      find_id(m, mjOBJ_BODY, b, "collision.allowed_pairs"));
  }
  if (config.collision.allow_initial_contacts) {
    std::unique_ptr<mjData, DataDeleter> data(mj_makeData(m));
    mj_kinematics(m, data.get());
    mj_collision(m, data.get());
    for (int i = 0; i < data->ncon; ++i) {
      const auto & contact = data->contact[i];
      const int b1 = m->geom_bodyid[contact.geom[0]];
      const int b2 = m->geom_bodyid[contact.geom[1]];
      if (!self->pair_allowed(b1, b2)) {
        self->allow(b1, b2);
        self->initial_contacts_.emplace_back(self->body_name(b1), self->body_name(b2));
      }
    }
  }
  return self;
}

ArmModel::~ArmModel() {
  if (model_) {
    mj_deleteModel(model_);
  }
}

int ArmModel::arm_index(const std::string & name) const {
  for (size_t i = 0; i < arms_.size(); ++i) {
    if (arms_[i].definition.name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int ArmModel::qpos_address(const std::string & joint) const {
  const int id = mj_name2id(model_, mjOBJ_JOINT, joint.c_str());
  if (id < 0 || (model_->jnt_type[id] != mjJNT_HINGE && model_->jnt_type[id] != mjJNT_SLIDE)) {
    return -1;
  }
  return model_->jnt_qposadr[id];
}

std::string ArmModel::body_name(int body) const {
  const char * name = mj_id2name(model_, mjOBJ_BODY, body);
  return name ? name : "body#" + std::to_string(body);
}

void ArmModel::allow(int body1, int body2) {
  allowed_[static_cast<size_t>(body1) * nbody_ + static_cast<size_t>(body2)] = true;
  allowed_[static_cast<size_t>(body2) * nbody_ + static_cast<size_t>(body1)] = true;
}

std::shared_ptr<const ArmModel> ArmModel::load_cached(const std::string & config_path) {
  static std::mutex mutex;
  static std::map<std::string, std::shared_ptr<const ArmModel>> cache;
  const auto resolved = resolve_package_path(config_path);
  // 載入期間持有鎖：同一個設定同時被要求時只載入一次（載入只發生在 non-RT 的 configure）
  std::lock_guard<std::mutex> lock(mutex);
  auto & entry = cache[resolved];
  if (!entry) {
    entry = load(load_arm_config(resolved));
  }
  return entry;
}

}  // namespace graphen::arm_ik
