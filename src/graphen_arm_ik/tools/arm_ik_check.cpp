// arm_ik.yaml 的離線自我檢查：載入模型、列出允許的接觸，並以隨機可達目標驗證 IK。
//   arm_ik_check <arm_ik.yaml> [--trials N] [--seed S]
//
// 每隻手臂：
//   - seed 姿態的末端位姿、swivel 與碰撞
//   - 來回測試：限位內隨機關節角 → 正向運動學得到目標（含該姿態的 swivel）→ 從 seed 求解，
//     統計收斂率、殘差、swivel 誤差與耗時
//   - swivel 測試：同一個末端目標改 swivel ±20°，末端應該不動
//   - 伺服測試：以 step() 從 seed 追一個目標，統計收斂所需的步數與每步耗時
//   - 碰撞探測：末端放在軀幹內部，必須偵測到碰撞
// 結束碼：所有手臂的來回測試收斂率 ≥ 90%、碰撞探測都偵測到時為 0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "graphen_arm_ik/arm_ik.hpp"

namespace {

using graphen::arm_ik::ArmIk;
using graphen::arm_ik::ArmModel;
using graphen::arm_ik::ArmVector;
using graphen::arm_ik::Contact;
using graphen::arm_ik::kArmDof;
using graphen::arm_ik::Target;

constexpr double kDeg = M_PI / 180.0;
// 隨機目標只取限位範圍內的這個比例，避開正好壓在限位上的姿態
constexpr double kRangeFraction = 0.8;
// 沒有限位的 joint 取 ±π
constexpr double kUnlimitedRange = M_PI;
constexpr double kSwivelTestAngle = 20.0 * kDeg;
constexpr double kServoRate = 100.0;  // Hz
constexpr int kServoMaxSteps = 2000;
constexpr double kPassRate = 0.9;

void print_usage() {
  std::fprintf(stderr, "Usage: arm_ik_check <arm_ik.yaml> [--trials N] [--seed S]\n");
}

double elapsed_us(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
}

std::string contacts_text(const ArmModel & model, const std::vector<Contact> & contacts) {
  if (contacts.empty()) {
    return "none";
  }
  std::string text;
  for (const auto & c : contacts) {
    char depth[32];
    std::snprintf(depth, sizeof(depth), " (%.1f mm)", c.depth * 1e3);
    text += (text.empty() ? "" : ", ") + model.body_name(c.body1) + "|" + model.body_name(c.body2) + depth;
  }
  return text;
}

ArmVector random_positions(const ArmModel::Arm & arm, std::mt19937 & rng) {
  ArmVector q{};
  for (size_t i = 0; i < kArmDof; ++i) {
    const double lo = std::isfinite(arm.lower[i]) ? arm.lower[i] : -kUnlimitedRange;
    const double hi = std::isfinite(arm.upper[i]) ? arm.upper[i] : kUnlimitedRange;
    const double mid = 0.5 * (lo + hi);
    const double half = 0.5 * (hi - lo) * kRangeFraction;
    q[i] = std::uniform_real_distribution<double>(mid - half, mid + half)(rng);
  }
  return q;
}

}  // namespace

int main(int argc, char ** argv) {
  if (argc < 2) {
    print_usage();
    return 2;
  }
  std::string config_path;
  int trials = 200;
  unsigned seed = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--trials" && i + 1 < argc) {
      trials = std::atoi(argv[++i]);
    } else if (arg == "--seed" && i + 1 < argc) {
      seed = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
    } else if (!arg.empty() && arg[0] != '-' && config_path.empty()) {
      config_path = arg;
    } else {
      print_usage();
      return 2;
    }
  }

  std::shared_ptr<const ArmModel> model;
  try {
    const auto start = std::chrono::steady_clock::now();
    model = ArmModel::load(graphen::arm_ik::load_arm_config(config_path));
    std::printf("Loaded %s (%.0f ms): nq=%ld, nbody=%ld, root_body %s\n",
      model->config().mjcf.c_str(), elapsed_us(start) / 1e3, static_cast<long>(model->model()->nq),
      static_cast<long>(model->model()->nbody), model->config().root_body.c_str());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  std::printf("Allowed initial contacts (%zu):", model->initial_contacts().size());
  for (const auto & [a, b] : model->initial_contacts()) {
    std::printf(" %s|%s", a.c_str(), b.c_str());
  }
  std::printf("\n");

  ArmIk ik(model);
  std::vector<Contact> contacts;
  contacts.reserve(64);
  std::mt19937 rng(seed);
  bool pass = true;

  for (size_t arm_index = 0; arm_index < model->arms().size(); ++arm_index) {
    const int arm = static_cast<int>(arm_index);
    const auto & a = model->arms()[arm_index];
    std::printf("\n== arm '%s' (tip %s)\n", a.definition.name.c_str(), a.definition.tip_body.c_str());

    // seed 姿態
    ik.reset();
    const auto seed_pose = ik.forward(arm);
    ik.check_collisions(-1, contacts);
    std::printf("  seed pose: tip [%.3f %.3f %.3f] m, swivel %s, collisions: %s\n",
      seed_pose.position.x(), seed_pose.position.y(), seed_pose.position.z(),
      std::isfinite(seed_pose.swivel) ?
      (std::to_string(seed_pose.swivel / kDeg).substr(0, 6) + " deg").c_str() : "undefined",
      contacts_text(*model, contacts).c_str());

    // 來回測試
    int converged = 0;
    int swivel_reached = 0;
    double worst_position = 0.0;
    double worst_orientation = 0.0;
    double total_us = 0.0;
    double max_us = 0.0;
    int swivel_tests = 0;
    int swivel_ok = 0;
    double swivel_worst_tip = 0.0;
    int colliding = 0;
    std::map<std::string, int> pair_counts;
    for (int t = 0; t < trials; ++t) {
      ik.reset();
      ik.set_arm(arm, random_positions(a, rng));
      // 隨機姿態的碰撞統計：確認碰撞偵測有作用，並看出常見的碰撞對
      if (ik.check_collisions(arm, contacts) > 0) {
        ++colliding;
        for (const auto & c : contacts) {
          ++pair_counts[model->body_name(c.body1) + "|" + model->body_name(c.body2)];
        }
      }
      const auto goal = ik.forward(arm);
      Target target;
      target.position = goal.position;
      target.orientation = goal.orientation;
      target.swivel = goal.swivel;

      const auto start = std::chrono::steady_clock::now();
      const auto result = ik.solve(arm, target, a.definition.seed);
      const double us = elapsed_us(start);
      total_us += us;
      max_us = std::max(max_us, us);
      if (result.converged) {
        ++converged;
        swivel_reached += result.swivel_reached ? 1 : 0;
        worst_position = std::max(worst_position, result.position_error);
        worst_orientation = std::max(worst_orientation, result.orientation_error);

        // swivel 測試：從這個解出發改 swivel，末端不應該動
        for (const double delta : {kSwivelTestAngle, -kSwivelTestAngle}) {
          if (!std::isfinite(result.swivel)) {
            break;
          }
          Target moved = target;
          moved.swivel = graphen::arm_ik::wrap_angle(result.swivel + delta);
          const auto r = ik.solve(arm, moved, result.positions);
          ++swivel_tests;
          if (r.converged) {
            swivel_worst_tip = std::max(swivel_worst_tip, r.position_error);
          }
          swivel_ok += (r.converged && r.swivel_reached) ? 1 : 0;
        }
      }
    }
    const double rate = trials > 0 ? static_cast<double>(converged) / trials : 0.0;
    pass = pass && rate >= kPassRate;
    std::printf("  round trip from seed: %d/%d converged (%.0f%%), swivel reached %d/%d; "
      "worst error %.2e m / %.2e rad; %.0f us avg, %.0f us max\n",
      converged, trials, rate * 100.0, swivel_reached, converged, worst_position, worst_orientation,
      trials > 0 ? total_us / trials : 0.0, max_us);
    std::vector<std::pair<int, std::string>> top;
    for (const auto & [pair, count] : pair_counts) {
      top.emplace_back(count, pair);
    }
    std::sort(top.rbegin(), top.rend());
    std::printf("  random poses in collision: %d/%d; most frequent:", colliding, trials);
    for (size_t i = 0; i < std::min<size_t>(top.size(), 5); ++i) {
      std::printf(" %s (%d)", top[i].second.c_str(), top[i].first);
    }
    std::printf("\n");
    std::printf("  swivel +-%.0f deg at the same tip target: %d/%d reached (others blocked by limits), "
      "worst tip error %.2e m\n", kSwivelTestAngle / kDeg, swivel_ok, swivel_tests, swivel_worst_tip);

    // 碰撞偵測的探測：把末端放在 root_body 原點（軀幹內部），解出的姿態一定和軀幹碰撞
    {
      ik.reset();
      Target inside;
      inside.orientation = ik.forward(arm).orientation;
      const auto r = ik.solve(arm, inside, a.definition.seed);
      ik.check_collisions(arm, contacts);
      const bool detected = !contacts.empty();
      pass = pass && detected;
      std::printf("  probe (tip at root_body origin, error %.3f m): %s — %s\n", r.position_error,
        detected ? "collision detected" : "NO collision detected",
        contacts_text(*model, contacts).c_str());
    }

    // 伺服測試：從 seed 追一個隨機目標
    ik.reset();
    ik.set_arm(arm, random_positions(a, rng));
    const auto goal = ik.forward(arm);
    Target target;
    target.position = goal.position;
    target.orientation = goal.orientation;
    ik.reset();
    ArmVector q = a.definition.seed;
    int steps = 0;
    double step_us = 0.0;
    const auto & solver = model->config().solver;
    graphen::arm_ik::ServoGains gains;
    for (; steps < kServoMaxSteps; ++steps) {
      const auto start = std::chrono::steady_clock::now();
      q = ik.step(arm, target, q, 1.0 / kServoRate, gains);
      step_us += elapsed_us(start);
      const auto now = ik.forward(arm);
      const double pe = (now.position - target.position).norm();
      const double oe = now.orientation.angularDistance(target.orientation);
      if (pe < 10 * solver.position_tolerance && oe < 10 * solver.orientation_tolerance) {
        break;
      }
    }
    constexpr int kCollisionRepeats = 200;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kCollisionRepeats; ++i) {
      ik.check_collisions(arm, contacts);
    }
    const double collision_us = elapsed_us(start) / kCollisionRepeats;
    std::printf("  servo at %.0f Hz: %s after %d steps (%.1f s), %.1f us/step; collision check %.1f us\n",
      kServoRate, steps < kServoMaxSteps ? "reached" : "NOT reached", steps + 1,
      (steps + 1) / kServoRate, step_us / (steps + 1), collision_us);
  }
  std::printf("\n%s\n", pass ? "OK" :
    "FAILED: convergence rate below 90% or collision probe not detected for some arm");
  return pass ? 0 : 1;
}
