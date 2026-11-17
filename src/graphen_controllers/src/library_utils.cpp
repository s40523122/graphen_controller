#include "graphen_controllers/library_utils.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace graphen::control {

namespace {

constexpr size_t kMaxItemNameLength = 64;
constexpr size_t kShortHashLength = 12;

}  // namespace

std::string resolve_path(const std::string & path) {
  const std::string scheme = "package://";
  if (path.rfind(scheme, 0) == 0) {
    const auto rest = path.substr(scheme.size());
    const auto slash = rest.find('/');
    if (slash == std::string::npos || slash == 0) {
      throw std::invalid_argument("invalid package path '" + path + "'");
    }
    try {
      return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) +
             rest.substr(slash);
    } catch (const std::exception & e) {
      throw std::invalid_argument("cannot resolve '" + path + "': " + e.what());
    }
  }
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (home == nullptr || *home == '\0') {
      throw std::invalid_argument("cannot resolve '" + path + "': HOME is not set");
    }
    return std::string(home) + path.substr(1);
  }
  return path;
}

bool is_valid_item_name(const std::string & name) {
  return !name.empty() && name.size() <= kMaxItemNameLength &&
         std::all_of(name.begin(), name.end(), [](char c) {
             return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
           });
}

void require_valid_item_name(const std::string & kind, const std::string & name) {
  if (!is_valid_item_name(name)) {
    throw std::invalid_argument(
            "invalid " + kind + " name '" + name + "': use 1 to " +
            std::to_string(kMaxItemNameLength) + " characters of [a-z0-9_-]");
  }
}

std::string read_text_file(const std::string & path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("cannot open '" + path + "'");
  }
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

std::string short_hash(const std::string & sha256) {
  return sha256.substr(0, kShortHashLength);
}

}  // namespace graphen::control
