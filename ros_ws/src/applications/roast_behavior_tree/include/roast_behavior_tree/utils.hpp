#ifndef ROAST_BEHAVIOR_TREE__UTILS_HPP_
#define ROAST_BEHAVIOR_TREE__UTILS_HPP_

#include <cstdlib>
#include <iostream>
#include <string>

namespace roast_behavior_tree {

/*
Util function to read the environment variables and get the path to the project
directory
*/
std::string get_project_path() {
  char *project_path;
  project_path = getenv("ROAST_DIR");
  if (project_path == NULL) {
    return std::string("~/.roast");
  }
  return std::string(project_path);
}
};  // namespace roast_behavior_tree

#endif  // ROAST_BEHAVIOR_TREE__UTILS_HPP_
