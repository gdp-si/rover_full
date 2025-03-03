#include "roast_interfaces/srv/delete_logs.hpp"

#include <dirent.h>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class DeleteLogs : public rclcpp::Node {
 public:
  DeleteLogs() : Node("delete_logs") {
    auto handle_delete_logs =
        [this](const std::shared_ptr<roast_interfaces::srv::DeleteLogs::Request>
                   request,
               std::shared_ptr<roast_interfaces::srv::DeleteLogs::Response>
                   response) {
          delete_files(request->folder_path, request->expiry_days,
                       request->type);
          delete_folders(
              request->folder_path,
              request->expiry_days);  // FIXME: Delete folder is still happening
                                      // even its commented
          response->success = true;
          return true;
        };

    delete_logs_service_ =
        this->create_service<roast_interfaces::srv::DeleteLogs>(
            "delete_logs", handle_delete_logs);
    RCLCPP_INFO(this->get_logger(), "Delete logs service started.");
  }

 private:
  bool is_expired(const std::string &file_name, int expiry_days) {
    std::string day_str = file_name.substr(0, 2);
    std::string month_str = file_name.substr(3, 2);
    std::string year_str = file_name.substr(6, 4);

    RCLCPP_DEBUG(this->get_logger(), "file_name: %s", file_name.c_str());
    RCLCPP_DEBUG(this->get_logger(), "day_str: %s", day_str.c_str());
    RCLCPP_DEBUG(this->get_logger(), "month_str: %s", month_str.c_str());
    RCLCPP_DEBUG(this->get_logger(), "year_str: %s", year_str.c_str());
    RCLCPP_DEBUG(this->get_logger(), "expiry_days: %d", expiry_days);

    int day = std::stoi(day_str);
    int month = std::stoi(month_str);
    int year = std::stoi(year_str);

    std::tm t = {};
    t.tm_mday = day;
    t.tm_mon = month - 1;     // tm_mon is zero-based
    t.tm_year = year - 1900;  // tm_year is the number of years since 1900

    auto file_time = std::mktime(&t);
    auto now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto diff = std::difftime(now, file_time) / (60 * 60 * 24);

    return diff >= expiry_days;
  }

  void delete_files(const std::string &folder_path, int expiry_days, int type) {
    DIR *dir;
    struct dirent *ent;

    if ((dir = opendir(folder_path.c_str())) != NULL) {
      while ((ent = readdir(dir)) != NULL) {
        std::string file_name(ent->d_name);
        if (type >= 0 && type <= 3) {
          delete_files_with_extension(folder_path, file_name, expiry_days,
                                      type);
        } else if (type == 4) {
          for (uint8_t i = 0; i <= 3; ++i) {
            delete_files_with_extension(folder_path, file_name, expiry_days, i);
          }
        } else {
          std::cerr << "Invalid input" << std::endl;
        }
      }
      closedir(dir);
    } else {
      std::cerr << "Error: Could not open directory '" << folder_path << "'."
                << std::endl;
    }
  }

  void delete_files_with_extension(const std::string &folder_path,
                                   std::string file_name, int expiry_days,
                                   int type) {
    roast_interfaces::srv::DeleteLogs::Request req;
    switch (type) {
      case req.LOG_TYPE_LOGFILE: {
        if (file_name.length() >= 10 &&
            file_name.substr(file_name.length() - 4) == ".log") {
          if (is_expired(file_name.substr(0, 10), expiry_days)) {
            std::string file_path = folder_path + "/" + file_name;
            std::filesystem::remove(file_path.c_str());
          }
        }
      }
      case req.LOG_TYPE_IMAGE: {
        if (file_name.length() >= 10 &&
            (file_name.substr(file_name.length() - 5) == ".jpeg" ||
             file_name.substr(file_name.length() - 4) == ".png")) {
          if (is_expired(file_name.substr(0, 10), expiry_days)) {
            std::string file_path = folder_path + "/" + file_name;
            std::filesystem::remove(file_path.c_str());
          }
        }
      }
      case req.LOG_TYPE_VIDEO: {
        if (file_name.length() >= 10 &&
            (file_name.substr(file_name.length() - 4) == ".mp4" ||
             file_name.substr(file_name.length() - 4) == ".mkv")) {
          if (is_expired(file_name.substr(0, 10), expiry_days)) {
            std::string file_path = folder_path + "/" + file_name;
            std::filesystem::remove(file_path.c_str());
          }
        }
      }
      case req.LOG_TYPE_ROSBAG: {
        if (file_name.length() >= 10 &&
            file_name.substr(file_name.length() - 5) == ".mcap") {
          if (is_expired(file_name.substr(0, 10), expiry_days)) {
            std::string file_path = folder_path + "/" + file_name;
            std::filesystem::remove(file_path.c_str());
          }
        }
      }
      default:
        break;
    }
  }

  // Delete the log folders that are older than the expiry days.
  void delete_folders(const std::string &folder_path, int expiry_days) {
    DIR *dir;
    struct dirent *ent;

    if ((dir = opendir(folder_path.c_str())) != NULL) {
      while ((ent = readdir(dir)) != NULL) {
        std::string folder_name(ent->d_name);
        if (folder_name.length() >= 10) {
          if (is_expired(folder_name, expiry_days)) {
            std::string delete_path = folder_path + "/" + folder_name;
            RCLCPP_DEBUG(this->get_logger(), "Deleting folder: %s",
                         delete_path.c_str());
            std::filesystem::remove_all(delete_path);
          }
        }
      }
      closedir(dir);
    } else {
      std::cerr << "Error: Could not open directory '" << folder_path << "'."
                << std::endl;
    }
  }
  rclcpp::Service<roast_interfaces::srv::DeleteLogs>::SharedPtr
      delete_logs_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeleteLogs>());
  rclcpp::shutdown();
  return 0;
}
