#pragma once
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"
#include <filesystem>
#include <iostream>
#include <string>

namespace utils {

class Logger {
  public:
    static void InitialLogger(const char* logger_name, const char* log_dir) {
        google::InitGoogleLogging(logger_name);   // 初始化glog
        google::SetLogDestination(google::INFO, log_dir);   // 把日志同时记录文件，最低级别为INFO，此时全部输出
        FLAGS_colorlogtostderr = true;                      // log为彩色
        FLAGS_stderrthreshold =
            google::INFO;   // INFO, WARNING, ERROR都输出，若为google::WARNING，则只输出WARNING, ERROR
        LOG(INFO) << "Logger init success!";
    }
    static void CloseLogger() { google::ShutdownGoogleLogging(); }

    static std::pair<std::string, std::string> LoadYaml(const std::string& logger_name) {
        std::filesystem::path exe_path      = std::filesystem::current_path();
        std::filesystem::path config_path   = exe_path.concat("/config/logger.yaml");
        YAML::Node            logger_config = YAML::LoadFile(config_path.string());
        YAML::Node            yaml_file     = logger_config[logger_name];
        std::string           log_dir       = yaml_file["log_dir"].as<std::string>();
        // check the logger directory is exist or not
        if (!std::filesystem::exists(log_dir) || !std::filesystem::is_directory(log_dir)) {
            std::filesystem::create_directories(log_dir);
        }
        std::string log_name = yaml_file["log_name"].as<std::string>();
        bool        is_log   = yaml_file["is_log"].as<bool>();
        if (!is_log) { FLAGS_minloglevel = google::FATAL + 1; }
        std::pair<std::string, std::string> log_info = std::make_pair(log_name, log_dir);
        return log_info;
    }
};

}   // namespace utils