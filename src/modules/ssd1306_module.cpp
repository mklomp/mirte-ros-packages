#include <algorithm>
#include <functional>

#include <boost/algorithm/string/replace.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/modules/ssd1306_module.hpp>

using namespace std::placeholders;  // for _1, _2, _3...

std::vector<std::shared_ptr<SSD1306_module>> SSD1306_module::get_ssd1306_modules(
  NodeData node_data, std::shared_ptr<Parser> parser, std::shared_ptr<tmx_cpp::Modules> modules)
{
  std::vector<std::shared_ptr<SSD1306_module>> new_modules;
  auto datas = parse_all_modules<SSD1306Data>(parser, node_data.board);
  for (auto data : datas) {
    auto module = std::make_shared<SSD1306_module>(node_data, data, modules);
    new_modules.push_back(module);
  }
  return new_modules;
}

SSD1306_module::SSD1306_module(
  NodeData node_data, SSD1306Data oled_data, std::shared_ptr<tmx_cpp::Modules> modules)
: Mirte_module(node_data, {oled_data.scl, oled_data.sda}, (ModuleData)oled_data), data(oled_data)
{
  tmx->setI2CPins(data.sda, data.scl, data.port);

  this->ssd1306 =
    std::make_shared<tmx_cpp::SSD1306_module>(data.port, data.addr, data.width, data.height);

  // FIXME: TO BE REMOVED
  // TODO: Or make configurable
  this->set_oled_service_legacy = nh->create_service<mirte_msgs::srv::SetOLEDImageLegacy>(
    "set_" + data.name + "_image_legacy",
    std::bind(&SSD1306_module::set_oled_callback_legacy, this, _1, _2));

  this->set_oled_text_service = nh->create_service<mirte_msgs::srv::SetOLEDText>(
    "set_" + data.name + "_text", std::bind(&SSD1306_module::set_oled_text_callback, this, _1, _2));

  this->set_oled_file_service = nh->create_service<mirte_msgs::srv::SetOLEDFile>(
    "set_" + data.name + "_file", std::bind(&SSD1306_module::set_oled_file_callback, this, _1, _2));

  modules->add_mod(this->ssd1306);
}

bool SSD1306_module::prewrite()
{
  default_image = false;
  if (!enabled) {
    RCLCPP_ERROR(nh->get_logger(), "Writing to OLED Module %s failed", data.name.c_str());
    return false;
  }
  return true;
}

bool SSD1306_module::set_text(std::string text)
{
  if (!prewrite()) return false;

  auto escaped_text = boost::algorithm::replace_all_copy(text, "\\n", "\n");
  if (escaped_text == last_text) return true;
  last_text = escaped_text;
  auto succes = ssd1306->send_text(escaped_text);
  if (not succes) enabled = false;

  return succes;
}

bool SSD1306_module::set_image(uint8_t width, uint8_t height, uint8_t img_buffer[])
{
  if (!prewrite()) return false;

  if (width != data.width || height != data.height) {
    RCLCPP_ERROR(
      nh->get_logger(), "Supplied image of wrong size. Expected %ux%u, got %ux%u", data.width,
      data.height, width, height);
    return false;
  }

  auto succes = ssd1306->send_image(width, height, img_buffer);
  if (not succes) enabled = false;

  return succes;
}

bool SSD1306_module::set_image_from_path(std::string path)
{
  fs::path actual_path;
  if (
    (path.size() > 6 && path.substr(0, 6) == "pkg://") ||
    (path.size() > 11 && path.substr(0, 10) == "package://")) {
    auto prefix_length = (path.substr(0, 6) == "pkg://") ? 6 : 10;
    auto data_path = path.substr(prefix_length);

    auto split_idx = data_path.find('/');

    auto package = data_path.substr(0, split_idx);
    auto remainder = data_path.substr(split_idx + 1);

    actual_path = fs::path(ament_index_cpp::get_package_share_directory(package)) / remainder;
  } else if (path[0] != '/') {
    // Check the local path from the DEFAULT_PACKAGE or Folder.
    // TODO: Make DEFAULT_LOCATION configurable
    std::string default_location = "/usr/local/src/mirte/mirte-oled-images";
    return set_image_from_path(default_location + "/" + path);
  } else {
    actual_path = fs::path(path);
  }

  return set_image_from_path(actual_path);
}

bool SSD1306_module::set_image_from_path(fs::path path)
{
  auto logger = nh->get_logger();

  /* Check if the specified path exists */
  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(logger, "The specified image path does not exist");
    return false;
  }

  if (path.has_filename() && path.has_extension()) {
    // Single Image
    cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    return set_image(image.cols, image.rows, image.data);
  } else {
    // Animation
    std::vector<cv::Mat> images;

    std::set<fs::path> paths;

    for (auto img_file : fs::directory_iterator(
           path, fs::directory_options::follow_directory_symlink |
                   fs::directory_options::skip_permission_denied))
      paths.insert(img_file);

    RCLCPP_DEBUG(logger, "Loading animation frames from %s", path.c_str());

    for (auto img_file : paths) {
      RCLCPP_DEBUG(logger, "Loading frame %ld: %s", images.size(), img_file.c_str());
      images.push_back(cv::imread(img_file.string(), cv::IMREAD_GRAYSCALE));
    }

    bool result = true;
    for (auto image : images) {
      result &= set_image(image.cols, image.rows, image.data);

      if (!result) {
        RCLCPP_ERROR(logger, "Some Error has occured during the animation playback.");
        return result;
      }
    }

    return result;
  }
}

void SSD1306_module::set_oled_callback_legacy(
  const std::shared_ptr<mirte_msgs::srv::SetOLEDImageLegacy::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetOLEDImageLegacy::Response> res)
{
  auto logger = nh->get_logger();

  // TODO: Maybe use default location as well?

  if (req->type == "text") {
    res->status = set_text(req->value);
  } else if (req->type == "image") {
    // If not an absolute path or a package local path use the old behavior
    auto path = (req->value[0] != '/' && req->value.substr(0, 6) != "pkg://" &&
                 req->value.substr(0, 10) != "package://" && req->value.rfind('.') == -1)
                  ? "/usr/local/src/mirte/mirte-oled-images/images/" + req->value + ".png"
                  : req->value;
    res->status = set_image_from_path(path);
  } else if (req->type == "animation") {
    auto path = (req->value[0] != '/' && req->value.substr(0, 6) != "pkg://" &&
                 req->value.substr(0, 10) != "package://" && req->value.back() != '/')
                  ? "/usr/local/src/mirte/mirte-oled-images/animations/" + req->value + "/"
                  : req->value;
    res->status = set_image_from_path(path);
  } else {
    RCLCPP_ERROR(
      logger, "'%s' is not a valid type. (Available: 'text', 'image', 'animation')",
      req->type.c_str());
    res->status = false;
  }
}

void SSD1306_module::set_oled_text_callback(
  const std::shared_ptr<mirte_msgs::srv::SetOLEDText::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetOLEDText::Response> res)
{
  res->status = set_text(req->text);
}

void SSD1306_module::set_oled_file_callback(
  const std::shared_ptr<mirte_msgs::srv::SetOLEDFile::Request> req,
  std::shared_ptr<mirte_msgs::srv::SetOLEDFile::Response> res)
{
  res->status = set_image_from_path(req->path);
}
