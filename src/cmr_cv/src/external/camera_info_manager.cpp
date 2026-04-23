/*
 * IMPORTANT CMR NOTE: this is copied from the rolling branch of the ros-perception
 * library, specifically this file:
 * https://github.com/ros-perception/image_common/pull/190/files#diff-11811183f44e7957caffe72285cb556d130b5046e45e173238198557fb97a4ee
 *
 * Once we upgrade to the next release of ROS2 after Humble, we should switch back
 * to using the camera_info_manager::CameraInfoManager class provided by the
 * library and delete this file.
 */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012 Jack O'Quin
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "cmr_cv/external/camera_info_manager.hpp"

#include <algorithm>
#include <cstdlib>
#include <locale>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "camera_calibration_parsers/parse.hpp"
#include "rcpputils/env.hpp"
#include "rcpputils/filesystem_helper.hpp"

/** @file

    @brief CameraInfo Manager implementation

    Provides CameraInfo, handles the SetCameraInfo service requests,
    saves and restores sensor_msgs/CameraInfo data.

    @author Jack O'Quin
 */

namespace cmr_cv
{

using camera_calibration_parsers::readCalibration;
using camera_calibration_parsers::writeCalibration;

/** URL to use when no other is defined. */
const std::string default_camera_info_url =
    "file://${ROS_HOME}/camera_info/${NAME}.yaml";

/** Constructor
 *
 * @param node node, normally for the driver's streaming name
 *           space ("camera").  The service name is relative to this
 *           handle.  Nodes supporting multiple cameras may use
 *           subordinate names, like "left/camera" and "right/camera".
 * @param cname default camera name
 * @param url default Uniform Resource Locator for loading and saving data.
 */
CameraInfoManager::CameraInfoManager(rclcpp::Node* node, const std::string& cname,
                                     const std::string& url)
    : CameraInfoManager(node->get_node_base_interface(),
                        node->get_node_services_interface(),
                        node->get_node_logging_interface(), cname, url)
{
}

CameraInfoManager::CameraInfoManager(rclcpp_lifecycle::LifecycleNode* node,
                                     const std::string& cname,
                                     const std::string& url)
    : CameraInfoManager(node->get_node_base_interface(),
                        node->get_node_services_interface(),
                        node->get_node_logging_interface(), cname, url)
{
}

// NOLINTNEXTLINE
CameraInfoManager::CameraInfoManager(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
        node_services_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logger_interface,
    // NOLINTNEXTLINE
    const std::string& cname, const std::string& url, rmw_qos_profile_t custom_qos)
    : m_logger(node_logger_interface->get_logger()),
      m_camera_name(cname),
      m_url(url),
      m_loaded_cam_info(false)
{
    using namespace std::placeholders;

    // register callback for camera calibration service request
    m_info_service = rclcpp::create_service<SetCameraInfo>(
        node_base_interface, node_services_interface, "~/set_camera_info",
        std::bind(&CameraInfoManager::set_camera_info_service, this, _1, _2),
        custom_qos, nullptr);
}

/** Get the current CameraInfo data.
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load is attempted but fails, an empty CameraInfo will be
 * supplied.
 *
 * The matrices are all zeros if no calibration is available. The
 * image pipeline handles that as uncalibrated data.
 *
 * @warning The caller @em must fill in the message Header of the
 *          CameraInfo returned.  The time stamp and frame_id should
 *          normally be the same as the corresponding Image message
 *          Header fields.
 */
CameraInfo CameraInfoManager::get_camera_info(void)
{
    while (rclcpp::ok()) {
        std::string cname;
        std::string url;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_loaded_cam_info) {
                return m_cam_info;  // all done
            }

            // load being attempted now
            m_loaded_cam_info = true;

            // copy the name and URL strings
            url = m_url;
            cname = m_camera_name;
        }  // release the lock

        // attempt load without the lock, it is not recursive
        load_calibration(url, cname);
    }

    return CameraInfo();
}

/** Get file name corresponding to a @c package: URL.
 *
 * @param url a copy of the Uniform Resource Locator
 * @return file name if package found, "" otherwise
 */
std::string CameraInfoManager::get_package_file_name(const std::string& url)
{
    RCLCPP_DEBUG(m_logger, "camera calibration url: %s", url.c_str());

    // Scan URL from after "package://" until next '/' and extract
    // package name.  The parseURL() already checked that it's present.
    size_t prefix_len = std::string("package://").length();
    size_t rest = url.find('/', prefix_len);
    std::string package(url.substr(prefix_len, rest - prefix_len));

    // Look up the ROS package path name.
    std::string pkg_path = ament_index_cpp::get_package_share_directory(package);
    if (pkg_path.empty()) {  // package not found?
        RCLCPP_WARN(m_logger, "unknown package: %s (ignored)", package.c_str());
        return pkg_path;
    } else {
        // Construct file name from package location and remainder of URL.
        return pkg_path + url.substr(rest);
    }
}

/** Is the current CameraInfo calibrated?
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load failed, CameraInfo will be empty and this predicate
 * will return false.
 *
 * @return true if the current CameraInfo is calibrated.
 */
bool CameraInfoManager::is_calibrated(void)
{
    while (true) {
        std::string cname;
        std::string url;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_loaded_cam_info) {
                return m_cam_info.k[0] != 0.0;
            }

            // load being attempted now
            m_loaded_cam_info = true;

            // copy the name and URL strings
            url = m_url;
            cname = m_camera_name;
        }  // release the lock

        // attempt load without the lock, it is not recursive
        load_calibration(url, cname);
    }
}

/** Load CameraInfo calibration data (if any).
 *
 * @pre m_mutex unlocked
 *
 * @param url a copy of the Uniform Resource Locator
 * @param cname is a copy of the m_camera_name
 * @return true if URL contains calibration data.
 *
 * sets m_cam_info, if successful
 */
bool CameraInfoManager::load_calibration(const std::string& url,
                                         const std::string& cname)
{
    bool success = false;  // return value

    const std::string res_url(resolve_url(url, cname));
    auto url_type = parse_url(res_url);

    if (url_type != UrlEmpty) {
        RCLCPP_INFO(m_logger, "camera calibration URL: %s", res_url.c_str());
    }

    switch (url_type) {
        case UrlEmpty: {
            RCLCPP_INFO(m_logger, "using default calibration URL");
            success = load_calibration(default_camera_info_url, cname);
            break;
        }
        case UrlFile: {
            success = load_calibration_file(res_url.substr(7), cname);
            break;
        }
        case UrlFlash: {
            RCLCPP_WARN(m_logger, "reading from flash not implemented yet");
            break;
        }
        case UrlPackage: {
            std::string filename(get_package_file_name(res_url));
            if (!filename.empty()) {
                success = load_calibration_file(filename, cname);
            }
            break;
        }
        default: {
            RCLCPP_ERROR(m_logger, "Invalid camera calibration URL: %s",
                         res_url.c_str());
            break;
        }
    }

    return success;
}

/** Load CameraInfo calibration data from a file.
 *
 * @pre m_mutex unlocked
 *
 * @param filename containing CameraInfo to read
 * @param cname is a copy of the m_camera_name
 * @return true if URL contains calibration data.
 *
 * Sets m_cam_info, if successful
 */
bool CameraInfoManager::load_calibration_file(const std::string& filename,
                                              const std::string& cname)
{
    bool success = false;

    RCLCPP_DEBUG(m_logger, "reading camera calibration from %s", filename.c_str());
    std::string cam_name;
    CameraInfo cam_info;

    if (readCalibration(filename, cam_name, cam_info)) {
        if (cname != cam_name) {
            RCLCPP_WARN(m_logger, "[%s] does not match %s in file %s", cname.c_str(),
                        cam_name.c_str(), filename.c_str());
        }
        success = true;
        {
            // lock only while updating m_cam_info
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cam_info = cam_info;
        }
    } else {
        RCLCPP_WARN(m_logger, "Camera calibration file %s not found",
                    filename.c_str());
    }

    return success;
}

/** Set a new URL and load its calibration data (if any).
 *
 * If multiple threads call this method simultaneously with different
 * URLs, there is no guarantee which will prevail.
 *
 * @param url new Uniform Resource Locator for CameraInfo.
 * @return true if new URL contains calibration data.
 *
 * @post @c m_loaded_cam_info true (meaning a load was attempted, even
 *       if it failed); @c m_cam_info updated, if successful.
 */
bool CameraInfoManager::load_camera_info(const std::string& url)
{
    std::string cname;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_url = url;
        cname = m_camera_name;
        m_loaded_cam_info = true;
    }

    // load using copies of the parameters, no need to hold the lock
    return load_calibration(url, cname);
}

/** Resolve Uniform Resource Locator string.
 *
 * @param url a copy of the Uniform Resource Locator, which may
 *            include <tt>${...}</tt> substitution variables.
 * @param cname is a copy of the m_camera_name
 *
 * @return a copy of the URL with any variable information resolved.
 */
// NOLINTNEXTLINE
std::string CameraInfoManager::resolve_url(const std::string& url,
                                           const std::string& cname)
{
    std::string resolved;
    size_t rest = 0;

    while (true) {
        // find the next '$' in the URL string
        size_t dollar = url.find('$', rest);

        if (dollar >= url.length()) {
            // no more variables left in the URL
            resolved += url.substr(rest);
            break;
        }

        // copy characters up to the next '$'
        resolved += url.substr(rest, dollar - rest);

        if (url.substr(dollar + 1, 1) != "{") {
            // no '{' follows, so keep the '$'
            resolved += "$";
        } else if (url.substr(dollar + 1, 6) == "{NAME}") {
            // substitute camera name
            resolved += cname;
            dollar += 6;
        } else if (url.substr(dollar + 1, 10) == "{ROS_HOME}") {
            // substitute $ROS_HOME
            std::string ros_home;
            std::string ros_home_env = rcpputils::get_env_var("ROS_HOME");
            std::string home_env = rcpputils::get_env_var("HOME");
            if (!ros_home_env.empty()) {
                // use environment variable
                ros_home = ros_home_env;
            } else if (!home_env.empty()) {
                // use "$HOME/.ros"
                ros_home = home_env;
                ros_home += "/.ros";
            }
            resolved += ros_home;
            dollar += 10;
        } else {
            // not a valid substitution variable
            RCLCPP_ERROR(m_logger, "invalid URL substitution (not resolved): %s",
                         url.c_str());
            resolved += "$";  // keep the bogus '$'
        }

        // look for next '$'
        rest = dollar + 1;
    }

    return resolved;
}

/** Parse calibration Uniform Resource Locator.
 *
 * @param url string to parse
 * @return URL type
 *
 * @note Recognized but unsupported URL types have enum values >= m_urlinvalid.
 */
CameraInfoManager::url_type_t CameraInfoManager::parse_url(const std::string& url)
{
    if (url.empty()) {
        return UrlEmpty;
    }

    // Easy C++14 replacement for boost::iequals from :
    // https://stackoverflow.com/a/4119881
    auto iequals = [](const std::string& a, const std::string& b) {
        return std::equal(a.begin(), a.end(), b.begin(), b.end(),
                          [](char c, char d) { return tolower(c) == tolower(d); });
    };

    if (iequals(url.substr(0, 8), "file:///")) {
        return UrlFile;
    }
    if (iequals(url.substr(0, 9), "flash:///")) {
        return UrlFlash;
    }
    if (iequals(url.substr(0, 10), "package://")) {
        // look for a '/' following the package name, make sure it is
        // there, the name is not empty, and something follows it
        size_t rest = url.find('/', 10);
        if (rest < url.length() - 1 && rest > 10) {
            return UrlPackage;
        }
    }
    return UrlInvalid;
}

/** Save CameraInfo calibration data.
 *
 * @pre m_mutex unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param url is a copy of the URL storage location (if empty, use
 *            @c file://${ROS_HOME}/camera_info/${NAME}.yaml)
 * @param cname is a copy of the m_camera_name
 * @return true, if successful
 */
bool CameraInfoManager::save_calibration(const CameraInfo& new_info,
                                         const std::string& url,
                                         const std::string& cname)
{
    bool success = false;

    const std::string res_url(resolve_url(url, cname));

    switch (parse_url(res_url)) {
        case UrlEmpty: {
            // store using default file name
            success = save_calibration(new_info, default_camera_info_url, cname);
            break;
        }
        case UrlFile: {
            success = save_calibration_file(new_info, res_url.substr(7), cname);
            break;
        }
        case UrlPackage: {
            std::string filename(get_package_file_name(res_url));
            if (!filename.empty()) {
                success = save_calibration_file(new_info, filename, cname);
            }
            break;
        }
        default: {
            // invalid URL, save to default location
            RCLCPP_ERROR(m_logger, "invalid url: %s (ignored)", res_url.c_str());
            success = save_calibration(new_info, default_camera_info_url, cname);
            break;
        }
    }

    return success;
}

/** Save CameraInfo calibration data to a file.
 *
 * @pre m_mutex unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param filename is local file to store data
 * @param cname is a copy of the m_camera_name
 * @return true, if successful
 */
bool CameraInfoManager::save_calibration_file(const CameraInfo& new_info,
                                              const std::string& filename,
                                              const std::string& cname)
{
    RCLCPP_INFO(m_logger, "writing calibration data to %s", filename.c_str());

    rcpputils::fs::path filepath(filename);
    rcpputils::fs::path parent = filepath.parent_path();

    if (!rcpputils::fs::exists(parent)) {
        if (!rcpputils::fs::create_directories(parent)) {
            RCLCPP_ERROR(m_logger, "unable to create path directory [%s]",
                         parent.string().c_str());
            return false;
        }
    }

    // Directory exists. Permissions might still be bad.
    // Currently, writeCalibration() always returns true no matter what
    // (ros-pkg ticket #5010).
    return writeCalibration(filename, cname, new_info);
}

/** Callback for SetCameraInfo request.
 *
 * Always updates m_cam_info class variable, even if save fails.
 *
 * @param req SetCameraInfo request message
 * @param rsp SetCameraInfo response message
 * @return true if message handled
 */
void CameraInfoManager::set_camera_info_service(
    const std::shared_ptr<SetCameraInfo::Request> req,
    std::shared_ptr<SetCameraInfo::Response> rsp)
{
    // copies of class variables needed for saving calibration
    std::string m_urlcopy;
    std::string cname;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cam_info = req->camera_info;
        m_urlcopy = m_url;
        cname = m_camera_name;
        m_loaded_cam_info = true;
    }

    if (!rclcpp::ok()) {
        RCLCPP_ERROR(m_logger,
                     "set_camera_info service called, but driver not running.");
        rsp->status_message = "Camera driver not running.";
        rsp->success = false;
        return;
    }

    rsp->success = save_calibration(req->camera_info, m_urlcopy, cname);
    if (!rsp->success) {
        rsp->status_message = "Error storing camera calibration.";
    }
}

/** Set a new camera name.
 *
 * @param cname new camera name to use for saving calibration data
 *
 * @return true if new name has valid syntax; valid names contain only
 *              alphabetic, numeric, or '_' characters.
 *
 * @post @c cam_name_ updated, if valid; since it may affect the URL,
 *       @c m_cam_info will be reloaded before being used again.
 */
bool CameraInfoManager::set_camera_name(const std::string& cname)
{
    // the camera name may not be empty
    if (cname.empty()) {
        return false;
    }

    // validate the camera name characters
    for (unsigned i = 0; i < cname.size(); ++i) {
        if ((isalnum(cname[i]) == 0) && cname[i] != '_') {
            return false;
        }
    }

    // The name is valid, so update our private copy.  Since the new
    // name might cause the existing URL to resolve somewhere else,
    // force @c m_cam_info to be reloaded before being used again.
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_camera_name = cname;
        m_loaded_cam_info = false;
    }

    return true;
}

/** Set the camera info manually
 *
 * @param camera_info new camera calibration data
 *
 * @return true if new camera info is set
 *
 * @post @c m_cam_info updated, if valid;
 */
bool CameraInfoManager::set_camera_info(const CameraInfo& camera_info)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_cam_info = camera_info;
    m_loaded_cam_info = true;

    return true;
}

/** Validate URL syntax.
 *
 * @param url Uniform Resource Locator to check
 *
 * @return true if URL syntax is supported by CameraInfoManager
 *              (although the resource need not actually exist)
 */
bool CameraInfoManager::validate_url(const std::string& url)
{
    std::string cname;  // copy of camera name
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cname = m_camera_name;
    }  // release the lock

    url_type_t m_urltype = parse_url(resolve_url(url, cname));
    return m_urltype < UrlInvalid;
}
}  // namespace cmr_cv