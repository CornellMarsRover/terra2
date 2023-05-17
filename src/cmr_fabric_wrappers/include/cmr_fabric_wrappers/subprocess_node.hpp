#pragma once
#include <pstreams/pstream.h>

#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

#include "cmr_fabric/fabric_node.hpp"

namespace cmr
{

/**
 * A `SubprocessNode` is a Fabric-managed way of running a particular command in
 * its own process.
 */
class SubprocessNode : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `SubprocessNode`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit SubprocessNode(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    std::string m_cmd;
    // search string that indicates the command started successfully
    std::string m_started_indicator;
    // if true, the node will not consider the command successful if it does not
    // print output
    bool m_expects_output = false;
    // if this number of seconds is exceeded without seeing the m_started_indicator,
    // then the node will fail to activate.
    int64_t m_indicator_timeout = 0;

    redi::pstream m_stream;
    std::thread m_out_thread;
    std::mutex m_mutex;
    std::condition_variable m_cv;

    std::stringstream m_out_buf, m_err_buf;

    bool m_out_read_done = false, m_err_read_done = false;
    bool m_found_indicator =
        false;  // true if m_started_indicator is found in output

    /**
     * @brief Reads from the specified stream, logs out full lines, and buffers
     * the remaining characters to the provided buffer.
     *
     * @param in the input stream
     * @param done_flag will be set to true when the stream hits EOF, indicating that
     * the stream is done.
     * @param other_flag will be set to true if the other stream hit an EOF.
     * @param out the output buffer
     * @param error true if log messages should be ERROR, false if they should be
     * INFO.
     */
    void read_stream(redi::pstream& in, bool& done_flag, std::stringstream& out,
                     bool& other_flag, bool error);

    /**
     * @brief Kills the running command. Returns true on success and false otherwise.
     */
    bool kill_command();

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};

}  // namespace cmr