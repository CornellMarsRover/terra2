#include "cmr_fabric_wrappers/subprocess_node.hpp"

#include <regex>

#include "cmr_utils/external/tomlcpp.hxx"

namespace cmr
{

const std::regex g_special_chars(
    "[\u001b\u009b][[()#;?]*(?:[0-9]{1,4}(?:;[0-9]{0,4})*)?[0-9A-ORZcf-nqry=><]");

SubprocessNode::SubprocessNode(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
}

bool SubprocessNode::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    const auto node_settings = table->getTable("node");

    auto [cmd_ok, cmd] = node_settings->getString("command");
    if (!cmd_ok) {
        CMR_LOG(ERROR, "Subprocess node failed to parse string 'command'.");
        return false;
    }
    m_cmd = cmd;

    auto [ind_ok, indicator] = node_settings->getString("started_indicator");
    if (!ind_ok) {
        CMR_LOG(ERROR,
                "Subprocess node failed to parse string 'started_indicator'.");
        return false;
    }
    m_started_indicator = indicator;

    auto [bool_ok, expects_output] = node_settings->getBool("expects_output");
    if (!bool_ok) {
        CMR_LOG(ERROR, "Subprocess node failed to parse bool 'expects_output'.");
        return false;
    }
    m_expects_output = expects_output;

    auto [int_ok, indicator_timeout] = node_settings->getInt("indicator_timeout");
    if (!int_ok) {
        CMR_LOG(ERROR, "Subprocess node failed to parse int 'indicator_timeout'.");
        return false;
    }
    m_indicator_timeout = indicator_timeout;

    return true;
}

// NOLINTNEXTLINE(readability-function-size)
bool SubprocessNode::activate()
{
    // reset the stream
    m_stream = redi::pstream{};
    // execute the command
    m_stream.open(m_cmd, redi::pstream::pstdin | redi::pstream::pstdout |
                             redi::pstream::pstderr | redi::pstream ::newpg);
    CMR_LOG(INFO, "Executing command: %s", m_cmd.c_str());

    // no need to perform additional checks if the user specified that no output is
    // expected.
    if (!m_expects_output) {
        return true;
    }

    // reset flags
    m_out_read_done = false;
    m_err_read_done = false;
    m_found_indicator = false;

    // read the output and error streams in a separate thread so that we don't
    // lock this node up while the command runs.
    m_out_thread = std::thread([this]() {
        while (true) {
            std::unique_lock<std::mutex> lk(m_mutex);
            if (m_out_read_done && m_err_read_done) {
                break;
            }

            read_stream(m_stream.err(), m_err_read_done, m_err_buf, m_out_read_done,
                        true);

            read_stream(m_stream.out(), m_out_read_done, m_out_buf, m_err_read_done,
                        false);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        m_cv.notify_all();
    });

    // check if we saw the indicator; if we did, activation was successful.
    {
        std::unique_lock<std::mutex> lk(m_mutex);
        if (m_cv.wait_for(lk, std::chrono::seconds(m_indicator_timeout),
                          [this] { return m_found_indicator; })) {
            return true;
        }
    }

    CMR_LOG(ERROR, "Did not find indicator (%s) within the timeout of %lds. Stop.",
            m_started_indicator.c_str(), m_indicator_timeout);
    kill_command();
    return false;
}

bool SubprocessNode::deactivate()
{
    if (!kill_command()) {
        CMR_LOG(ERROR, "Failed to kill process; you will have to kill it manually.");
        return false;
    }
    CMR_LOG(INFO, "Process terminated.");
    return true;
}

bool SubprocessNode::cleanup() { return true; }

bool SubprocessNode::kill_command()
{
    bool done = false;

    std::array<int, 3> sig = {SIGINT, SIGTERM, SIGKILL};
    std::array<std::string, 3> sig_names = {"SIGINT", "SIGTERM", "SIGKILL"};
    size_t sig_id = 0;

    do {
        m_stream.rdbuf()->killpg(sig.at(sig_id));
        std::unique_lock<std::mutex> lk(m_mutex);
        if (m_cv.wait_for(lk, std::chrono::seconds(5),
                          [this] { return m_out_read_done && m_err_read_done; })) {
            done = true;
        } else {
            sig_id++;
            if (sig_id == sig.size()) {
                // out of options, just let the user deal with it
                return false;
            }
            CMR_LOG(WARN, "Process still running. Escalating to %s.",
                    sig_names.at(sig_id).c_str());
        }
    } while (!done);

    m_stream.clear();
    m_out_thread.join();
    return true;
}

// NOLINTNEXTLINE(readability-function-size)
void SubprocessNode::read_stream(redi::pstream& in, bool& done_flag,
                                 std::stringstream& out, bool& other_flag,
                                 bool error)
{
    // Code adapted from bottom of https://pstreams.sourceforge.net/doc/
    std::array<char, 1024> buf{};
    std::streamsize n = 0;
    if (done_flag) {
        return;
    }

    while ((n = in.readsome(buf.begin(), sizeof(buf))) > 0) {
        for (std::streamsize i = 0; i < n; i++) {
            auto data = buf.at(static_cast<std::size_t>(i));
            out << data;
            if (data != '\n') {
                continue;
            }

            std::string line;
            std::getline(out, line);
            if (error) {
                CMR_LOG(ERROR, "%s", line.c_str());
            } else {
                CMR_LOG(INFO, "%s", line.c_str());
            }

            // remove special characters from line for comparison
            auto stripped_line = std::regex_replace(line, g_special_chars, "");

            // check for the startup indicator
            if (!m_found_indicator &&
                stripped_line.find(m_started_indicator) != std::string::npos) {
                m_found_indicator = true;
                m_cv.notify_all();
            }
        }
    }
    if (m_stream.eof()) {
        done_flag = true;
        if (!other_flag) {
            m_stream.clear();
        }
    }
}

}  // namespace cmr
