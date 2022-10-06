#include "cmr_bs/joystick.hpp"

//For joystick callback function
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>


namespace cmr
{

Joystick::Joystick(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
* Returns the number of axes on the controller or 0 if an error occurs.
*/
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
* Returns the number of buttons on the controller or 0 if an error occurs.
*/
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
* Current state of an axis.
*/
struct axis_state {
    short x, y;
};

/**
* Keeps track of the current axis state.
*
* NOTE: This function assumes that axes are numbered starting from 0, and that
* the X axis is an even number, and the Y axis is an odd number. However, this
* is usually a safe assumption.
*
* Returns the axis that the event indicated.
*/
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

void joystick_callback()
{
    const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    if (argc > 1)
        device = argv[1];
    else
        device = "/dev/input/js0";

    js = open(device, O_RDONLY);

    if (js == -1)
        perror("Could not open joystick");

    /* This loop will exit if the controller is unplugged. */

    
    switch (event.type)
    {
        case JS_EVENT_BUTTON:
            printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            break;
        case JS_EVENT_AXIS:
            axis = get_axis_state(&event, axes);
            if (axis < 3)
                printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
            break;
        default:
            /* Ignore init events. */
            break;
    }
    
    fflush(stdout);
    close(js);
}

bool Joystick::configure(const std::shared_ptr<toml::Table>&)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    // Ex. const auto node_settings = table->getTable("node");
    return true;
}

bool Joystick::activate()
{
    walltimer_ = ros::NodeHandle::createWallTimer(WallDuration 100ms, 
                void(T::*)(const WallTimerEvent &) joystick_callback(),
                T * Joystick );
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool Joystick::deactivate()
{
    // undo the effects of activate here
    walltimer_ = ros::NodeHandle::destruct()
    return true;
}

bool Joystick::cleanup()
{
    // undo the effects of configure here

    return true;
}

}  // namespace cmr
