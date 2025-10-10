#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/trajectory_setpoint.h>

#include <drivers/device/Device.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <mathlib/math/Functions.hpp>

extern "C" __EXPORT int arduino_bridge_main(int argc, char *argv[]);

namespace {

static int g_task{-1};
static volatile bool g_should_exit{false};

struct SerialConfig {
    const char *device{ "/dev/ttyS1" }; // default TELEM1
    int baud{ 57600 };
    int rate_hz{ 30 }; // 20-50Hz
};

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 921600: return B921600;
    default: return B57600;
    }
}

static int open_serial(const SerialConfig &cfg)
{
    int fd = ::open(cfg.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        PX4_ERR("open %s failed: %d", cfg.device, errno);
        return -errno;
    }

    struct termios t{};
    if (tcgetattr(fd, &t) != 0) {
        PX4_ERR("tcgetattr failed: %d", errno);
        ::close(fd);
        return -errno;
    }

    cfmakeraw(&t);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~CSTOPB; // 1 stop bit
    t.c_cflag &= ~CRTSCTS; // no HW flow

    const speed_t spd = baud_to_speed(cfg.baud);
    cfsetispeed(&t, spd);
    cfsetospeed(&t, spd);

    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        PX4_ERR("tcsetattr failed: %d", errno);
        ::close(fd);
        return -errno;
    }

    return fd;
}

static float wrap_pi(float a)
{
    while (a > M_PI) a -= 2.f * M_PI;
    while (a < -M_PI) a += 2.f * M_PI;
    return a;
}

static int run(const SerialConfig &cfg)
{
    int sfd = open_serial(cfg);
    if (sfd < 0) {
        return sfd;
    }

    uORB::Subscription sub_lpos{ORB_ID(vehicle_local_position)};
    uORB::Subscription sub_att{ORB_ID(vehicle_attitude)};
    uORB::Subscription sub_traj{ORB_ID(trajectory_setpoint)};

    vehicle_local_position_s lpos{};
    vehicle_attitude_s att{};
    trajectory_setpoint_s traj{};

    const int interval_us = 1000000 / math::constrain(cfg.rate_hz, 20, 50);

    while (!g_should_exit) {
        bool have_pose = false;
        bool have_att = false;
        bool have_target = false;

        if (sub_lpos.updated()) {
            sub_lpos.copy(&lpos);
            if (PX4_ISFINITE(lpos.x) && PX4_ISFINITE(lpos.y)) {
                have_pose = true;
            }
        }

        if (sub_att.updated()) {
            sub_att.copy(&att);
            have_att = true;
        }

        if (sub_traj.updated()) {
            sub_traj.copy(&traj);
            if (PX4_ISFINITE(traj.position[0]) && PX4_ISFINITE(traj.position[1])) {
                have_target = true;
            }
        }

        if (have_pose && have_att) {
            // NED -> ENU
            const float x_e = lpos.y; // y_ned -> x_enu
            const float y_e = lpos.x; // x_ned -> y_enu

            // yaw from quaternion (NED)
            const float qw = att.q[0];
            const float qx = att.q[1];
            const float qy = att.q[2];
            const float qz = att.q[3];
            const float yaw_ned = atan2f(2.f * (qw*qz + qx*qy), 1.f - 2.f * (qy*qy + qz*qz));
            const float yaw_enu = wrap_pi(yaw_ned + M_PI_2_F);

            char buf[96];
            int n = snprintf(buf, sizeof(buf), "POSE,%.3f,%.3f,%.4f\n", (double)x_e, (double)y_e, (double)yaw_enu);
            if (n > 0) {
                (void)::write(sfd, buf, (size_t)n);
            }
        }

        if (have_target) {
            const float tx_e = traj.position[1]; // y_ned -> x_enu
            const float ty_e = traj.position[0]; // x_ned -> y_enu
            char tbuf[64];
            int tn = snprintf(tbuf, sizeof(tbuf), "TARGET,%.3f,%.3f\n", (double)tx_e, (double)ty_e);
            if (tn > 0) {
                (void)::write(sfd, tbuf, (size_t)tn);
            }
        }

        px4_usleep(interval_us);
    }

    ::close(sfd);
    return 0;
}

static void usage()
{
    PX4_INFO("usage: arduino_bridge {start|stop|status} [-d device] [-b baud] [-r rate_hz]");
}

} // namespace

int arduino_bridge_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage();
        return -1;
    }

    if (!strcmp(argv[1], "start")) {
        if (g_task > 0) {
            PX4_INFO("already running");
            return 0;
        }

        SerialConfig cfg{};

        int ch;
        int myoptind = 1;
        const char *myoptarg = nullptr;
        while ((ch = px4_getopt(argc, argv, "d:b:r:", &myoptind, &myoptarg)) != EOF) {
            switch (ch) {
            case 'd': cfg.device = myoptarg; break;
            case 'b': cfg.baud = atoi(myoptarg); break;
            case 'r': cfg.rate_hz = atoi(myoptarg); break;
            default: usage(); return -1;
            }
        }

        // 最小实现：前台运行，便于在 extras.txt/NSH 直接启动，并确保参数立即生效
        g_should_exit = false;
        return run(cfg);

    } else if (!strcmp(argv[1], "stop")) {
        g_should_exit = true;
        return 0;

    } else if (!strcmp(argv[1], "status")) {
        PX4_INFO("running=%s", g_should_exit ? "no" : "yes");
        return 0;
    }

    usage();
    return -1;
}


