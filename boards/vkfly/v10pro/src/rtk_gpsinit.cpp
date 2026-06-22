/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

namespace
{

constexpr const char *default_device = "/dev/ttyS1";
constexpr unsigned default_baud = 460800;
constexpr useconds_t default_delay_us = 20000;

struct Config {
	const char *device{default_device};
	unsigned baud{default_baud};
	useconds_t delay_us{default_delay_us};
	const char *messages[8] {};
	int message_count{0};
};

speed_t baud_to_speed(unsigned baud)
{
	switch (baud) {
	case 9600: return B9600;

	case 19200: return B19200;

	case 38400: return B38400;

	case 57600: return B57600;

	case 115200: return B115200;

	case 230400: return B230400;
#ifdef B460800

	case 460800: return B460800;
#endif
#ifdef B921600

	case 921600: return B921600;
#endif

	default: return static_cast<speed_t>(0);
	}
}

int open_and_configure(const char *device, unsigned baud)
{
	const speed_t speed = baud_to_speed(baud);

	if (speed == static_cast<speed_t>(0)) {
		PX4_ERR("unsupported baud: %u", baud);
		return -1;
	}

	const int fd = ::open(device, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("open %s failed (%d)", device, errno);
		return -1;
	}

	struct termios uart_config {};

	if (tcgetattr(fd, &uart_config) != 0) {
		PX4_ERR("tcgetattr failed (%d)", errno);
		::close(fd);
		return -1;
	}

	cfmakeraw(&uart_config);
	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);
	uart_config.c_cflag |= (CLOCAL | CREAD);

	if (tcsetattr(fd, TCSANOW, &uart_config) != 0) {
		PX4_ERR("tcsetattr failed (%d)", errno);
		::close(fd);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);
	return fd;
}

int send_message(int fd, const char *message, useconds_t delay_us)
{
	const size_t len = strlen(message);
	const ssize_t written = ::write(fd, message, len);

	if (written != static_cast<ssize_t>(len)) {
		PX4_ERR("write failed for '%s' (%d)", message, errno);
		return -1;
	}

	tcdrain(fd);
	usleep(delay_us);
	return 0;
}

void add_default_unicore_messages(Config &config)
{
	config.messages[config.message_count++] = "UNLOG\r\n";
	config.messages[config.message_count++] = "mode rover uav default\r\n";
	config.messages[config.message_count++] = "GPGGA 0.1\r\n";
	config.messages[config.message_count++] = "GPRMC 0.1\r\n";
	config.messages[config.message_count++] = "GPHDT 0.1\r\n";
	// config.messages[config.message_count++] = "UNIAGRICA 0.1\r\n";
	// config.messages[config.message_count++] = "UNIHEADINGA 0.1\r\n";
	config.messages[config.message_count++] = "GPGST 0.2\r\n";
	config.messages[config.message_count++] = "GPGSA 0.2\r\n";
	// config.messages[config.message_count++] = "GPZDA 1.0\r\n";
}

void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Board-local helper for sending GPS initialization strings before the PX4 GPS driver starts.
Use this to keep Unicore-specific UART setup inside the V10Pro board directory.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("rtk_gpsinit", "command");
	PRINT_MODULE_USAGE_ARG("unicore", "Send built-in Unicore rover/NMEA/heading setup", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', default_device, "<device>", "UART device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', default_baud, 9600, 921600, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('w', default_delay_us / 1000, 0, 1000, "Delay between messages in ms", true);
	PRINT_MODULE_USAGE_PARAM_STRING('m', nullptr, "<message>", "Raw message to send (repeatable, CRLF added if missing)", true);
}

} // namespace

extern "C" __EXPORT int rtk_gpsinit_main(int argc, char *argv[])
{
	Config config {};
	bool use_unicore_defaults = false;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:w:m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			config.device = myoptarg;
			break;

		case 'b':
			config.baud = strtoul(myoptarg, nullptr, 10);
			break;

		case 'w':
			config.delay_us = static_cast<useconds_t>(strtoul(myoptarg, nullptr, 10) * 1000);
			break;

		case 'm':
			if (config.message_count >= static_cast<int>(sizeof(config.messages) / sizeof(config.messages[0]))) {
				PX4_ERR("too many messages");
				return -1;
			}

			config.messages[config.message_count++] = myoptarg;
			break;

		default:
			print_usage();
			return -1;
		}
	}

	if (myoptind < argc) {
		if (strcmp(argv[myoptind], "unicore") == 0) {
			use_unicore_defaults = true;

		} else {
			print_usage();
			return -1;
		}
	}

	if (use_unicore_defaults) {
		add_default_unicore_messages(config);
	}

	if (config.message_count == 0) {
		print_usage();
		return -1;
	}

	const int fd = open_and_configure(config.device, config.baud);

	if (fd < 0) {
		return -1;
	}

	for (int i = 0; i < config.message_count; i++) {
		char buffer[160] {};
		const char *message = config.messages[i];
		const bool needs_crlf = (strstr(message, "\r\n") == nullptr);

		if (needs_crlf) {
			snprintf(buffer, sizeof(buffer), "%s\r\n", message);
			message = buffer;
		}

		if (send_message(fd, message, config.delay_us) != 0) {
			::close(fd);
			return -1;
		}
	}

	::close(fd);
	return 0;
}
