/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CASensorSim boot logo: the application-side override of the banner
 * library's weak banner_logo() hook (enabled with CONFIG_BANNER_LOGO).
 * The block letters spell the product name; the Unicode art needs a
 * UTF-8 terminal.
 */

#include <zephyr/kernel.h>

#include "banner.h"

#define LOGO_CYAN  "\033[1;36m"
#define LOGO_RESET "\033[0m"

extern "C" void banner_logo(void)
{
	printk(LOGO_CYAN
	       "   ███████╗  █████╗  ███████╗ ██╗ ███╗   ███╗\n"
	       "   ██╔════╝ ██╔══██╗ ██╔════╝ ██║ ████╗ ████║\n"
	       "   ██║      ███████║ ███████╗ ██║ ██╔████╔██║\n"
	       "   ██║      ██╔══██║ ╚════██║ ██║ ██║╚██╔╝██║\n"
	       "   ╚██████╗ ██║  ██║ ███████║ ██║ ██║ ╚═╝ ██║\n"
	       "   ╚═════╝  ╚═╝  ╚═╝ ╚══════╝ ╚═╝ ╚═╝     ╚═╝\n"
	       LOGO_RESET);
}
