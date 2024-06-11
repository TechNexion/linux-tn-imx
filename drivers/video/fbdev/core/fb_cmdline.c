/*
 *  linux/drivers/video/fb_cmdline.c
 *
 *  Copyright (C) 2014 Intel Corp
 *  Copyright (C) 1994 Martin Schaller
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 * Authors:
 *    Vetter <danie.vetter@ffwll.ch>
 */
#include <linux/init.h>
#include <linux/fb.h>

static int ofonly __read_mostly;

const char *fb_mode_option;
EXPORT_SYMBOL_GPL(fb_mode_option);

/*
 * FB_MAX is the maximum number of framebuffer devices and also
 * the maximum number of video= parameters. Although not directly
 * related to each other, it makes sense to keep it that way.
 */
static const char *video_options[FB_MAX] __read_mostly;
static const char *video_option __read_mostly;
static int video_of_only __read_mostly;

static const char *__video_get_option_string(const char *name)
{
	const char *options = NULL;
	size_t name_len = 0;

	if (name)
		name_len = strlen(name);

	if (name_len) {
		unsigned int i;
		const char *opt;

		for (i = 0; i < ARRAY_SIZE(video_options); ++i) {
			if (!video_options[i])
				continue;
			if (video_options[i][0] == '\0')
				continue;
			opt = video_options[i];
			if (!strncmp(opt, name, name_len) && opt[name_len] == ':')
				options = opt + name_len + 1;
		}
	}

	/* No match, return global options */
	if (!options)
		options = video_option;

	return options;
}

/**
 * video_get_options - get kernel boot parameters
 * @name:	name of the output as it would appear in the boot parameter
 *		line (video=<name>:<options>)
 *
 * Looks up the video= options for the given name. Names are connector
 * names with DRM, or driver names with fbdev. If no video option for
 * the name has been specified, the function returns the global video=
 * setting. A @name of NULL always returns the global video setting.
 *
 * Returns:
 * The string of video options for the given name, or NULL if no video
 * option has been specified.
 */
const char *video_get_options(const char *name)
{
	return __video_get_option_string(name);
}
EXPORT_SYMBOL(video_get_options);

bool __video_get_options(const char *name, const char **options, bool is_of)
{
	bool enabled = true;
	const char *opt = NULL;

	if (video_of_only && !is_of)
		enabled = false;

	opt = __video_get_option_string(name);

	if (options)
		*options = opt;

	return enabled;
}
EXPORT_SYMBOL(__video_get_options);

/**
 * fb_get_options - get kernel boot parameters
 * @name:   framebuffer name as it would appear in
 *          the boot parameter line
 *          (video=<name>:<options>)
 * @option: the option will be stored here
 *
 * NOTE: Needed to maintain backwards compatibility
 */
int fb_get_options(const char *name, char **option)
{
	const char *options = NULL;
	bool is_of = false;
	bool enabled;

	if (name)
		is_of = strncmp(name, "offb", 4);

	enabled = __video_get_options(name, &options, is_of);

	if (options) {
		if (!strncmp(options, "off", 3))
			enabled = false;
	}

	if (option) {
		if (options)
			*option = kstrdup(options, GFP_KERNEL);
		else
			*option = NULL;
	}

	return enabled ? 0 : 1; // 0 on success, 1 otherwise
}
EXPORT_SYMBOL(fb_get_options);

/**
 *	video_setup - process command line options
 *	@options: string of options
 *
 *	Process command line options for frame buffer subsystem.
 *
 *	NOTE: This function is a __setup and __init function.
 *            It only stores the options.  Drivers have to call
 *            fb_get_options() as necessary.
 */
static int __init video_setup(char *options)
{
	if (!options || !*options)
		goto out;

	if (!strncmp(options, "ofonly", 6)) {
		ofonly = 1;
		goto out;
	}

	if (strchr(options, ':')) {
		/* named */
		int i;

		for (i = 0; i < FB_MAX; i++) {
			if (video_options[i] == NULL) {
				video_options[i] = options;
				break;
			}
		}
	} else {
		/* global */
		fb_mode_option = options;
	}

out:
	return 1;
}
__setup("video=", video_setup);
