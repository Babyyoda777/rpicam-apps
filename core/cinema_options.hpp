/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * cinema_options.hpp - options for rpicam-cinema (CinemaDNG sequence).
 *
 * Output layout (folder-per-clip):
 *   <output>/<reel>/<clip>/000000.dng
 */

#pragma once

#include <string>

#include "still_options.hpp"

struct CinemaOptions : public StillOptions
{
	CinemaOptions() : StillOptions()
	{
		using namespace boost::program_options;
		// clang-format off
		options_->add_options()
			("reel", value<std::string>(&reel_)->default_value("A001"),
			 "Reel directory name. Output is <output>/<reel>/<clip>/NNNNNN.dng")
			("clip", value<std::string>(&clip_)->default_value("C001"),
			 "Clip directory name. Output is <output>/<reel>/<clip>/NNNNNN.dng")
			("queue-depth", value<unsigned int>(&queue_depth_)->default_value(8),
			 "Max buffered frames for DNG writing. When full, frames are dropped.")
			;
		// clang-format on
	}

	std::string const &Reel() const { return reel_; }
	std::string const &Clip() const { return clip_; }
	unsigned int QueueDepth() const { return queue_depth_; }

private:
	std::string reel_;
	std::string clip_;
	unsigned int queue_depth_ = 8;
};
