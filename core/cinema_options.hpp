/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * cinema_options.hpp - cinema DNG sequence capture options (folder-per-clip).
 *
 * rpicam-cinema is based on rpicam-still style options/controls, but always
 * captures RAW and writes a DNG sequence:
 *   <output>/<reel>/<clip>/000000.dng, 000001.dng, ...
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
			("reel", value<std::string>(&v_->cinema_reel)->default_value("A001"),
			 "Reel name (directory component). Output is <output>/<reel>/<clip>/NNNNNN.dng")
			("clip", value<std::string>(&v_->cinema_clip)->default_value("C001"),
			 "Clip name (directory component). Output is <output>/<reel>/<clip>/NNNNNN.dng")
			("queue-depth", value<unsigned int>(&v_->cinema_queue_depth)->default_value(8),
			 "Max number of frames to buffer for DNG writing. When full, frames are dropped.")
			;
		// clang-format on
	}

	// Extend OptsInternal with our fields by "borrowing" unused strings is not OK.
	// So we store them in the base OptsInternal via additional members by subclassing
	// CinemaOptionsInternal. However Options keeps a unique_ptr<OptsInternal>.
	//
	// Easiest in this repo style is to stash these in OptsInternal by adding members,
	// but we cannot change core/options.hpp here. Therefore we store in this object.
	//
	// So we keep our own members and expose getters.

	std::string const &Reel() const { return reel_; }
	std::string const &Clip() const { return clip_; }
	unsigned int QueueDepth() const { return queue_depth_; }

	virtual bool Parse(int argc, char *argv[]) override
	{
		// Parse all base options first.
		if (StillOptions::Parse(argc, argv) == false)
			return false;

		// Pull our options from the boost variables map indirectly:
		// In this repo, values are written directly into v_ fields. Since we can't
		// extend v_ cleanly without editing core/options.hpp, we re-parse the raw args
		// ourselves for these three options using a tiny local options_description.

		// However, StillOptions has already consumed argv and set up boost parsing,
		// but does not expose the variables_map. Therefore we implement a simple manual
		// scan for --reel/--clip/--queue-depth. This is robust enough.

		reel_ = "A001";
		clip_ = "C001";
		queue_depth_ = 8;

		for (int i = 1; i < argc; i++)
		{
			std::string a = argv[i];
			auto get_value = [&](std::string const &opt) -> std::string {
				// Accept --opt=value or --opt value
				if (a.rfind(opt + "=", 0) == 0)
					return a.substr(opt.size() + 1);
				if (a == opt && i + 1 < argc)
					return std::string(argv[++i]);
				return {};
			};

			std::string v;
			if (!(v = get_value("--reel")).empty())
				reel_ = v;
			else if (!(v = get_value("--clip")).empty())
				clip_ = v;
			else if (!(v = get_value("--queue-depth")).empty())
				queue_depth_ = std::max(1u, (unsigned int)std::stoul(v));
		}

		return true;
	}

private:
	std::string reel_ = "A001";
	std::string clip_ = "C001";
	unsigned int queue_depth_ = 8;
};
