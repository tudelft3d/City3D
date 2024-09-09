/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef _STOP_WATCH_H_
#define _STOP_WATCH_H_


#ifdef WIN32
#	include <windows.h>
#else 
#	include <sys/time.h>
#endif // WIN32

#include <string>

//______________________________________________________________________


/**
* 
* The purpose of this file is to make a timer function
* that is as precise as possible on any given platform.
*
* usage example:
*   {
*      StopWatch w ;
*      // do task_1 ...
*      std::cout << "task_1 done. time: " << w.elapsed() << " sec.";
*	   w.start();
*      // do task_2 ...
*      std::cout << "task_2 done. time: " << w.elapsed() << " sec.";
*   } 
*/

class StopWatch
{
public :
	StopWatch() ; // the watch will automatically start in construction
	~StopWatch() ;

	void  start() ;

	// returns user elapsed time (in seconds) since the construction / start.
	double seconds() const ;

	/// the elapsed time string, e.g., 88ms, 2.3s, 1.7m, 0.1h. This function automatically determines the best unit.
	std::string time_string(int num_digits = 1) const;

private:

#ifdef WIN32
	LONGLONG  freq_;
	LONGLONG  start_count_;
#else
	timeval start_time_;
#endif

} ;


#endif

