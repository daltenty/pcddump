/*
 * Software License Agreement (BSD License)
 *
 *  Fakenect to PCL frame dumper
 *
 *  Copyright (c) 2015, David Tenty
 *
 *  Portions originally from:
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <boost/filesystem.hpp>
#include <pcl/io/image_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace boost::filesystem;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;

// Helper function for grabbing a cloud
void
cloud_callback (bool *signal_received,
                CloudT::ConstPtr *ptr_to_fill,
                const CloudT::ConstPtr &input_cloud)
{
    *signal_received = true;
    *ptr_to_fill = input_cloud;
}

void usage(const char *progname) {
    printf("Usage: %s [path_to_fakenect_dump]\n", progname);
    cout << endl << "Note: this program non-destructively modifies the target dir" << endl;
}

int main (int argc,char *argv[])
{
    if (argc !=2 ){
        usage(argv[0]);
        return 1;
    }

    // prepare the directory for grabber
    path p (argv[1]);

    if (!exists(p)) {
        cerr << "Dump directory does not exist" << endl;
        return 3;
    }

//    boost::filesystem::directory_iterator end_itr;
//    for (boost::filesystem::directory_iterator itr (p); itr != end_itr; ++itr) {
//#if BOOST_FILESYSTEM_VERSION == 3
//        extension = boost::algorithm::to_upper_copy(boost::filesystem::extension(itr->path()));
//        pathname = itr->path().string();
//        basename = boost::filesystem::basename(itr->path());
//#else
//    extension = boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->leaf ()));
//    pathname = itr->path ().filename ();
//    basename = boost::filesystem::basename (itr->leaf ());
//#endif
//    }

    // Get all clouds from the grabber
    pcl::ImageGrabber<PointT> grabber (argv[1], 0, false, false);
    vector<CloudT::ConstPtr> fakenect_clouds;
    CloudT::ConstPtr cloud_buffer;
    bool signal_received = false;
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
            fxn = boost::bind (cloud_callback, &signal_received, &cloud_buffer, _1);
    grabber.registerCallback (fxn);
    //grabber.setCameraIntrinsics (525., 525., 320., 240.); // Setting old intrinsics which were used to generate these tests
    grabber.setDepthImageUnits(-0.001);
    grabber.start ();
    for (size_t i = 0; i < grabber.size (); i++)
    {
        grabber.trigger ();
        size_t niter = 0;
        while (!signal_received)
        {
            boost::this_thread::sleep (boost::posix_time::microseconds (10000));
            if (++niter > 100)
            {
                cerr << "Stopped receiving frames from grabber" << endl;
                return 2;
            }
        }
        fakenect_clouds.push_back (cloud_buffer);
        signal_received = false;
    }
    clog << "Got " << grabber.size() << " frames from grabber" << endl;


    // write the clouds
    // FIXME: check float to ASCII conversion for imprecision
    for (vector<CloudT::ConstPtr>::iterator itr=fakenect_clouds.begin(); itr != fakenect_clouds.end(); itr++) {
            string timestamp=to_string((*itr)->header.stamp);
            pcl::io::savePCDFileASCII(string("frame_")+timestamp+string(".pcd"),**itr);
    }

    return 0;
}
