/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<librealsense2/rs.hpp>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;
rs2::pipeline rs_pipe;

#define RS 1
void configRS()
{
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_INFRARED,640,480,RS2_FORMAT_Y8,30);
    rs_cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    rs_pipe.start(rs_cfg);
}
void frame_to_mat(const rs2::frame &f, cv::Mat &img)
{
  auto vf = f.as<rs2::video_frame>();
cout<<endl<<"sunhuchang pos0.1:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
  const int w = vf.get_width();
cout<<endl<<"sunhuchang pos0.2:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
  const int h = vf.get_height();
cout<<endl<<"sunhuchang pos0.3,w="<<w<<",h="<<h<<","<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
  const int size = w * h;
  /*
  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    memcpy(img.ptr<cv::Vec3b>(), f.get_data(), size * 3);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    cv::Mat tmp(h, w, CV_8UC3, (void *)f.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(tmp, img, cv::COLOR_RGB2BGR);
  } else 
  */
cout<<endl<<"sunhuchang pos0:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
  if (f.get_profile().format() == RS2_FORMAT_Y8) {
cout<<endl<<"sunhuchang pos1:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
    //memcpy(img.ptr<uchar>(), f.get_data(), size);
  }
  else if (f.get_profile().format() == RS2_FORMAT_Z16) {
cout<<endl<<"sunhuchang pos2:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
    //memcpy(img.ptr<uchar>(), f.get_data(), (size<<1));
  }

cout<<endl<<"sunhuchang pos0:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
}
void getImagesFromRS(cv::Mat &imRGB, cv::Mat &imD)
{
cout<<endl<<"sunhuchang pos0:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
  rs2::frameset frames=rs_pipe.wait_for_frames();
cout<<endl<<"sunhuchang pos0.1:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
  

    rs2::frame infrared_frame = frames.get_infrared_frame();
cout<<endl<<"sunhuchang pos0.2:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
    
    rs2::frame depth_frame = frames.get_depth_frame();
cout<<endl<<"sunhuchang pos1:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
    imRGB=cv::Mat(cv::Size(640,480),CV_8UC1,(void *)infrared_frame.get_data());
    imD=cv::Mat(cv::Size(640,480),CV_16SC1,(void *)depth_frame.get_data());
    //frame_to_mat(infrared_frame,imRGB);
cout<<endl<<"sunhuchang pos2:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
    //frame_to_mat(depth_frame,imD);
//cout<<endl<<"sunhuchang pos3:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
 /*
  else
  {
    cout<<endl<<"sunhuchang Error:"<<__FILE__<<","<<__LINE__<<","<<__FUNCTION__<<"()";
    cout<<"poll_for_frames false";
    exit(0);
  }*/
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
#ifndef RS
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
#endif
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
#ifndef RS
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
#endif
    // Main loop
    configRS();
    cout << "sunhuchang configRS done" << endl;
    cv::Mat imRGB, imD;
    for(int ni=0; ni<10000; ni++)
    {
        // Read image and depthmap from file
#ifndef RS
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
#else
        getImagesFromRS(imRGB,imD);
        cout << "sunhuchang getImagesFromRS frame "<<ni<< " done" << endl;
#endif
#ifndef RS     
   double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }
#else
double tframe = 0;
#endif
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
#ifndef RS
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
#endif
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    /*sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
*/
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
