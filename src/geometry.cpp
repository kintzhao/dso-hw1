#include "geometry.hpp"
#include <fstream>
#include <sstream>

hw::GeometryUndistorter::GeometryUndistorter(std::string &path)
{
    w_rect_ = 0;
    h_rect_ = 0;

    std::string data_path = path;

    if(path.find_last_of("/") != path.size()-1)
        data_path = path + "/";

    std::string cam_file = data_path + "camera.txt";

    std::ifstream file(cam_file.c_str());
    assert(file.good());
    std::cout<< "[INFO]: Reading Geometry Calibration from " << cam_file <<std::endl;

    std::string param, size, setting, size_rect;

    std::getline(file, param);
    std::getline(file, size);
    std::getline(file, setting);
    std::getline(file, size_rect);

    std::stringstream ss;
    // line 1
    ss << param;
    double fx, fy, cx, cy, omega;
    ss >> fx >> fy >> cx >> cy >> k1_ >> k2_ >> k3_ >> k4_;
    // line 2
    ss.clear();
    ss << size;
    ss >> w_ >> h_;
    // line 4
    ss.clear();
    ss << size_rect;
    ss >> w_rect_ >> h_rect_;

    if(setting == "crop")
        crop = true;
    else
        crop = false;

    K_.setIdentity();
    if(cx < 1 && cy < 1)
    {
        K_(0,0) = fx*w_;
        K_(0,2) = cx*w_;
        K_(1,1) = fy*h_;
        K_(1,2) = cy*h_;
    }


    remap_.resize(w_rect_*h_rect_);
    makeOptimalK_crop();
    initRectifyMap();
}

// 计算矫正畸变后新的内参
void hw::GeometryUndistorter::makeOptimalK_crop()
{
    if(!crop)
        return;

    K_rect_.setIdentity();

    // TODO: read DSO source code, and find out way to calculate new K

    // K_rect_(0,0) = ;
	// K_rect_(1,1) = ;
	// K_rect_(0,2) = ;
	// K_rect_(1,2) = ;
}

// FoV model
// void hw::GeometryUndistorter::distortCoordinates(std::vector<Eigen::Vector2f> &in)
// {
//     float dist = omega_;
// 	float d2t = 2.0f * tan(dist / 2.0f);
//     int n = in.size();

//     float fx = K_(0, 0);
//     float fy = K_(1, 1);
//     float cx = K_(0, 2);
//     float cy = K_(1, 2);

//     float ofx = K_rect_(0, 0);
//     float ofy = K_rect_(1, 1);
//     float ocx = K_rect_(0, 2);
//     float ocy = K_rect_(1, 2);

//     for(int i=0; i<n; ++i)
//     {
//         float x = in[i][0];
//         float y = in[i][1];
//         float ix = (x - ocx) / ofx;
//         float iy = (y - ocy) / ofy;

//         float r = sqrtf(ix*ix + iy*iy);
// 		float fac = (r==0 || dist==0) ? 1 : atanf(r * d2t)/(dist*r);

// 		ix = fx*fac*ix+cx;
// 		iy = fy*fac*iy+cy;

// 		in[i][0] = ix;
// 		in[i][1] = iy;
//     }
// }

// Equidistant model
void hw::GeometryUndistorter::distortCoordinates(std::vector<Eigen::Vector2f> &in)
{
    float fx = K_(0, 0);
    float fy = K_(1, 1);
    float cx = K_(0, 2);
    float cy = K_(1, 2);
    float k1 = k1_;    
    float k2 = k2_;
    float k3 = k3_;
    float k4 = k4_;

    float ofx = K_rect_(0, 0);
    float ofy = K_rect_(1, 1);
    float ocx = K_rect_(0, 2);
    float ocy = K_rect_(1, 2);

    int n = in.size();
    
    for(int i=0;i<n; i++)
    {
        float x = in[i][0];
        float y = in[i][1];

        // TODO according Equidistant model, to calculate distorted coordinates

		// in[i][0] =;
		// in[i][1] =;
    }

}


void hw::GeometryUndistorter::initRectifyMap()
{
    for(int y = 0; y < h_rect_; ++y)
        for(int x = 0; x < w_rect_; ++x)
        {
            remap_[x+y*w_rect_][0] = x;
            remap_[x+y*w_rect_][1] = y;
        }
    
    distortCoordinates(remap_);

    for(int y = 0; y < h_rect_; ++y)
        for(int x = 0; x < w_rect_; ++x)
        {
            // make rounding resistant.
			float ix = remap_[x+y*w_rect_][0];
			float iy = remap_[x+y*w_rect_][1];


			if(ix > 0 && iy > 0 && ix < w_-1 &&  iy < h_-1)
			{
				remap_[x+y*w_rect_][0] = ix;
				remap_[x+y*w_rect_][1] = iy;
			}
			else
			{
				remap_[x+y*w_rect_][0] = -1;
				remap_[x+y*w_rect_][1] = -1;
			}
        }

}


void hw::GeometryUndistorter::processFrame(cv::Mat &src, cv::Mat &dst)
{
    assert(src.type() == CV_8UC1);

    if(src.cols != w_ && src.rows != h_)
    {
        std::cout << "[ERROR]: The solution of input isn't illegal. We need "
                  << w_ << " * " << h_ << "! "<< std::endl;
    }

    dst = cv::Mat(h_rect_, w_rect_, CV_8UC1);

    int cols = w_rect_;
    int rows = h_rect_;

    if(dst.isContinuous())
    {
        cols = cols * rows;
        rows = 1;
    }

    uchar* ptr_src = 0;
    uchar* ptr_dst = 0;
    for(int i = 0; i < rows; ++i)
    {
        ptr_dst = dst.ptr<uchar>(i);
        for(int j = 0; j < cols; ++j)
        {
            int id = i * cols + j;

            float x_f = remap_[id][0];
            float y_f = remap_[id][1];

            if(x_f < 0)
                ptr_dst[j] = 0;
            else
            {
                int x_i = std::floor(x_f);
                int y_i = std::floor(y_f);
                x_f -= x_i;
                y_f -= y_i;
                float xy_f = x_f*y_f;

                ptr_src = src.ptr<uchar>(y_i) + x_i;

                ptr_dst[j] = xy_f * ptr_src[1+w_]
                                + (y_f - xy_f) * ptr_src[w_]
                                + (x_f - xy_f) * ptr_src[1]
                                + (1-x_f-y_f+xy_f) * ptr_src[0];

            }
            
        }
    }    
        
}
