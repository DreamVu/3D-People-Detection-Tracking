# ifndef PAL_UNIFIED_DATA_EXCHANGE_H
# define PAL_UNIFIED_DATA_EXCHANGE_H

# include <sys/time.h> 
# include <opencv2/opencv.hpp>
# include <vector>

namespace PAL
{    
    struct BoundingBox
	{
		float x1, y1, x2, y2;

		BoundingBox() : x1(0), y1(0), x2(0), y2(0)
		{

		} 

		BoundingBox(float a, float b, float c, float d) 
		    : x1(a), y1(b), x2(c), y2(d)
		{

		}
	};

    struct Point
	{
		float x, y, z;
		unsigned char r, g, b, a;

		Point() : x(0.0f), y(0.0f), z(0.0f), r(0), g(0), b(0), a(255)
		{

		}

		Point(float x1, float y1, float z1, unsigned char r1, unsigned char g1, unsigned char b1)
			: x(x1), y(y1), z(z1), r(r1), g(g1), b(b1), a(255)
		{

		}
	};

    struct Loc3D
    {
        float x, y, z;
        Loc3D() : x(0.0f), y(0.0f), z(0.0f)
        {
        } 
        Loc3D(float x1, float y1, float z1) 
            : x(x1), y(y1), z(z1)
        {
        }
    }; 

     enum class Mode
    {
        IDLE,
        STEREO,
        DEPTH,
        PEOPLE_DETECTION,
        PEOPLE_DETECTION_WITH_DEPTH,
        // POINT_CLOUD_3D,       // rename later ---> python thread using it
        POINT_CLOUD,
        REFINED_LEFT,        //will remove later
        LASER_SCAN,
        TRACKING,
        SGBM,
        FOLLOWING,
    };

    enum States
    {
        OK = 0,
        SEARCHING = 1,
        TERMINATED = 2,
    };  
    
    namespace Data
    {
        struct Common
        {
            timeval timestamp;
            int iterations;

            Common():iterations(0){}
        };

        struct Camera : Common
        {
            cv::Mat cfm;
        };

        struct Stereo : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat refined_left;
        };
        
        
        struct Depth : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat depth;
        };        
 
        
        struct FloorMask : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat floor_mask;
        };          
        
        struct DepthFloorMask : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat depth;            
            cv::Mat floor_mask;
        };

        struct TrackND
        {
            float t_is_activated;
            float t_track_id;
            float active;
            PAL::BoundingBox boxes; 

            float t_score;
            Loc3D locations_3d;
        };

        struct TrackingData : Common
	    {
		    cv::Mat left;
		    cv::Mat right;
		    std::vector<PAL::Data::TrackND> current;
            std::vector<PAL::Data::TrackND> lost;
            std::vector<PAL::Data::TrackND> removed;
	    };
	
	    struct OutputData : Common
	    {
		    cv::Mat left;
		    cv::Mat right;
		    cv::Mat depth;
            cv::Mat floor;
		    std::vector<PAL::Data::TrackND> current;
            std::vector<PAL::Data::TrackND> lost;
            std::vector<PAL::Data::TrackND> removed;
	    };

        struct TrackingResults : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat depth;
            cv::Mat floor;
            std::vector<std::vector<PAL::Data::TrackND>> trackingData;
        };
    }
}


# endif //PAL_UNIFIED_DATA_EXCHANGE_H
