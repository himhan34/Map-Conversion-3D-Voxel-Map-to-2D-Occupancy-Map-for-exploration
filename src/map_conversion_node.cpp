#include <cstdio>
#include <nav_msgs/Odometry.h>
#include <ufo/map/occupancy_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <mapconversion/HeightMap.h>
// To convert between UFO and ROS
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include "MapConverter.hh"

#include "SaveToCSV.hh"

using namespace std;

class MapToMap{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subUfoMap;
        ros::Subscriber subOdom;

        //pub
        ros::Publisher pubMapUGV, pubMapUAV, pubMapFloor, pubMapCeiling ,pubHeightMap, pubMapSlope;
        tf::TransformListener listener;
        //msg 
        nav_msgs::OccupancyGrid mapMsg;
        mapconversion::HeightMap heightMsg;

        MapConverter* MC;
        ufo::map::OccupancyMap* ufoMap;
        
        double resolution;
        double slopeMax;
        double minimumZ;
        int minimumOccupancy;
        string mapFrame;
        double mapZpos;
	public:

	MapToMap(){
        ros::NodeHandle nh_priv("~");
        resolution = nh_priv.param("resolution", 0.0);
        if(resolution<=0.0){
            ROS_FATAL("The resolution is not set, node will terminate. \nAdd \'_resolution:=x\' to rosrun command, or \'<param name=\"resolution\" value=\"x\"/>\' in launch file.");
            ros::shutdown();
        }
        slopeMax = nh_priv.param("max_slope_ugv", INFINITY);
        minimumZ = nh_priv.param("minimum_z", 1.0);
        minimumOccupancy = nh_priv.param("minimum_occupancy", 10);
        mapFrame = nh_priv.param("map_frame", string("map"));
        mapZpos = nh_priv.param("map_position_z", 0.0);

        subUfoMap = nh.subscribe("/ufomap", 10, &MapToMap::mapCallback,this);
        
        pubMapUGV=nh.advertise<nav_msgs::OccupancyGrid>("/mapUGV",5);
        pubMapUAV=nh.advertise<nav_msgs::OccupancyGrid>("/mapUAV",5);
        pubMapFloor=nh.advertise<nav_msgs::OccupancyGrid>("/visualization_floor_map",5);
        pubMapCeiling=nh.advertise<nav_msgs::OccupancyGrid>("/visualization_ceiling_map",5);
        pubHeightMap=nh.advertise<mapconversion::HeightMap>("/heightMap",5);
        pubMapSlope=nh.advertise<nav_msgs::OccupancyGrid>("/visualization_slope_map",5);

        MC=new MapConverter(resolution,minimumZ,minimumOccupancy);
        ufoMap=new ufo::map::OccupancyMap(resolution);
	}

    ~MapToMap(){
        
    }

    void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg){
        //Convert ROS message to UFOMap
        ufomap_msgs::msgToUfo(msg->map, *ufoMap);
        
        //get size of updated regon
        ufo::map::OccupancyMap newMap(resolution);
        ufomap_msgs::msgToUfo(msg->map, newMap);
        ufo::geometry::AABB newAABB(newMap.getKnownBBX());
        ufo::math::Vector3 min_point(newAABB.getMin());
        ufo::math::Vector3 max_point(newAABB.getMax());
        //get height of complete map
        min_point.z()=(ufoMap->getKnownBBX().getMin().z());
        max_point.z()=(ufoMap->getKnownBBX().getMax().z());
        //create bounding box, x,y size is from new map, z is from complete map
        ufo::geometry::AABB aabb(min_point, max_point);

        vector<voxel> voxelList;
        //get all free and occupide voxels in boundign box 
        for (auto it = ufoMap->beginLeaves(aabb, true, true, false, false, 0),it_end = ufoMap->endLeaves(); it != it_end; ++it) {
            voxel v;
            v.position.x=it.getX();
            v.position.y=it.getY();
            v.position.z=it.getZ();
            v.halfSize=it.getHalfSize();
            v.occupied=it.isOccupied();
            voxelList.push_back(v);
        }
        MC->updateMap(voxelList);
        pub();
    }

    void pub(){
        mapMsg.header.stamp = ros::Time::now();
        mapMsg.header.frame_id=mapFrame;
        heightMsg.header=mapMsg.header;
        mapMsg.info.width = MC->map.sizeX();
        mapMsg.info.height = MC->map.sizeY();
        mapMsg.info.resolution = MC->map.getResulution();
        mapMsg.info.origin.position.x = MC->map.offsetX();
        mapMsg.info.origin.position.y = MC->map.offsetY();
        mapMsg.info.origin.position.z = mapZpos;
        heightMsg.info=mapMsg.info;
        mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height);
        heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height);
        heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height);

        //pub map for UGV
        if(pubMapUGV.getNumSubscribers()!=0){
            if(isinf(slopeMax)){
                ROS_WARN("The max slope UGV is not set; obstacles will not be included in the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or \'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
            }

            mapMsg.header.stamp = ros::Time::now();
            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    int index=x+y*MC->map.sizeX();  
            
                    mapMsg.data[index]=MC->map.get(x,y,slopeMax);

                }
            }
            pubMapUGV.publish(mapMsg);
        }

        //pub map for UAV
        if(pubMapUAV.getNumSubscribers()!=0){
            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    int index=x+y*MC->map.sizeX();  
            
                    mapMsg.data[index]=MC->map.get(x,y);

                }
            }
            pubMapUAV.publish(mapMsg);
        }

        //pub rviz visualization fro heigth map
        if(pubMapFloor.getNumSubscribers()!=0){
            if(MC->map.sizeY()==0||MC->map.sizeX()==0) return;
            double vMax=NAN,vMin=NAN;
            //find max min for normalization 
            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    double v=MC->map.getHeight(x,y);
                    if(isnan(v)) continue;
                    if(isnan(vMax)) vMax=v;
                    else vMax=max(vMax,v);
                    if(isnan(vMin)) vMin=v;
                    else vMin=min(vMin,v);
                }
            }

            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    int index=x+y*MC->map.sizeX();  
                    double value=MC->map.getHeight(x,y);
                    if(!isnan(value)){
                        value=(value-vMin)/(vMax-vMin)*199;
                        if(value>99)value-=199; // To get the full color range in the rviz cost map 
                    }
                    mapMsg.data[index]=value;
                    
                }

            }

            pubMapFloor.publish(mapMsg);
        }

        if(pubMapCeiling.getNumSubscribers()!=0){
            if(MC->map.sizeY()==0||MC->map.sizeX()==0) return;
            double vMax=NAN,vMin=NAN;
            //find max min for normalization 
            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    double v=MC->map.getHeightTop(x,y);
                    if(isnan(v)) continue;
                    if(isnan(vMax)) vMax=v;
                    else vMax=max(vMax,v);
                    if(isnan(vMin)) vMin=v;
                    else vMin=min(vMin,v);
                }
            }

            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    int index=x+y*MC->map.sizeX();  
                    double value=MC->map.getHeightTop(x,y);
                    if(!isnan(value)){
                        value=(value-vMin)/(vMax-vMin)*199;
                        if(value>99)value-=199; // To get the full color range in the rviz cost map 
                    }
                    mapMsg.data[index]=value;
                    
                }

            }

            pubMapCeiling.publish(mapMsg);
        }

        //pub height map 
        if(pubHeightMap.getNumSubscribers()!=0){
            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    int index=x+y*MC->map.sizeX();  
            
                    heightMsg.bottom[index]=MC->map.getHeight(x,y);
                    heightMsg.top[index]=MC->map.getHeightTop(x,y);
                }
            }
            pubHeightMap.publish(heightMsg);
        }

        //pub rviz visualization fro slope map
        if(pubMapSlope.getNumSubscribers()!=0){
            double vMax=slopeMax*2, vMin=0;

            for(int y=0; y<MC->map.sizeY();y++){
                for(int x=0;x<MC->map.sizeX();x++){ 
                    int index=x+y*MC->map.sizeX();  
                    double value=min(MC->map.getSlope(x,y),slopeMax*2);
                    if(!isnan(value)){
                        value=(value-vMin)/(vMax-vMin)*198+1;
                        if(value>99)value-=199; // soooo basically, rviz costmap are weird...
                    }
                    mapMsg.data[index]=value;

                }
            }
            pubMapSlope.publish(mapMsg);
        }
    }

	void spin(){
        ros::Rate rate(10);
        while(ros::ok()){

            rate.sleep();
        }
    }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_saver");
	

	MapToMap mtm;

	ros::spin();

	return 0;
}


