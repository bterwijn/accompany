
#include <accompany_uva_msg/MsgToMarkerArray.h>
#include <boost/functional/hash.hpp>

using namespace std;
using namespace boost;

size_t reverse_bits(size_t data)
{
  size_t ret=0;
  for (int i=0;i<sizeof(size_t)*8;i++)
  {
    if (((1<<i)&data)>0)
      ret|=1<<(sizeof(size_t)*8-i-1);
  }
  return ret;
}

// get a random color wich will be associated with the name on future invocations
std_msgs::ColorRGBA getRandomColor(string name,double a)
{
  static map<string,std_msgs::ColorRGBA> nameToColor;
  map<string,std_msgs::ColorRGBA>::const_iterator it=nameToColor.find(name);
  std_msgs::ColorRGBA color;
  if (it==nameToColor.end())
  {
    hash<string> hasher;
    size_t hash=reverse_bits(hasher(name));
    color.r=((hash>> 0)%255)/255.0; // trick to generate color from name
    color.g=((hash>> 8)%255)/255.0;
    color.b=((hash>>16)%255)/255.0;
    color.a=a;
    nameToColor.insert(pair<string,std_msgs::ColorRGBA>(name,color));
  }
  else
    color=it->second;
  return color;
}

visualization_msgs::MarkerArray toMarkerArray(const accompany_uva_msg::HumanLocations& msg,
                                              string name,int id)
{
  visualization_msgs::MarkerArray markerArray;
  for (int i=0;i<msg.locations.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg.locations[0].header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg.locations[i].vector.x;
    marker.pose.position.y = msg.locations[i].vector.y;
    marker.pose.position.z = msg.locations[i].vector.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color=getRandomColor(msg.locations[0].header.frame_id,0.5);
    markerArray.markers.push_back(marker);
  }
  return markerArray;
}
