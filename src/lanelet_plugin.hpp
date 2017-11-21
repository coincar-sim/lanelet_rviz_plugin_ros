#pragma once

// ROS
#include <ros/ros.h>

// ROS-Geodesy
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

// TF2
#include <tf2_ros/transform_listener.h>

// RVIZ
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>

// Ogre
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/exception/all.hpp>

// Map Element
#include "map_element.hpp"

namespace geodesy {
struct utmOffset {
    double altitude; // in meters
    double easting;  // in meters
    double northing; // in meters;
};
void fromMsgWithOffset(const geographic_msgs::GeoPoint& from,
                       const geodesy::utmOffset& offset,
                       geodesy::UTMPoint& to,
                       bool normalize = true);
} // end of namespace geodesy

namespace lanelet_rviz_plugin_ros {

class LaneletMapPlugin : public rviz::Display {
    Q_OBJECT
public:
    LaneletMapPlugin();
    virtual ~LaneletMapPlugin();

    // Overrides from Display
    virtual void reset();

protected:
    // Overrides from Display
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void fixedFrameChanged();

private Q_SLOTS:
    void mapFileChanged();
    void visibilityPropertyChanged();
    void referenceFrameChanged();
    void lineWidthChanged();

private:
    void clear();
    void loadMap();
    void createMapObject();
    bool checkMapFile();
    void resolveParameters(std::string& str);
    void resolveMapFile();
    double resolveDoubleParameters(std::string str);
    bool updateFrames();
    bool checkEnvVariables();

    // tf2-listener
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    // map
    std::string mapFileName_;
    std::string referenceFrame_;
    geographic_msgs::GeoPoint referenceFrameOrigin_;
    geodesy::UTMPoint fixedFrameOrigin_;
    std::unique_ptr<MapElement> mapElement_;

    // RVIZ-properties
    rviz::StringProperty mapFileProperty_;
    rviz::StringProperty referenceFrameProperty_;
    rviz::StringProperty referenceFrameLatProperty_;
    rviz::StringProperty referenceFrameLonProperty_;
    rviz::BoolProperty mapVisibilityProperty_;
    rviz::BoolProperty idVisibilityProperty_;
    rviz::BoolProperty seperatorVisibilityProperty_;
    rviz::BoolProperty regElementVisibilityProperty_;
    rviz::FloatProperty laneletWidthProperty_;
    rviz::FloatProperty seperatorWidthProperty_;
    rviz::FloatProperty stopLineWidthProperty_;
};
} // namespace lanelet_rviz_plugin_ros
