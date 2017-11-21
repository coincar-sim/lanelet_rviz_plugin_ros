#pragma once

#include "map_element_ogre_helper.h"

// ROS
#include <ros/ros.h>

// ROS-Geodesy
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

// Ogre
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>

// Rviz-Ogre Helper for movable Text (also includes OGRE/Fontmanager.h)
#include <rviz/ogre_helpers/movable_text.h>

// LibLanelet
#include <sim_lanelet/BoundingBox.hpp>
#include <sim_lanelet/Lanelet.hpp>
#include <sim_lanelet/LaneletMap.hpp>
#include <sim_lanelet/RegulatoryElement.hpp>
#include <sim_lanelet/lanelet_point.hpp>
#include <sim_lanelet/llet_xml.hpp>

// std
#include <tuple>
#include <typeinfo>
// Boost
#include <boost/filesystem.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/serialization/strong_typedef.hpp>

#include <assert.h>

namespace lanelet_rviz_plugin_ros {

enum ObjectClassification { UNKOWN, MAP, LANELETID, SEPERATOR, REGULATORYELEMENT, STOPLINE, SPEEDLIMIT };
// List of possible Object Classifications
static const ObjectClassification regElementClassifications[] = {
    REGULATORYELEMENT, STOPLINE, SPEEDLIMIT}; // List of Classifications that
                                              // are associated with regulatory
                                              // Elements
using ClassifiedMovableObject = std::pair<ObjectClassification, Ogre::MovableObject*>;

class MapElement {
public:
    MapElement(Ogre::SceneManager* scene_manager,
               Ogre::SceneNode* parent_node,
               std::string map_file,
               geodesy::UTMPoint fixedFrameOrigin,
               double laneletWidth = 1.0,
               double seperatorWidth = 0.5,
               double stopLineWidth = 0.5);
    virtual ~MapElement();

    void disable();
    void enable();

    void disable(ObjectClassification classification);
    void enable(ObjectClassification classification);

    static int manObjCounter_;
    static bool ogreInitialized_; // to check whether ogre settings have been applied

private:
    void loadMap(const std::string& map_file);
    void addLaneletToManualObject(const LLet::lanelet_ptr_t& lanelet, Ogre::ManualObject* manual);
    void addSeperatorToManualObject(const LLet::lanelet_ptr_t& lanelet, Ogre::ManualObject* manual);
    void attachStopLinesToSceneNode(const std::vector<LLet::member_variant_t>& stopLines, Ogre::SceneNode* parentNode);
    void attachLaneletIdToSceneNode(const LLet::lanelet_ptr_t& lanelet, Ogre::SceneNode* parentNode);
    void addRegulatoryElements(const LLet::lanelet_ptr_t& lanelet, Ogre::SceneNode* parentNode);
    Ogre::Line lineFromPts(const std::vector<LLet::point_with_id_t>& ptsVector);
    Ogre::Vector3 latLongPointToMeterPoint(const geographic_msgs::GeoPoint& geoPoint);
    Ogre::Vector3 latLongPointToMeterPoint(LLet::point_with_id_t point);

    std::vector<ClassifiedMovableObject> objects_;
    Ogre::SceneManager* sceneManager_;
    Ogre::SceneNode* sceneNode_;

    geodesy::UTMPoint fixedFrameOrigin_;

    Ogre::MaterialPtr material_;

    // line settings
    const Ogre::ColourValue colorLeft_ = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);      // red
    const Ogre::ColourValue colorRight_ = Ogre::ColourValue(0.0, 1.0, 0.0, 1.0);     // green
    const Ogre::ColourValue colorSeperator_ = Ogre::ColourValue(0.0, 0.0, 1.0, 1.0); // blue
    const Ogre::ColourValue colorStopLine_ = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);  // red
    double laneletWidth_;
    double seperatorWidth_;
    double stopLineWidth_;
};

} // namespace lanelet_rviz_plugin_ros
