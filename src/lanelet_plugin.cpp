/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lanelet_plugin.hpp"

#include <sensor_msgs/NavSatFix.h>

namespace {

Ogre::SceneNode* getChildSceneNodeAtFrameId(const tf2_ros::Buffer& tf_buffer,
                                            rviz::DisplayContext* context,
                                            Ogre::SceneNode* scene_node,
                                            std::string frame_id) {
    /**
     * Get transform
     */
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
        context->getFrameManager()->getFixedFrame(), frame_id, ros::Time(0), ros::Duration(10.0));

    orientation.x = transform.transform.rotation.x;
    orientation.y = transform.transform.rotation.y;
    orientation.z = transform.transform.rotation.z;
    orientation.w = transform.transform.rotation.w;

    position.x = transform.transform.translation.x;
    position.y = transform.transform.translation.y;
    position.z = transform.transform.translation.z;

    Ogre::SceneNode* child_scene_node = scene_node->createChildSceneNode(position, orientation);

    return child_scene_node;
}


} // end of anonymous namespace


namespace lanelet_rviz_plugin_ros {

LaneletMapPlugin::LaneletMapPlugin()
        : Display(), tfListener_{tfBuffer_}, mapFileName_(),
          mapFileProperty_("Map-File",
                           "",
                           "Path and filename of the Map (*.osm-File). Ros "
                           "parameters are allowed (${myparam}).",
                           this,
                           SLOT(mapFileChanged()),
                           this),
          navSatFixTopicProperty_("NavSatFix Topic",
                                  "",
                                  "",
                                  "Describes the frame_id with origin (x=0,y=0,z=0) at the given lat/lon coordinates",
                                  this,
                                  SLOT(referenceFrameChanged())),
          mapVisibilityProperty_("Visibility of Map", true, "", this, SLOT(visibilityPropertyChanged()), this),
          idVisibilityProperty_(
              "Visibility of LaneletIds", true, "", &mapVisibilityProperty_, SLOT(visibilityPropertyChanged()), this),
          seperatorVisibilityProperty_("Visibility of Lanelet Seperators",
                                       true,
                                       "",
                                       &mapVisibilityProperty_,
                                       SLOT(visibilityPropertyChanged()),
                                       this),
          regElementVisibilityProperty_("Visibility of Regulatory Elements (e.g. Stop Lines)",
                                        true,
                                        "",
                                        &mapVisibilityProperty_,
                                        SLOT(visibilityPropertyChanged()),
                                        this),
          laneletWidthProperty_("Linewidth",
                                0.3,
                                "Linewidth (green and red lines)",
                                &mapVisibilityProperty_,
                                SLOT(lineWidthChanged()),
                                this),
          seperatorWidthProperty_("Linewidth Separators",
                                  0.3,
                                  "Linewidth of Lanelet Separators (blue lines)",
                                  &mapVisibilityProperty_,
                                  SLOT(lineWidthChanged()),
                                  this),
          stopLineWidthProperty_("Linewidth Stop Lines",
                                 0.3,
                                 "Linewidth of Stop Lines (red lines)",
                                 &mapVisibilityProperty_,
                                 SLOT(lineWidthChanged()),
                                 this) {

    QString message_type = QString::fromStdString(ros::message_traits::datatype<sensor_msgs::NavSatFix>());
    navSatFixTopicProperty_.setMessageType(message_type);
    navSatFixTopicProperty_.setDescription(message_type + " topic to subscribe to.");
}

LaneletMapPlugin::~LaneletMapPlugin() {
}

void LaneletMapPlugin::reset() {
    Display::reset();
    clear();
}

void LaneletMapPlugin::onInitialize() {
    // call base class initialize function
    Display::onInitialize();
    checkEnvVariables();
}

void LaneletMapPlugin::onEnable() {
    clear();
    // Update Map Filename
    resolveMapFile();
    // Load Map
    loadMap();
}

void LaneletMapPlugin::onDisable() {
    clear();
}

void LaneletMapPlugin::clear() {
    mapElement_.reset();
    clearStatuses();
}

void LaneletMapPlugin::fixedFrameChanged() {
    // Call Base Class method
    Display::fixedFrameChanged();
    clear();
    loadMap();
}

void LaneletMapPlugin::loadMap() {
    if (!isEnabled()) {
        // do not load map if plugin is not enabled
        return;
    }
    if (checkEnvVariables() && checkMapFile()) {
        // (re)load Map
        createMapObject();
    }
}

/*
 * Load Map and and attach visible Map-Object to the scene_node.
 */
void LaneletMapPlugin::createMapObject() {

    if (!coordinateTransformPtr_) {
        setStatus(rviz::StatusProperty::Error,
                  "Transform",
                  "Coordinate transformation not available, cannot create map object!");
        return;
    } else {
        setStatus(rviz::StatusProperty::Ok,
                  "Transform",
                  QString("Transform ok, originFrameId_=") + QString::fromStdString(originFrameId_));
    }

    // try to load map and create map object
    try {
        Ogre::SceneNode* scene_node_origin_frame;
        scene_node_origin_frame = getChildSceneNodeAtFrameId(tfBuffer_, context_, scene_node_, originFrameId_);


        // create Map Element. It is attached to the scene_node on creation
        mapElement_ = std::make_unique<MapElement>(scene_manager_,
                                                   scene_node_origin_frame,
                                                   mapFileName_,
                                                   coordinateTransformPtr_,
                                                   latLonOrigin_,
                                                   static_cast<double>(laneletWidthProperty_.getFloat()),
                                                   static_cast<double>(seperatorWidthProperty_.getFloat()),
                                                   static_cast<double>(stopLineWidthProperty_.getFloat()));

        Ogre::Vector3 origin = scene_node_origin_frame->convertLocalToWorldPosition(Ogre::Vector3{0., 0., 0.});
        ROS_DEBUG("RVIZ:lanelet_plugin: Map loaded. Origin frame (\"%s\")is at x=%f, y=%f in the fixed frame",
                  originFrameId_.c_str(),
                  origin.x,
                  origin.y);

    } catch (std::exception& e) {
        setStatus(rviz::StatusProperty::Error, QString("Map"), QString("Error during map creation: ") + e.what());
        return;
    }
    // Set Visibility for individual Map Elements
    visibilityPropertyChanged();

    setStatus(rviz::StatusProperty::Ok, QString("Map"), "Map loaded successfully");


    return;
}

void LaneletMapPlugin::mapFileChanged() {
    clear();
    resolveMapFile();
    loadMap();
}

void LaneletMapPlugin::resolveMapFile() {
    mapFileName_ = mapFileProperty_.getStdString();
    // resolve parameters
    try {
        resolveParameters(mapFileName_);
    } catch (std::exception& e) {
        setStatus(rviz::StatusProperty::Error,
                  QString("Map"),
                  QString("Error resolving map filename (or parts) from parameter server."));
        return;
    }
}

void LaneletMapPlugin::resolveNavSatFixTopic() {
    navSatFixTopic_ = navSatFixTopicProperty_.getTopicStd();
}

void LaneletMapPlugin::createGeoCoordinateTransform() {
    setStatus(rviz::StatusProperty::Warn, "Transform", "No NavSatFix received yet!");
    coordinateTransformPtr_ = std::make_shared<util_geo_coordinates::CoordinateTransformRos>();
    coordinateTransformPtr_->waitForInit(navSatFixTopic_, 10);
    originFrameId_ = coordinateTransformPtr_->getOriginFrameId();
    std::tie(latLonOrigin_.lat, latLonOrigin_.lon) = coordinateTransformPtr_->getOriginLatLon();
    setStatus(rviz::StatusProperty::Ok, "Transform", "Transform received.");
}

void LaneletMapPlugin::referenceFrameChanged() {
    clear();
    resolveNavSatFixTopic();
    createGeoCoordinateTransform();
    loadMap();
}

void LaneletMapPlugin::visibilityPropertyChanged() {
    if (mapElement_) {
        if (mapVisibilityProperty_.getBool()) {
            mapElement_->enable();
            mapVisibilityProperty_.expand();
            // check if sub-elements are disabled
            if (!idVisibilityProperty_.getBool()) {
                mapElement_->disable(ObjectClassification::LANELETID);
            }
            if (!seperatorVisibilityProperty_.getBool()) {
                mapElement_->disable(ObjectClassification::SEPERATOR);
            }
            if (!regElementVisibilityProperty_.getBool()) {
                mapElement_->disable(ObjectClassification::REGULATORYELEMENT);
            }
        } else {
            mapElement_->disable();
            mapVisibilityProperty_.collapse();
        }
    }
}

void LaneletMapPlugin::lineWidthChanged() {
    loadMap();
}

/*
 * Helper function to resolve ROS-Parameters
 */
void LaneletMapPlugin::resolveParameters(std::string& str) {
    while (str.find("${") != std::string::npos) {
        auto pos = str.find("${");
        if (pos != std::string::npos) {
            auto endpos = str.find("}");
            auto paramName = str.substr(pos + 2, endpos - pos - 2);
            std::string value;
            if (!update_nh_.getParam(paramName, value)) {
                throw std::runtime_error(std::string("Could not get Parameter from Parameter Server: ") +
                                         paramName.c_str());
            }
            str.erase(pos, endpos - pos + 1);
            str.insert(pos, value);
        }
    }
}


/*
 * Helper function to check Filename and -Path
 */
bool LaneletMapPlugin::checkMapFile() {
    boost::filesystem::path map_path(mapFileName_);
    if (!boost::filesystem::exists(map_path) || map_path.extension() != ".osm") {
        setStatus(rviz::StatusProperty::Error,
                  "Map",
                  "Failed to find map under specified path(" + QString::fromStdString(mapFileName_) + ")");
        return false;
    }
    return true;
}

bool LaneletMapPlugin::checkEnvVariables() {
    // TODO: check for other compatible (decimal seperator is point) locales.

    std::string pLC_NUMERIC = std::string(getenv("LC_NUMERIC"));

    if (pLC_NUMERIC == "en_US.UTF-8") {
        deleteStatus(QString("LC_NUMERIC"));
        return true;
    } else {
        std::string errMessage =
            std::string("Environment Variable LC_NUMERIC is not set to set to \"en_US.UTF-8\". LC_NUMERIC=") +
            pLC_NUMERIC +
            std::string(" This may cause Errors when loading the map. Make sure you use an Environment "
                        "Variable that matches your used decimal separator. "
                        "For example by adding <env name=\"LC_NUMERIC\" value=\"en_US.UTF-8\"/> to your launchfile.");

        ROS_ERROR_THROTTLE(2, "%s", errMessage.c_str());
        setStatus(rviz::StatusProperty::Error, QString("LC_NUMERIC"), QString(QString::fromStdString(errMessage)));

        return false;
    }
}

} // namespace lanelet_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lanelet_rviz_plugin_ros::LaneletMapPlugin, rviz::Display)
