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

#include <lanelet2_interface_ros/lanelet2_interface_ros.hpp>

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
        : Display(), tfListener_{tfBuffer_},
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
          characterHeightProperty_(
              "Character Height", 1.0, "Character Height", &mapVisibilityProperty_, SLOT(reloadMap()), this),
          laneletWidthProperty_("Linewidth Boundaries",
                                0.3,
                                "Linewidth of Lanelet Boundaries",
                                &mapVisibilityProperty_,
                                SLOT(reloadMap()),
                                this),
          seperatorWidthProperty_("Linewidth Separators",
                                  0.3,
                                  "Linewidth of Lanelet Separators",
                                  &mapVisibilityProperty_,
                                  SLOT(reloadMap()),
                                  this),
          stopLineWidthProperty_(
              "Linewidth Stop Lines", 0.3, "Linewidth of Stop Lines", &mapVisibilityProperty_, SLOT(reloadMap()), this),
          laneletLeftBoundColorProperty_("Color Left Boundaries",
                                         QColor(120, 120, 120, 170),
                                         "Color of Left Boundaries",
                                         &mapVisibilityProperty_,
                                         SLOT(reloadMap()),
                                         this),
          laneletRightBoundColorProperty_("Color Right Boundaries",
                                          QColor(120, 120, 120, 170),
                                          "Color of Right Boundaries",
                                          &mapVisibilityProperty_,
                                          SLOT(reloadMap()),
                                          this),
          stopLineColorProperty_("Color Stop Lines",
                                 QColor(250, 10, 10, 255),
                                 "Color of Stop Lines",
                                 &mapVisibilityProperty_,
                                 SLOT(reloadMap()),
                                 this),
          seperatorColorProperty_("Color Separators",
                                  QColor(25, 75, 250, 180),
                                  "Color of Lanelet Separators",
                                  &mapVisibilityProperty_,
                                  SLOT(reloadMap()),
                                  this) {
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
}

void LaneletMapPlugin::onEnable() {
    reloadMap();
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
    reloadMap();
}

void LaneletMapPlugin::loadMap() {
    if (!isEnabled()) {
        // do not load map if plugin is not enabled
        return;
    }
    createMapObject();
}

/*
 * Load Map and and attach visible Map-Object to the scene_node.
 */
void LaneletMapPlugin::createMapObject() {

    if (!theMapPtr_) {
        // try to load map
        try {
            ROS_INFO("LaneletMapPlugin: retrieving map from lanelet2_interface_ros...");
            lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
            theMapPtr_ = ll2if.waitForMapPtr(10, 0.5);
            originFrameId_ = ll2if.waitForFrameIdMap(10, 0.1);
        } catch (std::exception& e) {
            setStatus(rviz::StatusProperty::Error,
                      QString("Map"),
                      QString("Error during map loading with lanelet2_interface_ros: ") + e.what());
            return;
        }
    }

    // try to create map element
    try {
        Ogre::SceneNode* scene_node_origin_frame;
        scene_node_origin_frame = getChildSceneNodeAtFrameId(tfBuffer_, context_, scene_node_, originFrameId_);
        // create Map Element. It is attached to the scene_node on creation
        VisualizationOptions visOptions{static_cast<double>(characterHeightProperty_.getFloat()),
                                        static_cast<double>(laneletWidthProperty_.getFloat()),
                                        static_cast<double>(seperatorWidthProperty_.getFloat()),
                                        static_cast<double>(stopLineWidthProperty_.getFloat()),
                                        laneletLeftBoundColorProperty_.getOgreColor(),
                                        laneletRightBoundColorProperty_.getOgreColor(),
                                        stopLineColorProperty_.getOgreColor(),
                                        seperatorColorProperty_.getOgreColor()};
        mapElement_ = std::make_unique<MapElement>(scene_manager_, scene_node_origin_frame, theMapPtr_, visOptions);

        Ogre::Vector3 origin = scene_node_origin_frame->convertLocalToWorldPosition(Ogre::Vector3{0., 0., 0.});
        ROS_DEBUG("RVIZ:lanelet_plugin: Map loaded. Origin frame (\"%s\")is at x=%f, y=%f in the fixed frame",
                  originFrameId_.c_str(),
                  origin.x,
                  origin.y);

    } catch (std::exception& e) {
        setStatus(
            rviz::StatusProperty::Error, QString("Map"), QString("Error during map element creation: ") + e.what());
        return;
    }
    // Set Visibility for individual Map Elements
    visibilityPropertyChanged();

    setStatus(rviz::StatusProperty::Ok, QString("Map"), "Map loaded successfully");

    return;
}

void LaneletMapPlugin::referenceFrameChanged() {
    reloadMap();
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

void LaneletMapPlugin::reloadMap() {
    clear();
    loadMap();
}

} // namespace lanelet_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lanelet_rviz_plugin_ros::LaneletMapPlugin, rviz::Display)
