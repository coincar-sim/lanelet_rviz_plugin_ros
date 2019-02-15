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

#include "map_element.hpp"

#include <util_rviz/util_rviz.hpp>

namespace lanelet_rviz_plugin_ros {


MapElement::MapElement(Ogre::SceneManager* scene_manager,
                       Ogre::SceneNode* parent_node,
                       lanelet::LaneletMapConstPtr theMap,
                       double laneletWidth,
                       double seperatorWidth,
                       double stopLineWidth)
        : sceneManager_(scene_manager), sceneNode_(parent_node->createChildSceneNode()), laneletWidth_(laneletWidth),
          seperatorWidth_(seperatorWidth), stopLineWidth_(stopLineWidth) {
    if (!ogreInitialized_) {
        // create Material
        material_ = ogre_helper::getMaterial("lanelet_material");
        // material settings
        material_->getTechnique(0)->setLightingEnabled(
            false); // appearance does not change with different scene orientation
        material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE); // map object "overlays" existing scene
        material_->getTechnique(0)->setDepthWriteEnabled(true);
        material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT + Ogre::TVC_DIFFUSE);
        material_->getTechnique(0)->setCullingMode(Ogre::CULL_NONE); // No culling

        ogreInitialized_ = true;
    }

    // load map and create + attach visible Ogre object
    visualizeMap(theMap);
}

MapElement::~MapElement() {
    // detach and destroy all objects
    for (auto object : objects_) {
        switch (object.first) {
        case ObjectClassification::MAP: {
            auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
            if (man_object) {
                man_object->getParentSceneNode()->detachObject(man_object);
                sceneManager_->destroyManualObject(man_object);
            }
            break;
        }

        case ObjectClassification::LANELETID: {
            auto mov_text = dynamic_cast<rviz::MovableText*>(object.second);
            if (mov_text) {
                mov_text->getParentSceneNode()->detachObject(mov_text);
                delete mov_text;
            }
            break;
        }

        case ObjectClassification::SEPERATOR: {
            auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
            if (man_object) {
                man_object->getParentSceneNode()->detachObject(man_object);
                sceneManager_->destroyManualObject(man_object);
            }
            break;
        }

        case ObjectClassification::STOPLINE: {
            auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
            if (man_object) {
                man_object->getParentSceneNode()->detachObject(man_object);
                sceneManager_->destroyManualObject(man_object);
            }
            break;
        }

        default: {
            if (object.second) {
                object.second->getParentSceneNode()->detachObject(object.second);
                delete object.second;
            }
        }
        }
    }
    // destroy all child scene nodes
    sceneNode_->removeAndDestroyAllChildren();
    // self detach from parent_scene_node
    sceneNode_->getParentSceneNode()->removeChild(sceneNode_);
    // delete sceneNode_
    sceneNode_->getCreator()->destroySceneNode(sceneNode_);
}

void MapElement::disable() {
    sceneNode_->setVisible(false);
}

void MapElement::enable() {
    sceneNode_->setVisible(true);
}

void MapElement::disable(ObjectClassification classification) {
    if (classification == REGULATORYELEMENT) {
        for (auto object : objects_) {
            if (std::find(std::begin(regElementClassifications), std::end(regElementClassifications), object.first) !=
                std::end(regElementClassifications)) {
                object.second->setVisible(false);
            }
        }
    } else {
        for (auto object : objects_) {
            if (object.first == classification) {
                object.second->setVisible(false);
            }
        }
    }
}

void MapElement::enable(ObjectClassification classification) {
    if (classification == REGULATORYELEMENT) {
        for (auto object : objects_) {
            if (std::find(std::begin(regElementClassifications), std::end(regElementClassifications), object.first) !=
                std::end(regElementClassifications)) {
                object.second->setVisible(true);
            }
        }
    } else {
        for (auto object : objects_) {
            if (object.first == classification) {
                object.second->setVisible(true);
            }
        }
    }
}

void MapElement::visualizeMap(lanelet::LaneletMapConstPtr theMap) {


    // Create manual objects that will be attached to the scene-node
    Ogre::ManualObject* mapManualObject =
        sceneManager_->createManualObject("llet_object_" + std::to_string(manObjCounter_++));
    Ogre::ManualObject* seperatorManualObject =
        sceneManager_->createManualObject("llet_object_" + std::to_string(manObjCounter_++));

    // Attach Lanelet to manual object
    mapManualObject->begin("lanelet_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    seperatorManualObject->begin("lanelet_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Iterate through all lanelets in map-graph
    for (const lanelet::ConstLanelet& lanelet : theMap->laneletLayer) {
        addLaneletToManualObject(lanelet, mapManualObject);
        addSeperatorToManualObject(lanelet, seperatorManualObject);
        addRegulatoryElements(lanelet, sceneNode_);
        attachLaneletIdToSceneNode(lanelet, sceneNode_);
    }

    mapManualObject->end();
    seperatorManualObject->end();

    // attach manual object to scene node
    if (mapManualObject->getNumSections()) {
        sceneNode_->attachObject(mapManualObject);
        // save ptr to manual object (needed later for deletion)
        objects_.push_back(std::make_pair(ObjectClassification::MAP, mapManualObject));
    }

    if (seperatorManualObject->getNumSections()) {
        sceneNode_->attachObject(seperatorManualObject);
        // save ptr to manual object (needed later for deletion)
        objects_.push_back(std::make_pair(ObjectClassification::SEPERATOR, seperatorManualObject));
    }
}

void MapElement::addRegulatoryElements(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parentNode) {
    // create child SceneNode. Only SceneNodes can be positioned.
    Ogre::SceneNode* regulatoryElementsNode = parentNode->createChildSceneNode();
    // get pointer to regulatory elements
    auto regulatoryElements = lanelet.regulatoryElements();
    // loop  over the regulatory elements
    for (auto&& regElement : regulatoryElements) {
        // Get reference lines
        auto refLines = regElement.get()->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
        attachRefLinesToSceneNode(refLines, regulatoryElementsNode);
    }
}

void MapElement::attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d>& stopLines,
                                           Ogre::SceneNode* parentNode) {
    // Create Manual Object, RefLines will be created as Manual Object using the
    // ogre_helper::drawLine helper function
    Ogre::ManualObject* stopLinesManualObject =
        sceneManager_->createManualObject("llet_object_" + std::to_string(manObjCounter_++));
    stopLinesManualObject->begin("lanelet_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (auto&& stopLine : stopLines) {
        auto line = MapElement::ogreLineFromLLetLineString(stopLine);
        ogre_helper::drawLine(line, stopLinesManualObject, colorStopLine_, stopLineWidth_);
    }
    if (stopLinesManualObject->getNumSections()) {
        parentNode->attachObject(stopLinesManualObject);
        // save ptr to manual object (needed later for deletion)
        objects_.push_back(std::make_pair(ObjectClassification::STOPLINE, stopLinesManualObject));
    }
    stopLinesManualObject->end();
}

void MapElement::attachLaneletIdToSceneNode(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parentNode) {
    // create child SceneNode. Only SceneNodes can be positioned.
    Ogre::SceneNode* childNode = parentNode->createChildSceneNode();

    rviz::MovableText* msg = new rviz::MovableText(std::to_string(lanelet.id()));
    msg->setCharacterHeight(2.0); // TODO: create property and read from the property
    msg->setColor(Ogre::ColourValue::White);
    msg->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE); // Center horizontally and
                                                                                    // display above the node

    Ogre::Vector3 trans = ogreVec3FromLLetPoint(lanelet.leftBound().front());
    util_rviz::setPositionSafely(childNode, trans);

    childNode->attachObject(msg);
    objects_.push_back(std::make_pair(ObjectClassification::LANELETID, msg));
}

void MapElement::addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
    // get line from left Linestrip points
    auto leftbound = lanelet.leftBound();
    auto lineLeft = ogreLineFromLLetLineString(leftbound);
    // draw line as Ogre Object
    ogre_helper::drawLine(lineLeft, manual, colorLeft_, laneletWidth_);
    // get line from right Linestrip points
    auto rightbound = lanelet.rightBound();
    auto lineRight = ogreLineFromLLetLineString(rightbound);
    // draw line as Ogre Object
    ogre_helper::drawLine(lineRight, manual, colorRight_, laneletWidth_);
}


void MapElement::addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual) {
    // get line from first Point of the left Linestrip to the first Point of the
    // right Linestrip
    lanelet::ConstPoints3d pointsF;
    pointsF.push_back(lanelet.leftBound().front());
    pointsF.push_back(lanelet.rightBound().front());
    // draw line as Ogre Object
    auto line = ogreLineFromLLetPts(pointsF);
    ogre_helper::drawLine(line, manual, colorSeperator_, seperatorWidth_);

    // get line from last Point of the left Linestrip to the last Point of the
    // right Linestrip
    lanelet::ConstPoints3d pointsL;
    pointsF.push_back(lanelet.leftBound().front());
    pointsF.push_back(lanelet.rightBound().front());
    // draw line as Ogre Object
    line = ogreLineFromLLetPts(pointsL);
    ogre_helper::drawLine(line, manual, colorSeperator_, seperatorWidth_);
}


ogre_helper::Line MapElement::ogreLineFromLLetLineString(lanelet::ConstLineString3d& lineString) {
    ogre_helper::Line line;
    for (lanelet::ConstPoint3d point : lineString) {
        line.push_back(ogreVec3FromLLetPoint(point));
    }
    return line;
}


ogre_helper::Line MapElement::ogreLineFromLLetPts(lanelet::ConstPoints3d& ptsVector) {
    ogre_helper::Line line;
    for (lanelet::ConstPoint3d point : ptsVector) {
        line.push_back(ogreVec3FromLLetPoint(point));
    }
    return line;
}


Ogre::Vector3 MapElement::ogreVec3FromLLetPoint(lanelet::ConstPoint3d point) {
    using boost::numeric_cast;
    using boost::numeric::bad_numeric_cast;
    return Ogre::Vector3(numeric_cast<Ogre::Real>(point.x()), numeric_cast<Ogre::Real>(point.y()), 0.f);
}

} // namespace lanelet_rviz_plugin_ros
