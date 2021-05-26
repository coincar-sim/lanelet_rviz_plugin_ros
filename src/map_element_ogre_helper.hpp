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

#pragma once

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

namespace ogre_helper {

using namespace Ogre;

typedef std::vector<Vector3> Line;
typedef std::vector<Vector3> MonoPolygon;


inline MaterialPtr getMaterial(const std::string& matName) {
    auto& manager = MaterialManager::getSingleton();
    MaterialPtr matIt = manager.createOrRetrieve(matName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)
                            .first.dynamicCast<Material>();
    return matIt;
}

namespace {
template <typename Iter>
inline Vector3 getNormal(Iter it, Iter begin, Iter end) {
    Vector3 zero(0, 0, 0);
    // iterate backwards until direction vector is nonzero (in case of points on same position)
    auto dirBefore = zero;
    int i = 1;
    while (it != begin && dirBefore == zero) {
        dirBefore = *it - *std::prev(it, i);
        i++;
    }
    // same thing forwards
    auto dirAfter = zero;
    i = 1;
    while (std::next(it) != end && dirAfter == zero) {
        dirAfter = *std::next(it, i) - *it;
        i++;
    }
    // if no directions could be calculated, return 0 (only happens if there are no points or all points are identical)
    if (dirBefore == zero && dirAfter == zero)
        return zero;
    if (dirAfter == zero)
        dirAfter = dirBefore;
    if (dirBefore == zero)
        dirBefore = dirAfter;

    // calculate direction as mean from both line orientations
    auto dirCombined = dirAfter * (1 / dirAfter.length()) + dirBefore * (1 / dirBefore.length());
    dirCombined *= 1 / dirCombined.length();
    return Vector3(dirCombined.y, -dirCombined.x, 0); // rotate 90 degrees
}

inline MonoPolygon bufferSegment(const Line& line, double buffer_length) {
    MonoPolygon buffered;
    buffered.reserve(line.size());
    assert(line.size() >= 2);
    for (auto it = line.begin(); it != line.end(); ++it) {
        auto normal = getNormal(it, line.begin(), line.end());
        buffered.push_back(*it + normal * buffer_length / 2);
    }
    for (auto it = line.rbegin(); it != line.rend(); ++it) {
        auto normal = getNormal(it, line.rbegin(), line.rend());
        buffered.push_back(*it + normal * buffer_length / 2);
    }
    return buffered;
}

/**
 * @brief drawMonoPolygon draws a monotone polygon in ogre
 * @param poly
 * @param color
 * @param objcv::norm(dir)
 */
inline void drawMonoPolygon(const MonoPolygon& poly,
                            Ogre::ManualObject* obj,
                            Ogre::ColourValue color = ColourValue::White) {
    if (poly.size() < 3)
        return;

    auto itLeft = poly.begin();
    auto itRight = --poly.end();
    const auto startIndex = obj->getCurrentVertexCount();
    auto count = 0u;
    //    std::raise(SIGINT);
    //    obj->begin("osm_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (; itRight >= itLeft; ++itLeft, --itRight) {
        obj->position(*itLeft);
        obj->normal(0, 0, 1);
        obj->colour(color);
        if (count >= 2) {
            assert(obj->getCurrentVertexCount() > startIndex + count);
            obj->triangle(startIndex + count - 2, startIndex + count - 1, startIndex + count);
        }
        count++;
        obj->position(*itRight);
        obj->normal(0, 0, 1);
        obj->colour(color);
        if (count >= 2) {
            assert(obj->getCurrentVertexCount() > startIndex + count);
            assert(obj->getCurrentVertexCount() < 65535);
            obj->triangle(startIndex + count, startIndex + count - 1, startIndex + count - 2);
        }
        count++;
    }
}
} // namespace

inline void drawArea(const Line& line, ManualObject* obj, ColourValue color = ColourValue::White) {
    if (line.size() < 2) {
        return;
    }
    drawMonoPolygon(line, obj, color);
}

inline void drawLine(const Line& line, ManualObject* obj, ColourValue color = ColourValue::White, double width = 0.1) {
    if (width <= 0)
        return;
    if (line.size() < 2)
        return;
    auto buffered = bufferSegment(line, width);
    drawMonoPolygon(buffered, obj, color);
}

} // namespace ogre_helper
