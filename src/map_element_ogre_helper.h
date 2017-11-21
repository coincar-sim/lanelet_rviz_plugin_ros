#pragma once

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

namespace Ogre {

typedef std::vector<Vector3> Line;
typedef std::vector<Vector3> MonoPolygon;

namespace {
MaterialPtr getMaterial(const std::string& matName) {
    auto& manager = MaterialManager::getSingleton();
    MaterialPtr matIt = manager.createOrRetrieve(matName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)
                            .first.dynamicCast<Material>();
    return matIt;
}

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
            assert(startIndex + count - 2 >= 0);
            assert(obj->getCurrentVertexCount() > startIndex + count);
            obj->triangle(startIndex + count - 2, startIndex + count - 1, startIndex + count);
        }
        count++;
        obj->position(*itRight);
        obj->normal(0, 0, 1);
        obj->colour(color);
        if (count >= 2) {
            assert(startIndex + count - 2 >= 0);
            assert(obj->getCurrentVertexCount() > startIndex + count);
            assert(obj->getCurrentVertexCount() < 65535);
            obj->triangle(startIndex + count, startIndex + count - 1, startIndex + count - 2);
        }
        count++;
    }
}
} // end of anonymous namespace for helper functions

inline void drawLine(const Line& line, ManualObject* obj, ColourValue color = ColourValue::White, double width = 0.1) {
    if (width <= 0)
        return;
    if (line.size() < 2)
        return;
    auto buffered = bufferSegment(line, width);
    drawMonoPolygon(buffered, obj, color);
}

} // namespace ogre_helper
