#ifndef Magnum_Examples_OctreeExample_LooseOctree_h
#define Magnum_Examples_OctreeExample_LooseOctree_h
/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vladimír Vondruš <mosra@centrum.cz>
        2020 — Nghia Truong <nghiatruong.vn@gmail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <Magnum/Magnum.h>

#include <array>
#include <cstdint>
#include <climits>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace Magnum { namespace Examples {
class OctreeNode;
class LooseOctree;

struct OctreePoint {
    /* Not allow to copy, as each point store a unique index to the point array */
    OctreePoint(const OctreePoint&) = delete;
    OctreePoint& operator=(const OctreePoint&) = delete;

    OctreePoint(const std::vector<Vector3>& positions) : _positions(positions) {}
    const Vector3& getPosition() const { return _positions[_idx]; }

    const std::size_t           _idx;       /* index of the point in the original array */
    const std::vector<Vector3>& _positions; /* array of original points */

    OctreeNode*  _pNode = nullptr;          /* pointer to the octree node containing the point */
    OctreePoint* _pNext = nullptr;          /* pointer to the next node in the point list of the octree node */

    /* Flag to keep track of point validity
     * During tree update, set it to true if:
     *  1) the point is still contained in the tree node that it has previously been inserted to, and
     *  2) the depth of current node reaches maxDepth
     */
    bool _bValid = true;
};

/* Forward declaration */
struct OctreeNodeBlock;

class OctreeNode {
    friend class LooseOctree;
public:
    OctreeNode(const OctreeNode&) = delete;
    OctreeNode& operator=(const OctreeNode&) = delete;

    /*
     * Default constructor called during memory allocation
     */
    OctreeNode() :
        _pTree(nullptr),
        _pParent(nullptr),
        _center(Vector3(0, 0, 0)),
        _halfWidth(0),
        _depth(0),
        _maxDepth(0),
        _bIsLeaf(true) {}

    /*
     * Constructor called during node splitting
     */
    explicit OctreeNode(LooseOctree* const tree, OctreeNode* const pParent,
                        const Vector3& nodeCenter, const Float halfWidth, const std::size_t depth);

    bool isLeaf() const { return _bIsLeaf; }

    /* Get a child node (child idx from 0 to 7) */
    OctreeNode* getChildNode(const std::size_t childIdx) const;

    /*
     * Return the head node of the point list in the node
     */
    OctreePoint* getPointList() const { return _pPointListHead; }

    /*
     * Get the number of points of the given type in this node
     */
    std::size_t getPointCount() const { return _pointCount; }

    /*
     * Recursively clear point data (linked list and counter)
     * Note that the point data still exist in the main octree list, they are just removed from the node
     */
    void clearPointData();

    /* Split node (requesting 8 children nodes from memory pool) */
    void split();

    /*
     * Recursively remove all descendant nodes (return them back to memory pool)
     * As a result, after calling to this function, the current node will become a leaf node
     */
    void removeAllDescendants();

    /*
     * Recursively remove all descendant nodes that are empty (all 8 children of a node are removed at the same time)
     */
    void removeEmptyDescendants();

    /*
     * Keep the point at this node as cannot pass it down further to any child node
     */
    void keepPoint(OctreePoint* const pPrimitive);

    /*
     * Insert a point point into the subtree in a top-down manner
     */
    void insertPoint(OctreePoint* const pPrimitive);

    /*
     * Check if the given point is contained in the node boundary (bounding box)
     */
    bool contains(const Vector3& point) { return contains(point[0], point[1], point[2]); }
    bool contains(const Float x, const Float y, const Float z) {
        return x > _lowerBound[0]
               && y > _lowerBound[1]
               && z > _lowerBound[2]
               && x < _upperBound[0]
               && y < _upperBound[1]
               && z < _upperBound[2];
    }

    /*
     * Check if the given point is contained in the node loose boundary (which is 2X bigger than the bounding box)
     */
    bool looselyContains(const Vector3& point) { return looselyContains(point[0], point[1], point[2]); }
    bool looselyContains(const Float x, const Float y, const Float z) {
        return x > _lowerExtendedBound[0]
               && y > _lowerExtendedBound[1]
               && z > _lowerExtendedBound[2]
               && x < _upperExtendedBound[0]
               && y < _upperExtendedBound[1]
               && z < _upperExtendedBound[2];
    }

    // todo
    /*
     * Recursively update debug geometry by adding lines drawing bounding boxes of the active nodes
     * \return True if debug lines have been added to visualize the bounding box of the current node
     */
    bool updateDebugGeometry();

private:
    LooseOctree*     _pTree;               /* pointer to the octree, used to request children from memory pool during splitting node */
    OctreeNode*      _pParent;             /* pointer to the parent node */
    OctreeNodeBlock* _pChildren = nullptr; /* pointer to a memory block containing 8 children nodes */

    const Vector3     _center;             /* center of this node */
    const Vector3     _lowerBound;         /* the lower corner of the node */
    const Vector3     _upperBound;         /* the upper corner of the node */
    const Vector3     _lowerExtendedBound; /* the extended lower corner of the node, which is 2X bigger than the exact AABB */
    const Vector3     _upperExtendedBound; /* the extended upper corner of the node, which is 2X bigger than the exact AABB */
    const Float       _halfWidth;          /* the half width of the node */
    const std::size_t _depth;              /* depth of this node (depth > 0, depth = 1 starting at the root node) */
    std::size_t       _maxDepth;           /* the maximum depth level possible */
    bool              _bIsLeaf = true;

    /* Head of the link list storing octree data points */
    OctreePoint* _pPointListHead;

    /* Count the number of points stored in this node */
    std::size_t _pointCount;
};

/*
 * This is a data structure to store a memory block of 8 tree node at a time
 * Using a block of 8 nodes at a time can reduce node allocation/merging/slitting overhead
 */
struct OctreeNodeBlock {
    OctreeNode       m_Nodes[8];
    OctreeNodeBlock* m_NextBlock = nullptr; /* pointer to the next block in the memory pool */
};

/*
 * Loose octree: each tree node has a loose boundary which is exactly twice big as its exact boundary
 * During tree update, a primitive is moved around from node to node
 * If removed from a node, the primitive is moving up to find the lowest node that exactly contains it,
 * then it is move down to the lowest level possible, stopping at a node that loosely contains it
 */
class LooseOctree {
    friend class OctreeNode;
public:
    /*
     * \param center The center of the tree, which also is the center of the root node
     * \param width Width of the octree bounding box
     * \param minWidth Minimum allowed width of the tree nodes, valid only if there are only points primitives
     * \param minWidthRatio If there is primitive that is not a point, minWidth will be recomputed as minWidth = min(width of all non-point primitives) * minWidthRatio
     * \param name Name of the octree
     */
    explicit LooseOctree(const Vector3& center, const Float width, const Float minWidth,
                         const Float minWidthRatio = 1.0);

    /*
     * Cleanup memory here
     */
    virtual ~LooseOctree();

    /*
     * Clear all data, but still keep allocated nodes in memory pool
     */
    virtual void clear();

    /*
     * Completely remove all point data
     */
    void clearPoints();

    const Vector3 getCenter() const { return _center; }
    Float getWidth() const { return _width; }
    Float getMinWidth() const { return _minWidth; }
    std::size_t getMaxDepth() const { return _maxDepth; }
    std::size_t getNumAllocatedNodes() const { return _numAllocatedNodes; }
    OctreeNode* getRootNode() const { return _pRootNode; }

    /*
     * Count the maximum number of points stored in a tree node
     */
    std::size_t getMaxNumPointInNodes() const;

    /*
     * Add a PointSet geometry into the tree
     * (the points will not be populated to tree nodes until calling to build())
     */
    std::size_t addPointSet(const std::shared_ptr<PointSet>& pointset);

    /*
     * Set the alwaysRebuild flag (true: rebuilt the tree from scratch in every update, false: incrementally update from the current state)
     */
    void setAlwaysRebuild(const bool bAlwaysRebuild) { _bAlwaysRebuild = bAlwaysRebuild; }

    void build();
    void update();

    // todo
    /*
     * Generate the debug geometry for debug rendering (a bounding box for each node)
     * \param maxLevel The tree will be visualized up to maxLevel levels
     */
    std::shared_ptr<DebugRenderGeometry> getDebugGeometry(const std::size_t maxLevel, bool bDrawNonEmptyParent = true);

    /* Update the boundary lines after updated tree */
    void updateBoundaryData();

private:
    /* Rebuild the tree from scratch */
    void rebuild();

    /* Populate point to tree nodes, from top (root node) down to leaf nodes */
    void populatePointPrimitives();

    /* Incrementally update octree from current state */
    void incrementalUpdate();

    /* For each point, check if it is still loosely contained in the tree node */
    void checkValidity();

    /*
     * Remove all invalid points from the tree nodes previously contained them
     */
    void removeInvalidPointsFromNodes();

    /*
     * For each invalid point, insert it back to the tree in a top-down manner
     * starting from the lowest ancestor node that tightly contains it (that node was found during validity check)
     */
    void reinsertInvalidPoints();

    /*
     * Request a block of 8 tree nodes from memory pool (this is called only during splitting node)
     * If the memory pool is exhausted, 64 more blocks will be allocated from the system memory
     */
    OctreeNodeBlock* requestChildrenFromPool();

    /*
     * Return 8 children nodes to memory pool (this is called only during destroying descendant nodes)
     */
    void returnChildrenToPool(OctreeNodeBlock* const pNodeBlock);

    /*
     * Pre-allocate a given number of node blocks (each block contains 8 nodes)
     * and add them to the memory pool
     */
    void allocateMoreNodeBlock(const std::size_t numBlocks);

    /*
     * Deallocate all node block in memory pool, called only during octree destructor
     */
    void deallocateMemoryPool();

    const Vector3 _center;                                /* center of the tree */
    const Float   _width;                                 /* width of the tree bounding box */

    const Float _minWidth;                                /* minimum width allowed for the tree nodes */
    std::size_t _maxDepth;                                /* max depth of the tree, which is computed based on _minWidth */

    OctreeNode* const _pRootNode;                         /* root node, should not be reassigned throughout the existence of the tree */
    OctreeNodeBlock*  _pNodeBlockPoolHead      = nullptr; /* the pool of tree nodes, storing pre - allocated nodes as a linked list */
    std::size_t       _numAvaiableBlocksInPool = 0;       /* count the number of nodes available in memory pool */
    std::size_t       _numAllocatedNodes;                 /* count the total number of allocated nodes so far */

    /* Set of node blocks that are in use (node blocks that have been taken from memory pool) */
    std::unordered_set<OctreeNodeBlock*> _sActiveTreeNodeBlocks;

    /*
     * During memory allocation for tree nodes, multiple node blocks are allocated at the same time from a big memory block
     * This variable store the first address of such big memory block, used during memory pool deallocation
     */
    std::vector<OctreeNodeBlock*> _pNodeBigBlocks;

    /* Store pointers of octree point data */
    std::vector<OctreePoint*> _octreePoints;

    /*
     * During memory allocation for octree point data,
     *  multiple points are allocated at the same time from a big memory block
     * This variable store the first address of such big memory block, used during primitive deallocation
     */
    std::vector<OctreePoint*> _pOctreePointBlocks;

    bool _bAlwaysRebuild = false;
    bool _bCompleteBuild = false;

    /* Maximum level of nodes for which the boundary lines will be generated */
    std::size_t _maxLevelBoundaryLineGeneration;

    /* Generate boundary lines for the empty inner nodes */
    bool _bBoundaryLineForNonEmptyParent = true;
};
} }

#endif
