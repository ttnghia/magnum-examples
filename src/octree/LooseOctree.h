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

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include <unordered_set>

namespace Magnum { namespace Examples {
class OctreeNode;
class LooseOctree;

struct OctreePoint {
    OctreePoint(const OctreePoint&) = delete;
    OctreePoint& operator=(const OctreePoint&) = delete;

    OctreePoint() = default;
    OctreePoint(const Vector3& position) : _position(position) {}

    Vector3      _position;
    OctreeNode*  _pNode { nullptr }; /* pointer to the octree node containing this point */
    OctreePoint* _pNext { nullptr }; /* pointer to the next point in the point list of the octree node */

    /* Flag to keep track of point validity
     * During tree update, it is true if:
     *  1) the point is still contained in the tree node that it has previously been inserted to, and
     *  2) depth of the current node reaches maxDepth
     */
    bool _bValid { true };
};

/* Forward declaration */
struct OctreeNodeBlock;

class OctreeNode {
    friend class LooseOctree;
public:
    OctreeNode(const OctreeNode&) = delete;
    OctreeNode& operator=(const OctreeNode&) = delete;

    /* Default constructor called during memory allocation */
    OctreeNode() :
        _pTree(nullptr),
        _pParent(nullptr),
        _pChildren(nullptr),
        _center(Vector3(0, 0, 0)),
        _halfWidth(0),
        _depth(0),
        _maxDepth(0),
        _bIsLeaf(true) {}

    /* Constructor called during node splitting */
    explicit OctreeNode(LooseOctree* const tree, OctreeNode* const pParent,
                        const Vector3& nodeCenter, const Float halfWidth,
                        const std::size_t depth);

    bool isLeaf() const { return _bIsLeaf; }
    const Vector3 getCenter() const { return _center; }
    Float getHalfWidth() const { return _halfWidth; }

    /* Get a child node (child idx from 0 to 7) */
    OctreeNode* getChildNode(const std::size_t childIdx) const;

    /* Return the head of the point list in the node */
    OctreePoint* getPointList() const { return _pPointListHead; }

    /* Get the number of points in this node */
    std::size_t getPointCount() const { return _pointCount; }

    /*
     * Recursively clear octree point data (linked list and counter)
     * Note that the point data still exists in the main octree list, they are just removed from the node
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
     * Recursively remove all descendant nodes that are empty
     *  (all 8 children of a node are removed at the same time)
     */
    void removeEmptyDescendants();

    /* Keep the point at this node as cannot pass it down further to any child node */
    void keepPoint(OctreePoint* const pPoint);

    /* Insert a point point into the subtree in a top-down manner */
    void insertPoint(OctreePoint* const pPoint);

    /*
     * Check if the given point is contained in the node boundary (bounding box)
     */
    bool contains(const Vector3& point) const { return contains(point[0], point[1], point[2]); }
    bool contains(const Float x, const Float y, const Float z) const {
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
    bool looselyContains(const Vector3& point) const { return looselyContains(point[0], point[1], point[2]); }
    bool looselyContains(const Float x, const Float y, const Float z) const {
        return x > _lowerBoundExtended[0]
               && y > _lowerBoundExtended[1]
               && z > _lowerBoundExtended[2]
               && x < _upperBoundExtended[0]
               && y < _upperBoundExtended[1]
               && z < _upperBoundExtended[2];
    }

    #if 0
    // todo
    /*
     * Recursively update debug geometry by adding lines drawing bounding boxes of the active nodes
     * \return True if debug lines have been added to visualize the bounding box of the current node
     */
    bool updateDebugGeometry();
#endif
private:
    LooseOctree*     _pTree;               /* pointer to the octree */
    OctreeNode*      _pParent;             /* pointer to the parent node */
    OctreeNodeBlock* _pChildren;           /* pointer to a memory block containing 8 children nodes */

    const Vector3     _center;             /* center of this node */
    const Vector3     _lowerBound;         /* the lower corner of the node */
    const Vector3     _upperBound;         /* the upper corner of the node */
    const Vector3     _lowerBoundExtended; /* the extended lower corner of the node, which is 2X bigger than the exact AABB */
    const Vector3     _upperBoundExtended; /* the extended upper corner of the node, which is 2X bigger than the exact AABB */
    const Float       _halfWidth;          /* the half width of the node */
    const std::size_t _depth;              /* depth of this node (depth > 0, depth = 1 starting at the root node) */
    std::size_t       _maxDepth;           /* the maximum depth level possible */
    bool              _bIsLeaf;

    /* Head of the link list storing octree data points */
    OctreePoint* _pPointListHead { nullptr };

    /* Count the number of points stored in this node */
    std::size_t _pointCount { 0 };
};

/*
 * This is a data structure to store a memory block of 8 tree nodes at a time
 * This can reduce node allocation/merging/slitting overhead
 */
struct OctreeNodeBlock {
    OctreeNode       _nodes[8];
    OctreeNodeBlock* _nextBlock { nullptr }; /* pointer to the next block in the memory pool */
};

/*
 * Loose octree: each tree node has a loose boundary which is exactly twice big as its exact boundary
 * During tree update, a primitive is moved around from node to node
 * If removed from a node, the primitive is moving up to find the lowest node that exactly contains it,
 *  then it is move down to the lowest level possible, stopping at a node that loosely contains it
 */
class LooseOctree {
    friend class OctreeNode;
public:
    /*
     * \param center The center of the tree, which also is the center of the root node
     * \param width Width of the octree bounding box
     * \param minWidth Minimum allowed width of the tree nodes
     */
    explicit LooseOctree(const Vector3& center, const Float width, const Float minWidth) :
        _center(center),
        _width(width),
        _minWidth(minWidth),
        _pRootNode(new OctreeNode(this, nullptr, center, width * static_cast<Float>(0.5), 0)),
        _numAllocatedNodes(1u) {}

    /* Cleanup memory here */
    ~LooseOctree();

    const Vector3 getCenter() const { return _center; }
    Float getWidth() const { return _width; }
    Float getMinWidth() const { return _minWidth; }
    std::size_t getMaxDepth() const { return _maxDepth; }
    std::size_t getNumAllocatedNodes() const { return _numAllocatedNodes; }
    OctreeNode* getRootNode() const { return _pRootNode; }

    /* Clear all data, but still keep allocated nodes in memory pool */
    void clear();

    /* Completely remove all octree point data */
    void clearPoints();

    /* Count the maximum number of points stored in a tree node */
    std::size_t getMaxNumPointInNodes() const;

    /*
     * Set points data for the tree
     * (the points will not be populated to tree nodes until calling to build())
     */
    void setPoints(const std::vector<Vector3>& points);

    /*
     * true: rebuilt the tree from scratch in every update
     * false: incrementally update from the current state
     */
    void setAlwaysRebuild(const bool bAlwaysRebuild) { _bAlwaysRebuild = bAlwaysRebuild; }

    /* Get all memory block of active nodes */
    const std::unordered_set<OctreeNodeBlock*>& getActiveTreeNodeBlocks() const
    { return _sActiveTreeNodeBlocks; }

    void build();
    void update();

    #if 0
    // todo
    /*
     * Generate the debug geometry for debug rendering (a bounding box for each node)
     * \param maxLevel The tree will be visualized up to maxLevel levels
     */
    std::shared_ptr<DebugRenderGeometry> getDebugGeometry(const std::size_t maxLevel, bool bDrawNonEmptyParent = true);

    /* Update the boundary lines after updated tree */
    void updateBoundaryData();
#endif
private:
    /* Rebuild the tree from scratch */
    void rebuild();

    /* Populate point to tree nodes, from top (root node) down to leaf nodes */
    void populatePoints();

    /* Incrementally update octree from current state */
    void incrementalUpdate();

    /* For each point, check if it is still loosely contained in the tree node */
    void checkValidity();

    /* Remove all invalid points from the tree nodes previously contained them */
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
     * Deallocate all node block in memory pool, called only during octree destructor
     */
    void deallocateMemoryPool();

    const Vector3 _center;                                  /* center of the tree */
    const Float   _width;                                   /* width of the tree bounding box */

    const Float _minWidth;                                  /* minimum width allowed for the tree nodes */
    std::size_t _maxDepth;                                  /* max depth of the tree, which is computed based on _minWidth */

    OctreeNode* const _pRootNode;                           /* root node, should not be reassigned throughout the existence of the tree */
    OctreeNodeBlock*  _pNodeBlockPoolHead      { nullptr }; /* the pool of tree nodes, storing pre - allocated nodes as a linked list */
    std::size_t       _numAvaiableBlocksInPool { 0 };       /* count the number of nodes available in memory pool */
    std::size_t       _numAllocatedNodes;                   /* count the total number of allocated nodes so far */

    /* Set of node blocks that are in use (node blocks that have been taken from memory pool) */
    std::unordered_set<OctreeNodeBlock*> _sActiveTreeNodeBlocks;

    /*
     * During memory allocation for tree nodes, multiple node blocks are allocated at the same time as a big memory block
     * This variable store the first address of such big memory block, used during memory pool deallocation
     */
    std::vector<OctreeNodeBlock*> _pNodeBigBlocks;

    /* Store pointer to the first address of the allocated octree point data */
    OctreePoint* _octreePointsPtr { nullptr };

    /* Number of points in the tree */
    std::size_t _nPoints { 0 };

    bool _bAlwaysRebuild { false };
    bool _bCompleteBuild { false };

    /* Maximum level of nodes for which the boundary lines will be generated */
    std::size_t _maxLevelBoundaryLineGeneration;

    /* Generate boundary lines for the empty inner nodes */
    bool _bBoundaryLineForNonEmptyParent { true };
};
} }

#endif
