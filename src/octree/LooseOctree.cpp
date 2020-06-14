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

#include "LooseOctree.h"

namespace Magnum { namespace Examples {
OctreeNode::OctreeNode(LooseOctree* const tree, OctreeNode* const pParent,
                       const Vector3& nodeCenter, const Float halfWidth,
                       const size_t depth) :
    _pTree(tree),
    _pParent(pParent),
    _pChildren(nullptr),
    _center(nodeCenter),
    _lowerBound(nodeCenter - Vector3(halfWidth, halfWidth, halfWidth)),
    _upperBound(nodeCenter + Vector3(halfWidth, halfWidth, halfWidth)),
    _lowerBoundExtended(nodeCenter - 2 * Vector3(halfWidth, halfWidth, halfWidth)),
    _upperBoundExtended(nodeCenter + 2 * Vector3(halfWidth, halfWidth, halfWidth)),
    _halfWidth(halfWidth),
    _depth(depth),
    _maxDepth(tree->_maxDepth),
    _bIsLeaf(true) {}

OctreeNode* OctreeNode::getChildNode(const std::size_t childIdx) const {
    CORRADE_INTERNAL_ASSERT(_pChildren != nullptr);
    return &_pChildren->_nodes[childIdx];
}

void OctreeNode::clearPointData() {
    _pPointListHead = nullptr;
    _pointCount     = 0;

    if(!isLeaf()) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            _pChildren->_nodes[childIdx].clearPointData();
        }
    }
}

void OctreeNode::split() {
    if(!isLeaf() || _depth == _maxDepth) {
        return;
    }

    if(isLeaf()) {
        /*--------------------------------------------------------
         *
         *           6-------7
         *          /|      /|
         *         2-+-----3 |
         *         | |     | |   y
         *         | 4-----+-5   | z
         *         |/      |/    |/
         *         0-------1     +--x
         *
         *         0   =>   0, 0, 0
         *         1   =>   0, 0, 1
         *         2   =>   0, 1, 0
         *         3   =>   0, 1, 1
         *         4   =>   1, 0, 0
         *         5   =>   1, 0, 1
         *         6   =>   1, 1, 0
         *         7   =>   1, 1, 1
         *
         *--------------------------------------------------------*/

        _pChildren = _pTree->requestChildrenFromPool();

        const auto childHalfWidth = _halfWidth * static_cast<Float>(0.5);
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            auto newCenter = _center;
            newCenter[0] += (childIdx & 1) ? childHalfWidth : -childHalfWidth;
            newCenter[1] += (childIdx & 2) ? childHalfWidth : -childHalfWidth;
            newCenter[2] += (childIdx & 4) ? childHalfWidth : -childHalfWidth;

            OctreeNode* const pChildNode = &_pChildren->_nodes[childIdx];

            /* Placement new: re-use existing memory block, just call constructor to re-initialize data */
            new(pChildNode) OctreeNode(_pTree, this, newCenter, childHalfWidth, _depth + 1u);
        }

        /* Must explicitly mark as non-leaf node, and must do this after all children nodes are ready */
        _bIsLeaf = false;
    }
}

void OctreeNode::removeAllDescendants() {
    if(isLeaf()) {
        return;
    }

    /* Must explicitly mark as leaf node */
    _bIsLeaf = true;

    for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
        auto& pChildNode = _pChildren->_nodes[childIdx];
        pChildNode.removeAllDescendants();
    }
    _pTree->returnChildrenToPool(_pChildren);
}

void OctreeNode::removeEmptyDescendants() {
    if(isLeaf()) {
        return;
    }

    bool bAllEmpty  = true;
    bool bAllLeaves = true;
    for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
        auto& pChildNode = _pChildren->_nodes[childIdx];
        pChildNode.removeEmptyDescendants();
        bAllLeaves &= pChildNode.isLeaf();
        bAllEmpty  &= (pChildNode._pointCount == 0);
    }

    // Remove all 8 children nodes iff they are all leaf nodes and all empty nodes
    if(bAllEmpty && bAllLeaves) {
        _pTree->returnChildrenToPool(_pChildren);
        _bIsLeaf = true;
    }
}

void OctreeNode::keepPoint(OctreePoint* const pPoint) {
    pPoint->_pNode  = this;
    pPoint->_bValid = true;
    pPoint->_pNext  = _pPointListHead;
    _pPointListHead = pPoint;
    _pointCount    += 1u;
}

void OctreeNode::insertPoint(OctreePoint* const pPoint) {
    if(_depth == _maxDepth) {
        keepPoint(pPoint);
        return;
    }

    /* Split node if this is a leaf node */
    split();

    /* Compute the index of the child node that contains this point */
    const Vector3 ppos     = pPoint->_position;
    std::size_t   childIdx = 0;
    for(std::size_t dim = 0; dim < 3; ++dim) {
        if(_center[dim] < ppos[dim]) {
            childIdx |= (1 << dim);
        }
    }
    _pChildren->_nodes[childIdx].insertPoint(pPoint);
}

#if 0
// todo
bool OctreeNode::updateDebugGeometry() {
    if(_depth > _pTree->_maxLevelBoundaryLineGeneration) {
        return false;
    }

    int     renderCount = 0;
    Vector3 vertices[8];
    bool    rendered[8] { false, false, false, false,
                          false, false, false, false };

    // Also call add lines recursively
    for(std::size_t i = 0; i < 8; i++) {
        vertices[i]     = _center;
        vertices[i][0] += (i & 1) ? _halfWidth : -_halfWidth;
        vertices[i][1] += (i & 2) ? _halfWidth : -_halfWidth;
        vertices[i][2] += (i & 4) ? _halfWidth : -_halfWidth;

        if(!isLeaf()) {
            rendered[i] = _pChildren->_nodes[i].updateDebugGeometry();
            if(rendered[i]) {
                ++renderCount;
            }
        }
    }

    //--------------------------------------------------------
    //
    //           6-------7
    //          /|      /|
    //         2-+-----3 |
    //         | |     | |   y
    //         | 4-----+-5   | z
    //         |/      |/    |/
    //         0-------1     +--x
    //
    //         0   =>   0, 0, 0
    //         1   =>   0, 0, 1
    //         2   =>   0, 1, 0
    //         3   =>   0, 1, 1
    //         4   =>   1, 0, 0
    //         5   =>   1, 0, 1
    //         6   =>   1, 1, 0
    //         7   =>   1, 1, 1
    //
    //--------------------------------------------------------

    // No primitive in this node
    if(_pointCount[OctreePrimitiveType::Point] == 0
       && _pointCount[OctreePrimitiveType::Triangle] == 0
       && _pointCount[OctreePrimitiveType::AnalyticalGeometry] == 0) {
        if(!_pTree->_bBoundaryLineForNonEmptyParent) {
            return (renderCount > 0);
        }

        if(renderCount == 0                 // Children did not render
           && _pTree->_pRootNode != this) { // Not root node, and no data in this node)
            return false;
        }
    }

    if(renderCount < 8) { // If renderCount == 8 then no need to render this node
        const auto& debugLines = _pTree->m_DebugGeometry;
        for(int i = 0; i < 8; ++i) {
            if((i & 1) && (!rendered[i] || !rendered[i - 1])) {
                debugLines->appendVertex(vertices[i]);
                debugLines->appendVertex(vertices[i - 1]);
            }
            if((i & 2) && (!rendered[i] || !rendered[i - 2])) {
                debugLines->appendVertex(vertices[i]);
                debugLines->appendVertex(vertices[i - 2]);
            }
            if((i & 4) && (!rendered[i] || !rendered[i - 4])) {
                debugLines->appendVertex(vertices[i]);
                debugLines->appendVertex(vertices[i - 4]);
            }
        }
    }
    return true;
}

#endif

LooseOctree::~LooseOctree() {
    /* Firstly clear data recursively */
    clear();

    /* Deallocate memory pool */
    deallocateMemoryPool();

    /* Remove root node */
    delete _pRootNode;
}

void LooseOctree::clear() {
    /* Return all tree nodes to memory pool except the root node */
    _pRootNode->removeAllDescendants();

    clearPoints();

    /* Set state to imcomplete build */
    _bCompleteBuild = false;
}

void LooseOctree::clearPoints() {
    /* Recursively clear point data */
    _pRootNode->clearPointData();

    /* Deallocate memory blocks */
    if(_octreePointsPtr != nullptr && _nPoints > 0) {
        delete _octreePointsPtr;
        _nPoints = 0;
    }
}

std::size_t LooseOctree::getMaxNumPointInNodes() const {
    std::size_t count { 0 };
    for(const OctreeNodeBlock* pNodeBlock : _sActiveTreeNodeBlocks) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            const auto& pNode = pNodeBlock->_nodes[childIdx];
            if(count < pNode._pointCount) {
                count = pNode._pointCount;
            }
        }
    }
    return count;
}

void LooseOctree::setPoints(const std::vector<Vector3>& points) {
    clearPoints();
    _nPoints = points.size();
    if(_nPoints == 0) {
        return;
    }
    _octreePointsPtr = new OctreePoint[_nPoints];
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        const auto pPoint = &_octreePointsPtr[idx];
        new(pPoint) OctreePoint(points[idx]); /* placement new */
    }
}

void LooseOctree::build() {
    /* Compute max depth that the tree can reach */
    _maxDepth = 0u;
    std::size_t numLevelNodes   = 1u;
    std::size_t maxNumTreeNodes = 1u;
    Float       nodeWidth       = _width;

    while(nodeWidth * static_cast<Float>(0.5) > _minWidth) {
        ++_maxDepth;
        numLevelNodes   *= 8;
        maxNumTreeNodes += numLevelNodes;
        nodeWidth       *= static_cast<Float>(0.5);
    }
    _pRootNode->_maxDepth = _maxDepth;
    rebuild();
    _bCompleteBuild = true;

    Debug() << "Octree info:";
    Debug() << "       center:" << _center;
    Debug() << "       width:" << _width;
    Debug() << "       min width:" << _minWidth;
    Debug() << "       max depth:" << _maxDepth;
    Debug() << "       max tree nodes:" << maxNumTreeNodes;
}

void LooseOctree::update() {
    if(!_bCompleteBuild) {
        build();
    }
    (!_bAlwaysRebuild) ? incrementalUpdate() : rebuild();
}

void LooseOctree::rebuild() {
    /* Recursively remove all tree nodes other than root node */
    _pRootNode->removeAllDescendants();

    /* Clear root node point data */
    _pRootNode->clearPointData();

    /* Populate all points to tree nodes in a top-down manner */
    populatePoints();
}

void LooseOctree::populatePoints() {
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        _pRootNode->insertPoint(&_octreePointsPtr[idx]);
    }
}

void LooseOctree::incrementalUpdate() {
    checkValidity();
    removeInvalidPointsFromNodes();
    reinsertInvalidPoints();

    /* Recursively remove all empty nodes, returning them to memory pool for recycling */
    _pRootNode->removeEmptyDescendants();
}

void LooseOctree::checkValidity() {
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        OctreePoint&  point = _octreePointsPtr[idx];
        OctreeNode*   pNode = point._pNode;
        const Vector3 ppos  = point._position;
        if(!pNode->looselyContains(ppos) && pNode != _pRootNode) {
            /* Go up, find the node tightly containing it (or stop if reached root node) */
            while(pNode != _pRootNode) {
                pNode = pNode->_pParent;
                if(pNode->contains(ppos) || pNode == _pRootNode) {
                    point._bValid = false;
                    point._pNode  = pNode;
                    break;
                }
            }
        } else {
            point._bValid = (pNode != _pRootNode) ? true : false;
        }
    }
}

void LooseOctree::removeInvalidPointsFromNodes() {
    for(OctreeNodeBlock* const pNodeBlock: _sActiveTreeNodeBlocks) {
        for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
            OctreeNode&        pNode    = pNodeBlock->_nodes[childIdx];
            OctreePoint* const pOldHead = pNode._pPointListHead;
            if(!pOldHead) {
                continue;
            }

            OctreePoint* pIter    = pOldHead;
            OctreePoint* pNewHead = nullptr;
            std::size_t  count    = 0;
            while(pIter) {
                const auto pNext = pIter->_pNext;
                if(pIter->_bValid) {
                    pIter->_pNext = pNewHead;
                    pNewHead      = pIter;
                    ++count;
                }
                pIter = pNext;
            }
            pNode._pPointListHead = pNewHead;
            pNode._pointCount     = count;
        }
    }
}

void LooseOctree::reinsertInvalidPoints() {
    for(std::size_t idx = 0; idx < _nPoints; ++idx) {
        const auto point = &_octreePointsPtr[idx];
        if(!point->_bValid) {
            _pRootNode->insertPoint(point);
        }
    }
}

OctreeNodeBlock* LooseOctree::requestChildrenFromPool() {
    if(_numAvaiableBlocksInPool == 0) {
        /* Allocate 64 more node blocks and put to the pool */
        const auto pBigBlock = new OctreeNodeBlock[64];
        _pNodeBigBlocks.push_back(pBigBlock);
        for(std::size_t i = 0; i < 64; ++i) {
            const auto pBlock = &pBigBlock[i];
            pBlock->_nextBlock  = _pNodeBlockPoolHead;
            _pNodeBlockPoolHead = pBlock;
        }
        _numAvaiableBlocksInPool += 64;
        _numAllocatedNodes       += 64 * 8;
    }

    const auto pNodeBlock = _pNodeBlockPoolHead;
    _pNodeBlockPoolHead       = pNodeBlock->_nextBlock;
    _numAvaiableBlocksInPool -= 1u;
    _sActiveTreeNodeBlocks.insert(pNodeBlock);
    return pNodeBlock;
}

void LooseOctree::returnChildrenToPool(OctreeNodeBlock* const pNodeBlock) {
    pNodeBlock->_nextBlock    = _pNodeBlockPoolHead;
    _pNodeBlockPoolHead       = pNodeBlock;
    _numAvaiableBlocksInPool += 1u;
    _sActiveTreeNodeBlocks.erase(pNodeBlock);
}

void LooseOctree::deallocateMemoryPool() {
    if(_numAllocatedNodes != _numAvaiableBlocksInPool * 8 + 1u) {
        Fatal{} << "Internal data corrupted, may be all nodes were not returned from tree";
    }

    for(const auto pBigBlock: _pNodeBigBlocks) {
        delete[] pBigBlock;
    }
    _pNodeBigBlocks.resize(0);
    _numAllocatedNodes       = 1u; /* root node still remains */
    _numAvaiableBlocksInPool = 0u;
}

#if 0
std::shared_ptr<DebugRenderGeometry>
Octree::getDebugGeometry(const std::size_t maxLevel, bool bDrawNonEmptyParent /*= true*/) {
    m_MaxLevelDebugRender = maxLevel;
    m_bDrawNonEmptyParent = bDrawNonEmptyParent;
    if(!m_DebugGeometry) {
        m_DebugGeometry.reset();
    }

    // Create debug geometry and set default rendering mateirial
    m_DebugGeometry = std::make_shared<DebugRenderLines>("OctreeDebugRendering");

    // Update debug rendering data (if any)
    _pRootNode->updateDebugGeometry();
    m_DebugGeometry->setDataModified(true);

    return std::static_pointer_cast<DebugRenderGeometry>(m_DebugGeometry);
}

void LooseOctree::updateDebugGeometry() {
    LOG_IF(FATAL, (!m_DebugGeometry)) << "Debug geometry has not been created";
    m_DebugGeometry->clear();
    _pRootNode->updateDebugGeometry();
    m_DebugGeometry->setDataModified(true);
}

#endif
} }
